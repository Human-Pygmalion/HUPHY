"""제어 노드 — `ControlLoop` 를 자기 스레드에서 돌리고 ROS 와 이어 줌.

    ros2 run huphy_ros control_node --ros-args -p config:=/경로/robot.yaml

## 이 파일이 하는 일은 배선뿐임

조립도 제어도 `huphy` 쪽 코드가 그대로 함.

    build_biped / build_leg     scripts/bringup.py
    ControlLoop                 control/loop.py
    Motion                      bridge.CommandBridge.motion 또는 control/stand.py

**새로 하는 일은 토픽을 `Motion` 으로 바꾸는 것 하나임.**

## 스레드

    메인        rclpy.spin()            구독·서비스·퍼블리시 타이머
    제어        ControlLoop.run()       100Hz. CAN 송수신
    can-rx x2   CanBus._read_loop       채널마다. huphy 가 이미 띄움

제어 스레드에서 `publish()` 를 부르지 않음 -- 직렬화와 DDS 가 제어 주기 안에 들어감.
제어 스레드는 `CommandBridge` 에 써 두기만 하고 타이머가 꺼내 보냄
(`docs/ros.md` 7-3).

## 모드

`policy` 파라미터가 비어 있으면 **토픽에서 받은 목표만** 씀. 값이 있으면 그 정책을
노드 안에서 돌림 -- 정책을 별도 노드로 빼지 않는 이유는 `docs/ros.md` 6-3.

## 안전

    명령이 안 오면        bridge 가 None 을 내 전송이 멈춤 (bridge.DEFAULT_STALE_S)
    모터가 답을 안 하면   LinkWatch 가 루프를 멈춤 (safety/link.py)
    노드를 끄면           ControlLoop.finally 가 감쇠 뒤 토크 차단 (control/loop.py)

**`kill -9` 는 마지막 것을 못 탐.** 토크가 켜진 채 남음.
"""

from __future__ import annotations

import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from huphy.config import load_robot
from huphy.control import ControlLoop, Mode
from huphy.scripts.bringup import build_biped, build_leg
from huphy.scripts.commission import all_legs

from . import convert
from .bridge import CommandBridge

PUBLISH_HZ = 50.0
"""관찰을 내보내는 주기.

제어 주기(기본 100Hz)보다 낮게 둠 -- 내보내는 값은 기록·감시용이라 매 주기 필요하지
않고, 퍼블리시가 GIL 을 쥐는 시간을 절반으로 줄임 (`docs/ros.md` 7-5).
"""


class ControlNode(Node):
    def __init__(self) -> None:
        super().__init__("huphy_control")

        self.declare_parameter("config", "")
        self.declare_parameter("limb", "")
        self.declare_parameter("hz", 100.0)
        self.declare_parameter("drain_ms", 2.0)
        self.declare_parameter("allow_uncalibrated", False)
        self.declare_parameter("stale_s", 0.2)
        self.declare_parameter("observe_only", True)

        path = self.get_parameter("config").value
        if not path:
            raise SystemExit("config 파라미터에 robot.yaml 경로를 줄 것")

        robot_config = load_robot(path)
        limb = self.get_parameter("limb").value
        options = dict(
            allow_uncalibrated=bool(self.get_parameter("allow_uncalibrated").value),
            drain_s=float(self.get_parameter("drain_ms").value) / 1000.0,
        )
        if limb:
            self.robot = build_leg(robot_config, robot_config.limbs[limb], **options)
        else:
            self.robot = build_biped(robot_config, all_legs(robot_config), **options)

        self.bridge = CommandBridge(stale_s=float(self.get_parameter("stale_s").value))

        # 기본이 OBSERVE 임. 토크를 넣는 것은 명시해야 하는 일이라, 파라미터를
        # 빠뜨렸을 때 안 움직이는 쪽으로 떨어지게 함 (control/loop.py 의 Mode).
        observe = bool(self.get_parameter("observe_only").value)
        self.mode = Mode.OBSERVE if observe else Mode.CONTROL

        self.pub = self.create_publisher(
            JointState, "~/joint_states", qos_profile_sensor_data
        )
        self.sub = self.create_subscription(
            JointState, "~/command", self._on_command, qos_profile_sensor_data
        )
        self.create_service(Trigger, "~/enable", self._on_enable)
        self.create_service(Trigger, "~/disable", self._on_disable)
        self.create_timer(1.0 / PUBLISH_HZ, self._publish)

        self.loop = ControlLoop(
            self.robot, hz=float(self.get_parameter("hz").value), mode=self.mode
        )
        self._thread = threading.Thread(
            target=self.loop.run, args=(self.bridge.motion,), daemon=True
        )

    # ---- 수명 -------------------------------------------------------------
    def start(self) -> None:
        self.robot.connect()
        self._names = tuple(self.robot.joint_names)
        """관절 이름을 **한 번만** 읽어 둠. 매 퍼블리시마다 로봇에 묻지 않음."""
        self.get_logger().info(
            f"{self.robot.id} {self.mode.value} {self.loop.hz:.0f}Hz "
            f"관절 {len(self._names)}개"
        )
        self._thread.start()

    def shutdown(self) -> None:
        """루프를 세우고 기다린 뒤 끊음.

        `loop.stop()` 만 부르고 바로 `disconnect()` 하면 감쇠 절차가 도는 중에
        채널이 닫힘. `join` 으로 `finally` 가 끝나기를 기다림.
        """
        self.loop.stop()
        if self._thread.is_alive():
            self._thread.join(timeout=10.0)
        self.robot.disconnect()

    # ---- 콜백 — 짧게 끝낼 것 ------------------------------------------------
    def _on_command(self, msg: JointState) -> None:
        try:
            action = convert.command_to_action(msg.name, msg.position)
        except ValueError as e:
            self.get_logger().warning(f"명령을 버림: {e}")
            return
        self.bridge.put_command(action)

    def _on_enable(self, request, response):
        del request
        self.robot.enable()
        response.success = True
        response.message = "토크 넣음"
        return response

    def _on_disable(self, request, response):
        del request
        self.bridge.clear_command()
        self.robot.disable()
        response.success = True
        response.message = "토크 끊음"
        return response

    def _publish(self) -> None:
        observation = self.bridge.take_observation()
        if not observation:
            return
        names, position, velocity, effort = convert.observation_to_arrays(
            observation, self._names
        )
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = names
        msg.position = position
        msg.velocity = velocity
        msg.effort = effort
        self.pub.publish(msg)


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = ControlNode()
    node.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
