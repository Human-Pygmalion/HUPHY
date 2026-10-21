"""제어 노드 — 모션보드에서 돎. 모터를 몲.

    ros2 run huphy_ros control_node --ros-args -p config:=/경로/robot.yaml

## 이 파일이 하는 일은 배선뿐임

조립도 제어도 `huphy` 쪽 코드가 그대로 함 (`docs/ros.md` 8-2).

    build_biped     scripts/bringup.py
    ControlLoop     control/loop.py
    가드·통신두절   safety/guards.py · safety/link.py

**정책을 모름.** 목표각을 누가 보냈는지도 모름 -- `policy_node` 가 보냈든 사람이
`ros2 topic pub` 으로 보냈든 같은 토픽임 (`docs/ros.md` 결정 8).

## 모드를 바꾸는 방법

`ControlLoop` 는 모드를 생성할 때 받고 `_enter` 에서 `enable()`/`disable()` 을
부름. 돌고 있는 루프의 모드를 바꾸는 길이 없음.

그래서 **루프를 새로 띄워서** 바꿈.

    시작        OBSERVE 루프   토크 안 들어감
    /enable     OBSERVE 를 멈추고 CONTROL 루프를 띄움
    /disable    CONTROL 을 멈춤 -> finally 가 감쇠 후 토크 차단

로봇 객체는 그대로 두고 루프만 바꿈. `_enter` 가 `is_connected` 를 보고 이미
연결돼 있으면 다시 안 엶.

`CONTROL` 을 멈출 때는 `_exit` 가 `DAMPING_S`(5초) 동안 감쇠를 보냄. 그만큼 걸림.

## 토크는 명시해야 들어감

기본이 `OBSERVE` 임. 파라미터를 빠뜨렸을 때 안 움직이는 쪽으로 떨어지게 함.
"""

from __future__ import annotations

import threading
from typing import Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState
from std_srvs.srv import Trigger

from huphy.config import ConfigError, load_robot
from huphy.control import ControlLoop, Mode
from huphy.scripts.bringup import build_biped, build_leg
from huphy.scripts.commission import all_legs

from . import convert
from .freshness import CONTROL_TARGET_STALE_S, Freshness, Latest

STATUS_HZ = 5.0
"""`loop_status` 를 내보내는 주기. 사람이 보는 값이라 느려도 됨."""

JOIN_S = 12.0
"""루프 스레드를 기다리는 시간. 감쇠 5초 + 여유.

모자라면 `disconnect()` 가 감쇠 도중에 채널을 닫음.
"""


class ControlNode(Node):
    def __init__(self) -> None:
        super().__init__("huphy_control")

        self.declare_parameter("config", "")
        self.declare_parameter("limb", "")
        self.declare_parameter("hz", 100.0)
        self.declare_parameter("state_hz", 100.0)
        self.declare_parameter("drain_ms", 2.0)
        self.declare_parameter("target_stale_ms", CONTROL_TARGET_STALE_S * 1000.0)
        self.declare_parameter("allow_uncalibrated", False)

        path = self.get_parameter("config").value
        if not path:
            raise SystemExit("config 파라미터에 robot.yaml 경로를 줄 것")
        try:
            config = load_robot(path)
        except ConfigError as e:
            raise SystemExit(f"{e}") from e

        options = dict(
            allow_uncalibrated=bool(self.get_parameter("allow_uncalibrated").value),
            drain_s=float(self.get_parameter("drain_ms").value) / 1000.0,
        )
        limb = self.get_parameter("limb").value
        if limb:
            self.robot = build_leg(config, config.limbs[limb], **options)
        else:
            self.robot = build_biped(config, all_legs(config), **options)

        self.hz = float(self.get_parameter("hz").value)
        self.targets = Freshness(
            float(self.get_parameter("target_stale_ms").value) / 1000.0
        )
        self.observation = Latest()
        """제어 스레드가 써 두고 발행 타이머가 꺼냄. 제어 스레드에서 publish 를
        부르지 않으려는 것임 -- 직렬화와 DDS 가 제어 주기 안에 들어감."""

        self.loop: Optional[ControlLoop] = None
        self.thread: Optional[threading.Thread] = None
        self.mode = Mode.OBSERVE
        self.stopped_by_stale = False
        """목표각이 끊겨서 멈춘 것인지. 상태 토픽에 적음."""

        self.joints = self.create_publisher(
            JointState, "/huphy/joint_states", qos_profile_sensor_data)
        self.imu_pub = self.create_publisher(
            Imu, "/huphy/imu", qos_profile_sensor_data)
        self.gravity_pub = self.create_publisher(
            Vector3Stamped, "/huphy/gravity", qos_profile_sensor_data)
        self.status_pub = self.create_publisher(
            DiagnosticArray, "/huphy/loop_status", 10)
        self.create_subscription(
            JointState, "/huphy/joint_targets", self._on_targets,
            qos_profile_sensor_data)
        self.create_service(Trigger, "/huphy/enable", self._on_enable)
        self.create_service(Trigger, "/huphy/disable", self._on_disable)

        self.create_timer(1.0 / float(self.get_parameter("state_hz").value), self._publish)
        self.create_timer(1.0 / STATUS_HZ, self._publish_status)

    # ---- 수명 -------------------------------------------------------------
    def start(self) -> None:
        self.robot.connect()
        self.names = tuple(self.robot.joint_names)
        """관절 이름은 한 번만 읽음. 매 발행마다 로봇에 안 물어봄."""
        self.imus = tuple(self.robot.imus)
        self.get_logger().info(
            f"{self.robot.id}  {self.hz:.0f}Hz  관절 {len(self.names)}개  "
            f"IMU {len(self.imus)}개  — OBSERVE 로 시작 (토크 없음)")
        self._run_loop(Mode.OBSERVE)

    def shutdown(self) -> None:
        self._stop_loop()
        self.robot.disconnect()

    def _run_loop(self, mode: Mode) -> None:
        """그 모드로 루프를 새로 띄움. 돌고 있으면 먼저 멈춤."""
        self._stop_loop()
        self.mode = mode
        self.loop = ControlLoop(self.robot, hz=self.hz, mode=mode)
        self.thread = threading.Thread(
            target=self._loop_body, name=f"huphy-{mode.value}", daemon=True)
        self.thread.start()

    def _loop_body(self) -> None:
        try:
            self.loop.run(self._motion)
        except Exception as e:  # noqa: BLE001 - 스레드가 조용히 죽으면 안 됨
            self.get_logger().error(f"제어 루프가 예외로 끝남: {e}")

    def _stop_loop(self) -> None:
        if self.loop is not None:
            self.loop.stop()
        if self.thread is not None and self.thread.is_alive():
            # CONTROL 이면 finally 가 감쇠 5초를 보냄. 그게 끝나기를 기다림.
            self.thread.join(timeout=JOIN_S)
            if self.thread.is_alive():
                self.get_logger().warning("제어 루프가 제때 안 끝남")
        self.loop, self.thread = None, None

    # ---- 제어 스레드 -------------------------------------------------------
    def _motion(self, t: float, observation):
        """`ControlLoop` 가 매 주기 부름. **제어 스레드에서 돎.**

        관찰을 써 두고 최신 목표각을 냄. `None` 을 내면 그 주기는 아무것도 안 보냄
        (`control/loop.py` 의 `step`).

        목표각이 낡았고 CONTROL 이면 **루프를 멈춤** -- `finally` 가 감쇠 뒤 토크를
        끊음 (`docs/ros.md` 결정 2). 여기서 `stop()` 을 부르는 것은 플래그만 세우는
        일이라 이 주기는 정상으로 끝남.
        """
        del t
        self.observation.put(observation)
        action, fresh = self.targets.take()
        if fresh:
            return action
        if self.mode is Mode.CONTROL and action is not None and self.loop is not None:
            self.get_logger().error(
                f"목표각이 {self.targets.stale_s * 1000:.0f}ms 넘게 안 옴. "
                f"감쇠 후 토크를 끊음")
            self.stopped_by_stale = True
            self.loop.stop()
        return None

    # ---- ROS 콜백 — 짧게 끝낼 것 -------------------------------------------
    def _on_targets(self, msg: JointState) -> None:
        try:
            self.targets.put(convert.arrays_to_targets(msg.name, msg.position))
        except ValueError as e:
            self.get_logger().warning(f"목표각을 버림: {e}")

    def _on_enable(self, request, response):
        del request
        self.targets.clear()
        """켜는 순간 직전 목표각이 한 주기 나가지 않게 비움."""
        self.stopped_by_stale = False
        self._run_loop(Mode.CONTROL)
        response.success = True
        response.message = "CONTROL. 토크 들어감"
        return response

    def _on_disable(self, request, response):
        del request
        self.targets.clear()
        self._run_loop(Mode.OBSERVE)
        response.success = True
        response.message = "OBSERVE. 감쇠 후 토크 끊음"
        return response

    # ---- 발행 -------------------------------------------------------------
    def _publish(self) -> None:
        observation = self.observation.take()
        if not observation:
            return
        stamp = self.get_clock().now().to_msg()

        names, position, velocity, effort = convert.observation_to_arrays(
            observation, self.names)
        msg = JointState()
        msg.header.stamp = stamp
        msg.name, msg.position, msg.velocity, msg.effort = (
            names, position, velocity, effort)
        self.joints.publish(msg)

        states = self.robot.imu_states()
        for imu in self.imus:
            state = states.get(imu.name)
            if state is None:
                continue
            gyro, accel, gravity = convert.imu_arrays(state)
            m = Imu()
            m.header.stamp, m.header.frame_id = stamp, imu.name
            m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = gyro
            (m.linear_acceleration.x, m.linear_acceleration.y,
             m.linear_acceleration.z) = accel
            self.imu_pub.publish(m)

            g = Vector3Stamped()
            g.header.stamp, g.header.frame_id = stamp, imu.name
            g.vector.x, g.vector.y, g.vector.z = gravity
            self.gravity_pub.publish(g)

    def _publish_status(self) -> None:
        loop, stats = self.loop, getattr(self.loop, "stats", None)
        pairs = {
            "mode": self.mode.value,
            "running": str(self.thread is not None and self.thread.is_alive()),
            "target_age_s": f"{self.targets.age():.3f}",
            "target_stale_count": str(self.targets.stale_count),
            "stopped_by_stale": str(self.stopped_by_stale),
        }
        if stats is not None:
            pairs.update({
                "cycles": str(stats.cycles),
                "mean_hz": f"{stats.mean_hz:.1f}",
                "overruns": str(stats.overruns),
                "missing_cycles": str(stats.missing_cycles),
                "link_loss": str(stats.link_loss or ""),
            })
        del loop

        status = DiagnosticStatus()
        status.name = "huphy/control"
        status.hardware_id = self.robot.id
        status.level = (
            DiagnosticStatus.ERROR if self.stopped_by_stale
            else DiagnosticStatus.OK
        )
        status.message = self.mode.value
        status.values = [KeyValue(key=k, value=v) for k, v in pairs.items()]

        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self.status_pub.publish(array)


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
