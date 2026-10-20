"""관찰 노드 — 로봇을 **읽기만** 함. 모터를 안 움직임.

    ros2 run huphy_ros telemetry_node --ros-args -p config:=/경로/robot.yaml

## 가장 먼저 만드는 것

`docs/ros.md` 13장의 3단계임. 토크를 넣기 전에 ROS 쪽 배선을 전부 확인하는 자리임.

    단위        도가 아니라 라디안으로 나가나
    이름        right_leg__knee 로 나가나
    주기        ros2 topic hz 로 실제 몇 Hz 인가
    QoS         받는 쪽이 실제로 붙나

`Mode.OBSERVE` 로 돌므로 `ControlLoop._enter` 가 **먼저 토크를 끊음**
(`control/loop.py`). 설정이 잘못돼 있어도 다리가 안 움직임.

## 제어 노드와 같이 띄우면 안 됨

둘 다 같은 CAN 채널을 열려고 함. 채널은 socketcan 이라 열리기는 하는데, 두
프로세스가 같은 버스에 명령을 보내면 모터 응답이 섞여서 어느 쪽 것인지 구분이 안 됨
(`motors/robstride/bus.py` 의 `collect_until` 이 id 로 짝을 맞추는데, 같은 id 응답이
둘 오면 먼저 온 것을 씀).

상태만 보고 싶으면 이 노드, 움직이려면 `control_node` 임. `control_node` 도 관찰을
내보냄.
"""

from __future__ import annotations

import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float32MultiArray

from huphy.config import load_robot
from huphy.control import ControlLoop, Mode
from huphy.scripts.bringup import build_biped, build_leg
from huphy.scripts.commission import all_legs

from . import convert
from .bridge import CommandBridge

PUBLISH_HZ = 50.0


class TelemetryNode(Node):
    def __init__(self) -> None:
        super().__init__("huphy_telemetry")

        self.declare_parameter("config", "")
        self.declare_parameter("limb", "")
        self.declare_parameter("hz", 100.0)
        self.declare_parameter("drain_ms", 2.0)

        path = self.get_parameter("config").value
        if not path:
            raise SystemExit("config 파라미터에 robot.yaml 경로를 줄 것")

        robot_config = load_robot(path)
        limb = self.get_parameter("limb").value
        options = dict(
            allow_uncalibrated=True,
            drain_s=float(self.get_parameter("drain_ms").value) / 1000.0,
        )
        # 읽기만 하므로 실측값이 없어도 진행함. 토크가 안 나가서 위험하지 않고,
        # 실측 전에 센서 배선을 확인하는 것이 이 노드의 용도임.
        if limb:
            self.robot = build_leg(robot_config, robot_config.limbs[limb], **options)
        else:
            self.robot = build_biped(robot_config, all_legs(robot_config), **options)

        self.bridge = CommandBridge()
        self.joints = self.create_publisher(
            JointState, "~/joint_states", qos_profile_sensor_data
        )
        self.imus = {
            imu.name: self.create_publisher(
                Imu, f"~/imu/{imu.name}", qos_profile_sensor_data
            )
            for imu in self.robot.imus
        }
        self.gravity = {
            imu.name: self.create_publisher(
                Float32MultiArray, f"~/imu/{imu.name}/gravity", qos_profile_sensor_data
            )
            for imu in self.robot.imus
        }
        """중력방향은 `sensor_msgs/Imu` 에 자리가 없음. 정책이 실제로 쓰는 값이라
        쿼터니언 왕복 없이 그대로 내보냄 (`convert.imu_arrays` 참조)."""

        self.create_timer(1.0 / PUBLISH_HZ, self._publish)
        self.loop = ControlLoop(
            self.robot, hz=float(self.get_parameter("hz").value), mode=Mode.OBSERVE
        )
        self._thread = threading.Thread(
            target=self.loop.run, args=(self.bridge.motion,), daemon=True
        )

    def start(self) -> None:
        self.robot.connect()
        self._names = tuple(self.robot.joint_names)
        self.get_logger().info(
            f"{self.robot.id} 관찰 전용 {self.loop.hz:.0f}Hz "
            f"관절 {len(self._names)}개 IMU {len(self.imus)}개"
        )
        self._thread.start()

    def shutdown(self) -> None:
        self.loop.stop()
        if self._thread.is_alive():
            self._thread.join(timeout=10.0)
        self.robot.disconnect()

    def _publish(self) -> None:
        observation = self.bridge.take_observation()
        if observation:
            names, position, velocity, effort = convert.observation_to_arrays(
                observation, self._names
            )
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = names
            msg.position = position
            msg.velocity = velocity
            msg.effort = effort
            self.joints.publish(msg)

        states = self.robot.imu_states()
        for name, publisher in self.imus.items():
            state = states.get(name)
            if state is None:
                continue
            gyro, accel, grav = convert.imu_arrays(state)
            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = name
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = gyro
            (msg.linear_acceleration.x, msg.linear_acceleration.y,
             msg.linear_acceleration.z) = accel
            publisher.publish(msg)

            g = Float32MultiArray()
            g.data = [float(v) for v in grav]
            self.gravity[name].publish(g)


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = TelemetryNode()
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
