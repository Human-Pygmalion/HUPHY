"""전류계 노드 — 로봇 전체 DC 전류를 재서 냄.

    ros2 launch huphy_ros current.launch.py
    ros2 launch huphy_ros current.launch.py port:=/dev/ttyUSB1
    ros2 run huphy_ros current_node --ros-args -p port:=/dev/ttyUSB1

    current_node  ─>  /huphy/bus_current  ─>  log_node  ─>  current/total_a 열

제어도 추론도 안 함. `log_node` 가 정책 줄마다 가장 최근 값을 그 줄에 넣음.


## 왜 따로 있나

**필요할 때만 꽂는 장비임.** 전류계가 없는 실행이 대부분이라 `control_node` 에 넣으면
안 꽂혀 있을 때마다 그 노드가 경고를 내거나 포트를 뒤짐.

그리고 `control_node` 에 넣으면 **제어 주기에 끼어듦.** 장비는 9600 baud 라 8바이트
프레임 하나가 선로에 8.3ms 걸림 -- 100Hz 주기가 10ms 임. 읽기는 스레드가 하므로
주기 안에서 기다리지는 않지만, 같은 프로세스에 둘 이유가 없음.

전류계는 **어느 기계에 꽂아도 됨.** 모터와 달리 CAN 과 무관해서, 작업 기계에 꽂고
이 노드만 그쪽에서 띄워도 `log_node` 가 받음 (`ros/README.md` 결정 13).


## 왜 타이머로 내나

장비 출력 주기가 로그 주기(50Hz)보다 느림. 프레임마다 발행하면 발행 주기가 장비에
끌려가고, 받는 쪽은 어차피 줄마다 최근 값 하나만 씀. 그래서 `rate_hz` 로 내고
`age_ms` 를 같이 실어 **그 값이 언제 온 것인지**를 받는 쪽이 알게 함.


## 못 열리면 멈춤

여기는 전류계를 재는 것이 일이므로, 포트를 못 열면 노드가 멈춤 (`SystemExit`).
전류계 때문에 로봇이 멈추지는 않음 -- 이 노드는 아무것도 제어하지 않고, 제어·정책
노드는 이 토픽을 보지 않음. 기록에 열이 안 생길 뿐임.
"""

from __future__ import annotations

import rclpy
from huphy_ros_interfaces.msg import BusCurrent
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from huphy.sensors.wcm1600 import Wcm1600

from . import logbridge

DEFAULT_HZ = 20.0
"""기본 발행 주기. 로그 주기(50Hz)보다 느려도 되는 이유는 위 docstring 에 있음."""


class CurrentNode(Node):
    def __init__(self) -> None:
        super().__init__("huphy_current")
        self.log_handler = logbridge.forward(self.get_logger())

        self.declare_parameter("port", "")
        self.declare_parameter("rate_hz", DEFAULT_HZ)
        port = str(self.get_parameter("port").value) or None
        hz = float(self.get_parameter("rate_hz").value)
        if hz <= 0.0:
            raise SystemExit(f"rate_hz 는 0 보다 커야 함 (받은 값 {hz})")

        self.meter = Wcm1600(port=port)
        try:
            self.meter.connect()
        except Exception as e:
            raise SystemExit(f"전류계를 열지 못함: {e}") from e

        self.pub = self.create_publisher(
            BusCurrent, "/huphy/bus_current", qos_profile_sensor_data)
        self.create_timer(1.0 / hz, self._tick)
        self.warned_ac = False
        self.get_logger().info(
            f"전류계 {self.meter.port} 를 {hz:.0f}Hz 로 냄 -> /huphy/bus_current "
            f"(기록에 열을 남기려면 log_node 를 띄울 것)")

    def _tick(self) -> None:
        state = self.meter.read()
        msg = BusCurrent()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.amps = float(state.amps) if state.is_valid else float("nan")
        msg.age_ms = float(state.age_ms())
        msg.is_ac = bool(state.is_ac)
        self.pub.publish(msg)

        if state.is_ac and not self.warned_ac:
            self.warned_ac = True
            self.get_logger().warning(
                "AC 프레임이 옴. 이 로봇은 DC 이므로 장비 설정을 확인할 것")
        if not state.is_valid:
            self.get_logger().warning(
                "전류계 프레임을 아직 못 받았음 -- Continuous 모드인지 확인할 것",
                throttle_duration_sec=5.0)

    def destroy_node(self) -> None:
        self.meter.disconnect()
        super().destroy_node()


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = CurrentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        logbridge.stop(node.log_handler)
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
