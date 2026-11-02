"""RViz 용으로 발목 링키지를 채움. `view.launch.py` 가 띄움 -- 따로 띄울 일 없음.

    /huphy/joint_states  ─>  linkage_node  ─>  /huphy/joint_states_view  ─>  robot_state_publisher

`joint_states` 에는 모터 12개만 있음 (발목은 크랭크 `ankle_a/b`). URDF 의 발은
`ankle_pitch/roll` 에, 막대는 크랭크에 따로 매달려 있어서 이 값들이 없으면 발이 가만히 있고
막대가 발에서 떨어져 보임. 크랭크각에서 그 12개(발 4 + 막대 8)를 풀어 붙여 냄
(`linkage.py`).

**보기용임.** 제어·정책은 이 토픽을 안 봄. 계산이 한 번에 수 ms 라 화면 주기(`hz`, 기본
30)로만 풂 -- 100Hz 를 다 풀 필요가 없음.
"""

from __future__ import annotations

import os
from typing import Optional

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

from . import logbridge
from .linkage import Linkage


class LinkageNode(Node):
    def __init__(self) -> None:
        super().__init__("huphy_linkage")
        self.log_handler = logbridge.forward(self.get_logger())
        self.declare_parameter("hz", 30.0)
        path = os.path.join(get_package_share_directory("huphy_ros"), "urdf", "huphy.urdf")
        with open(path, encoding="utf-8") as f:
            self.linkage = Linkage(f.read())
        self.latest: Optional[JointState] = None
        self.pub = self.create_publisher(
            JointState, "/huphy/joint_states_view", qos_profile_sensor_data)
        self.create_subscription(
            JointState, "/huphy/joint_states", self._on_joints, qos_profile_sensor_data)
        hz = float(self.get_parameter("hz").value)
        self.create_timer(1.0 / hz, self._tick)
        self.get_logger().info(f"발목 링키지 {len(self.linkage.names)}개를 {hz:.0f}Hz 로 채움")

    def _on_joints(self, msg: JointState) -> None:
        self.latest = msg

    def _tick(self) -> None:
        msg, self.latest = self.latest, None
        if msg is None:
            return
        q = dict(zip(msg.name, msg.position))
        solved = self.linkage.solve(q)
        if self.linkage.worst_m > self.linkage.tol_m:
            self.get_logger().warning(
                f"발목 링키지를 못 풂 (막대 끝이 {self.linkage.worst_m * 1000:.1f}mm 어긋남) -- "
                f"크랭크각이 모델 범위 밖. 직전 자세로 그림",
                throttle_duration_sec=2.0)
        out = JointState()
        out.header = msg.header
        keep = [i for i, n in enumerate(msg.name) if n not in solved]
        out.name = [msg.name[i] for i in keep] + list(solved)
        out.position = [msg.position[i] for i in keep] + list(solved.values())
        self.pub.publish(out)


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = LinkageNode()
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
