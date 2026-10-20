"""HUPHY 제어를 ROS 2 노드로 감싸는 패키지.

**`huphy` 를 고치지 않음.** import 만 함.

    convert.py        단위·이름 변환. rclpy 를 import 하지 않음
    bridge.py         토픽 <-> Motion. rclpy 를 import 하지 않음
    control_node.py   제어 루프를 자기 스레드에서 돌리고 ROS 와 이어 줌
    telemetry_node.py 상태를 토픽으로 내보냄

앞의 둘이 `rclpy` 를 안 쓰는 것이 의도임 -- ROS 가 없는 기계에서도 테스트가 돎
(`tests/test_ros_bridge.py`). 노드 쪽은 배선뿐이라 테스트할 것이 적음.

설계 근거는 `docs/ros.md`.
"""
