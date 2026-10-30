"""`huphy` 를 라이브러리로 쓰는 ROS 2 패키지.

    convert.py       단위·이름 변환.          rclpy 를 import 하지 않음
    freshness.py     수신 시각과 나이 판정.    rclpy 를 import 하지 않음
    latency_tap.py   송신·수신 사건을 큐에.    rclpy 를 import 하지 않음
    logbridge.py     파이썬 logging -> ROS 로그
    control_node.py
    policy_node.py
    log_node.py      기록 토픽을 받아 JSON 으로 씀 (선택)
    teleop_node.py   창으로 조작 (선택)
    gamepad_node.py  패드로 조작 (선택)

앞의 셋이 `rclpy` 를 안 쓰는 것이 의도임. 거기 있는 판단이 틀리면 로봇이 잘못
움직이거나 기록이 엉뚱한 관절에 붙는데 **틀려도 길이와 자릿수가 그대로라 코드로는
안 잡힘** -- ROS 없는 기계에서 테스트가 돌아야 고정해 둘 수 있음
(`tests/test_ros_bridge.py`, `tests/test_latency_tap.py`).

메시지 정의는 `ros/huphy_ros_interfaces/` 에 따로 있음. 기록 토픽 넷만 직접 정의한
타입이고 제어 토픽은 표준 타입 그대로임.

`src/huphy/` 는 **공개 API 로만** 씀. 거기에 기능이 늘 수는 있지만 ROS 없이 돌던
것이 깨지면 안 됨 (`ros/README.md` 9장).

계약과 설계는 `ros/README.md`.
"""
