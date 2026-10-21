"""HUPHY 제어를 ROS 2 노드로 감싸는 패키지.

    convert.py     단위·이름 변환.        rclpy 를 import 하지 않음
    freshness.py   수신 시각과 나이 판정.  rclpy 를 import 하지 않음
    control_node.py
    policy_node.py

앞의 둘이 `rclpy` 를 안 쓰는 것이 의도임. 거기 있는 판단이 틀리면 로봇이 잘못
움직이는데 **틀려도 길이와 자릿수가 그대로라 코드로는 안 잡힘** -- ROS 없는
기계에서 테스트가 돌아야 고정해 둘 수 있음.

`src/huphy/` 는 고치지 않음. import 만 함.

계약은 `docs/ros.md`.
"""
