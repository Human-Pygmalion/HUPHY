"""테스트가 `ros/huphy_ros` 도 import 할 수 있게 함.

`tests/test_ros_bridge.py` 가 `huphy_ros.convert` / `huphy_ros.bridge` 를 봄. 그 둘은
**`rclpy` 를 import 하지 않아** ROS 없는 기계에서도 돎.

`PYTHONPATH` 에 손으로 넣지 않고 여기서 거는 이유: 지금 `PYTHONPATH=src` 하나만
걸면 되는 호출 방식을 바꾸지 않으려는 것임.

    PYTHONPATH=src python3 -m pytest -q        <- 이대로 전부 돎

ROS 가 깔린 기계에서 colcon 이 설치한 `huphy_ros` 가 이미 경로에 있으면 그쪽이
먼저 잡힘 -- `sys.path` 뒤에 붙이므로 덮어쓰지 않음.
"""

import sys
from pathlib import Path

ROS_PACKAGE = Path(__file__).resolve().parent.parent / "ros" / "huphy_ros"

if ROS_PACKAGE.is_dir():
    sys.path.append(str(ROS_PACKAGE))
