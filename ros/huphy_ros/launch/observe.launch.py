"""읽기만 하는 노드를 띄움. **토크가 안 들어감.**

    ros2 launch huphy_ros observe.launch.py

ROS 쪽 배선을 확인하는 자리임 (`docs/ros.md` 13장 3단계). 단위·이름·주기·QoS 를
여기서 전부 확인한 뒤에 control.launch.py 로 감.

    ros2 topic list
    ros2 topic echo /huphy_telemetry/joint_states --once
    ros2 topic hz /huphy_telemetry/joint_states
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DEFAULT_CONFIG = "/home/chanhue/Desktop/chanhuegit/HUPHY/config/robot.yaml"


def generate_launch_description():
    config = LaunchConfiguration("config")
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=DEFAULT_CONFIG),
        Node(
            package="huphy_ros",
            executable="telemetry_node",
            name="huphy_telemetry",
            parameters=[{"config": config, "hz": 100.0, "drain_ms": 2.0}],
            output="screen",
            emulate_tty=True,
        ),
    ])
