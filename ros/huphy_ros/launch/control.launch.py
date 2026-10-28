"""모션보드에서 띄움. 모터를 모는 쪽.

    ros2 launch huphy_ros control.launch.py

**토크는 안 들어감.** OBSERVE 로 시작하고 /huphy/enable 서비스를 받아야 들어감.

    ros2 topic echo /huphy/joint_states --once
    ros2 topic hz /huphy/joint_states
    ros2 service call /huphy/enable std_srvs/srv/Trigger
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value="config/robot.yaml"),
        DeclareLaunchArgument("hz", default_value="100.0"),
        DeclareLaunchArgument("state_hz", default_value="100.0"),
        DeclareLaunchArgument("drain_ms", default_value="2.0"),
        DeclareLaunchArgument("send_mode", default_value="burst"),
        DeclareLaunchArgument("target_stale_ms", default_value="100.0"),
        DeclareLaunchArgument("allow_uncalibrated", default_value="false"),
        Node(
            package="huphy_ros",
            executable="control_node",
            name="huphy_control",
            parameters=[{
                "config": LaunchConfiguration("config"),
                "hz": LaunchConfiguration("hz"),
                "state_hz": LaunchConfiguration("state_hz"),
                "drain_ms": LaunchConfiguration("drain_ms"),
                "send_mode": LaunchConfiguration("send_mode"),
                "target_stale_ms": LaunchConfiguration("target_stale_ms"),
                "allow_uncalibrated": LaunchConfiguration("allow_uncalibrated"),
            }],
            output="screen",
            emulate_tty=True,
        ),
    ])
