"""작업 기계에서 띄움. 어디로 갈지 정하는 쪽.

    ros2 launch huphy_ros policy.launch.py

띄우기만 하면 아무것도 안 냄. 시작시켜야 함.

    ros2 service call /huphy/start_policy std_srvs/srv/Trigger
    ros2 topic pub -r 20 /huphy/cmd_vel geometry_msgs/msg/Twist \
      '{linear: {x: 0.0}, angular: {z: 0.0}}'

ROS_DOMAIN_ID 가 모션보드와 같아야 서로 보임 -- 다르면 에러 없이 안 보임.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "weights", default_value="config/policies/generalized_walk.pt"),
        DeclareLaunchArgument("hz", default_value="50.0"),
        DeclareLaunchArgument("state_stale_ms", default_value="60.0"),
        DeclareLaunchArgument("approach_s", default_value="3.0"),
        Node(
            package="huphy_ros",
            executable="policy_node",
            name="huphy_policy",
            parameters=[{
                "weights": LaunchConfiguration("weights"),
                "hz": LaunchConfiguration("hz"),
                "state_stale_ms": LaunchConfiguration("state_stale_ms"),
                "approach_s": LaunchConfiguration("approach_s"),
            }],
            output="screen",
            emulate_tty=True,
        ),
    ])
