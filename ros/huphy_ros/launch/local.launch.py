"""한 기계에서 둘 다 띄움. 개발·측정용.

    ros2 launch huphy_ros local.launch.py
    ros2 launch huphy_ros local.launch.py checkpoint:=37400      # 같은 꾸러미의 다른 가중치

실제 배치는 기계 둘임 (ros/README.md 결정 13). 이것은 네트워크 없이 배선을
확인하거나, 두 기계로 나눴을 때와 지연을 비교할 때 씀.

토크는 여전히 /huphy/enable 을 받아야 들어감.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _include(name, arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("huphy_ros"), "launch", name])),
        launch_arguments=arguments,
    )


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value="config/robot.yaml"),
        DeclareLaunchArgument(
            "policy", default_value="config/policies/generalized_walk"),
        DeclareLaunchArgument("checkpoint", default_value=""),
        _include("control.launch.py",
                 [("config", LaunchConfiguration("config"))]),
        _include("policy.launch.py",
                 [("policy", LaunchConfiguration("policy")),
                  ("checkpoint", LaunchConfiguration("checkpoint"))]),
    ])
