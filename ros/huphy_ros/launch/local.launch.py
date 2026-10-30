"""한 기계에서 둘 다 띄움. 개발·측정용.

    ros2 launch huphy_ros local.launch.py
    ros2 launch huphy_ros local.launch.py checkpoint:=37400      # 같은 꾸러미의 다른 가중치
    ros2 launch huphy_ros local.launch.py debug:=true            # 기록까지 (log_node 포함)

실제 배치는 기계 둘임 (ros/README.md 결정 13). 이것은 네트워크 없이 배선을
확인하거나, 두 기계로 나눴을 때와 지연을 비교할 때 씀.

토크는 여전히 /huphy/enable 을 받아야 들어감.

`debug:=true` 면 기록 노드까지 띄우고 양쪽 노드의 기록용 토픽을 켬. 파일은
`log:=` 가 가리키는 폴더에 남음 (기본 `$HOME/HUPHY/ros/logs`). 자세한 것은
`log.launch.py`.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare


def _include(name, arguments, condition=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("huphy_ros"), "launch", name])),
        launch_arguments=arguments,
        condition=condition,
    )


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value="config/robot.yaml"),
        DeclareLaunchArgument(
            "policy", default_value="config/policies/generalized_walk"),
        DeclareLaunchArgument("checkpoint", default_value=""),
        DeclareLaunchArgument("debug", default_value="false"),
        DeclareLaunchArgument(
            "log", default_value=[EnvironmentVariable("HOME"), "/HUPHY/ros/logs"]),
        _include("control.launch.py",
                 [("config", LaunchConfiguration("config")),
                  ("debug", LaunchConfiguration("debug"))]),
        _include("policy.launch.py",
                 [("policy", LaunchConfiguration("policy")),
                  ("checkpoint", LaunchConfiguration("checkpoint")),
                  ("debug", LaunchConfiguration("debug"))]),
        # 기록 노드는 debug 일 때만. 꺼 두면 아무 토픽도 안 와서 빈 채로 돎.
        _include("log.launch.py",
                 [("log", LaunchConfiguration("log"))],
                 condition=IfCondition(LaunchConfiguration("debug"))),
    ])
