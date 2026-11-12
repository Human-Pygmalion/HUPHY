"""한 기계에서 둘 다 띄움. 개발·측정용.

    ros2 launch huphy_ros local.launch.py
    ros2 launch huphy_ros local.launch.py checkpoint:=37400      # 같은 꾸러미의 다른 가중치

실제 배치는 기계 둘임 (ros/README.md 결정 13). 이것은 네트워크 없이 배선을
확인하거나, 두 기계로 나눴을 때와 지연을 비교할 때 씀.

토크는 여전히 /huphy/enable 을 받아야 들어감.

기록은 다른 터미널에서 `log.launch.py` 를 띄움.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare


# include 는 런치 설정 범위를 새로 안 만들고, DeclareLaunchArgument 는 이미
# 있는 이름에 기본값을 안 넣음. 그래서 아래 두 런치의 인자 이름이 겹치면 먼저
# include 한 쪽 값이 뒤쪽 노드에 들어감 (`motor_hz` / `policy_hz` 를 따로 둔
# 이유). 인자를 더할 때 두 파일에 같은 이름을 쓰지 말 것.
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
            "policy", default_value="config/policies/walk"),
        DeclareLaunchArgument("checkpoint", default_value=""),
        DeclareLaunchArgument("extra", default_value="0"),
        _include("control.launch.py",
                 [("config", LaunchConfiguration("config")),
                  ("extra", LaunchConfiguration("extra"))]),
        _include("policy.launch.py",
                 [("policy", LaunchConfiguration("policy")),
                  ("checkpoint", LaunchConfiguration("checkpoint"))]),
    ])
