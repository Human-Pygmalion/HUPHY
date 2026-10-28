"""작업 기계에서 띄움. `huphy-run --teleop` 과 같은 pygame 창으로 명령을 조작함.

    ros2 launch huphy_ros teleop.launch.py policy:=$HOME/HUPHY/config/policies/huphy_squat
    ros2 launch huphy_ros teleop.launch.py policy:=$HOME/HUPHY/config/policies/generalized_walk \
        max:="vx=0.8" accel:="vx=0.4" decel:="vx=1.5"

`policy` 는 policy_node 에 준 것과 같은 꾸러미여야 함 -- 그 정책의 명령만 조작함.
화면이 있어야 함 (ssh 면 -X). 키는 ros/README.md 의 "창으로 조작".

    R      기준 자세로 (/huphy/approach) -- 토크가 켜져 있을 때만
    Enter  정책 시작 (/huphy/start_policy)
    Esc    정책·토크 끔 (/huphy/stop_policy -> /huphy/disable)

토크 켜기(/huphy/enable)는 키로 안 함. 명령으로 침.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "policy", default_value="config/policies/generalized_walk"),
        DeclareLaunchArgument("max", default_value=""),
        DeclareLaunchArgument("accel", default_value=""),
        DeclareLaunchArgument("decel", default_value=""),
        Node(
            package="huphy_ros",
            executable="teleop_node",
            name="huphy_teleop",
            parameters=[{
                "policy": LaunchConfiguration("policy"),
                "max": LaunchConfiguration("max"),
                "accel": LaunchConfiguration("accel"),
                "decel": LaunchConfiguration("decel"),
            }],
            output="screen",
            emulate_tty=True,
        ),
    ])
