"""제어 노드를 띄움.

    # 관찰만 (기본). 토크 안 들어감
    ros2 launch huphy_ros control.launch.py

    # 토크를 넣음. 명시해야 함
    ros2 launch huphy_ros control.launch.py observe_only:=false

`observe_only` 기본이 `true` 인 것이 의도임 -- 인자를 빠뜨렸을 때 안 움직이는 쪽으로
떨어지게 함.

토크를 넣은 뒤 명령을 보내려면

    ros2 service call /huphy_control/enable std_srvs/srv/Trigger
    ros2 topic pub -r 50 /huphy_control/command sensor_msgs/msg/JointState \
      '{name: ["right_leg__knee"], position: [0.35]}'

명령이 0.2초 넘게 안 오면 전송이 멈춤 (`bridge.DEFAULT_STALE_S`). `-r 50` 으로
계속 보내야 함.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DEFAULT_CONFIG = "/home/chanhue/Desktop/chanhuegit/HUPHY/config/robot.yaml"


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=DEFAULT_CONFIG),
        DeclareLaunchArgument("observe_only", default_value="true"),
        DeclareLaunchArgument("allow_uncalibrated", default_value="false"),
        DeclareLaunchArgument("hz", default_value="100.0"),
        Node(
            package="huphy_ros",
            executable="control_node",
            name="huphy_control",
            parameters=[{
                "config": LaunchConfiguration("config"),
                "observe_only": LaunchConfiguration("observe_only"),
                "allow_uncalibrated": LaunchConfiguration("allow_uncalibrated"),
                "hz": LaunchConfiguration("hz"),
                "drain_ms": 2.0,
                "stale_s": 0.2,
            }],
            output="screen",
            emulate_tty=True,
        ),
    ])
