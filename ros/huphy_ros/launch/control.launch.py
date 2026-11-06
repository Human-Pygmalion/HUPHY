"""모션보드에서 띄움. 모터를 모는 쪽.

    ros2 launch huphy_ros control.launch.py
    ros2 launch huphy_ros control.launch.py motor_hz:=50.0        # 제어 주기를 낮춰 볼 때
    ros2 launch huphy_ros control.launch.py extra:=2               # 전류·전압도 기록할 때

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
        # 런치 인자 이름이 `hz` 가 아니라 `motor_hz` 임. `local.launch.py` 가
        # 이 런치와 policy.launch.py 를 **같은 문맥에** include 하고,
        # DeclareLaunchArgument 는 이미 있는 이름에 기본값을 안 넣음 -- 이름이
        # 같으면 먼저 선언된 쪽 값이 뒤쪽 노드에 그대로 들어감. 노드 파라미터
        # 이름은 `hz` 그대로임.
        DeclareLaunchArgument("motor_hz", default_value="100.0"),
        DeclareLaunchArgument("state_hz", default_value="100.0"),
        DeclareLaunchArgument("drain_ms", default_value="4.0"),
        DeclareLaunchArgument("send_mode", default_value="burst"),
        DeclareLaunchArgument("target_stale_ms", default_value="100.0"),
        DeclareLaunchArgument("allow_uncalibrated", default_value="false"),
        # 0 = 전류·전압을 안 읽음. 1 이상이면 한 주기에 다리마다 그만큼 파라미터
        # 읽기 요청을 보냄. 모터 6개 x 값 2개 = 12칸을 돌므로 2 면 값 하나가
        # 약 17Hz 로 갱신됨 (100Hz 기준). 그만큼 CAN 프레임이 늘어남.
        DeclareLaunchArgument("extra", default_value="0"),
        Node(
            package="huphy_ros",
            executable="control_node",
            name="huphy_control",
            parameters=[{
                "config": LaunchConfiguration("config"),
                "hz": LaunchConfiguration("motor_hz"),
                "state_hz": LaunchConfiguration("state_hz"),
                "drain_ms": LaunchConfiguration("drain_ms"),
                "send_mode": LaunchConfiguration("send_mode"),
                "target_stale_ms": LaunchConfiguration("target_stale_ms"),
                "allow_uncalibrated": LaunchConfiguration("allow_uncalibrated"),
                "extra": LaunchConfiguration("extra"),
            }],
            output="screen",
            emulate_tty=True,
        ),
    ])
