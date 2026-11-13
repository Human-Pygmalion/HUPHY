"""전류계 노드. 로봇 전체 DC 전류를 재서 `log_node` 가 기록에 남기게 함.

    ros2 launch huphy_ros current.launch.py
    ros2 launch huphy_ros current.launch.py port:=/dev/ttyUSB1
    ros2 launch huphy_ros current.launch.py rate_hz:=50

**필요할 때만 띄우는 런치임.** 다른 런치에 들어 있지 않음 -- 전류계가 없는 실행이
대부분이고, 없으면 노드가 포트를 못 열어 멈춤.

`log_node` 와 같이 띄워야 기록에 열이 남음 (`current/total_a`, `current/age`).
전류계가 꽂힌 기계에서 이 런치를 띄울 것 -- 모터와 달리 CAN 과 무관해서 작업 기계든
모션보드든 상관없음.

`port` 를 비워 두면 USB 직렬 장치를 찾음. **IMU 도 USB 로 붙어 있으면 반드시
직접 줄 것** -- 둘 다 FTDI 면 어느 것인지 알 수 없어 노드가 멈춤.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("port", default_value=""),
        DeclareLaunchArgument("rate_hz", default_value="20.0"),
        Node(
            package="huphy_ros",
            executable="current_node",
            name="huphy_current",
            parameters=[{
                "port": ParameterValue(LaunchConfiguration("port"), value_type=str),
                "rate_hz": ParameterValue(
                    LaunchConfiguration("rate_hz"), value_type=float),
            }],
            output="screen",
            emulate_tty=True,
        ),
    ])
