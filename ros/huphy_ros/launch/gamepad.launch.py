"""작업 기계에서 띄움. Xbox 패드로 명령·시작·멈춤. `joy_node` 와 `gamepad_node` 를 같이 올림.

    ros2 launch huphy_ros gamepad.launch.py policy:=$HOME/HUPHY/config/policies/generalized_walk
    ros2 launch huphy_ros gamepad.launch.py policy:=$HOME/HUPHY/config/policies/huphy_squat \
        device:=/dev/input/js1 max:="vx=0.8"

`policy` 는 policy_node 에 준 것과 같은 꾸러미여야 함 -- 그 정책의 명령만 조작함.
화면은 필요 없음 (`teleop.launch.py` 와 다른 점).

    왼쪽 스틱 세로 vx   왼쪽 스틱 가로 vy   오른쪽 스틱 가로 wz   십자키 세로 h_star
    RB       누르고 있는 동안만 명령이 먹음 (데드맨). 떼면 속도 0
    A        기준 자세로 (/huphy/approach) -- 토크가 켜져 있을 때만
    RB+Menu  정책 시작 (/huphy/start_policy)
    B        정책·토크 끔 (/huphy/stop_policy -> /huphy/disable)
    View 1초 토크 넣기 (/huphy/enable)

축·버튼 번호는 드라이버마다 다름. `ros2 topic echo /joy` 로 보고 `axis_vx:=1`,
`button_deadman:=5` 처럼 덮어쓸 것 (이름은 gamepad_node.py 의 AXES·BUTTONS).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "policy", default_value="config/policies/generalized_walk"),
        DeclareLaunchArgument("device", default_value="/dev/input/js0"),
        DeclareLaunchArgument("max", default_value=""),
        DeclareLaunchArgument("accel", default_value=""),
        DeclareLaunchArgument("deadzone", default_value="0.08"),
        DeclareLaunchArgument("enable_hold_s", default_value="1.0"),
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            parameters=[{
                "device_name": "",
                "dev": LaunchConfiguration("device"),
                # joy_node 자체의 불감대는 0 으로 두고 gamepad_node 에서 한 번만 자름.
                # 두 군데서 자르면 실제로 적용되는 값이 뭔지 알기 어려움.
                "deadzone": 0.0,
                "autorepeat_rate": 20.0,
            }],
            output="screen",
            emulate_tty=True,
        ),
        Node(
            package="huphy_ros",
            executable="gamepad_node",
            name="huphy_gamepad",
            parameters=[{
                "policy": LaunchConfiguration("policy"),
                "max": LaunchConfiguration("max"),
                "accel": LaunchConfiguration("accel"),
                "deadzone": LaunchConfiguration("deadzone"),
                "enable_hold_s": LaunchConfiguration("enable_hold_s"),
            }],
            output="screen",
            emulate_tty=True,
        ),
    ])
