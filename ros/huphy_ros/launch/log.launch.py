"""기록 노드. `huphy-run --log` 과 같은 JSON 파일을 남김.

    ros2 launch huphy_ros log.launch.py
    ros2 launch huphy_ros log.launch.py log:=/mnt/usb/huphy_logs

**정책을 시작하기 전에 띄워 둘 것.** 그 전에 흘러간 줄은 못 받음.

내는 쪽도 켜야 함 -- `control.launch.py` 와 `policy.launch.py` 에 `debug:=true`.
`local.launch.py` 는 셋을 한 번에 띄우고 `debug` 를 양쪽에 넘김.

    ros2 launch huphy_ros control.launch.py debug:=true     # 모션보드
    ros2 launch huphy_ros policy.launch.py  debug:=true     # 작업 기계
    ros2 launch huphy_ros log.launch.py                     # 아무 기계

끝나면 두 파일이 나옴 (`<log>` 아래).

    policy/<월일>_policy_<정책>.json     tools/policy_viewer.html
    latency/<월일>_latency_<정책>.json   tools/latency_viewer.html

기본 폴더가 **절대경로**인 이유: 상대경로면 이 런치를 어느 폴더에서 쳤는지에 따라
파일이 다른 데 생김. `$HOME` 은 노드를 띄우는 기계에서 풀리므로 두 기계의 사용자
이름이 달라도 각자 자기 홈을 가리킴.

레포를 분리하면 이 기본값 한 줄을 그 레포 경로로 바꾸면 됨. 노드 코드에는 경로가
들어 있지 않음 -- 설치된 노드는 소스가 어디인지 알 수 없기 때문임
(`ros/README.md` 기록 절).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "log", default_value=[EnvironmentVariable("HOME"), "/HUPHY/ros/logs"]),
        Node(
            package="huphy_ros",
            executable="log_node",
            name="huphy_log",
            parameters=[{
                # 경로를 숫자로 읽힐 일은 없지만, 다른 런치와 같은 방식으로 둠.
                "log": ParameterValue(LaunchConfiguration("log"), value_type=str),
            }],
            output="screen",
            emulate_tty=True,
        ),
    ])
