"""작업 기계에서 띄움. 기준 궤적을 재생하는 쪽.

    ros2 launch huphy_ros motion.launch.py policy:=$HOME/HUPHY/config/policies/union_dr_00545

`policy.launch.py` 와 같은 자리임 -- 둘 중 하나만 띄울 것. 둘 다 띄우면 같은
`joint_targets` 토픽에 두 노드가 목표각을 내 모터가 둘을 번갈아 받음.

띄우기만 하면 아무것도 안 냄. 기준 자세로 보낸 뒤 시작시켜야 함.

    ros2 service call /huphy/approach std_srvs/srv/Trigger       # 기준 자세로 가서 기다림
    ros2 service call /huphy/start_policy std_srvs/srv/Trigger   # 재생 시작
    ros2 topic echo /huphy/policy_status                         # frame / progress 확인
    ros2 service call /huphy/stop_policy std_srvs/srv/Trigger    # 감쇠

명령 토픽은 없음 -- 따라갈 궤적이 꾸러미 안에 있음.

끝까지 재생하면 단계가 `done` 이 되고 **마지막 목표각을 유지함.** 쪼그려 앉은
자세로 끝나는 궤적이라 거기서 토크를 끊으면 주저앉음. 복귀는 `approach`,
정지는 `stop_policy`.

ROS_DOMAIN_ID 가 모션보드와 같아야 서로 보임 -- 다르면 에러 없이 안 보임.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "policy", default_value="config/policies/union_dr_00545"),
        DeclareLaunchArgument("checkpoint", default_value=""),
        # `control.launch.py` 의 `motor_hz` 와 같은 이유로 노드 파라미터 이름
        # (`hz`) 과 다르게 둠. 런치 인자 이름이 겹치면 같은 문맥에 include 한
        # 쪽의 값이 들어감 (ros/README.md 의 "런치 인자 이름").
        DeclareLaunchArgument("motion_hz", default_value="0.0"),
        DeclareLaunchArgument("state_stale_ms", default_value="60.0"),
        DeclareLaunchArgument("approach_s", default_value="3.0"),
        DeclareLaunchArgument("start_frame", default_value="0"),
        Node(
            package="huphy_ros",
            executable="motion_node",
            name="huphy_motion",
            parameters=[{
                "policy": LaunchConfiguration("policy"),
                # 숫자만 주면 ROS 가 정수로 읽어 문자열 파라미터와 타입이 안 맞음.
                "checkpoint": ParameterValue(LaunchConfiguration("checkpoint"), value_type=str),
                "hz": LaunchConfiguration("motion_hz"),
                "state_stale_ms": LaunchConfiguration("state_stale_ms"),
                "approach_s": LaunchConfiguration("approach_s"),
                "start_frame": LaunchConfiguration("start_frame"),
            }],
            output="screen",
            emulate_tty=True,
        ),
    ])
