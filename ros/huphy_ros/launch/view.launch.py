"""로봇을 RViz 에 3D 로 띄움. 실물 관절각대로 움직임. **화면이 있는 기계**에서 띄움.

    ros2 launch huphy_ros view.launch.py              # robot_state_publisher + RViz
    ros2 launch huphy_ros view.launch.py rviz:=false  # RViz 없이 /tf 만 (화면 없는 기계)

control_node 가 따로 떠 있어야 움직임 (OBSERVE 면 됨 -- 토크 없이 손으로 다리를 움직여 봄).

    /huphy/joint_states ─> linkage_node ─> /huphy/joint_states_view ─> robot_state_publisher ─> /tf ─> RViz
                           (발목 채움)                                     ↑ urdf/huphy.urdf
    world ─(0 으로 고정)─> base_link

robot_state_publisher · rviz2 · static_transform_publisher 는 ROS 기성 프로그램임. 우리가
만든 것은 발목을 채우는 `linkage_node` 하나 (아래 "발목").

## world -> base_link 를 0 으로 박는 이유

몸통이 바닥에 대해 어디 있는지(위치·자세 6자유도) 추정하는 것이 없음. 그래서 몸통은 공중에
고정되고 다리만 실물대로 움직임. IMU 자세를 넣으면 기울기는 보일 수 있음 (아직 안 함).

## 발목

URDF 는 크랭크·막대가 발에 닿는 고리를 못 적음. 발은 `ankle_pitch/roll` 관절에, 막대는
크랭크에 따로 매달려 있음. `joint_states` 에는 크랭크각(`ankle_a/b`)만 있으므로
`linkage_node` 가 URDF 치수로 발 자세와 막대 각도를 풀어 붙임 (`huphy_ros/linkage.py`).
RViz 는 계산을 안 하고 받은 각도대로 그리기만 함.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("huphy_ros")
    with open(os.path.join(share, "urdf", "huphy.urdf"), encoding="utf-8") as f:
        urdf = f.read()

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),
        Node(
            package="huphy_ros",
            executable="linkage_node",
            name="huphy_linkage",
            output="screen",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": urdf}],
            # robot_state_publisher 는 /joint_states 를 들음. 발목을 채운 쪽을 이어 줌.
            remappings=[("/joint_states", "/huphy/joint_states_view")],
            output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="world_to_base_link",
            arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", os.path.join(share, "rviz", "huphy.rviz")],
            condition=IfCondition(LaunchConfiguration("rviz")),
            output="screen",
        ),
    ])
