"""D455 를 띄움. 인텔 공식 노드(`realsense2_camera`)를 우리 설정으로 부르기만 함.

    ros2 launch huphy_ros camera.launch.py
    ros2 launch huphy_ros camera.launch.py pointcloud:=true color:=true
    ros2 launch huphy_ros camera.launch.py depth_profile:=424x240x30 decimation:=true

**노드를 우리가 만들지 않음.** 카메라를 열어 토픽으로 내는 일은 인텔이 만든
`realsense2_camera_node` (C++) 가 다 함 -- USB 통신, 스트림 설정, ROS 메시지로
포장, `CameraInfo`, 카메라 안 좌표계의 `/tf_static`. 이 파일은 그 패키지의
`rs_launch.py` 를 포함하면서 인자만 우리 값으로 고정함.

깔려 있어야 함 (`ros/install.sh` 가 안 깖 -- README "D455 설치" 참조):

    sudo apt install -y ros-humble-librealsense2* ros-humble-realsense2-*

**카메라가 USB 로 꽂힌 기계에서 띄움.** 로봇에서는 모션보드, 책상에서 확인할
때는 작업 기계. 이 노드는 혼자 돌고 다른 노드와 아무 관계가 없음 -- 구독자가
없어도 그냥 토픽을 냄.

## 나오는 토픽 (기본 설정)

    /huphy/d455/depth/image_rect_raw   sensor_msgs/Image       깊이. Z16, 한 칸이 1mm
    /huphy/d455/depth/camera_info      sensor_msgs/CameraInfo  초점거리·주점. 역투영에 필요
    /tf_static                         카메라 안 좌표계 관계 (드라이버가 냄)

`pointcloud:=true` 면 드라이버가 역투영까지 해서 하나 더 냄.

    /huphy/d455/depth/color/points     sensor_msgs/PointCloud2

`color:=true`, `imu:=true` 를 켜면 각각 `color/*`, `imu` 가 더 생김.

## 아직 안 정한 것

이 뒤에 붙을 지형 처리 노드는 아직 없음. 정책이 지형을 어떤 모양으로 받는지가
정해져야 만들 수 있음 (README "지형 -- 아직 정하는 중"). 지금 이 런치 파일은
**카메라가 실제로 무엇을 얼마나 빨리 내는지 재 보려는 것**까지가 목적임.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

ARGUMENTS = [
    # 토픽이 /huphy/d455/... 로 나오게 함. 기본값(camera/camera)을 그대로 두면
    # /camera/camera/... 가 되어 우리 토픽과 따로 놂.
    ("namespace", "huphy", "토픽 이름공간. /<namespace>/<name>/... 으로 나옴"),
    ("name", "d455", "카메라 이름. 여러 대를 달면 서로 달라야 함"),
    ("serial", "", "카메라를 일련번호로 고름. 여러 대일 때만 씀"),

    # "848x480x30" 또는 "848,480,30" 둘 다 됨 (드라이버가 [xX,] 로 끊음 --
    # profile_manager.cpp 의 self_regex). D455 깊이 센서의 기본 해상도가
    # 848x480 이라 그대로 씀. 지원 안 하는 조합을 주면 드라이버가 경고하고
    # 기본값으로 돌아감 -- 목록은 `ros2 param describe` 로 볼 수 있음.
    ("depth_profile", "848x480x30", "깊이 스트림 해상도x해상도x프레임률"),

    # 깊이만 쓰면 색은 USB 대역폭만 먹음. 눈으로 확인할 때만 켬.
    ("color", "false", "컬러 스트림. true 면 color/image_raw 가 생김"),
    ("color_profile", "848x480x30", "컬러 스트림 해상도x해상도x프레임률"),

    # 드라이버가 깊이 영상을 3차원 점으로 바꿔 줌 (C++ 쪽에서 함). 켜면 우리가
    # 역투영을 안 해도 되지만 토픽 하나당 바이트가 늘어남. 어느 쪽이 나은지는
    # 실제로 재 보고 정함.
    ("pointcloud", "false", "깊이를 PointCloud2 로도 냄"),

    # D455 에는 IMU 가 들어 있음 (D435 에는 없음). 로봇 IMU 와 별개임 --
    # 쓸지 안 쓸지 안 정했으므로 기본은 끔.
    ("imu", "false", "카메라 내장 자이로·가속도계"),
    ("unite_imu_method", "2", "자이로·가속도계를 imu 토픽 하나로 합치는 법 [0-안 함, 1-복사, 2-선형보간]"),

    # 해상도를 드라이버 쪽에서 미리 줄임. magnitude 2 면 가로세로 반 -> 점 수 1/4.
    # 우리가 받은 뒤 줄이는 것보다 싸지만, 켜면 CameraInfo 도 같이 바뀜.
    ("decimation", "false", "드라이버에서 깊이 영상을 미리 줄임"),
    ("decimation_magnitude", "2", "줄이는 배수. 2 면 가로세로 각각 1/2"),

    # 카메라가 이상한 상태로 남아 있을 때(앞 실행이 비정상 종료) 열리지 않는
    # 일이 있음. 켜면 열기 전에 장치를 한 번 리셋함. 시작이 몇 초 느려짐.
    ("initial_reset", "false", "열기 전에 장치를 리셋"),
    ("log_level", "info", "드라이버 로그 수준 [DEBUG|INFO|WARN|ERROR|FATAL]"),
]


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=default, description=note)
         for name, default, note in ARGUMENTS]
        + [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution(
                    [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"])),
                # 왼쪽은 rs_launch.py 가 선언한 인자 이름임 (고칠 수 없음).
                # 오른쪽이 위에서 우리가 받은 값.
                launch_arguments=[
                    ("camera_namespace", LaunchConfiguration("namespace")),
                    ("camera_name", LaunchConfiguration("name")),
                    ("serial_no", LaunchConfiguration("serial")),

                    ("enable_depth", "true"),
                    ("depth_module.depth_profile", LaunchConfiguration("depth_profile")),

                    ("enable_color", LaunchConfiguration("color")),
                    ("rgb_camera.color_profile", LaunchConfiguration("color_profile")),

                    ("pointcloud.enable", LaunchConfiguration("pointcloud")),

                    ("enable_gyro", LaunchConfiguration("imu")),
                    ("enable_accel", LaunchConfiguration("imu")),
                    ("unite_imu_method", LaunchConfiguration("unite_imu_method")),

                    ("decimation_filter.enable", LaunchConfiguration("decimation")),
                    ("decimation_filter.filter_magnitude",
                     LaunchConfiguration("decimation_magnitude")),

                    ("initial_reset", LaunchConfiguration("initial_reset")),
                    ("log_level", LaunchConfiguration("log_level")),
                    ("output", "screen"),
                ],
            ),
        ])
