# `huphy_ros` — HUPHY 제어의 ROS 2 래퍼

설계 근거는 [`docs/ros.md`](../../docs/ros.md). 여기는 **쓰는 법**만 적음.

```
huphy_ros/
├── convert.py         단위·이름 변환. rclpy 를 import 하지 않음
├── bridge.py          토픽 <-> Motion. rclpy 를 import 하지 않음
├── control_node.py    제어 루프를 자기 스레드에서 돌림
└── telemetry_node.py  읽기만. 토크 안 들어감
```

`huphy` 를 **고치지 않음.** import 만 함.

---

## 설치

ROS 환경을 먼저 불러온 뒤 `huphy` 를 같은 환경에 넣어야 함. 순서가 중요함 --
`pip install` 이 어느 파이썬에 설치되는지가 환경에 달려 있음.

```bash
source /opt/ros/humble/setup.bash
cd ~/Desktop/chanhuegit/HUPHY
pip install -e .
```

워크스페이스에 링크하고 빌드함.

```bash
mkdir -p ~/ros2_ws/src
ln -s ~/Desktop/chanhuegit/HUPHY ~/ros2_ws/src/huphy
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

`colcon` 은 `src/` 아래를 재귀로 뒤져 `package.xml` 이 있는 폴더만 패키지로 봄.
레포 루트에는 `package.xml` 이 없으므로 `ros/huphy_ros` 하나만 빌드됨.

확인:

```bash
ros2 pkg list | grep huphy
python3 -c "import huphy, huphy_ros; print(huphy.__file__)"
```

---

## 돌리는 순서

### 1. CAN 채널을 올림

```bash
huphy-can up
```

### 2. 읽기만 — 토크 안 들어감

```bash
ros2 launch huphy_ros observe.launch.py
```

다른 터미널에서 (`source install/setup.bash` 먼저):

```bash
ros2 topic list
ros2 topic echo /huphy_telemetry/joint_states --once
ros2 topic hz /huphy_telemetry/joint_states
ros2 topic echo /huphy_telemetry/imu/main/gravity --once
```

**여기서 전부 확인한 뒤에 다음으로 감.**

| 보는 것 | 맞는 값 |
|---|---|
| 관절 이름 | `right_leg__knee` 꼴. `/` 가 없어야 함 |
| 위치 단위 | **라디안.** 무릎이 20도면 약 0.35 |
| 주기 | `ros2 topic hz` 가 50 근처 |
| 중력방향 | 로봇이 서 있으면 z 가 −1 근처 |

### 3. 제어 노드 — 아직 관찰 모드

```bash
ros2 launch huphy_ros control.launch.py
```

`observe_only` 기본이 `true` 라 토크가 안 들어감. 토픽이 2번과 같은지 확인함.

### 4. 토크를 넣음

**명시해야 함.**

```bash
ros2 launch huphy_ros control.launch.py observe_only:=false
```

```bash
ros2 service call /huphy_control/enable std_srvs/srv/Trigger
```

### 5. 명령을 보냄

```bash
ros2 topic pub -r 50 /huphy_control/command sensor_msgs/msg/JointState \
  '{name: ["right_leg__knee"], position: [0.35]}'
```

**계속 보내야 함.** 0.2초 넘게 안 오면 전송이 멈춤 (`bridge.DEFAULT_STALE_S`).
`-r 50` 을 빼고 한 번만 보내면 한 번 움직이고 멈춤.

끊을 때:

```bash
ros2 service call /huphy_control/disable std_srvs/srv/Trigger
```

---

## 토픽과 서비스

| 이름 | 타입 | 방향 |
|---|---|---|
| `/huphy_control/joint_states` | `sensor_msgs/JointState` | 나감 |
| `/huphy_control/command` | `sensor_msgs/JointState` | 들어옴. `name` 과 `position` 만 씀 |
| `/huphy_control/enable` | `std_srvs/Trigger` | 토크 넣기 |
| `/huphy_control/disable` | `std_srvs/Trigger` | 토크 끊기 |
| `/huphy_telemetry/joint_states` | `sensor_msgs/JointState` | 나감 |
| `/huphy_telemetry/imu/<이름>` | `sensor_msgs/Imu` | 각속도·가속도 |
| `/huphy_telemetry/imu/<이름>/gravity` | `std_msgs/Float32MultiArray` | 중력방향 3칸 |

관찰 토픽은 `SensorDataQoS`(best effort) 임. **받는 쪽도 같은 QoS 여야 연결됨** --
안 맞으면 에러 없이 조용히 아무것도 안 옴. `ros2 topic info -v <토픽>` 으로 확인함.

---

## 파라미터

| 이름 | 기본 | 무엇 |
|---|---|---|
| `config` | 없음. **필수** | `robot.yaml` 경로 |
| `limb` | 빈 값 = 양다리 | 다리 하나만 돌릴 때 그 이름 |
| `hz` | 100.0 | 제어 주기 |
| `drain_ms` | 2.0 | 응답을 기다리는 최대 시간 |
| `observe_only` | `true` | `false` 여야 토크가 들어감 |
| `allow_uncalibrated` | `false` | 실측 전에도 토크를 넣음 |
| `stale_s` | 0.2 | 명령이 이보다 오래되면 전송을 멈춤 |

**게인·관절 한계·모터 id 는 여기 없음.** `robot.yaml` 과
`config/calibration/*.json` 이 유일한 출처임. 파라미터로 또 받으면 어긋났을 때 어느
쪽이 진짜인지 알 수 없음 ([`docs/ros.md`](../../docs/ros.md) 9장).

---

## 테스트

`convert.py` 와 `bridge.py` 는 ROS 없이 돎.

```bash
cd ~/Desktop/chanhuegit/HUPHY
PYTHONPATH=src python3 -m pytest tests/test_ros_bridge.py -q
```

`tests/conftest.py` 가 `ros/huphy_ros` 를 경로에 넣으므로 `PYTHONPATH` 에 따로 안
적어도 됨. 노드 둘은 `rclpy` 가 있어야 import 되어 여기서 안 봄.

---

## 안 되는 것

| | |
|---|---|
| `observe.launch.py` 와 `control.launch.py` 를 같이 | 둘 다 같은 CAN 채널을 열어 응답이 섞임 |
| 정책을 노드 안에서 돌리기 | 아직 안 넣었음. `huphy-run --policy stand` 를 쓸 것 |
| `kill -9` 로 끄기 | 감쇠·토크 차단 절차를 못 탐. 토크가 켜진 채 남음 |
