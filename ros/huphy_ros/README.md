# `huphy_ros` — HUPHY 제어의 ROS 2 래퍼

계약과 근거는 [`docs/ros.md`](../../docs/ros.md). 여기는 **쓰는 법**만 적음.

```
huphy_ros/
├── convert.py       단위·이름 변환.      rclpy 를 import 하지 않음
├── freshness.py     받은 값의 나이 판정.  rclpy 를 import 하지 않음
├── control_node.py  모션보드에서. 모터를 몲
└── policy_node.py   작업 기계에서. 어디로 갈지 정함
```

`src/huphy/` 를 **고치지 않음.** import 만 함.

---

## 기계 둘

| | 거기서 띄우는 것 | 붙은 것 |
|---|---|---|
| 모션보드 | `control.launch.py` | CAN 어댑터, IMU |
| 작업 기계 | `policy.launch.py` | 사람. (나중) 카메라·라이다 |

**같은 레포를 양쪽에 두고 런치만 다르게 띄움.** 코드에 상대 기계의 주소를 안 적음.

---

## 설치 (양쪽 기계에서)

ROS 환경을 먼저 불러온 뒤 `huphy` 를 같은 환경에 넣음. **순서가 중요함** —
`pip install` 이 어느 파이썬에 설치되는지가 환경에 달려 있음.

```bash
source /opt/ros/humble/setup.bash
cd <레포 경로>
pip install -e .

mkdir -p ~/ros2_ws/src
ln -s <레포 경로> ~/ros2_ws/src/huphy
cd ~/ros2_ws && colcon build --symlink-install
source install/setup.bash
```

`colcon` 은 `src/` 아래를 재귀로 뒤져 `package.xml` 이 있는 폴더만 패키지로 봄.
레포 루트에는 없으므로 `ros/huphy_ros` 하나만 빌드됨.

확인:

```bash
ros2 pkg list | grep huphy
python3 -c "import huphy, huphy_ros; print(huphy.__file__)"
```

### 두 기계가 서로 보이게

```bash
export ROS_DOMAIN_ID=7      # 양쪽 같은 값으로. .bashrc 에
```

**안 맞으면 에러 없이 서로 안 보임.** 확인하는 법은 아래 "안 보일 때".

---

## 돌리는 순서

### 1. 모션보드 — CAN 올리고 제어 노드

```bash
huphy-can up
ros2 launch huphy_ros control.launch.py config:=$PWD/config/robot.yaml
```

**토크는 안 들어감.** `OBSERVE` 로 시작함.

### 2. 어디서든 — 읽히는지 확인

```bash
ros2 topic echo /huphy/joint_states --once
ros2 topic hz /huphy/joint_states
ros2 topic echo /huphy/gravity --once
ros2 topic echo /huphy/loop_status --once
```

| 보는 것 | 맞는 값 |
|---|---|
| 관절 이름 | `right_leg__knee` 꼴. `/` 가 없어야 함 |
| 위치 단위 | **라디안.** 무릎 20도면 약 0.35 |
| 주기 | `hz` 가 100 근처 |
| 중력방향 | 서 있으면 z 가 −1 근처 |

**여기서 전부 확인한 뒤에 다음으로.**

### 3. 작업 기계 — 정책 노드

```bash
ros2 launch huphy_ros policy.launch.py weights:=$PWD/config/policies/generalized_walk.pt
```

띄우기만 하면 **아무것도 안 냄.** `/huphy/policy_status` 의 `stage` 가 `off` 임.

### 4. 토크

```bash
ros2 service call /huphy/enable std_srvs/srv/Trigger
```

### 5. 정책 시작

```bash
ros2 service call /huphy/start_policy std_srvs/srv/Trigger
```

`approach_s`(기본 3초) 동안 기준 자세로 옮긴 뒤 신경망으로 바뀜. `stage` 가
`off → approach → running` 으로 감.

### 6. 속도 명령

```bash
ros2 topic pub -r 20 /huphy/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.2}, angular: {z: 0.0}}'
```

`linear.x`(vx) 와 `angular.z`(wz) 만 씀. `linear.y` 는 받아도 버림 — 정책 관찰에
`vy` 칸이 없음.

안 보내면 `(0, 0)` 이고 그게 제자리임.

### 멈출 때

```bash
ros2 service call /huphy/stop_policy std_srvs/srv/Trigger   # 목표각을 안 냄
ros2 service call /huphy/disable std_srvs/srv/Trigger       # 감쇠 후 토크 차단
```

`disable` 은 감쇠에 5초가 걸림 (`control/loop.py` 의 `DAMPING_S`).

---

## 토픽과 서비스

| 이름 | 타입 | 방향 |
|---|---|---|
| `/huphy/joint_states` | `sensor_msgs/JointState` | 모션 → 작업 |
| `/huphy/imu` | `sensor_msgs/Imu` | 모션 → 작업 |
| `/huphy/gravity` | `geometry_msgs/Vector3Stamped` | 모션 → 작업 |
| `/huphy/loop_status` | `diagnostic_msgs/DiagnosticArray` | 모션 → 작업 |
| `/huphy/joint_targets` | `sensor_msgs/JointState` | 작업 → 모션 |
| `/huphy/policy_status` | `diagnostic_msgs/DiagnosticArray` | 작업 기계 안 |
| `/huphy/cmd_vel` | `geometry_msgs/Twist` | 작업 기계 안 |
| `/huphy/enable` · `/disable` | `std_srvs/Trigger` | 작업 → 모션 |
| `/huphy/start_policy` · `/stop_policy` | `std_srvs/Trigger` | 작업 기계 안 |

관찰과 목표각은 `SensorDataQoS`(best effort) 임. **받는 쪽도 같은 QoS 여야 연결됨.**

---

## 끊기면

| | |
|---|---|
| 관절 상태가 **60ms** 안 옴 | `policy_node` 가 목표각을 안 냄 |
| 목표각이 **100ms** 안 옴 | `control_node` 가 감쇠 후 토크 차단 |
| 모터가 **5주기** 연속 무응답 | `LinkWatch` 가 루프 정지 (`robot.yaml` 의 `safety.link_loss_cycles`) |

`100 > 60` 이라 정책이 먼저 멈추고 그 뒤에 제어가 멈춤.

`kill -9` 는 **감쇠·토크차단 절차를 못 탐.** 토크가 켜진 채 남음.

---

## 파라미터

### `control_node`

| 이름 | 기본 | 무엇 |
|---|---|---|
| `config` | 없음. **필수** | `robot.yaml` 경로 |
| `limb` | 빈 값 = 양다리 | 다리 하나만 돌릴 때 |
| `hz` | 100.0 | 제어 주기 |
| `state_hz` | 100.0 | 상태 발행 주기 |
| `drain_ms` | 2.0 | 응답을 기다리는 최대 시간 |
| `target_stale_ms` | 100.0 | 목표각이 이보다 안 오면 감쇠 |
| `allow_uncalibrated` | false | 실측 전에도 토크를 넣음 |

### `policy_node`

| 이름 | 기본 | 무엇 |
|---|---|---|
| `weights` | 없음. **필수** | `.pt` 경로 |
| `hz` | 50.0 | 추론 주기 |
| `state_stale_ms` | 60.0 | 상태가 이보다 낡으면 목표각을 안 냄 |
| `approach_s` | 3.0 | 기준 자세로 옮기는 시간 |

**게인·관절 한계·모터 id 는 양쪽 다 없음.** `robot.yaml` 과
`config/calibration/*.json` 이 유일한 출처임.

---

## 테스트

`convert.py` 와 `freshness.py` 는 ROS 없이 돎.

```bash
cd <레포 경로>
PYTHONPATH=src python3 -m pytest tests/test_ros_convert.py tests/test_ros_freshness.py -q
```

`tests/conftest.py` 가 `ros/huphy_ros` 를 경로에 넣으므로 `PYTHONPATH` 에 따로 안
적어도 됨. 노드 둘은 `rclpy` 가 있어야 import 되어 여기서 안 봄.

---

## 안 보일 때

```bash
echo $ROS_DOMAIN_ID          # 양쪽 같아야 함
echo $ROS_LOCALHOST_ONLY     # 비어 있거나 0
echo $RMW_IMPLEMENTATION     # 양쪽 같아야 함
ros2 node list               # 상대 노드가 보여야 함
ros2 topic info -v /huphy/joint_states    # QoS 가 양쪽 맞는지
```

| 증상 | 흔한 원인 |
|---|---|
| `ros2 node list` 에 상대가 없음 | `ROS_DOMAIN_ID` 가 다름 / 멀티캐스트가 막힘 |
| 토픽은 보이는데 값이 안 옴 | QoS 가 다름 |
| `joint_states` 가 안 뜸 | `control_node` 가 CAN 을 못 열었음. 로그를 볼 것 |

---

## 안 되는 것

| | |
|---|---|
| `control.launch.py` 를 두 기계에서 | 같은 CAN 채널을 둘이 열어 응답이 섞임 |
| 정책과 `pose` 를 동시에 | `joint_targets` 토픽 하나를 같이 씀. 번갈아 들어감 |
| 정책 교체 | 없음. `.pt` 를 바꾸려면 노드 재시작 |
