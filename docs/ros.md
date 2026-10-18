# ROS 2 Humble 이식 — 처음 쓰는 사람을 위한 정리

**아직 안 만든 것의 설계**임. ROS 를 써 본 적이 없다는 전제로 개념부터 적고, 그
개념이 **지금 이 레포의 무엇에 해당하는지** 짝을 지어 둠.

관련 문서: 계층은 [architecture.md](architecture.md), 한 주기의 흐름은
[cycle.md](cycle.md), 양다리 구성은 [full_robot.md](full_robot.md).

---

## 0. 결론부터

| | |
|---|---|
| 레포를 나누나 | **안 나눔.** 이 레포 안에 `ros/huphy_ros/` 폴더 하나를 더 둠 |
| `src/huphy/` 를 고치나 | **안 고침.** ROS 는 `huphy` 를 import 만 함 |
| 지금 명령어(`huphy-run` 등) | 그대로 남음. ROS 없이도 돎 |
| 지금 테스트 | 그대로 돎. ROS 설치 없이 |
| 새로 만드는 것 | 노드 2개, 메시지 몇 개, 런치 파일 1개 |

`ControlLoop` 는 ROS 안으로 **안 들어감.** 자기 스레드에서 지금처럼 돌고, ROS 와는
큐로만 주고받음. 이유는 7장.

---

## 1. ROS 가 무엇인가 — 지금 코드에 짝지어서

ROS 2 는 로봇 소프트웨어를 **여러 프로세스로 나누고 그 사이를 잇는 규약과 도구
모음**임. 프레임워크라기보다 "프로세스 간 배선 + 공통 자료형 + 진단 도구" 에 가까움.

핵심 개념이 여섯 개임.

### 1-1. 노드 (Node)

프로세스 하나 안에서 도는 작업 단위임. 지금으로 치면 `huphy-run` 같은 진입점
하나가 노드 하나가 됨.

```python
import rclpy
from rclpy.node import Node

class ControlNode(Node):
    def __init__(self):
        super().__init__("huphy_control")      # 이 이름으로 네트워크에 보임
```

### 1-2. 토픽 (Topic) — 계속 흐르는 값

이름이 붙은 단방향 방송임. **받는 쪽이 없어도 보냄.** 주기적으로 흐르는 값에 씀.

지금 코드의 짝은 **텔레메트리**임.

| 지금 | ROS |
|---|---|
| `telemetry/udp.py` 가 UDP 로 쏨 | 퍼블리셔가 토픽으로 보냄 |
| `telemetry/csv.py` 가 파일에 씀 | `ros2 bag record` 가 받아 적음 |
| 그래프 도구로 UDP 를 받아 그림 | `rqt_plot` 이 토픽을 그림 |
| `snapshot.build()` 가 `Dict[str, float]` 하나로 정규화 | 그 사전이 곧 메시지 내용이 됨 |

**이미 하고 있는 일임.** 보내는 경로가 하나 늘 뿐임.

### 1-3. 서비스 (Service) — 한 번 묻고 한 번 답함

요청-응답 한 쌍임. 상태를 바꾸는 조작에 씀.

| 지금 | ROS |
|---|---|
| `huphy-run` 에서 Enter 를 눌러 정책 시작 | `/huphy/start` 서비스 호출 |
| Ctrl-C 로 멈춤 | `/huphy/stop` 서비스 |
| `robot.enable()` / `disable()` | `/huphy/enable` / `/huphy/disable` |

### 1-4. 액션 (Action) — 오래 걸리고 중간 보고가 있는 것

"3초에 걸쳐 기준 자세로 옮김" 같은 것임. 시작·중간 진행률·완료·취소가 있음.

지금의 `staged()` 접근 단계 ([run.py:197](../src/huphy/scripts/run.py#L197))가 여기
해당함. 다만 **처음부터 안 만들어도 됨** — 서비스로 시작만 시키고 진행 상황은
토픽으로 내보내도 충분함.

### 1-5. 파라미터 (Parameter)

노드마다 가지는 설정값임. 실행 중에 바꿀 수도 있음.

**여기를 조심해야 함.** 지금 설정은 `robot.yaml` 하나가 진짜임 — 로더가 모르는
키를 거부하고 ([loader.py:60](../src/huphy/config/loader.py#L60)), 실측값과
배선값을 파일로 나눠 둠. ROS 파라미터로 게인이나 한계를 **또** 받으면 같은 사실이
두 군데가 되고, 어긋났을 때 어느 쪽이 진짜인지 알 수 없음.

ROS 파라미터는 **경로와 실행 옵션만** 받는 것으로 둠 (9장).

### 1-6. 메시지 (Message) — 주고받는 자료형

`.msg` 파일로 정의하고 빌드 때 파이썬 클래스가 생성됨. 표준 타입이 많아서 웬만하면
그걸 씀 (`sensor_msgs/JointState`, `sensor_msgs/Imu`).

---

## 2. 넣어서 얻는 것 / 안 넣어도 되는 것

정직하게 적어 둠. ROS 가 공짜가 아님.

### 얻는 것

| | 지금은 |
|---|---|
| 기록·재생 | `ros2 bag` 이 모든 토픽을 타임스탬프와 함께 저장하고 그대로 재생함. 지금은 CSV 를 직접 파싱해야 함 |
| 그래프 | `rqt_plot`, `PlotJuggler` 가 바로 붙음. 지금은 UDP 수신기를 따로 돌림 |
| 다른 프로그램과 연결 | 보행 계획, 조이스틱, 카메라를 **다른 프로세스**로 붙임. 지금은 같은 파이썬 프로세스 안이어야 함 |
| 원격 | 다른 기계에서 토픽을 보고 서비스를 부름. 지금은 UDP 단방향뿐 |
| 표준 도구 | `ros2 topic hz` 로 주기 확인, `ros2 topic echo` 로 값 확인 |

### 안 얻는 것 — 오해하기 쉬움

| | |
|---|---|
| 실시간성 | ROS 2 파이썬은 실시간이 **아님.** 지금 `ControlLoop` 가 지키는 100Hz 를 ROS 가 대신 지켜 주지 않음. 오히려 지터가 얹힘 |
| 안전 | `LinkWatch`, `guards.py` 는 그대로 우리 것임. ROS 가 대신 막아 주지 않음 |
| 설정 관리 | ROS 파라미터가 `robot.yaml` 보다 낫지 않음. 오히려 모르는 키 거부 같은 것이 없음 |
| CAN | `python-can` 그대로 씀. `ros2_canopen` 같은 것은 CANopen 용이라 MIT 프로토콜과 무관함 |

**한 프로세스로 끝나는 일에는 ROS 를 넣을 이유가 없음.** 지금 `huphy-run` 이 그럼.
ROS 가 필요해지는 시점은 **다른 프로그램을 붙일 때**임 — 보행 계획기, 조이스틱,
상태 기계, 비전.

---

## 3. Humble 준비

### 3-1. 전제

| | |
|---|---|
| OS | Ubuntu 22.04 |
| 파이썬 | 3.10 |
| 지원 기한 | 2027년 5월 |

이 레포 코드는 3.9 문법 범위 안에 있어서 (`match` 없음) 그대로 돎.

### 3-2. 설치

```bash
sudo apt install ros-humble-ros-base        # 로봇 쪽. GUI 없음
sudo apt install ros-humble-desktop         # 개발 기계. rqt·rviz 포함
sudo apt install python3-colcon-common-extensions
```

쓸 때마다 환경을 불러와야 함. 안 하면 `ros2` 명령 자체가 없음.

```bash
source /opt/ros/humble/setup.bash
```

`~/.bashrc` 에 넣어 두는 것이 보통임.

### 3-3. 워크스페이스

ROS 는 **빌드한 결과를 따로 둠.** 소스 폴더와 설치 폴더가 나뉨.

```
~/ros2_ws/
├── src/        <- 소스. 여기에 레포를 링크함
├── build/      <- 빌드 중간 산물. 지워도 됨
├── install/    <- 여기를 source 함
└── log/
```

```bash
mkdir -p ~/ros2_ws/src
ln -s ~/Desktop/chanhuegit/HUPHY ~/ros2_ws/src/huphy
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

`colcon` 은 `src/` 아래를 **재귀로 뒤져 `package.xml` 이 있는 폴더**를 패키지로
봄. 그래서 레포 안 하위 폴더에 둬도 됨.

`--symlink-install` 은 파이썬 파일을 복사하지 않고 링크함 — 고칠 때마다 다시 빌드
안 해도 됨.

---

## 4. 패키지 구조

```
HUPHY/
├── src/huphy/              지금 그대로. 순수 pip 패키지. ROS 를 모름
├── pyproject.toml
├── tests/
├── config/
├── docs/
└── ros/
    └── huphy_ros/          <- 새로 만드는 것
        ├── package.xml
        ├── setup.py
        ├── setup.cfg
        ├── resource/huphy_ros
        ├── msg/
        │   └── LoopStatus.msg
        ├── launch/
        │   └── stand.launch.py
        └── huphy_ros/
            ├── __init__.py
            ├── control_node.py
            ├── telemetry_node.py
            └── convert.py
```

### 4-1. `huphy` 를 ROS 패키지로 만들지 않는 이유

`src/huphy/` 에 `package.xml` 을 붙여 하나로 합칠 수도 있는데, 둘이 걸림.

**빌드 경로가 둘이 됨.** `ament_python` 은 `setup.py` 를 보는데 지금은
`pyproject.toml` 만 있음. 둘을 같이 두면 `pip install -e .` 과 `colcon build` 가
서로 다른 경로로 설치해서, 어느 쪽이 import 되는지가 환경에 따라 달라짐.

**ROS 없이 못 돌게 됨.** 지금 `huphy-test`/`huphy-run` 과 `tests/` 전체가 ROS 가
없는 기계에서 돎. `package.xml` 을 붙이면 그 전제가 흐려짐.

그래서 `huphy` 는 pip 로 넣고 `huphy_ros` 만 ROS 패키지로 둠.

```bash
cd ~/Desktop/chanhuegit/HUPHY
pip install -e .            # ROS 환경을 source 한 상태에서
```

### 4-2. 만드는 법

```bash
cd ~/Desktop/chanhuegit/HUPHY/ros
ros2 pkg create --build-type ament_python huphy_ros
```

네 파일이 생김.

| 파일 | 무엇 |
|---|---|
| `package.xml` | 패키지 이름, 의존, 라이선스. colcon 이 이것을 보고 패키지로 인식함 |
| `setup.py` | 파이썬 설치 설정. 노드 진입점을 여기 적음 |
| `setup.cfg` | 스크립트가 설치될 자리 |
| `resource/huphy_ros` | **빈 파일.** ament 가 패키지 목록을 만들 때 씀. 지우면 안 됨 |

**`package.xml`**

```xml
<?xml version="1.0"?>
<package format="3">
  <name>huphy_ros</name>
  <version>0.1.0</version>
  <description>HUPHY 제어를 ROS 2 노드로 감쌈</description>
  <maintainer email="hwagchan@gmail.com">chanhue</maintainer>
  <license>MIT</license>

  <depend>rclpy</depend>
  <depend>sensor_msgs</depend>
  <depend>std_msgs</depend>
  <depend>std_srvs</depend>

  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

`huphy` 자체는 **여기 적을 수 없음.** `rosdep` 은 apt 패키지와 몇몇 pip 패키지만
아는데 `huphy` 는 둘 다 아님. 그냥 `pip install -e .` 를 전제로 두고 README 에
적는 것이 현실적임.

**`setup.py` 의 진입점**

```python
entry_points={
    "console_scripts": [
        "control_node = huphy_ros.control_node:main",
        "telemetry_node = huphy_ros.telemetry_node:main",
    ],
},
```

이러면 `ros2 run huphy_ros control_node` 로 돎. 지금 `pyproject.toml` 의
`[project.scripts]` 와 같은 구조임.

---

## 5. 어디서 갈라붙나 — 경계 두 개

ROS 를 붙일 자리가 **이미 두 군데 있음.** 새로 만드는 것이 아니라 그 자리에
꽂으면 됨.

### 5-1. 들어오는 쪽 — `Motion`

[loop.py:157](../src/huphy/control/loop.py#L157) 의 계약임.

```python
Motion = Callable[[float, Dict[str, Any]], Optional[Action]]
#                  시간    관찰              관절 이름 -> 목표각(도)
```

`ControlLoop` 는 이 함수가 어디서 왔는지 모름. 지금 세 종류가 같은 자리에 들어감.

```
motions.hold({"knee": 0.0})            정해진 자세
policy.policy_motion(model, imu, ...)  0.5 모델
stand.motion(model, imu)               1.0 서기 모델
```

**ROS 노드는 네 번째를 하나 더 만드는 것뿐임.**

```python
def ros_motion(t, observation):
    with lock:
        return dict(latest_command) if latest_command else None
```

`None` 을 내면 그 주기는 아무것도 안 보냄 ([loop.py:308](../src/huphy/control/loop.py#L308)).
명령이 아직 안 왔을 때 쓸 수 있음.

### 5-2. 나가는 쪽 — `snapshot`

[snapshot.py](../src/huphy/telemetry/snapshot.py) 가 로봇 상태를 `Dict[str, float]`
하나로 정규화함. **필드 이름을 정하는 유일한 곳**임.

```python
snapshot.build_fast(robot, t=t, loop_dt_ms=dt)   # 매 주기. 위치·목표·오차
snapshot.build_diag(robot, t=t)                  # 가끔. 온도·카운터
snapshot.build_imu(robot, t=t)                   # IMU
```

UDP 와 CSV 가 이 사전을 소비함. 퍼블리셔가 **세 번째 소비자**로 붙음.

---

## 6. 노드 두 개

### 6-1. `control_node`

```
들어옴   /huphy/command          sensor_msgs/JointState   목표각
         /huphy/enable           std_srvs/Trigger         토크 넣기
         /huphy/disable          std_srvs/Trigger         토크 끊기
나감     /huphy/joint_states     sensor_msgs/JointState   실측 위치·속도·토크
         /huphy/status           huphy_ros/LoopStatus     주기, 밀림, 무응답
```

하는 일이 적음.

```python
robot = build_biped(config, limbs, **options)       # scripts/bringup.py 그대로
loop = ControlLoop(robot, hz=100.0, mode=Mode.CONTROL)
thread = threading.Thread(target=loop.run, args=(ros_motion,), daemon=True)
thread.start()
rclpy.spin(node)
```

`build_biped` 와 `ControlLoop` 는 **지금 `run.py` 가 쓰는 것 그대로**임. 노드가
새로 하는 일은 토픽을 `Motion` 으로 바꾸는 것뿐임.

### 6-2. `telemetry_node`

따로 둘 수도 있고 `control_node` 안에 둘 수도 있음. 따로 두면 프로세스가 하나 더
늘고, 로봇 상태를 읽으려면 공유가 필요해서 **처음에는 한 노드 안에 두는 것**이
간단함.

### 6-3. 정책은 어디에

두 가지 방식이 있음.

| | 정책이 도는 곳 | 좋은 점 | 나쁜 점 |
|---|---|---|---|
| A | `control_node` 안 | 지금과 같음. 지연 없음 | 정책을 바꾸려면 노드를 재시작 |
| B | 별도 `policy_node` | 정책만 따로 갈아끼움. 다른 언어도 가능 | 토픽 왕복 지연이 제어 주기에 들어감 |

**A 로 시작할 것.** 서기 정책은 50Hz 이고 `stand.motion` 이 이력을 들고 있어서
([stand.py:252](../src/huphy/control/stand.py#L252)), 토픽으로 나눴다가 한 주기라도
밀리면 `q(t-1)` 이 학습 때와 다른 시점이 됨.

B 가 필요해지는 것은 보행 계획처럼 **느려도 되는 것**이 붙을 때임. 그때는
`/huphy/command` 로 목표를 보내고 `control_node` 가 그것을 지키는 구조가 맞음.

---

## 7. 주기와 스레드 — 가장 중요한 부분

### 7-1. `ControlLoop` 를 ROS 타이머에 얹으면 안 됨

이렇게 쓰고 싶어짐.

```python
self.create_timer(0.01, self.one_cycle)      # ← 이렇게 하면 안 됨
```

`ControlLoop` 는 주기를 **자기가 지킴**.

| | 어디 |
|---|---|
| 마감을 절대 시각으로 들고 감 | [loop.py:500](../src/huphy/control/loop.py#L500) `_sleep_until` |
| 마감 3ms 전부터는 자지 않고 돌며 기다림 | `SPIN_THRESHOLD_S` [loop.py:96](../src/huphy/control/loop.py#L96) |
| 밀렸으면 따라잡지 않음 | 한 주기 늦게 시작하고 맒 |
| 밀림을 셈 | `OVERRUN_RATIO` [loop.py:89](../src/huphy/control/loop.py#L89) |

`rclpy` 타이머는 executor 가 콜백을 꺼내 부르는 구조라 그 위에 **지터가 얹힘.**
스핀 구간으로 0.1ms 까지 맞춰 둔 것이 무의미해짐.

### 7-2. 구조

```
메인 스레드          rclpy.spin(node)        토픽·서비스 콜백
제어 스레드          loop.run(ros_motion)    100Hz. CAN 송수신
수신 스레드 x2       CanBus._read_loop       can0 / can1 (이미 있음)
```

제어 스레드와 ROS 스레드가 공유하는 것은 **두 개뿐**이어야 함.

| | 방향 | 보호 |
|---|---|---|
| 최신 명령 | ROS → 제어 | 락 하나. 사전 통째로 교체 |
| 최신 관찰 | 제어 → ROS | 락 하나. 사전 통째로 교체 |

사전을 통째로 바꾸면 읽는 쪽이 **절반만 갱신된 상태**를 보지 않음. 항목마다 따로
쓰면 그 사이에 읽혀서 일부는 이번 주기, 일부는 지난 주기가 섞임.

### 7-3. 퍼블리시를 제어 스레드에서 하지 말 것

`Motion` 안에서 `publisher.publish()` 를 부르면 그 시간이 제어 주기에 들어감.
DDS 가 몇 ms 를 쓸 수 있어서 100Hz(10ms) 예산을 넘길 수 있음.

제어 스레드는 사전에 **써 두기만** 하고, ROS 타이머가 그것을 꺼내 보냄.

```python
# 제어 스레드
def ros_motion(t, observation):
    with self.lock:
        self.latest_obs = observation          # 쓰기만
        command = dict(self.latest_cmd) if self.latest_cmd else None
    return command

# ROS 타이머 (50Hz 정도면 충분)
def publish(self):
    with self.lock:
        obs = self.latest_obs
    if obs:
        self.pub.publish(to_joint_state(obs))
```

### 7-4. Executor

기본 `rclpy.spin()` 은 콜백을 **한 번에 하나씩** 부름. 느린 콜백 하나가 나머지를
막음. 서비스가 오래 걸리면 `MultiThreadedExecutor` 를 쓰되, 그러면 콜백끼리
동시에 돌 수 있어 락이 더 필요해짐.

**처음에는 단일 스레드로 두고**, 콜백 안에서 오래 걸리는 일을 안 하면 됨.

### 7-5. QoS

토픽마다 신뢰성 설정이 있음. 기본은 `RELIABLE` 로 **빠진 것을 다시 보냄.**
고주기 센서 값에는 안 맞음 — 다시 보낼 때쯤이면 이미 낡은 값임.

```python
from rclpy.qos import qos_profile_sensor_data     # BEST_EFFORT, depth 5

self.create_publisher(JointState, "/huphy/joint_states", qos_profile_sensor_data)
```

| 토픽 | QoS |
|---|---|
| `/huphy/joint_states` | `sensor_data` (best effort) |
| `/huphy/command` | `sensor_data` |
| `/huphy/status` | 기본 (reliable). 가끔 가고 놓치면 안 됨 |

**받는 쪽과 보내는 쪽 설정이 안 맞으면 조용히 연결이 안 됨.** ROS 에서 가장 흔한
"아무것도 안 오는데 에러도 없음" 의 원인임. `ros2 topic info -v <토픽>` 으로 확인함.

---

## 8. 메시지와 단위

### 8-1. 단위 — 도 ↔ 라디안

ROS 는 **SI 단위**를 씀 (REP 103). 각도는 라디안임.

이 레포는 **도**를 씀. 모델 경계에서만 라디안으로 바꿈
([cycle.md](cycle.md) 13장).

```
huphy 안쪽        도            robots/, safety/, telemetry/
모델 경계         라디안        control/policy.py, control/stand.py
ROS 경계          라디안        huphy_ros/convert.py     <- 새로 생김
```

**변환은 `convert.py` 한 곳에만 둘 것.** 지금 도↔라디안이 두 곳(`mit.py`,
`policy.py`)에 있고 서로 안 만나는 구조인데, 세 번째가 여기저기 흩어지면 어디서
한 번 더 변환됐는지 추적이 안 됨.

### 8-2. 관절 이름

양다리면 `right_leg/knee` 처럼 `/` 가 들어감
([biped.py:76](../src/huphy/robots/biped.py#L76)).

ROS 토픽 이름에서 `/` 는 네임스페이스 구분자지만, `JointState.name` 은 **그냥
문자열 배열**이라 `/` 가 들어가도 됨. 그래도 헷갈리므로 `right_leg__knee` 같이
바꿀지는 정해야 함. 바꾼다면 그 변환도 `convert.py` 에 둠.

### 8-3. 표준 메시지로 되는 것

**`sensor_msgs/JointState`** — 관찰과 명령 둘 다 이걸로 됨.

```
std_msgs/Header header       stamp, frame_id
string[] name                관절 이름
float64[] position           rad
float64[] velocity           rad/s
float64[] effort             Nm
```

`Leg.get_observation()` 이 내는 `{관절}.pos/.vel/.torque` 가 그대로 들어감
([leg.py:453](../src/huphy/robots/leg.py#L453)).

**`sensor_msgs/Imu`** — IMU.

```
geometry_msgs/Quaternion orientation
geometry_msgs/Vector3 angular_velocity        rad/s
geometry_msgs/Vector3 linear_acceleration     m/s^2
```

우리 `ImuState` 는 자세를 **중력방향 3칸**으로 들고 있음
([sensors/base.py:105](../src/huphy/sensors/base.py#L105)). `Imu` 메시지에는 그
칸이 없어서 쿼터니언으로 되돌리거나, 중력방향을 `Vector3` 로 따로 내보내야 함.
정책이 쓰는 것은 중력방향이므로 **따로 내는 쪽**이 맞음.

### 8-4. 직접 만들어야 하는 것

주기·실패 집계는 표준 메시지에 없음. `msg/LoopStatus.msg` 하나면 됨.

```
std_msgs/Header header
float32 mean_hz
float32 target_hz
uint32  cycles
uint32  overruns
uint32  missing_cycles
float32 worst_dt_ms
bool    kept_up
string[] link_loss_motors      비어 있으면 정상
```

`LoopStats` ([loop.py:203](../src/huphy/control/loop.py#L203))에 이미 다 있는 값임.

메시지를 만들려면 `package.xml` 에 `rosidl_default_generators` 가 있어야 하고
(4-2 에 넣어 뒀음), `CMakeLists.txt` 가 필요함 — **`ament_python` 패키지는 메시지를
만들 수 없음.** 둘 중 하나임.

| | |
|---|---|
| 메시지 전용 패키지를 하나 더 둠 | `huphy_msgs` (`ament_cmake`). ROS 에서 흔한 구조임 |
| 표준 메시지로 때움 | `diagnostic_msgs/DiagnosticArray` 에 key-value 로 넣음 |

처음에는 **두 번째로 시작하고**, 필요해지면 `huphy_msgs` 를 만드는 것이 빠름.

---

## 9. 파라미터

받는 것을 **경로와 실행 옵션으로 제한**함.

```yaml
huphy_control:
  ros__parameters:
    config: /home/chanhue/Desktop/chanhuegit/HUPHY/config/robot.yaml
    policy: stand                 # 없으면 정책 없이 명령만 받음
    weights: config/policies/stand_bi.pt
    ankle_space: ab
    hz: 100.0
    drain_ms: 2.0
    allow_uncalibrated: true
```

**여기 없어야 하는 것**: 게인, 관절 한계, 관절 이름, 모터 id, 발목 기하.

전부 `robot.yaml` 과 `calibration/*.json` 에 있고, 그 두 파일이 나뉜 기준이 "이
숫자를 어디서 얻었나" 임 ([config/README.md](../config/README.md)). 파라미터로
덮어쓸 수 있게 두면 그 기준이 깨짐.

실제로 그 실수를 한 적이 있음 — `huphy-run` 이 `robot.yaml` 의 게인을 코드 상수로
덮어써서 무릎 kp 가 220 대신 20 으로 나갔음 (커밋 `4dbdfa2`). 값이 두 군데 있으면
생기는 일임.

---

## 10. 런치

노드 여러 개를 한 번에 띄우고 파라미터를 먹이는 파이썬 스크립트임.

```python
# launch/stand.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="huphy_ros",
            executable="control_node",
            name="huphy_control",
            parameters=[{
                "config": "/home/chanhue/Desktop/chanhuegit/HUPHY/config/robot.yaml",
                "policy": "stand",
                "weights": "config/policies/stand_bi.pt",
                "ankle_space": "ab",
                "hz": 100.0,
            }],
            output="screen",
        ),
    ])
```

```bash
ros2 launch huphy_ros stand.launch.py
```

---

## 11. 안전 — ROS 가 늘리는 위험

지금 구조는 **한 프로세스가 죽으면 전부 멈춤**임. ROS 는 그걸 나눠서 새 실패
방식을 만듦.

| 새로 생기는 것 | 막는 법 |
|---|---|
| 명령을 보내는 노드가 죽음 | `control_node` 가 **마지막 명령 시각**을 보고, 일정 시간 안 오면 `None` 을 내 전송을 멈춤. `LinkWatch` 와 같은 발상 |
| 네트워크가 끊김 | 위와 같음 |
| 다른 노드가 엉뚱한 토픽에 명령을 씀 | 토픽 이름을 네임스페이스로 묶고, `guards.py` 가 어차피 한계를 봄 |
| 노드를 `kill -9` 로 죽임 | 토크가 켜진 채 남음. 지금 `finally` 의 감쇠 절차를 못 탐 |

마지막 것이 제일 위험함. `rclpy.spin()` 은 SIGINT 를 잡아 주지만 `SIGKILL` 은
못 잡음 — 지금 터미널에서 Ctrl-C 로 멈추는 것과 같은 수준까지만 보장됨.

**명령 타임아웃은 처음부터 넣을 것.**

```python
STALE_S = 0.2        # 이 시간 안 오면 전송을 멈춤

def ros_motion(t, observation):
    with self.lock:
        age = time.monotonic() - self.cmd_stamp
        if age > STALE_S:
            return None                # 전송 안 함
        return dict(self.latest_cmd)
```

---

## 12. 테스트

**지금 테스트는 건드리지 않음.** `tests/` 전체가 ROS 없이 돎.

`huphy_ros` 쪽은 노드가 얇아서 테스트할 것이 적음.

| 무엇 | 어떻게 |
|---|---|
| `convert.py` 의 도↔라디안, 이름 변환 | 순수 함수. `rclpy` 없이 테스트됨 |
| `ros_motion` 의 타임아웃 | 시각을 주입해서. `rclpy` 없이 |
| 노드가 실제로 뜨는지 | `launch_testing`. 느리고 환경을 탐 |

앞의 둘만 `tests/` 에 두고 `rclpy` import 를 피하면, ROS 없는 기계에서도 계속 돎.

---

## 13. 순서

| 단계 | 무엇 | 끝났다는 기준 |
|---|---|---|
| 1 | Humble 설치, 워크스페이스 | `ros2 topic list` 가 돎 |
| 2 | 빈 `huphy_ros` 패키지 | `ros2 run huphy_ros control_node` 가 "안녕" 을 찍음 |
| 3 | `telemetry_node` 먼저 | 로봇을 **읽기만** 함. `ros2 topic echo` 로 관절 각도가 보임. 모터가 안 움직이므로 안전함 |
| 4 | `convert.py` + 테스트 | 도↔라디안, 이름 변환이 테스트로 고정됨 |
| 5 | `control_node` — 관찰만 | `Mode.OBSERVE` 로 띄움. 토크 안 들어감 |
| 6 | 명령 받기 | `/huphy/command` 로 `huphy-test pose` 와 같은 일을 함 |
| 7 | 정책 | `policy` 파라미터로 `stand` 를 안에서 돌림 |
| 8 | 타임아웃·서비스 | 11장 |
| 9 | 런치 | 한 줄로 뜸 |

**3번을 먼저 하는 것이 중요함.** 읽기만 하는 노드로 ROS 쪽 배선(QoS, 단위, 이름,
주기)을 전부 확인한 뒤에 토크를 넣는 쪽으로 감.

---

## 14. 안 할 것

| | 왜 |
|---|---|
| `ControlLoop` 를 `rclpy` 타이머로 | 7-1 |
| `huphy` 를 ROS 패키지로 | 4-1 |
| 게인·한계를 ROS 파라미터로 | 9장 |
| 정책을 별도 노드로 (지금은) | 6-3 |
| `ros2_control` / `ros2_canopen` | 전자는 C++ 하드웨어 인터페이스 틀이라 지금 `Leg`/`Biped` 를 다시 짜야 하고, 후자는 CANopen 전용이라 MIT 프로토콜과 무관함 |
| `robot_state_publisher` / URDF (지금은) | rviz 로 보고 싶어질 때 함. 제어에는 안 쓰임 |

---

## 15. 한 장 요약

```
지금                                   ROS 를 붙인 뒤

huphy-run                              ros2 launch huphy_ros stand.launch.py
  └ build_biped                          └ control_node
     └ ControlLoop(100Hz)                   ├ 제어 스레드: 똑같은 ControlLoop(100Hz)
        └ stand.motion                      │    └ ros_motion 또는 stand.motion
           └ model                          └ ROS 스레드: 토픽·서비스
  └ telemetry (UDP/CSV)                        ├ /huphy/joint_states
                                               ├ /huphy/status
                                               └ /huphy/command 받음
```

**가운데 기둥(`build_biped` → `ControlLoop` → `Motion` → CAN)은 한 줄도 안 바뀜.**
바뀌는 것은 맨 위(무엇이 `Motion` 을 주는가)와 맨 아래(상태를 어디로 내보내는가)
둘뿐임.
