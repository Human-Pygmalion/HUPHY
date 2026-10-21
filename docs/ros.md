# ROS 2 이식 — 노드 계약

**아직 안 만든 것의 설계**임. ROS 를 써 본 적이 없다는 전제로 1장에 개념을 적고,
2장부터 이 로봇에 맞춘 계약을 적음.

기기 이름은 안 씀. 역할로 부름 — **모션보드**(로봇에 붙고 CAN 어댑터와 IMU 가
달린 쪽), **작업 기계**(신경망을 돌리고 사람이 쓰는 쪽).

**기계는 둘임.** 그림은 [ros.drawio](ros.drawio).

관련 문서: 계층은 [architecture.md](architecture.md), 한 주기의 흐름은
[cycle.md](cycle.md), 정책은 [`control/POLICY.md`](../src/huphy/control/POLICY.md).

정책은 `generalized_walk` 하나임
([`control/generalized_walk.py`](../src/huphy/control/generalized_walk.py)).
관찰 44칸, 행동 12개, 명령 2칸(`vx`, `wz`). `vy` 는 **칸 자체가 없음.**

---

## 0. 정해진 것

| # | |
|---|---|
| 1 | 관절상태 → 정책 → 관절목표각 한 바퀴가 **네트워크를 건너감** |
| 2 | 목표각이 끊기면 **감쇠 후 토크 차단** |
| 3 | 기준 자세로 옮기는 접근 단계는 **`policy_node`** 가 함 |
| 4 | 사람이 보내는 것: 속도 명령(`vx`, `wz`) / 시작·정지·토크 / 관절 목표각 직접 |
| 5 | **정책 교체 없음.** `.pt` 를 바꾸려면 노드 재시작. 정책 자체가 하나뿐임 |
| 6 | 추론은 **타이머 50Hz**. 상태가 낡으면 목표각을 안 내보냄 |
| 7 | IMU 는 **모션보드**에 붙음. 관절 상태와 같이 네트워크를 건너감 |
| 8 | `/huphy/joint_targets` **하나**를 정책과 pose 가 같이 씀. 중재 없음 |
| 9 | 기계 간 **시계 동기 없음.** 각 노드가 자기 시계로 도착 간격만 봄 |
| 10 | 상태 발행 주기는 파라미터, **기본 100Hz** |
| 11 | **표준 메시지**로 따로 보냄. 직접 정의한 메시지 없음 |
| 12 | 낡음 기준 **60ms / 100ms**, 통신두절 5주기 |
| 13 | **기계 둘.** 모션보드에 `control_node`, 작업 기계에 `policy_node` 와 사람 |

`src/huphy/` 는 **안 고침.** ROS 쪽이 import 만 함.

---

## 1. ROS 개념 — 이 로봇에 짝지어서

| 개념 | 무엇 | 여기서 |
|---|---|---|
| 노드 | 프로세스 하나 안에서 도는 작업 단위 | `control_node`, `policy_node` |
| 토픽 | 이름이 붙은 단방향 방송. 받는 쪽이 없어도 보냄 | 관절 상태, IMU, 목표각, 속도 명령 |
| 서비스 | 요청-응답 한 쌍 | 토크 on/off, 정책 시작·정지 |
| 파라미터 | 노드마다 가지는 설정값 | 설정 파일 경로, 주기, 임계값 |
| 메시지 | 주고받는 자료형 | `sensor_msgs/JointState` 등 표준만 씀 |
| QoS | 토픽마다의 신뢰성 설정 | 3-4 |
| 런치 | 노드 여러 개를 한 번에 띄우는 파이썬 스크립트 | 7장 |

**액션은 안 씀.** 오래 걸리는 것이 접근 단계 하나인데 그건 `policy_node` 안에서
끝남 (결정 3).

### 1-1. Humble 준비

| | |
|---|---|
| OS | Ubuntu 22.04 |
| 파이썬 | 3.10 |

```bash
sudo apt install ros-humble-ros-base python3-colcon-common-extensions
source /opt/ros/humble/setup.bash
```

워크스페이스는 소스와 설치 폴더가 나뉨.

```bash
mkdir -p ~/ros2_ws/src
ln -s <레포 경로> ~/ros2_ws/src/huphy
cd ~/ros2_ws && colcon build --symlink-install
source install/setup.bash
```

`colcon` 은 `src/` 아래를 재귀로 뒤져 `package.xml` 이 있는 폴더만 패키지로 봄.
레포 루트에는 없으므로 `ros/huphy_ros` 하나만 빌드됨.

---

## 1-2. 전체 그림

그림 파일은 [ros.drawio](ros.drawio). 아래는 같은 내용의 텍스트 판임 -- 터미널과
`git diff` 에서 보려고 같이 둠.

```
        모션보드  (로봇에 붙음)                    작업 기계
 ┌──────────────────────────┐          ┌──────────────────────────┐
 │ control_node             │          │ policy_node              │
 │                          │          │                          │
 │  메인    rclpy.spin()    │          │  메인   rclpy.spin()     │
 │  제어    ControlLoop     │          │         + 20ms 타이머    │
 │          100Hz           │          │                          │
 │  rx x2   CanBus          │          │  .pt -> numpy MLP        │
 │                          │          │  q(t-1), last_action 보관│
 └───┬──────────────────┬───┘          └────┬────────────────┬────┘
     │                  │                   │                │
     │ CAN              │   joint_states    │                │
     │ canr/canl        ├──────100Hz───────>│                │
     │ 시리얼 IMU       │   imu, gravity    │                │
     │                  │                   │                │
     │                  │<─────50Hz─────────┤                │
     │                  │   joint_targets   │                │
     v                  │                   │   cmd_vel      │
  모터 12개             │<┄┄ enable ┄┄┄┄┄┄┄ │   start_policy │
  IMU                   │     disable       │        ↓       │
                        │                   │  ┌──────────┐  │
                        ├┄┄ loop_status ┄┄> │  │  사람    │<─┘
                        │                   │  │ bag·rqt  │
                        │                   │  └──────────┘
                        │                   │  (나중) 카메라·라이다
                        └───────────────────┴──────────────────────┘
                              네트워크
                     ROS_DOMAIN_ID 양쪽 같게 · 유선
```

### 네트워크를 건너는 것과 안 건너는 것

```
건넘       joint_states  imu  gravity      100Hz   제어 -> 정책
           joint_targets                    50Hz   정책 -> 제어
           cmd_vel                       10~20Hz   작업PC -> 정책
           loop_status  policy_status       5Hz   -> 작업PC

안 건넘    CAN 프레임                            모션보드 안
           시리얼 IMU 읽기                        모션보드 안
           가드·통신두절 판정                     모션보드 안
           신경망 추론                            작업 기계 안
           q(t-1) 보관                            작업 기계 안
           cmd_vel                               작업 기계 안
           start_policy / stop_policy            작업 기계 안
```

**50Hz 한 바퀴가 네트워크를 건너감** (결정 1). 그 지연과 지터가 제어에 그대로
들어가므로 실물에서 재야 함 (5-2).

### 한 주기에 값이 지나는 길

```
 1  모터 12개 ──CAN──> CanBus ──> Leg ──> Biped
                                            │ Observation (도, cal)
 2                            control_node ─┤
                                            │ JointState (rad)  + Imu + Vector3
 3                      ─────네트워크─────> policy_node
                                            │
 4                            observation_vector(44칸)
                                 0-2   각속도        rad/s
                                 3-5   중력방향      단위벡터
                                 6-17  q(t-1)-기본   rad
                                 18-29 q(t)  -기본   rad
                                 30-41 직전 행동     날것
                                 42-43 vx, wz        cmd_vel 에서
 5                            model(vector) -> 행동 12
 6                            joint_targets() -> 목표각 12 (도)
                                            │ JointState (rad)
 7                      <────네트워크────── │
                                            │
 8  control_node ──> Biped ──> Leg ──> 가드 ──> MitCommand ──CAN──> 모터
```

4번의 44칸 배치는 `generalized_walk.observation_vector` 가 정함. **`policy_node`
가 그 함수를 그대로 부름** -- 배치를 ROS 쪽에서 다시 만들지 않음.

### 노드가 들고 있는 상태

둘 다 스레드가 둘 이상이라, 스레드 사이에 넘기는 값만 적음.

```
control_node
  Latest      최신 관찰        제어 스레드 -> 발행 타이머
  Freshness   최신 목표각      구독 콜백   -> 제어 스레드   기준 100ms

policy_node
  Freshness   최신 관절 상태   구독 콜백   -> 추론 타이머   기준 60ms
  Freshness   최신 IMU         구독 콜백   -> 추론 타이머   기준 60ms
  Latest      최신 cmd_vel     구독 콜백   -> 추론 타이머   나이 안 봄
  (평범한 변수) q(t-1), last_action        추론 타이머만 씀
```

`cmd_vel` 만 나이를 안 보는 이유: 안 오면 `(0, 0)` 으로 두면 되고, 그게 제자리
서기임 (`generalized_walk.COMMAND_ZERO`). 끊겼다고 로봇을 멈출 이유가 아님.

---

## 2. 노드 둘

### 2-1. `control_node` — 모션보드

```
하는 일   ControlLoop 를 자기 스레드에서 돌림. CAN 송수신. IMU 읽기
          가드(safety/guards.py)와 통신두절 판정(safety/link.py)은 여기서 돎
받음      /huphy/joint_targets        관절 목표각 12개
          /huphy/enable, /disable     서비스
보냄      /huphy/joint_states         관절 12개 위치·속도·토크
          /huphy/imu                  각속도·가속도
          /huphy/gravity              중력방향 3칸
          /huphy/loop_status          주기·밀림·무응답
```

**정책을 모름.** 목표각을 누가 보냈는지도 모름 (결정 8).

### 2-2. `policy_node` — 작업 기계

```
하는 일   .pt 를 읽어 추론. 관찰 44칸 조립, q(t-1) 보관, 행동 -> 목표각
          접근 단계(기준 자세로 보간)도 여기서 함
받음      /huphy/joint_states, /huphy/imu, /huphy/gravity
          /huphy/cmd_vel              속도 명령
          /huphy/start_policy, /stop_policy   서비스
보냄      /huphy/joint_targets        관절 목표각 12개
          /huphy/policy_status        단계·상태 나이·멈춘 이유
```

**CAN 을 모름.** 모터가 몇 개인지, id 가 뭔지 모름.

### 2-3. 사람이 치는 것

노드를 안 만듦. `policy_node` 와 **같은 기계**에서 기성 도구와 명령줄로 함.

```
ros2 service call /huphy/enable std_srvs/srv/Trigger
ros2 topic pub -r 20 /huphy/cmd_vel geometry_msgs/msg/Twist '{}'
ros2 bag record -a
rqt_plot /huphy/joint_states/position[3]
```

---

## 3. 토픽

### 3-1. 목록

| 토픽 | 타입 | 내는 쪽 | 주기 | QoS |
|---|---|---|---|---|
| `/huphy/joint_states` | `sensor_msgs/JointState` | control | 100Hz | sensor_data |
| `/huphy/imu` | `sensor_msgs/Imu` | control | 100Hz | sensor_data |
| `/huphy/gravity` | `geometry_msgs/Vector3Stamped` | control | 100Hz | sensor_data |
| `/huphy/loop_status` | `diagnostic_msgs/DiagnosticArray` | control | 5Hz | 기본 |
| `/huphy/joint_targets` | `sensor_msgs/JointState` | policy 또는 사람 | 50Hz | sensor_data |
| `/huphy/cmd_vel` | `geometry_msgs/Twist` | 사람 | 10~20Hz | 기본 |

**네트워크를 건너는 것은 넷뿐임** — `joint_states`, `imu`, `gravity`,
`joint_targets`, `loop_status`, 그리고 `enable`/`disable` 서비스. `cmd_vel` 과
`start_policy`/`stop_policy` 는 작업 기계 안에서만 오감 (결정 13).

`cmd_vel` 은 `Twist` 의 **`linear.x` 와 `angular.z` 두 칸만** 씀. 정책 관찰에
`vy` 칸이 없으므로 `linear.y` 는 받아도 버림
([`generalized_walk.py`](../src/huphy/control/generalized_walk.py) 의
`COMMAND_ZERO`).

지금 `huphy-run` 은 명령을 **실행 시작 때 한 번 정해서 고정**으로 줌
(`--vx`, `--wz`). `policy_node` 는 그것을 매 주기 바뀔 수 있는 값으로 바꾸는 것이고,
`generalized_walk.motion` 의 `command` 인자가 호출 전체에서 고정이므로 **그 부분은
`policy_node` 가 직접 조립해야 함** (`observation_vector` 를 매 주기 부르는 쪽으로).
| `/huphy/policy_status` | `diagnostic_msgs/DiagnosticArray` | policy | 5Hz | 기본 |

**전부 표준 메시지임** (결정 11). 직접 정의한 메시지가 없으므로 메시지 전용
패키지도 없고, `ament_python` 하나로 끝남.

### 3-2. 중력방향을 따로 내보내는 이유

`sensor_msgs/Imu` 에는 `orientation`(쿼터니언), `angular_velocity`,
`linear_acceleration` 만 있음. **중력방향 3칸이 들어갈 자리가 없음.**

이 저장소의 `ImuState` 는 자세를 중력방향으로만 들고 있음
([`sensors/base.py`](../src/huphy/sensors/base.py) 의 `gravity`). 벤더 모듈이
쿼터니언이든 오일러든 받아서 중력방향으로 만들어 올리는 구조이고, 정책이 쓰는 것도
그 3칸임 (`generalized_walk` 관찰 3-5칸).

쿼터니언으로 되돌려 `Imu.orientation` 에 넣었다가 정책 쪽에서 다시 풀면 **왕복에서
틀릴 자리가 생김.** 그래서 `Vector3Stamped` 로 그대로 내보냄.

`Imu` 토픽도 같이 내는 이유는 표준 도구(rviz, 기록)가 그걸 보기 때문임.

### 3-3. `joint_targets` 에 `JointState` 를 쓰는 이유

목표각 전용 표준 메시지가 없음. `JointState` 의 `name` 과 `position` 만 채우고
`velocity`/`effort` 는 비움.

**받는 쪽은 `name` 을 봐야 함.** 배열 순서를 믿으면 보내는 쪽이 순서를 바꿨을 때
조용히 엉뚱한 관절이 움직임.

### 3-4. QoS

| | |
|---|---|
| `sensor_data` | BEST_EFFORT, depth 5. 빠진 것을 다시 안 보냄 |
| 기본 | RELIABLE, depth 10. 빠지면 다시 보냄 |

고주기 값에 RELIABLE 을 쓰면 안 됨 — 다시 보낼 때쯤이면 이미 낡은 값이고, 재전송이
새 값 뒤에 줄을 섬.

**보내는 쪽과 받는 쪽 설정이 다르면 연결이 아예 안 됨.** 에러도 경고도 없이 토픽이
비어 있음. `ros2 topic info -v <토픽>` 으로 양쪽을 확인함.

---

## 4. 단위와 이름

### 4-1. 단위

ROS 는 SI 를 씀 (REP 103). 이 저장소는 도를 씀.

| | huphy 안쪽 | ROS 경계 |
|---|---|---|
| 각도 | 도 | 라디안 |
| 각속도 | 도/초 | rad/s |
| 토크 | Nm | Nm (같음) |

변환은 `huphy_ros/convert.py` **한 곳에만** 둠. 저장소 안에 도↔라디안이 이미 두
곳 있는데(프로토콜 경계 `codec/mit.py`, 모델 경계 `control/generalized_walk.py`)
서로 안 만남. 세 번째가 여기저기 흩어지면 어디서 한 번 더 변환됐는지 추적이 안 됨.

### 4-2. 관절 이름

`Biped` 는 `right_leg/knee` 처럼 `/` 로 팔다리를 앞에 붙임
([`robots/biped.py`](../src/huphy/robots/biped.py) 의 `SEP`).

ROS 에서 `/` 는 네임스페이스 구분자라 사람이 토픽 이름과 헷갈림. 경계에서 바꿈.

```
huphy   right_leg/knee
ROS     right_leg__knee
```

---

## 5. 주기와 끊김

### 5-1. 정책 추론은 타이머가 돌림

`policy_node` 가 20ms 타이머로 추론함 (결정 6). 메시지 도착이 아니라 타이머인
이유: **네트워크 지터가 그대로 추론 간격의 지터가 되기 때문**임.

정책은 20ms 간격으로 불린다고 가정하고 학습했고, 관찰 6-17칸이 `q(t-1)` 인데 그게
정확히 한 정책 주기 전 값이어야 함
([`generalized_walk.py`](../src/huphy/control/generalized_walk.py) 의
`HISTORY_FRAMES`, `HZ`).

```
타이머 20ms
  마지막 joint_states 나이 < 60ms   -> 추론하고 목표각을 냄
  나이 >= 60ms                      -> 아무것도 안 냄
```

`q(t-1)` 은 **직전 추론 때 쓴 관절각**임. 메시지 순번이 아니라 추론 호출 기준이므로,
중간에 상태가 두 번 와도 한 번만 쓰고 한 번은 버림.

### 5-2. 끊겼을 때 — 두 단계

```
상태가 60ms 넘게 안 옴
  policy_node 가 목표각 발행을 멈춤        policy_status 에 이유를 적음
      |
      | 40ms 더 지남
      v
control_node 가 목표각을 100ms 넘게 못 받음
  ControlLoop 을 멈춤 -> damp(kp=0) -> 토크 차단
```

`100 > 60` 이므로 순서가 뒤바뀌지 않음. 정책이 먼저 멈추고, 그 뒤에 제어가 멈춤.

세 임계값임.

| 값 | 어디 | 뜻 |
|---|---|---|
| 60ms | `policy_node` | 상태가 이보다 낡으면 목표각을 안 냄. 50Hz 기준 3주기 |
| 100ms | `control_node` | 목표각이 이보다 안 오면 감쇠. 50Hz 기준 5주기 |
| 5주기 | `robot.yaml` 의 `safety.link_loss_cycles` | 모터가 연속으로 답을 안 하면 정지. 이미 있는 값 |

앞의 둘은 **실물 측정 전의 제안값**임. 네트워크 왕복 지연과 그 분포를 재고 다시
정해야 함.

### 5-3. 시계

기계 간 시계를 안 맞춤 (결정 9). 각 노드가 **자기 시계로 마지막 수신 이후 흐른
시간**만 봄.

```
policy_node   now - 마지막 joint_states 수신 시각     <- 자기 시계
control_node  now - 마지막 joint_targets 수신 시각    <- 자기 시계
```

편도 지연은 알 수 없음. 끊겼는지는 알 수 있음.

메시지의 `header.stamp` 는 **채워서 보내되 나이 판정에는 안 씀.** 기록(`ros2 bag`)과
나중에 시계를 맞출 때를 위해 남겨 두는 것임.

---

## 6. 안전

| 상황 | 막는 것 | 어디 |
|---|---|---|
| 목표각이 안 옴 | 감쇠 후 토크 차단 | `control_node`, 100ms |
| 상태가 안 옴 | 목표각 발행 중단 | `policy_node`, 60ms |
| 모터가 답을 안 함 | 루프 정지 | `safety/link.py`, 이미 있음 |
| 한계·점프 | 자름 | `safety/guards.py`, 이미 있음 |
| 명령값이 NaN | 그 모터만 안 보냄 | `safety/guards.py`, 이미 있음 |
| 노드를 `kill -9` | **못 막음.** 토크가 켜진 채 남음 | — |

**토크는 명시해야 들어감.** `control_node` 가 `Mode.OBSERVE` 로 시작하고,
`/huphy/enable` 서비스를 받아야 `Mode.CONTROL` 이 됨. 파라미터를 빠뜨렸을 때 안
움직이는 쪽으로 떨어지게 함.

**정책과 pose 를 동시에 쓰면 안 됨** (결정 8). 토픽 하나를 같이 쓰므로 둘 다 쏘면
번갈아 들어감. 사람이 `policy_node` 를 시작·정지해서 정함.

---

## 7. 파라미터와 런치

### 7-1. `control_node`

| 이름 | 기본 | 무엇 |
|---|---|---|
| `config` | 없음. 필수 | `robot.yaml` 경로 |
| `limb` | 빈 값 = 양다리 | 다리 하나만 돌릴 때 |
| `hz` | 100.0 | 제어 주기 |
| `state_hz` | 100.0 | 상태 발행 주기 |
| `drain_ms` | 2.0 | 응답을 기다리는 최대 시간 |
| `target_stale_ms` | 100.0 | 목표각이 이보다 안 오면 감쇠 |
| `allow_uncalibrated` | false | 실측 전에도 토크를 넣음 |

### 7-2. `policy_node`

| 이름 | 기본 | 무엇 |
|---|---|---|
| `weights` | 없음. 필수 | `.pt` 경로 |
| `hz` | 50.0 | 추론 주기 |
| `state_stale_ms` | 60.0 | 상태가 이보다 낡으면 목표각을 안 냄 |
| `approach_s` | 3.0 | 기준 자세로 옮기는 시간 |

**게인·관절 한계·모터 id 는 양쪽 다 없음.** `robot.yaml` 과
`config/calibration/*.json` 이 유일한 출처임. 파라미터로 또 받으면 어긋났을 때 어느
쪽이 진짜인지 알 수 없음 — 실제로 그 실수를 한 적이 있음(커밋 `4dbdfa2`, 무릎 kp 가
설정의 값 대신 코드 상수로 나감).

### 7-3. 런치 파일

```
control.launch.py    모션보드에서. control_node 하나
policy.launch.py     작업 기계에서. policy_node 하나
local.launch.py      한 기계에서 둘 다. 측정·개발용
```

두 기계로 나눌 때는 양쪽에서 각각 띄움. `ROS_DOMAIN_ID` 를 같게 맞춰야 서로 보임 —
**안 맞으면 에러 없이 안 보임.**

---

## 8. 파일 구성

```
ros/huphy_ros/
├── package.xml  setup.py  setup.cfg  resource/huphy_ros
├── README.md
├── launch/
│   ├── control.launch.py
│   ├── policy.launch.py
│   └── local.launch.py
└── huphy_ros/
    ├── convert.py        단위·이름 변환.           rclpy 안 씀
    ├── freshness.py      수신 시각과 나이 판정.    rclpy 안 씀
    ├── control_node.py
    └── policy_node.py
```

`convert.py` 와 `freshness.py` 가 `rclpy` 를 import 하지 않는 것이 의도임. 거기
있는 판단(단위, 이름, 낡음)이 틀리면 로봇이 잘못 움직이는데 **틀려도 길이와 자릿수가
그대로라 코드로는 안 잡힘.** ROS 없는 기계에서 테스트가 돌아야 고정해 둘 수 있음
(`tests/test_ros_convert.py`, `tests/test_ros_freshness.py`).

### 8-1. `huphy` 를 ROS 패키지로 안 만드는 이유

`ament_python` 은 `setup.py` 를 보는데 이 저장소는 `pyproject.toml` 만 있음. 둘을
같이 두면 `pip install -e .` 과 `colcon build` 가 다른 경로로 설치해서 어느 쪽이
import 되는지가 환경에 달라짐.

그리고 `src/huphy/` 와 `tests/` 는 ROS 없는 기계에서 돎. `package.xml` 을 붙이면 그
전제가 흐려짐.

```bash
source /opt/ros/humble/setup.bash
cd <레포 경로> && pip install -e .
```

`rosdep` 은 apt 패키지와 몇몇 pip 패키지만 알아서 `huphy` 를 `package.xml` 에 적을
수 없음. 위 명령을 전제로 둠.

---

## 8-2. `src/huphy/` 는 안 고침

두 노드가 쓰는 것이 이미 전부 공개돼 있음.

| 노드 | `huphy` 에서 쓰는 것 |
|---|---|
| `control_node` | `config.load_robot`, `scripts.bringup.build_biped`, `control.ControlLoop`, `control.Mode` |
| `policy_node` | `control.rsl_rl.load`, `control.generalized_walk` 의 `ORDER`·`SPEC`·`DEFAULT_POSE_DEG`·`LIMITS_DEG`·`observation_vector`·`joint_targets`, `scripts.selftest.approach` |

### 한 군데만 쓰는 방식이 다름

`generalized_walk.motion(model, imu, command=...)` 은 `command` 를 **호출 전체에서
고정**으로 받음. `huphy-run --vx --wz` 가 시작할 때 한 번 정하는 구조임.

`policy_node` 는 `/huphy/cmd_vel` 로 매 주기 바뀔 수 있는 값을 받으므로 `motion()`
을 안 씀. 그 안의 함수 둘을 직접 부름.

```python
vector  = generalized_walk.observation_vector(obs, imu_state, last_action, prev, command)
action  = model(vector)
targets = generalized_walk.joint_targets(action)
```

`motion()` 이 하는 일이 이 셋에 `last_action`·`q(t-1)` 보관을 더한 것뿐이라,
`policy_node` 가 그 보관을 자기가 함. **`generalized_walk.py` 는 안 고침.**

고치는 선택지도 있었음 -- `motion()` 의 `command` 를 값 대신 함수로 받게. 안 한
이유는 `huphy-run` 이 그 인자를 고정값으로 쓰고 있어서, 바꾸면 ROS 를 안 쓰는
경로까지 같이 바뀌기 때문임.

### 접근 단계

`scripts.selftest.approach(targets, start, seconds)` 를 그대로 씀. 지금 자세에서
목표까지 선형 보간하는 함수를 내는 순수 함수라 ROS 와 무관함. `policy_node` 가
`/huphy/start_policy` 를 받으면

```
1  마지막으로 받은 joint_states 를 시작 자세로 읽음
2  approach(generalized_walk.default_pose(), 시작 자세, approach_s) 로 보간
3  approach_s 가 지나면 신경망 출력으로 바꿈
```

`control_node` 는 이 단계를 모름 -- 목표각만 받아 보냄 (결정 3).

---

## 9. 스레드

`control_node` 한 프로세스 안에 네 개임.

| 스레드 | 하는 일 |
|---|---|
| 메인 | `rclpy.spin()`. 구독·서비스·발행 타이머 |
| 제어 | `ControlLoop.run()`. 100Hz, CAN 송수신 |
| 수신 ×2 | `CanBus._read_loop`. 채널마다. `huphy` 가 이미 띄움 |

**제어 스레드에서 `publish()` 를 부르지 않음.** 직렬화와 DDS 가 제어 주기 안에
들어감. 제어 스레드는 최신 관찰을 한 칸에 써 두기만 하고, 발행 타이머가 꺼내 보냄.

**`ControlLoop` 를 `rclpy` 타이머에 얹지 않음.** 절대 마감·스핀 구간·밀림 측정으로
주기를 자기가 지키는데([`control/loop.py`](../src/huphy/control/loop.py) 의
`_sleep_until`, `SPIN_THRESHOLD_S`), executor 가 콜백을 꺼내 부르는 구조 위에
얹으면 그 지터가 더해짐.

**콜백 안에서 계산하지 않음.** 파이썬은 GIL 을 쥔 스레드가
`sys.getswitchinterval()`(기본 0.005초) 동안 시스템콜 없이 계속 돌 수 있음. 수거
예산이 2ms 이므로, 계산만 하는 콜백 하나가 그 주기의 수거를 통째로 넘기게 만듦.

---

## 10. 순서

| 단계 | 무엇 | 끝났다는 기준 |
|---|---|---|
| 1 | Humble 설치, 워크스페이스 | `ros2 topic list` 가 돎 |
| 2 | `convert.py`, `freshness.py` + 테스트 | ROS 없이 테스트 통과 |
| 3 | `control_node` — 관찰만 | `Mode.OBSERVE`. `ros2 topic echo` 로 관절각이 보임 |
| 4 | 같은 기계에서 둘 다 | `local.launch.py`. 토크 없이 목표각이 오가는지 |
| 5 | 토크 | `/huphy/enable`. 사람이 지켜보며 |
| 6 | 두 기계로 나눔 | 왕복 지연을 재고 임계값 다시 정함 |
| 7 | 센서 | 11장 |

3번을 먼저 하는 것이 중요함. 읽기만 하는 상태로 단위·이름·주기·QoS 를 전부 확인한
뒤에 토크를 넣음.

---

## 11. 센서를 붙일 때

카메라·라이다를 **모션보드에 붙이지 않음.** 거기는 CAN 과 100Hz 제어뿐임.

그러면 작업 기계에 붙게 되는데, **거기가 정책이 도는 기계임** (결정 13). 그래서
막아야 할 것이 제어가 아니라 **정책**임.

| 무엇이 다투나 | 왜 |
|---|---|
| CPU | 포인트클라우드 처리와 50Hz 추론이 같은 코어를 씀 |
| GIL | 센서 노드가 파이썬이면 9장의 전환 간격 문제가 추론 타이머에 걸림 |
| 대역 | 프레임 하나가 수십 MB 라 UDP 조각으로 나감. 같은 링크의 `joint_targets` 가 그 뒤에 줄을 섬 |

마지막 것이 제일 나쁨. `joint_targets` 가 50Hz 로 가야 하는데 한 번 늦으면
모션보드의 100ms 임계에 가까워지고, 몇 번 이어지면 감쇠로 떨어짐 (5-2).

대책 넷임. 위에서부터 효과가 큼.

| | |
|---|---|
| 센서를 **전용 보드**에 | 정책과 CPU·GIL 을 안 나눔. 기계가 셋이 됨 |
| 망을 나눔 | 제어용 링크와 센서용 링크를 물리적으로 분리 |
| `ROS_DOMAIN_ID` 를 나눔 | 센서 도메인과 제어 도메인을 나누고 필요한 토픽만 넘김 |
| 센서 노드를 C++ 로 | `realsense-ros`·`ouster-ros` 가 이미 C++ 임. GIL 을 안 씀 |

마지막 것이 공짜임 — 기성 드라이버가 이미 C++ 라 **파이썬 노드를 따로 안 만들면**
GIL 문제는 안 생김. CPU 와 대역은 남음.

**붙이기 전에 재야 할 것**: 센서 노드를 띄운 상태에서 `/huphy/joint_targets` 의
주기가 50Hz 를 지키는지. `ros2 topic hz` 로 보면 됨.

---

## 12. 안 하는 것

| | 왜 |
|---|---|
| `ros2_control` | 발목이 크랭크 두 개로 두 축을 만드는 링키지라 기성 컨트롤러에 그 변환이 없음. 커스텀 컨트롤러를 C++ 로 써야 해서 기성품을 쓰는 이점이 사라짐 |
| 직접 정의한 메시지 | 표준으로 됨. 만들면 `ament_cmake` 패키지가 하나 더 필요함 |
| URDF / `robot_state_publisher` | rviz 로 보고 싶어질 때. 제어에는 안 쓰임 |
| 정책 교체 서비스 | 결정 5 |
| 액션 | 오래 걸리는 것이 접근 단계뿐이고 `policy_node` 안에서 끝남 |
