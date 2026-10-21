# `ros/` — ROS 2 래퍼

`src/huphy/` 를 **고치지 않고** ROS 2 노드로 감싼 것. 설계 판단, 코드 설명, 쓰는
법을 한 곳에 둠.

```
ros/
└── huphy_ros/                 ament_python 패키지. colcon 이 이것만 빌드함
    ├── package.xml  setup.py  setup.cfg  resource/huphy_ros
    ├── launch/                control · policy · local
    └── huphy_ros/
        ├── convert.py         단위·이름 변환.      rclpy 를 import 하지 않음
        ├── freshness.py       받은 값의 나이 판정.  rclpy 를 import 하지 않음
        ├── control_node.py    모터를 몲
        └── policy_node.py     갈 자세를 냄
```

그림은 [`ros.drawio`](ros.drawio).

기기 이름은 안 씀. 역할로 부름 — **모션보드**(CAN 어댑터와 IMU 가 달린 쪽),
**작업 기계**(신경망을 돌리고 사람이 쓰는 쪽).

---

## 0. 정해진 것

| # | |
|---|---|
| 1 | 관절상태 → 정책 → 관절목표각 한 바퀴가 **네트워크를 건너감** |
| 2 | 목표각이 끊기면 **감쇠 후 토크 차단** |
| 3 | 기준 자세로 옮기는 접근 단계는 **`policy_node`** 가 함 |
| 4 | 사람이 보내는 것: 속도 명령(`vx`, `wz`) / 시작·정지·토크 / 관절 목표각 직접 |
| 5 | **정책 교체 없음.** `.pt` 를 바꾸려면 노드 재시작 |
| 6 | 추론은 **타이머 50Hz**. 상태가 낡으면 목표각을 안 내보냄 |
| 7 | IMU 는 **모션보드**에 붙음. 관절 상태와 같이 네트워크를 건너감 |
| 8 | `/huphy/joint_targets` **하나**를 정책과 pose 가 같이 씀. 중재 없음 |
| 9 | 기계 간 **시계 동기 없음.** 각 노드가 자기 시계로 도착 간격만 봄 |
| 10 | 상태 발행 주기는 파라미터, **기본 100Hz** |
| 11 | **표준 메시지**만 씀. 직접 정의한 메시지 없음 |
| 12 | 낡음 기준 **60ms / 100ms**, 통신두절 5주기 |
| 13 | **기계 둘.** 모션보드에 `control_node`, 작업 기계에 `policy_node` 와 사람 |

정책은 `generalized_walk` 하나임
([`control/generalized_walk.py`](../src/huphy/control/generalized_walk.py)).
관찰 44칸, 행동 12개, 명령 2칸(`vx`, `wz`). `vy` 는 **칸 자체가 없음.**

---

## 1. ROS 개념 — 이 로봇에 짝지어서

| 개념 | 무엇 | 여기서 |
|---|---|---|
| 노드 | 프로세스 하나 안에서 도는 작업 단위 | `control_node`, `policy_node` |
| 토픽 | 이름이 붙은 단방향 방송. 받는 쪽이 없어도 보냄 | 관절 상태, IMU, 목표각, 속도 명령 |
| 서비스 | 요청-응답 한 쌍 | 토크 on/off, 정책 시작·정지 |
| 파라미터 | 노드마다 가지는 설정값 | 설정 파일 경로, 주기, 임계값 |
| 메시지 | 주고받는 자료형 | `sensor_msgs/JointState` 등 표준만 |
| QoS | 토픽마다의 신뢰성 설정 | 3-4 |
| 런치 | 노드 여러 개를 한 번에 띄우는 파이썬 스크립트 | 7-3 |

**액션은 안 씀.** 오래 걸리는 것이 접근 단계 하나인데 그건 `policy_node` 안에서
끝남 (결정 3).

---

## 2. 전체 그림

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
                        └───────────────────┴──────────────────────┘
                              네트워크
                     ROS_DOMAIN_ID 양쪽 같게 · 유선
```

### 네트워크를 타는 것과 안 타는 것

```
탐         joint_states  imu  gravity      100Hz   모션 -> 작업
           joint_targets                    50Hz   작업 -> 모션
           loop_status                       5Hz   모션 -> 작업
           enable  disable                   사람  작업 -> 모션

안 탐      CAN 프레임                            모션보드 안
           시리얼 IMU 읽기                        모션보드 안
           가드·통신두절 판정                     모션보드 안
           신경망 추론                            작업 기계 안
           q(t-1) 보관                            작업 기계 안
           cmd_vel                               작업 기계 안
           start_policy / stop_policy            작업 기계 안
```

### 한 주기에 값이 지나는 길

```
 1  모터 12개 ──CAN──> CanBus ──> Leg ──> Biped
                                            │ Observation (도, cal)
 2                            control_node ─┤
                                            │ JointState (rad) + Imu + Vector3
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
가 그 함수를 그대로 부름** — 배치를 ROS 쪽에서 다시 만들지 않음.

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

## 3. 노드 둘

### 3-1. `control_node` — 모션보드

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

### 3-2. `policy_node` — 작업 기계

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

### 3-3. 사람이 치는 것

노드를 안 만듦. `policy_node` 와 **같은 기계**에서 기성 도구로 함.

```
ros2 service call /huphy/enable std_srvs/srv/Trigger
ros2 topic pub -r 20 /huphy/cmd_vel geometry_msgs/msg/Twist '{}'
ros2 bag record -a
rqt_plot /huphy/joint_states/position[3]
```

---

## 4. 토픽

| 토픽 | 타입 | 내는 쪽 | 주기 | QoS |
|---|---|---|---|---|
| `/huphy/joint_states` | `sensor_msgs/JointState` | control | 100Hz | sensor_data |
| `/huphy/imu` | `sensor_msgs/Imu` | control | 100Hz | sensor_data |
| `/huphy/gravity` | `geometry_msgs/Vector3Stamped` | control | 100Hz | sensor_data |
| `/huphy/loop_status` | `diagnostic_msgs/DiagnosticArray` | control | 5Hz | 기본 |
| `/huphy/joint_targets` | `sensor_msgs/JointState` | policy 또는 사람 | 50Hz | sensor_data |
| `/huphy/cmd_vel` | `geometry_msgs/Twist` | 사람 | 10~20Hz | 기본 |
| `/huphy/policy_status` | `diagnostic_msgs/DiagnosticArray` | policy | 5Hz | 기본 |

**전부 표준 메시지임** (결정 11). 직접 정의한 메시지가 없으므로 메시지 전용
패키지도 없고, `ament_python` 하나로 끝남.

### 4-1. 중력방향을 따로 내보내는 이유

`sensor_msgs/Imu` 에는 `orientation`(쿼터니언), `angular_velocity`,
`linear_acceleration` 만 있음. **중력방향 3칸이 들어갈 자리가 없음.**

이 저장소의 `ImuState` 는 자세를 중력방향으로만 들고 있음
([`sensors/base.py`](../src/huphy/sensors/base.py) 의 `gravity`). 벤더 모듈이
쿼터니언이든 오일러든 받아서 중력방향으로 만들어 올리는 구조이고, 정책이 쓰는 것도
그 3칸임 (`generalized_walk` 관찰 3-5칸).

쿼터니언으로 되돌려 넣었다가 정책 쪽에서 다시 풀면 **왕복에서 틀릴 자리가 생김.**
그래서 `Vector3Stamped` 로 그대로 내보냄.

`Imu` 토픽도 같이 내는 이유는 표준 도구(rviz, 기록)가 그걸 보기 때문임.

### 4-2. `joint_targets` 에 `JointState` 를 쓰는 이유

목표각 전용 표준 메시지가 없음. `JointState` 의 `name` 과 `position` 만 채우고
`velocity`/`effort` 는 비움.

**받는 쪽은 `name` 을 봐야 함.** 배열 순서를 믿으면 보내는 쪽이 순서를 바꿨을 때
조용히 엉뚱한 관절이 움직임.

### 4-3. QoS

| | |
|---|---|
| `sensor_data` | BEST_EFFORT, depth 5. 빠진 것을 다시 안 보냄 |
| 기본 | RELIABLE, depth 10. 빠지면 다시 보냄 |

고주기 값에 RELIABLE 을 쓰면 안 됨 — 다시 보낼 때쯤이면 이미 낡은 값이고, 재전송이
새 값 뒤에 줄을 섬.

**보내는 쪽과 받는 쪽 설정이 다르면 연결이 아예 안 됨.** 에러도 경고도 없이 토픽이
비어 있음. `ros2 topic info -v <토픽>` 으로 양쪽을 확인함.

---

## 5. 단위와 이름

### 5-1. 단위

ROS 는 SI 를 씀 (REP 103). 이 저장소는 도를 씀.

| | huphy 안쪽 | ROS 경계 |
|---|---|---|
| 각도 | 도 | 라디안 |
| 각속도 | 도/초 | rad/s |
| 토크 | Nm | Nm (같음) |

변환은 `huphy_ros/convert.py` **한 곳에만** 둠. 저장소 안에 도↔라디안이 이미 두
곳 있는데(프로토콜 경계 `codec/mit.py`, 모델 경계 `control/generalized_walk.py`)
서로 안 만남. 세 번째가 여기저기 흩어지면 어디서 한 번 더 변환됐는지 추적이 안 됨.

### 5-2. 관절 이름

`Biped` 는 `right_leg/knee` 처럼 `/` 로 팔다리를 앞에 붙임
([`robots/biped.py`](../src/huphy/robots/biped.py) 의 `SEP`).

ROS 에서 `/` 는 네임스페이스 구분자라 사람이 토픽 이름과 헷갈림. 경계에서 바꿈.

```
huphy   right_leg/knee
ROS     right_leg__knee
```

---

## 6. 주기와 끊김

### 6-1. 정책 추론은 타이머가 돌림

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

### 6-2. 끊겼을 때 — 두 단계

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

| 값 | 어디 | 뜻 |
|---|---|---|
| 60ms | `policy_node` | 상태가 이보다 낡으면 목표각을 안 냄. 50Hz 기준 3주기 |
| 100ms | `control_node` | 목표각이 이보다 안 오면 감쇠. 50Hz 기준 5주기 |
| 5주기 | `robot.yaml` 의 `safety.link_loss_cycles` | 모터가 연속으로 답을 안 하면 정지 |

앞의 둘은 **실물 측정 전의 제안값**임.

### 6-3. 시계

기계 간 시계를 안 맞춤 (결정 9). 각 노드가 **자기 시계로 마지막 수신 이후 흐른
시간**만 봄. 편도 지연은 알 수 없고, 끊겼는지는 알 수 있음.

메시지의 `header.stamp` 는 **채워서 보내되 나이 판정에는 안 씀.** 기록(`ros2 bag`)과
나중에 시계를 맞출 때를 위해 남겨 두는 것임.

---

## 7. 안전

| 상황 | 막는 것 | 어디 |
|---|---|---|
| 목표각이 안 옴 | 감쇠 후 토크 차단 | `control_node`, 100ms |
| 상태가 안 옴 | 목표각 발행 중단 | `policy_node`, 60ms |
| 모터가 답을 안 함 | 루프 정지 | `safety/link.py`, 이미 있음 |
| 한계·점프 | 자름 | `safety/guards.py`, 이미 있음 |
| 명령값이 NaN | 그 모터만 안 보냄 | `safety/guards.py`, 이미 있음 |
| 노드를 `kill -9` | **못 막음.** 토크가 켜진 채 남음 | — |

**토크는 명시해야 들어감.** `control_node` 가 `Mode.OBSERVE` 로 시작하고,
`/huphy/enable` 서비스를 받아야 `Mode.CONTROL` 이 됨.

**정책과 pose 를 동시에 쓰면 안 됨** (결정 8). 토픽 하나를 같이 쓰므로 둘 다 쏘면
번갈아 들어감. 사람이 `policy_node` 를 시작·정지해서 정함.

---

## 8. 코드

### 8-1. 파일

| 파일 | 하는 일 | `rclpy` |
|---|---|---|
| `huphy_ros/convert.py` | 도↔라디안, 관절 이름 변환 | **안 씀** |
| `huphy_ros/freshness.py` | 받은 값과 그 나이. 낡음 판정 | **안 씀** |
| `huphy_ros/control_node.py` | `ControlLoop` 를 자기 스레드에서. CAN | 씀 |
| `huphy_ros/policy_node.py` | 20ms 타이머 추론. 접근 단계 | 씀 |

앞의 둘이 `rclpy` 를 안 쓰는 것이 의도임. 거기 있는 판단(단위, 이름, 낡음)이 틀리면
로봇이 잘못 움직이는데 **틀려도 길이와 자릿수가 그대로라 코드로는 안 잡힘.** ROS 가
없는 기계에서 테스트가 돌아야 고정해 둘 수 있음.

### 8-2. `convert.py`

| 함수 | 방향 |
|---|---|
| `to_ros_name` / `to_huphy_name` | `right_leg/knee` ↔ `right_leg__knee` |
| `joint_names` | 관찰 사전에서 관절 이름을 뽑음. `.pos` 가 있는 것만 |
| `observation_to_arrays` | 관찰(도) → `JointState` 네 배열(rad) |
| `arrays_to_observation` | `JointState` → 관찰 사전. `policy_node` 가 씀 |
| `targets_to_arrays` | 목표각(도) → `JointState`(rad). 순서는 `ORDER` 가 정함 |
| `arrays_to_targets` | `JointState` → `Action`(도). `control_node` 가 씀 |
| `imu_arrays` | `ImuState` → 각속도(rad/s)·가속도·중력방향 |

**없는 항목은 `0.0` 으로 채움.** `JointState` 는 네 배열의 길이가 같아야 하고 다르면
받는 쪽이 짝을 못 맞춤 — 0 이 옳은 값이라는 뜻이 아니라 길이를 맞추는 것이 필수라서임.

**길이가 다르면 멈춤.** `zip` 은 짧은 쪽에 맞춰 조용히 잘라서, 관절 하나가 빠진
명령이 그대로 통과함.

### 8-3. `freshness.py`

```python
Freshness(stale_s, clock=time.monotonic)
    put(value)          받은 값을 넣음. ROS 콜백에서
    take() -> (값, 쓸 수 있나)
    age()               마지막 수신 이후 초. 없었으면 -1.0
    clear()             비움

Latest()
    put(value) / take()     나이를 안 봄
```

`Latest` 는 반대 방향에 씀 — 제어 스레드가 관찰을 써 두고 발행 타이머가 꺼냄.
발행이 한 번 늦는 것은 문제가 아니므로 낡음 판정이 필요 없음.

**"아직 없음" 을 `None` 으로 둠.** `0.0` 을 쓰면 안 되는 이유는 `clock()` 이
정말로 `0.0` 을 낼 수 있기 때문임 — `monotonic` 은 기준점이 정해져 있지 않음.
만들다 이걸 테스트가 잡았음.

`time.monotonic` 을 쓰는 이유: 벽시계는 보정으로 뒤로 갈 수 있어 나이가 음수가 됨.
`huphy` 전체가 같은 이유로 `monotonic` 을 씀.

### 8-4. `control_node.py` — 모드를 루프 재시작으로 바꿈

`ControlLoop` 는 모드를 **생성할 때** 받고 `_enter` 에서 `enable()`/`disable()` 을
부름. 돌고 있는 루프의 모드를 바꾸는 길이 없음.

```
시작        OBSERVE 루프를 띄움        토크 안 들어감
/enable     OBSERVE 를 멈추고 CONTROL 루프를 새로 띄움
/disable    CONTROL 을 멈춤 -> finally 가 감쇠 후 토크 차단
```

로봇 객체는 그대로 두고 루프만 바꿈. `_enter` 가 `is_connected` 를 보고 이미
연결돼 있으면 다시 안 엶.

`CONTROL` 을 멈출 때는 `_exit` 가 `DAMPING_S`(5초) 동안 감쇠를 보냄. 그만큼 걸림.

**`enable` 서비스는 토크가 실제로 들어갔는지 확인하고 답함.** 루프를 띄우기만 하고
성공으로 답하면, `robot.enable()` 이 실측값이 없어 예외를 낼 때 스레드가 조용히
죽는데 부르는 쪽은 토크가 들어간 줄 앎.

**목표각이 낡으면 `_motion` 안에서 `loop.stop()`.** 플래그만 세우는 일이라 그 주기는
정상으로 끝나고, `finally` 가 감쇠 뒤 토크를 끊음 (결정 2).

### 8-5. `policy_node.py` — 세 단계

```
꺼짐      아무것도 안 냄
접근      지금 자세에서 기준 자세로 approach_s 초 동안 보간한 목표각
정책      신경망 출력
```

`/huphy/start_policy` 가 꺼짐 → 접근, `approach_s` 가 지나면 접근 → 정책.
`/huphy/stop_policy` 가 꺼짐으로.

접근을 여기서 하는 이유 (결정 3): 기준 자세
(`generalized_walk.DEFAULT_POSE_DEG`)를 아는 쪽이 여기뿐임. 모션보드에도 두면 같은
값이 두 군데가 됨.

**`generalized_walk.motion()` 을 안 씀.** 그 함수는 `command` 를 호출 전체에서
고정으로 받음 — `huphy-run --vx --wz` 가 시작할 때 한 번 정하는 구조임. 여기는
`/huphy/cmd_vel` 로 매 주기 바뀔 수 있어서, 안의 함수 둘을 직접 부르고
`q(t-1)`·`last_action` 보관을 노드가 함.

```python
vector  = generalized_walk.observation_vector(obs, imu_state, last_action, prev, command)
action  = model(vector)
targets = generalized_walk.joint_targets(action)
```

`motion()` 의 인자를 함수로 바꾸는 선택지도 있었는데, 그러면 ROS 를 안 쓰는
`huphy-run` 경로까지 같이 바뀌어서 안 했음.

접근 단계는 `scripts.selftest.approach(targets, start, seconds)` 를 그대로 씀.
지금 자세에서 목표까지 선형 보간하는 함수를 내는 순수 함수라 ROS 와 무관함.

---

## 9. `src/huphy/` 는 안 고침

두 노드가 쓰는 것이 이미 전부 공개돼 있음.

| 노드 | `huphy` 에서 쓰는 것 |
|---|---|
| `control_node` | `config.load_robot`, `scripts.bringup.build_biped`, `control.ControlLoop`, `control.Mode` |
| `policy_node` | `control.rsl_rl.load`, `control.generalized_walk` 의 `ORDER`·`SPEC`·`DEFAULT_POSE_DEG`·`LIMITS_DEG`·`observation_vector`·`joint_targets`, `scripts.selftest.approach` |

### `huphy` 를 ROS 패키지로 안 만드는 이유

**빌드 경로가 둘이 됨.** `ament_python` 은 `setup.py` 를 보는데 이 저장소는
`pyproject.toml` 만 있음. 둘을 같이 두면 `pip install -e .` 과 `colcon build` 가
다른 경로로 설치해서 어느 쪽이 import 되는지가 환경에 달라짐.

**ROS 없이 못 돌게 됨.** 지금 `src/huphy/` 와 `tests/` 가 ROS 없는 기계에서 돎.
`package.xml` 을 붙이면 그 전제가 흐려짐.

`rosdep` 은 apt 패키지와 몇몇 pip 패키지만 알아서 `huphy` 를 `package.xml` 에 적을
수 없음. `pip install -e .` 를 전제로 둠.

---

## 10. 스레드

`control_node` 한 프로세스 안에 넷임.

| 스레드 | 하는 일 |
|---|---|
| 메인 | `rclpy.spin()`. 구독·서비스·발행 타이머 |
| 제어 | `ControlLoop.run()`. 100Hz, CAN 송수신 |
| 수신 ×2 | `CanBus._read_loop`. 채널마다. `huphy` 가 이미 띄움 |

**제어 스레드에서 `publish()` 를 부르지 않음.** 직렬화와 DDS 가 제어 주기 안에
들어감. 제어 스레드는 최신 관찰을 `Latest` 에 써 두기만 하고 발행 타이머가 꺼냄.

**`ControlLoop` 를 `rclpy` 타이머에 얹지 않음.** 절대 마감·스핀 구간·밀림 측정으로
주기를 자기가 지키는데([`control/loop.py`](../src/huphy/control/loop.py) 의
`_sleep_until`, `SPIN_THRESHOLD_S`), executor 가 콜백을 꺼내 부르는 구조 위에
얹으면 그 지터가 더해짐.

**콜백 안에서 계산하지 않음.** 파이썬은 GIL 을 쥔 스레드가
`sys.getswitchinterval()`(기본 0.005초) 동안 시스템콜 없이 계속 돌 수 있음. 수거
예산이 2ms 이므로, 계산만 하는 콜백 하나가 그 주기의 수거를 통째로 넘기게 만듦.

---

## 11. 설치 (양쪽 기계에서)

| | |
|---|---|
| OS | Ubuntu 22.04 |
| ROS | Humble |
| 파이썬 | 3.10 |

ROS 환경을 먼저 불러온 뒤 `huphy` 를 같은 환경에 넣음. **순서가 중요함** —
`pip install` 이 어느 파이썬에 설치되는지가 환경에 달려 있음.

```bash
sudo apt install ros-humble-ros-base python3-colcon-common-extensions
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

```bash
ros2 pkg list | grep huphy
python3 -c "import huphy, huphy_ros; print(huphy.__file__)"
```

### 두 기계가 서로 보이게

```bash
export ROS_DOMAIN_ID=7      # 양쪽 같은 값으로. .bashrc 에
```

**안 맞으면 에러 없이 서로 안 보임.**

---

## 12. 실행 순서

### 1. 모션보드 — CAN 올리고 제어 노드

```bash
huphy-can up
ros2 launch huphy_ros control.launch.py config:=$PWD/config/robot.yaml
```

**토크는 안 들어감.** `OBSERVE` 로 시작함.

### 2. 읽히는지 확인

```bash
ros2 topic echo /huphy/joint_states --once
ros2 topic hz   /huphy/joint_states
ros2 topic echo /huphy/gravity --once
ros2 topic echo /huphy/loop_status --once
```

| 보는 것 | 맞는 값 |
|---|---|
| 관절 이름 | `right_leg__knee` 꼴. `/` 가 없어야 함 |
| 위치 단위 | **라디안.** 무릎 20도면 약 0.35. 손으로 움직이면 따라 변해야 함 |
| 주기 | `hz` 가 100 근처 |
| 중력방향 | 서 있으면 z 가 −1 근처 |
| `missing_cycles` | 거의 안 늘어야 함 |

**여기서 전부 확인한 뒤에 다음으로.**

### 3. 작업 기계 — 정책 노드

```bash
ros2 launch huphy_ros policy.launch.py weights:=$PWD/config/policies/generalized_walk.pt
```

띄우기만 하면 **아무것도 안 냄.** `/huphy/policy_status` 의 `stage` 가 `off` 임.

### 4. 토크 없이 정책을 먼저

아직 `OBSERVE` 라 모터는 안 움직임. 정책이 말이 되는 값을 내는지만 봄.

```bash
ros2 service call /huphy/start_policy std_srvs/srv/Trigger
ros2 topic hz   /huphy/joint_targets        # 50 근처
ros2 topic echo /huphy/joint_targets --once # -2 ~ 2 rad 범위면 정상
ros2 service call /huphy/stop_policy std_srvs/srv/Trigger
```

### 5. 토크 — 로봇을 들거나 테더에 맨 상태에서

```bash
ros2 service call /huphy/enable std_srvs/srv/Trigger
```

`success: true` 가 나와야 함. `false` 면 메시지에 이유가 있음.

### 6. 정책 시작

```bash
ros2 service call /huphy/start_policy std_srvs/srv/Trigger
```

`approach_s`(기본 3초) 동안 기준 자세로 옮긴 뒤 신경망으로 바뀜. `stage` 가
`off → approach → running` 으로 감.

```bash
ros2 topic pub -r 20 /huphy/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.2}, angular: {z: 0.0}}'
```

`linear.x`(vx) 와 `angular.z`(wz) 만 씀. `linear.y` 는 받아도 버림 — 정책 관찰에
`vy` 칸이 없음. 안 보내면 `(0, 0)` 이고 그게 제자리임.

### 멈출 때

```bash
ros2 service call /huphy/stop_policy std_srvs/srv/Trigger   # 목표각을 안 냄
ros2 service call /huphy/disable std_srvs/srv/Trigger       # 감쇠 후 토크 차단
```

`disable` 은 감쇠에 5초가 걸림. 급하면 런치 터미널에서 **Ctrl-C** — 같은 절차를 탐.

### 한 기계에서 둘 다

개발·측정용임. 실제 배치는 기계 둘임 (결정 13).

```bash
ros2 launch huphy_ros local.launch.py \
  config:=$PWD/config/robot.yaml \
  weights:=$PWD/config/policies/generalized_walk.pt
```

---

## 13. 파라미터

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
`config/calibration/*.json` 이 유일한 출처임. 파라미터로 또 받으면 어긋났을 때 어느
쪽이 진짜인지 알 수 없음 — 실제로 그 실수를 한 적이 있음(커밋 `4dbdfa2`, 무릎 kp 가
설정의 값 대신 코드 상수로 나감).

---

## 14. 테스트

`convert.py` 와 `freshness.py` 는 ROS 없이 돎.

```bash
cd <레포 경로>
PYTHONPATH=src python3 -m pytest tests/test_ros_convert.py tests/test_ros_freshness.py -q
```

`tests/conftest.py` 가 `ros/huphy_ros` 를 경로에 넣으므로 `PYTHONPATH` 에 따로 안
적어도 됨. 노드 둘은 `rclpy` 가 있어야 import 되어 여기서 안 봄.

---

## 15. 안 보일 때

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

ROS 1(Noetic)이 같이 깔려 있으면 **같은 셸에서 둘 다 `source` 하지 말 것.**
`ROS_DISTRO`, `PYTHONPATH`, `CMAKE_PREFIX_PATH` 가 서로 덮어씀.

---

## 16. 센서를 붙일 때

카메라·라이다를 **모션보드에 붙이지 않음.** 거기는 CAN 과 100Hz 제어뿐임.

그러면 작업 기계에 붙게 되는데, **거기가 정책이 도는 기계임** (결정 13). 그래서
막아야 할 것이 제어가 아니라 **정책**임.

| 무엇이 다투나 | 왜 |
|---|---|
| CPU | 포인트클라우드 처리와 50Hz 추론이 같은 코어를 씀 |
| GIL | 센서 노드가 파이썬이면 10장의 전환 간격 문제가 추론 타이머에 걸림 |
| 대역 | 프레임 하나가 수십 MB 라 UDP 조각으로 나감. 같은 링크의 `joint_targets` 가 그 뒤에 줄을 섬 |

마지막 것이 제일 나쁨. `joint_targets` 가 50Hz 로 가야 하는데 한 번 늦으면
모션보드의 100ms 임계에 가까워지고, 몇 번 이어지면 감쇠로 떨어짐.

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

## 17. 안 하는 것

| | 왜 |
|---|---|
| `ros2_control` | 발목이 크랭크 두 개로 두 축을 만드는 링키지라 기성 컨트롤러에 그 변환이 없음. 커스텀 컨트롤러를 C++ 로 써야 해서 기성품을 쓰는 이점이 사라짐 |
| 직접 정의한 메시지 | 표준으로 됨. 만들면 `ament_cmake` 패키지가 하나 더 필요함 |
| URDF / `robot_state_publisher` | rviz 로 보고 싶어질 때. 제어에는 안 쓰임 |
| 정책 교체 서비스 | 결정 5 |
| 액션 | 오래 걸리는 것이 접근 단계뿐이고 `policy_node` 안에서 끝남 |

### 아직 못 한 것

`rclpy` 가 없는 기계에서 만들어서 **노드 둘을 import 조차 못 해 봤음.** 문법과
`huphy` 쪽 호출은 확인했고, ROS 가 깔린 기계에서 처음 띄울 때 오타가 나올 수 있음.

두 기계 사이 왕복 지연도 안 쟀음. 6-2 의 60ms / 100ms 는 제안값이고, 재고 나서 다시
정해야 함.
