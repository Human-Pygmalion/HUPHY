# 한 주기 — 데이터가 어떤 모양으로 어디를 지나는가

100Hz 한 사이클에서 값이 거치는 모든 변환을 순서대로 적음. 각 단계의 **자료형**,
**단위**, **그 일을 하는 코드**를 함께 둠.

호출 관계 그림은 [flow_diagrams.md](flow_diagrams.md), 계층을 나눈 이유는
[architecture.md](architecture.md).

---

## 0. 한눈에

양다리면 맨 위에 한 겹이 더 붙음 (`robots/biped.py`). 이름 앞의 팔다리로 나누고,
아래는 다리마다 똑같이 흐름.

```
Action              {"left_leg/knee": 30.0, ...}  양다리면 이름에 팔다리가 붙음
  |  robots/biped.py   split_action -- 첫 "/" 앞으로 나눔
Motion              {"knee": 30.0, ...}          관절 이름 -> 도 (cal)
  |  robots/leg.py     기구학 · 가드 · 캘리브레이션
MitCommand          {10: MitCommand(...)}        모터 id -> 도 (raw)
  |  robstride/codec/mit.py   deg -> rad -> 고정점 양자화
CanFrame            id=10, data=8바이트
  |  motors/canbus.py   python-can
CAN 선              11-bit 표준 프레임
  |  모터 펌웨어      tau = kp*(목표-현재) + kd*(0-속도) + tau_ff
CAN 선              응답 프레임 8바이트
  |  motors/canbus.py   drain
CanFrame            id=10, data=8바이트, stamp
  |  robstride/codec/mit.py   역양자화 -> rad -> deg
MotorState          {10: MotorState(...)}        모터 id -> 도 (raw)
  |  robots/leg.py     캘리브레이션 · 기구학
Observation         {"knee.pos": 29.97, ...}     관절 이름 -> 도 (cal)
```

| 단계 | 자료형 | 코드 |
|---|---|---|
| 0 나눔 (양다리) | `{팔다리: Action}` | [biped.py:352](../src/huphy/robots/biped.py#L352) |
| 1 관찰 | `Observation` = `Dict[str, float]` | [leg.py:383](../src/huphy/robots/leg.py#L383) |
| 2 목표 | `Action` = `Dict[str, float]` | [policy.py:285](../src/huphy/control/policy.py#L285) · [motions.py](../src/huphy/control/motions.py) |
| 3 명령 계산 | `Dict[int, MitCommand]` | [leg.py:520](../src/huphy/robots/leg.py#L520) |
| 4 패킹 | `List[CanFrame]` | [bus.py:199](../src/huphy/motors/robstride/bus.py#L199) |
| 5 전송 | `can.Message` | [canbus.py:343](../src/huphy/motors/canbus.py#L343) |
| 6 수거 | `List[CanFrame]` | [canbus.py:373](../src/huphy/motors/canbus.py#L373) |
| 7 해석 | `Dict[int, MotorState]` | [bus.py:231](../src/huphy/motors/robstride/bus.py#L231) |
| 8 기록 | `Dict[str, float]` | [snapshot.py:244](../src/huphy/telemetry/snapshot.py#L244) |
| 9 대기 | — | [loop.py:408](../src/huphy/control/loop.py#L408) |

한 사이클의 시작은 [loop.py:257](../src/huphy/control/loop.py#L257) `ControlLoop.step`.

---

## 1. 관찰 — `Observation`

`Leg.get_observation()` [leg.py:383](../src/huphy/robots/leg.py#L383).
**새로 통신하지 않음** — 직전 주기가 수거해 캐시에 넣은 값을 꺼내 변환만 함.

```python
{
    "hip_pitch.pos": 12.31,   # 도, cal 공간
    "hip_pitch.vel": -3.02,   # 도/초, raw 그대로 (부호 변환 없음)
    "hip_pitch.torque": 1.84, # Nm, 모터가 보고한 값
    "hip_pitch.temp": 31.4,   # 도C
    ...                       # 모터 6개 x 4필드 = 24
    "ankle_pitch.pos": 4.10,  # 도, FK 로 푼 관절각
    "ankle_roll.pos": -1.22,
    "ankle_pitch.vel": 0.31,  # 도/초, 야코비안 역으로 푼 값
    "ankle_roll.vel": -0.05,
    "stale_motors": 0,        # 한 번도 응답 없던 모터 수
}
```

키는 `motor_name.field` 이고 `observation_features`
[leg.py:275](../src/huphy/robots/leg.py#L275) 가 목록을 냄.

양다리면 `Biped.get_observation` [biped.py:274](../src/huphy/robots/biped.py#L274)
가 이름 앞에 팔다리를 붙여 합침 (`left_leg/knee.pos`).

변환 두 가지가 여기서 일어남.

| | 어디서 | 식 |
|---|---|---|
| raw → cal | [base.py:216](../src/huphy/motors/base.py#L216) `raw_to_cal` | `wrap180(sign * raw + offset)` |
| 모터각 → 발목각 | [leg.py:418](../src/huphy/robots/leg.py#L418) `_update_ankle_pose` | 뉴턴 반복 FK |

발목 FK 는 **주기당 한 번만** 풂. `collect`/`refresh` 가 끝날 때 계산해 두고
`get_observation` 은 꺼내 쓰기만 함 — 한 주기에 세 번 불리는데(정책·텔레메트리 두
갈래) 부를 때마다 풀면 같은 계산을 세 번 함.

---

## 2. 목표 — `Action`

`Motion` 계약 [loop.py:155](../src/huphy/control/loop.py#L155):

```
(경과 초, Observation) -> Optional[Action]
```

`Action` 은 **관절 이름 → 도 (cal 공간)** 임. `None` 이면 그 주기는 아무것도 안 보냄.

```python
{"hip_pitch": 0.0, "hip_roll": 0.0, "hip_yaw": 0.0,
 "knee": 30.0, "ankle_pitch": 5.0, "ankle_roll": 2.0}
```

### 학습한 정책이 낼 때

[policy.py](../src/huphy/control/policy.py) 가 dict ↔ 벡터를 오감. **모델은 관절
이름을 모르고 순서만 앎** — `JOINT_ORDER`
[policy.py:70](../src/huphy/control/policy.py#L70) 가 그 순서를 정함.

**순서가 하나가 아님.** 발목을 무엇으로 내느냐와 다리가 몇이냐로 갈리고,
`policy_motion(order=...)` 으로 넘김.

| | 마지막 두 칸 | 상수 |
|---|---|---|
| 관절 공간 | `ankle_pitch` `ankle_roll` | `JOINT_ORDER` [policy.py:70](../src/huphy/control/policy.py#L70) |
| 모터 공간 | `ankle_a` `ankle_b` | `MOTOR_ORDER` [policy.py:84](../src/huphy/control/policy.py#L84) |
| 양다리 12칸 | 위 여섯을 다리마다 | `BIPED_ORDERS` [policy.py:141](../src/huphy/control/policy.py#L141) |

양다리는 **왼다리 6칸 다음 오른다리 6칸**임 (`BIPED_LEGS`
[policy.py:116](../src/huphy/control/policy.py#L116)). 학습 쪽과 정한 규격이라
`robot.yaml` 의 다리 순서와 무관하게 고정돼 있음.

순서가 둘임. 발목을 발판 자세로 학습했으면 `JOINT_ORDER`, 모터로 학습했으면
`MOTOR_ORDER` 임 (`ankle_a`/`ankle_b`). `huphy-run --ankle-space` 로 고르고,
`ab` 면 발목이 기구학을 지나가지 않고 그대로 모터로 감.

**자세를 원본 형식으로 안 씀.** 센서가 오일러를 주든 쿼터니언을 주든 벤더 모듈이
중력방향으로 만들어 올리고, 정책은 `imu_state.gravity` 3칸만 봄
([sensors/base.py](../src/huphy/sensors/base.py)).

```
Observation(dict, 도) + ImuState
   |  observation_vector()          policy.py:210
np.float32[24]  또는 [26]           라디안, rad/s
   |  (x - mean) / std              가중치 파일에 들어 있음
   |  model(vector)                 rsl_rl.py
np.float32[6]                       행동 (무차원)
   |  joint_targets()               policy.py:255
Action(dict, 도)                    목표 = degrees(action_scale * 행동)
```

관찰 벡터의 칸 배치 (시뮬 mjlab 과 같아야 함):

| 칸 | 길이 | 값 | 단위 |
|---|---|---|---|
| `base_ang_vel` | 3 | IMU 각속도 | rad/s |
| `projected_gravity` | 3 | 중력 방향을 몸체 좌표로. **IMU 모듈이 만들어 올림** | 단위벡터 |
| `joint_pos` | 6 | 관절각 | rad |
| `joint_vel` | 6 | 관절 각속도 | rad/s |
| `actions` | 6 | 직전에 모델이 낸 값 그대로 | — |
| `hop_phase` | 2 | `sin`/`cos` | — (hopping 만) |

관절 칸 셋은 관절 수를 따라감 — 양다리면 6 이 아니라 12 임. 전체 길이는
`observation_size(n)` [policy.py:147](../src/huphy/control/policy.py#L147) 가
`3 + 3 + 3n` (+2) 으로 냄. 한 다리 24/26, 양다리 42/44.

길이가 `spec.obs_dim` 과 다르면 에러임
[policy.py:255](../src/huphy/control/policy.py#L255) — 순서가 어긋나면 값은 전부
정상인데 로봇만 엉뚱하게 움직여 코드로는 안 잡힘.

**단위가 갈리는 유일한 자리임.** 저장소 전체가 도인데 모델만 라디안을 씀.

---

## 3. 명령 계산 — `Action` → `Dict[int, MitCommand]`

`Leg.build_commands()` [leg.py:520](../src/huphy/robots/leg.py#L520).
**CAN 을 전혀 쓰지 않음** — 순수 계산이라 버스가 둘일 때 전송을 몰아 보낼 수 있음.

```
Action                         관절 이름 -> 도 (cal)
  0  _ankle_space()            leg.py:443   들어온 이름으로 rp / ab 판정
  1  _motor_targets()          leg.py:475   rp 면 발목만 IK, ab 면 그대로
     -> 모터 이름 -> 도 (cal)
  2  guards.apply()            guards.py:107  NaN · 한계 · 점프
     -> 잘린 도 (cal)
  3  cal_to_raw()              base.py:219
     -> 도 (raw)
  4  MitCommand                bus.py:59
```

양다리면 `Biped.build_commands` [biped.py:340](../src/huphy/robots/biped.py#L340)
가 먼저 이름으로 나누고 다리마다 위를 돌림. 결과는 **팔다리별로 나뉜 꾸러미**임 —
모터 id 는 버스 안에서만 유일해서 한 사전에 담으면 채널이 다른 같은 id 가 서로를
덮음.

```python
{"right_leg": {10: MitCommand(...)}, "left_leg": {4: MitCommand(...)}}
```

### 3-1. 관절 → 모터

**발목은 두 공간 중 하나로 옴.** 어느 쪽인지는 플래그가 아니라 **들어온 이름**이
말함 (`_ankle_space` [leg.py:443](../src/huphy/robots/leg.py#L443)). 플래그를 따로
들면 명령과 그 해석이 어긋날 수 있는데, 명령 자체가 공간을 말하면 어긋날 수가 없음.

```python
# rp -- 발판 자세. IK 를 거침
{"knee": 30.0, "ankle_pitch": 5.0, "ankle_roll": 2.0}
  -> {"knee": 30.0, "ankle_a": 2.44, "ankle_b": -6.99}

# ab -- 모터 각도. 그대로 통과
{"knee": 30.0, "ankle_a": 10.0, "ankle_b": -7.0}
  -> {"knee": 30.0, "ankle_a": 10.0, "ankle_b": -7.0}
```

나머지 4관절은 어느 쪽이든 이름만 바뀜(관절 이름 = 모터 이름).

- 모르는 이름은 **에러**임 — 오타를 무시하면 그 관절만 직전 명령을 유지해 자세가 어긋남.
- 발목은 **한 쌍을 통째로** 줘야 함. 하나만 오면 에러임.
- 두 공간을 **섞으면** 에러임. 어느 쪽을 따를지 정할 근거가 없음.
- `ab` + 발목 토크 출력은 **에러**임. 토크 경로가 관절 공간 PD 라 pitch/roll 이 있어야 하고, 그냥 두면 발목만 조용히 명령을 못 받음.
- IK 가 안 풀리면 발목을 **통째로** 버림 — 한쪽만 보내면 두 로드가 다른 자세를 요구해 링크가 비틀림.

### 3-2. 가드 — cal 공간에서

`guards.apply` [guards.py:107](../src/huphy/safety/guards.py#L107) 가 세 관문을
이 순서로 통과시킴.

| 순서 | 검사 | 결과 | 왜 이 순서인가 |
|---|---|---|---|
| 1 | 유한값 | NaN/Inf 면 **거부** | NaN 은 `min`/`max` 를 그냥 통과함. 뒤 클램프가 전부 무력화됨 |
| 2 | 위치 한계 | `command_margin_deg`(3도) 안쪽으로 **클리핑** | 안전한 목표를 먼저 정함 |
| 3 | 점프 | `max_delta_deg` 만큼만 **클리핑** | 거기로 가는 속도를 제한함 |

결과는 `GuardResult(value, reject, clips)` 이고 무엇이 걸렸는지 `counters` 에 쌓임
[leg.py:556](../src/huphy/robots/leg.py#L556). 클리핑은 **조용한 변조**라 반드시
세어 내보냄.

**변환보다 검사가 먼저인 이유**: 한계가 cal 공간에 있음. raw 로 내린 뒤 검사하면
`sign = -1` 인 관절에서 부호가 뒤집혀 한계가 반대로 걸림.

### 3-3. `MitCommand`

[bus.py:59](../src/huphy/motors/robstride/bus.py#L59). 다섯 칸이고 각도는 **raw
공간**임.

```python
MitCommand(position_deg=28.4, velocity_deg_s=0.0, kp=20.0, kd=1.0, torque_nm=0.0)
```

튜플이 아니라 이름 있는 필드로 두는 이유: 다섯 개가 전부 `float` 이라 순서를
틀려도 조용히 통과함. `kp` 자리에 위치가 들어가면 모터가 전력으로 튐.

발목을 토크로 보낼 때는 채우는 칸이 다름
[leg.py:574](../src/huphy/robots/leg.py#L574).

| | q | dq | kp | kd | tau_ff | 누가 PD 를 하나 |
|---|---|---|---|---|---|---|
| 위치 | 목표각 | 0 | 게인 | 게인 | 0 | 모터 펌웨어 |
| 토크 | 0 | 0 | 0 | 0 | 야코비안으로 내린 값 | 이 코드 |

---

## 4. 패킹 — `MitCommand` → 8바이트

`send_mit` [bus.py:199](../src/huphy/motors/robstride/bus.py#L199) 가 모터별
인코딩 범위를 붙여 `pack_command`
[mit.py:105](../src/huphy/motors/robstride/codec/mit.py#L105) 를 부름.

CAN 2.0 프레임은 데이터가 8바이트뿐이라 부동소수를 못 실음. 각 값을 `[-max, max]`
안에서 정수로 양자화함.

```
uint = (값 + max) / (2*max) * (2**bits - 1)
```

64비트를 이렇게 나눠 씀:

```
Byte0~1               목표각    16bit  <->  -12.57 ~ 12.57 rad
Byte2 + Byte3[7:4]    목표속도  12bit  <->  모델마다 다름
Byte3[3:0] + Byte4    Kp        12bit  <->  모델마다 다름
Byte5 + Byte6[7:4]    Kd        12bit  <->  모델마다 다름
Byte6[3:0] + Byte7    목표토크  12bit  <->  모델마다 다름
```

범위표는 [tables.py](../src/huphy/motors/robstride/tables.py) 의 `MIT_ENCODING`.
**모델마다 다름.**

    RS00  ±14 Nm   ±33 rad/s   Kp 0~500    RS03  ±60 Nm   ±20 rad/s  Kp 0~5000
    RS02  ±17 Nm   ±44 rad/s   Kp 0~500    RS04  ±120 Nm  ±15 rad/s  Kp 0~5000

한 다리 안에서도 갈림 — 0.5 는 발목만 RS00 이고, 1.0 은 hip_yaw 와 발목이 RS03 임. 표가 틀리면 그 비율만큼 토크가 어긋나는데,
프레임에는 Nm 이 아니라 눈금만 실려 나가므로 실물에서 찾기 매우 어려움.

### 실제 값

RS02 모터에 `position=30도, kp=20, kd=1, 나머지 0` 을 보낼 때:

```
q     = radians(30) = 0.5236 rad -> (0.5236+12.57)/25.14 * 65535 = 34132 = 0x8554
dq    = 0                        -> 절반값 2047                        = 0x7FF
kp_u  = 20/500 * 4095            = 163                                 = 0x0A3
kd_u  = 1/5 * 4095               = 819                                 = 0x333
tau   = 0                        -> 절반값 2047                        = 0x7FF

data  = 85 54 7F F0 A3 33 37 FF
```

위치 해상도는 `2 x 12.57 / 65535 rad = 0.022도`. 텔레메트리를 소수점 둘째 자리로
반올림해도 정보를 잃지 않는 근거가 이것임
[udp.py:45](../src/huphy/telemetry/udp.py#L45).

**범위를 넘으면 조용히 클램프됨** [mit.py:83](../src/huphy/motors/robstride/codec/mit.py#L83).
감싸지(wrap) 않으므로 폭주하지는 않지만 명령한 값과 다른 것이 나감. NaN 은 이
클램프도 통과해 최대값(720도 목표)이 되므로 3-2 에서 걸러야 함.

---

## 5. 전송 — `CanFrame` → 선

```python
CanFrame(can_id=10, data=b"\x85\x54\x7f\xf0\xa3\x33\x37\xff", is_extended=False)
```

[canbus.py:95](../src/huphy/motors/canbus.py#L95). `python-can` 의 `Message` 를
그대로 위로 흘리지 않는 이유: 그러면 `python-can` 없는 환경에서 import 가 깨져 순수
계산 계층의 테스트까지 막힘. 변환은 `send_many` 안에서만 일어남
[canbus.py:197](../src/huphy/motors/canbus.py#L197).

`is_extended=False` — MIT 프로토콜은 11-bit 표준 프레임이고 중재 id 가 곧 대상 모터
CAN id 임. private 프로토콜은 29-bit 확장이라 여기가 갈림.

`send_many` [canbus.py:343](../src/huphy/motors/canbus.py#L343) 가 6프레임을 락
**한 번** 잡고 연속으로 내보냄.

- 프레임마다 락을 잡았다 놓으면 사이에 다른 프레임이 끼어 한 관절의 명령이 버스에서 흩어짐.
- 중간에 실패해도 나머지를 계속 보냄. 첫 프레임 실패로 멈추면 나머지 5개가 직전 명령을 유지해 자세가 어긋남.
- 실패는 예외가 아니라 `tx_errors` 로만 셈.

1 Mbps 에서 8바이트 표준 프레임 하나가 약 0.13 ms 라 6개면 약 0.8 ms 임. 이 지연은
남아 있음 ([이슈 #10](issues.md)).

### 제어 명령은 배치가 다름

토크 on/off·고장 클리어는 값이 아니라 명령 코드임
[bus.py:81](../src/huphy/motors/robstride/bus.py#L81).

```
data[0:6] = 0xFF    data[6] = F_CMD    data[7] = 명령 코드
```

`F_CMD` 가 `0xFF` 면 기본 동작, 다른 값이면 변형임 — 같은 명령 바이트가 두 가지
뜻을 가짐. 코드표는 [tables.py:190](../src/huphy/motors/robstride/tables.py#L190).

---

## 6. 모터 펌웨어

받은 다섯 값으로 매 프레임 PD 를 계산함.

```
tau = kp * (목표각 - 현재각) + kd * (목표속도 - 현재속도) + 토크_FF
```

게인이 레지스터에 저장되는 방식이 아니라 **명령마다 실려 나감.** 그래서 한 주기
안에서도 관절마다 다르게 줄 수 있고, 브링업에서 전체를 한꺼번에 낮추는 것도 여기서
함 (`Gains.scaled` [base.py:74](../src/huphy/motors/base.py#L74)).

`kp = kd = tau_ff = 0` 이면 토크가 0 이라 아무 일도 안 일어남. `refresh_states`
[bus.py:277](../src/huphy/motors/robstride/bus.py#L277) 가 이걸 씀 — MIT 모드에는
읽기 전용 명령이 없어서, **움직이지 않는 명령을 보내고 그 응답으로 상태를 받음.**

---

## 7. 수거와 해석 — 선 → `MotorState`

### 7-1. `drain`

[canbus.py:373](../src/huphy/motors/canbus.py#L373).

```python
frames = bus.drain(expect=6, timeout_s=bus.drain_s, poll_s=0.0002)
```

`recv(timeout=2ms)` 를 한 번 부르면 큐가 비었을 때 2 ms 를 통째로 버림. 0.2 ms 씩
끊어 폴링하고 총 예산만 따로 둠. `expect` 개를 채우면 **즉시** 빠져나가므로
정상 주기에는 예산을 안 씀.

**예산은 버스가 들고 있음** — `RobStrideBus.drain_s`
[bus.py:101](../src/huphy/motors/robstride/bus.py#L101). 기본값이
`DEFAULT_DRAIN_S`(2 ms)이고 `huphy-test`/`huphy-run` 의 `--drain-ms` 로 바꿈.
**다리마다 따로 기다리므로 양다리 최악은 두 배**임.

`expect` 는 [leg.py:710](../src/huphy/robots/leg.py#L710) 에서
`len(self._awaiting)` — **직전에 명령을 보낸 모터 수**임. 전체를 기다리면 명령하지
않은 모터가 무응답으로 잡힘.

예산을 다 쓰면 `drain_timeouts` 를 올리고 나옴. 예외를 던지지 않음.

받은 프레임에 수신 시각(`time.monotonic()`)을 찍어 둠
[canbus.py:464](../src/huphy/motors/canbus.py#L464).

### 7-1-1. 수신 스레드 — 켜면 경로가 갈림

`CanBus(channel, reader=True)` 면 백그라운드 스레드가 `recv` 를 돌려 `deque` 에
쌓고, `drain` 은 **큐에서 꺼내기만** 함
[canbus.py:435](../src/huphy/motors/canbus.py#L435).

```
끄면   can1 기다림 -> can2 기다림      대기가 버스 수만큼 곱해짐
켜면   두 채널이 동시에 받음            큐에서 꺼내기만
```

양다리를 묶을 때 켬 (`build_biped`). **수신 스레드는 송신 락을 잡지 않음** — 잡으면
`recv` 에서 블로킹하는 동안 송신이 밀려 스레드를 둔 이유가 사라짐. 공유하는 것은
`deque` 뿐이고 `append`/`popleft` 가 원자적이라 락이 필요 없음.

큐가 차면 오래된 것부터 버리고 `rx_dropped` 를 올림. 닫을 때는 **스레드를 먼저
세우고** 채널을 닫음 — 반대면 닫힌 소켓에서 `recv` 를 부름.

예산을 넘겨 늦게 온 프레임은 **큐에 남음.** 제어 경로에는 `flush_rx` 가 없어서
다음 주기에 그것이 먼저 꺼내짐 (7-6).

### 7-2. 응답 프레임 배치

**명령과 배치가 다름** — 응답은 앞에 모터 id 가 붙어 한 칸씩 밀림.

```
Byte0                 모터 CAN ID
Byte1~2               현재각    16bit
Byte3 + Byte4[7:4]    현재속도  12bit
Byte4[3:0] + Byte5    현재토크  12bit
Byte6[7:6]            모드      0 Reset / 1 Cali / 2 Motor
Byte6[5]              고장      1 이면 고장 있음
Byte6[4]              경고
Byte6[3:0] + Byte7    권선 온도  12bit, 0.1도 단위
```

**온도는 12비트임.** Byte6 상위 4비트는 온도가 아니라 모드·고장·경고 플래그임
(매뉴얼 6.1절). 16비트로 읽으면 토크를 켰을 때 Motor 모드 비트가 서서 온도가
3300도로 나옴 — 토크가 꺼져 있을 때만 맞아서 벤치에서는 안 드러남.

플래그는 `decode_flags` [mit.py:168](../src/huphy/motors/robstride/codec/mit.py#L168)
가 따로 꺼냄. 고장 비트가 **상태 프레임마다 딸려 오므로** `read_fault` 왕복 없이
제어 주기 안에서 볼 수 있음. 다만 **아직 부르는 코드가 없음.**

인코딩 범위를 고를 때 중재 id 가 아니라 **`data[0]`** 을 봄
[bus.py:267](../src/huphy/motors/robstride/bus.py#L267). 중재 id 는 모델을 알려주지
않는데 한 다리에 RS02 와 RS00 이 섞여 있어 범위가 다름.

```
data = 0A 85 53 80 08 5F 01 3A
     -> decode_state()   mit.py:141
     -> (10, 29.97도, 0.46도/초, 0.79Nm, 31.4도C)
```

온도만 양자화가 아니라 `uint12 / 10` 임.

### 7-3. 캐시 갱신

[bus.py:250](../src/huphy/motors/robstride/bus.py#L250).

```python
self._states[10] = MotorState(
    position_deg=29.97,   # raw 공간
    velocity_deg_s=0.46,
    torque_nm=0.79,
    temp_c=31.4,
    stamp=time.monotonic(),
)
```

`stamp` 가 `time.monotonic()` 인 이유: 벽시계는 NTP 보정으로 뒤로 갈 수 있어 나이
계산이 음수가 됨. `stamp = 0` 은 **한 번도 못 받음** 을 뜻함(`is_valid`).

`collect` 는 **응답이 없었던 모터 id 목록**을 반환함. 예외가 아님 — 한 모터가 한
주기 빠지는 것은 흔한 일이고 그때마다 루프가 죽으면 안 됨.

### 7-4. 응답이 곧 ack

`Leg._note_link` [leg.py:717](../src/huphy/robots/leg.py#L717) 가 모터별로 둘을 셈.

| | 무엇 | 어디서 보나 |
|---|---|---|
| 연속 무응답 | 답하면 0 으로 돌아감 | `link_status` [leg.py:752](../src/huphy/robots/leg.py#L752) · 통신 두절 판정 |
| 누적 무응답 | 안 돌아감. 명령 수와 같이 셈 | `link_counts` [leg.py:738](../src/huphy/robots/leg.py#L738) · 실행 끝 집계 |

가끔씩 빠지는 모터는 연속 횟수로는 안 보임. 끝나고 "몇 번 빠졌나" 는 누적이 말함.

MIT 모드는 명령을 받으면 **반드시** 상태 프레임으로 답함. 안 오면 그 모터가 명령을
처리하지 않은 것임 — 이것이 애플리케이션 레벨 ack 임. CAN 하드웨어 ACK 는 버스의
아무 노드나 찍어주므로 "누군가 들었다" 일 뿐이고 그건 `tx_errors` 가 봄.

```
tx_errors = 0 인데 ack = 0   ->  모터가 명령을 무시함 (프로토콜·제어모드 불일치)
tx_errors > 0                ->  버스에 아무도 없음 (배선·전원)
```

### 7-5. 발목 FK

수거 직후 `_update_ankle_pose` [leg.py:418](../src/huphy/robots/leg.py#L418) 가
모터각 두 개에서 발목 `(pitch, roll)` 을 풂. 직전 결과를 초기 추정으로 씀 — 같은
모터각 조합이 서로 다른 자세 둘에 대응하므로 추정이 가까워야 함.

못 풀면 `_ankle_pose = None` 이고 추정값은 그대로 둠. 다음 주기가 마지막 성공
지점에서 다시 시작함.

### 7-6. 늦게 온 응답은 다음 주기로 밀림

`drain(expect=6)` 은 여섯 개를 채우면 즉시 나오고 **나머지는 큐에 둠.** 제어
경로에는 `flush_rx` 가 없음 — 부르는 곳은 `refresh_states`
[bus.py:277](../src/huphy/motors/robstride/bus.py#L277) 와 `read_fault` 뿐임.

그래서 예산을 넘겨 도착한 프레임이 있으면 다음 주기에 그것이 먼저 꺼내짐. 이번
주기 응답 일부가 또 큐에 남고, 그 모터가 무응답으로 찍힘. **한 번 밀리면 계속
밀림.**

관찰 모드는 매 주기 `refresh_states` 를 지나 큐를 비우므로 해당 없음. **제어
모드에서만** 생김.

증상은 실행 끝 집계로 갈림 (`scripts/failures.py`).

| | 뜻 |
|---|---|
| 보냄 ≈ 받음, 대기초과 많음 | 유실은 없고 늦게 온 것. 예산이나 비트레이트 |
| 받음이 확연히 적음 | 실제로 안 돌아옴. 배선·전원·프로토콜 |

---

## 8. 기록 — `Dict[str, float]` → UDP/CSV

[snapshot.py](../src/huphy/telemetry/snapshot.py) 가 **필드 이름을 정하는 유일한
곳**임. UDP 와 CSV 가 같은 사전을 소비함.

기록 단위는 로봇이 아니라 **팔다리**임 (`snapshot.parts`
[snapshot.py:155](../src/huphy/telemetry/snapshot.py#L155)). 다리 하나면 자기 자신
하나라 이름이 `right_leg/knee/pos` 그대로이고, 양다리여도 `huphy/right_leg/...` 로
깊어지지 않음. CSV 는 한 줄에 두 다리를 합치고 UDP 는 팔다리마다 한 패킷으로 보냄.

로봇에서 **읽기만** 함 — 새로 통신하지 않음. 기록이 CAN 을 건드리면 주기가 흔들림.

```
t                            시작부터 흐른 초
right_leg/knee/pos           실측 위치 (cal)
right_leg/knee/tgt           실제로 나간 목표. 명령한 값이 아님
right_leg/knee/err           tgt - pos
right_leg/ankle_pitch/pos    관절각 (FK)
right_leg/ankle_a/tau_cmd    tau_ff 로 시킨 토크
right_leg/knee/ack           1 응답 / 0 씹힘 / -1 명령 안 함
right_leg/guard/clip_limit   누적 카운터
right_leg/can/tx_errors      누적 카운터
imu/main/grav_x              중력방향. 정책이 실제로 본 값
imu/main/qw                  센서 고유 값. 목록이 센서마다 다름
```

IMU 이름에는 팔다리가 안 붙음. 센서는 다리 소속이 아니라 **로봇이 든 것**이라
`imus_of` [snapshot.py:168](../src/huphy/telemetry/snapshot.py#L168) 가 로봇이 든
것과 다리가 든 것을 `all_imus` 로 합쳐 한 묶음으로 봄. 이름이 겹치면 안 되는 이유임.

두 갈래로 나감.

| | 주기 | 필드 | 왜 |
|---|---|---|---|
| `build_fast` | 매 주기 | `pos tgt err vel tau` | 게인 튜닝에서 보는 값 |
| `build_diag` | 10주기마다 | `temp age ack miss`, 카운터 | 초 단위로 변하거나 사건 때만 변함 |

**나누는 이유는 패킷 크기임.** 합치면 약 1.8 KB 로 이더넷 MTU(1500)를 넘어
조각나고, 조각 하나만 잃어도 패킷 전체가 버려짐. CSV 는 크기 제약이 없어 한 줄에
다 담음.

UDP 는 JSON 한 줄이고 소수점 둘째 자리로 반올림함
[udp.py](../src/huphy/telemetry/udp.py). **보내고 잊음** — 받는 쪽이 없어도, 꺼져
있어도 제어 루프가 멈추지 않음. TCP 는 상대가 안 받으면 송신이 막혀 주기가 통째로
밀림.

`tgt` 가 **명령한 값이 아니라 실제로 나간 값**인 것이 중요함
[snapshot.py:244](../src/huphy/telemetry/snapshot.py#L244). 명령한 값을 기록하면
한계에 걸린 것과 게인이 낮은 것이 그래프에서 구분되지 않음.

기록 실패는 삼킴 [loop.py:438](../src/huphy/control/loop.py#L438) — 관측이 제어를
멈추면 관측할 대상이 없어짐.

`huphy-run` 과 `huphy-test` 는 **텔레메트리를 안 켬.** 대신 끝날 때 실패 집계를
찍음 (`scripts/failures.py`). 텔레메트리를 켜는 것은 `huphy-bringup` 뿐임.

---

## 9. 대기

[loop.py:408](../src/huphy/control/loop.py#L408) `_sleep_until`. 마감은 **절대
시각**(`cycle_start + period_s`)임. 매 주기 남은 시간을 새로 계산하면 오차가 쌓여
서서히 밀림.

```
남은 시간 > 3ms   ->  (남은 시간 - 3ms) 만큼 time.sleep
남은 시간 <= 3ms  ->  자지 않고 돌면서 마감을 봄
남은 시간 <= 0    ->  바로 돌아옴. 따라잡지 않음
```

`time.sleep` 은 요청보다 오래 잠. 마진 3 ms 를 빼 일찍 깨우고 나머지를 스핀으로
메꿈 — 마진이 잠의 오차보다 **작으면** 한 번의 긴 잠이 마감을 지나쳐 스핀 구간이
없어짐. 실측표가 [loop.py:97](../src/huphy/control/loop.py#L97) 에 있음. 실제 스핀은
주기당 약 1 ms 임.

**늦었으면 따라잡지 않음.** 밀린 만큼 다음 주기를 줄이면 그 주기가 더 짧아져 또
밀림.

---

## 10. 시간 예산 — 100Hz, 10 ms

| | 값 | 어디서 정함 |
|---|---|---|
| 프레임 6개 전송 | 약 0.8 ms | 1 Mbps · 8바이트 표준 프레임 |
| 수거 폴링 단위 | 0.2 ms | `DEFAULT_POLL_S` [canbus.py:77](../src/huphy/motors/canbus.py#L77) |
| 수거 총 예산 | 다리마다 2 ms | `RobStrideBus.drain_s` [bus.py:114](../src/huphy/motors/robstride/bus.py#L114) |
| 스핀 구간 | 3 ms | `SPIN_THRESHOLD_S` [loop.py:94](../src/huphy/control/loop.py#L94) |
| 밀림 판정 | 15 ms | `OVERRUN_RATIO` 1.5 [loop.py:88](../src/huphy/control/loop.py#L88) |

수거 예산은 **버스마다 따로 듦.** 기본값은 `DEFAULT_DRAIN_S`
[canbus.py:80](../src/huphy/motors/canbus.py#L80) 2 ms 이고, 실행할 때
`--drain-ms` 로 바꿈. 양다리는 순차로 기다리므로 **최악이 다리 수 배**임 — 100Hz
에 2 ms 면 최악 4 ms 로 주기의 40% 임.

예산은 **다 쓰지 않는 것이 정상임.** `expect` 를 채우면 즉시 나옴. 실행 끝 집계의
`수거 대기` 줄이 이 비율을 찍고, 절반을 넘으면 경고함
([failures.py:89](../src/huphy/scripts/failures.py#L89)).

`대기초과` 가 잦으면 둘 중 하나임.

| | 뜻 | 손댈 곳 |
|---|---|---|
| 보냄 ≈ 받음 | 응답이 예산보다 늦게 옴 | `--drain-ms` 를 늘리거나 Hz 를 낮춤. 비트레이트 확인 |
| 받음이 적음 | 실제로 안 옴 | 배선·전원 |

늦게 온 응답은 다음 주기로 밀림 — §7-6.

주기를 못 지켰는지는 두 가지로 봄 [loop.py:192](../src/huphy/control/loop.py#L192).

| | 무엇을 잡나 | 못 잡는 것 |
|---|---|---|
| `overruns` | 튀는 주기 | 꾸준히 24%씩 느린 것 |
| `kept_up` | 평균이 목표의 90% 미만 | 어느 주기가 튀었는지 |

`loop_dt` 를 매 주기 내보내는 이유: 느려진 것을 모른 채 게인을 튜닝하면 게인이
아니라 주기가 문제인데 게인을 계속 만지게 됨.

---

## 11. 모드에 따라 갈리는 곳

`ControlLoop.step` [loop.py:257](../src/huphy/control/loop.py#L257).

| | CONTROL | OBSERVE |
|---|---|---|
| 진입 | `robot.enable()` | `robot.disable()` — 토크 차단 |
| 목표 | `motion(t, obs)` | 없음 |
| 전송 | `build_commands` → `send` | `refresh()` 가 `kp=kd=tau=0` 명령을 보냄 |
| 수거 | `collect(expect=명령한 수)` | `collect(expect=전체)` |
| 통신 두절 판정 | `LinkWatch` 가 봄 [loop.py:280](../src/huphy/control/loop.py#L280) | 안 봄 |
| 종료 | `hold` 5주기 → 토크 차단 | 토크 차단 |

**관찰 모드도 통신함.** MIT 에는 읽기 전용 명령이 없어서 아무것도 안 보내면
아무것도 안 옴.

`refresh_states` 는 보내기 전에 `flush_rx` 로 큐를 비움
[bus.py:293](../src/huphy/motors/robstride/bus.py#L293). 직전 주기의 응답이 남아
있으면 이번 것으로 오해함. **제어 경로에는 이것이 없음** — §7-6.

종료는 **경로가 하나임** [loop.py:307](../src/huphy/control/loop.py#L307). 정상
종료든 예외든 `finally` 를 지나며 `hold` → 토크 차단 → 텔레메트리 flush 를 탐. 바로
끊으면 서 있는 다리가 주저앉음.

---

## 12. 어느 단계가 무엇으로 실패하나

| 단계 | 실패 | 처리 | 남는 흔적 |
|---|---|---|---|
| 정책 | 관찰 길이 불일치 | 예외 | — |
| 정책 | 출력 길이 불일치 | 예외 | — |
| 이름 나누기 | 모르는 팔다리·접두어 없는 이름 | 예외 | — |
| 관절→모터 | 모르는 관절 이름 | 예외 | — |
| 관절→모터 | 발목 반쪽·두 공간 섞임 | 예외 | — |
| 관절→모터 | `ab` + 토크 출력 | 예외 | — |
| 관절→모터 | 발목 IK 안 풀림 | 발목 **통째로** 버림 | 경고 로그 |
| 가드 | NaN/Inf | 그 모터만 안 보냄 | `reject_nan` |
| 가드 | 현재 위치 모름 | 그 모터만 안 보냄 | `reject_nostate` |
| 가드 | 한계 초과 | **자름** | `clip_limit` |
| 가드 | 급격한 변화 | **자름** | `clip_jump` |
| 패킹 | 인코딩 범위 초과 | 조용히 클램프 | 없음 |
| 전송 | `bus.send` 예외 | 나머지는 계속 보냄 | `tx_errors` |
| 수거 | 예산 초과 | 받은 것만 씀 | `drain_timeouts` |
| 수거 | 해석 실패 | 그 프레임만 버림 | 디버그 로그 |
| 수거 | 모터 무응답 | 직전 상태 유지 | `ack=0`, `miss`, `age` |
| 수거 | 다리 하나가 예외 | 그 다리 모터를 전부 무응답으로 | 경고 로그 |
| 통신 | 연속 무응답 5주기 | **루프를 멈춤** | `LinkLoss` |
| FK | 안 풀림 | 마지막 자세를 냄 | 디버그 로그 |
| 기록 | 어떤 예외든 | 삼킴 | 경고 로그 |

**버리는 것과 자르는 것을 구분함.** 버리면 그 모터만 직전 명령을 유지해 자세가
어긋남. 자르면 `max_delta` 씩 슬루해서 목표에 도달함 — 클리핑이 곧 속도 제한임.

한 다리가 통째로 죽어도 **다른 다리는 계속 돎.** `Biped._gather`
[biped.py:395](../src/huphy/robots/biped.py#L395) 가 예외를 잡고, 그 다리 모터를
전부 무응답 목록에 넣어 올림. 여기서 멈추면 뒤 다리를 수거하지 못해 멀쩡한 다리가
옛 상태로 남기 때문임.

정지는 `LinkWatch` [link.py:85](../src/huphy/safety/link.py#L85) 가 판정함.
`collect()` 뒤에 `robot.link_status()` 를 읽어 연속 무응답이 `link_loss_cycles`
(야믈, 기본 5) 에 닿은 모터가 있으면 `LinkLoss` 를 던짐. **제어 모드에서만** 봄
[loop.py:280](../src/huphy/control/loop.py#L280) — 관찰 모드는 세기만 함.

급정지가 아님. 예외가 `run()` 의 `finally` 를 지나므로 `hold` → 토크 차단 순서를
그대로 탐.

**주의 — 두 경로가 안 이어져 있음.** 연속 무응답을 세는 곳은 `Leg._note_link`
[leg.py:717](../src/huphy/robots/leg.py#L717) 이고 이것은 `Leg.collect()` 안에
있음. 그 `collect()` 가 예외로 끝나면 `_note_link` 를 못 지나 **그 다리의 연속
무응답이 안 늘어남.** `_gather` 가 올린 목록은 반환값일 뿐이고 판정에는 안 들어감.
즉 모터가 답을 안 하는 경우는 잡히지만, 버스 자체가 예외를 내는 경우는 `LinkWatch`
가 못 잡음.

---

## 13. 공간과 단위가 바뀌는 지점

| 경계 | 왼쪽 | 오른쪽 | 코드 |
|---|---|---|---|
| 모델 ↔ 저장소 | rad, 벡터 | 도, dict | [policy.py](../src/huphy/control/policy.py) |
| 로봇 ↔ 팔다리 | `right_leg/knee` | `knee` | [biped.py:78](../src/huphy/robots/biped.py#L78) |
| 관절 ↔ 모터 | 관절 이름 6 | 모터 이름 6 | [leg.py:475](../src/huphy/robots/leg.py#L475) |
| cal ↔ raw | 관절 각도 | 모터 보고 각도 | [base.py:219](../src/huphy/motors/base.py#L219) |
| 이름 ↔ id | 모터 이름 | CAN id | [leg.py:254](../src/huphy/robots/leg.py#L254) |
| 값 ↔ 바이트 | 도, Nm | 8바이트 | [mit.py](../src/huphy/motors/robstride/codec/mit.py) |
| 프레임 ↔ 선 | `CanFrame` | `can.Message` | [canbus.py:358](../src/huphy/motors/canbus.py#L358) |

각 경계가 **한 파일에만** 있음. 도↔라디안은 `mit.py` 와 `policy.py` 두 곳인데,
전자는 프로토콜 경계이고 후자는 모델 경계라 서로 만나지 않음.

팔다리 경계가 맨 위에 있어서 아래 계층은 접두어를 모름. `Leg` 는 `knee` 만 알고,
`right_leg/knee` 는 `Biped` 와 텔레메트리에서만 보임.
