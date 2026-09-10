# 한 주기 — 데이터가 어떤 모양으로 어디를 지나는가

100Hz 한 사이클에서 값이 거치는 모든 변환을 순서대로 적음. 각 단계의 **자료형**,
**단위**, **그 일을 하는 코드**를 함께 둠.

호출 관계 그림은 [flow_diagrams.md](flow_diagrams.md), 계층을 나눈 이유는
[architecture.md](architecture.md).

---

## 0. 한눈에

```
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
| 1 관찰 | `Observation` = `Dict[str, float]` | [leg.py:359](../src/huphy/robots/leg.py#L359) |
| 2 목표 | `Action` = `Dict[str, float]` | [policy.py:194](../src/huphy/control/policy.py#L194) · [motions.py](../src/huphy/control/motions.py) |
| 3 명령 계산 | `Dict[int, MitCommand]` | [leg.py:457](../src/huphy/robots/leg.py#L457) |
| 4 패킹 | `List[CanFrame]` | [bus.py:201](../src/huphy/motors/robstride/bus.py#L201) |
| 5 전송 | `can.Message` | [canbus.py:210](../src/huphy/motors/canbus.py#L210) |
| 6 수거 | `List[CanFrame]` | [canbus.py:240](../src/huphy/motors/canbus.py#L240) |
| 7 해석 | `Dict[int, MotorState]` | [bus.py:218](../src/huphy/motors/robstride/bus.py#L218) |
| 8 기록 | `Dict[str, float]` | [snapshot.py:185](../src/huphy/telemetry/snapshot.py#L185) |
| 9 대기 | — | [loop.py:350](../src/huphy/control/loop.py#L350) |

한 사이클의 시작은 [loop.py:226](../src/huphy/control/loop.py#L226) `ControlLoop.step`.

---

## 1. 관찰 — `Observation`

`Leg.get_observation()` [leg.py:359](../src/huphy/robots/leg.py#L359).
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
[leg.py:246](../src/huphy/robots/leg.py#L246) 가 목록을 냄.

변환 두 가지가 여기서 일어남.

| | 어디서 | 식 |
|---|---|---|
| raw → cal | [base.py:216](../src/huphy/motors/base.py#L216) `raw_to_cal` | `wrap180(sign * raw + offset)` |
| 모터각 → 발목각 | [leg.py:394](../src/huphy/robots/leg.py#L394) `_update_ankle_pose` | 뉴턴 반복 FK |

발목 FK 는 **주기당 한 번만** 풂. `collect`/`refresh` 가 끝날 때 계산해 두고
`get_observation` 은 꺼내 쓰기만 함 — 한 주기에 세 번 불리는데(정책·텔레메트리 두
갈래) 부를 때마다 풀면 같은 계산을 세 번 함.

---

## 2. 목표 — `Action`

`Motion` 계약 [loop.py:142](../src/huphy/control/loop.py#L142):

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
[policy.py:51](../src/huphy/control/policy.py#L51) 가 그 순서를 정함.

순서가 둘임. 발목을 발판 자세로 학습했으면 `JOINT_ORDER`, 모터로 학습했으면
`MOTOR_ORDER` 임 (`ankle_a`/`ankle_b`). `huphy-run --ankle-space` 로 고르고,
`ab` 면 발목이 기구학을 지나가지 않고 그대로 모터로 감.

**자세를 원본 형식으로 안 씀.** 센서가 오일러를 주든 쿼터니언을 주든 벤더 모듈이
중력방향으로 만들어 올리고, 정책은 `imu_state.gravity` 3칸만 봄
([sensors/base.py](../src/huphy/sensors/base.py)).

```
Observation(dict, 도) + ImuState
   |  observation_vector()          policy.py:120
np.float32[24]  또는 [26]           라디안, rad/s
   |  (x - mean) / std              가중치 파일에 들어 있음
   |  model(vector)                 rsl_rl.py
np.float32[6]                       행동 (무차원)
   |  joint_targets()               policy.py:158
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

길이가 `spec.obs_dim` 과 다르면 에러임
[policy.py:150](../src/huphy/control/policy.py#L150) — 순서가 어긋나면 값은 전부
정상인데 로봇만 엉뚱하게 움직여 코드로는 안 잡힘.

**단위가 갈리는 유일한 자리임.** 저장소 전체가 도인데 모델만 라디안을 씀.

---

## 3. 명령 계산 — `Action` → `Dict[int, MitCommand]`

`Leg.build_commands()` [leg.py:457](../src/huphy/robots/leg.py#L457).
**CAN 을 전혀 쓰지 않음** — 순수 계산이라 버스가 둘일 때 전송을 몰아 보낼 수 있음.

```
Action                         관절 이름 -> 도 (cal)
  1  _motor_targets()          leg.py:419   발목만 IK
     -> 모터 이름 -> 도 (cal)
  2  guards.apply()            guards.py:107  NaN · 한계 · 점프
     -> 잘린 도 (cal)
  3  cal_to_raw()              base.py:219
     -> 도 (raw)
  4  MitCommand                bus.py:58
```

### 3-1. 관절 → 모터

```python
{"knee": 30.0, "ankle_pitch": 5.0, "ankle_roll": 2.0}
  -> {"knee": 30.0, "ankle_a": 2.44, "ankle_b": -6.99}
```

발목만 `solve_ik` 를 거쳐 두 모터각으로 풀림. 나머지 4관절은 이름만 바뀜(관절 이름
= 모터 이름).

- 모르는 관절 이름은 **에러**임 — 오타를 무시하면 그 관절만 직전 명령을 유지해 자세가 어긋남.
- 발목은 pitch/roll 을 **함께** 줘야 함. 하나만 오면 에러임.
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
[leg.py:487](../src/huphy/robots/leg.py#L487). 클리핑은 **조용한 변조**라 반드시
세어 내보냄.

**변환보다 검사가 먼저인 이유**: 한계가 cal 공간에 있음. raw 로 내린 뒤 검사하면
`sign = -1` 인 관절에서 부호가 뒤집혀 한계가 반대로 걸림.

### 3-3. `MitCommand`

[bus.py:58](../src/huphy/motors/robstride/bus.py#L58). 다섯 칸이고 각도는 **raw
공간**임.

```python
MitCommand(position_deg=28.4, velocity_deg_s=0.0, kp=20.0, kd=1.0, torque_nm=0.0)
```

튜플이 아니라 이름 있는 필드로 두는 이유: 다섯 개가 전부 `float` 이라 순서를
틀려도 조용히 통과함. `kp` 자리에 위치가 들어가면 모터가 전력으로 튐.

발목을 토크로 보낼 때는 채우는 칸이 다름
[leg.py:508](../src/huphy/robots/leg.py#L508).

| | q | dq | kp | kd | tau_ff | 누가 PD 를 하나 |
|---|---|---|---|---|---|---|
| 위치 | 목표각 | 0 | 게인 | 게인 | 0 | 모터 펌웨어 |
| 토크 | 0 | 0 | 0 | 0 | 야코비안으로 내린 값 | 이 코드 |

---

## 4. 패킹 — `MitCommand` → 8바이트

`send_mit` [bus.py:186](../src/huphy/motors/robstride/bus.py#L186) 가 모터별
인코딩 범위를 붙여 `pack_command`
[mit.py:76](../src/huphy/motors/robstride/codec/mit.py#L76) 를 부름.

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

**범위를 넘으면 조용히 클램프됨** [mit.py:54](../src/huphy/motors/robstride/codec/mit.py#L54).
감싸지(wrap) 않으므로 폭주하지는 않지만 명령한 값과 다른 것이 나감. NaN 은 이
클램프도 통과해 최대값(720도 목표)이 되므로 3-2 에서 걸러야 함.

---

## 5. 전송 — `CanFrame` → 선

```python
CanFrame(can_id=10, data=b"\x85\x54\x7f\xf0\xa3\x33\x37\xff", is_extended=False)
```

[canbus.py:70](../src/huphy/motors/canbus.py#L70). `python-can` 의 `Message` 를
그대로 위로 흘리지 않는 이유: 그러면 `python-can` 없는 환경에서 import 가 깨져 순수
계산 계층의 테스트까지 막힘. 변환은 `send_many` 안에서만 일어남
[canbus.py:225](../src/huphy/motors/canbus.py#L225).

`is_extended=False` — MIT 프로토콜은 11-bit 표준 프레임이고 중재 id 가 곧 대상 모터
CAN id 임. private 프로토콜은 29-bit 확장이라 여기가 갈림.

`send_many` [canbus.py:210](../src/huphy/motors/canbus.py#L210) 가 6프레임을 락
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
뜻을 가짐. 코드표는 [tables.py:145](../src/huphy/motors/robstride/tables.py#L145).

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
[bus.py:258](../src/huphy/motors/robstride/bus.py#L258) 가 이걸 씀 — MIT 모드에는
읽기 전용 명령이 없어서, **움직이지 않는 명령을 보내고 그 응답으로 상태를 받음.**

---

## 7. 수거와 해석 — 선 → `MotorState`

### 7-1. `drain`

[canbus.py:240](../src/huphy/motors/canbus.py#L240).

```python
frames = bus.drain(expect=6, timeout_s=0.002, poll_s=0.0002)
```

`recv(timeout=2ms)` 를 한 번 부르면 큐가 비었을 때 2 ms 를 통째로 버림. 0.2 ms 씩
끊어 폴링하고 총 예산만 2 ms 로 둠. `expect` 개를 채우면 **즉시** 빠져나가므로
정상 주기에는 예산을 안 씀.

`expect` 는 [leg.py:634](../src/huphy/robots/leg.py#L634) 에서
`len(self._awaiting)` — **직전에 명령을 보낸 모터 수**임. 전체를 기다리면 명령하지
않은 모터가 무응답으로 잡힘.

예산을 다 쓰면 `drain_timeouts` 를 올리고 나옴. 예외를 던지지 않음.

받은 프레임에 수신 시각(`time.monotonic()`)을 찍어 둠
[canbus.py:278](../src/huphy/motors/canbus.py#L278).

### 7-2. 응답 프레임 배치

**명령과 배치가 다름** — 응답은 앞에 모터 id 가 붙어 한 칸씩 밀림.

```
Byte0                 모터 CAN ID
Byte1~2               현재각    16bit
Byte3 + Byte4[7:4]    현재속도  12bit
Byte4[3:0] + Byte5    현재토크  12bit
Byte6~7               권선 온도  0.1도 단위
```

인코딩 범위를 고를 때 중재 id 가 아니라 **`data[0]`** 을 봄
[bus.py:248](../src/huphy/motors/robstride/bus.py#L248). 중재 id 는 모델을 알려주지
않는데 한 다리에 RS02 와 RS00 이 섞여 있어 범위가 다름.

```
data = 0A 85 53 80 08 5F 01 3A
     -> decode_state()   mit.py:112
     -> (10, 29.97도, 0.46도/초, 0.79Nm, 31.4도C)
```

온도만 양자화가 아니라 `uint16 / 10` 임.

### 7-3. 캐시 갱신

[bus.py:237](../src/huphy/motors/robstride/bus.py#L237).

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

`Leg._note_link` [leg.py:639](../src/huphy/robots/leg.py#L639) 가 모터별 연속
무응답을 셈.

MIT 모드는 명령을 받으면 **반드시** 상태 프레임으로 답함. 안 오면 그 모터가 명령을
처리하지 않은 것임 — 이것이 애플리케이션 레벨 ack 임. CAN 하드웨어 ACK 는 버스의
아무 노드나 찍어주므로 "누군가 들었다" 일 뿐이고 그건 `tx_errors` 가 봄.

```
tx_errors = 0 인데 ack = 0   ->  모터가 명령을 무시함 (프로토콜·제어모드 불일치)
tx_errors > 0                ->  버스에 아무도 없음 (배선·전원)
```

### 7-5. 발목 FK

수거 직후 `_update_ankle_pose` [leg.py:394](../src/huphy/robots/leg.py#L394) 가
모터각 두 개에서 발목 `(pitch, roll)` 을 풂. 직전 결과를 초기 추정으로 씀 — 같은
모터각 조합이 서로 다른 자세 둘에 대응하므로 추정이 가까워야 함.

못 풀면 `_ankle_pose = None` 이고 추정값은 그대로 둠. 다음 주기가 마지막 성공
지점에서 다시 시작함.

---

## 8. 기록 — `Dict[str, float]` → UDP/CSV

[snapshot.py](../src/huphy/telemetry/snapshot.py) 가 **필드 이름을 정하는 유일한
곳**임. UDP 와 CSV 가 같은 사전을 소비함.

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
[snapshot.py:205](../src/huphy/telemetry/snapshot.py#L205). 명령한 값을 기록하면
한계에 걸린 것과 게인이 낮은 것이 그래프에서 구분되지 않음.

기록 실패는 삼킴 [loop.py:380](../src/huphy/control/loop.py#L380) — 관측이 제어를
멈추면 관측할 대상이 없어짐.

---

## 9. 대기

[loop.py:350](../src/huphy/control/loop.py#L350) `_sleep_until`. 마감은 **절대
시각**(`cycle_start + period_s`)임. 매 주기 남은 시간을 새로 계산하면 오차가 쌓여
서서히 밀림.

```
남은 시간 > 3ms   ->  (남은 시간 - 3ms) 만큼 time.sleep
남은 시간 <= 3ms  ->  자지 않고 돌면서 마감을 봄
남은 시간 <= 0    ->  바로 돌아옴. 따라잡지 않음
```

`time.sleep` 은 요청보다 오래 잠. 마진 3 ms 를 빼 일찍 깨우고 나머지를 스핀으로
메꿈 — 마진이 잠의 오차보다 **작으면** 한 번의 긴 잠이 마감을 지나쳐 스핀 구간이
없어짐. 실측표가 [loop.py:92](../src/huphy/control/loop.py#L92) 에 있음. 실제 스핀은
주기당 약 1 ms 임.

**늦었으면 따라잡지 않음.** 밀린 만큼 다음 주기를 줄이면 그 주기가 더 짧아져 또
밀림.

---

## 10. 시간 예산 — 100Hz, 10 ms

| | 값 | 어디서 정함 |
|---|---|---|
| 프레임 6개 전송 | 약 0.8 ms | 1 Mbps · 8바이트 표준 프레임 |
| 수거 폴링 단위 | 0.2 ms | `DEFAULT_POLL_S` [canbus.py:63](../src/huphy/motors/canbus.py#L63) |
| 수거 총 예산 | 2 ms | `DEFAULT_DRAIN_S` [canbus.py:66](../src/huphy/motors/canbus.py#L66) |
| 스핀 구간 | 약 1 ms | `SPIN_THRESHOLD_S` [loop.py:81](../src/huphy/control/loop.py#L81) |
| 밀림 판정 | 15 ms | `OVERRUN_RATIO` 1.5 [loop.py:75](../src/huphy/control/loop.py#L75) |

수거 예산은 **다 쓰지 않는 것이 정상임.** `expect` 를 채우면 즉시 나옴.

주기를 못 지켰는지는 두 가지로 봄 [loop.py:151](../src/huphy/control/loop.py#L151).

| | 무엇을 잡나 | 못 잡는 것 |
|---|---|---|
| `overruns` | 튀는 주기 | 꾸준히 24%씩 느린 것 |
| `kept_up` | 평균이 목표의 90% 미만 | 어느 주기가 튀었는지 |

`loop_dt` 를 매 주기 내보내는 이유: 느려진 것을 모른 채 게인을 튜닝하면 게인이
아니라 주기가 문제인데 게인을 계속 만지게 됨.

---

## 11. 모드에 따라 갈리는 곳

`ControlLoop.step` [loop.py:226](../src/huphy/control/loop.py#L226).

| | CONTROL | OBSERVE |
|---|---|---|
| 진입 | `robot.enable()` | `robot.disable()` — 토크 차단 |
| 목표 | `motion(t, obs)` | 없음 |
| 전송 | `build_commands` → `send` | `refresh()` 가 `kp=kd=tau=0` 명령을 보냄 |
| 수거 | `collect(expect=명령한 수)` | `collect(expect=전체)` |
| 종료 | `hold` 5주기 → 토크 차단 | 토크 차단 |

**관찰 모드도 통신함.** MIT 에는 읽기 전용 명령이 없어서 아무것도 안 보내면
아무것도 안 옴.

`refresh_states` 는 보내기 전에 `flush_rx` 로 큐를 비움
[bus.py:268](../src/huphy/motors/robstride/bus.py#L268). 직전 주기의 응답이 남아
있으면 이번 것으로 오해함.

종료는 **경로가 하나임** [loop.py:319](../src/huphy/control/loop.py#L319). 정상
종료든 예외든 `finally` 를 지나며 `hold` → 토크 차단 → 텔레메트리 flush 를 탐. 바로
끊으면 서 있는 다리가 주저앉음.

---

## 12. 어느 단계가 무엇으로 실패하나

| 단계 | 실패 | 처리 | 남는 흔적 |
|---|---|---|---|
| 정책 | 관찰 길이 불일치 | 예외 | — |
| 관절→모터 | 모르는 관절 이름 | 예외 | — |
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
| FK | 안 풀림 | 마지막 자세를 냄 | 디버그 로그 |
| 기록 | 어떤 예외든 | 삼킴 | 경고 로그 |

**버리는 것과 자르는 것을 구분함.** 버리면 그 모터만 직전 명령을 유지해 자세가
어긋남. 자르면 `max_delta` 씩 슬루해서 목표에 도달함 — 클리핑이 곧 속도 제한임.

무응답이 몇 주기 이어져도 **루프가 자동으로 멈추지는 않음.** 세어서 내보내기만 하고
판단은 사람이 함.

---

## 13. 공간과 단위가 바뀌는 지점

| 경계 | 왼쪽 | 오른쪽 | 코드 |
|---|---|---|---|
| 모델 ↔ 저장소 | rad, 벡터 | 도, dict | [policy.py](../src/huphy/control/policy.py) |
| 관절 ↔ 모터 | 관절 이름 6 | 모터 이름 6 | [leg.py:419](../src/huphy/robots/leg.py#L419) |
| cal ↔ raw | 관절 각도 | 모터 보고 각도 | [base.py:216](../src/huphy/motors/base.py#L216) |
| 이름 ↔ id | 모터 이름 | CAN id | [leg.py:225](../src/huphy/robots/leg.py#L225) |
| 값 ↔ 바이트 | 도, Nm | 8바이트 | [mit.py](../src/huphy/motors/robstride/codec/mit.py) |
| 프레임 ↔ 선 | `CanFrame` | `can.Message` | [canbus.py:210](../src/huphy/motors/canbus.py#L210) |

각 경계가 **한 파일에만** 있음. 도↔라디안은 `mit.py` 와 `policy.py` 두 곳인데,
전자는 프로토콜 경계이고 후자는 모델 경계라 서로 만나지 않음.
