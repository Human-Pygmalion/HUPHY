# `stand_bi.pt` 실행 파이프라인 감사

**작업일**: 2026-10-07. **대상**: `huphy-run --robot --policy stand --ankle-space ab
--weights config/policies/stand_bi.pt` 가 실제로 도는 전체 경로.

**방법**: 코드를 전부 읽어 데이터가 어디서 와서 어디로 가는지 줄 단위로 따라감.
새로 만든 것은 테스트 코드 두 개(`tests/test_pipeline_audit.py`,
`tests/test_stand_bi_sensitivity.py`)뿐이고, **기존 소스는 한 글자도 고치지
않았음.** 이 문서의 모든 수치·경로는 `grep`/`python3` 로 실제 파일을 읽거나 돌려서
얻은 값이고, 추정은 "추정" 이라고 적음.

관련 문서: 한 주기의 흐름은 [cycle.md](cycle.md), CAN 계층 설계 이력은
[issues.md](issues.md), 정책 실행기 설계는 [policy_runner.md](policy_runner.md).
이 문서는 그것들과 겹치지 않는 부분 — **실제 가중치 파일과 실제 설정 파일로
파이프라인이 끝까지 맞물려 도는지** — 를 확인한 결과임.

---

## 1. 전체 경로 — 호출 순서 그대로

```
huphy-run --robot --policy stand --ankle-space ab --weights config/policies/stand_bi.pt
  │
  ├─ run.py:397        load_robot(config/robot.yaml)
  ├─ run.py:409        _check_standing()                 조합 검증 (2.1절)
  ├─ run.py:447        rsl_rl.load(stand_bi.pt, spec=stand.SPEC, action_dim=12)
  │                      └─ obs_dim/action_dim 을 가중치 파일에서 읽어 대조 (3절)
  ├─ run.py:467        build_biped(robot, limbs)          다리 둘 + IMU 조립 (6절)
  ├─ run.py:541        stand.motion(model, leg.imus[0])   정책을 Motion 으로
  ├─ run.py:545        staged(접근, 대기, 정책)            영점 -> 대기 -> Enter -> 정책
  └─ run.py:556        loop.run(motion)                   ControlLoop (7절)
         │
         └─ 매 주기 (ControlLoop.step, loop.py:295)
              1. robot.get_observation()                  모터 상태 읽기 (cal 공간)
              2. motion(t, observation)                    아래로 ↓
                   stand.motion.step (stand.py:255)
                     a. imu.read()                         IMU 최신값 (2절)
                     b. stand.observation_vector()          45칸 벡터 조립 (4절)
                     c. model(vector)                       정규화 + 4층 MLP (3절)
                     d. stand.joint_targets()               행동 -> 목표각(도), 자름
              3. robot.send(robot.build_commands(action))  관절 -> 모터 -> MitCommand (5절)
                   Leg.build_commands (leg.py:529)
                     - guards.apply()                       한계·점프 cal 공간에서 자름
                     - cal_to_raw()                          sign/offset 적용 (5.2절)
                     - MitCommand(position_deg, kp, kd)       게인은 robot.yaml 에서 (5.1절)
              4. RobStrideBus.send_mit -> CanBus.send_many   CAN 프레임 송신 (8절)
              5. robot.collect()                            응답 수거, 상태 갱신
              6. telemetry.record()                          **호출되지 않음** (9절 — 문제)
```

---

## 2. IMU 경로

### 2.1 물리 계층 — `sensors/ebimu/imu.py`

- 시리얼 포트를 **백그라운드 스레드**가 계속 읽음(`EbimuImu._read_loop`,
  [imu.py:216-242](../src/huphy/sensors/ebimu/imu.py#L216)). 제어 루프는 그
  스레드가 마지막으로 채워 둔 값을 `read()`로 꺼내기만 함 — 통신 대기가 제어
  주기에 안 끼어듦.
- `connect()`가 핸드셰이크로 **필드 개수**를 확인함([imu.py:156-191](../src/huphy/sensors/ebimu/imu.py#L156))
  — 센서에 저장된 `output` 설정과 코드의 `output` 인자 개수가 다르면 연결
  자체를 거부함. **순서가 다른 경우는 못 잡음** — 그건 `huphy-imu show`가 사람이
  보고 대조해야 함(코드 주석이 이미 이렇게 밝힘).
- `read()`는 한 번도 못 받았으면 `ImuState()`(기본값, `is_valid=False`)를 내고,
  **스레드가 죽은 뒤에도 예외 없이 마지막 값을 계속 냄** — `age_ms()`만 계속
  자람. 스레드가 죽었는지 자체를 감시하는 코드는 없음(재연결 로직 없음).

### 2.2 제어 경로가 이 값을 쓰는 방식 — `control/stand.py`

```python
out.extend(math.radians(v) for v in imu_state.gyro_dps)   # stand.py:194
out.extend(imu_state.gravity)                              # stand.py:197
```

**`imu_state.is_valid`도 `age_ms()`도 검사하지 않음.** 센서가 끊겨도, 통신이
끊긴 지 10분이 지났어도, 여기 들어오는 값은 똑같이 "유효한 관찰"로 신경망에
들어감. 이건 설계 의도("관측이 제어를 막으면 안 됨", `sensors/group.py:15-21`
독스트링)와 일관되긴 하지만, **그 의도가 "넘어지는 중에도 로봇을 세우지 말 것"과
"IMU 가 죽은 채로 계속 서 있으라고 명령할 것" 둘 다를 뜻하는지는 코드에 안
적혀 있음.** 9.2절에서 별도 문제로 적음.

### 2.3 데이터 흐름 요약

```
EBIMU(시리얼, 100Hz) -> protocol.decode() -> ImuState(gravity, gyro_dps, stamp)
   -> EbimuImu._latest (스레드가 덮어씀, 락으로 보호)
   -> imu.read()  (정책 호출 시점, 통신 없음)
   -> stand.observation_vector()[0:6]  (각속도 rad/s 변환, 중력은 그대로)
   -> 정규화 -> 신경망 입력
```

**검증한 것** (`tests/test_pipeline_audit.py::TestImuWiring`):
`is_valid=False`인 `ImuState`를 넣어도 `observation_vector`가 예외 없이 45칸을
내는 것, 그리고 `ControlLoop._check_link`(통신 두절 판정)가 모터 응답만 보고
IMU 는 전혀 언급하지 않는 것을 코드로 고정해 재현함. 둘 다 **통과함** — 즉
"IMU 유효성 검사 없음"은 버그가 아니라 지금 코드가 실제로 그렇게 동작한다는
사실 확인임.

---

## 3. 가중치 파일 읽기 — `control/rsl_rl.py`

`.pt`는 zip이고 `torch` 없이 numpy만으로 읽음([rsl_rl.py:102-141](../src/huphy/control/rsl_rl.py#L102)).
직접 돌려서 확인함:

```
$ PYTHONPATH=src python3 -c "
from huphy.control import stand, rsl_rl
model = rsl_rl.load('config/policies/stand_bi.pt', spec=stand.SPEC, action_dim=12)
print(model.obs_dim, model.action_dim)"
45 12
```

`obs_dim`/`action_dim`을 가중치 파일 안의 텐서 크기에서 직접 읽어 `stand.SPEC`
(45)과 `len(stand.ORDER)`(12)에 **일치하는지 대조한 뒤**에야 통과함
([rsl_rl.py:166-178](../src/huphy/control/rsl_rl.py#L166)). 어긋나면 모터를 켜기
전에 `SystemExit`.

정규화는 가중치 파일에 저장된 `obs_normalizer._mean`/`._std`로 하고, 표준편차에
`NORM_EPS`(1e-2)를 더함 — 자리 채우기가 아니라 학습이 실제로 쓴 값이라고 코드
주석이 명시함([rsl_rl.py:53-68](../src/huphy/control/rsl_rl.py#L53)).

**지금까지 빠져 있던 것**: `tests/test_rsl_rl.py`의 `WEIGHTS` 표에는
`balance`/`hopping`만 있고 **`stand_bi.pt`는 없었음** — 읽히는지, 규격이
맞는지, 입력을 바꾸면 출력이 바뀌는지 전부 테스트가 없었음. 9.3절에서 문제로
적고, `tests/test_stand_bi_sensitivity.py`로 메움.

---

## 4. 관찰 45칸 — 정확한 배치

`control/stand.py:177-218`에서 조립함. 전부 실측 확인함(`tests/test_stand.py`,
기존 테스트):

| 칸 | 내용 | 단위 | 출처 |
|---|---|---|---|
| 0-2 | 각속도 (x,y,z) | rad/s | IMU, `math.radians(gyro_dps)` |
| 3-5 | 중력방향 | 단위벡터 | IMU, 그대로 |
| 6-17 | `q(t-1)` − 기준자세, `ORDER` 순서 12개 | rad | 이전 정책 호출 때의 관절각 |
| 18-29 | `q(t)` − 기준자세, `ORDER` 순서 12개 | rad | 이번 호출의 관절각 |
| 30-41 | 직전 행동(모델 출력 그대로, 목표각 아님) | 무차원 | `last_action` |
| 42-44 | 속도 명령 (vx, vy, wz) | — | 항상 `(0,0,0)` — 제자리 서기 전용 |

`ORDER`는 **오른다리 6개 먼저, 왼다리 6개**([stand.py:61-79](../src/huphy/control/stand.py#L61))
— `policy.BIPED_LEGS`(왼다리 먼저)와 **정반대**임. 두 정책이 서로 다른 학습
산출물이라 상수를 공유하면 안 된다고 코드 주석이 명시함.

`DEFAULT_POSE_DEG`가 기준자세이고 0이 아님(오른쪽 엉덩이 −10.03도, 무릎
20.05도 등). **관절각 관찰 칸은 이 값을 뺀 상대값**이지 원시 각도가 아님 —
이 뺄셈이 안 되면(예: 기준자세를 0으로 착각) 정책이 학습 때 본 적 없는 큰
입력을 받음.

### 4.1 실제 가중치로 민감도 확인 (`tests/test_stand_bi_sensitivity.py`)

직접 실행한 결과(소스는 아래 "테스트 코드"절):

```
$ PYTHONPATH=src python3 -m pytest tests/test_stand_bi_sensitivity.py -q
...
9 passed
```

구체적 수치(손으로 한 번 더 돌려 확인):

```
baseline 출력          [-0.0069  0.0671  0.0092 -0.1122 -0.7685  0.5993  0.0304 -0.0318  0.0366  0.0458  0.7236 -0.8464]
각속도만 30deg/s 교란   delta norm = 0.523
중력방향만 기울임       delta norm = 0.588
무릎 각도만 +10도       delta norm = 4.241
```

**세 경로 다 출력이 실제로 바뀜** — 관찰 칸이 신경망 입력까지 끊김 없이
이어진다는 뜻임. 무릎 각도 교란이 가장 크게 움직이는 것도 타당함(위치 피드백이
이력 두 프레임에 걸쳐 두 번 들어가므로).

---

## 5. 행동 → 모터 명령

### 5.1 게인(kp/kd) — robot.yaml에서 CAN 프레임까지

```
config/robot.yaml  Motor(gains=Gains(kp, kd))
  -> Leg.config.motors[name].gains          (robots/leg.py, 설정 그대로 보관)
  -> Leg.build_commands()                    leg.py:571-572
       MitCommand(..., kp=motor.gains.kp, kd=motor.gains.kd)
  -> RobStrideBus.send_mit()                 bus.py:214-227
       mit.pack_command(..., kp=c.kp, kd=c.kd, enc=...)
  -> CAN 프레임 byte3[3:0]+byte4(kp), byte5+byte6[7:4](kd)
```

`stand` 정책은 **게인을 덮어쓰지 않음** — `run.py:458-465`가
`if not standing:` 일 때만 `gains=Gains(POLICY_KP, POLICY_KD)`를 `build_leg`에
넘김. `stand`는 그 분기를 안 타므로 `robot.yaml`의 값이 그대로 씀. 이건
의도된 설계임: `run.py:66-77` 주석이 "1.0은 robot.yaml이 곧 학습이 가정한
사양값"이라고 명시함.

**실측(현재 `config/robot.yaml`, 심볼릭 링크가 가리키는 파일)**:

```
hip_pitch / hip_roll / hip_yaw   kp 150.0   kd 6.0
knee                             kp 220.0   kd 6.0
ankle_a / ankle_b                kp 22.266  kd 1.414
```

`tests/test_pipeline_audit.py::TestGainPropagation`이 **이 값이 `build_commands()`가
내는 `MitCommand.kp/kd`와 정확히 같은지** 모터 12개(양다리) 전부 대조함 —
실제 `robot.yaml`을 `load_robot()`으로 읽고 실제 `build_leg()`를 불러서 확인함
(CAN 전송 계층만 가짜로 바꿈). **통과함** — 설정값이 중간에 상수로 덮이거나
누락되지 않고 그대로 프레임 직전까지 옴.

`TestGainOverride`는 `balance`/`hopping` 경로(`gains=Gains(20.0, 0.502)`로
덮어쓰는 경우)도 같은 방식으로 확인함 — **12모터 전부**가 덮어써지는지, 그리고
덮어쓰지 않을 때(`stand`) 원래 값이 남는지 둘 다 통과함.

### 5.2 부호(sign)/오프셋 — cal ↔ raw

```python
# motors/base.py
raw_to_cal = wrap180(sign * raw + offset_deg)
cal_to_raw = wrap180((cal - offset_deg) / sign)
```

**실측(`config/calibration/{left,right}_leg_v1.0.json`)**: 12모터 전부
`sign: 1.0, offset_deg: 0.0`. 파일 자체의 `note` 필드가 "1.0 하드웨어용이고
아직 아무것도 재기 전임"이라고 적어 둠 — 즉 지금 이 값은 **실측이 아니라
자리 채우기(항등 변환)**임.

`tests/test_pipeline_audit.py::TestSignAndOffset`가 이 가정(`sign==1.0`,
`offset==0.0`)을 코드로 고정해 재현하고, 그 조건에서 `cal_to_raw`가 항등
변환임을 확인함 — **통과함.** 단, 이 테스트가 통과한다는 것은 "지금 코드가
이렇게 동작한다"는 뜻이지 "실측이 끝났다"는 뜻이 아님. 9.4절에서 운영상
문제로 별도로 적음.

---

## 6. IMU·다리 조립 — `build_biped`

`--robot`이면 `build_biped`가 다리 둘을 만들고, **IMU는 다리가 아니라
로봇(Biped) 전체에 붙음**([bringup.py:151-188](../src/huphy/scripts/bringup.py#L151),
`imus=[]`를 `build_leg`에 넘겨 다리에는 안 붙임). `run.py:473`이
`leg.imus[0]`로 꺼내 쓰는데, 이건 **"이 로봇에 붙은 IMU 목록의 첫 번째"**라는
뜻 — IMU가 둘 이상이면 설정의 `imus` 딕셔너리 삽입 순서로 첫 번째가 정해짐
(`config/robot.yaml`은 지금 `main` 하나뿐이라 모호함이 없음).

`reader=True`가 `build_biped`에서 기본으로 켜져 두 CAN 채널이 동시에
수신됨(이미 이번 대화에서 `docs/cycle.md:398-417`에 세부 메커니즘을 적어
둠 — 중복 설명 생략).

---

## 7. 제어 루프 — 정책 주기와 전송 주기

`stand`는 `POLICY_HZ=50`로 고정([run.py:132-139](../src/huphy/scripts/run.py#L132)).
전송 주기(`--hz`)를 그보다 올리면 `held()`([loop.py:166-199](../src/huphy/control/loop.py#L166))가
정책 계산은 50Hz로만 하고 그 사이 주기는 직전 목표를 그대로 반복 전송함 —
보간 없음. `outer_hz`가 `inner_hz`의 정수배가 아니면 시작 전에 거부함.

`--hz`를 기본값(50)대로 두면 `held()`는 아예 안 씌워짐(`run.py:551-552`:
`if args.hz != POLICY_HZ:`) — 전송도 50Hz.

---

## 8. CAN 프레임 구조 — 바이트 단위

### 8.1 명령 (Command 3, MIT Dynamic Parameters)

```
Byte0-1               목표각    16bit   (−pmax ~ pmax rad)
Byte2 + Byte3[7:4]     목표속도  12bit
Byte3[3:0] + Byte4     Kp        12bit
Byte5 + Byte6[7:4]     Kd        12bit
Byte6[3:0] + Byte7     목표토크  12bit
```
([codec/mit.py:107-125](../src/huphy/motors/robstride/codec/mit.py#L107), `pack_command`)

`stand`는 `ankle_output=position`(기본값)만 쓰므로 토크 칸(Byte6[3:0]+Byte7)은
항상 0 — `ankle_output=torque`를 쓰는 `--ankle-output torque`와는 다른 경로임
(`stand`는 그 조합을 `_check_standing`에서 시작 전에 거부함, [run.py:361-365](../src/huphy/scripts/run.py#L361)).

### 8.2 응답 (Command 1 응답, Data Feedback)

```
Byte0                  모터 CAN ID
Byte1-2                현재각     16bit
Byte3 + Byte4[7:4]     현재속도   12bit
Byte4[3:0] + Byte5     현재토크   12bit
Byte6[7:6]             동작 모드  (0 Reset / 1 Cali / 2 Motor)
Byte6[5]               고장 있음
Byte6[4]               경고 있음
Byte6[3:0] + Byte7     권선 온도  12bit (0.1도 단위)
```
([codec/mit.py:124-136](../src/huphy/motors/robstride/codec/mit.py#L124))

**명령과 응답의 바이트 배치가 다름** — 응답은 모터 ID가 맨 앞에 붙어 한 칸씩
밀림. 이 모터 ID는 **CAN arbitration ID가 아니라 데이터 payload의 첫 바이트**임
— arbitration ID 자체는 모터와 무관하게 **호스트 ID 하나로 고정**됨
(`robstride/README.md:282`). 이 구조가 이번 대화에서 candump로 실측 확인한
CAN 버스 경합의 근본 원인임 — 상세는 9.5절.

### 8.3 인코딩 범위는 모델마다 다름

```
RS00  각도 ±12.57rad  속도 ±33    Kp 0~500    Kd 0~5     토크 ±14 N·m
RS02  각도 ±12.57rad  속도 ±44    Kp 0~500    Kd 0~5     토크 ±17 N·m
RS03  각도 ±12.57rad  속도 ±20    Kp 0~5000   Kd 0~100   토크 ±60 N·m
RS04  각도 ±12.57rad  속도 ±15    Kp 0~5000   Kd 0~100   토크 ±120 N·m
```
([tables.py:101-117](../src/huphy/motors/robstride/tables.py#L101))

**이 표가 틀리면 kp/kd가 조용히 배율만큼 어긋남** — 5.1절에서 확인한
`kp=150/220/22.266`은 각 모터의 `model` 필드(`robot.yaml`의 `RS04`/`RS03`)로
이 표에서 올바른 범위를 골라 양자화됨. `TestGainPropagation`이 대조한 값은
**양자화 이후 디코딩값이 아니라 `MitCommand`에 실리는 부동소수 그대로**임 —
바이트 단위 양자화 왕복 자체는 `tests/test_codec.py`가 이미 덮음(`kp=500`,
`kp=5000` 최대값 왕복 확인).

---

## 9. 발견된 문제

번호를 매겨 각각 독립적으로 적음. 전부 **실제로 돌려서(테스트 실행, 실제
코드 읽기, candump 실측) 확인한 것**이고, 추정인 항목은 "추정"이라고 명시함.

### 9.1 — `huphy-run`이 텔레메트리를 전혀 안 씀

**위치**: [run.py:498](../src/huphy/scripts/run.py#L498)
```python
loop = ControlLoop(leg, hz=args.hz, mode=Mode.CONTROL)
```
`telemetry=` 인자가 없음. `huphy-bringup`([bringup.py:668-674](../src/huphy/scripts/bringup.py#L668))만
`tele.Telemetry.from_config(...)`를 만들어 넘김. 결과: **정책을 실제로 돌릴
때 IMU 값·관절 오차·게인·CAN 카운터 중 아무것도 UDP나 CSV로 안 나감.**
`control/stand.py`가 신경망에 무엇을 넣고 있는지, 실행 중에는 로그로 확인할
방법이 없음 — `print`로 찍힌 시작 화면 외에는 아무 기록도 안 남음.

`huphy-test`([scripts/selftest.py:746](../src/huphy/scripts/selftest.py#L746))도
`ControlLoop(machine, hz=hz, mode=Mode.OBSERVE)`로 같은 문제를 가짐(이번
대화 중 이미 확인·보고함).

**영향**: 정책이 넘어지거나 이상하게 움직였을 때, 그 순간 IMU가 뭘 봤는지,
관절 목표가 뭐였는지 사후 분석이 불가능함. `failures.report()`(종료 시
1회성 요약)만 남음.

### 9.2 — IMU 유효성 검사가 제어 경로 어디에도 없음

**위치**: [control/stand.py:193-197](../src/huphy/control/stand.py#L193),
[control/policy.py:230-235](../src/huphy/control/policy.py#L230),
[control/loop.py:318-342](../src/huphy/control/loop.py#L318)

`observation_vector`들은 `imu_state.is_valid`/`age_ms()`를 안 봄.
`ControlLoop._check_link`(통신 두절 판정)는 `robot.link_status`만 보고,
그건 **모터 응답**을 보는 것이지 IMU와 무관함(코드에 `imu` 문자열 자체가
없음 — `test_pipeline_audit.py::test_loop_link_check_does_not_cover_imu`로
고정).

**결과**: EBIMU 시리얼이 뽑히거나 스레드가 죽어도, 로봇은 멈추지 않고
**마지막으로 받은 자세를 영원히 "지금 자세"라고 믿은 채 서기 정책을 계속
돌림.** 실제로 기울어지고 있어도 신경망은 "안 넘어지고 있다"는 입력을 계속
받음 — 정책이 복구 동작을 할 기회 자체가 없어짐.

**추정**: 설계 의도(`sensors/group.py:15-21`)는 "센서 고장이 로봇을 못
움직이게 하면 안 됨"이었던 것으로 보이나, 그 의도가 "IMU 없이도 서기
정책을 계속 믿고 돌려라"까지 포함하는지는 코드에 명시돼 있지 않음 — 확인
필요.

### 9.3 — `stand_bi.pt`가 회귀 테스트 대상에서 빠져 있었음

**위치**: [tests/test_rsl_rl.py:16-19](../tests/test_rsl_rl.py#L16)의 `WEIGHTS`
표에 `balance`/`hopping`만 있고 `stand`가 없음.

**결과**: 실제 배포 가중치(`config/policies/stand_bi.pt`)가 올바른 모양으로
읽히는지, 결정적인지, 입력 변화에 반응하는지를 지금까지 **한 번도 자동으로
확인한 적이 없었음.** `tests/test_stand.py`는 `observation_vector`/
`joint_targets` 같은 **어댑터 코드**만 가짜 모델로 확인했고, 진짜 가중치
파일을 로드해서 돌려보는 테스트가 없었음.

**조치**: 이번에 `tests/test_stand_bi_sensitivity.py`로 메움(9개 테스트,
전부 통과 — 4절 참조).

### 9.4 — 1.0 캘리브레이션: 한계각은 실측됐고, sign/offset 은 아직 자리 채우기

**위치**: `config/calibration/{left,right}_leg_v1.0.json`

실제 파일 내용을 직접 읽어 대조함:

```
hip_pitch  sign 1.0  offset 0.0  zero_reference ''  limits [-17.75, 112.75]
hip_roll   sign 1.0  offset 0.0  zero_reference ''  limits [-79.5, 19.5]
hip_yaw    sign 1.0  offset 0.0  zero_reference ''  limits [-40.5, 40.5]
knee       sign 1.0  offset 0.0  zero_reference ''  limits [-114.0, -6.0]
ankle_a    sign 1.0  offset 0.0  zero_reference ''  limits [-61.88, 61.88]
ankle_b    sign 1.0  offset 0.0  zero_reference ''  limits [-61.88, 61.88]
```

**한계각(`limits_deg`)은 12모터 전부 실측값이 채워져 있음** — 전부 `null`이
아님. 그런데 **`sign`은 전부 1.0, `offset_deg`는 전부 0.0, `zero_reference`는
전부 빈 문자열임** — 이건 "쟀는데 우연히 1.0/0.0"이 아니라 **영점을 아직
한 번도 잡지 않은 상태**라는 뜻임(`zero_reference`가 비어 있다는 것 자체가
`commission zero`를 아직 안 돌렸다는 증거 — `MotorCalibration.zero_reference`의
독스트링이 "이 메모가 없으면 나중에 영점을 재현할 수 없음"이라고 명시함,
[base.py:187-194](../src/huphy/motors/base.py#L187)).

**즉 캘리브레이션은 절반만 끝남**: `commission sweep`(한계각 측정)은
돌았고, `commission zero`(영점 잡기)는 아직임.

**영향**: `stand.DEFAULT_POSE_DEG`(오른쪽 엉덩이 −10.03도, 무릎 20.05도 등)는
cal 공간 값 — "모터가 보고하는 0도 = 조립 기계 영점"이라는 전제 위에서만
의미가 있음. 영점을 아직 안 잡았다면 `offset_deg=0.0`은 실제 기계 영점과
무관한 임시값이고, 그만큼 `DEFAULT_POSE_DEG`가 가리키는 **물리적 다리 자세**가
학습 때의 자세와 어긋나 있을 수 있음 — 정책은 다른 위치에서 "기준자세에
있다"고 착각한 채 시작함.

**교차 확인**: 이 상태는 테스트 스위트 전체 실행에서도 드러남 —
`tests/test_config.py::TestRealFile::test_limits_live_in_the_calibration_file`가
**지금 깨져 있음(이번 작업으로 깨진 것이 아니라 기존 상태).** 그 테스트는
"한계각이 전부 `null`(안 쟀음)"을 전제로 작성됐고, 테스트 자신의 주석이
"이 테스트가 깨지면 실측이 들어왔다는 뜻임 -- 그때 기대값을 뒤집을 것"이라고
미리 적어 둔 상황이 정확히 지금 벌어진 것임([test_config.py:105-119](../tests/test_config.py#L105)).
**한계각 실측은 들어왔는데, 그 사실을 반영해 테스트를 갱신하는 절차가 아직
안 된 상태**로 보임 — 코드 수정 금지 지침에 따라 이 문서에서는 고치지 않고
사실만 적음.

**권고**: 정책을 실물에 돌리기 전에 1.0 영점 커미셔닝(`huphy-commission zero`)이
끝났는지 확인 절차로 넣을 것. 끝났다면 `zero_reference`가 채워지고 `offset_deg`도
실측값으로 바뀌어야 하므로, 그 변화 자체가 "영점이 잡혔다"는 신호가 됨.

### 9.5 — CAN 버스: 같은 주기 안에서 모터 응답이 통째로 빠지는 현상 (실측, 미해결)

**출처**: 이번 세션 중 `candump -tz -x`로 `can0`/`can1` 양쪽을 실측.

**관측**: 6모터(한 다리)에 명령을 보낸 뒤, **일부 모터의 응답이 다음 명령
주기(10~12ms)가 시작될 때까지 한 번도 나타나지 않음.** 어느 모터가 빠지는지는
**주기마다 다름**(특정 모터가 고정으로 빠지는 패턴이 아님) — 배선 하나가
나쁜 것이라기보다 버스 전체에 걸리는 전기적 요인(터미네이션·노이즈·전원)을
가리킴.

**근본 원인 후보(코드로 확인된 사실)**: 8.2절대로 모든 모터의 응답이 **같은
CAN arbitration ID(호스트 ID)**를 씀. 두 모터가 동시에 응답을 시도하면
데이터필드(byte0=모터 ID)로 비트 중재가 넘어가는데, 실측 결과 이 중재가
"작은 모터 ID가 항상 이긴다"는 예측과도 어긋났음(예: `knee`(0x0A)가
`ankle_a`(0x0B)·`ankle_b`(0x0C)보다 수치상 작은데도 실제로는 더 늦게 응답한
사례를 확인함) — **즉 지금 가진 가설(중재 우선순위)만으로는 실측 패턴이
전부 설명되지 않음.**

**대역폭은 원인이 아님**: 1Mbps에서 프레임당 ~130비트로 계산하면 한 다리
(명령 6 + 응답 6 = 12프레임)는 ~1.56ms — 100Hz(10ms) 예산의 15.6%, 200Hz
(5ms)에서도 31.2%뿐이라 **여유가 충분함.** 실측된 수 ms급 지연/결측은
순수 전송 시간으로는 설명 안 됨.

**미해결 — 다음에 볼 것** (이 문서가 아니라 운영 중 확인이 필요함):
1. `ip -details -statistics link show can0/can1`의 `bus-error`/`error-passive`/
   `bus-off`/`restarts` — 이 저장소 어디서도 안 읽는 커널 레벨 통계. 전기적
   문제의 직접 증거가 됨.
2. 같은 모터가 반복해서 빠지는지(배선 문제) 매번 다른 모터가 빠지는지
   (버스 전체 문제)를 더 긴 캡처로 통계화.
3. `docs/issues.md`의 **이슈 #10 "남은 것"**이 "실물 양다리에서 주기 예산을
   재 보는 것 -- 아직 확인 못 함"이라고 이미 적어 둔 항목과 같은 작업임 —
   이번 실측이 그 작업의 **일부**(현상 확인)이고, 원인 규명은 아직 안 끝남.

### 9.6 — (참고, 이번 파이프라인과 무관) 전체 테스트 스위트에 기존 실패 2건

전체 테스트(`PYTHONPATH=src python3 -m pytest tests -q`, 1261개)를 돌려
**이번에 추가한 파일이 기존 테스트를 깨지 않았는지** 확인하는 과정에서,
이번 작업과 무관하게 **이미 실패하고 있던 테스트 2개**를 발견함(내가 만든
파일이 원인이 아님 — 둘 다 내가 건드리지 않은 파일):

1. `tests/test_config.py::TestRealFile::test_limits_live_in_the_calibration_file`
   — 9.4절에서 다룬 것과 같은 사안(한계각 실측이 들어왔는데 테스트 기대값이
   옛날 그대로).
2. `tests/test_run.py::TestRun::test_it_shows_what_it_will_do` — 화면 출력에서
   `"kp=20.0"`을 찾는데 실제 출력은 `"kp=20"`임(`bringup.py`의 `_gain_line`이
   `f"kp={kp:g}"` 포맷을 쓰므로 `20.0`이 `20`으로 찍힘). 게인 값 자체는
   맞게 찍히고 있음 — 표시 자릿수와 테스트의 문자열 기대값만 어긋난 것으로
   보임(추정 — `:g` 포맷 규칙상 정수값은 소수점이 빠짐).

**둘 다 코드를 고치지 말라는 이번 지침에 따라 그대로 둠.** 사실만 보고함 —
실제 동작(게인 전달, 한계각 측정)에는 문제가 없고, 테스트의 기대값이 최근의
실측/코드 변화를 안 따라간 것으로 보임.

```
2 failed, 1259 passed in 300.65s
```

---

## 10. 테스트 코드

### 10.1 `tests/test_pipeline_audit.py`

실제 `config/robot.yaml`과 실제 `config/calibration/*_v1.0.json`을
`load_robot()`/`build_leg()`로 그대로 읽어 돈다. CAN 전송 계층만
`tests/test_leg.py`와 같은 방식(`FakeBus`)으로 바꿈 — 그 위는 전부 실제 코드.

```
TestGainPropagation        robot.yaml 의 kp/kd 가 MitCommand 까지 그대로 오는가
TestGainOverride           huphy-run 의 Gains() 덮어쓰기가 설계대로 전체 모터에 걸리는가
TestSignAndOffset          cal -> raw 가 지금 캘리브레이션(sign=1, offset=0)에서 항등인가
TestImuWiring              IMU 무효/미검사 동작을 코드로 고정 재현
```

### 10.2 `tests/test_stand_bi_sensitivity.py`

`config/policies/stand_bi.pt`를 실제로 읽어, 입력(IMU 각속도/중력방향,
관절각 현재값/이력)을 각각 바꿨을 때 **실제 신경망 출력이 변하는지**를
확인함. 가짜 모델이 아니라 진짜 가중치로 돈다는 점이 `tests/test_stand.py`
(어댑터만 확인, 가짜 모델 사용)와 다름.

### 10.3 실행 결과

```
$ PYTHONPATH=src python3 -m pytest tests/test_pipeline_audit.py tests/test_stand_bi_sensitivity.py -q
..................
18 passed in 0.24s
```

### 10.4 실행 방법

```bash
PYTHONPATH=src python3 -m pytest tests/test_pipeline_audit.py -v
PYTHONPATH=src python3 -m pytest tests/test_stand_bi_sensitivity.py -v
```

하드웨어도 `torch`도 필요 없음 — `config/`의 실제 파일과 numpy만 씀.

---

## 11. 검증 범위 밖 — 확인하지 못한 것

- **시뮬 대조**: 학습 쪽(mjlab)이 낸 참조 입출력과 이 저장소의 numpy 재구현이
  수치까지 일치하는지는 `balance`/`hopping`만 `pyg_*_caps` 꾸러미로 확인된
  이력이 있고([rsl_rl.py:61](../src/huphy/control/rsl_rl.py#L61)),
  **`stand_bi.pt`는 그 대조가 이 저장소에 없음.** 이번 감사도 "읽히고,
  입력에 반응한다"까지만 확인했지 "학습 쪽과 같은 숫자를 낸다"는 확인하지
  못함 — 그 참조 데이터가 저장소 밖에 있어서임(`docs/policy_runner.md`가
  이미 같은 한계를 적어 둠).
- **실물 로봇에서의 동작**: 전부 하드웨어 없이(가짜 CAN, 실제 가중치)
  확인함. 실제 모터·IMU·배선에서 이 경로가 끝까지 도는지는 이 문서의
  범위 밖임.
- **9.5절 CAN 문제의 근본 원인**: 현상은 실측했으나 전기적 원인인지 다른
  이유인지는 미확정.
