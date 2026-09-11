# 최종 정책 실행기 — 설정 파일 하나로 바로 도는 것 (설계 검토)

**아직 만들지 않은 것의 설계**임. 최종 단계에서 학습 결과(가중치 + 설정 파일)만
넣으면 인자 없이 바로 동작하는 실행기를 만들 때, 무엇을 어떻게 옮기고 무엇을 새로
만들어야 하는지 정리함.

지금 쓰는 `huphy-run` 은 그대로 둠. 새 실행기는 명령어가 다름.

관련 문서: 정책 실행의 현재 상태는 [`control/POLICY.md`](../src/huphy/control/POLICY.md),
양다리 구성은 [full_robot.md](full_robot.md), 한 주기의 흐름은 [cycle.md](cycle.md).

---

## 0. 한눈에

| | 지금 (`huphy-run`) | 최종 실행기 |
|---|---|---|
| 용도 | 개발·검증. 인자로 이것저것 바꿔 봄 | 학습 결과를 그대로 돌림 |
| 모델 규격 | 파이썬 코드에 적혀 있음 | 가중치 옆 설정 파일 |
| 발목 공간 | `--ankle-space` 로 사람이 고름 | 설정 파일이 정함 |
| 발목 출력 | `--ankle-output` 로 사람이 고름 | 설정 파일이 정함 |
| 게인·주기 | `run.py` 상수 | 설정 파일 |
| 다리 하나 / 양다리 | `--limb` / `--robot` | 설정 파일의 관절 이름으로 판별 |
| 새 모델 추가 | 코드 수정 | 파일 두 개 넣음 |

---

## 1. 왜 따로 만드나

`huphy-run` 은 **실험 도구**임. 같은 가중치를 발목 공간·출력 모드·주기를 바꿔 가며
돌려 보는 것이 목적이라, 그 값들이 인자로 열려 있음.

최종 단계는 반대임. 학습 쪽이 정한 값에서 **벗어나면 안 됨.** 인자가 열려 있으면
잘못 칠 수 있고, 잘못 쳐도 코드가 못 잡는 조합이 있음 (3.4 참조).

그래서 역할을 나눔.

| | `huphy-run` | 최종 실행기 |
|---|---|---|
| 값의 출처 | 사람 (인자) | 학습 (설정 파일) |
| 인자로 덮어쓰기 | 됨 | 모델에 딸린 값은 안 됨 (7장) |
| 남겨 두는 이유 | 튜닝·실험·고장 진단 | — |

두 실행기가 **같은 조립·제어 코드**를 씀 (10장). 다른 것은 값을 어디서 가져오느냐뿐임.

---

## 2. 지금 `huphy-run` 이 하는 일

### 2.1 명령

```bash
huphy-run --limb right_leg --policy balance                          # 다리 하나, 발목 rp, 위치
huphy-run --limb right_leg --policy balance --ankle-output torque    # 발목 토크
huphy-run --limb right_leg --policy balance --ankle-space ab         # 발목 모터 공간
huphy-run --robot --policy balance --weights runs/biped.pt           # 양다리 12칸
```

### 2.2 흐름

`src/huphy/scripts/run.py` 의 `main()` (284줄) 순서임.

| 단계 | 하는 일 | 자리 |
|---|---|---|
| 1 | `robot.yaml` 읽기 | `run.py:284` 이하 |
| 2 | 규격 고르기 — `--policy` 이름으로 | `run.py:100` `SPECS` |
| 3 | 조합 거부 — `ab` + `torque` | `run.py:312` |
| 4 | 관절 순서 고르기 — 다리 하나면 `ORDERS`, 양다리면 `BIPED_ORDERS` | `policy.py:101`, `141` |
| 5 | 양다리면 관찰 길이 다시 계산 | `policy.py:185` `for_joints` |
| 6 | 가중치 읽기 + 입력·출력 개수 대조 | `rsl_rl.py:134` `load` |
| 7 | 조립 — `build_leg` 또는 `build_biped` | `run.py:350-352` |
| 8 | IMU 확인 | `run.py:356` |
| 9 | CAN 연결 | |
| 10 | 영자세로 천천히 옮김 → 대기 → Enter → 정책 | `run.py:150` `staged` |

1~8 은 **모터를 켜기 전**임. 설정이나 규격이 어긋나면 여기서 멈춤.

### 2.3 모델에 딸린 값이 박혀 있는 곳

| 값 | 자리 | 어디서 온 값인가 |
|---|---|---|
| `action_scale`, `obs_dim`, 위상 | `policy.py:179-183` `BALANCE`/`HOPPING` | 학습 설정 |
| 관절 순서 | `policy.py:70`, `84`, `116` | 학습 설정 |
| 제어 주기 | `run.py:109` `POLICY_HZ` | 학습 설정 (시뮬 dt × decimation) |
| 게인 | `run.py:116-117` `POLICY_KP/KD` | 학습 설정 (시뮬 액추에이터) |
| 가중치 위치 | `run.py:106` `WEIGHTS_DIR` | 관례 |
| 발목 공간 | CLI 인자 | 학습 설정 |
| 발목 출력 | CLI 인자 | 사람 |

**전부 학습 쪽에서 오는 숫자인데 파이썬 코드나 사람 손에 있음.** 모델이 바뀔 때마다
코드를 고치거나 인자를 맞게 쳐야 함.

`robot.yaml` 이 캘리브레이션과 파일을 나눈 기준이 "이 숫자를 어디서 얻었나" 임
([`config/README.md`](../config/README.md)). 같은 기준을 적용하면 이 값들은 **가중치
옆 파일**에 있어야 함.

---

## 3. 설정 파일

### 3.1 위치

```
config/policies/
  balance.yaml     규격
  balance.pt       가중치
  biped_walk.yaml
  biped_walk.pt
```

한 모델이 파일 두 개임. 이름이 같아서 짝이 보이고, 가중치를 옮길 때 규격이 같이
따라감.

형식은 **YAML** — `robot.yaml` 과 같게 둠. 로더도 같은 방식으로 씀 (3.5).

### 3.2 예시

**다리 하나, 발목 관절 공간, 토크**

```yaml
weights: balance.pt
hz: 50
action_scale: 0.25
gains: {kp: 20.0, kd: 0.502}
ankle_output: torque

joints:
  - hip_pitch
  - hip_roll
  - hip_yaw
  - knee
  - ankle_pitch
  - ankle_roll

observation:
  - base_ang_vel
  - projected_gravity
  - joint_pos
  - joint_vel
  - actions
```

**다리 하나, 발목 모터 공간**

```yaml
weights: balance_ab.pt
hz: 50
action_scale: 0.25
gains: {kp: 20.0, kd: 0.502}
ankle_output: position          # 모터 공간은 위치만 됨

joints:
  - hip_pitch
  - hip_roll
  - hip_yaw
  - knee
  - ankle_a                     # 여기가 모터 이름이면 곧 모터 공간
  - ankle_b

observation: [base_ang_vel, projected_gravity, joint_pos, joint_vel, actions]
```

**양다리 12칸, 뛰기**

```yaml
weights: biped_hop.pt
hz: 50
action_scale: 0.5
gains: {kp: 20.0, kd: 0.502}
ankle_output: position

joints:                         # 왼다리 먼저 -- 5장
  - left_leg/hip_pitch
  - left_leg/hip_roll
  - left_leg/hip_yaw
  - left_leg/knee
  - left_leg/ankle_pitch
  - left_leg/ankle_roll
  - right_leg/hip_pitch
  - right_leg/hip_roll
  - right_leg/hip_yaw
  - right_leg/knee
  - right_leg/ankle_pitch
  - right_leg/ankle_roll

observation:
  - base_ang_vel
  - projected_gravity
  - joint_pos
  - joint_vel
  - actions
  - hop_phase: {period_s: 0.6}
```

### 3.3 필드

| 필드 | 필수 | 뜻 | 지금 어디 있나 |
|---|---|---|---|
| `weights` | ✅ | 가중치 파일. 설정 파일 기준 상대 경로 | `run.py:106` + `--weights` |
| `hz` | ✅ | 제어 주기. 시뮬 주기와 같아야 함 | `run.py:109` |
| `action_scale` | ✅ | 목표 = 기본자세 + `action_scale` × 행동 | `policy.py:179` |
| `gains.kp`, `gains.kd` | ✅ | 학습에 쓴 게인. 네 관절과 발목 토크 둘 다 | `run.py:116-117` |
| `ankle_output` | | `position` (기본) / `torque` | `--ankle-output` |
| `joints` | ✅ | 모델 출력과 관찰의 순서 | `policy.py:70`, `84`, `141` |
| `observation` | ✅ | 관찰 항목과 순서 (4장) | `policy.py:210` 코드에 박혀 있음 |
| `default_pose` | | 관절별 기본 자세 (8.1) | 없음. 전부 0 으로 가정 |
| `meta` | | 학습 출처 기록 (8.5) | 없음 |

### 3.4 적지 않고 알아내는 것

**같은 사실을 두 곳에 적지 않음.** 두 곳에 적으면 서로 어긋날 수 있는데, 어긋나도
코드로 못 잡는 것들이 있음.

| 알아낼 것 | 어디서 | 적으면 생기는 문제 |
|---|---|---|
| 발목 공간 (rp / ab) | `joints` 에 `ankle_a` 가 있으면 ab | `joints` 는 ab 인데 `ankle_space: rp` 로 적혀도 모름 |
| 관찰 길이 | `observation` 과 `joints` 개수 | 적은 값과 목록이 어긋나도 모름 |
| 행동 개수 | `len(joints)` | 같음 |
| 다리 하나 / 양다리 | `joints` 이름에 `/` 가 있나 | `--robot` 과 `joints` 가 어긋날 수 있음 |

발목 공간을 `joints` 에서 알아내는 것은 **`Leg` 이 이미 하는 방식과 같음.** `Leg` 은
플래그를 받지 않고 들어온 이름을 보고 판정함 (`robots/leg.py` 의 `_ankle_space`).

**이것이 지금 `huphy-run` 의 구멍을 닫음.** 지금은 rp 모델에 `--ankle-space ab` 를 줘도
관찰 개수·행동 개수가 같아서 가중치 검사를 통과함 (`policy.py` 모듈 설명의 "이름이
어긋나도 코드로는 안 잡힘"). 설정 파일이 가중치와 한 쌍으로 다니면 어긋날 자리가
없어짐.

### 3.5 로더

`config/loader.py` 와 같은 방식으로 씀.

- **모르는 키는 에러** — `_check_keys` (`loader.py:57`). YAML 은 오타를 조용히 삼킴
- **기본값은 dataclass 에만** — 로더는 있는 키만 골라 넘김
- 읽은 결과는 frozen dataclass. 지금의 `PolicySpec` 을 넓히거나 새 자료형을 둠

---

## 4. 관찰 항목

설정은 **어떤 항목을 어떤 순서로** 넣는지만 말함. 각 항목을 어떻게 계산하는지는
코드에 남음.

| 이름 | 칸 | 출처 | 단위 (모델 입력) | 지금 계산하는 곳 |
|---|---|---|---|---|
| `base_ang_vel` | 3 | IMU 각속도 | rad/s | `policy.py:231` |
| `projected_gravity` | 3 | IMU 중력방향. 센서 모듈이 만들어 올림 | 단위벡터 | `policy.py:235` |
| `joint_pos` | n | 관절 각도 − 기본 자세 | rad | `policy.py:238` |
| `joint_vel` | n | 관절 속도 | rad/s | `policy.py:239` |
| `actions` | n | 직전 주기에 모델이 낸 값 그대로 | — | `policy.py:241` |
| `hop_phase` | 2 | 위상 sin/cos | — | `policy.py` 의 `hop_phase` |

n 은 `len(joints)`. 다리 하나 6, 양다리 12.

코드 쪽에 **이름 → 계산 함수** 표를 두고, 설정에 모르는 이름이 오면 거부함. 새 항목이
필요하면 표에 한 줄 더하는 것이 코드 변경의 전부가 됨.

`joint_pos` / `joint_vel` 은 `joints` 순서를 따름. `Leg.get_observation()` 이 모터 값과
FK 로 푼 발목 자세를 **둘 다** 내므로 발목 공간이 어느 쪽이든 읽을 수 있음.

---

## 5. 출력 규격 — 양다리

학습 쪽과 정한 규격임.

| 칸 | 이름 |
|---|---|
| 0-5 | `left_leg/hip_pitch` `hip_roll` `hip_yaw` `knee` `ankle_pitch` `ankle_roll` |
| 6-11 | `right_leg/hip_pitch` `hip_roll` `hip_yaw` `knee` `ankle_pitch` `ankle_roll` |

**왼다리가 먼저임.** 지금 코드에 `policy.BIPED_LEGS` (`policy.py:116`) 로 고정돼 있고,
`huphy-run --robot` 이 이 순서를 씀.

`robot.yaml` 의 `limbs` 순서(지금 오른다리가 먼저)와 **무관함.** 모델에 이름을 붙이는
곳은 `joint_targets` 의 `zip(order, action)` 하나뿐이고, 이름이 붙은 뒤로는 `Biped` 가
이름을 보고 다리별로 나눔. 그래서 `Biped.joint_names` 의 순서는 결과에 영향이 없음.

**`Biped.joint_names` 에서 순서를 끌어오면 안 됨.** 그러면 오른다리가 먼저 됨.

설정 파일로 가면 `joints` 목록이 이 표를 그대로 적은 것이 됨. 다리 이름은
`robot.yaml` 의 `limbs` 키와 같아야 함 — `Biped` 가 그 이름으로 명령을 나눔.

---

## 6. 시작 전 검사

**전부 CAN 을 열기 전**에 함. 모터를 켠 뒤에 걸리면 이미 토크가 들어간 뒤임.

| # | 검사 | 걸리면 | 지금 `huphy-run` 에 있나 |
|---|---|---|---|
| 1 | 설정 파일에 모르는 키 | 오타. 멈춤 | — (설정 파일 없음) |
| 2 | 필수 필드 빠짐 | 멈춤 | — |
| 3 | `joints` 의 다리 이름이 `robot.yaml` 에 있음 | `Biped` 가 못 나눔 | 있음 (`--robot` 경로) |
| 4 | `joints` 의 관절 이름이 `Leg` 이 받는 이름임 | 첫 주기에 "모르는 관절" | 첫 주기에만 걸림 |
| 5 | 발목 공간 판정 — 한 쌍이 다 있음, 섞이지 않음 | 첫 주기에 거부 | 첫 주기에만 걸림 |
| 6 | 모터 공간 + 토크 | 발목만 명령을 못 받음 | 있음 (`run.py:312`) |
| 7 | 계산한 관찰 길이 == 가중치 입력 개수 | 신경망이 엉뚱한 자리를 읽음 | 있음 (`rsl_rl.py:151`) |
| 8 | `len(joints)` == 가중치 출력 개수 | 행동이 엉뚱한 관절로 감 | 있음 (`rsl_rl.py:156`) |
| 9 | 정규화 값 개수 == 입력 개수 | 정규화가 틀림 | 있음 |
| 10 | 관찰에 IMU 항목이 있으면 IMU 가 붙어 있음 | 6칸이 0 | 있음 (`run.py:356`) |
| 11 | 가중치 파일이 있음 | | 있음 |
| 12 | 캘리브레이션이 채워져 있음 | 토크가 안 나감 | 있음 (`enable` 에서) |

4·5 는 지금 첫 주기에 `Leg` 이 거부함 — 그때는 이미 `enable()` 뒤임. 설정 파일이 있으면
`joints` 를 미리 알 수 있으므로 **시작 전으로 당길 수 있음.**

---

## 7. 명령줄에 남는 것

모델에 딸린 값을 인자로 덮어쓰게 두면 3.4 의 구멍이 다시 열림. **실행마다 다른 것만**
인자로 둠.

| 인자 | 남기나 | 이유 |
|---|---|---|
| 정책 이름 (또는 설정 파일 경로) | ✅ | 무엇을 돌릴지 |
| `--duration` | ✅ | 실행마다 다름 |
| `--approach` | ✅ | 로봇이 놓인 자세에 따라 다름 |
| `--allow-uncalibrated` | ✅ | 안전 스위치. 명시해야 함 |
| `--limb` | 한 다리 모델일 때만 | 어느 다리에 돌릴지는 모델이 모름 (11장) |
| `--hz` | ❌ | 학습 주기와 달라지면 시뮬과 다르게 움직임 |
| 게인 | ❌ | 학습에 쓴 값이어야 함 |
| 발목 공간·출력 | ❌ | 모델이 정함 |
| `--robot` | ❌ | `joints` 가 정함 |

---

## 8. 추가로 검토한 것

검토 중에 나온, 설정 파일에 **자리가 있어야 하는데 지금 코드에 없는 것**들임.

### 8.1 기본 자세 — 가장 중요함

지금 코드는 **기본 자세가 전부 0** 이라고 가정함.

| 자리 | 가정 |
|---|---|
| `policy.py:263-265` `joint_targets` | "기본 자세가 전부 0 이라 두 번째 항만 남음" |
| `policy.py:237` `observation_vector` | "기본 자세가 전부 0 이라 상대 각도가 곧 각도임" |
| `run.py:124` `ZERO_POSE` | 시작 자세가 전부 0 |

학습에서 기본 자세가 0 이 아니면 (예: 무릎을 굽힌 채 서는 자세를 기준으로 학습)
**세 곳이 전부 틀림.**

```
목표   = 기본자세 + action_scale × 행동      지금은 기본자세 항이 없음
관찰   = 실측 − 기본자세                      지금은 빼지 않음
시작   = 기본자세로 옮김                      지금은 0 으로 옮김
```

값은 다 정상 범위라 **코드가 못 잡음.** 로봇은 매 순간 기본자세만큼 어긋난 곳을 목표로
삼음.

설정에 `default_pose` 를 두고 없으면 0 으로 봄.

```yaml
default_pose:
  left_leg/knee: 30.0
  right_leg/knee: 30.0
```

[`POLICY.md`](../src/huphy/control/POLICY.md) 5.2 의 "실물 영점 자세가 시뮬의 기준 자세와
같은지" 확인 항목과 짝임.

### 8.2 관찰 스케일·행동 클립

학습 설정에 따라 관찰 항목마다 배율을 곱하거나 (예: 각속도 × 0.25), 행동을 범위로
자르는 경우가 있음. 지금 코드에는 둘 다 없음.

학습이 그렇게 짰으면 설정에 자리가 필요함.

```yaml
observation:
  - base_ang_vel: {scale: 0.25}
  - projected_gravity
  ...
action_clip: 1.0
```

지금 두 모델(`balance`, `hopping`)은 배율 없이 학습됐으므로 **기본은 배율 1, 자르지
않음.**

### 8.3 IMU 고르기

지금 `run.py:406` 이 `leg.imus[0]` — **첫 번째 IMU** 를 씀. 센서가 둘 이상이면 어느
것이 쓰일지가 설정 순서에 달림.

설정에 이름으로 적음.

```yaml
imu: main
```

`robot.yaml` 의 `imus` 키와 같은 이름. 없는 이름이면 시작 전에 멈춤.

### 8.4 발목 토크 게인

`ankle_output: torque` 일 때 발목에 거는 **관절** 게인은 모터 게인과 다른 값임
(`robots/leg.py` 의 `ankle_kp`/`ankle_kd`). 지금은 `run.py` 가 같은 `POLICY_KP/KD` 를
두 곳에 넣음.

시뮬이 발목에 다른 게인을 썼으면 따로 적어야 함.

```yaml
gains:
  kp: 20.0
  kd: 0.502
  ankle: {kp: 20.0, kd: 0.502}     # 없으면 위 값
```

### 8.5 메타데이터

어느 학습에서 나온 가중치인지 기록함. 실물 로그와 시뮬 결과를 대조할 때 필요함.

```yaml
meta:
  run: 2026-09-20_biped_hop
  checkpoint: 49999
  sim_config: half_huphy.xml
  commit: 1a2b3c4
```

동작에는 안 쓰고, 시작 화면과 텔레메트리 CSV 머리에 찍음.

### 8.6 모델 출력 이상값

[`POLICY.md`](../src/huphy/control/POLICY.md) 5.5 — 모델이 NaN 을 내면 대비가 없음.

위치 명령은 `safety/guards.py` 가 NaN 을 거부함 (`reject_nan`). **발목 토크는 가드를 안
지나감** (POLICY.md 5.4). 최종 실행기가 토크를 쓰려면 이 구멍이 먼저 막혀야 함.

---

## 9. 선행 조건 — 이것 없이는 "바로 동작" 이 아님

설정 파일이 있어도 아래가 없으면 최종 실행기로 쓸 수 없음.

| | 무엇이 없나 | 자리 |
|---|---|---|
| 캘리브레이션 | 1.0 은 양다리 다 `limits_deg` 가 비어 있음 | [full_robot.md](full_robot.md) 5장 |
| 상태 기계 | 넘어져도 안 멈춤. 정책 하나만 돎 | POLICY.md 6장 |
| 발목 토크 가드 | 토크 모드에 한계 검사 없음 | POLICY.md 5.4 |
| 수거 밀림 | 제어 경로에 `flush_rx` 가 없어 늦게 온 응답이 다음 주기로 밀림 | 주기 작업 때 같이 봄 |
| torch 대조 | numpy 계산이 torch 와 같은지 확인 안 됨 | POLICY.md 5.1 |
| 기본 자세 | 0 이 아닌 기본 자세를 못 씀 | 8.1 |

---

## 10. 재사용할 것 / 새로 만들 것

### 그대로 쓰는 것

| | 자리 | 이미 되는 것 |
|---|---|---|
| 양다리 조립 | `scripts/bringup.py` `build_biped` | 수신 스레드, IMU 를 로봇에 붙임 |
| 다리별로 나누기 | `robots/biped.py` `split_action` | 이름의 `/` 앞으로 나눔 |
| 발목 공간 판정 | `robots/leg.py` `_ankle_space` | 이름으로 rp / ab 판정 |
| 가중치 읽기 | `control/rsl_rl.py` `load` | 입력·출력 개수 대조 |
| 관찰·행동 변환 | `control/policy.py` | `order` 인자로 순서를 받음 |
| 제어 루프 | `control/loop.py` | 통신 두절 시 정지 |
| 시작 절차 | `scripts/run.py` `staged` | 영자세로 옮김 → 대기 → 시작 |

### 새로 만드는 것

| | 무엇 |
|---|---|
| 설정 자료형 | 3.3 의 필드를 담는 frozen dataclass |
| 설정 로더 | `config/loader.py` 방식. 모르는 키 거부 |
| 관찰 항목 표 | 4장의 이름 → 계산 함수. `observation_vector` 가 이것을 따라감 |
| 기본 자세 반영 | `joint_targets`, `observation_vector`, 영자세 (8.1) |
| 진입점 | 새 명령어. 설정을 읽어 위 조립·제어를 부름 |
| 시작 전 검사 | 6장 중 지금 첫 주기에만 걸리는 것을 앞으로 당김 |

`huphy-run` 은 **안 고침.** 실험 도구로 그대로 남음.

---

## 11. 정해야 할 것

| | 선택지 | 추천 |
|---|---|---|
| 명령어 이름 | — | 정해 주실 것 |
| 설정 파일을 누가 쓰나 | 학습 쪽이 가중치 저장할 때 같이 / 사람이 손으로 | **학습 쪽.** 손으로 옮기면 학습 설정과 어긋날 여지가 남음 |
| 한 다리 모델은 어느 다리에 | `--limb` 인자 / 설정에 적음 | `--limb`. 같은 모델을 좌우에 돌릴 수 있게 |
| `BALANCE`/`HOPPING` 상수 | 설정 파일로 옮김 / 코드에 둠 | 새 실행기는 설정만 봄. `huphy-run` 은 지금 상수 그대로 |
| 설정을 이름으로 찾나 경로로 받나 | `config/policies/<이름>.yaml` / 파일 경로 | 둘 다 받음. 이름이면 그 폴더, 경로면 그대로 |

---

## 12. 예상 순서

| 단계 | 무엇 | 끝났다는 기준 |
|---|---|---|
| 1 | 학습 쪽과 설정 파일 형식 합의 | 3.3 의 필드와 8장 항목이 확정됨 |
| 2 | 설정 자료형과 로더 | 모르는 키·빠진 필드를 거부하는 테스트 통과 |
| 3 | 관찰 항목 표 | 지금 `balance`/`hopping` 가중치가 설정 파일로 같은 결과를 냄 |
| 4 | 기본 자세 반영 | 0 이 아닌 기본 자세에서 목표·관찰·시작 자세가 맞음 |
| 5 | 진입점과 시작 전 검사 | 6장 검사가 전부 CAN 을 열기 전에 걸림 |
| 6 | 9장 선행 조건 | 상태 기계·토크 가드·수거 밀림 |
| 7 | 실물 | 설정 파일 하나로 인자 없이 돎 |

3 에서 **지금 두 모델로 대조**하는 것이 중요함. 설정 파일 경로와 `huphy-run` 이 같은
관찰 벡터·같은 목표를 내는지 테스트로 고정하면, 옮기다 틀어진 곳이 바로 드러남.
