# `config/` — 설정 값

**코드가 아니라 데이터임.** 읽는 코드는 `src/huphy/config/` 에 있음.

```
config/
├── robot.yaml           -> robot_v0.5.yaml   심볼릭 링크. 지금 쓰는 쪽
├── robot_v0.5.yaml         사람이 적는 것. RS02 x4 + RS00 x2
├── robot_v1.0.yaml         사람이 적는 것. RS04 x3 + RS03 x3
└── calibration/            조립을 재서 얻는 것. 버전·다리마다 하나
    ├── right_leg_v0.5.json
    ├── left_leg_v0.5.json
    ├── right_leg_v1.0.json
    └── left_leg_v1.0.json
```

---

## 하드웨어 버전을 파일로 나눔

모터가 바뀌면 **한 줄이 아니라 파일 전체가 다른 로봇**이 됨 — 모델, 게인 시작값,
`max_delta_deg`, 실측값이 전부 갈림. 한 파일에 두고 주석으로 갈라 두면 어느 줄이
어느 로봇 것인지 매번 세어야 함.

    robot_v0.5.yaml   고관절·무릎 RS02, 발목 RS00
    robot_v1.0.yaml   hip_pitch/hip_roll/knee RS04, hip_yaw/발목 RS03

**바꾸는 방법은 링크를 다시 거는 것임.**

```bash
ln -sf robot_v1.0.yaml config/robot.yaml
```

스크립트는 `config/robot.yaml` 이라는 이름 하나만 찾으므로 코드는 손댈 것이 없음.
한 번만 다른 쪽으로 돌려 볼 때는 `--config` 로도 됨.

```bash
huphy-commission --config config/robot_v1.0.yaml --limb right_leg scan
```

**실측값도 버전마다 나뉨.** `sign`/`offset_deg`/`limits_deg` 가 전부 조립 결과라
모터를 갈면 무효임. 파일 이름에 버전이 붙어 있어 **덮어쓸 수 없음** — 1.0 커미셔닝을
하다 중간에 0.5 로 되돌려도 0.5 의 실측값이 그대로 남아 있음.

    robot_v0.5.yaml  ->  calibration/right_leg_v0.5.json   limits 6/6 잼
                         calibration/left_leg_v0.5.json    아직 안 잼
    robot_v1.0.yaml  ->  calibration/right_leg_v1.0.json   아직 안 잼
                         calibration/left_leg_v1.0.json    아직 안 잼

1.0 파일은 `limits_deg` 가 전부 `null` 인 빈 틀임. `null` 은 "제한 없음" 이 아니라
**"아직 안 잼"** 이고, 그 상태에서는 `Motor.is_configured` 가 `False` 라 제어 진입이
막힘. `huphy-commission sweep` 이 재서 채움.

---

## 파일이 둘인 이유

숫자에 두 종류가 있음.

**도면 보고 적는 숫자** — 로봇을 만지지 않아도 앎.

```
무릎 모터는 CAN id 10 번이고 RS02 다
오른다리는 can1 에 붙어 있다
```

**만져서 알아내는 숫자** — 실제로 조립하고 재봐야 앎.

```
이 모터는 반대로 돈다             -> sign = -1
여기가 관절 0도다                -> offset_deg
여기부터 여기까지 돈다            -> limits_deg
영점을 다리 편 자세에서 잡았다     -> zero_reference
```

| | 어디서 오나 |
|---|---|
| `sign` | 설계. 쓰는 코드가 없음 |
| `offset_deg`, `limits_deg` | `commission sweep` |
| `zero_reference` | `commission zero` |
| `kp`, `kd` | 사람이 튜닝하며 손으로 적음 |

**한 파일에 두면 한쪽을 고칠 때 다른 쪽을 덮어씀.** 캘리브레이션 절차는 파일을
통째로 새로 쓰는데, `robot.yaml` 은 주석이 많아서 프로그램이 다시 쓰면 주석이
전부 날아감. 그래서 프로그램은 JSON 만 씀.

### 어느 파일인지 고르는 법

숫자를 보고 물어보면 됨.

> **"이 숫자를 어디서 얻었나?"**

```
도면 보고 적었다        ->  robot.yaml
로봇을 만져서 알아냈다   ->  calibration/*.json
```

게인만 예외임. 실물에서 찾는 값이지만 **사람이 주석과 함께 손으로 적는 값**이라
`robot.yaml` 에 둠. 프로그램이 쓰지 않음.

### 벤더 사양은 여기 없음

인코딩 범위, 명령 바이트 같은 것은 데이터시트에서 오고 모터를 바꾸지 않는 한
변하지 않음. `src/huphy/motors/robstride/tables.py` 에 있음.

---

## `robot.yaml`

### 팔다리

```yaml
limbs:
  right_leg:          # 개체 이름. 이 로봇에서 이 팔다리를 부르는 이름
    kind: leg         # 종류. 어떤 기구학을 쓰는지
    side: right       # 기하. 거울상인지
    channel: can1
    interface: socketcan
    control_hz: 100.0
    calibration: calibration/right_leg_v0.5.json
    motors:
      knee: {id: 10, model: RS02, kp: 20.0, kd: 1.0}
```

**이름·종류·기하를 나눠 둠.** 하나가 셋을 겸하면 팔이 붙거나 허리처럼 좌우가 없는
부위가 생길 때 막힘 (이슈 #5).

개체 이름이 키인 이유: 같은 이름이 둘일 수 없게 하려는 것임.

**팔다리 하나가 CAN 채널 하나를 씀.** 양다리를 한 버스에 묶으면 12개 모터가 같은
선을 나눠 쓰게 되어 주기 예산이 두 배가 됨. 두 버스는 물리적으로 독립이라 진짜로
겹쳐 보낼 수 있음 (이슈 #10).

### 모터

관절 이름이 키임. 목록이 아님 — 목록이면 이름이 없어 어느 관절인지 말할 수 없음.

| 키 | 뜻 |
|---|---|
| `id` | CAN id. 한 채널 안에서 유일해야 함 |
| `model` | `RS00` `RS02` `RS03` `RS04`. 인코딩 범위가 다름 (아래) |
| `kp`, `kd` | MIT 모드에서 매 프레임 실려 나가는 게인 |

**한계각은 여기 없음.** `calibration/*.json` 의 `limits_deg` 임 — `commission sweep`
이 재서 적음. 여기 적으면 거부함: 같은 값이 두 군데 있으면 어긋났을 때 어느 쪽이
진짜인지 알 수 없음 (이슈 #2).

`model` 이 실물과 다르면 **조용히 틀린 토크가 나감.** 프레임에는 N·m 이 아니라 범위
안의 눈금만 실리므로, 표가 다르면 그 비율만큼 어긋나는데 프레임도 응답도 정상임.

```
      토크        속도       Kp / Kd
RS00  ±14 N·m     ±33 rad/s  0~500 / 0~5
RS02  ±17 N·m     ±44 rad/s  0~500 / 0~5
RS03  ±60 N·m     ±20 rad/s  0~5000 / 0~100
RS04  ±120 N·m    ±15 rad/s  0~5000 / 0~100
```

각도는 네 모델 다 ±12.57 rad 임. **게인 범위가 제일 위험함** -- RS02 표로 RS04 에
`kp=20` 을 보내면 모터가 199 로 받음. 자세한 것은
[`docs/motor_setup.md`](../docs/motor_setup.md).

### `kp` / `kd` 는 튜닝 시작값임

오른다리는 `kp: 20.0, kd: 1.0` 임. **여기서부터 올리거나 내리는 자리**이지 찾은
값이 아님.

```
토크 = kp * (목표각 - 현재각) + kd * (0 - 속도) + 토크_FF
```

`kp` 는 목표에서 벗어난 만큼 당기는 힘이고, `kd` 는 속도를 깎는 힘임. 다리를
받침대에 올리고 `bringup` 으로 계단 응답을 보며 찾음 — 목표를 지나쳤다 돌아오면
`kd` 가 부족하고, 못 미치면 `kp` 가 부족하고, 부르르 떨면 `kp` 가 과함.

**처음 만질 때는 `--gain-scale 0.1` 로 더 낮춰 시작할 것.** 값이 들어 있다고 해서
튜닝된 것은 아님.

`0` 은 "안 정해짐" 이 아니라 **"힘 없음"** 임. 명령을 보내도 아무 힘이 안 나감.
그리고 잠금장치를 겸함 — `Motor.is_configured` 가 `kp > 0` 을 보므로, 0이면 제어
진입 자체가 막힘. 왼다리가 그 상태임.

### 왼다리의 `limits_deg` 가 `null` 인 이유

아직 연결되지 않았고 한계도 실측 전임.

**둥근 수를 넣어 두면 실측한 값처럼 보여서 위험함.** 비워 두면
`Motor.is_configured` 가 `False` 가 되어 제어 진입이 막힘 — 값이 없는 것과
"제한 없음" 은 다름.

### 안전

```yaml
safety:
  command_margin_deg: 3.0   # 한계에서 이만큼 안쪽까지만 명령함
  max_delta_deg: 50.0       # 한 주기에 움직일 수 있는 최대 각도
  enforce_limits: true      # false 는 커미셔닝 전용
```

`command_margin_deg` 는 오버슛·관성·측정오차를 흡수하는 여유임. 게인을 튜닝하면
필요한 값이 달라지므로 3도는 임의로 잡은 출발점임.

`max_delta_deg` 는 100Hz 기준이라 50도면 초당 5000도임. 실제로 그렇게 도는 것이
아니라, 계산이 튀었을 때 그 이상은 안 나가게 막는 상한임.

### 텔레메트리

```yaml
telemetry:
  host: null                # 비우면 UDP 송신 안 함
  port: 9870
  csv_path: null
  csv_flush_every: 50       # N 주기마다 디스크에 씀
```

매 주기 디스크에 쓰면 제어 주기가 튐.

### IMU

```yaml
imus:
  main:                       # 개체 이름. 텔레메트리 필드 앞에 붙음
    model: ebimu              # sensors/registry.py 의 키
    port: /dev/ebimu          # udev 로 고정한 심볼릭 링크
    mount: right_leg          # 어디 붙었는지
    output: [quat, gyro, accel, dist, temp, time]
    accel_mode: gravity
    dist_mode: local
    rate_hz: 100
```

`baudrate` 를 안 적으면 **벤더 출하 기본값**을 씀 (EBIMU 115200, Xsens 921600).
여기 한 숫자를 기본값으로 박아 두면 다른 센서 설정에서 이 줄을 생략했을 때 조용히
안 붙음.

`output` 계열은 EBIMU 전용임 — **패킷에 무엇이 켜져 있는지가 안 적혀 있어서** 여기
적어 둬야 함. 센서를 이 목록에 맞추는 것은 `huphy-imu apply` 가 함.

`limbs` 와 나란히 있음. 같은 센서가 다리에 붙었다가 몸통으로 옮겨가므로, 팔다리
안에 두면 옮길 때 설정 구조와 필드 이름이 같이 바뀜.

`mount` 는 팔다리 이름이거나 `torso` `pelvis` `head` 임. 팔다리 이름인데 그런
팔다리가 없으면 멈춤. 한 포트를 두 IMU 가 써도 멈춤.

`baudrate` 가 센서 설정과 다르면 **조용히 아무 패킷도 안 들어옴.**

IMU 가 없어도 로봇은 그대로 돎. 자세한 것은 `src/huphy/sensors/README.md` 참조.

### 경로

`calibration:` 의 상대 경로는 **`robot.yaml` 이 있는 폴더 기준**으로 풀림. 실행
위치가 달라져도 같은 파일을 가리키게 하려는 것임.

### 오타는 에러가 됨

YAML 은 모르는 키를 조용히 넘김. `contorl_hz: 200` 이라고 쓰면 그 줄이 무시되고
기본값 100Hz 로 돎 — **설정을 고쳤는데 아무것도 안 바뀜.**

증상이 "느리다" 로 나타나므로 원인을 설정에서 찾을 이유가 없어 오래 걸림. 그래서
읽는 순간 멈춤.

```
ConfigError: limbs.right_leg: 모르는 키 ['contorl_hz']
             (가용: ['calibration', 'channel', 'control_hz', 'interface', ...])
```

쓸 수 있는 키를 같이 보여주므로 무엇을 잘못 썼는지 바로 보임.

---

## `calibration/*.json`

```json
{
  "schema_version": 1,
  "limb": "right_leg",
  "note": "사람이 남기는 메모",
  "motors": {
    "knee": {"sign": 1.0, "offset_deg": 0.0, "zero_reference": ""}
  }
}
```

```
cal = sign * raw + offset
```

### 관절 이름으로 키를 맞춤

CAN id 는 바뀔 수 있음 (`commissioning.set_can_id`). **관절 자리는 안 바뀜.**

그리고 `robot.yaml` 도 관절 이름을 쓰므로 두 파일을 나란히 놓고 대조하기 쉬움.

### 각 항목

| 항목 | 어떻게 얻나 |
|---|---|
| `zero_reference` | 기계 영점을 **어느 자세에서 잡았는지** 사람이 적음 |
| `sign` | 무동력으로 관절을 + 방향으로 밀고 raw 가 증가하는지 봄 |
| `offset_deg` | 기준 자세에서 읽은 raw 값 |

`zero_reference` 가 필요한 이유: 모터는 영점 값을 저장하지만 **"그때 다리가 어떤
자세였는지" 는 어디에도 남지 않음.** 이 메모가 없으면 영점을 재현할 수 없고,
재현할 수 없으면 `offset` 실측이 무의미해짐.

`commissioning.set_zero` 가 이 메모를 필수 인자로 받는 이유임.

`sign` 이 반대면 목표에서 **멀어지는 방향**으로 토크가 걸림. 가장 먼저 확인할 값임.

### 왜 `sign` 이 필요한가

양다리는 거울상이라 같은 굽힘에 모터 회전 방향이 반대임.

| 물리적 자세 | 오른 무릎 raw | 왼 무릎 raw | cal (양쪽 동일) |
|---|---|---|---|
| 편 상태 | 0 | 0 | 0 |
| 45도 굽힘 | +45 | −45 | 45 |

보행 궤적이 "무릎 45도" 라고 하면 양다리가 같은 동작을 함. raw 로 말하면 다리마다
부호를 뒤집어야 하고, 그걸 잊는 자리가 코드 곳곳에 생김.

### 현재 상태

| | 다리 | 한계 | sign / offset | kp / kd | zero_reference |
|---|---|---|---|---|---|
| 0.5 | right | 채워짐 | 1.0 / 0.0 (재기 전) | 20.0 / 1.0 (튜닝 전) | 비어 있음 |
| 0.5 | left | 없음 | 재기 전 | 없음 | 비어 있음 |
| 1.0 | right | 없음 | 재기 전 | 10.0 / 1.0 (튜닝 전) | 비어 있음 |
| 1.0 | left | 없음 | 재기 전 | 10.0 / 1.0 (튜닝 전) | 비어 있음 |

1.0 의 `kp` 가 0.5 의 절반인 이유: RS04 는 RS02 보다 토크가 7배라 같은 각도 오차에서
훨씬 큰 힘이 나감.

**지금은 `sign=1, offset=0` 이라 cal 과 raw 가 같은 숫자임.** 어느 쪽으로 해석해도
동작이 같아서 두 공간을 섞어 써도 드러나지 않음. `commission sweep` 이 `offset_deg`
를 넣는 순간 갈라짐 (이슈 #2).

왼다리는 `kp = 0` 이라 `LimbConfig.is_configured` 가 `False` 임. 제어 진입이 막힘 —
**의도된 동작임** (이슈 #9).

오른다리는 게인과 한계가 있어 `is_configured` 는 `True` 인데, `zero_reference` 가
비어 있어 `Leg.is_calibrated` 는 아직 `False` 임. `commission zero` 를 실물에서
돌리기 전까지는 `--allow-uncalibrated` 가 필요함.

---

## 누가 읽나

```
robot.yaml
    │  huphy.config.load_robot()
    ▼
RobotConfig
    ├── LimbConfig      팔다리마다. 모터 목록, 채널, 주기
    ├── SafetyConfig    safety.guards 가 그대로 받아 씀
    └── TelemetryConfig
```

**제어 경로는 읽기만 함.** 쓰기는 커미셔닝·캘리브레이션 절차에서만 일어남 —
제어 중에 실측값이 바뀌면 안 됨.

---

## 전제조건

CAN 채널은 이 파일들이 아니라 시스템에서 먼저 올림. 이름은 `channel` 과 같아야 함 —
0.5 는 `can1` / `can0`, 1.0 은 `canr` / `canl` (USB 어댑터, `slcand` 로 올림.
루트 [README](../README.md) 4번).

```bash
sudo ip link set can1 up type can bitrate 1000000
ip -details link show can1
```

속도를 포함한 채널 설정은 커널이 관리함. 코드는 이미 올라와 있는 채널에 붙기만 함.

모터 쪽 전제(MIT 프로토콜, `zero_sta`)는
[motors/README.md](../src/huphy/motors/README.md) 의 "하드웨어 전제" 참조.
