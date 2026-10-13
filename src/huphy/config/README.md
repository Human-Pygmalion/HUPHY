# `config/` — 설정 읽기

```
config/
├── schema.py   설정이 어떻게 생겼는지
└── loader.py   robot.yaml 에서 읽는 법
```

값 자체는 여기 없음. 저장소 루트의 [`config/`](../../../config/) 에 있음.

`schema` 는 의존이 없음. `loader` 는 PyYAML 을 함수 안에서 import함.

---

## 모양과 읽기를 나눔

```
schema.py   설정이 무엇인지     <- 파일 형식을 모름
loader.py   파일에서 읽는 법     <- YAML 을 앎
```

나중에 명령줄 인자로 설정을 만들거나 다른 형식을 쓰게 되어도 `schema.py` 는 그대로임.

---

## `schema.py`

```
RobotConfig          로봇 전체
 ├─ LimbConfig       팔다리 하나 = CAN 채널 하나
 │   └─ Motor        관절 하나 (motors/base.py)
 ├─ SafetyConfig
 └─ TelemetryConfig
```

### 전부 frozen

```python
c = SafetyConfig()
c.max_delta_deg = 999      # 에러
```

설정은 시작할 때 한 번 읽고 끝임. **제어 도중에 누가 한계를 넓히거나 게인을 올리면
안 됨.**

### 기본값이 여기 있음

```python
command_margin_deg: float = 3.0
max_delta_deg: float = 50.0
enforce_limits: bool = True
```

`loader.py` 에는 기본값이 없음. 두 군데 있으면 어느 쪽이 실제로 쓰이는지 알 수 없음.

### 스스로 검사함

```python
def __post_init__(self):
    if self.control_hz <= 0:
        raise ValueError(...)
    if self.side not in (None, "left", "right"):
        raise ValueError(...)
```

**파일에서 읽든 코드로 만들든 같은 검사를 받음.** 검사를 `loader.py` 에만 두면
테스트에서 손으로 만든 설정은 그냥 통과함.

### 이름·종류·기하를 나눔

```yaml
limbs:
  right_leg:          # 개체 이름. 키
    kind: leg         # 종류. 어떤 기구학
    side: right       # 기하. 거울상인지
```

하나가 셋을 겸하면 팔이 붙거나 허리처럼 좌우가 없는 부위가 생길 때 막힘 (이슈 #5).

개체 이름이 키인 이유: 같은 이름이 둘일 수 없게 하려는 것임.

`side` 는 "거울상이다" 라는 사실만 적음. **실제 부호 뒤집기는 캘리브레이션의 `sign`
이 함** — 그건 재서 얻는 값이라 여기 올 수 없음.

### `LimbConfig` 가 채널 하나인 이유

양다리를 한 버스에 묶으면 12개 모터의 프레임이 같은 선을 나눠 쓰게 되어 주기 예산이
두 배가 됨. 두 버스는 물리적으로 독립이라 진짜로 겹쳐 보낼 수 있음 (이슈 #10).

### 조회 도우미

| | |
|---|---|
| `limb.period_s` | 100Hz -> 0.01초 |
| `limb.motors_by_id()` | `{10: Motor}`. `RobStrideBus` 가 받는 형태 |
| `limb.joint_of(10)` | `"knee"`. 진단 메시지용 |
| `limb.unconfigured()` | 아직 못 채운 관절 이름들 |
| `limb.is_configured` | 한계와 게인이 다 있는지. 한계는 `Leg` 가 캘리브레이션에서 채움 |
| `robot.limb(name)` | 없으면 가용 목록을 알려줌 |
| `robot.limbs_of_kind("leg")` | 두 다리에 같은 처리를 걸 때 |
| `robot.channels` | 중복 없이, 나온 순서대로 |

`motors_by_id()` 가 경계임. **버스 계층은 관절 이름을 모름** — 여기서 이름을 버리고
모터 id 만 넘김.

### 채널 안에서 id 가 겹치는지 봄

`RobotConfig.__post_init__` 이 **팔다리를 넘어서** 확인함. 다른 팔다리라도 같은
채널이면 같은 선을 쓰므로, id 가 겹치면 응답이 충돌해 구분되지 않음.

버스가 다르면 겹쳐도 됨.

---

## `loader.py`

### 모르는 키를 막음

```python
ROBOT_KEYS = {"name", "limbs", "safety", "telemetry"}
LIMB_KEYS  = {"kind", "side", "channel", "interface", "control_hz",
              "calibration", "motors"}
MOTOR_KEYS = {"id", "model", "kp", "kd"}
```

YAML 은 모르는 키를 조용히 넘김. `contorl_hz: 200` 이라고 쓰면 그 줄이 무시되고
기본값 100Hz 로 돎 — **설정을 고쳤는데 아무것도 안 바뀜.**

증상이 "느리다" 로 나타나므로 원인을 설정에서 찾을 이유가 없어 오래 걸림.

```
ConfigError: limbs.right_leg: 모르는 키 ['contorl_hz']
             (가용: ['calibration', 'channel', 'control_hz', 'interface', ...])
```

쓸 수 있는 키를 같이 냄. 무엇을 잘못 썼는지 바로 보임.

### 있는 키만 골라 넘김

```python
def _pick(data, keys):
    return {k: data[k] for k in keys if k in data}
```

없는 키는 아예 안 넘김. 그래야 `schema.py` 의 기본값이 쓰임.

`data.get("control_hz", 100.0)` 이라고 쓰면 기본값이 여기 또 생김.

### 에러가 위치를 말함

```
limbs.right_leg.motors.knee: 모르는 키 ['kpp']
```

YAML 은 줄 번호를 안 알려주지만 경로를 따라가면 바로 찾음.

### 상대 경로를 품

```yaml
calibration: calibration/right_leg_v0.5.json
```

**`robot.yaml` 이 있는 폴더 기준**으로 절대 경로가 됨. 어디서 실행하든 같은 파일을
가리킴.

### 모터는 목록이 아니라 사전

```yaml
motors:
  knee: {id: 10, model: RS02}
```

목록이면 이름이 없어 **어느 관절인지 말할 수 없음.** 캘리브레이션 파일도 관절 이름을
키로 쓰므로 두 파일을 나란히 놓고 대조할 수 있음.

목록으로 쓰면 무엇을 고쳐야 하는지 알려주고 멈춤.

---

## 쓰는 법

```python
from huphy.config import load_robot

robot = load_robot("config/robot.yaml")
leg = robot.limb("right_leg")

leg.channel          # "canr" (1.0). 0.5 면 "can1"
leg.period_s         # 0.01
leg.motors["knee"]   # Motor(id=10, model='RS02', limits_deg=None, ...)
leg.motors_by_id()   # RobStrideBus 로 넘길 형태
```

한계도 게인도 없는 상태라 `leg.is_configured` 가 `False` 임. 무엇이 비었는지는
`leg.unconfigured()` 로 봄.

---

## 여기 없는 것

**실측값** — `sign`, `offset_deg`, `zero_reference` 는
[`calibration/`](../calibration/README.md) 이 읽음. 무효화 시점이 달라 파일을 나눔.

**벤더 사양** — 인코딩 범위, 명령 바이트는 데이터시트에서 오고 모터를 바꾸지 않는
한 변하지 않음. `motors/robstride/tables.py` 에 있음.

---

## 테스트

```bash
PYTHONPATH=src python3 -m pytest tests/test_config.py -q
```

45개. 대부분 거부 조건임 — 설정이 잘못됐을 때 조용히 기본값으로 돌지 않고 멈추는지.

실제 `config/robot.yaml` 을 읽는 테스트가 있음. **스키마와 파일이 어긋나면 여기서
걸림** — 한쪽만 고치고 넘어가는 것을 막음.
