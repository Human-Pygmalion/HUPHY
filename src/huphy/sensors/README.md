# `sensors/` — IMU

```
sensors/
├── base.py         ImuState, Imu. 벤더를 모르는 자료형과 계약
├── group.py        ImuGroup. 붙어 있는 센서 묶음
├── registry.py     설정의 model 문자열 -> 구현체
├── ebimu/          E2BOX EBIMU-9DOF. 지금 쓰는 것
│   ├── commands.py       매뉴얼에서 오는 값. 명령표, 블록별 필드 수
│   ├── protocol.py       한 줄 <-> 값. 순수 함수
│   ├── imu.py            EbimuImu. 위쪽이 쓰는 것
│   └── commissioning.py  센서 설정을 바꿈. huphy-imu 가 부름
└── xsens/          Xsens MTi
    ├── imu.py      XsensImu
    └── xbus/       받아 온 시리얼 읽기 코드
```

`motors/` 와 같은 나눔임. 중립 계층이 자료형과 계약을 정하고 벤더 폴더가 구현함.
센서를 갈아 끼워도 위쪽 코드는 바뀌지 않음.

---

## IMU 는 팔다리의 부속이 아님

같은 센서가 다리에 붙었다가 몸통으로 옮겨감. 그래서 실행 객체에서도 다리와
**나란히** 놓임.

```
Biped
 ├ Leg("right_leg")
 ├ Leg("left_leg")
 └ ImuGroup          <- 여기
```

설정에서 `ImuConfig` 를 `limbs` 안이 아니라 로봇 밑에 둔 것과 짝임. 옮길 때
`mount` 한 줄만 바뀌게 하려는 것인데, 실행 객체에서 다리가 센서를 소유하고 있으면
옮길 때 소유자까지 바뀜.

`ImuGroup` 이 하는 일은 둘뿐임.

| | |
|---|---|
| `connect` / `disconnect` | 전부 엶·닫음. **하나가 실패해도 나머지는 씀** |
| `states()` | 개체 이름 -> `ImuState`. **새로 통신하지 않음** |

여는 데 실패해도 예외를 올리지 않음 — IMU 는 관측이지 제어가 아니라서, 센서
때문에 로봇을 못 움직이면 고장 났을 때 안전한 자세로 되돌리는 것조차 막힘. 값이
꼭 필요한 쪽(정책)이 시작 전에 `len()` 으로 확인함.

다리 하나만 돌릴 때는 `Leg` 가 자기 센서를 들고 있어도 됨. 어디에 넣을지는
조립하는 쪽이 정함.

---

## 받아오는 값

`Imu.read()` 가 `ImuState` 하나를 냄. 새로 통신하지 않고 **가장 최근 값만** 꺼냄.

**제어가 쓰는 것만 필수로 두고, 나머지는 센서가 정함.**

| 필드 | 단위 | 무엇 |
|---|---|---|
| `gravity` | 단위벡터 | 몸체 좌표에서 본 중력. 수평이면 `(0, 0, -1)` |
| `gyro_dps` | 도/초 | 각속도 |
| `accel_mps2` | m/s² | 몸체 좌표계 가속도. **중력 포함** |
| `extra` | | 센서마다 다른 값. 텔레메트리로만 나감 |
| `stamp` | 초 | 이 값을 파싱한 시각. `time.monotonic()` 기준 |
| `is_valid` | | 한 번이라도 패킷을 받았는지 |

`age_ms()` 로 마지막 값 이후 경과를 냄. 한 번도 못 받았으면 `-1`.

못 받았어도 `ImuState` 객체는 나옴. `is_valid` 가 거짓이고 값은 수평·정지와 같음 —
그것이 사실이라는 뜻이 아니라 모른다는 뜻임.

### 자세를 원본 형식으로 안 올림

센서마다 주는 형식이 다름 — EBIMU 는 쿼터니언, Xsens 는 오일러각. 그걸 필수 칸으로
두면 **한 센서를 붙이려고 형식을 정하는 순간 다른 센서가 전부 따라와야 함.**

정책이 실제로 쓰는 것은 중력방향 3개뿐이므로 그것만 필수로 두고, **형식을 아는
벤더 모듈이 계산해 올림.**

```python
gravity_from_quat(quat)               # EBIMU
gravity_from_euler(roll, pitch)       # Xsens. ZYX 순서를 전제함
```

**축이 각속도·가속도와 같아야 함.** EBIMU 는 쿼터니언만 `(X, -Y, -Z)` 축(X 앞 / Y
오른쪽 / Z 아래) 기준으로 보내서, `to_quaternion` 이 순서를 바꾸면서 y, z 부호도
뒤집음. 안 뒤집으면 중력의 **x 성분만** 부호가 반대로 나옴 -- 크기는 1 이라 검사로
안 잡히고, `huphy-imu check` 의 가속도계 대조에서만 드러남 (0.5 가까이 벌어짐).

원본 자세는 `extra` 로 감. 그래프에는 그대로 나옴.

### `extra` 는 센서마다 다름

키 목록은 구현체가 `extra_fields` 로 미리 냄. **고정이어야 함** — CSV 헤더를 실행
전에 써야 하고, 열이 중간에 사라지면 파일이 밀림.

```
EBIMU   qw qx qy qz  roll pitch yaw  dx dy dz  temp sensor_ms
Xsens   roll pitch yaw  temp sensor_ms
```

EBIMU 는 쿼터니언을 받지만 `roll/pitch/yaw` 도 같이 냄 — 사람은 자세를 도로 봄.
원본을 옆에 두는 이유: 그래프가 이상할 때 센서가 이상한 것인지 변환이 이상한
것인지 구분하려면 둘 다 있어야 함. **이 변환은 제어 경로가 아님.**

### 시각이 둘임

    stamp                우리가 그 줄을 파싱한 시각
    extra["sensor_ms"]   센서가 측정한 시각. 센서가 찍어 보냄

`stamp` 만으로는 커널 버퍼에 머문 시간을 못 봄. 스레드가 밀리면 50ms 전에 측정된
값도 지금 파싱하니 `age_ms` 가 0을 냄.

    sensor_ms 는 규칙적인데 stamp 간격이 튐    우리 쪽이 밀림
    sensor_ms 증가량이 주기의 배수로 뜀        패킷이 빠지는 중
    둘 다 안 변하고 age 만 자람                센서가 멈춤

각속도는 도/초임. 센서가 라디안으로 주면 벤더 모듈이 바꿔서 올림.

---

## 설정

```yaml
imus:
  main:
    model: ebimu
    port: /dev/ebimu
    mount: right_leg
    output: [quat, gyro, accel, dist, temp, time]
    accel_mode: gravity
    dist_mode: local
    rate_hz: 100
```

| 키 | 기본값 | 무엇 |
|---|---|---|
| `model` | 필수 | 어느 센서인지. `registry.MODELS` 의 키 |
| `port` | 필수 | 시리얼 장치 경로 |
| `baudrate` | 벤더 기본값 | 센서에 저장된 값과 같아야 함 |
| `mount` | 없음 | 어디 붙었는지. 팔다리 이름이거나 `torso` `pelvis` `head` |
| `output` | `quat gyro accel` | 센서가 보내는 항목. **순서가 곧 패킷 순서** |
| `accel_mode` | `gravity` | `gravity` / `local` / `global` |
| `dist_mode` | `local` | `local` / `global` |
| `rate_hz` | `100` | 센서 출력 주기 |

`baudrate` 에 숫자를 안 적으면 **벤더 모듈의 출하 기본값**을 씀 (EBIMU 115200,
Xsens 921600). 여기 한 숫자를 기본값으로 박아 두면 다른 센서 설정에서 이 줄을
생략했을 때 조용히 안 붙음.

`output` 계열은 EBIMU 전용임. Xsens 설정에 적혀 있어도 무시됨 — Xsens 는 패킷에
어떤 항목인지가 들어 있어 설정으로 알려줄 필요가 없음.

**EBIMU 는 패킷에 무엇이 켜져 있는지가 안 적혀 있음.** 숫자만 오고, 필드 수가 같은
조합이 여럿이라 개수로도 알 수 없음. 그래서 `output` 이 기준이 되고, 센서를 그
목록에 맞추는 것은 `huphy-imu apply` 가 함.

`imus` 는 `limbs` 와 나란히 있음. 팔다리 안에 두지 않는 이유는 같은 센서가 다리에
붙었다가 몸통으로 옮겨가기 때문임 — 옮길 때 `mount` 한 줄만 바뀜.

`port` 는 udev 로 고정한 심볼릭 링크를 쓸 것. USB 는 꽂는 순서대로 `ttyUSB0`,
`ttyUSB1` 이 붙어 재부팅마다 달라짐.

```
# /etc/udev/rules.d/99-xsens.rules
SUBSYSTEM=="tty", ATTRS{idVendor}=="2639", ATTRS{serial}=="<시리얼>", SYMLINK+="xsens_mti"
```

한 포트를 두 IMU 가 쓰면 설정을 읽을 때 멈춤. `mount` 가 팔다리 이름인데 그런
팔다리가 없어도 멈춤.

---

## 갈아 끼우기

`registry.py` 의 표가 `model` 문자열을 구현체로 품.

```python
MODELS = {"xsens_mti": _xsens, "ebimu": _ebimu}
```

다른 센서를 붙이면 `sensors/<벤더>/` 를 만들고 표에 한 줄 더함. 벤더 모듈은
`make_imu` 안에서 import 함 — 안 쓰는 센서의 의존성까지 깔려 있어야 설정을 읽을 수
있으면 곤란함.

구현체가 갖출 것은 `Imu` 프로토콜임. 상속하지 않아도 됨.

```python
name: str
extra_fields: Tuple[str, ...]
is_connected -> bool
connect()  disconnect()
read() -> ImuState
```

`name` 은 설정의 개체 이름임. **텔레메트리 필드 앞에 붙음.**

`extra_fields` 는 그 센서가 `ImuState.extra` 에 넣는 키 목록임. 없어도 동작함 --
테스트의 가짜 IMU 가 이것까지 갖추도록 강요할 이유가 없음.

---

## 붙는 자리

```python
robot.imus_on("right_leg")     # 그 팔다리에 붙은 것만
Leg(limb, bus, ..., imus=[...])
```

`build_leg()` 가 `mount` 를 보고 골라서 넣음. `Leg.imus` 는 비어 있어도 됨.

`leg.connect()` 가 IMU 도 같이 엶. **여는 데 실패해도 다리는 그대로 씀** — IMU 는
관측이지 제어가 아님. 로그에 경고만 남고 `read()` 는 `is_valid=False` 를 냄.

`leg.imu_states()` 가 개체 이름 -> `ImuState` 를 냄.

---

## 텔레메트리

**공통 열은 센서를 바꿔도 같고, 고유 열만 달라짐.**

```
공통    imu/main/gx gy gz      각속도
        imu/main/ax ay az      가속도
        imu/main/grav_x/y/z    중력방향. 정책에 그대로 들어가는 값
        imu/main/age

EBIMU   + qw qx qy qz  roll pitch yaw  dx dy dz  temp  sensor_ms  sensor_dt
Xsens   + roll pitch yaw  temp  sensor_ms  sensor_dt
```

`grav_*` 를 내는 이유: **정책이 실제로 본 값**임. 자세가 이상해 보일 때 센서 원본이
이상한 것인지 중력방향 계산이 이상한 것인지 여기서 갈림.

`sensor_dt` 는 직전 패킷과의 센서 시각 차임. 100Hz 면 10이 정상이고 20이면 한 개가
빠진 것임 -- `age` 는 이걸 못 잡음.

앞에 붙는 것이 **팔다리 이름이 아니라 IMU 개체 이름**임. 다리에서 몸통으로 옮겨도
필드 이름이 그대로라 예전 로그와 그래프 레이아웃이 맞음.

UDP 는 다리 패킷과 **따로** 나감. 다리 하나가 이미 MTU 에 가깝고, 붙는 자리도
다리와 무관함. IMU 가 없으면 이 패킷은 안 보냄.

CSV 는 한 줄에 다 들어감. 값이 없어도 키는 나가고 `age` 가 `-1` 임.

---

## 의존성

```
pyserial    시리얼 포트          EBIMU, Xsens
numpy       패킷 해석            Xsens 만
```

`connect()` 안에서 import 함. 센서를 안 쓰는 실행에서는 없어도 됨.

EBIMU 는 ASCII 한 줄을 `split(",")` 로 자르는 것이 전부라 numpy 가 필요 없음.

---

## EBIMU 세팅

센서 설정은 **비휘발성 메모리에 자동 저장됨.** 전원을 껐다 켜도 남고, 되돌리려면
반대 명령을 보내야 함.

```bash
huphy-imu show          # 센서 설정을 읽어 robot.yaml 과 대조. 안 바꿈
huphy-imu apply --yes   # robot.yaml 대로 센서를 맞춤. [영구]
huphy-imu check         # 부착 방향을 가속도계와 대조. 안 바꿈
huphy-imu watch         # 들어오는 값을 계속 보여줌
```

**설정 파일이 기준임.** 센서에 물어보고 코드가 따라가면, 센서를 갈아 끼우거나 누가
설정을 바꿨을 때 동작이 조용히 달라짐.

### 부착 방향 확인

정지 상태의 가속도계는 **중력방향을 직접 잼.** 자세에서 계산한 값과 같아야 함.

```
$ huphy-imu check
    자세에서 계산한 중력   (  0.643, -0.383, -0.663)
    가속도계가 잰 중력     (  0.643, -0.383, -0.663)
    오차                   0.0003
```

**두 축을 동시에 기울여 놓고 잴 것.** 한 축만 기울이면 못 잡는 어긋남이 있고,
수평이면 부착이 틀려도 통과함 -- 그때는 판정을 거부함.

---

## `xsens/xbus/` 는 받아 온 코드임

출처: `maido-39/Huphychan-RIP_Sim2Real` 의 `utils_imu/`. 그쪽은 다시
`jiminghe/Xsens_MTi_Serial_Reader` 를 감싼 것임.

파일 이름과 내용을 바꾸지 않음 — 원본과 대조할 수 있어야 함. **고친 것은 import
하나뿐**임(평평한 import 를 상대 import 로).

```
SerialHandler.py      시리얼 포트
XbusPacket.py         프레임 조립·체크섬
DataPacketParser.py   패킷 해석
SetOutput.py          센서의 출력 항목 설정
imu_reader.py         위 셋을 감싼 리더. 백그라운드 스레드로 받음
example_usage.py      원본 사용 예
```

여기 것을 직접 쓰지 말 것. 위쪽은 `XsensImu` 를 씀 — 그것이 리더의 dict 를
`ImuState` 로 바꿔 올림.
