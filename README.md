# HUPHY

휴머노이드 로봇 제어. RobStride 액추에이터를 CAN 으로 제어함.

**양다리**(모터 12개, CAN 채널 2개)를 한 루프로 돌림. 다리 하나만 따로 돌리는 것도
그대로 됨 (`--limb`). 팔·상체가 붙어도 같은 구조가 그대로 늘어남.

---

## 목차

- [1. 하드웨어 준비](#1-하드웨어-준비)
- [2. 모터 설정](#2-모터-설정)
- [3. 환경 만들기](#3-환경-만들기)
- [4. CAN 채널 올리기](#4-can-채널-올리기)
- [5. 처음 브링업 순서](#5-처음-브링업-순서) — **여기까지가 세팅**
- [6. 동작 확인](#6-동작-확인) — 움직여 보기
- [7. 명령어 목록](#7-명령어-목록) — 참조
- [하드웨어 버전 바꾸기](#하드웨어-버전-바꾸기)
- [조정해야 할 값](#조정해야-할-값)
- [지금 무엇이 비어 있나](#지금-무엇이-비어-있나)
- [계층 구조](#계층-구조)
- [한 주기에 무슨 일이 일어나나](#한-주기에-무슨-일이-일어나나)
- [왜 이렇게 나눴나](#왜-이렇게-나눴나)
- [테스트](#테스트)

---

## 1. 하드웨어 준비

**하드웨어 버전이 둘임. 어느 쪽인지부터 정할 것.**

```
0.5   robot_v0.5.yaml
  RobStride RS02 x4    고관절 3, 무릎 1
  RobStride RS00 x2    발목 링키지

1.0   robot_v1.0.yaml
  RobStride RS04 x3    hip_pitch, hip_roll, knee
  RobStride RS03 x3    hip_yaw, 발목 2
```

`config/robot.yaml` 은 **심볼릭 링크**임. 그 링크가 가리키는 파일이 읽힘.

```bash
readlink config/robot.yaml                   # 지금 어느 쪽인지
ln -sf robot_v1.0.yaml config/robot.yaml     # 1.0 으로 바꿈
ln -sf robot_v0.5.yaml config/robot.yaml     # 0.5 로 바꿈
```

**모든 명령이 이 링크를 따라감.** 스크립트는 `config/robot.yaml` 이라는 이름 하나만
찾으므로, 링크만 바꾸면 명령어도 코드도 손댈 것이 없음. 캘리브레이션 파일도 각
버전의 yaml 이 자기 것을 가리킴.

자세한 것은 [하드웨어 버전 바꾸기](#하드웨어-버전-바꾸기).

공통:

```
라즈베리파이
CAN 어댑터            USB(CANable 등) 또는 CAN HAT. 다리마다 하나
EBIMU-9DOF           IMU. 시리얼
24V 전원
```

**모델마다 인코딩 범위가 다름.** 토크가 RS00 ±14, RS02 ±17, RS03 ±60, RS04 ±120
N·m 이고, **RS03·RS04 는 게인 범위가 나머지의 10배**임 (`0~5000` vs `0~500`).
프레임에는 N·m 이 아니라 범위 안의 눈금만 실리므로, 설정의 `model` 이 실물과 다르면
그 비율만큼 어긋나는데 프레임도 응답도 정상이라 실물에서 찾기 매우 어려움
([`docs/motor_setup.md`](docs/motor_setup.md)).

어댑터 종류에 따라 **채널 올리는 방법만** 달라짐 ([4번](#4-can-채널-올리기)).
코드와 설정은 같음.

### 배선

모터를 **한 줄로 데이지체인**함. 양 끝에 120Ω 종단저항이 있어야 함 — 없으면 통신이
불안정해짐.

```
어댑터 ── m7 ── m8 ── m9 ── m10 ── m11 ── m12
[120Ω]                                  [120Ω]
```

오른다리는 `can1`, 왼다리는 `can0` 을 씀. **두 다리를 한 버스에 묶지 않음** —
12개 모터가 같은 선을 나눠 쓰면 주기 예산이 두 배가 됨.

### CAN id 와 모델

```
오른다리 (can1)                  왼다리 (can0)
 7   hip_pitch    RS02          1   hip_pitch    RS02
 8   hip_roll     RS02          2   hip_roll     RS02
 9   hip_yaw      RS02          3   hip_yaw      RS02
10   knee         RS02          4   knee         RS02
11   ankle_a      RS00          5   ankle_a      RS00
12   ankle_b      RS00          6   ankle_b      RS00
```

이름이 곧 축임. **오른손 좌표계, X 가 앞(발이 나가는 방향), Z 가 위, Y 가 왼쪽.**

```
roll    X축 회전    다리를 옆으로 기울임
pitch   Y축 회전    다리를 앞뒤로 듦
yaw     Z축 회전    발끝 방향을 돌림
```

몸통에서 발로 내려가며 pitch → roll → yaw → knee → 발목 순서임.

`config/robot.yaml` 과 **정확히 같아야 함.** 다르면 응답이 빠지거나, 더 나쁘게는
엉뚱한 관절이 움직임.

다르면 둘 중 하나를 고침 — 모터의 id 를 바꾸거나, 설정 파일의 `id` 를 바꾸거나.
바꾸는 절차는 [`docs/motor_setup.md`](docs/motor_setup.md).

---

## 2. 모터 설정

**코드가 확인하지 않고 맞다고 가정하는 값들임.** 어긋나면 조용히 잘못 동작함.

| 항목 | 값 | 어디서 정하나 |
|---|---|---|
| 통신 프로토콜 | **MIT** (11-bit 표준 프레임) | 모터 설정 |
| `zero_sta` (`0x7029`) | **1** | 모터 설정. 플래시 저장 |
| 제어 모드 | **MIT** (전원 투입 기본값) | 모터 설정 |
| 비트레이트 | **1 Mbps** | 모터와 CAN 채널 양쪽 |

### 어긋나면

| | 증상 |
|---|---|
| 프로토콜이 private | **연결도 되고 에러도 없는데 모터만 안 움직임** |
| `zero_sta = 0` | 위치가 `[0,360)` 으로 보고되어 음수 각도가 340도로 나옴 |
| 비트레이트 불일치 | 통신이 아예 안 됨 |

**프로토콜이 가장 위험함.** 공장 기본값이 private 인데 이 코드는 MIT 로 보냄.
증상이 "모터만 안 움직인다" 라서 배선·전원·CAN id 를 먼저 의심하게 되고, 프로토콜은
마지막에 떠오름.

### 어떻게 설정하나

**갓 뜯은 모터라면 [`docs/motor_setup.md`](docs/motor_setup.md) 를 따라갈 것.**
`cansend` 로 보낼 프레임이 명령 단위로 다 적혀 있음.

```bash
cansend can1 0000FD7F#0000000000000000     # Type 0   지금 id 확인
cansend can1 070AFD7F#0000000000000000     # Type 7   CAN id -> 10      즉시 적용
cansend can1 1200FD0A#2970000001000000     # Type 18  zero_sta -> 1     RAM 에만
cansend can1 1600FD0A#0102030405060708     # Type 22  플래시에 커밋
cansend can1 1900FD0A#0102030405060200     # Type 25  프로토콜 -> MIT    재투입 필요
```

**이 단계에서는 `huphy-commission` 을 못 씀.** 그 도구는 `robot.yaml` 에 적힌 id 로
11-bit 표준 프레임만 보내는데, 출하 상태 모터는 그 id 를 안 쓰고 프레임 형식도 다름.

MotorStudio 로 해도 됨.

**id 가 부여되고 MIT 로 바뀐 뒤부터** 이 저장소 도구가 붙음. 나중에 모터를 교체하면
그 한 대만 다시 밟으면 됨.

```bash
huphy-commission --limb right_leg protocol knee --to mit --yes
# 전원을 재투입해야 적용됨
```

바뀌었는지 코드가 확인하지는 않음 ([이슈 #11](docs/issues.md)). 재투입 후 `scan` 이
응답하면 MIT 임 ([`docs/motor_setup.md`](docs/motor_setup.md) 8절).

---

## 3. 환경 만들기

### 파이썬

3.9 이상. 라즈베리파이 OS 는 보통 그 이상이 이미 있음.

```bash
python3 --version
```

### 시스템 패키지 (라즈베리파이)

```bash
sudo apt update
sudo apt install -y python3-venv python3-pip can-utils
```

`can-utils` 는 `candump` / `cansend` 를 줌. 코드가 쓰지는 않지만 **배선이 살아
있는지 확인할 때** 있으면 편함.

### 가상환경

```bash
git clone https://github.com/chanhue/HUPHY.git
cd HUPHY

python3 -m venv .venv
source .venv/bin/activate
```

프롬프트 앞에 `(.venv)` 가 붙으면 들어간 것임.

**가상환경을 쓰는 이유**: 시스템 파이썬에 패키지를 깔면 다른 프로그램과 버전이
부딪힘. 라즈베리파이는 시스템 파이썬을 OS 도구가 쓰고 있어서 특히 그럼.

### 설치

```bash
pip install --upgrade pip
pip install -e .            # python-can, numpy, PyYAML
pip install -e ".[dev]"     # + pytest
pip install -e ".[imu]"     # + pyserial. IMU 를 붙일 때만
```

`-e` 는 **소스를 그대로 씀.** 코드를 고치면 다시 설치하지 않아도 바로 반영됨.
설정을 튜닝하면서 계속 고치게 되므로 이쪽이 편함.

### 확인

```bash
python -m pytest tests -q          # 1200 passed
huphy-commission --help
```

**하드웨어 없이 전부 통과해야 함.** 안 되면 설치가 덜 된 것임.

### 다음부터

```bash
cd HUPHY
source .venv/bin/activate
```

빠져나올 때는 `deactivate`.

### 가상환경 없이 쓰려면

`PYTHONPATH` 로도 됨. 설치 없이 돌려볼 때 씀.

```bash
PYTHONPATH=src python3 -m huphy.scripts.commission --limb right_leg scan
PYTHONPATH=src python3 -m pytest tests -q
```

---

## 4. CAN 채널 올리기

**속도는 커널이 정함.** 파이썬이 바꿀 수 없으므로 채널을 올릴 때 넣어야 함.

**어댑터 종류에 따라 방법이 다름.** 어디서 속도를 정하느냐가 갈림길임.

### USB 어댑터 (CANable 등)

USB 로 붙으면 커널이 **시리얼 포트**로 봄. `slcand` 가 그걸 CAN 장치로 바꿔 줌.

```bash
sudo slcand -o -c -s8 /dev/canable0 can1
sudo ip link set can1 up
```

| | 뜻 |
|---|---|
| `-o` | 어댑터의 CAN 채널을 엶 |
| `-c` | 데몬이 끝날 때 닫음 |
| `-s8` | **비트레이트 1 Mbps** |
| `/dev/canable0` | 시리얼 포트 |
| `can1` | 만들어낼 커널 장치 이름 |

`-sN` 은 속도 코드임.

```
s0  10k    s3  100k    s6  500k
s1  20k    s4  125k    s7  800k
s2  50k    s5  250k    s8  1M     <- 우리
```

**여기서는 `ip link` 에 `bitrate` 를 못 넘김.** 속도를 어댑터가 들고 있어서 커널
쪽에 비트타이밍이 없음.

#### 장치 이름을 고정할 것

USB 는 꽂는 순서대로 `/dev/ttyACM0`, `ttyACM1` 이 붙음. 어댑터가 둘이면 **어느 게
어느 다리인지 매번 달라짐.**

udev 규칙으로 고정함.

```bash
udevadm info -a -n /dev/ttyACM0 | grep -i serial    # 어댑터 시리얼 번호 확인
```

```
# /etc/udev/rules.d/99-canable.rules
SUBSYSTEM=="tty", ATTRS{idVendor}=="16d0", ATTRS{serial}=="<시리얼>", SYMLINK+="canable0"
```

키는 **시리얼 번호**로 잡을 것. VID/PID 로 잡으면 같은 모델 둘을 구분 못 함.

### 네이티브 CAN (HAT, MCP2515)

부팅할 때 커널 드라이버가 `canX` 를 이미 만들어 둠. 올리기만 하면 됨.

```bash
sudo ip link set can1 up type can bitrate 1000000
```

이쪽은 **커널이 비트타이밍을 들고 있어서** `ip link` 에 속도를 넘김.

### 어느 쪽이든 확인은 같음

```bash
ip -details link show can1
```

`state UP` 이 보여야 함.

### 배선이 살아 있는지

```bash
candump can1        # 다른 터미널에서. 모터가 응답하면 프레임이 보임
```

### 내릴 때

```bash
sudo ip link set can1 down
sudo pkill slcand          # USB 어댑터면 데몬도
```

채널이 안 올라와 있으면 `connect()` 가 실패하고 이 명령을 알려줌.

### 코드는 어느 쪽인지 모름

`slcand` 가 만들었든 HAT 이 만들었든 **커널 입장에서 같은 socketcan 장치**임.

```python
CanBus("can1", interface="socketcan")
```

`config/robot.yaml` 에는 채널 이름만 있음. 어댑터를 바꿔도 설정은 그대로임 —
달라지는 건 셸에서 채널 올리는 방법 하나뿐.

---

## 5. 처음 브링업 순서

1~4 가 끝났다고 보고, 로봇 앞에서 하는 순서.

각 명령이 무엇인지는 [7번](#7-명령어-목록).

**시작 전에 링크가 맞는지 볼 것.** 여기서부터 나오는 모든 명령이 이것을 따라감.

```bash
readlink config/robot.yaml      # robot_v1.0.yaml
```

```bash
# a  응답 확인 — 6개 다 응답하나
huphy-commission --limb right_leg scan
huphy-commission --limb right_leg fault

# b  관절 매핑 확인 — 어느 모터가 어느 관절인지 눈으로. 관절마다
huphy-commission --limb right_leg nudge          # 목록에서 하나씩 골라 가며

# c  영점 — 다리를 기준 자세로 손으로 잡아 놓고, 관절 전부
huphy-commission --limb right_leg zero --yes     # 관절마다 Enter, 하나씩 잡힘

# d  0도와 가동 범위 — 전부 영점 잡은 뒤. 관절마다 두 번 Enter
huphy-commission --limb right_leg sweep

# e  게인 튜닝 — 한 관절씩 골라 가며
huphy-bringup --limb right_leg --gain-scale 0.1 --allow-uncalibrated

# f  다리 전체가 버티는지 -- 6번
huphy-test --limb right_leg zero
huphy-test --limb right_leg range
```

b·c·d 의 결과는 `config/calibration/right_leg_v0.5.json` 에 바로 적힘. 붙여 넣을 것이
없음. b·c 는 관절을 생략했음 — 목록이 뜨고 골라서 진행함. 관절과 옵션을 다 적어 한 줄로
치는 형태는 [7번](#7-명령어-목록)에 있음.

### b 는 모터를 바꾸거나 왼다리를 붙일 때

오른다리 매핑은 실물에서 확인됨 ([이슈 #8](docs/issues.md)).

    7 hip_pitch   8 hip_roll   9 hip_yaw   10 knee   11 ankle_a   12 ankle_b

**모터를 교체하거나 CAN id 를 바꾸면 다시 확인할 것.** 틀리면 엉뚱한 관절이
움직이는데, 프레임은 정상적으로 나가고 응답도 와서 **코드로는 알 수 없음.**

### c 전에는 어느 자세가 0도인지 모름

영점을 안 잡으면 그 뒤가 전부 의미 없음.

다리를 원하는 자세로 손으로 잡아 놓고 **관절 전부**를 그 자세에서 잡음. 여섯 개를
한 번에 잡음 — 나눠 치면 그 사이에 다리가 무너짐.

그때 어떤 자세였는지를 적어야 나중에 재현할 수 있음. 관절을 생략하면 이어서 물어보고,
**관절마다 Enter 를 받아 하나씩 잡음.**

```
  right_leg -- 무엇을 영점 잡을까요?
  ...
  선택 [a]:

  옵션 -- 쉼표로 구분, 비우면 기본값

    1) note    (필수)     어느 자세에서 잡는지. 나중에 재현하려면 필요함

  입력 [(필수)]: 다리 편 상태, 발바닥 평면 접촉

right_leg 영점: hip_pitch, hip_roll, hip_yaw, knee, ankle_a, ankle_b
  자세: "다리 편 상태, 발바닥 평면 접촉"

  자세를 잡은 채로 관절마다 Enter.

  hip_pitch  Enter:
  hip_pitch  잡음
  hip_roll   Enter:
  hip_roll   잡음
  ...
```

토크는 **맨 앞에서 관절 전부** 한 번에 끊음. 하나 잡을 때마다 끊으면 그 사이에
자세가 무너짐.

관절별로 성공·실패가 따로 나오고, 프레임이 실제로 나간 관절만 메모가 저장됨.
하나라도 실패하면 종료 코드 `1` — 그 관절만 다시 하면 됨.

### d 가 c 뒤인 이유

`sweep` 이 재는 값은 **기계 영점을 기준으로 한 각도**임. 영점을 다시 잡으면
오프셋도 범위도 다시 재야 함.

그래서 영점을 **전부 끝낸 뒤에** `sweep` 을 한 번 돌림.

### 영점이 둘인 이유

    zero    모터가 각도를 어디부터 세는지        모터 플래시에 남음
    sweep   그 각도 중 어디를 관절 0도로 부를지   캘리브레이션 파일에 남음

`cal = sign x raw + offset` 에서 `sweep` 이 정하는 것이 `offset` 임. 기계 영점을
제대로 잡았으면 `sweep` 의 offset 이 **0 근처로 나옴** — 그것이 곧 확인임.

`sign` 은 설계가 정한 값이라 손댈 일이 없음. 지금 1 임.

### 무엇을 보나

**`commission scan`** — 응답하지 않는 모터를 냄.

```
응답 없음: ['ankle_b']
```

**`commission nudge knee`** — 명령한 만큼 안 움직이면 알려줌.

```
시작    29.99
최대    34.72   (움직인 양 +4.73)
끝      30.23
```

**`commission sweep`** — 관절마다 0도를 정하고, 그 기준으로 최대·최소를 기록함.

```
  [4/5] knee
       0도 자세로 두고 Enter:
       knee       offset   -33.40

       양쪽 끝까지 미세요. 끝나면 Enter.

       관절                최소        지금        최대        범위
       knee       -20.61     30.12     74.76     95.37
```

끝나면 `robot.yaml` 에 붙일 수 있는 형태로 냄. **파일을 고치지는 않음** — 주석이
많은 파일이라 프로그램이 다시 쓰면 날아감.

**`bringup`** — 그래프를 보며 게인을 찾음.

```
1  loop_dt 부터 확인      주기를 못 지키면 게인 문제가 아님
2  자세 유지              처지나, 떨리나
3  계단 응답              여기서 대부분이 결정됨
4  사인파                 추종 지연과 진폭 감쇠
```

찾은 값은 `config/robot.yaml` 의 `kp`/`kd` 에 적음. 무엇을 보고 어떻게 판단하는지는
[`control/README.md`](src/huphy/control/README.md) 와
[`docs/monitoring.md`](docs/monitoring.md).

---

## 6. 동작 확인

5번이 끝났으면 다리가 움직임. **정해진 패턴을 계속 돌려 놓고 보는 자리**임.

```bash
huphy-test --limb right_leg zero      # 관절 전부를 0도로 두고 붙잡음
huphy-test --limb right_leg range     # 관절마다 최소~최대를 오감
huphy-test --limb right_leg pose 0 10 0 30 0 0    # 준 각도로 옮기고 붙잡음
huphy-test --robot zero               # 양다리를 한 루프로
```

**Ctrl-Q 를 누를 때까지** 계속함. 사람이 답을 고르는 자리가 없어서 다리를 보거나
그래프를 보는 데 손을 쓸 수 있음. `bringup` 이 한 관절씩 게인을 찾는 자리라면
여기는 **찾은 게인으로 다리 전체가 버티는지** 보는 자리임.

### `zero` — 자세가 유지되는가

관절 전부가 0도라 **어긋난 관절이 눈으로 바로 보임.**

    처지면        kp 가 부족
    부르르 떨면    kp 가 과함
    한쪽만 틀어짐  그 관절의 영점이나 매핑

### `range` — 끝까지 가는가

관절마다 최소~최대를 사인파로 오감. 설정한 한계까지 실제로 도달하는지, 도중에
걸리는 데가 없는지, 양 끝에서 부딪히는 소리가 나지 않는지 봄.

```
  관절                최소        최대
  hip_pitch  -112.07    -26.07
  knee       -15.65     69.79
  ankle_pitch    -35.00     35.00
```

**한계가 없는 관절은 빠짐** — 어디까지 가도 되는지 모르는 관절을 흔들 수 없음.
화면에 어느 관절을 뺐는지 냄. 왼다리는 아직 전부 빠짐.

발목만 캘리브레이션 파일이 아니라 기구학 쪽 시험 범위를 씀. 모터 두 개가 로드로
물려 있어 **모터 한계를 관절 한계로 옮길 수 없기** 때문임 — 한 모터의 최대각이
다른 모터의 자세에 따라 달라짐.

### `pose` — 원하는 자세를 직접

각도를 두 방식 중 하나로 줌. 섞으면 거부함.

```bash
huphy-test --limb right_leg pose 0 10 0 30 0 0                 # 6개를 순서대로
huphy-test --robot pose 0 10 0 30 0 0  0 10 0 30 0 0           # 12개. 왼다리 먼저
huphy-test --robot pose left_leg/ankle_a=10 right_leg/knee=-20 # 나머지는 0
```

숫자 순서는 `hip_pitch hip_roll hip_yaw knee` 다음 발목 두 개이고, 로봇 전체면
**왼다리 6개 다음 오른다리 6개**임. 양다리 모델의 출력 순서와 같아서 모델이 낼
자세를 그대로 옮겨 넣을 수 있음.

**발목 두 칸은 기본이 모터 각도**임 (`ankle_a`/`ankle_b`). 준 값이 그대로 모터로
나감.

```bash
huphy-test --robot pose --ankle-space rp left_leg/ankle_pitch=10 left_leg/ankle_roll=0
```

`--ankle-space rp` 를 주면 발판 자세로 받아 기구학을 거쳐 모터각으로 바뀜. 그때만
발목 한계가 실측값이 아니라 `AnkleEnvelope` 의 시험 범위임 — 모터 두 개가 물려 있어
한 모터의 최대각이 다른 모터의 자세에 따라 달라져서, 모터 한계를 관절 한계로 옮길 수
없기 때문임.

시작 전에 목표 전부를 표로 찍고 한계 밖이면 표시함. 음수는 그냥 치면 됨 (`-10`).

### `--robot` — 양다리를 한 루프로

```bash
huphy-test --robot zero
huphy-test --limb left_leg,right_leg zero    # 일부만 묶기 (드묾)
```

프로세스를 둘 띄우는 것과 다름 — **명령이 같은 주기에 나가고, 한쪽 통신이 끊기면
양쪽이 같이 멈춤.** 따로 띄우면 한쪽이 죽어도 다른 쪽은 계속 움직여 넘어짐.

`--robot` 이면 관절 이름 앞에 다리가 붙음 (`right_leg/knee`). 다리 하나면 안 붙음.
주기는 팔다리 중 가장 느린 `control_hz` 에 맞춤.

`--robot` 과 `--limb` 을 같이 주면 거부함.

### 시작할 때 천천히 감

지금 자세가 어디든 목표까지 `--approach` 초(기본 3)에 걸쳐 옮긴 뒤에 패턴을 시작함.
토크를 넣는 순간 목표가 멀리 있으면 관절이 튐 — 점프 가드가 자르기는 하지만 그 전에
큰 토크가 한 번 나감.

### 끝나면 실패 집계를 찍음

텔레메트리를 켜지 않아도 **이 주기로 돌렸을 때 몇 번 빠졌는지** 보임.

```
  주기       1000회 10.0초  평균 99.8Hz / 목표 100Hz  밀림 3회  최악 14.2ms
  수거 대기  2ms x 2다리 = 최악 4ms  (주기 10.0ms 의 40%)
  무응답     12주기 (1.2%)

  채널        송신실패  수신실패  대기초과      버림
  can1               0         0         7         0
  can0               0         0         5         0

  모터                      빠짐    명령    비율
  left_leg/ankle_a             7    1001    0.7%

  가드 자름   limit 4
  가드 거부   없음
```

| 줄 | 무엇 |
|---|---|
| 밀림 | 목표 주기를 크게 넘긴 횟수 |
| 대기초과 | 기다린 개수를 못 채우고 수거 대기를 다 쓴 횟수 |
| 버림 | 수신 스레드 큐가 차서 버린 프레임. `--robot` 일 때만 |
| 모터별 | 명령한 횟수 대비 응답이 없던 횟수. **빠진 모터만** 나옴 |

**연결한 뒤부터의 누적**임. 연결 직후 상태 조회와 영자세로 옮기는 구간이 들어감.

`huphy-run` 도 같은 집계를 찍음.

### `--drain-ms` — 응답을 기다리는 최대 시간

```bash
huphy-test --robot --hz 100 --drain-ms 4 zero
```

기본 2ms. 기다린 개수가 다 오면 즉시 빠져나가므로 **정상 주기에는 이만큼 쓰지
않음.**

| | |
|---|---|
| 늘리면 | 늦게 온 응답을 그 주기에 받아 무응답·대기초과가 줄어듦 |
| 대신 | 모터 하나가 죽으면 매 주기 이만큼을 통째로 써서 주기가 밀림 |

양다리는 다리마다 따로 기다려 **최악 두 배**임. 주기의 절반을 넘으면 집계에 경고가
찍힘. `--hz` 와 같이 바꿔 가며 실패 집계를 비교하는 용도임.

### 플래그

| | 무엇 |
|---|---|
| `--approach 3` | 시작 자세까지 옮기는 시간 |
| `--period 6` | `range` 한 번 왕복하는 시간. 길수록 천천히 |
| `--margin 5` | 한계에서 남길 여유. 한계는 하드스톱을 잰 값이라 그대로 명령하면 부딪힘 |
| `--drain-ms 2` | 응답을 기다리는 최대 시간 (ms) |
| `--robot` | 다리 전부를 한 루프로. `--limb` 과 같이 못 씀 |
| `--gain-scale` `--hz` `--allow-uncalibrated` | 브링업과 같음 |

플래그는 서브명령 앞뒤 어디에 적어도 됨.

```bash
huphy-test --limb right_leg range --period 10
huphy-test range --limb right_leg --period 10
```

---

## 7. 명령어 목록

### 커미셔닝 — 조립할 때 한 번

**참조용 목록임. 실제로 치는 순서는 [5번](#5-처음-브링업-순서).**

관절 이름은 **생략할 수 있음.** 빼고 치면 관절 목록, 이어서 옵션을 띄우고 고르게 함.

```
  right_leg -- 무엇을 움직일까요?

    1) hip_pitch  id=7   RS02
    ...
    6) ankle_b    id=12  RS00

  선택: 4

  옵션 -- 쉼표로 구분, 비우면 기본값

    1) delta   5.0      몇 도 움직였다 되돌릴지. 20도까지
    2) kp      5.0      위치 게인. 안 움직이면 조금씩 올릴 것
    3) kd      0.5      속도 게인

  입력 [5.0, 5.0, 0.5]: 3

  실행: huphy-commission --limb right_leg nudge knee --delta 3.0 --kp 5.0 --kd 0.5
```

`clear-fault` `sweep` `zero` 는 관절을 Enter 만 쳐도 전부로 감. 옵션은 빈 칸이
기본값이고, 끝에 같은 뜻의 명령줄을 내주므로 다음부터는 아래 목록대로 바로 쳐도 됨.
관절을 명령줄에 적으면 옵션은 묻지 않음.

#### 읽기만 함 — 아무것도 안 움직임

```bash
huphy-commission --limb right_leg scan
```
어느 모터가 응답하는지. **가장 먼저 치는 것.** 빠지면 배선·전원·CAN id·프로토콜이
후보인데, 넷이 여기서 구분되지 않음.

```bash
huphy-commission --limb right_leg state
```
지금 각도·속도·토크·온도. `raw` 와 `cal` 을 나란히 냄. 손으로 관절을 움직이며
값이 따라오는지 볼 때도 씀.

```bash
huphy-commission --limb right_leg fault
```
고장 비트. `과열`, `스톨`, `저전압` 등. 응답이 없는 것과 고장이 없는 것은 다르게 냄.

#### 고치기

```bash
huphy-commission --limb right_leg clear-fault          # 전부
huphy-commission --limb right_leg clear-fault knee     # 하나만
```
고장 상태를 지움. **원인이 남아 있으면 다시 뜸.**

#### 재기 — 토크를 끄고 사람이 움직임

```bash
huphy-commission --limb right_leg nudge knee --delta 5
```
모터 하나를 5도 움직였다 되돌림. **어느 모터가 어느 관절인지 눈으로 확인하는 용도.**
낮은 게인(`kp=5`)으로 살살 밀고, 명령한 만큼 안 움직이면 알려줌. 20도까지만 허용함.

```bash
huphy-commission --limb right_leg sweep                # 전 관절
huphy-commission --limb right_leg sweep knee           # 하나만
huphy-commission --limb right_leg sweep 10             # id 로도 됨
```
토크를 끄고 **관절마다 두 가지를 정함.** 초당 20번 재고, 시작할 때 화면에 알려줌.

```
  [4/5] knee
       0도 자세로 두고 Enter:            <- 여기를 관절 0도로 부름
       knee       offset   -33.40

       양쪽 끝까지 미세요. 끝나면 Enter.

       관절                최소        지금        최대        범위
       knee       -20.61     30.12     74.76     95.37
```

**0도를 먼저 받는 이유**: 그 기준으로 최대·최소를 기록하므로 화면에 뜬 값이 곧
관절 좌표계 각도임. `robot.yaml` 에 그대로 옮기면 됨.

`ankle_a` 과 `ankle_b` 는 한 단계로 묶임 — 로드로 발판에 물려 있어 한쪽만 손으로
돌릴 수 없음. 발을 잡고 움직이면 두 범위가 한 번에 나옴.

끝나면 오프셋은 캘리브레이션 파일에 적고, 한계각은 `robot.yaml` 에 붙일 형태로 냄 —
**`robot.yaml` 은 고치지 않음.**

**기계 영점을 잡은 뒤에 해야 함.** 영점을 다시 잡으면 오프셋과 한계각을 둘 다 다시 재야 함.

#### [영구] 영점 — `--yes` 가 있어야 나감

되돌리기 어려움. 승인 확인이 **버스를 열기 전에** 일어남.

```bash
huphy-commission --limb right_leg zero                        # 전부, 관절마다 Enter
huphy-commission --limb right_leg zero knee --note "..." --yes  # 하나만
```
지금 자세를 그 모터의 0도로 잡음. **전원을 꺼도 남음.** `--note` 는 그때 다리가
어떤 자세였는지 — 모터는 값만 저장하고 자세는 아무 데도 안 남으므로, 나중에 모터를
갈 때 재현하려면 이 메모가 필요함. 캘리브레이션 파일에 자동으로 적힘.

토크가 켜져 있으면 거부함 — 좌표계가 옮겨가는데 직전 목표각은 옛 좌표계 값이라
그 차이만큼 관절이 튐. 그래서 관절 전부의 토크를 **맨 앞에서 한 번에** 끊음.

관절을 생략하면 한 번 실행에 전부를 잡되 **관절마다 Enter** 를 받음. 자세를 잡은
채로 여섯 번 나눠 명령을 치는 것은 불가능함. 결과는 관절별로 나오고, 프레임이 실제로
나간 관절만 메모가 저장됨. 하나라도 실패하면 종료 코드 `1`.

#### 모터 자체의 설정 — 평소에 쓸 일 없음

**모터가 어떻게 말하고 어떻게 움직이는지를 바꿈.** 브링업이나 제어와는 상관없고,
모터를 새로 사거나 갈아 끼울 때만 씀. 세 개 다 **한 관절씩만** 다룸 — 전부를 고를 수
없게 해 뒀음.

잘못 건드리면 **그 모터가 통신에서 사라짐.** 그때 `scan` 은 "응답 없음" 만 내는데,
배선·전원·CAN id·프로토콜이 전부 후보라 여기서는 구분되지 않음.

```bash
huphy-commission --limb right_leg mode knee --to mit
```
제어 모드. 본 프로젝트는 `mit` 을 씀 — 전원 투입 기본값이기도 해서 손댈 일이 거의
없음. 즉시 적용되고 되돌릴 수 있음. `--to` 를 생략하면 `mit` 임.

다른 모드로 가 있으면 MIT 프레임을 보내도 **모터가 조용히 무시함.** 프레임은
정상적으로 나가고 CAN 에러도 안 뜨는데 응답만 안 옴.

```bash
huphy-commission --limb right_leg can-id knee --to 20 --yes
```
모터의 CAN id 변경. **바꾼 뒤 `robot.yaml` 의 그 관절 `id` 도 고쳐야 함** — 안 고치면
설정이 옛 번호를 부르므로 그 관절만 통째로 응답이 없음.

이미 쓰는 id 는 거부함 — 같은 id 가 둘이면 응답이 충돌해 어느 쪽인지 구분조차 안 됨.
1~127 밖도 거부함.

```bash
huphy-commission --limb right_leg protocol knee --to mit --yes
```
통신 프로토콜. **가장 위험함.** 전원을 재투입해야 적용되고, 그 전까지는 옛 포맷으로
통신함. `--to` 에 기본값을 두지 않았음 — 어느 쪽으로 갈지를 대신 정해 줄 수 없음.

바뀌었는지는 **코드로 확인 못 함** ([이슈 #11](docs/issues.md)). 되돌리려면 반대
포맷으로 다시 보내야 하는데, 그 시점엔 모터가 이미 다른 포맷으로 듣고 있음.

#### `--limb` 은 생략할 수 없음

다리마다 CAN 채널이 달라 **잘못 고르면 엉뚱한 다리가 움직임.** 팔다리가 하나뿐인
설정에서만 생략됨.

### 브링업 — 반복해서 움직여 봄

```bash
# 처음 만질 때
huphy-bringup --limb right_leg --gain-scale 0.1 --allow-uncalibrated

# 실측이 끝난 뒤
huphy-bringup --limb right_leg
```

| 플래그 | 무엇 |
|---|---|
| `--gain-scale 0.1` | 게인을 10% 로 낮춰 시작 |
| `--allow-uncalibrated` | 실측 전에도 토크를 넣음 |
| `--hz 200` | 제어 주기. 기본은 설정의 `control_hz` |
| `--no-precise` | 마감 직전 스핀을 끔. CPU 가 빠듯하면 |

**영점 메모가 비어 있으면 `--allow-uncalibrated` 없이는 토크 항목이 거부됨.**
`commission zero` 를 실물에서 돌리기 전까지는 필요함.

메뉴:

```
1. 상태 보기          raw/cal, 속도·토크·온도, ack·age, 발목 pitch/roll
2. 카운터 보기        가드·CAN 카운터, 루프 통계
3. 자세 유지 [토크]    처지나, 떨리나
4. 한 관절 옮기기 [토크]
5. 계단 응답 [토크]    게인 튜닝의 핵심
6. 사인파 왕복 [토크]
```

### 동작 확인 — 정해진 패턴을 계속

```bash
huphy-test --limb right_leg zero        # 관절 전부를 0도로 두고 붙잡음
huphy-test --limb right_leg range       # 관절마다 최소~최대를 오감
huphy-test --limb right_leg pose 0 10 0 30 0 0     # 준 각도로
huphy-test --robot zero                 # 다리 전부를 한 루프로
huphy-test --robot --hz 200 --drain-ms 4 zero      # 주기·수거 대기를 바꿔 가며
```

Ctrl-Q 까지 계속함. 끝나면 실패 집계를 찍음. 자세한 것은 [6번](#6-동작-확인).

### 정책 실행 — 학습한 모델로 움직임

```bash
huphy-run --limb right_leg --policy balance                        # 다리 하나
huphy-run --limb right_leg --policy balance --ankle-output torque  # 발목을 토크로
huphy-run --robot --policy balance --weights runs/biped.pt         # 양다리 12칸
```

`.pt` 에는 신경망만 들어 있고 `action_scale` 이나 관찰 구성은 안 들어 있음. 그건
`--policy` 이름이 정함 (`balance`, `hopping`). **파일의 입력·출력 개수가 그 규격과
다르면 모터를 켜기 전에 멈춤.**

영자세로 `--approach` 초에 걸쳐 옮긴 뒤 **Enter 를 눌러야** 정책이 시작됨. Ctrl-C
로 멈추고, 멈출 때 자세를 붙잡은 뒤 토크를 끊음.

| | 무엇 |
|---|---|
| `--policy` | **필수.** 어느 규격인지. 관찰 개수와 `action_scale` 이 여기서 정해짐 |
| `--weights` | 기본값은 `config/policies/<정책이름>.pt` |
| `--ankle-space rp` | 모델이 발목을 무엇으로 내는지. `rp` 발판 자세 / `ab` 모터 각도 |
| `--ankle-output position` | 발목에 각도를 보낼지 토크를 보낼지. `torque` 는 `rp` 에서만 |
| `--robot` | 양다리 모델(12칸). 왼다리 6개 → 오른다리 6개 순서 |
| `--hz 50` | 제어 주기. **학습 주기와 같아야 함** |
| `--duration` `--approach` `--drain-ms` `--allow-uncalibrated` | 위와 같음 |

**`--ankle-space` 를 잘못 골라도 코드가 못 잡음.** 두 공간이 관찰 개수도 행동
개수도 같아서 가중치 검사를 그냥 통과함. 시작 화면에 어느 쪽인지 찍으므로 눈으로
확인할 것. 설정 파일로 옮기는 설계는 [`docs/policy_runner.md`](docs/policy_runner.md).

IMU 가 없으면 시작 전에 멈춤 — 관찰 24칸 중 6칸이 IMU 값임 (각속도 3, 중력방향 3).

### IMU — 센서를 설정하고 확인

```bash
huphy-imu show          # 센서 설정을 읽어 robot.yaml 과 대조. 안 바꿈
huphy-imu apply --yes   # [영구] robot.yaml 대로 센서를 맞춤
huphy-imu check         # 부착 방향을 가속도계와 대조. 안 바꿈
huphy-imu watch         # 들어오는 값을 계속 보여줌
```

**설정이 센서 비휘발성 메모리에 저장됨.** 전원을 껐다 켜도 남으므로 `apply` 는
`--yes` 를 요구함.

`check` 는 **두 축을 동시에 기울여 가만히 둔 상태**에서 실행할 것. 정지 상태의
가속도계가 중력방향을 직접 재므로, 자세에서 계산한 값과 대조하면 센서가 예상과
다른 방향으로 붙었는지 드러남. 수평이면 부착이 틀려도 통과하므로 판정을 거부함.

IMU 가 여럿이면 `--imu <이름>` 으로 고름.

### 공통

```bash
--config <경로>    기본값: 위로 올라가며 config/robot.yaml 을 찾음
-v                 자세한 로그
```

`--config` 로 하드웨어 버전을 골라 씀 ([하드웨어 버전 바꾸기](#하드웨어-버전-바꾸기)).

---

## 하드웨어 버전 바꾸기

**설정 파일만 바뀌고 코드는 그대로임.** 스크립트는 현재 폴더부터 위로 올라가며
`config/robot.yaml` 이라는 **이름 하나만** 찾음.

```
config/
├── robot.yaml           -> robot_v1.0.yaml   심볼릭 링크. 지금 쓰는 쪽
├── robot_v0.5.yaml         RS02 x4 + RS00 x2
├── robot_v1.0.yaml         RS04 x3 + RS03 x3
└── calibration/
    ├── right_leg_v0.5.json     한계 6/6 잼
    ├── left_leg_v0.5.json      아직 안 잼
    ├── right_leg_v1.0.json     아직 안 잼
    └── left_leg_v1.0.json      아직 안 잼
```

### 바꾸기

```bash
ln -sf robot_v1.0.yaml config/robot.yaml     # 1.0 으로
ln -sf robot_v0.5.yaml config/robot.yaml     # 0.5 로
readlink config/robot.yaml                   # 지금 어느 쪽인지
```

### 링크를 안 건드리고 한 번만

네 진입점 전부 `--config` 를 받음.

```bash
huphy-commission --config config/robot_v1.0.yaml --limb right_leg scan
huphy-bringup    --config config/robot_v1.0.yaml --limb right_leg
huphy-imu        --config config/robot_v1.0.yaml show
huphy-test       --config config/robot_v1.0.yaml --limb right_leg zero
```

### 실측값은 같이 못 씀

`sign` / `offset_deg` / `limits_deg` 는 전부 **조립 결과**라 모터를 갈면 무효임.
그래서 캘리브레이션 파일도 버전마다 따로 있고, 각 `robot_vX.Y.yaml` 이 자기 것을
가리킴.

파일 이름에 버전이 붙어 있어 **덮어쓸 수 없음** — 1.0 커미셔닝을 하다 0.5 로
되돌려도 0.5 의 실측값이 그대로 남아 있음.

### 1.0 으로 바꾼 뒤 할 일

1.0 캘리브레이션은 `limits_deg` 가 전부 `null` 인 빈 틀임. `null` 은 "제한 없음" 이
아니라 **"아직 안 잼"** 이고, 그 상태에서는 `Motor.is_configured` 가 `False` 라
제어 진입이 막힘.

```
1  프로토콜을 MIT 로, CAN id 를 7~12 로   docs/motor_setup.md
2  zero_sta 를 1 로                       2번
3  huphy-imu apply --yes                  IMU 출력 설정
4  브링업 순서를 처음부터                  5번
```

**`model` 이 실물과 다르면 조용히 틀린 토크가 나감.** RS03 은 ±60, RS04 는 ±120 N·m
이라 RS02(±17) 설정으로 RS04 를 돌리면 시킨 것의 7배가 나감. 프레임도 응답도
정상이라 실물에서 찾기 매우 어려움.

---

## 조정해야 할 값

### `config/robot.yaml` — 사람이 적는 것

링크가 가리키는 파일임 (`robot_v0.5.yaml` 또는 `robot_v1.0.yaml`).

| 값 | 언제 고치나 | 지금 |
|---|---|---|
| `kp` / `kd` | 튜닝할 때 | 0.5 오른다리 20 / 1, 1.0 양다리 10 / 1 (둘 다 튜닝 전) |
| `command_margin_deg` | 게인을 바꾸면 다시 봄 | 3.0 |
| `max_delta_deg` | 주기를 바꾸면 다시 봄 | 50.0 |
| `channel` | 배선이 바뀌면 | `can1` / `can0` |
| `control_hz` | — | 100.0 |
| `telemetry.host` | 그래프를 볼 때 | 비어 있음 (UDP 꺼짐) |

**`kp`/`kd` 는 튜닝 시작값임** — 여기서부터 올리거나 내리는 자리이지 찾은 값이
아님. 처음 만질 때는 `--gain-scale 0.1` 로 더 낮춰 시작할 것.

왼다리는 0으로 비워 둠. 0은 "안 정해짐" 이 아니라 **"힘 없음"** 임 — 명령을 보내도
아무 힘이 안 나가고, `Motor.is_configured` 가 `False` 라 제어 진입 자체가 막힘.
한계를 모르는 관절에 게인만 넣으면 어디까지 가도 되는지 모르는 채로 토크가 나감.

### `config/calibration/*.json` — 로봇을 만져서 알아내는 것

**프로그램이 씀.** 손으로 고칠 일이 없음.

**버전·다리마다 파일 하나임** (`right_leg_v0.5.json` 등). 모터를 갈면 값이 전부
무효라 같이 못 씀 ([하드웨어 버전 바꾸기](#하드웨어-버전-바꾸기)).

| 값 | 누가 적나 | 지금 |
|---|---|---|
| `limits_deg` | `commission sweep` | 0.5 오른다리만 있음. 나머지 셋은 `null` |
| `offset_deg` | `commission sweep` | 전부 0.0 (재기 전) |
| `zero_reference` | `commission zero` | 전부 비어 있음 |
| `sign` | 설계. 쓰는 코드가 없음 | 전부 1.0 |

**두 파일을 나눈 이유** — 숫자에 두 종류가 있음.

```
도면 보고 적는다        ->  robot.yaml
로봇을 만져서 알아낸다   ->  calibration/*.json
```

`robot.yaml` 은 주석이 많아서 **프로그램이 다시 쓰면 주석이 전부 날아감.** 그래서
프로그램은 JSON 만 씀. `limits_deg` 를 `robot.yaml` 에 적으면 거부함 — 같은 값이
두 군데 있으면 어긋났을 때 어느 쪽이 진짜인지 알 수 없음.

게인만 예외임. 실물에서 찾는 값이지만 사람이 주석과 함께 손으로 적는 값이라
`robot.yaml` 에 둠.

자세한 것은 [`config/README.md`](config/README.md).

---

## 지금 무엇이 비어 있나

| | 무엇이 막히나 | 어디서 채우나 |
|---|---|---|
| **게인 미튜닝** (`kp = 20`, 시작값) [#9] | 너무 크면 튀고 작으면 처짐 | `bringup` 으로 튜닝 |
| **영점 미실측** (`zero_reference` 비어 있음) [#9] | `cal` 이 `raw` 와 같음. 좌표계가 없는 것과 같음 | `commission zero` |
| **발목 기하 출처 미확인** [#13] | 어느 다리 것인지 모름. 반대쪽은 계산으로 만든 거울상 | 발 각도를 재서 |
| **양다리 한계 없음** (1.0) [#9] | 제어 진입이 막힘. 모터를 갈아 0.5 값을 못 씀 | `commission sweep` |
| **전제를 코드로 못 읽음** [#11] | `zero_sta` 와 프로토콜을 확인할 방법이 없음 | 외부 도구로 대체 중 |

**전부 실물이 있어야 채워지는 것들임.** 코드로 할 수 있는 것은 다 되어 있고,
`--allow-uncalibrated` 로 넘겨야 실측을 시작할 수 있음.

번호와 근거는 [`docs/issues.md`](docs/issues.md).

---

## 계층 구조

```
scripts/          터미널 진입점
   │
   ├─ commission.py   조립할 때 한 번 하는 조작
   ├─ bringup.py      하나씩 골라 움직여 보는 메뉴
   ├─ selftest.py     정해진 패턴을 Ctrl-Q 까지
   └─ run.py          학습한 정책으로 움직임
   │
control/          제어 루프. 주기와 안전
   │
   ├─ motions.py      매 주기 무엇을 시킬지
   ├─ policy.py       학습한 정책을 Motion 으로
   └─ rsl_rl.py       체크포인트(.pt) 읽기. numpy 만 씀
   │
robots/           ─── 관절 이름 ↔ 모터 id, cal ↔ raw 경계 ───
   │
   ├─ kinematics/     발목 pitch/roll ↔ a1/a2
   ├─ safety/         한계·점프·NaN 검사
   ├─ config/         robot.yaml 읽기
   └─ calibration/    실측값 읽기·쓰기
   │
motors/           모터 id 와 raw 각도만 앎
   │
   ├─ base.py         벤더 중립 자료형
   ├─ canbus.py       CAN 전송. python-can 유일 사용처
   └─ robstride/      벤더 사양, 코덱, 버스, 커미셔닝
   │
sensors/          모터가 아닌 센서. IMU
   │
   ├─ base.py         ImuState, Imu. 벤더 중립
   ├─ registry.py     model 문자열 -> 구현체
   ├─ ebimu/          E2BOX EBIMU-9DOF. ASCII 시리얼. 지금 쓰는 것
   └─ xsens/          Xsens MTi. 시리얼 Xbus
   │
telemetry/        옆에서 관찰. 제어를 방해하지 않음
```

### 어느 계층이 무엇을 아나

| 계층 | 아는 것 | 모르는 것 |
|---|---|---|
| `control/` | 시간, 주기 | 관절 이름도, 모터도 모름 |
| `robots/` | 관절 이름, cal 각도 | 바이트, 프레임 |
| `motors/` | 모터 id, raw 각도 | "무릎" 이 무엇인지 |
| `canbus.py` | 8바이트와 CAN id | 바이트의 뜻 |

**`robots/` 가 경계임.** 위는 관절로 말하고 아래는 모터로 말함.

### `python-can` 을 쓰는 곳

```
canbus.py    ← 여기 하나뿐
```

그 위는 `CanFrame` 만 다룸. 그래서 **테스트 956개가 `python-can` 없이 돌아감.**

---

## 한 주기에 무슨 일이 일어나나

```
ControlLoop.run()
  │
  ├─ motion(t, obs)                   무엇을 시킬지          control/motions.py
  │     -> {"knee": 30.0, ...}        관절 이름, cal 공간
  │
  ├─ leg.build_commands(action)       계산만. CAN 안 씀      robots/leg.py
  │     │
  │     ├─ 발목 pitch/roll -> a1/a2                          kinematics/ankle.py
  │     ├─ 한계·점프·NaN 검사 (cal 공간)                       safety/guards.py
  │     ├─ cal -> raw                                        calibration
  │     └─ MitCommand                                        robstride/bus.py
  │
  ├─ leg.send(commands)               전송만                 robstride/bus.py
  │     └─ pack_command -> 8바이트                            robstride/codec/mit.py
  │           └─ CanBus.send_many                            motors/canbus.py
  │
  ├─ leg.collect()                    수거. 상태 갱신
  │     └─ CanBus.drain -> decode_state
  │
  ├─ telemetry.record()               기록. 읽기만 함        telemetry/
  │
  └─ 다음 주기까지 기다림
```

### 계산·전송·수거를 나눈 이유

버스가 둘일 때 이 순서를 짜야 함.

```
왼다리 계산 -> 오른다리 계산 -> 왼다리 전송 -> 오른다리 전송 -> 수거
```

한 함수가 셋을 다 하면 **두 다리의 명령 시각이 벌어짐** — 수거는 큐가 빌 때까지
기다리므로 그 시간이 그대로 오른다리 전송 지연이 됨.

다리 하나뿐이면 `send_action()` 하나로 충분함.

---

## 왜 이렇게 나눴나

각 계층을 만들 때 무엇에 중점을 뒀는지.

### `safety/` — 조용한 실패를 막음

**NaN 하나가 720도 명령이 됨.**

```python
min(10, nan)                             # 10      비교가 False 라 통과
float_to_uint(nan, -12.57, 12.57, 16)    # 65535 = 720도
```

파이썬의 `min`/`max` 가 NaN 을 통과시키므로 인코딩 단계의 클램프가 무력화됨.
그래서 **유한값 검사가 첫 관문**임.

**버리지 않고 자름.** 명령을 버리면 그 모터만 직전 명령을 유지해 다리 자세가
어긋남 — 발목처럼 두 모터가 연동된 곳에서 특히 나쁨.

자른 것은 반드시 세어 내보냄. 클리핑은 **조용한 변조**이기 때문임.

### `motors/` — 벤더 중립과 전송 격리

**적는 것과 재는 것을 나눔.** `Motor` 는 사람이 적고 `MotorCalibration` 은 조립을
잼. 무효화 시점이 달라서 한 파일에 두면 한쪽을 고칠 때 다른 쪽을 덮어씀.

**`python-can` 을 `canbus.py` 안에 가둠.** 위 계층은 `CanFrame` 만 다룸.
`codec/mit.py` 가 라디안 변환을 혼자 떠안는 것과 같은 방식임.

**전송과 수거를 나눔.** `recv()` 는 큐가 비면 타임아웃만큼 블로킹하므로, 순차로
수거하면 그 시간이 버스 수만큼 곱해짐.

### `robstride/` — 벤더 사양을 데이터로

**프로토콜과 제어 모드는 다른 축임.** `Protocol` 은 프레임 포맷, `ControlMode` 는
무엇을 명령할지. 이름이 겹치지만 독립임.

**인코딩 범위가 `[프로토콜][모델]` 임.** 예전 매뉴얼 판본은 같은 RS02 를 MIT 은 ±33 rad/s, private
은 ±44 rad/s. 이 축이 없으면 private 값을 MIT 에 가져다 쓰는 실수가 남.

**되돌리기 어려운 조작을 격리함.** 영점·CAN id·프로토콜 전환은 `commissioning.py`
로 감. `MotorsBus` 계약에 없으므로 제어 코드에서 **부를 방법 자체가 없음.**

### `config/` — 오타를 읽는 순간 잡음

YAML 은 모르는 키를 조용히 넘김.

```
contorl_hz: 200     ->  무시되고 기본값 100Hz 로 돎
```

**설정을 고쳤는데 아무것도 안 바뀜.** 증상이 "느리다" 로 나타나므로 원인을
설정에서 찾을 이유가 없어 오래 걸림. 그래서 모르는 키가 있으면 멈춤.

**기본값은 스키마에만 둠.** 두 군데 있으면 어느 쪽이 쓰이는지 알 수 없음.

### `kinematics/` — 자기일관성을 고정함

발목만 있음. 다른 관절은 모터 하나가 관절 하나를 돌리므로 변환할 것이 없음.

**두 모터 각도를 같은 규약(`[-180,180)`)으로 냄.** 한쪽만 `[0,360)` 이면 IK 가
340도를 돌려주고 모터는 -20도를 보고해 360도 차이가 남.

**FK 는 답이 하나가 아님.** 같은 모터각 조합이 서로 다른 자세 둘에 대응함.
링키지의 성질이지 버그가 아님 — 시험 범위 안에서는 문제가 없다는 것을 격자 187개로
확인함.

### `robots/` — 경계를 한 곳에 모음

네 가지가 **여기서만** 일어남: 관절 이름 → 모터 id, cal → raw, 발목 IK, 안전 검사.

**한계 검사가 cal 공간에서 일어남.** raw 로 내린 뒤 검사하면 `sign` 이 -1 인
관절에서 부호가 뒤집혀 한계가 반대로 걸림.

**실제로 나간 명령을 돌려줌.** 무엇을 보냈는지가 아니라 **무엇이 실행됐는지**를
기록해야 로그를 믿을 수 있음.

### `telemetry/` — 제어보다 먼저 만듦

게인을 튜닝하려면 목표와 실측을 겹쳐 봐야 함. **그래프가 없으면 게인을 찾을 수
없고, 게인이 없으면 다리가 안 움직임.**

**필드 이름을 한 곳에서만 정함.** 두 군데에서 만들면 CSV 헤더에는 있는데 UDP 에는
없는 값이 생김.

**예외를 던지지 않음.** 네트워크가 끊기거나 디스크가 차는 것은 정상 상황임. 관측이
제어를 멈추면 관측할 대상이 없어짐.

**패킷을 둘로 나눔.** 한 다리가 필드 66개면 MTU(1500)를 넘어 조각나고, 조각 하나만
잃어도 패킷 전체가 버려짐.

### `control/` — 주기를 정직하게 잼

**두 가지를 따로 봄.**

```
overruns   튀는 주기.     목표의 1.5배를 넘긴 횟수
kept_up    꾸준한 느림.   평균이 목표의 90% 미만
```

매 주기 24%씩 넘으면 **한 번도 "밀림" 으로 세지 않으면서** 주파수만 떨어짐.
주기가 밀리는데 게인을 튜닝하면 게인이 아니라 주기가 문제인데 게인을 계속 만지게 됨.

**마감 직전은 자지 않고 돌면서 기다림.** `time.sleep` 은 요청한 만큼 정확히 자지
않아서, 100Hz 에서 84.7Hz 가 나옴. 고치면 99.9Hz.

**멈출 때 자세를 먼저 붙잡음.** 서 있는 다리에서 힘이 갑자기 빠지면 주저앉음.
예외로 빠져나가도 같은 순서를 탐.

### `scripts/` — 움직이는 것은 전부 루프를 탐

진입점이 로봇을 직접 부르면 그 경로에서만 텔레메트리·주기 측정·정지 순서가 빠짐.
**그러면 그래프가 안 나오는데 텔레메트리가 고장난 줄 알게 됨.**

진입점은 `Motion` 만 정하고 루프에 넘김. 테스트가 `ControlLoop.run` 을 감시해 이것을
고정함.

`bringup` 은 한 관절씩 게인을 찾는 자리고, `selftest` 는 찾은 게인으로 다리 전체가
버티는지 보는 자리임. 사람이 답을 고르지 않아서 다리를 보고 있을 수 있음.

---

## 폴더별 문서

| | |
|---|---|
| [`config/`](config/README.md) | 설정 값. 두 파일을 나눈 이유 |
| [`src/huphy/config/`](src/huphy/config/README.md) | 설정 읽기 |
| [`src/huphy/calibration/`](src/huphy/calibration/README.md) | 실측값 읽기·쓰기 |
| [`src/huphy/safety/`](src/huphy/safety/README.md) | 명령의 최종 관문 |
| [`src/huphy/motors/`](src/huphy/motors/README.md) | 벤더 중립 자료형, CAN 전송, 하드웨어 전제 |
| [`src/huphy/motors/robstride/`](src/huphy/motors/robstride/README.md) | 벤더 사양, 코덱, 버스, 커미셔닝 |
| [`src/huphy/kinematics/`](src/huphy/kinematics/README.md) | 발목 링키지 |
| [`src/huphy/robots/`](src/huphy/robots/README.md) | 관절 ↔ 모터 경계 |
| [`src/huphy/telemetry/`](src/huphy/telemetry/README.md) | 관찰 |
| [`src/huphy/control/`](src/huphy/control/README.md) | 제어 루프, 게인 튜닝 |
| [`src/huphy/scripts/`](src/huphy/scripts/README.md) | 터미널 진입점 |
| [`tests/`](tests/README.md) | 무엇을 고정했나 |
| [`docs/motor_setup.md`](docs/motor_setup.md) | 출하 상태 모터를 쓸 수 있게 만들기. 보낼 프레임까지 |
| [`docs/cycle.md`](docs/cycle.md) | 한 주기. 값이 어떤 모양으로 어디를 지나나 |
| [`docs/issues.md`](docs/issues.md) | 미해결 항목과 근거 |

---

## 테스트

```bash
PYTHONPATH=src python3 -m pytest tests -q
```

```
956 passed in 18.2s
```

**하드웨어도 `python-can` 도 필요 없음.**

- 순수 계산 계층은 애초에 `python-can` 을 안 씀
- 전송 계층은 `import can` 이 함수 안에 있어 가짜 모듈로 갈아끼움
- 가짜 버스가 **명령에 응답함** — 실제 모터가 명령을 받은 뒤 답하는 것과 같은 순서

파일 하나만 돌릴 수도 있음. 무엇을 고정하는지는 [`tests/README.md`](tests/README.md).

```bash
PYTHONPATH=src python3 -m pytest tests/test_leg.py -q
PYTHONPATH=src python3 -m pytest tests -q -k sweep
```

`test_bringup.py` 가 가장 오래 걸림(약 7초) — 루프가 실제로 도는 항목이 있음.

### 확인되지 않는 것

| | 왜 |
|---|---|
| 전송 지연, CAN 중재 | 실물의 물리 |
| 모터가 실제로 응답하는지 | 프로토콜 모드가 맞아야 함 |
| 게인 값이 적절한지 | 다리 무게와 감속비에 달림 |
| 발목 기하가 실물과 맞는지 | 발 각도를 재야 함 |
| 실제 제어 주기 | 스케줄러 정밀도와 부하 |

**자기일관성은 정확성이 아님.** 기하값이 틀려도 IK↔FK 왕복은 성립함.
