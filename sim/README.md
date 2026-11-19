# `sim/` — 실물 대신 MuJoCo 에 올려 보기

정책 꾸러미를 **이 저장소의 제어 경로 그대로** MuJoCo 안에서 돌림. 실물에 올리기
전에 우리 코드가 정상인지 보는 것이 목적임.

이 폴더는 `ros/` 와 같은 급임 -- 최상위에 따로 있고, `huphy` 패키지에 들어가지
않고, `huphy` 를 가져다 쓰기만 함. 통째로 지워도 나머지가 멀쩡함.


## 0. 실행

```bash
pip install mujoco                 # 한 번만. 또는 pip install -e .[sim]
pip install pygame                 # --teleop 으로 키 조작할 때만
```

레포 루트에서 돌림. `huphy` 를 설치하지 않아도 됨 -- `rollout.py` 가 `src/` 를
경로에 넣음.

```bash
python3 sim/rollout.py --policy walk
python3 sim/rollout.py --policy walk --cmd vx=0.3 --duration 10
python3 sim/rollout.py --policy motion399 --duration 10
python3 sim/rollout.py --policy squat --cmd h_star=0.8
python3 sim/rollout.py --policy walk --view                  # MuJoCo 창
python3 sim/rollout.py --policy walk --teleop --view         # 키로 조작
python3 sim/rollout.py --policy walk --cmd vx=0.3 --log      # sim/logs/ 에 기록
```

| 인자 | |
|---|---|
| `--policy` | **필수.** `config/policies/` 의 꾸러미 이름 |
| `--checkpoint` | 가중치 파일. 생략하면 계약의 기본값 |
| `--duration` | 이 초만큼만 돎. **생략하면 멈출 때까지** (`huphy-run` 과 같음) |
| `--approach` | 기준 자세로 옮기는 시간. 기본 0.2초 |
| `--now` | Enter 를 기다리지 않고 바로 시작 |
| `--teleop` | pygame 창으로 명령을 실시간 조작 |
| `--max`/`--accel`/`--decel` | `--teleop` 의 상한·가속·감속 |
| `--echo 초` | 정책 출력값을 찍는 간격. 기본 1초, `0` 이면 안 찍음 |
| `--no-ghost` | `--view` 창에 명령 자세를 겹치지 않음 |
| `--cmd 이름=값` | 정책 명령. 여러 번 줄 수 있음 |
| `--hz` | 전송 주기. 기본은 정책 주기(50). 정수 배만 됨 |
| `--log [폴더]` | 기록. 기본 `sim/logs/` |
| `--view` | MuJoCo 창 |
| `--config` | 로봇 설정. 기본 `config/robot.yaml` |

**실시간으로 돎.** `--duration 10` 이면 벽시계로 약 12초 (정지 감쇠 5초 포함).
`--duration` 을 생략하면 `huphy-run` 처럼 **멈출 때까지 돎** -- `Ctrl-C`, `--teleop`
창의 `Esc`, 또는 넘어짐.

멈추면 **`kp=0` 으로 5초 감쇠한 뒤 토크를 끊음** (`ControlLoop._exit`, 실물과 같음).
그때 로봇이 주저앉는 것은 정상이고 넘어진 것이 아님 -- 창을 띄워 두면 이것이 제일
먼저 눈에 띔. 넘어짐은 돌고 있는 중에 `** 넘어짐 **` 으로 찍힘.

### 시작은 실물과 같음 -- Enter

`huphy-run` 과 같은 순서임: 기준 자세로 옮김 -> **그 자세로 기다림 -> Enter 를
누르면 정책이 시작됨.** 기다리는 코드도 같은 것임 (`scripts/run.py` 의
`EnterWatcher`, `staged`).

기다리는 동안 **물리를 굴리지 않음** (`SimWorld.frozen`). 기준 자세는 가만히 못
버티므로(아래 5절) 굴리면 Enter 를 누르기 전에 넘어짐. 실물에서는 그동안 사람이
로봇을 붙잡고 있고, 그 자리에 해당하는 것이 이 멈춤임. 창을 띄워 뒀으면 멈춘
동안에도 화면은 계속 다시 그려서 시점을 돌려볼 수 있음.

`--duration` 은 **정책이 돈 시간만 셈.** Enter 를 늦게 눌러도 돌아가는 시간이
줄지 않음.

화면이 아니면(파이프, 시험) Enter 를 누를 사람이 없으므로 바로 시작함 --
`EnterWatcher` 가 원래 그렇게 함. `--now` 는 화면에서도 그렇게 하는 것임.

### 명령 넣기 -- 고정값 또는 키 조작

명령 이름은 꾸러미 계약이 정함. `walk`·`stair_slope` 는 `vx`·`wz`, `squat` 은
`h_star`, `motion399`·`motion545` 는 명령이 없음.

**고정값**은 `--cmd` 임. 실행 내내 그 값으로 감.

```bash
python3 sim/rollout.py --policy walk  --cmd vx=0.3 --cmd wz=0.1
python3 sim/rollout.py --policy squat --cmd h_star=0.80
```

`squat` 의 `h_star` 만 계약에 변화 속도 상한(`max_rate 0.1`)과 범위(`0.65~0.895`)가
있어서, 그 자리에서 뛰지 않고 시작값 0.8914 에서 0.1 m/s 로 내려감. 범위를 벗어난
값은 실행 전에 거부됨.

**키 조작**은 `--teleop` 임. **실물과 같은 코드·같은 키임** (`control/teleop.py`,
pygame 창을 별도 프로세스로 띄움).

```bash
pip install pygame                                    # 한 번만
python3 sim/rollout.py --policy walk  --teleop --view
python3 sim/rollout.py --policy squat --teleop --max h_star=0.9
```

```
walk, stair_slope   W/S 로 vx, A/D 로 wz. 누르는 동안 가속, 떼면 0 으로 감속
squat               위/아래 화살표로 h_star
창 안에서            Enter 시작, Space 속도 0, Esc 멈춤
```

상한·가속·감속은 `--max vx=1.0 --accel vx=2.0 --decel vx=3.0` 으로 바꿈. 키가
만든 값은 매 주기 정책에 들어가고 `--echo` 줄과 기록의 `obs/cmd/<이름>` 열에 그대로
남음. 속도 명령은 `--teleop` 과 `--cmd` 를 같이 주지 못함 (키를 떼면 0 이 되므로
고정값이 뜻을 잃음).

화면이 없으면(ssh) 창을 못 띄우고 멈춤 -- 그때는 `--cmd` 로 줄 것.

### 정책이 낸 값 보기

두 가지로 같은 값을 보여줌.

**창**: `--view` 면 **명령받은 자세를 반투명 로봇으로 겹쳐 그림**
(`SimWorld.ghost`). 실제 관절은 그 목표를 PD 로 따라가므로 둘이 벌어진 만큼이
추종 오차임. 물리에는 영향이 없음 -- 별도 `MjData` 에 자세를 넣고 순기구학만 풀어
뷰어의 `user_scn` 에 장식 geom 으로 넣음. 몸통 위치·자세는 실측 그대로 씀(명령에
몸통 얘기가 없음)이라 **두 로봇의 발이 벌어지는 것이 다리 추종 오차임.**

**글자**: `--echo` 간격마다 목표각과 실측 최대 오차를 찍음. 창을 안 띄우고 돌릴
때(ssh, 시험) 정책이 무엇을 냈는지 볼 길이 이것뿐임.

```
  정책  1.00초  몸통 0.896 m  기울기 0.00
    right_leg 목표도 hp  -8.8  hr  +3.1  hy  +1.1  kn +17.3  aa  +1.3  ab  -3.5   (실측과 최대 5.3도)
    left_leg  목표도 hp +10.1  hr  -3.6  hy  -2.4  kn -17.5  aa  -2.0  ab  +1.0   (실측과 최대 5.9도)
```

`hp` 엉덩이 pitch, `hr` roll, `hy` yaw, `kn` 무릎, `aa`·`ab` 발목 크랭크 A·B.
같은 값이 `--log` 기록의 `tgt/<관절>` 열에 전부 남음.

### `--view` 로 창을 띄울 때

`mujoco 3.17` + `glfw 2.11` 조합에서 뷰어를 닫은 뒤 프로세스가 끝날 때 glfw 가
죽음 (`posix_thread.c:70` assertion, 코어 덤프). **창을 열고 닫기만 하는 10줄짜리
예제도 같이 죽어서** 우리 코드와는 무관함. `with launch_passive(...)` 로 바꿔도 같음.

`rollout.py` 는 `--view` 일 때 요약을 찍은 뒤 `os._exit` 로 바로 나감 -- 인터프리터
종료를 건너뜀. 롤아웃·기록·요약은 그 전에 다 끝나 있음.

glfw 파이썬 패키지 버전을 내리면 없어질 수 있지만 확인하지 않았음.


### 기록

```
sim/logs/policy/<월일>_policy_<정책>[_인자]_sim.json
```

**실물 기록(`logs/`)과 폴더를 나눠 둠** -- 이름 끝의 `_sim` 과 파일 안
`run.sim: true` 로 구분되지만, 폴더가 같으면 목록을 볼 때마다 가려내야 함.
열 이름은 실물과 같아서 `tools/policy_viewer.html` 로 나란히 볼 수 있음.

열 구성 (`walk` 기준 81열, `PolicyLog` 이 만듦):

```
t                    1    제어 시작부터 흐른 초
obs/...             44    정책에 들어간 입력 벡터 (gyro 3, grav 3, q_prev 12, q 12, last_act 12, cmd 2)
act/<관절>          12    정책이 뱉은 날값 (스케일 전)
tgt/<관절>          12    각도로 바꾼 목표 (도)
pos/<관절>          12    시뮬 모터가 실제로 간 각도 (도)
```

머리말 `run` 에 `sim`, `mjcf`(모델 파일 이름), `timestep_ms` 가 더 들어감 -- 모델이
바뀌면 결과가 달라지므로 파일에 남아야 함.

**안 남는 것**: 모터 속도·토크·온도(값은 있지만 `PolicyLog` 이 `pos` 만 남김),
상전류·버스전압(시뮬이 모름), 몸통 높이·기울기(화면 요약에만 찍힘).


## 1. 무엇이 바뀌고 무엇이 그대로인가

`huphy-run` 이 CAN 으로 모터 12대를 잡는 자리에 `SimBus` 가 들어감. **그 위는 전부
실물 코드임.**

```
                       huphy-run (실물)            sim/rollout.py
관측 조립               control/deploy.py           같음
추론                   control/rsl_rl.py           같음
행동 -> 목표각          control/deploy.py           같음
로봇 계층              robots/leg.py · biped.py    같음
발목 IK/FK             kinematics/ankle.py         같음
24칸 관절 풀이          kinematics/mjcf_loop.py     같음
한계·점프 가드          safety/guards.py            같음
제어 루프              control/loop.py             같음
기록                   telemetry/policy_log.py     같음  (열 이름까지)
--------------------------------------------------------------------
버스                   motors/robstride/bus.py     sim/huphy_sim/bus.py
IMU                    sensors/ebimu/              sim/huphy_sim/imu.py
모터·바닥              실물                        MuJoCo
```

`Leg` 가 버스에 쓰는 면은 14개뿐이고(`state`, `send_mit`, `collect`, ...) 버스는
생성자 인자임. 그래서 버스 하나만 바꿔 끼우면 `Leg`·`Biped` 가 그대로 돎 --
cal<->raw 변환, 캘리브레이션 파일, 발목 기구학, 가드, 카운터가 다 실물 것임.


## 2. 잡히는 것

```
관측 칸 순서가 계약과 다름                      (deploy_contract.json 과 어긋남)
자이로 도/초 <-> rad/s 변환 누락                 57배 틀림
중력 벡터 부호 반대
행동 스케일·기준 자세 오프셋이 틀림
발목 pitch/roll -> 크랭크 a/b 기구학 부호
24칸 관절 풀이 (mjcf_loop) 오류
왼다리·오른다리 순서 뒤바뀜
이전 행동·이력 프레임 쌓는 순서
한계 가드가 정상 동작을 자름
게인(robot.yaml)으로 정책을 따라가는지
```

마지막 줄이 되는 이유: 토크를 **명령에 실려 온 kp/kd** 로 계산함. 실물 모터
펌웨어와 같은 식임 (`MitCommand` 의 다섯 칸). MJCF 액추에이터의 게인(학습 설정의
값)을 쓰지 않음.


## 3. 안 잡히는 것

```
캘리브레이션 sign/offset      명령에 cal->raw 를 걸고 버스가 raw->cal 로 되돌려서 상쇄됨.
                             조립을 재야 아는 값이고 시뮬에는 조립이 없음
CAN 지연·지터·응답 누락       collect() 가 늘 "전부 답했다" 를 냄
인코더 영점 오차              실측 최대 9.4도. 시뮬은 0
모터 발열                    온도는 NaN 으로 둠. 가짜 숫자를 넣으면 로그를 믿게 됨
상전류·버스전압               --extra 의 iq/vbus 자리가 빔. 전력 열이 안 나옴
IMU 노이즈·바이어스·지연
지형                        평지만. 학습 쪽의 10단계 지형·계단·경사는 없음
외력                        밀기·던지기 없음
DR                          발 마찰(0.3~1.2), 질량, 게인 흔들기 없음. 전부 공칭값
```


## 4. 구성

```
sim/
├── rollout.py              실행. 꾸러미 고르기·기록 이름은 scripts/run.py 함수를 씀
├── conftest.py             pytest 경로
├── huphy_sim/
│   ├── world.py            MuJoCo 모델·데이터. 액추에이터·바닥을 붙이고 20 ms 씩 굴림
│   ├── actuator.py         PD 토크 -> 토크-속도 곡선 클램프 (MuJoCo 를 모름)
│   ├── bus.py              SimBus. RobStrideBus 자리
│   ├── imu.py              SimImu. Imu 프로토콜
│   └── build.py            robot.yaml -> Leg x2 (SimBus) -> Biped
├── assets/
│   ├── pygmalion_legonly_ab.xml      MJCF 사본. **고치지 않음**
│   ├── meshes/                        STL 34개
│   └── motor_spec/*.csv               RS03·RS04 48V 토크-속도 곡선 (실측)
└── tests/test_sim.py
```

### 자산 출처

학습 레포 `HuphyChan-dev/mjlab_policy/pygmalion_mjlab/assets/` 사본임
(2026-11-18). MJCF 는 **한 글자도 고치지 않음** -- 원본이 바뀌면 `diff` 로 바로 알
수 있어야 함. 다시 가져오려면:

```bash
S=../HuphyChan-dev/mjlab_policy/pygmalion_mjlab/assets
cp $S/pygmalion_legonly_ab.xml sim/assets/
cp $S/meshes/*.stl sim/assets/meshes/
cp $S/motor_spec/*.csv sim/assets/motor_spec/
```

### MJCF 에 없어서 코드가 붙이는 것

```
액추에이터 12개   관절마다 토크 액추에이터. 상한은 모델의 tmax_nm (RS03 60, RS04 120)
바닥             무한 평면. 마찰은 MuJoCo 기본값 1.0
솔버 설정        iterations 100 / ls 50 / ccd 50  (학습 쪽 SimulationCfg 와 같은 값)
```

armature·frictionloss·damping 은 MJCF 안에 이미 적혀 있어서 손대지 않음.
timestep 은 MuJoCo 기본값 0.002 를 그대로 씀 -- 학습 쪽도 0.002 임. 정책 주기
20 ms 는 그 10배이고, 한 주기에 10 스텝을 굴림.

### 토크-속도 곡선

PD 토크를 48 V 실측 곡선으로 깎음. 구동(토크와 속도가 같은 부호)일 때만 곡선이
상한이고, 제동일 때는 피크까지 허용함 -- 학습 쪽 `tn_actuator.py` 의 `tn_clamp`
와 같은 규칙임. 식만 numpy 로 옮겼고 mjlab 을 설치하지 않음.


## 5. 알고 있어야 하는 것

### 기준 자세는 가만히 못 버팀

기준 자세를 **정지 명령으로 붙잡고 있으면 1초 안에 앞으로 넘어짐.** 발끝을 축으로
기움. 서 있게 하는 것은 정책이고, 정지 자세 유지가 아님. 측정값:

```
그 자세로 2초를 굴림    ** 넘어짐: 기울기 0.71, 1.00초 **
```

그래서 정책이 시작되기 전(접근·Enter 대기) 구간에는 **물리를 멈춰 둠** (위 0절).
실물에서는 그 구간에 사람이 로봇을 붙잡고 있음.

이 때문에 `--approach` 는 시뮬에서 하는 일이 없음 -- 기준 자세에 이미 놓고
시작하므로 옮길 거리가 없고, 그 구간은 멈춰 있음. 기본값을 0.2초로 둔 이유임
(실물 기본값은 3초이고, 그쪽에서는 관절이 튀지 않게 천천히 가는 시간임).

### 끝에서 주저앉는 것은 정상

루프가 끝나면 `ControlLoop._exit` 이 `DAMPING_S`(5초) 동안 감쇠한 뒤 토크를 끊음.
토크가 없으면 서 있을 수 없으므로 몸통 높이가 0.07 m 가 됨. 요약은 **제어 중
마지막 높이**와 감쇠 뒤 높이를 따로 보여줌.

### 실시간으로 돎

물리는 실시간의 6배 빠르게 계산되지만, `ControlLoop` 이 주기를 지키며 기다려서
10초 롤아웃이 10초 걸림. 더 빨리 돌리려면 `control/loop.py` 를 고쳐야 해서 넣지
않았음.

### 꾸러미 넷이 다 서 있음

10초씩, 명령 없이 (`--duration 10`):

```
walk          서 있음   기울기 최대 0.01
stair_slope   서 있음   기울기 최대 0.08
motion399     서 있음   기울기 최대 0.52
squat         서 있음   기울기 최대 0.03
```

`squat` 이 한동안 1초 안에 넘어졌는데, **넘어진 것은 정책이 아니라 정책이
시작되기 전 구간이었음** -- 기준 자세를 정지 명령으로 붙잡고 있던 `--approach`
2초 구간임. 지금은 그 구간에 물리를 멈추므로 안 생김
([docs/issues.md #17](../docs/issues.md)).

`motion399` 의 0.52 는 넘어짐 판정(0.7)에 가까움. 동작 재생이라 몸통을 크게
쓰는 것이지만 꾸러미를 바꿀 때 같이 볼 것.


## 6. 시험

```bash
python3 -m pytest sim/tests -q                 # 21개
python3 -m pytest sim/tests -q -m "not slow"   # 정책을 굴리는 것 빼고
```

`mujoco` 가 없으면 전부 건너뜀 (`importorskip`).

무엇을 지키는 시험인지는 `tests/test_sim.py` 첫머리에 적어 둠.


## 7. 나중에

```
ROS 에서도 시뮬로            control_node 에 파라미터 하나로 build_biped 대신 SimRobot.
                            convert.py 왕복과 토픽 지연까지 검증 범위에 들어옴
mjlab 판 액추에이터          시뮬에서 걷는데 실물에서 안 걸을 때 액추에이터 모델을
                            의심하게 되면, Robot 계약이 같으므로 갈아 끼울 수 있음
지형·외력                   계단·경사·밀기. 학습 쪽 생성기를 옮기는 일임
```
