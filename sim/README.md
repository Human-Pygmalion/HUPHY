# `sim/` — 실물 대신 MuJoCo 에 올려 보기

정책 꾸러미를 **이 저장소의 제어 경로 그대로** MuJoCo 안에서 돌림. 실물에 올리기
전에 우리 코드가 정상인지 보는 것이 목적임.

이 폴더는 `ros/` 와 같은 급임 -- 최상위에 따로 있고, `huphy` 패키지에 들어가지
않고, `huphy` 를 가져다 쓰기만 함. 통째로 지워도 나머지가 멀쩡함.


## 0. 실행

```bash
pip install mujoco                 # 한 번만. 또는 pip install -e .[sim]
```

레포 루트에서 돌림. `huphy` 를 설치하지 않아도 됨 -- `rollout.py` 가 `src/` 를
경로에 넣음.

```bash
python3 sim/rollout.py --policy walk
python3 sim/rollout.py --policy walk --cmd vx=0.3 --duration 10
python3 sim/rollout.py --policy motion399 --duration 10
python3 sim/rollout.py --policy squat --cmd h_star=0.8
python3 sim/rollout.py --policy walk --view                  # MuJoCo 창
python3 sim/rollout.py --policy walk --cmd vx=0.3 --log      # sim/logs/ 에 기록
```

| 인자 | |
|---|---|
| `--policy` | **필수.** `config/policies/` 의 꾸러미 이름 |
| `--checkpoint` | 가중치 파일. 생략하면 계약의 기본값 |
| `--duration` | 돌릴 초. 기본 10 |
| `--approach` | 기준 자세로 옮기는 시간. 기본 0.2초. 끝나면 정책이 바로 시작됨. **1초를 넘기면 정책이 시작되기 전에 넘어짐** |
| `--cmd 이름=값` | 정책 명령. 여러 번 줄 수 있음 |
| `--hz` | 전송 주기. 기본은 정책 주기(50). 정수 배만 됨 |
| `--log [폴더]` | 기록. 기본 `sim/logs/` |
| `--view` | MuJoCo 창 |
| `--config` | 로봇 설정. 기본 `config/robot.yaml` |

**실시간으로 돎.** `--duration 10` 이면 벽시계로 약 12초 (정지 감쇠 5초 포함).
`Ctrl-C` 로 멈춤. 넘어지면 스스로 멈춤.

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
기움. 서 있게 하는 것은 정책이고, 정지 자세 유지가 아님.

그래서 `rollout.py` 는 `--approach` 가 끝나는 대로 정책을 시작시킴. 실물은 사람이
Enter 를 눌러 시작하는데(`scripts/run.py` 의 `EnterWatcher`), 시뮬에는 누를 사람이
없음.

**`--approach` 기본값이 0.2초인 이유가 이것임.** 실물 기본값(2초)을 쓰면 정책이
시작되기도 전에 넘어짐.

```
--approach 2.0    ** 넘어짐: 기울기 0.71, 1.00초 **   <- 정책이 아직 안 돌았음
--approach 0.2    서 있음 (기울기 최대 0.41)
```

시뮬은 기준 자세에 이미 놓고 시작하므로 옮길 거리가 없음 -- 이 구간은 사실 할 일이
없는 구간임. 넘어짐 메시지에 "정책이 시작되기 전임" 이 같이 찍히면 그 뜻임.

### 끝에서 주저앉는 것은 정상

루프가 끝나면 `ControlLoop._exit` 이 `DAMPING_S`(5초) 동안 감쇠한 뒤 토크를 끊음.
토크가 없으면 서 있을 수 없으므로 몸통 높이가 0.07 m 가 됨. 요약은 **제어 중
마지막 높이**와 감쇠 뒤 높이를 따로 보여줌.

### 실시간으로 돎

물리는 실시간의 6배 빠르게 계산되지만, `ControlLoop` 이 주기를 지키며 기다려서
10초 롤아웃이 10초 걸림. 더 빨리 돌리려면 `control/loop.py` 를 고쳐야 해서 넣지
않았음.

### `squat` 은 지금 넘어짐 (미해결)

`walk`·`stair_slope`·`motion399` 는 서 있고, `squat` 은 1초 안에 기욺(기울기 0.74).
원인을 아직 모름. 후보:

```
우리 쪽 관측 조립이 squat 계약(43칸, h_star)에서 틀림      <- 그렇다면 이 도구가 찾은 첫 버그임
squat 이 다른 저장소(huphy_squat)에서 다른 조건으로 학습됨
```

MJCF 자체는 거의 같음 (`huphy_squat/assets/huphy_legonly_ab.xml` 과 51줄 차이이고
대부분 주석·이름임). 가중치와 계약은 참조 입출력 검사를 통과함(2.0e-06)이라
신경망과 계약 읽기는 정상임 -- 남은 것은 **살아 있는 관측을 만드는 경로**와 학습
조건 차이임.


## 6. 시험

```bash
python3 -m pytest sim/tests -q                 # 17개
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
