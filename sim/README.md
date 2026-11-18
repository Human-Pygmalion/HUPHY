# `sim/` — 실물 대신 MuJoCo 에 올려 보기

정책 꾸러미를 **이 저장소의 제어 경로 그대로** MuJoCo 안에서 돌림. 실물에 올리기
전에 우리 코드가 정상인지 보는 것이 목적임.

```bash
pip install mujoco                      # 또는 pip install -e .[sim]
python3 sim/rollout.py --policy walk
python3 sim/rollout.py --policy motion399 --duration 10 --log logs
python3 sim/rollout.py --policy walk --view        # MuJoCo 창
```

이 폴더는 `ros/` 와 같은 급임 -- 최상위에 따로 있고, `huphy` 패키지에 들어가지
않고, `huphy` 를 가져다 쓰기만 함. 통째로 지워도 나머지가 멀쩡함.


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
없음. `--approach` 를 길게 주면 그 동안 넘어짐.

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
