# `union` — 모션 추종 정책

HuphyChan-dev 커밋
[`ee7770d7`](https://github.com/Human-Pygmalion/HuphyChan-dev/commit/ee7770d7a72c18dc1c7ddeb25c7e57045a3fcd04)
의 `huphy_00545_uniondr_30000` 릴리스를 이 레포에서 돌리게 옮긴 것.

**실기 안전 검증본이 아님.** 릴리스 README 의 Safety 절이 그렇게 적고 있고, 아래
3절에 이 레포 쪽에서 확인한 어긋남이 더 있음. 모터를 켜기 전에 3절을 읽을 것.

---

## 1. 무엇이 다른가

지금까지 돌린 정책은 **속도 명령** 정책임 — 사람이 `vx`·`wz`·`h*` 를 주고 정책이
그 방향으로 걸음. 이것은 **모션 추종** 정책임 — 파일에 든 관절 궤적 659프레임을
재생하고 정책은 그 궤적을 따라가려 함. 사람이 줄 명령이 없음.

```
                 generalized_walk          union
관찰              44칸                      114칸
명령 칸           vx, wz (2칸)              기준 프레임의 관절 24개 + 속도 24개 (48칸)
관절 관찰         모터 12개 x 2프레임        관절 24개 위치 + 24개 속도
상태              없음 (명령은 사람이)        몇 번째 프레임인지
주기              50Hz                      50Hz
행동              모터 12개 목표각           같음
ROS 노드          policy_node               motion_node
```

관찰 114칸의 순서 (계약의 `observations`):

```
[  0: 48]  기준 프레임.  관절 24개 위치(rad) + 24개 속도(rad/s). **절대값**
[ 48: 51]  몸통 프레임 중력방향
[ 51: 54]  몸통 프레임 각속도 (rad/s)
[ 54: 78]  관절 24개 위치 - 기본자세 (rad)
[ 78:102]  관절 24개 속도 (rad/s)
[102:114]  직전 행동 날값 12개
```

목표각은 `기본자세 + 0.25 x 행동` 임 (릴리스 `deploy.yaml` 의
`actions.scale_rad_per_unit`).

---

## 2. 관절 24개 중 12개는 모터가 없음

관찰이 보는 관절은 다리마다 12개임.

```
모터 있음   hip_pitch, hip_roll, hip_yaw, knee, crank_A, crank_B
모터 없음   rod_A_u1, rod_A_u2, rod_B_u1, rod_B_u2, ankle_pitch, ankle_roll
```

뒤의 6개는 인코더가 없음. 대신 **크랭크각 2개로 정해짐** — 로드 끝이 발판에 붙어
있어 루프가 닫혀 있고, 미지수 6개에 구속 6개임 (로드마다 끝점 3개). 푸는 것은
[`kinematics/mjcf_loop.py`](../../src/huphy/kinematics/mjcf_loop.py) 임.

검증: 학습에 쓴 기준 모션 659프레임의 24관절 값이 참값임. 크랭크각 2개만 주고 푼
결과가 나머지 6개와 **최대 6.8e-8 rad (3.9e-6 도)** 안에서 같음
(`tests/test_union.py` 의 `TestLoop`). 속도는 야코비안 x 모터 속도로 내고
기준 모션의 속도와 최대 9.5e-3 rad/s 차이임 (모션 변환 스크립트가 낸 값과 비교한
것이고, 학습 때 넣은 속도 노이즈는 0.5 rad/s 였음).

푸는 비용은 한 다리 약 0.53ms (자세 + 속도), 양다리 약 1.1ms 임. 50Hz 주기 20ms
안에 들어감. 데스크톱 실측이고 추론은 작업 기계에서 돎.

---

## 3. 확인된 어긋남 — 모터를 켜기 전에 읽을 것

### 3-1. 발목 링키지가 레포 실측과 다름

이 레포의 `kinematics/ankle.py` 는 **실측** 링키지이고, 학습 MJCF 의 링키지는
수치가 다름. 같은 크랭크각에 대해 발목각이 이만큼 다름 (기준 모션 659프레임에서
`ankle.py` 의 FK 와 MJCF 값을 비교).

```
피치   상관계수 1.000.  범위가 MJCF 가 약 1.1배 넓음 (-50.0~10.0 vs -44.9~8.5 도)
롤     오른다리 상관계수 -0.972 -- **부호 규약이 반대**
       크기도 MJCF 가 약 1.4~1.5배 큼 (2.0~20.0 vs -12.9~-1.2 도)
기본자세  MJCF 는 크랭크 (+8.25, -8.43)도 에서 발목 피치 +10.04도.
          `ankle.py` 는 크랭크 (0, 0) 에서 발목 (0, 0) -- 같은 크랭크각을 넣으면 8.8도
```

**관찰에는 MJCF 값을 넣음.** 정책이 MJCF 가 내는 숫자로 학습했으므로 관찰을 실측
링키지로 만들면 학습과 어긋난 입력을 주게 됨. 그래서 `mjcf_loop.py` 를 따로 두고
`ankle.py` 와 섞지 않음.

**다만 물리 쪽 어긋남은 남음.** 정책이 "발목 롤 20도" 를 의도해 크랭크각을 내면
실제 발은 실측 링키지대로 약 13도만 돎. 발 자세가 시뮬과 다르게 나옴. 이건 코드로
메울 수 없음 — MJCF 를 실측값으로 고쳐 다시 학습하거나, 어느 쪽 수치가 맞는지
실물로 재야 함.

### 3-2. 기준 궤적이 로봇 가동범위를 벗어남

기준 모션의 관절 범위를 `config/calibration/*_v1.0.json` 의 가동범위와 대조한 것
(단위 도, 넘치는 양).

```
right_leg/hip_yaw   -45.0~ 19.6   가동범위 -40.5~ 40.5    하한 -4.5
right_leg/knee        0.0~120.0   가동범위   6.0~114.0    하한 -6.0  상한 +6.0
left_leg/hip_pitch  -24.9~ 45.5   가동범위 -17.8~112.8    하한 -7.2
left_leg/hip_yaw    -10.9~ 45.0   가동범위 -40.5~ 40.5    상한 +4.5
left_leg/knee      -120.0~ -2.0   가동범위 -114.0~ -6.0   하한 -6.0  상한 +4.0
```

6개 관절이 가동범위 밖으로 나감. 안전 가드가 자르므로 **그 구간에서는 기준 궤적을
따라갈 수 없음.** 계약의 `clip_rad` 는 MJCF 관절 범위라 더 넓고, 실제로 자르는
것은 모션보드의 가드임 (`safety/guards.py`).

### 3-3. 궤적이 쪼그려 앉는 동작이고 되돌아오지 않음

```
몸통 높이     1.032 m -> 0.551 m
무릎          최대 120도
길이          659프레임, 13.16초
첫 프레임과 끝 프레임의 관절차   최대 37.6도
```

끝이 쪼그려 앉은 자세임. 그래서 `motion_node` 는 **끝에서 꺼지지 않고 마지막
목표각을 유지함** — 목표각을 안 내면 모션보드가 100ms 뒤 감쇠, 5초 뒤 토크를 끊어
그 자세에서 주저앉음. 복귀는 `approach`.

**되돌려 반복하지 않음.** 첫 프레임과 끝 프레임이 37.6도 떨어져 있어 이어 붙이면
한 주기에 그만큼 뛰는 목표각이 나감.

첫 프레임의 관절 속도가 최대 5.04 rad/s 임 — 궤적이 정지 상태에서 시작하지 않음.
기본자세에 선 채로 프레임 0 부터 시작하면 기준과 실측이 처음부터 벌어져 있음.
학습 때도 에피소드 길이가 10초(궤적은 13.16초)라 임의 프레임에서 시작했음.

### 3-4. PD 게인이 레포 설정과 다름

릴리스 `deploy.yaml` 의 게인 (Nm/rad). 계약의 `pd_gains_nm_per_rad` 로 옮겨 적어
두었지만 **쓰이지 않음** — 게인의 출처는 `robot.yaml` 하나임 (`ros/README.md` 의
"게인·관절 한계·모터 id 는 양쪽 다 없음").

```
hip_pitch/roll/yaw  kp 150   kd 6.0      robot.yaml: 135 / 135 / 75,  kd 6/6/4
knee                kp 220   kd 6.0      robot.yaml: 130, kd 6
crank_A/B           kp 22.3  kd 1.41     robot.yaml: 60, kd 2.7
```

크랭크는 학습이 레포보다 2.7배 부드럽고, 무릎은 1.7배 단단함. 바꿀지는 사람이
정할 일임.

---

## 4. 꾸러미 만들기

받은 릴리스에서 다시 만들 수 있음.

```bash
python3 tools/policy_package/build_union.py \
  --release <릴리스 폴더>/huphy_00545_uniondr_30000
```

내는 것:

```
config/policies/union/
  deploy_contract.json   이 레포 실행기가 읽는 계약 (72 KB)
  model_30000.pt         릴리스 그대로 (5.7 MB, sha256 대조)
  motion_00545.npz       기준 모션에서 쓰는 배열만 (126 KB, 659프레임)
```

릴리스의 `.pt` 와 `policy.onnx` 의 가중치가 바이트 단위로 같은지, ONNX 의 정규화
나눗셈 값이 `std + 1e-2`(`rsl_rl.NORM_EPS`) 와 같은지 확인함. 계약의 참조 입출력
16쌍은 **ONNX 가중치로 따로 구현한 순전파**로 만든 값임 — `.pt` 를 읽는 이 레포
코드(`control/rsl_rl.py`)를 다른 파일·다른 코드로 대조하려는 것임. 꾸러미를 읽을
때 그 16쌍을 다시 돌려 맞춰 봄 (`load_model`). 지금 최대차 0.0.

---

## 5. 돌리기

```bash
# 모션보드
ros2 launch huphy_ros control.launch.py config:=$HOME/HUPHY/config/robot.yaml

# 작업 기계 -- policy.launch.py 대신 이것 (둘 다 띄우지 말 것)
ros2 launch huphy_ros motion.launch.py \
  policy:=$HOME/HUPHY/config/policies/union

ros2 service call /huphy/enable std_srvs/srv/Trigger          # 토크
ros2 service call /huphy/approach std_srvs/srv/Trigger        # 기준 자세로
ros2 service call /huphy/start_policy std_srvs/srv/Trigger    # 재생 시작
ros2 topic echo /huphy/policy_status                          # frame / progress
ros2 service call /huphy/stop_policy std_srvs/srv/Trigger     # 감쇠
ros2 service call /huphy/disable std_srvs/srv/Trigger         # 토크 끊기
```

패드로도 같은 것을 함. 스틱은 할 일이 없고 버튼만 씀.

```bash
ros2 launch huphy_ros gamepad.launch.py policy:=$HOME/HUPHY/config/policies/union
```

```
View 1초   토크 넣기
A          기준 자세로
RB+Menu    재생 시작 (RB 를 쥐고 있어야 나감)
B          멈춤 -> 토크 끄기
```

패드나 `gamepad_node` 가 죽으면 `cmd_vel` 이 끊기고, `motion_node` 가 0.5초 뒤
재생을 멈추고 모션보드가 감쇠 후 토크를 끊음. 서비스로만 돌릴 때는 그 감시가
안 걸림 (`cmd_vel` 을 한 번도 안 받았으므로).

런치 인자:

```
policy          꾸러미 폴더
checkpoint      빈 값 = 계약의 30000
motion_hz       0 = 계약의 50Hz
state_stale_ms  60. 관절 상태가 이보다 낡으면 목표각을 안 냄
approach_s      3.0
start_frame     0. 궤적의 몇 번째 프레임부터 재생할지
```

단계는 `policy_node` 와 같고 끝이 하나 더 있음.

```
off -> approach -> ready -> running -> done
                                       마지막 목표각을 유지함
```

`log_node` 를 띄우면 `running` 구간이 기록됨. 관찰 열 이름이 114칸으로 나가고
뷰어의 관찰 그래프에 `obs/ref_q/...`, `obs/q24/...`, `obs/dq24/...` 로 보임.

---

## 6. 아직 안 한 것

```
실물 확인          한 번도 안 돌림. 3절의 어긋남을 재 보고 결정할 일
MJCF 링키지 수치   실측과 다름 (3-1). 어느 쪽이 맞는지 실물로 재야 함
기준 궤적 가동범위  6개 관절이 밖으로 나감 (3-2). 궤적을 다시 리타게팅할지 판단 필요
huphy-run          `--policy config/policies/union` 로 재생은 됨 (명령 칸이
                   없어 --cmd 가 필요 없음). 다만 끝에 닿은 것을 알리지 않고 마지막
                   프레임의 목표각을 계속 냄. 끝 처리와 프레임 표시는 ROS 쪽만 넣었음
```
