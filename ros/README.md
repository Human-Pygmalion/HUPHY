# `ros/` — ROS 2 래퍼

`src/huphy/` 를 고치지 않고 ROS 2 노드로 감싼 것. 노드 둘 + 조작 노드 둘 (창, 패드).

| 노드 | 하는 일 | 어디서 |
|---|---|---|
| `control_node` | 모터를 몲. 100Hz, CAN, 가드 | 모션보드 (CAN·IMU 가 붙은 쪽) |
| `policy_node` | 갈 자세를 냄. 정책 주기로 추론 (지금 정책 둘 다 50Hz) | 작업 기계 (사람이 쓰는 쪽) |
| `teleop_node` | (선택) `huphy-run --teleop` 과 같은 창으로 명령·시작·멈춤 | 작업 기계 (화면 있는 쪽) |
| `gamepad_node` | (선택) Xbox 패드로 명령·토크·시작·멈춤. `joy_node` 와 같이 띄움 | 작업 기계 (패드 꽂은 쪽) |

그림은 [`ros.drawio`](ros.drawio). 설계 근거는 **아래쪽 "아래는 설계"** 부터.

로봇 없이 ROS 만 테스트하려면 [`without_robot.md`](without_robot.md).

---

## 설치 (새 컴퓨터)

**한 번만 하면 됨.** 끝나면 터미널을 새로 열 때 따로 할 것이 없음.
두 기계(모션보드, 작업 기계) 모두 똑같이 함.

| | |
|---|---|
| OS | Ubuntu 22.04 |
| ROS | Humble |
| 파이썬 | 3.10 (Ubuntu 22.04 기본) |

```bash
sudo apt install -y git
cd ~
git clone https://github.com/chanhue/HUPHY.git
cd HUPHY
ros/install.sh
```

중간에 sudo 비밀번호를 물어봄 (ROS 설치). `끝.` 이 나오면 **새 터미널을 열고** 바로
`ros2 launch` 를 씀.

**여러 번 실행해도 됨.** 이미 된 단계는 건너뛰고, 잘못된 것은 고침. 설치가 꼬인 것
같으면 일단 다시 실행함.

### 스크립트가 하는 일

| 단계 | 하는 일 |
|---|---|
| 1 | ROS 저장소를 추가하고 ROS 2 Humble · colcon 설치. 이미 있으면 건너뜀 |
| 2 | `.venv` 를 `--system-site-packages` 로 만듦 (가상환경 안에서 ROS 가 보이게). 옵션 없이 만든 것이 있으면 다시 만듦 |
| 3 | `pip install -e ".[imu]"` |
| 4 | `~/ros2_ws` 를 만들고 가상환경 파이썬으로 빌드 (`python -m colcon build`) |
| 5 | `~/.bashrc` 에 자동 설정을 넣음 (`# >>> huphy >>>` 블록). 다시 실행하면 블록을 바꿔 끼움 |
| 6 | `huphy_ros` 가 보이는지, 노드가 가상환경 파이썬으로 도는지 확인 |

`ROS_DOMAIN_ID` 는 7 로 들어감. **두 기계가 같은 값이어야 함** — 다르면 에러 없이
서로 안 보임. 바꾸려면 `HUPHY_DOMAIN_ID=9 ros/install.sh`.

### 다시 빌드할 때

**거의 없음.** 파이썬 코드를 고치면 그대로 반영됨. 노드를 새로 추가했거나
`setup.py` · `package.xml` 을 바꿨을 때만 `ros/install.sh` 를 다시 실행함.

**`colcon build` 를 직접 치지 말 것.** 시스템 파이썬으로 빌드되어 노드가
`No module named 'huphy'` 로 죽음. 그렇게 됐으면 `ros/install.sh` 를 다시 실행하면 고쳐짐.

### 안 될 때

| 메시지 | 할 것 |
|---|---|
| `ros2: 명령을 찾을 수 없습니다` | 설치 후 새 터미널을 안 엶 |
| `Package 'huphy_ros' not found` | 새 터미널을 안 엶. 그래도 안 되면 `ros/install.sh` 다시 |
| `No module named 'huphy'` | `colcon build` 를 직접 쳤음. `ros/install.sh` 다시 |
| 스크립트가 `✗` 로 멈춤 | 그 줄에 이유가 있음. 고치고 다시 실행 |

---

## 실행

**띄우는 방법이 둘임. 해당하는 단락 하나만 따라감** -- 두 단락은 서로를 참조하지 않음.

| 띄우는 방법 | 언제 | 단락 |
|---|---|---|
| **두 기계** | 실제 배치. 모션보드는 로봇에, 작업 기계는 사람 앞에 | [A. 두 기계](#a-두-기계) |
| **한 기계** | 개발·점검. CAN·IMU 가 붙은 기계 하나에서 노드 둘 다 | [B. 한 기계](#b-한-기계) |

| 기계 | 붙은 것 | 띄우는 노드 |
|---|---|---|
| **모션보드** | CAN 어댑터 (모터) · IMU | `control_node` -- 모터를 읽고 몲 |
| **작업 기계** | 사람이 쓰는 PC | `policy_node` -- 갈 자세를 계산해 모션보드로 보냄 |

각 단락은 **평소 쓰는 순서**(띄우기 -> 토크 -> 기준 자세 -> 정책 -> 명령 -> 멈춤)가 먼저 있고, 그 아래에
**확인**(읽히는지, 토크 없이 정책 출력만)이 따로 있음. 처음 돌리거나 모터·배선을 바꿨으면
토크 넣기 전에 확인부터 함.

### 기존 명령과 어떻게 대응하나

ROS 없이 돌리는 `huphy-run` 한 번이 아래처럼 쪼개져 있음. **ROS 는 "토크 넣기" 와 "정책
시작" 이 따로임** -- `huphy-run` 은 켜자마자 토크를 넣고 기준 자세로 감.

| `huphy-*` 로 하던 것 | ROS 에서 |
|---|---|
| `huphy-can up` | 같음 (`huphy-can up`) |
| `huphy-test --robot watch` -- 토크 없이 읽기만 | `control.launch.py` 를 띄운 직후 (`OBSERVE`) + `ros2 topic echo` 로 봄 |
| `huphy-run --policy <이름>` 의 정책 불러오기 | `policy.launch.py policy:=<폴더>` -- 불러만 두고 아무것도 안 냄 |
| `huphy-run` 시작 = 토크 켜고 기준 자세로 가서 기다림 | `enable` (토크, 감쇠로 대기) -> `approach` (기준 자세로 3초, 도착하면 붙잡고 기다림) |
| `huphy-run` 의 **Enter** -- 정책 시작 | `start_policy` |
| `--teleop` -- pygame 창으로 조작 | `teleop.launch.py` -- 같은 창. R(기준 자세)·Enter(시작)·Esc(끔) |
| `--cmd vx=0.2` | `ros2 topic pub /huphy/cmd_vel ...` |
| `--cmd h_star=0.80` / `--teleop` 의 ↑↓ | `ros2 topic pub /huphy/h_star ...` |
| **Ctrl-Q** -- 감쇠 5초 후 토크 끔 | `stop_policy` -> `disable` (감쇠 5초 후 토크 끔). 또는 모션보드 런치에서 Ctrl-C |
| (없음) | 토크 없이 정책 출력만 보기 -- `OBSERVE` 에서 `approach` -> `start_policy` |

**launch 는 띄운 터미널을 계속 차지함** (끄면 노드도 꺼짐). 그 뒤 명령은 **새 터미널**에서
침.

---

### A. 두 기계

모션보드(로봇) + 작업 기계(사람 앞). 각 단계 제목의 `[ ]` 가 **치는 기계**임.
`ros2 service` · `ros2 topic` 은 네트워크 전체에 통해서 어느 기계에서 쳐도 되지만, 사람이 앉은
**작업 기계**에서 치는 것으로 적음.

두 기계의 `ROS_DOMAIN_ID` 가 같아야 함 (설치 스크립트가 7 로 넣음). 다르면 에러 없이 서로
안 보임.

#### A-1. CAN 올리고 제어 노드 띄우기  `[모션보드]`

```bash
huphy-can up
ros2 launch huphy_ros control.launch.py config:=$HOME/HUPHY/config/robot.yaml
```

| 명령 | 하는 일 | 기존으로 치면 |
|---|---|---|
| `huphy-can up` | CAN 채널 canr·canl 을 1 Mbps 로 올림. 재부팅·어댑터 재연결 뒤마다 다시 | 같음 |
| `control.launch.py` | 모터·IMU 를 100Hz 로 읽어 `joint_states` 로 내보내고, 들어온 목표각으로 모터를 몲. **`OBSERVE` 로 시작 -- 토크 없음** | `huphy-test --robot watch` 를 띄운 상태 |

모터 설정(호스트 id)이 어긋나 있으면 여기서 멈춤 -- 아래 [안 될 때](#안-될-때-1).

#### A-2. 정책 노드 띄우기  `[작업 기계]`

**둘 중 하나만** 침 -- 어느 정책을 돌릴지 고르는 것.

```bash
# 걷기 -- /huphy/cmd_vel 로 앞뒤·회전 속도를 줌
ros2 launch huphy_ros policy.launch.py policy:=$HOME/HUPHY/config/policies/generalized_walk

# 제자리 스쿼트 -- /huphy/h_star 로 골반 높이를 줌
ros2 launch huphy_ros policy.launch.py policy:=$HOME/HUPHY/config/policies/huphy_squat
```

| 명령 | 하는 일 | 기존으로 치면 |
|---|---|---|
| `policy.launch.py` | 정책 꾸러미를 불러 두고 모션보드의 `joint_states` 를 받기 시작함. **띄우기만 하면 아무것도 안 냄** (`stage` = `off`) | `huphy-run --policy <이름>` 의 불러오기까지 |

`checkpoint:=4500`(스쿼트) / `checkpoint:=37400`(걷기 예전 기본. 지금 기본은 49999) 로 다른 체크포인트, 안 주면 계약의 기본값 (`huphy-run --checkpoint`
와 같음).

#### A-3. 토크 넣기  `[작업 기계]`

**로봇을 들거나 테더에 맨 상태에서.**

```bash
ros2 service call /huphy/enable std_srvs/srv/Trigger
```

| 명령 | 하는 일 | 기존으로 치면 |
|---|---|---|
| `enable` | 모션보드를 `OBSERVE` -> `CONTROL`. **모터에 힘이 들어감.** 정책이 없는 동안은 감쇠 명령(위치를 안 붙잡고 빠른 움직임만 막음)으로 버팀 | `huphy-run` 을 켠 순간의 토크 넣기 (단, `huphy-run` 은 곧바로 기준 자세로 감) |

`success: true` 가 나와야 함. `false` 면 메시지에 이유가 있음.

#### A-4. 기준 자세로  `[작업 기계]`

```bash
ros2 service call /huphy/approach std_srvs/srv/Trigger
```

| 명령 | 하는 일 | 기존으로 치면 |
|---|---|---|
| `approach` | 3초에 걸쳐 정책의 기준 자세로 간 뒤 **그 자세를 붙잡고 기다림**. `stage` 가 `off -> approach -> ready` | `huphy-run` 이 켜지고 기준 자세로 가서 기다리는 부분 |

로봇이 기준 자세로 제대로 섰는지 보고 다음으로.

#### A-5. 정책 시작  `[작업 기계]`

```bash
ros2 service call /huphy/start_policy std_srvs/srv/Trigger
```

| 명령 | 하는 일 | 기존으로 치면 |
|---|---|---|
| `start_policy` | 신경망으로 바뀜. `stage` 가 `ready -> running`. **`ready` 가 아니면 거절**하고 이유를 줌 (먼저 approach / 아직 가는 중 n초) | **Enter** |

어느 정책(걷기/스쿼트)인지는 A-2 에서 띄울 때 정해짐 -- 여기서 고르지 않음. 지금 불린 것은
`ros2 topic echo /huphy/policy_status --once` 의 `hardware_id`. 바꾸려면 정책 런치를 다시 띄움.

#### A-6. 명령 주기 (cmd)  `[작업 기계]`

**방법 셋 -- 창(teleop), 패드(gamepad), 또는 명령.**

창으로 조작 -- `huphy-run --teleop` 과 같은 pygame 창. 화면이 있는 **작업 기계**에서 (ssh 면 `-X`).
`policy:=` 는 정책 노드에 준 것과 같은 꾸러미.

**정책 노드(A-2)에 준 것과 같은 정책**을 줌 -- 그 정책의 명령 키만 생김.

```bash
# 걷기 -- W/S 앞뒤, A/D 회전
ros2 launch huphy_ros teleop.launch.py policy:=$HOME/HUPHY/config/policies/generalized_walk

# 스쿼트 -- ↑/↓ 높이
ros2 launch huphy_ros teleop.launch.py policy:=$HOME/HUPHY/config/policies/huphy_squat
```

| 창에서 누르는 키 | 하는 일 | 기존으로 치면 |
|---|---|---|
| W/S · A/D | 걷기 앞뒤 · 회전. 누르는 동안 가속, 떼면 감속해 0 | `--teleop` 의 W/S/A/D |
| ↑/↓ | 스쿼트 높이 | `--teleop` 의 ↑/↓ |
| Space | 속도 즉시 0 | 같음 |
| **R** | 기준 자세로 (`approach`). **토크가 켜져 있을 때만** -- 아니면 안 보내고 로그에 "먼저 enable" | (없음 -- `huphy-run` 은 켜자마자 감) |
| **Enter** | 정책 시작 (`start_policy`). 기준 자세 도착 뒤에만 | Enter |
| **Esc** · 창 닫기 | 정책 끔 -> 토크 끔 (`stop_policy` -> `disable`), 창도 닫힘 | Esc |

창 맨 위에 지금 상태가 뜸 -- `Torque OFF` / `R: go to ready pose` / `Moving to ready pose` /
`Ready. Enter: start policy` / `POLICY RUNNING`. **토크 켜기(`enable`)는 키로 안 함** -- 명령으로
침. 속도 상한 등은 `max:="vx=0.8" accel:="vx=0.4" decel:="vx=1.5"` (`--max` 등과 같음).

창을 쓰면 A-4·A-5·A-6 의 명령을 대신함 (enable 만 명령).

---

패드로 조작 -- Xbox 패드. 화면이 필요 없음. 패드를 USB 나 블루투스로 붙이면
`/dev/input/js0` 가 생기고, 런치가 `joy_node`(ROS 의 `joy` 패키지)와 `gamepad_node` 를 같이 띄움.
`policy:=` 는 정책 노드에 준 것과 같은 꾸러미.

```bash
# 걷기 -- 왼쪽 스틱 앞뒤, 오른쪽 스틱 회전
ros2 launch huphy_ros gamepad.launch.py policy:=$HOME/HUPHY/config/policies/generalized_walk

# 스쿼트 -- 십자키 위아래로 높이
ros2 launch huphy_ros gamepad.launch.py policy:=$HOME/HUPHY/config/policies/huphy_squat
```

| 패드 | 하는 일 |
|---|---|
| 왼쪽 스틱 세로 · 가로 | `vx` 앞뒤 · `vy` 옆 (스틱값 × 그 명령의 `max`) |
| 오른쪽 스틱 가로 | `wz` 회전 |
| 십자키 세로 | `h_star` 높이. `accel`(기본 0.05 m/s)씩 움직이고 계약의 범위에서 멈춤 |
| **RB** | 누르고 있는 동안에만 명령이 먹음. 떼면 속도 0, 높이는 마지막 값 유지 |
| **A** | 기준 자세로 (`approach`). **토크가 켜져 있을 때만** |
| **RB + Menu** | 정책 시작 (`start_policy`). **RB 를 누르고 있어야 나감** |
| **B** | 정책 끔 -> 토크 끔 (`stop_policy` -> `disable`), 노드도 끝남 |
| **View 1초** | 토크 넣기 (`enable`). **1초 계속 눌러야** 한 번 나감 |

**창(teleop)과 다른 점은 `enable` 이 패드에 있다는 것뿐임.** 토크가 들어가는 순간이 가장
위험해서, 한 번 눌리는 것으로는 안 나가게 `enable_hold_s`(기본 1.0초)를 둠. 정책 시작도
RB 를 쥐고 있어야 나감 -- 손이 데드맨에 가 있어야 정책이 돈다는 뜻임. 둘 다 버튼이
**안 눌림 -> 눌림** 으로 바뀐 주기에만 나가므로, Menu 를 누른 채로 RB 를 더하는 것은 안 됨
(Menu 를 뗐다가 RB 를 먼저 쥐고 다시 누를 것).

**끊기면:** `/joy` 가 0.5초 넘게 안 오면 (선이 빠짐, `joy_node` 가 죽음) 전부 놓은 것으로 보고
속도 0 을 냄. `gamepad_node` 까지 죽으면 `cmd_vel` 이 끊겨 정책 노드가 0.5초 뒤 0 으로 돌림.
**둘 다 속도만 0 이고 토크는 안 꺼짐** -- 끄는 것은 B 또는 A-7 의 명령.

축·버튼 번호는 드라이버마다 다름 (USB 의 `xpad` 와 블루투스의 `xpadneo` 가 다름). 안 맞으면
`ros2 topic echo /joy` 로 실제 번호를 보고 덮어씀.

```bash
ros2 launch huphy_ros gamepad.launch.py policy:=... \
  --ros-args -p axis_vx:=1 -p button_deadman:=5
```

패드 하나로 A-3·A-4·A-5·A-6·A-7 을 다 함.

---

명령으로 줄 때는 정책에 맞는 것 **하나**. `pub -r 20` 은 초당 20번 계속 보냄. 안 보내면 걷기는
(0, 0) = 제자리, 스쿼트는 시작값 0.8914.

**끊으면:** 속도(`cmd_vel`)는 **0.5초 뒤 0** 으로 돌아감 -- 보내던 쪽이 죽어도 마지막 속도로
계속 걸어가지 않게 (`policy_node` 의 `cmd_timeout_s`. `--teleop` 은 0.2초). 높이(`h_star`)는
마지막 값이 유지됨.

```bash
# 걷기: linear.x 앞뒤 (m/s), angular.z 돌기 (rad/s)
ros2 topic pub -r 20 /huphy/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.2}, angular: {z: 0.0}}'

# 스쿼트: 골반 높이 (m). 0.70 ~ 0.891
ros2 topic pub -r 20 /huphy/h_star std_msgs/msg/Float64 '{data: 0.80}'
```

| 명령 | 하는 일 | 기존으로 치면 |
|---|---|---|
| `topic pub cmd_vel` | 걷기 속도 명령 | `--cmd vx=0.2 wz=0` 또는 `--teleop` 의 W·S·A·D |
| `topic pub h_star` | 스쿼트 높이 명령 | `--cmd h_star=0.80` 또는 `--teleop` 의 ↑↓ |

#### A-7. 멈춤 · 토크 끄기  `[작업 기계]` · 급하면 `[모션보드]`

```bash
ros2 service call /huphy/stop_policy std_srvs/srv/Trigger   # 목표각 중단
ros2 service call /huphy/disable std_srvs/srv/Trigger       # 감쇠 5초 후 토크 차단
```

| 명령 | 하는 일 | 기존으로 치면 |
|---|---|---|
| `stop_policy` | 정책이 목표각을 그만 냄(어느 단계에서든). 모션보드는 감쇠로 버팀 | (없음 -- 둘이 한 번에 일어남) |
| `disable` | 감쇠 5초 후 토크를 끊고 `OBSERVE` 로 | **Ctrl-Q** |

급하면 **모션보드**의 런치 터미널에서 **Ctrl-C** -- 같은 절차(감쇠 후 차단)를 탐. 작업
기계의 정책 런치를 끊으면 목표각이 끊겨 모션보드가 100ms 뒤 감쇠로 떨어지지만, 토크는
`disable` 이나 모션보드 Ctrl-C 로 끊어야 함.

다시 돌리려면 A-3 (`enable`) 부터.

#### A 확인 1. 읽히는지  `[작업 기계]` -- A-1 뒤, 토크 전에

```bash
ros2 topic hz   /huphy/joint_states          # 초당 몇 번 오는지. 100 근처. Ctrl-C 로 멈춤
ros2 topic echo /huphy/joint_states --once   # 관절 각도·속도 한 번
ros2 topic echo /huphy/gravity --once        # IMU 가 본 중력 방향 한 번
ros2 topic echo /huphy/loop_status --once    # 제어 루프 상태 (모드, 무응답 수)
```

| 명령 | 하는 일 | 기존으로 치면 |
|---|---|---|
| `topic hz joint_states` | 모션보드 상태가 **네트워크 너머로** 제 주기로 오는지 | (없음 -- ROS 만의 확인) |
| `topic echo joint_states` | 관절 각도·속도 | `watch` 의 실시간 뷰어 숫자 |
| `topic echo gravity` | IMU 중력방향 | 실시간 뷰어 IMU 그래프 |
| `topic echo loop_status` | 모드·무응답 수 | 끝에 찍히는 실패 집계 |

`hz` 는 계속 찍고 안 끝남 -- 확인했으면 **Ctrl-C**. `--once` 는 하나 찍고 끝남.

| 보는 것 | 맞는 값 |
|---|---|
| 관절 이름 | `right_leg__knee` 꼴, **모터 12개.** 발목은 `ankle_a`/`ankle_b` (기존 레포·정책 계약과 같음. `ankle_pitch`/`ankle_roll` 이 보이면 예전 노드임) |
| 위치 | **라디안.** 무릎 20도면 0.35. 손으로 움직이면 따라 변함 |
| 중력방향 | 서 있으면 z 가 −1 근처 |
| `missing_cycles` | 거의 안 늚 |

#### A 확인 2. 토크 없이 정책 출력만  `[작업 기계]` -- A-2 뒤, A-3 전에

```bash
ros2 service call /huphy/approach std_srvs/srv/Trigger       # 기준 자세 목표각 (3초 뒤 ready)
ros2 service call /huphy/start_policy std_srvs/srv/Trigger   # 정책 켬
ros2 topic hz   /huphy/joint_targets                         # 목표각이 초당 몇 번. Ctrl-C
ros2 topic echo /huphy/joint_targets --once                  # 목표각 한 번
ros2 service call /huphy/stop_policy std_srvs/srv/Trigger    # 정책 끔
```

| 명령 | 하는 일 | 기존으로 치면 |
|---|---|---|
| `approach` -> `start_policy` | 정책이 목표각을 내기 시작. 모션보드가 아직 `OBSERVE` 라 **모터는 안 움직임** | **없음** -- `huphy-run` 은 늘 토크를 넣고 돎 |
| `topic hz/echo joint_targets` | 목표각이 정책 주기(50Hz 근처)로, −2 ~ 2 rad 안에서 나오는지 | (없음) |
| `stop_policy` | 정책 끔. 이 뒤에 A-3 으로 | (없음) |

---

### B. 한 기계

**모든 명령을 CAN·IMU 가 붙은 그 기계 하나에서** 침. 노드 둘을 런치 하나로 같이 띄우는 것만
다르고, 그 뒤 서비스·토픽은 A 와 같음. 런치 뒤 명령은 새 터미널에서.

#### B-1. CAN 올리고 노드 둘 띄우기

```bash
huphy-can up
ros2 launch huphy_ros local.launch.py \
  config:=$HOME/HUPHY/config/robot.yaml \
  policy:=$HOME/HUPHY/config/policies/generalized_walk
```

| 명령 | 하는 일 | 기존으로 치면 |
|---|---|---|
| `huphy-can up` | CAN 채널을 1 Mbps 로 올림 | 같음 |
| `local.launch.py` | 제어 노드(`OBSERVE`, 토크 없음)와 정책 노드(불러만 둠)를 **한 번에** 띄움. 정책은 `policy:=` 로 고름 -- `generalized_walk` / `huphy_squat` | `huphy-test --robot watch` 상태 + `huphy-run --policy <이름>` 의 불러오기 |

로그 끝에 `approach 를 기다림` 과 `OBSERVE 로 시작` 이 나오면 된 것임.

로봇 없이 점검할 때는 `huphy-can up` 대신 가짜 채널을 올리고 `config:=/tmp/robot_vcan.yaml`
-- [`without_robot.md`](without_robot.md).

#### B-2. 토크 넣기

**로봇을 들거나 테더에 맨 상태에서.**

```bash
ros2 service call /huphy/enable std_srvs/srv/Trigger
```

`OBSERVE` -> `CONTROL`. 정책이 없는 동안은 감쇠로 버팀. `huphy-run` 을 켠 순간의 토크
넣기에 해당 (단, 기준 자세로는 아직 안 감). `success: true` 가 나와야 함.

#### B-3. 기준 자세로

```bash
ros2 service call /huphy/approach std_srvs/srv/Trigger       # 3초에 걸쳐 기준 자세로, 도착하면 붙잡고 기다림
```

`stage` 가 `off -> approach -> ready`. `huphy-run` 이 켜지고 기준 자세에서 기다리는 부분.
로봇이 제대로 섰는지 보고 다음으로.

#### B-4. 정책 시작

```bash
ros2 service call /huphy/start_policy std_srvs/srv/Trigger   # ready -> running (= huphy-run 의 Enter)
```

`ready` 가 아니면 거절하고 이유를 줌. 어느 정책인지는 B-1 의 `policy:=` 로 정해짐 -- 바꾸려면
`disable` 후 런치를 다시 띄움 (제어 노드도 같이 뜨는 런치라서).

#### B-5. 명령 주기 (cmd)

창으로 -- 같은 기계에 화면이 있으면 (A-6 의 키와 같음. R 기준 자세, Enter 시작, Esc 끔):

```bash
# B-1 의 policy:= 와 같은 것
ros2 launch huphy_ros teleop.launch.py policy:=$HOME/HUPHY/config/policies/generalized_walk   # 걷기 W/S/A/D
ros2 launch huphy_ros teleop.launch.py policy:=$HOME/HUPHY/config/policies/huphy_squat        # 스쿼트 ↑/↓
```

명령으로 -- 정책에 맞는 것 하나. 끊으면 속도는 0.5초 뒤 0, 높이는 유지.

```bash
ros2 topic pub -r 20 /huphy/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.2}, angular: {z: 0.0}}'                      # 걷기 (= --cmd vx=0.2)
ros2 topic pub -r 20 /huphy/h_star std_msgs/msg/Float64 '{data: 0.80}'   # 스쿼트 (= --cmd h_star=0.80)
```

#### B-6. 멈춤 · 토크 끄기

```bash
ros2 service call /huphy/stop_policy std_srvs/srv/Trigger   # 목표각 중단
ros2 service call /huphy/disable std_srvs/srv/Trigger       # 감쇠 5초 후 토크 차단 (= Ctrl-Q)
```

급하면 런치 터미널에서 **Ctrl-C** -- 같은 절차를 탐. 다시 돌리려면 B-2 (`enable`) 부터.

#### B 확인 1. 읽히는지 -- B-1 뒤, 토크 전에

```bash
ros2 topic hz   /huphy/joint_states          # 100 근처. Ctrl-C
ros2 topic echo /huphy/joint_states --once   # 관절 각도·속도
ros2 topic echo /huphy/gravity --once        # 중력 방향
ros2 topic echo /huphy/loop_status --once    # 모드, 무응답 수
```

| 명령 | 하는 일 | 기존으로 치면 |
|---|---|---|
| `topic hz joint_states` | 상태가 제 주기로 나오는지 | (없음) |
| `topic echo joint_states` | 관절 각도·속도 | `watch` 의 실시간 뷰어 숫자 |
| `topic echo gravity` | IMU 중력방향 | 실시간 뷰어 IMU 그래프 |
| `topic echo loop_status` | 모드·무응답 수 | 끝에 찍히는 실패 집계 |

맞는 값: 이름 `right_leg__knee` 꼴로 모터 12개(발목은 `ankle_a`/`ankle_b`), 위치는 **라디안**(무릎 20도면 0.35), 중력방향 z 가
−1 근처, `missing_cycles` 가 거의 안 늚.

#### B 확인 2. 토크 없이 정책 출력만 -- B-1 뒤, B-2 전에

```bash
ros2 service call /huphy/approach std_srvs/srv/Trigger       # 기준 자세 목표각 (3초 뒤 ready)
ros2 service call /huphy/start_policy std_srvs/srv/Trigger   # 정책 켬 -- 모터는 안 움직임
ros2 topic hz   /huphy/joint_targets                         # 정책 주기 근처. Ctrl-C
ros2 topic echo /huphy/joint_targets --once                  # −2 ~ 2 rad 면 정상
ros2 service call /huphy/stop_policy std_srvs/srv/Trigger    # 정책 끔. 이 뒤에 B-2 로
```

`huphy-run` 에는 없는 단계임 -- 토크 없이 정책이 무엇을 내는지만 봄.

---

## 파라미터

### `control_node`

| 이름 | 기본 | 무엇 |
|---|---|---|
| `config` | 없음. **필수** | `robot.yaml` 경로 |
| `limb` | 빈 값 = 양다리 | 다리 하나만 돌릴 때 |
| `hz` | 100.0 | 제어 주기 |
| `state_hz` | 100.0 | 상태 발행 주기 |
| `drain_ms` | 4.0 | 응답을 기다리는 최대 시간. 다 오면 즉시 넘어가서 평소엔 안 씀. 2 면 마지막 응답 모터(ankle_b)가 가끔 무응답으로 잡혀 통신 두절로 멈췄음 |
| `send_mode` | `burst` | 명령을 내는 방식. `burst` 는 모터마다 호스트 id 가 0xF0+id 여야 함 -- 시작할 때 확인하고 아니면 멈춤. 못 맞춘 모터가 있으면 `seq` (`docs/can_communication.md` 4절) |
| `target_stale_ms` | 100.0 | 목표각이 이보다 안 오면 감쇠 |
| `allow_uncalibrated` | false | 실측 전에도 토크를 넣음 |

### `policy_node`

| 이름 | 기본 | 무엇 |
|---|---|---|
| `policy` | 없음. **필수** | 정책 꾸러미 폴더 (`config/policies/<이름>`) |
| `checkpoint` | 빈 값 = 계약의 기본값 | 꾸러미 안의 다른 체크포인트 번호 (예: 스쿼트 `4500`, 걷기 `37400` -- 걷기 기본은 `49999`). `local.launch.py` 에도 같은 인자 |
| `hz` | 0 = 계약의 `control_rate_hz` | 추론 주기. 보통 비워 둠 |
| `state_stale_ms` | 60.0 | 상태가 이보다 낡으면 목표각을 안 냄 |
| `approach_s` | 3.0 | 기준 자세로 옮기는 시간 |
| `cmd_timeout_s` | 0.5 | 속도 명령(`cmd_vel`)이 이만큼 안 오면 속도를 0 으로. 높이는 유지 |

### `teleop_node`

| 이름 | 기본 | 무엇 |
|---|---|---|
| `policy` | 없음. **필수** | 정책 꾸러미 폴더. 정책 노드와 같은 것 -- 그 정책의 명령만 조작함 |
| `max` / `accel` / `decel` | 빈 값 = 기본 | `"vx=0.8 wz=1.0"` 꼴. `huphy-run --max/--accel/--decel` 과 같음 |

**게인·관절 한계·모터 id 는 양쪽 다 없음.** `robot.yaml` 과
`config/calibration/*.json` 이 유일한 출처임. 파라미터로 또 받으면 어긋났을 때 어느
쪽이 진짜인지 알 수 없음 — 실제로 그 실수를 한 적이 있음(커밋 `4dbdfa2`, 무릎 kp 가
설정의 값 대신 코드 상수로 나감).

---

---

## 테스트

`convert.py` 와 `freshness.py` 는 ROS 없이 돎.

```bash
cd ~/HUPHY
PYTHONPATH=src python3 -m pytest tests/test_ros_convert.py tests/test_ros_freshness.py -q
```

`tests/conftest.py` 가 `ros/huphy_ros` 를 경로에 넣으므로 `PYTHONPATH` 에 따로 안
적어도 됨. 노드 둘은 `rclpy` 가 있어야 import 되어 여기서 안 봄.

---

---

## 안 될 때

```bash
echo $ROS_DOMAIN_ID          # 양쪽 같아야 함
echo $ROS_LOCALHOST_ONLY     # 비어 있거나 0
echo $RMW_IMPLEMENTATION     # 양쪽 같아야 함
ros2 node list               # 상대 노드가 보여야 함
ros2 topic info -v /huphy/joint_states    # QoS 가 양쪽 맞는지
```

| 증상 | 흔한 원인 |
|---|---|
| `ros2 node list` 에 상대가 없음 | `ROS_DOMAIN_ID` 가 다름 / 멀티캐스트가 막힘 |
| 토픽은 보이는데 값이 안 옴 | QoS 가 다름 |
| `joint_states` 가 안 뜸 | `control_node` 가 CAN 을 못 열었음. 로그를 볼 것 |
| `control_node` 가 시작하자마자 `호스트 id 가 규칙과 다른 모터` 로 끝남 | 갈아 끼운 모터의 호스트 id 를 안 맞춤. `python3 tools/can_scan.py` 로 보고 맞추거나 `-p send_mode:=seq` |

ROS 1(Noetic)이 같이 깔려 있으면 **같은 셸에서 둘 다 `source` 하지 말 것.**
`ROS_DISTRO`, `PYTHONPATH`, `CMAKE_PREFIX_PATH` 가 서로 덮어씀.

---

## 로그

노드 메시지와 `huphy` 라이브러리(CAN·모터·루프) 메시지가 **같은 ROS 로그**로 나감.
이름 앞에 노드가 붙어서 어디서 온 줄인지 보임.

```
[INFO] [...] [huphy_control]: OBSERVE 로 시작
[WARN] [...] [huphy_control.huphy.robots.leg]: right_leg/knee 응답 없음
```

| 어디서 보나 | 명령 |
|---|---|
| launch 를 띄운 터미널 | 그대로 찍힘 |
| 지난 실행 | `~/.ros/log/` 아래 실행마다 폴더 하나 |
| 두 기계 로그를 한곳에서, 레벨·노드로 걸러서 | `ros2 run rqt_console rqt_console` (`/rosout` 을 봄. 처음 한 번 `sudo apt install ros-humble-rqt-console`) |
| 화면 없이 | `ros2 topic echo /rosout` |

**자세히 보기 (DEBUG)** — 노드 이름을 붙여서 줄 것. 그냥 `debug` 를 주면 ROS 내부
(`rcl`, `rmw`) 로그까지 쏟아짐.

```bash
ros2 run huphy_ros control_node --ros-args -p config:=... --log-level huphy_control:=debug
```

특정 모듈만 조용히: `--log-level huphy_control.huphy.motors.canbus:=warn`. 노드보다 낮게
(더 자세히) 내리는 것은 안 됨 — 파이썬 쪽 문턱이 노드 레벨에 맞춰져 있음.

---


---

# 아래는 설계

왜 이렇게 만들었는지. 고치거나 늘릴 때 볼 것.

## 0. 정해진 것

| # | |
|---|---|
| 1 | 관절상태 → 정책 → 관절목표각 한 바퀴가 **네트워크를 건너감** |
| 2 | 목표각이 끊기면 **감쇠 후 토크 차단** |
| 3 | 기준 자세로 옮기는 접근 단계는 **`policy_node`** 가 함 |
| 4 | 사람이 보내는 것: 속도 명령(`vx`, `wz`) / 시작·정지·토크 / 관절 목표각 직접 |
| 5 | **정책 교체 없음.** 다른 정책은 `policy:=` 를 바꿔 노드 재시작 |
| 6 | 추론은 **타이머**, 주기는 정책 계약을 따름 (지금 둘 다 50Hz). 상태가 낡으면 목표각을 안 내보냄 |
| 7 | IMU 는 **모션보드**에 붙음. 관절 상태와 같이 네트워크를 건너감 |
| 8 | `/huphy/joint_targets` **하나**를 정책과 pose 가 같이 씀. 중재 없음 |
| 9 | 기계 간 **시계 동기 없음.** 각 노드가 자기 시계로 도착 간격만 봄 |
| 10 | 상태 발행 주기는 파라미터, **기본 100Hz** |
| 11 | **표준 메시지**만 씀. 직접 정의한 메시지 없음 |
| 12 | 낡음 기준 **60ms / 100ms**, 통신두절 5주기 |
| 13 | **기계 둘.** 모션보드에 `control_node`, 작업 기계에 `policy_node` 와 사람 |

정책은 꾸러미 폴더 하나임 (`config/policies/<이름>/deploy_contract.json` + `.pt`).
`policy_node` 는 [`control/deploy.py`](../src/huphy/control/deploy.py) 로 그 계약을
읽어 돌림 — `huphy-run` 과 같은 코드임. 명령은 정책에 있는 칸만 씀.

| 정책 | 관찰 | 명령 토픽 |
|---|---|---|
| `generalized_walk` | 44 | `/huphy/cmd_vel` (Twist) 의 `linear.x` -> `vx`, `angular.z` -> `wz`. `vy` 칸 없음 |
| `huphy_squat` | 43 | `/huphy/h_star` (Float64) -> `h_star` [m]. 0.1 m/s 로 다가감 |

---

---

## 1. ROS 개념 — 이 로봇에 짝지어서

| 개념 | 무엇 | 여기서 |
|---|---|---|
| 노드 | 프로세스 하나 안에서 도는 작업 단위 | `control_node`, `policy_node` |
| 토픽 | 이름이 붙은 단방향 방송. 받는 쪽이 없어도 보냄 | 관절 상태, IMU, 목표각, 속도 명령 |
| 서비스 | 요청-응답 한 쌍 | 토크 on/off, 정책 시작·정지 |
| 파라미터 | 노드마다 가지는 설정값 | 설정 파일 경로, 주기, 임계값 |
| 메시지 | 주고받는 자료형 | `sensor_msgs/JointState` 등 표준만 |
| QoS | 토픽마다의 신뢰성 설정 | 3-4 |
| 런치 | 노드 여러 개를 한 번에 띄우는 파이썬 스크립트 | 7-3 |

**액션은 안 씀.** 오래 걸리는 것이 접근 단계 하나인데 그건 `policy_node` 안에서
끝남 (결정 3).

---

---

## 2. 전체 그림

```
        모션보드  (로봇에 붙음)                    작업 기계
 ┌──────────────────────────┐          ┌──────────────────────────┐
 │ control_node             │          │ policy_node              │
 │                          │          │                          │
 │  메인    rclpy.spin()    │          │  메인   rclpy.spin()     │
 │  제어    ControlLoop     │          │         + 20ms 타이머    │
 │          100Hz           │          │                          │
 │  rx x2   CanBus          │          │  .pt -> numpy MLP        │
 │                          │          │  q(t-1), last_action 보관│
 └───┬──────────────────┬───┘          └────┬────────────────┬────┘
     │                  │                   │                │
     │ CAN              │   joint_states    │                │
     │ canr/canl        ├──────100Hz───────>│                │
     │ 시리얼 IMU       │   imu, gravity    │                │
     │                  │                   │                │
     │                  │<─────50Hz─────────┤                │
     │                  │   joint_targets   │                │
     v                  │                   │   cmd_vel      │
  모터 12개             │<┄┄ enable ┄┄┄┄┄┄┄ │ approach·start │
  IMU                   │     disable       │        ↓       │
                        │                   │  ┌──────────┐  │
                        ├┄┄ loop_status ┄┄> │  │  사람    │<─┘
                        │                   │  │ bag·rqt  │
                        │                   │  └──────────┘
                        └───────────────────┴──────────────────────┘
                              네트워크
                     ROS_DOMAIN_ID 양쪽 같게 · 유선
```

### 네트워크를 타는 것과 안 타는 것

```
탐         joint_states  imu  gravity      100Hz   모션 -> 작업
           joint_targets                    50Hz   작업 -> 모션
           loop_status                       5Hz   모션 -> 작업
           enable  disable                   사람  작업 -> 모션

안 탐      CAN 프레임                            모션보드 안
           시리얼 IMU 읽기                        모션보드 안
           가드·통신두절 판정                     모션보드 안
           신경망 추론                            작업 기계 안
           q(t-1) 보관                            작업 기계 안
           cmd_vel                               작업 기계 안
           approach / start_policy / stop_policy 작업 기계 안
```

### 한 주기에 값이 지나는 길

```
 1  모터 12개 ──CAN──> CanBus ──> Leg ──> Biped
                                            │ Observation (도, cal)
 2                            control_node ─┤
                                            │ JointState (rad) + Imu + Vector3
 3                      ─────네트워크─────> policy_node
                                            │
 4                            observation_vector(44칸)
                                 0-2   각속도        rad/s
                                 3-5   중력방향      단위벡터
                                 6-17  q(t-1)-기본   rad
                                 18-29 q(t)  -기본   rad
                                 30-41 직전 행동     날것
                                 42-43 vx, wz        cmd_vel 에서
 5                            model(vector) -> 행동 12
 6                            joint_targets() -> 목표각 12 (도)
                                            │ JointState (rad)
 7                      <────네트워크────── │
                                            │
 8  control_node ──> Biped ──> Leg ──> 가드 ──> MitCommand ──CAN──> 모터
```

4번의 관찰 배치는 계약 파일이 정하고 `deploy.DeployPolicy.observation_vector` 가
만듦. **`policy_node` 는 `deploy.Runner.step` 을 그대로 부름** — 배치를 ROS 쪽에서
다시 만들지 않음.

### 노드가 들고 있는 상태

둘 다 스레드가 둘 이상이라, 스레드 사이에 넘기는 값만 적음.

```
control_node
  Latest      최신 관찰        제어 스레드 -> 발행 타이머
  Freshness   최신 목표각      구독 콜백   -> 제어 스레드   기준 100ms

policy_node
  Freshness   최신 관절 상태   구독 콜백   -> 추론 타이머   기준 60ms
  Freshness   최신 IMU         구독 콜백   -> 추론 타이머   기준 60ms
  Runner      q(t-1), last_action, 명령    추론 타이머 (명령은 구독 콜백이 set)
```

명령 토픽만 나이를 안 보는 이유: 안 오면 계약의 시작값으로 두면 되고(walk 는
제자리, squat 는 기본자세 높이), 끊겼다고 로봇을 멈출 이유가 아님.

---

---

## 3. 노드 둘

### 3-1. `control_node` — 모션보드

```
하는 일   ControlLoop 를 자기 스레드에서 돌림. CAN 송수신. IMU 읽기
          가드(safety/guards.py)와 통신두절 판정(safety/link.py)은 여기서 돎
받음      /huphy/joint_targets        관절 목표각 12개
          /huphy/enable, /disable     서비스
보냄      /huphy/joint_states         모터 12개 위치·속도·토크 (발목은 ankle_a/b)
          /huphy/imu                  각속도·가속도
          /huphy/gravity              중력방향 3칸
          /huphy/loop_status          주기·밀림·무응답
```

**정책을 모름.** 목표각을 누가 보냈는지도 모름 (결정 8).

### 3-2. `policy_node` — 작업 기계

```
하는 일   .pt 를 읽어 추론. 관찰 44칸 조립, q(t-1) 보관, 행동 -> 목표각
          접근 단계(기준 자세로 보간)도 여기서 함
받음      /huphy/joint_states, /huphy/imu, /huphy/gravity
          /huphy/cmd_vel              속도 명령
          /huphy/approach, /start_policy, /stop_policy   서비스
보냄      /huphy/joint_targets        관절 목표각 12개
          /huphy/policy_status        단계·상태 나이·멈춘 이유
```

**CAN 을 모름.** 모터가 몇 개인지, id 가 뭔지 모름.

### 3-3. 사람이 치는 것

노드를 안 만듦. `policy_node` 와 **같은 기계**에서 기성 도구로 함.

```
ros2 service call /huphy/enable std_srvs/srv/Trigger
ros2 topic pub -r 20 /huphy/cmd_vel geometry_msgs/msg/Twist '{}'
ros2 bag record -a
rqt_plot /huphy/joint_states/position[3]
```

---

---

## 4. 토픽

| 토픽 | 타입 | 내는 쪽 | 주기 | QoS |
|---|---|---|---|---|
| `/huphy/joint_states` | `sensor_msgs/JointState` | control | 100Hz | sensor_data |
| `/huphy/imu` | `sensor_msgs/Imu` | control | 100Hz | sensor_data |
| `/huphy/gravity` | `geometry_msgs/Vector3Stamped` | control | 100Hz | sensor_data |
| `/huphy/loop_status` | `diagnostic_msgs/DiagnosticArray` | control | 5Hz | 기본 |
| `/huphy/joint_targets` | `sensor_msgs/JointState` | policy 또는 사람 | 50Hz | sensor_data |
| `/huphy/cmd_vel` | `geometry_msgs/Twist` | 사람 | 10~20Hz | 기본 |
| `/huphy/policy_status` | `diagnostic_msgs/DiagnosticArray` | policy | 5Hz | 기본 |

**전부 표준 메시지임** (결정 11). 직접 정의한 메시지가 없으므로 메시지 전용
패키지도 없고, `ament_python` 하나로 끝남.

### 4-1. 중력방향을 따로 내보내는 이유

`sensor_msgs/Imu` 에는 `orientation`(쿼터니언), `angular_velocity`,
`linear_acceleration` 만 있음. **중력방향 3칸이 들어갈 자리가 없음.**

이 저장소의 `ImuState` 는 자세를 중력방향으로만 들고 있음
([`sensors/base.py`](../src/huphy/sensors/base.py) 의 `gravity`). 벤더 모듈이
쿼터니언이든 오일러든 받아서 중력방향으로 만들어 올리는 구조이고, 정책이 쓰는 것도
그 3칸임 (계약의 `imu_gravity` 항목).

쿼터니언으로 되돌려 넣었다가 정책 쪽에서 다시 풀면 **왕복에서 틀릴 자리가 생김.**
그래서 `Vector3Stamped` 로 그대로 내보냄.

`Imu` 토픽도 같이 내는 이유는 표준 도구(rviz, 기록)가 그걸 보기 때문임.

### 4-2. `joint_targets` 에 `JointState` 를 쓰는 이유

목표각 전용 표준 메시지가 없음. `JointState` 의 `name` 과 `position` 만 채우고
`velocity`/`effort` 는 비움.

**받는 쪽은 `name` 을 봐야 함.** 배열 순서를 믿으면 보내는 쪽이 순서를 바꿨을 때
조용히 엉뚱한 관절이 움직임.

### 4-3. QoS

| | |
|---|---|
| `sensor_data` | BEST_EFFORT, depth 5. 빠진 것을 다시 안 보냄 |
| 기본 | RELIABLE, depth 10. 빠지면 다시 보냄 |

고주기 값에 RELIABLE 을 쓰면 안 됨 — 다시 보낼 때쯤이면 이미 낡은 값이고, 재전송이
새 값 뒤에 줄을 섬.

**보내는 쪽과 받는 쪽 설정이 다르면 연결이 아예 안 됨.** 에러도 경고도 없이 토픽이
비어 있음. `ros2 topic info -v <토픽>` 으로 양쪽을 확인함.

---

---

## 5. 단위와 이름

### 5-1. 단위

ROS 는 SI 를 씀 (REP 103). 이 저장소는 도를 씀.

| | huphy 안쪽 | ROS 경계 |
|---|---|---|
| 각도 | 도 | 라디안 |
| 각속도 | 도/초 | rad/s |
| 토크 | Nm | Nm (같음) |

변환은 `huphy_ros/convert.py` **한 곳에만** 둠. 저장소 안에 도↔라디안이 이미 두
곳 있는데(프로토콜 경계 `codec/mit.py`, 모델 경계 `control/deploy.py`)
서로 안 만남. 세 번째가 여기저기 흩어지면 어디서 한 번 더 변환됐는지 추적이 안 됨.

### 5-2. 관절 이름

`Biped` 는 `right_leg/knee` 처럼 `/` 로 팔다리를 앞에 붙임
([`robots/biped.py`](../src/huphy/robots/biped.py) 의 `SEP`).

ROS 에서 `/` 는 네임스페이스 구분자라 사람이 토픽 이름과 헷갈림. 경계에서 바꿈.

```
huphy   right_leg/knee
ROS     right_leg__knee
```

---

---

## 6. 주기와 끊김

### 6-1. 정책 추론은 타이머가 돌림

`policy_node` 가 20ms 타이머로 추론함 (결정 6). 메시지 도착이 아니라 타이머인
이유: **네트워크 지터가 그대로 추론 간격의 지터가 되기 때문**임.

정책은 20ms 간격으로 불린다고 가정하고 학습했고, 관찰의 관절 이력 `q(t-1)` 이
정확히 한 정책 주기 전 값이어야 함 (계약의 `motor_encoder_rel.history_frames`,
`control_rate_hz`).

```
타이머 20ms
  마지막 joint_states 나이 < 60ms   -> 추론하고 목표각을 냄
  나이 >= 60ms                      -> 아무것도 안 냄
```

`q(t-1)` 은 **직전 추론 때 쓴 관절각**임. 메시지 순번이 아니라 추론 호출 기준이므로,
중간에 상태가 두 번 와도 한 번만 쓰고 한 번은 버림.

### 6-2. 끊겼을 때 — 두 단계

```
상태가 60ms 넘게 안 옴
  policy_node 가 목표각 발행을 멈춤        policy_status 에 이유를 적음
      |
      | 40ms 더 지남
      v
control_node 가 목표각을 100ms 넘게 못 받음
  ControlLoop 을 멈춤 -> damp(kp=0) 5초 -> 토크 차단 -> 스스로 OBSERVE 로 돌아감
```

`stop_policy` 를 불러도, `policy_node` 가 꺼져도 이 길을 탐. **OBSERVE 로 돌아가므로 상태는
계속 읽힘.** 다시 돌리려면 `enable` → `approach` → `start_policy`.

`enable` 직후 목표각이 **아직 한 번도 안 왔으면** 이 판정을 안 함 — 그 동안은 감쇠 명령을 보냄
(실행 A-3 / B-2).

`100 > 60` 이므로 순서가 뒤바뀌지 않음. 정책이 먼저 멈추고, 그 뒤에 제어가 멈춤.

| 값 | 어디 | 뜻 |
|---|---|---|
| 60ms | `policy_node` | 상태가 이보다 낡으면 목표각을 안 냄. 50Hz 기준 3주기 |
| 100ms | `control_node` | 목표각이 이보다 안 오면 감쇠. 50Hz 기준 5주기 |
| 5주기 | `robot.yaml` 의 `safety.link_loss_cycles` | 모터가 연속으로 답을 안 하면 정지 |

앞의 둘은 **실물 측정 전의 제안값**임.

### 6-3. 시계

기계 간 시계를 안 맞춤 (결정 9). 각 노드가 **자기 시계로 마지막 수신 이후 흐른
시간**만 봄. 편도 지연은 알 수 없고, 끊겼는지는 알 수 있음.

메시지의 `header.stamp` 는 **채워서 보내되 나이 판정에는 안 씀.** 기록(`ros2 bag`)과
나중에 시계를 맞출 때를 위해 남겨 두는 것임.

---

---

## 7. 안전

| 상황 | 막는 것 | 어디 |
|---|---|---|
| 목표각이 안 옴 | 감쇠 후 토크 차단 | `control_node`, 100ms |
| 상태가 안 옴 | 목표각 발행 중단 | `policy_node`, 60ms |
| 모터가 답을 안 함 | 루프 정지 | `safety/link.py`, 이미 있음 |
| 한계·점프 | 자름 | `safety/guards.py`, 이미 있음 |
| 명령값이 NaN | 그 모터만 안 보냄 | `safety/guards.py`, 이미 있음 |
| 노드를 `kill -9` | **못 막음.** 토크가 켜진 채 남음 | — |

**토크는 명시해야 들어감.** `control_node` 가 `Mode.OBSERVE` 로 시작하고,
`/huphy/enable` 서비스를 받아야 `Mode.CONTROL` 이 됨.

**정책과 pose 를 동시에 쓰면 안 됨** (결정 8). 토픽 하나를 같이 쓰므로 둘 다 쏘면
번갈아 들어감. 사람이 `policy_node` 를 시작·정지해서 정함.

---

---

## 8. 코드

### 8-1. 파일

| 파일 | 하는 일 | `rclpy` |
|---|---|---|
| `huphy_ros/convert.py` | 도↔라디안, 관절 이름 변환 | **안 씀** |
| `huphy_ros/freshness.py` | 받은 값과 그 나이. 낡음 판정 | **안 씀** |
| `huphy_ros/control_node.py` | `ControlLoop` 를 자기 스레드에서. CAN | 씀 |
| `huphy_ros/policy_node.py` | 20ms 타이머 추론. 접근 단계 | 씀 |
| `huphy_ros/logbridge.py` | 파이썬 `logging` → ROS 로그. 노드 시작 때 붙임 | 씀 |

앞의 둘이 `rclpy` 를 안 쓰는 것이 의도임. 거기 있는 판단(단위, 이름, 낡음)이 틀리면
로봇이 잘못 움직이는데 **틀려도 길이와 자릿수가 그대로라 코드로는 안 잡힘.** ROS 가
없는 기계에서 테스트가 돌아야 고정해 둘 수 있음.

### 8-2. `convert.py`

| 함수 | 방향 |
|---|---|
| `to_ros_name` / `to_huphy_name` | `right_leg/knee` ↔ `right_leg__knee` |
| `joint_names` | 관찰 사전에서 관절 이름을 뽑음. `.pos` 가 있는 것만 |
| `observation_to_arrays` | 관찰(도) → `JointState` 네 배열(rad) |
| `arrays_to_observation` | `JointState` → 관찰 사전. `policy_node` 가 씀 |
| `targets_to_arrays` | 목표각(도) → `JointState`(rad). 순서는 `ORDER` 가 정함 |
| `arrays_to_targets` | `JointState` → `Action`(도). `control_node` 가 씀 |
| `imu_arrays` | `ImuState` → 각속도(rad/s)·가속도·중력방향 |

**없는 항목은 `0.0` 으로 채움.** `JointState` 는 네 배열의 길이가 같아야 하고 다르면
받는 쪽이 짝을 못 맞춤 — 0 이 옳은 값이라는 뜻이 아니라 길이를 맞추는 것이 필수라서임.

**길이가 다르면 멈춤.** `zip` 은 짧은 쪽에 맞춰 조용히 잘라서, 관절 하나가 빠진
명령이 그대로 통과함.

### 8-3. `freshness.py`

```python
Freshness(stale_s, clock=time.monotonic)
    put(value)          받은 값을 넣음. ROS 콜백에서
    take() -> (값, 쓸 수 있나)
    age()               마지막 수신 이후 초. 없었으면 -1.0
    clear()             비움

Latest()
    put(value) / take()     나이를 안 봄
```

`Latest` 는 반대 방향에 씀 — 제어 스레드가 관찰을 써 두고 발행 타이머가 꺼냄.
발행이 한 번 늦는 것은 문제가 아니므로 낡음 판정이 필요 없음.

**"아직 없음" 을 `None` 으로 둠.** `0.0` 을 쓰면 안 되는 이유는 `clock()` 이
정말로 `0.0` 을 낼 수 있기 때문임 — `monotonic` 은 기준점이 정해져 있지 않음.
만들다 이걸 테스트가 잡았음.

`time.monotonic` 을 쓰는 이유: 벽시계는 보정으로 뒤로 갈 수 있어 나이가 음수가 됨.
`huphy` 전체가 같은 이유로 `monotonic` 을 씀.

### 8-4. `control_node.py` — 모드를 루프 재시작으로 바꿈

`ControlLoop` 는 모드를 **생성할 때** 받고 `_enter` 에서 `enable()`/`disable()` 을
부름. 돌고 있는 루프의 모드를 바꾸는 길이 없음.

```
시작        OBSERVE 루프를 띄움        토크 안 들어감
/enable     OBSERVE 를 멈추고 CONTROL 루프를 새로 띄움
/disable    CONTROL 을 멈춤 -> finally 가 감쇠 후 토크 차단
```

로봇 객체는 그대로 두고 루프만 바꿈. `_enter` 가 `is_connected` 를 보고 이미
연결돼 있으면 다시 안 엶.

`CONTROL` 을 멈출 때는 `_exit` 가 `DAMPING_S`(5초) 동안 감쇠를 보냄. 그만큼 걸림.

**`enable` 서비스는 토크가 실제로 들어갔는지 확인하고 답함.** 루프를 띄우기만 하고
성공으로 답하면, `robot.enable()` 이 실측값이 없어 예외를 낼 때 스레드가 조용히
죽는데 부르는 쪽은 토크가 들어간 줄 앎.

**목표각이 낡으면 `_motion` 안에서 `loop.stop()`.** 플래그만 세우는 일이라 그 주기는
정상으로 끝나고, `finally` 가 감쇠 뒤 토크를 끊음 (결정 2).

### 8-5. `policy_node.py` — 네 단계

```
꺼짐      아무것도 안 냄
접근      지금 자세에서 기준 자세로 approach_s 초 동안 보간한 목표각
대기      기준 자세를 계속 냄. 사람이 자리를 잡는 시간
정책      신경망 출력
```

`/huphy/approach` 가 꺼짐 → 접근, `approach_s` 가 지나면 저절로 대기. `/huphy/start_policy`
가 대기 → 정책 (`huphy-run` 의 Enter) -- 대기가 아니면 이유를 붙여 거절함.
`/huphy/stop_policy` 가 어디서든 꺼짐으로.

접근과 정책 시작을 나눈 이유: 하나로 묶여 있으면 기준 자세에 도착하자마자 정책으로 넘어가서,
로봇이 제대로 섰는지 보고 시작할 틈이 없음. `huphy-run` 도 기준 자세에서 Enter 를 기다림.

접근을 여기서 하는 이유 (결정 3): 기준 자세(계약의 행동 오프셋,
`DeployPolicy.default_pose()`)를 아는 쪽이 여기뿐임. 모션보드에도 두면 같은 값이 두
군데가 됨.

**`DeployPolicy.motion()` 대신 `Runner` 를 직접 들고 있음.** 명령이 토픽으로 매 주기
바뀔 수 있어서, 구독 콜백이 `runner.commands.set(이름, 값)` 으로 넣고 타이머가
`runner.step()` 을 부름. `huphy-run` 은 같은 `Runner` 를 `motion()` 으로 감싸 쓸 뿐임.

```python
self.policy = deploy.load(folder)
self.runner = self.policy.runner(self.policy.load_model(checkpoint))
targets = self.runner.step(observation, imu_state)     # 20ms 타이머
```

접근 단계는 `scripts.selftest.approach(targets, start, seconds)` 를 그대로 씀.
지금 자세에서 목표까지 선형 보간하는 함수를 내는 순수 함수라 ROS 와 무관함.

---

---

## 9. `src/huphy/` 는 안 고침

두 노드가 쓰는 것이 이미 전부 공개돼 있음.

| 노드 | `huphy` 에서 쓰는 것 |
|---|---|
| `control_node` | `config.load_robot`, `scripts.bringup.build_biped`, `control.ControlLoop`, `control.Mode` |
| `policy_node` | `control.deploy` 의 `load`·`DeployPolicy.load_model`·`runner`·`default_pose`·`order`, `scripts.selftest.approach` |

### `huphy` 를 ROS 패키지로 안 만드는 이유

**빌드 경로가 둘이 됨.** `ament_python` 은 `setup.py` 를 보는데 이 저장소는
`pyproject.toml` 만 있음. 둘을 같이 두면 `pip install -e .` 과 `colcon build` 가
다른 경로로 설치해서 어느 쪽이 import 되는지가 환경에 달라짐.

**ROS 없이 못 돌게 됨.** 지금 `src/huphy/` 와 `tests/` 가 ROS 없는 기계에서 돎.
`package.xml` 을 붙이면 그 전제가 흐려짐.

`rosdep` 은 apt 패키지와 몇몇 pip 패키지만 알아서 `huphy` 를 `package.xml` 에 적을
수 없음. `pip install -e .` 를 전제로 둠.

---

---

## 10. 스레드

`control_node` 한 프로세스 안에 넷임.

| 스레드 | 하는 일 |
|---|---|
| 메인 | `rclpy.spin()`. 구독·서비스·발행 타이머 |
| 제어 | `ControlLoop.run()`. 100Hz, CAN 송수신 |
| 수신 ×2 | `CanBus._read_loop`. 채널마다. `huphy` 가 이미 띄움 |

**제어 스레드에서 `publish()` 를 부르지 않음.** 직렬화와 DDS 가 제어 주기 안에
들어감. 제어 스레드는 최신 관찰을 `Latest` 에 써 두기만 하고 발행 타이머가 꺼냄.

**`ControlLoop` 를 `rclpy` 타이머에 얹지 않음.** 절대 마감·스핀 구간·밀림 측정으로
주기를 자기가 지키는데([`control/loop.py`](../src/huphy/control/loop.py) 의
`_sleep_until`, `SPIN_THRESHOLD_S`), executor 가 콜백을 꺼내 부르는 구조 위에
얹으면 그 지터가 더해짐.

**콜백 안에서 계산하지 않음.** 파이썬은 GIL 을 쥔 스레드가
`sys.getswitchinterval()`(기본 0.005초) 동안 시스템콜 없이 계속 돌 수 있음. 수거
예산이 2ms 이므로, 계산만 하는 콜백 하나가 그 주기의 수거를 통째로 넘기게 만듦.

---

---

## 11. 센서를 붙일 때

카메라·라이다를 **모션보드에 붙이지 않음.** 거기는 CAN 과 100Hz 제어뿐임.

그러면 작업 기계에 붙게 되는데, **거기가 정책이 도는 기계임** (결정 13). 그래서
막아야 할 것이 제어가 아니라 **정책**임.

| 무엇이 다투나 | 왜 |
|---|---|
| CPU | 포인트클라우드 처리와 50Hz 추론이 같은 코어를 씀 |
| GIL | 센서 노드가 파이썬이면 10장의 전환 간격 문제가 추론 타이머에 걸림 |
| 대역 | 프레임 하나가 수십 MB 라 UDP 조각으로 나감. 같은 링크의 `joint_targets` 가 그 뒤에 줄을 섬 |

마지막 것이 제일 나쁨. `joint_targets` 가 50Hz 로 가야 하는데 한 번 늦으면
모션보드의 100ms 임계에 가까워지고, 몇 번 이어지면 감쇠로 떨어짐.

대책 넷임. 위에서부터 효과가 큼.

| | |
|---|---|
| 센서를 **전용 보드**에 | 정책과 CPU·GIL 을 안 나눔. 기계가 셋이 됨 |
| 망을 나눔 | 제어용 링크와 센서용 링크를 물리적으로 분리 |
| `ROS_DOMAIN_ID` 를 나눔 | 센서 도메인과 제어 도메인을 나누고 필요한 토픽만 넘김 |
| 센서 노드를 C++ 로 | `realsense-ros`·`ouster-ros` 가 이미 C++ 임. GIL 을 안 씀 |

마지막 것이 공짜임 — 기성 드라이버가 이미 C++ 라 **파이썬 노드를 따로 안 만들면**
GIL 문제는 안 생김. CPU 와 대역은 남음.

**붙이기 전에 재야 할 것**: 센서 노드를 띄운 상태에서 `/huphy/joint_targets` 의
주기가 50Hz 를 지키는지. `ros2 topic hz` 로 보면 됨.

---

---

## 12. 안 하는 것

| | 왜 |
|---|---|
| `ros2_control` | 발목이 크랭크 두 개로 두 축을 만드는 링키지라 기성 컨트롤러에 그 변환이 없음. 커스텀 컨트롤러를 C++ 로 써야 해서 기성품을 쓰는 이점이 사라짐 |
| 직접 정의한 메시지 | 표준으로 됨. 만들면 `ament_cmake` 패키지가 하나 더 필요함 |
| URDF / `robot_state_publisher` | rviz 로 보고 싶어질 때. 제어에는 안 쓰임 |
| 정책 교체 서비스 | 결정 5 |
| 액션 | 오래 걸리는 것이 접근 단계뿐이고 `policy_node` 안에서 끝남 |

### 아직 못 한 것

| 확인함 | 안 함 |
|---|---|
| 한 기계에서 실물로 B 확인 1·2 (상태 100Hz, IMU, 토크 없이 목표각) | 토크 넣기·기준 자세·정책 시작 (A-3~A-5 / B-2~B-4) — **ROS 로 토크를 넣어 본 적 없음** |
| 로봇 없이 가짜 CAN 으로 노드 둘 ([`without_robot.md`](without_robot.md)) | **두 기계** 구성 |
| | `huphy-run` 에만 있는 것: 로그(`--log`), 실시간 그래프(`--live`), `--profile`, `--gain-scale`, `--teleop` |

두 기계 사이 왕복 지연도 안 쟀음. 6-2 의 60ms / 100ms 는 제안값이고, 재고 나서 다시
정해야 함.
