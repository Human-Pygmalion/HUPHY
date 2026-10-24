# 로봇 없이 점검

가짜 CAN 장치를 만들어 **ROS 배선만** 확인함. 모터가 없어도 됨.

실행 절차 자체는 [`README.md`](README.md) 의 "실행" 과 같고, 바뀌는 것은 설정
파일에 적힌 채널뿐임.

| 확인됨 | 안 됨 |
|---|---|
| 토픽·주기·이름·단위·QoS | 모터 응답 (아무도 안 답함) |
| 정책이 받아서 추론하고 목표각을 내는지 | 실제 움직임·토크 |
| 서비스 | 캘리브레이션 |
| `control_node` 가 CAN 에 프레임을 내보내는지 | 발목 기구학이 실물과 맞는지 |

---

## 1. 가짜 채널

```bash
sudo modprobe vcan
for c in vcan0 vcan1; do
  sudo ip link add dev $c type vcan 2>/dev/null
  sudo ip link set up $c
done
ip -br link show type vcan
```

`huphy-can up` 은 **쓰면 안 됨** — `ip link set ... type can bitrate` 를 거는데
vcan 에는 비트레이트가 없어서 실패함. 위 명령으로 올림.

## 2. 설정 사본

채널 이름만 바꾼 사본을 만듦. `robot.yaml` 은 안 건드림.

```bash
cd ~/HUPHY
sed -e 's/channel: canr/channel: vcan0/' -e 's/channel: canl/channel: vcan1/' \
    config/robot.yaml > /tmp/robot_vcan.yaml
```

## 3. 띄움

```bash
ros2 launch huphy_ros local.launch.py \
  config:=/tmp/robot_vcan.yaml \
  policy:=$HOME/HUPHY/config/policies/generalized_walk
```

한 기계에서 둘 다 띄우는 런치임. `OBSERVE` 로 시작하므로 토크 경로를 안 탐.

---

## 4. ROS 쪽 확인

```bash
ros2 node list                          # /huphy_control, /huphy_policy
ros2 topic hz   /huphy/joint_states     # 100 근처
ros2 topic echo /huphy/joint_states --once
ros2 topic echo /huphy/loop_status --once
```

| 보는 것 | 맞는 값 |
|---|---|
| 관절 이름 | `right_leg__hip_pitch` 꼴, 12개 |
| `position` | **전부 0.** 아무도 안 답하니 상태가 안 들어옴 |
| `hz` | 100 근처. 첫 줄이 찍히기까지 몇 초 걸림 |
| `missing_cycles` | **주기마다 늘어남.** `cycles` 와 같은 값 |
| `mean_hz` · `overruns` | **40 근처, 밀림이 계속 늚.** 매 주기 응답을 끝까지 기다리기 때문 |

`position` 0, `missing_cycles` 증가, 루프 40Hz 는 **여기서는 맞는 결과**임. 실물에서
같은 값이 나오면 그건 고장임.

launch 로그의 `IMU main 를 열지 못함` 경고도 IMU 가 없는 기계에서는 정상임.

## 5. CAN 쪽 확인

```bash
sudo apt install can-utils
candump vcan0
```

`control_node` 가 보내는 MIT 프레임이 찍혀야 함.

**안 찍히면 ROS 가 아니라 그 아래 계층 문제임** — `CanBus` 가 채널을 못 열었거나
`ControlLoop` 이 안 도는 것. 노드 로그를 볼 것.

## 6. 정책 확인

```bash
ros2 service call /huphy/start_policy std_srvs/srv/Trigger
ros2 topic hz   /huphy/joint_targets         # 50 근처
ros2 topic echo /huphy/policy_status --once  # stage: approach -> running
```

```bash
ros2 topic pub -r 20 /huphy/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.2}, angular: {z: 0.0}}'
ros2 topic echo /huphy/policy_status --once  # vx 가 0.200
```

여기까지 되면 **ROS 배선은 전부 맞는 것임.** 남은 것은 모터뿐.

---

## `enable` 을 부르지 말 것

`OBSERVE` 는 무응답을 세기만 함. `enable` 로 `CONTROL` 에 들어가면 `LinkWatch` 가
**5주기 만에 루프를 멈춤** (`robot.yaml` 의 `safety.link_loss_cycles`) — 아무도 안
답하니 당연한 동작임. 가짜 CAN 에서 토크 경로는 볼 것이 없음.

## sudo 를 못 쓸 때

`python-can` 내장 가상 버스를 씀. 커널 모듈이 필요 없는 대신 `candump` 로 프레임을
못 봄 — 5번을 건너뛰게 됨.

```bash
sed -e 's/channel: canr/channel: v0/' -e 's/channel: canl/channel: v1/' \
    -e 's/interface: socketcan/interface: virtual/' \
    config/robot.yaml > /tmp/robot_virtual.yaml
```

## 치우기

```bash
for c in vcan0 vcan1; do sudo ip link del $c; done
```

---

## ROS 없이, 로봇도 없이

`huphy` 계층만 보려면 ROS 를 안 거쳐도 됨. 같은 가짜 채널에 대고 돌림.

```bash
huphy-test --robot --config /tmp/robot_vcan.yaml zero
```

끝나면 실패 집계가 찍힘. 여기서 프레임이 안 나가면 ROS 를 올려도 안 나감.
