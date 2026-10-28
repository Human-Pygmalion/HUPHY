# 빠른 사용법

레포를 받은 뒤 CAN 을 올리고 정책을 돌리기까지. 왜 그런지는
[루트 README](README.md), ROS 는 [`ros/README.md`](ros/README.md).

```bash
pip install -e .
```

---

## 1. CAN 선

```bash
huphy-can up          # 로봇 돌리기 전에. 내렸다가 1 Mbps 로 올림
huphy-can status      # 어댑터·채널·오류 상태
huphy-can down        # 끝나고
```

채널 이름은 `config/robot.yaml` 에서 읽음. 특정 채널만 하려면 `--channel canr`.

### 안 되면

```bash
huphy-can status
```

| 증상 | 볼 것 |
|---|---|
| 채널이 안 보임 | USB 어댑터가 꽂혀 있는지. `ip -br link show type can` |
| `up` 이 실패 | 권한. `sudo` 가 필요할 수 있음 |
| 올라갔는데 응답이 없음 | 전원, 종단저항, 배선. [`docs/can_communication.md`](docs/can_communication.md) |

---

## 2. 모터가 살아 있나 — 정책 없이

**토크를 넣기 전에 여기부터.**

```bash
huphy-test --robot watch        # 토크 끄고 위치만 읽음. 손으로 움직여 봄
```

`watch` 는 힘이 안 들어감. 손으로 관절을 움직였을 때 값이 따라 변하면 통신·실측이
맞는 것임.

```bash
huphy-test --robot zero                      # 전부 0도로 붙잡음
huphy-test --robot pose knee=20 hip_pitch=-10   # 준 각도로
huphy-test --robot studio                    # 웹에서 토크·게인·자세를 다룸
```

처음 통전할 때는 게인을 낮춰서.

```bash
huphy-test --robot zero --gain-scale 0.3
```

Ctrl-Q 로 멈춤. 끝나면 실패 집계가 찍힘.

---

## 3. 정책

### 있는 정책

| `--policy` | 무엇 | 명령 (`--cmd`) | 단위 | 체크포인트 |
|---|---|---|---|---|
| `generalized_walk` | 속도 명령으로 걷기 | `vx` 앞뒤<br>`wz` 돌기 | m/s<br>rad/s | `37400` |
| `huphy_squat` | 골반 높이를 잡고 서기 | `h_star` 높이 | m. **0.70~0.891** | `5000`(기본) `4500` `4000` |

둘 다 50Hz, 관절 12개. 명령을 안 주면 전부 0 — 걷기는 제자리, 스쿼트는 계약의
기본 높이.

**명령 이름은 정책이 정함.** `config/policies/<이름>/deploy_contract.json` 에
적혀 있고, 없는 이름을 주면 시작 전에 멈춤.

### 인자

| 인자 | 기본 | 무엇 |
|---|---|---|
| `--policy NAME` | **필수** | `config/policies/<이름>/` 폴더. 경로도 됨 |
| `--cmd NAME=VALUE` | 계약의 기본값 | 명령. 여러 번 줄 수 있음. **실행 내내 고정** |
| `--teleop` | 끔 | 고정 대신 pygame 창으로 실시간 조작 |
| `--checkpoint N` | 계약의 기본 | 같은 정책의 다른 가중치 |
| `--gain-scale S` | 1.0 | 게인 배율. 숫자 둘이면 `나머지 발목` |
| `--approach S` | 3 | 기준 자세까지 옮기는 시간(초) |
| `--duration S` | 없음 | 이 초만 돌고 나옴 |
| `--live [PORT]` | 끔 | 브라우저로 봄. 기본 8770 |
| `--log [DIR]` | 끔 | JSON 으로 남김. 기본 `logs/`, 종류별 폴더 |

---

### 쓰는 법

```bash
huphy-run --policy generalized_walk                       # 제자리
huphy-run --policy generalized_walk --cmd vx=0.3          # 앞으로
huphy-run --policy generalized_walk --cmd vx=0.3 --cmd wz=0.2
huphy-run --policy huphy_squat --cmd h_star=0.80
huphy-run --policy huphy_squat --checkpoint 4500
```

3초에 걸쳐 기준 자세로 간 뒤 **Enter 를 눌러야** 정책이 시작됨. **Ctrl-Q** 로 멈춤.

### 실시간 조작

```bash
huphy-run --policy generalized_walk --teleop
```

pygame 창이 뜸. 창에서 **Enter 시작**, **Space 속도 0**, **Esc 멈춤**.

| 정책 | 키 |
|---|---|
| `generalized_walk` | `W`/`S` 앞뒤, `A`/`D` 돌기. 누르는 동안 가속, 떼면 0 으로 |
| `huphy_squat` | `↑`/`↓` 높이 |

상한과 가감속.

```bash
huphy-run --policy generalized_walk --teleop \
  --max vx=0.5 --accel vx=0.5 --decel vx=1.0
```

화면이 필요함. ssh 면 `ssh -X`.

### 첫 통전

```bash
huphy-run --policy generalized_walk --gain-scale 0.3 --duration 10
```

`--gain-scale` 은 게인을 낮춰 첫 통전을 확인하는 용도임. **정책은 학습 게인을
전제로 하므로 낮추면 시뮬과 다르게 움직임.**

---

## 4. 보면서 돌리기

```bash
huphy-run --policy generalized_walk --live
```

`http://<이 기계 IP>:8770` 에서 관절 목표·실측을 봄. 같은 망의 다른 기계에서도
열림. 포트를 바꾸려면 `--live 9000`.

```bash
huphy-run --policy generalized_walk --log
```

`logs/policy/` 와 `logs/latency/` 에 JSON 으로 남김. 열어 보는 법.

```
tools/policy_viewer.html      정책 입출력
tools/latency_viewer.html     관절별 응답 지연
```

둘 다 쓸 수 있음.

```bash
huphy-run --policy generalized_walk --live --log
```

---

## 5. 통신이 이상할 때

```bash
huphy-run --policy generalized_walk --drain-ms 6   # 정책은 50Hz(20ms) 라 4 보다 늘릴 여유가 있음
```

| 인자 | 무엇 |
|---|---|
| `--drain-ms 6` | 응답을 기다리는 시간을 늘림. 기본 4ms (100Hz 에서는 4 가 사실상 상한, 50Hz 면 더 늘려도 됨) |
| `--send-mode seq` | 한 모터씩 보냄. 기본은 `burst`(6개를 연달아). burst 는 모터마다 호스트 id 가 0xF0+id 여야 하고, 아니면 연결할 때 멈춤 -- 그때 쓰는 길 |
| `--extra` | 추가 값(모터 전류 iq·전압 VBUS)을 중간중간 읽어 옴. 실시간 뷰어와 `--log` 기록에 `extra/<모터>/iq·temp·vbus` 열이 생김. burst 에서만. 한 주기 읽기 수는 `--extra-per-cycle N` (다리당, 기본 2) |
| `--hz 100` | 모터 전송 주기. 정책은 항상 자기 주기(50Hz)로만 불리고, 그 사이는 직전 목표를 반복 전송 |
| `--profile` | `huphy-run` 만. 끝에 한 주기 안 단계(관측/정책/송신/수거)별 소요 시간을 평균·p95·최대로 찍음. 주기를 못 지킬 때 어느 단계가 범인인지 좁힘 |

끝나면 실패 집계가 찍힘. 자세한 것은
[`docs/can_communication.md`](docs/can_communication.md).

---

## 자주 쓰는 것만

```bash
huphy-can up                                          # 선 올리기
huphy-test --robot watch                              # 손으로 움직여 확인
huphy-test --robot zero --gain-scale 0.3              # 토크 넣어 보기
huphy-run --policy generalized_walk --gain-scale 0.3 --duration 10
huphy-run --policy generalized_walk --teleop --live
huphy-can down                                        # 끝나고
```

멈춤은 **Ctrl-Q**. 멈출 때 `kp=0` 으로 속도를 죽인 뒤 토크를 끊음 (5초).

---

## 그 밖의 명령

| | |
|---|---|
| `huphy-commission` | 조립할 때 한 번. 영점·가동범위 실측 |
| `huphy-bringup` | 관절 하나씩 골라 움직여 보는 메뉴 |
| `huphy-imu` | IMU 설정·부착 방향 확인 |

`--help` 로 각자 인자를 볼 수 있음.
