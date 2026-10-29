# `tools/` — 기록을 보고 센서를 확인하는 도구

```
tools/
├── policy_viewer.html    정책 입출력 기록을 재생하듯 봄
├── latency_viewer.html   관절별 응답 지연 히스토그램
│   (실시간 뷰어)          huphy-test / huphy-run --live -- 도는 동안 목표·실측
├── imu_viewer.py         정책이 받는 IMU 값을 실시간으로 보고 단위·좌표계를 검사
├── make_sequences.py     studio 시퀀스(스쿼트, 정적 보행)를 계산으로 만들고 한계를 검사
├── sequence_preview.html 그 시퀀스를 막대 그림으로 재생 (make_sequences.py 가 채워 씀)
├── can_scan.py           CAN 라인을 훑어 모터마다 호스트 id·전압·전류·zero_sta 를 표로
├── gamepad_viewer.py     패드 입력을 브라우저로 실시간 -- ROS 없이 패드를 직접 읽음
└── rs_ota.py             RobStride 펌웨어 업데이트 (CAN, private 프로토콜)
```

**셋 다 외부 라이브러리가 없음.** 로봇(Jetson)에서 인터넷 없이 열려야 해서임. html 은
브라우저로 열고 파일을 끌어다 놓거나 "파일 열기" 로 고름. 기록 파일은 `huphy-run --log`
과 `huphy-test --log` 이 `logs/` 에 남김.

```bash
huphy-run --robot --policy stand --ankle-space ab --weights config/policies/stand_bi.pt \
          --hz 50 --drain-ms 4 --log
# 끝나면
#   logs/policy/<월일>_policy_<정책>[_<인자>].json    -> policy_viewer.html
#   logs/latency/<월일>_latency_<정책>[_<인자>].json  -> latency_viewer.html

huphy-test --robot --hz 50 --drain-ms 4 zero --log
# 끝나면
#   logs/track/<월일>_track_zero[_<인자>].json      -> policy_viewer.html (목표·실측만)
#   logs/latency/<월일>_latency_zero[_<인자>].json  -> latency_viewer.html
```

---

## `policy_viewer.html`

정책이 불릴 때마다 남은 한 줄을 시간축으로 재생함.

### 기록의 열

| 열 | 무엇 | 단위 |
|---|---|---|
| `t` | 첫 정책 호출부터 흐른 초 | s |
| `obs/...` | 모델 입력 그대로 (정규화 전). 이름은 `stand.OBS_NAMES` / `policy.observation_names` | rad, rad/s |
| `act/<관절>` | 모델 출력 그대로 (배율·기본자세 입히기 전) | — |
| `tgt/<관절>` | 관절 목표각. 루프에 실제로 넘긴 것 | 도 |
| `pos/<관절>` | 그 순간의 실측 관절각 | 도 |
| `imu/<이름>/<값>` | IMU. `huphy-test --log`·실시간 녹화에만 있음. roll/pitch/yaw, gx/gy/gz, grav_x/y/z, ax/ay/az, age | 도, 도/초, —, m/s², ms |
| `extra/<모터>/<값>` | 추가 값. `--extra` 로 돌렸을 때만 있음. iq(전류), temp(온도), vbus(전압). 전류·전압은 몇 주기마다 갱신되고 그 사이는 마지막 값 | A, °C, V |

`stand` 의 관찰은 `obs/gyro/{x,y,z}`, `obs/grav/{x,y,z}`, `obs/q_prev/<관절>`,
`obs/q/<관절>`, `obs/last_act/<관절>`, `obs/cmd/{vx,vy,wz}` 순서임 (45칸).

### 화면

| 그래프 | 무엇 |
|---|---|
| 관절별 추종 | 관절마다 한 칸. 목표(점선)·실측(실선)을 겹치고 사이를 칠해 오차를 면적으로 보임. 머리에 지금 오차와 보이는 창의 RMS·최대 |
| 오차 | 목표 − 실측을 고른 관절끼리 한 그래프에. 0 기준선 |
| 추종 겹쳐 보기 | 고른 관절의 목표·실측을 한 그래프에 (기본 꺼짐) |
| 관찰 · gyro / grav / q / q_prev / last_act / cmd | 모델 입력 묶음 |
| 행동 | 모델 출력 |

오차는 파일에 없고 열 때 `tgt − pos` 로 계산함. 예전 기록도 그대로 열림.

**`huphy-test` 기록(`track_*.json`)도 열림.** `t`, `tgt/`, `pos/`, `imu/` 만 있는 파일임 --
관절 목록을 `tgt/` 에서 읽고, 관찰·행동 그래프 대신 IMU 그래프(자세·각속도 기본)가 나옴. 목표가 고정인 `zero`·`pose` 에서
실측이 어떻게 따라가고 얼마나 처지는지 보는 용도.

| 조작 | |
|---|---|
| 재생 · 속도 | 0.25x ~ 4x |
| 창 | 보이는 시간 폭 2/5/10/30 s · 전체 |
| y 창 기준 | 끄면 실행 전체 범위로 y 를 고정, 켜면 보이는 창에 맞춤. 시작 직후 크게 튄 구간이 있으면 켤 것 |
| 관절 칩 | 그래프에 낼 관절. 전체 / 오른 / 왼 / 없음 |
| 그래프 클릭·끌기 | 그 시각으로 이동 |
| `Space` / `←` `→` / `Shift+←` `→` / `Home` `End` | 재생·정지 / 한 줄 / 1초 / 처음·끝 |

**정책이 한 번도 안 불리면(Enter 를 안 누름) 기록이 안 남음.** 메모리에 쌓았다가 끝날
때 쓰므로 강제로 죽인 실행도 안 남음.

---

## 실시간 뷰어 (`--live`)

`policy_viewer` 의 "관절별 추종" 을 **도는 동안** 보는 화면. 파일이 아니라 실행 중인
`huphy-test` / `huphy-run` 이 직접 내줌 (`src/huphy/telemetry/live_viewer.html`).

```bash
huphy-test --robot --hz 50 zero --live          # 포트 8770
huphy-run --robot --policy stand --ankle-space ab --hz 50 --live 8771
# 시작할 때 주소를 찍음
#   실시간 뷰어  http://127.0.0.1:8770   http://192.168.0.x:8770
```

| 화면 | 무엇 |
|---|---|
| 관절별 | 관절마다 목표(점선)·실측(실선)·사이 칠함. 머리에 지금 값과 보이는 창의 RMS·최대 오차 |
| 오차 | 고른 관절의 목표 − 실측을 한 그래프에 (끌 수 있음) |
| IMU | 자세(roll/pitch/yaw)·각속도가 기본. 옆 패널에서 중력방향·가속도·수신 나이를 켬. 머리 줄에 축별 지금 값 |
| 지금 값 | 관절별 목표·실측·오차 표 |
| 녹화 / `R` | 누른 때부터 정지까지를 JSON 으로 저장. 다시 누르면 정지하고 저장 |
| 멈춤 / `Space` | 화면만 멈춤. 받기는 계속함 |
| 창 | 5/10/30/60 s. 가로축은 "지금으로부터 몇 초 전" |

**녹화 파일**은 `logs/track/<월일>_track_live_<이름>.json` (`--log DIR` 을 줬으면 그
폴더). 형식이 `track_*.json` 과 같아서 위 `policy_viewer.html` 로 열림. 저장한 경로는
화면 머리와 터미널에 찍힘. 녹화 중에 실행이 끝나면 거기까지 저장함. 녹화는 60초
화면 고리와 따로 모으므로 길게 해도 잘리지 않음.

`huphy-test studio` 면 이 화면 위에 **제어 패널**이 붙음 (토크·게인·자세·동작 순서).
설명은 [scripts/README.md](../src/huphy/scripts/README.md) 의 `studio`.

`huphy-test watch`(토크 없이 손으로)는 목표가 없어 **실측만** 그림. 관절 칸 머리에
지금 값과 보이는 창의 범위가 나오고 오차 그래프는 꺼짐. 녹화 파일도 `tgt/` 없이
`pos/`·`imu/` 만 있고 `policy_viewer.html` 로 그대로 열림.

실행이 끝나면 "끊김" 으로 바뀌고 마지막 화면이 남음. 같은 포트로 새로 실행하면 알아서
처음부터 다시 그림. `--log` 와 같이 켜도 됨 -- 실시간으로 보고 파일로도 남김.

---

## `latency_viewer.html`

모터마다 **명령 묶음을 보내기 직전부터 그 응답이 도착할 때까지**를 잰 기록을
히스토그램으로 봄. 도착 시각은 수신 스레드가 찍음 (양다리 `--robot` 은 켜져 있음).

### 기록

```
# drain_ms=4 hz=50               뷰어가 기준선을 그리는 데 씀
t,joint,latency_ms,status        한 줄이 응답 하나
,right_leg/knee,,lost            유실은 개수만큼 빈 지연으로
```

| status | 뜻 | 많으면 |
|---|---|---|
| `ontime` | 그 주기 수거에서 받음 | — |
| `late` | 수거 대기를 넘겨 다음 주기에 들어옴. 지연은 원래 명령 기준 | `--drain-ms` 를 늘릴 자리 |
| `lost` | 끝내 안 옴 (보낸 수 − 받은 수) | 어댑터·배선 쪽 |

MIT 응답에는 순번이 없어서, 도착 시각이 이번 송신보다 앞이면 직전 명령의 응답으로 봄.
같은 표가 `huphy-run` 끝에 터미널로도 찍힘 (관절별 p50/p95/p99/최대, 늦음 %, 유실 %).

### 화면

- 맨 위에 전체, 그 아래 관절마다 한 칸. x 는 지연(ms), y 는 정규화 빈도
- 파랑 제때, 주황 늦음, **빨간 점선 = `drain_ms`**
- 맨 아래 표: 관절별 보냄·받음·p50·p95·p99·최대·늦음·유실

| 조작 | |
|---|---|
| 구간 폭 | 0.05 ~ 1 ms |
| x 끝 | 자동(p99.5) 또는 5/10/20/50 ms |
| 정규화 | 받은 것 기준 / 보낸 것 기준(유실까지 분모에) |
| 늦음 포함 | 끄면 제때 받은 것만 |
| 관절 y축 통일 | 관절 칸끼리 높이를 비교할 때 |

---

## `imu_viewer.py`

정책이 받는 IMU 값(관찰 0-5칸)을 보고, **gyro 와 중력이 서로 맞는지** 검사함. 읽기
전용이고 모터는 열지 않음. 설정은 `config/robot.yaml` 의 `imus` 를 읽음.

```bash
python3 tools/imu_viewer.py serve [--port 8765] [--host 127.0.0.1]
python3 tools/imu_viewer.py check [--seconds 45] [--url http://127.0.0.1:8765]
```

### `serve`

브라우저(`http://127.0.0.1:8765`)에 roll/pitch/yaw(도, 화면용)와 gyro x/y/z(rad/s)를
실시간으로 띄움. `stand.observation_vector` 를 그대로 거치므로 **모델이 받는 숫자와
같음.** `/data` 로 나가는 JSON 에는 중력(`ax ay az`, 관찰 3-5칸)도 있음.

다른 기기에서 보려면 `--host 0.0.0.0`.

### `check`

`--seconds` 동안 기록하면서 IMU 를 **한 축씩 천천히 90도쯤** 돌릴 것. 몸체 좌표에서 본
중력의 변화율 `g_dot = -w x g` 로 두 가지를 판정함.

| | 판정 |
|---|---|
| 크기 | 맞는 배율이 1 근처면 gyro 가 rad/s. 0.0175 근처면 라디안을 한 번 더 바꾼 것, 57 근처면 도/초 그대로 |
| 부호 | 축 부호를 뒤집어 가며 중력과 맞는 조합을 찾음. **(1,1,1) 이 아니면 gyro 와 중력이 서로 다른 좌표계** -- 정책이 실제로 없는 자세를 봄 |

어느 쪽이 "진짜" 인지가 아니라 둘이 **서로 맞는지**를 봄. 덜 돌린 축은 부호가 덜
확실하다고 따로 알려 줌.

**IMU 포트는 한 프로세스만 열 수 있음.** `serve` 가 떠 있으면 `check --url` 로 거기서
값을 받고, 정책 실행 중에는 둘 다 못 씀. 부착 방향(정지 상태에서 가속도계와 대조)은
`huphy-imu check` 가 따로 봄.

---

## `make_sequences.py` · `sequence_preview.html`

`huphy-test --robot studio` 에서 재생할 시퀀스를 **계산으로** 만듦. 손으로 자세를 찍는
대신 "발을 어디에, 얼마나 들고, 골반을 얼마나 옮길지" 로 정하고 관절각은 역기구학으로
구함.

```bash
python3 tools/make_sequences.py                                   # config/sequences/sim_*.json 을 씀
python3 tools/make_sequences.py --check                           # 쓰지 않고 표만 찍음
python3 tools/make_sequences.py --preview logs/seq_preview.html   # 미리보기도 같이 씀
python3 tools/make_sequences.py --lift 0.08 --step 0.12           # 걷기 발 높이 / 보폭 (m)
python3 tools/make_sequences.py --roll-sign -1                    # 발목 roll 보정 방향 뒤집기
```

| 시퀀스 | 단계 | 시간 |
|---|---|---|
| `sim_roll_check` | 준비 -> 골반 왼쪽 3도 -> 가운데 -> 오른쪽 3도 -> 가운데 | 19.5 s |
| `sim_squat` | 준비(무릎 20) -> 반(45) -> 깊게(70) -> 반 -> 준비, 3회. 발목은 깊을수록 발판 수평보다 덜 젖힘 (깊게에서 10도) | 14.1 s x 3 |
| `sim_walk` | 준비(무릎 30) -> 걸음마다 체중 이동 -> 발 들기 -> 내딛기 -> 내려놓기 -> 가운데로, 4걸음 -> 두 발 모음 | 39.2 s |

키프레임마다 `label` 에 단계 이름이 있음 (studio 는 읽을 때 무시함). 기존 `walk.json` 등
손으로 만든 시퀀스는 안 건드림.

### 모델

    허벅지 = 정강이 = 0.39 m       발목 기하 실측(무릎 -> 발목 393 mm). 허벅지는 같다고 가정
    발판 수평                       ankle_pitch = knee - hip  (발목 모터각은 AnkleKinematics.solve_ik)
    좌우 체중 이동                  hip_roll 로 골반을 디딤발 위로 (발 간격 0.20 m 가정 -> 7.6도),
                                   발목 roll 로 발판을 되돌림

정책 기준자세(엉덩이 10, 무릎 20)의 발목 모터각을 FK 로 풀면 pitch 8.8도 -- 이 모델의 10도와
거의 맞음.

**물리 시뮬이 아니라 기구학임.** 관절이 한계 안인지, 발이 어디 놓이는지, 발판이 수평인지만
계산함. 무게중심과 넘어짐은 안 봄. **처음에는 매달거나 잡은 채로 돌릴 것.**

### 검사

모든 키프레임에서 관절 한계(`command_margin_deg` 3도 안쪽), 발목 도달 범위(pitch ±40,
roll ±25), 모터각을 봄. 하나라도 넘으면 파일을 안 씀. 걷기에서 보폭을 키우면 **뒷발을 들 때
발목 pitch 가 먼저 한계에 닿음** (발 0.08 m 들 때 보폭 0.14 m 에서 39.9도, 0.16 m 는 거부).

**발목 roll 부호와 hip_roll 방향은 확인되지 않은 가정임** (docs/issues.md #13). 걷기 전에
`sim_roll_check` 를 돌려 골반이 실제로 그쪽으로 가는지, 발바닥이 바닥에 붙어 있는지 볼 것.
발 모서리가 들리면 `--roll-sign -1` 로 다시 만듦.

### 미리보기 (`sequence_preview.html`)

`--preview <경로>` 로 만든 파일을 브라우저로 열면 시퀀스를 막대 그림으로 재생함.

| 화면 | 무엇 |
|---|---|
| 옆에서 (앞이 오른쪽) | 골반을 고정하고 허벅지·정강이·발판. 파랑 왼다리, 주황 오른다리, 갈색 선이 바닥. 무릎 높이·발 앞뒤·발판 수평을 봄 |
| 뒤에서 (로봇 왼쪽이 왼쪽) | 골반 막대와 기운 다리. 체중 이동 때 골반이 디딤발 위로 가는지 봄 |
| 지금 관절 | 양다리 엉덩이·무릎·발목 pitch (굽힘 크기, 도), 골반 기울기 |
| 단계 | 키프레임 목록과 이동·유지 시간. 재생 중인 단계를 강조 |

키프레임 사이는 studio 처럼 이동 시간 동안 선형 보간하고 유지 시간만큼 멈춤 -- 타이밍이
실제 재생과 같음. 첫 키프레임은 그 자세에서 바로 시작함 (studio 는 지금 자세에서 옮겨 감).

안 보여주는 것: 중력·무게중심(골반이 공중에 고정된 그림), 발목 roll(발판은 늘 수평으로
그림), 모터 추종 오차. 원본 html 을 직접 열면 빈 화면임 -- `make_sequences.py` 가 계산
결과를 넣어 씀.

---

## `can_scan.py` · `rs_ota.py` -- 모터 설정 확인과 펌웨어

자세한 절차는 [docs/motor_setup.md](../docs/motor_setup.md) 14~16절. 둘 다 python-can 만 씀.

```bash
python3 tools/can_scan.py               # canl, canr. id 1~12 + 0x7F
python3 tools/can_scan.py canr --ids 7-12
```

출력 예 (11번이 설정 전이라고 할 때):

```
ch     id proto   host  mode  fault   temp    VBUS    iqf zero  note
canr   10 mit     0xFA  0     False   25.0   48.47   0.00    1
canr   11 mit     0xFD  0     False   24.0       -      -    -  호스트 id 가 0xFB 이어야 함; Command 14 무응답 -> 펌웨어 업데이트 대상
```

**제어가 돌고 있을 때 쓰지 말 것** -- 모터마다 정지 명령을 보내 상태를 받음.

```bash
python3 tools/rs_ota.py --channel canr --motor 8 --info                    # UID·버전 (private 일 때만)
python3 tools/rs_ota.py --channel canr --motor 8 --bin rs04_0.4.1.38.bin   # 약 10초
```

`rs_ota.py` 는 쓰기 전에 모터의 버전과 파일 이름의 모델(rs03 / rs04)을 맞춰 보고 다르면
멈춤. 모델이 다른 펌웨어도 파일 앞부분이 같아서 이 확인이 없으면 못 가림.

---

## `gamepad_viewer.py` -- 패드 입력 실시간

**이 파일 하나로 끝.** ROS·런치·노드·`joy` 패키지 안 씀. 리눅스 조이스틱 장치(`/dev/input/js*`)를
파이썬 표준 라이브러리로 직접 읽어 브라우저에 보여 줌. 읽기만 하고, 장치는 여러 프로그램이
같이 열 수 있어서 `gamepad.launch.py` 가 돌고 있어도 됨.

```bash
python3 tools/gamepad_viewer.py                  # http://127.0.0.1:8780
python3 tools/gamepad_viewer.py --host 0.0.0.0   # 같은 망의 다른 기계에서도
python3 tools/gamepad_viewer.py --device /dev/input/js1
```

| 칸 | 무엇 |
|---|---|
| 맨 위 | 패드 이름, 장치, 버튼·축 개수, 마지막 입력. 끊기면 "연결 안 됨" (Xbox 버튼으로 깨우면 다시 엶) |
| 스틱 | 왼쪽·오른쪽 위치 점과 값. 커널 값이라 위로 밀면 세로가 − |
| 버튼 | 번호와 **장치가 알려 준 입력 코드로 붙인 이름** -- `A`, `X`, `RB`, `View`, `Menu` 등. 누르면 초록, 방금 바뀐 줄은 밝아짐 |
| 축 | 번호·이름·막대·값 (−1 ~ +1) |

이름은 장치에 입력 코드를 물어 붙이므로 USB·블루투스를 따로 맞출 필요가 없음. 블루투스 Xbox 는
오른쪽 스틱이 `Z`/`RZ`, 트리거가 `GAS`(RT)/`BRAKE`(LT) 로 옴 (USB 는 `RX`/`RY` 와 `Z`/`RZ`).
버튼의 `C`·`Z` 는 옛 게임패드 형식의 빈 칸이라 안 눌림 -- 그래서 블루투스에서 X·Y·LB·RB·View 가 한 칸씩 밀림.
리눅스 별칭 `NORTH`/`WEST` 는 Xbox 에서 위치가 반대라 안 씀 -- 위쪽 버튼은 Y(4번), 왼쪽은 X(3번).

**번호는 커널 번호임** -- ROS `joy_node` 의 `/joy` 번호와 다를 수 있음. 블루투스에서는 0~14 가
같았고 joy_node 가 끝에 하나를 더 붙였음 (16개). gamepad_node 의 번호를 정할 때는 이름으로 맞출 것.
