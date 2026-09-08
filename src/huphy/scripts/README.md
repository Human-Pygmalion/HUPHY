# `scripts/` — 터미널 진입점

```
scripts/
├── commission.py   조립할 때 한 번 하는 조작
├── bringup.py      다리를 실제로 움직여 보는 대화형 메뉴
└── selftest.py     정해진 패턴으로 계속 움직여 봄
```

설정 파일에서 모터 목록을 읽으므로 모터 id 를 손으로 적지 않음.

`--config` 를 주지 않으면 현재 폴더부터 위로 올라가며 `config/robot.yaml` 을 찾음.
설정에 오류가 있으면 채널을 열기 전에 멈춤.

`--limb` 은 팔다리가 둘 이상이면 필수임. 팔다리마다 CAN 채널이 다름.

`huphy-test` 는 로봇 전체를 한 루프로 돌릴 수 있음.

```bash
huphy-test --robot zero                   # kind: leg 인 팔다리 전부
huphy-test --limb right_leg zero          # 다리 하나
huphy-test --limb left_leg,right_leg zero # 일부만 묶기 (드묾)
```

**플래그가 곧 만들어지는 객체임.**

| | 만들어지는 것 | 관절 이름 |
|---|---|---|
| `--limb <이름>` | `Leg` | `knee` |
| `--robot` | `Biped` | `right_leg/knee` |

`--limb` 은 "팔다리가 여럿이라 헷갈리니 하나를 짚어라" 는 뜻임. 로봇 전체는 그것과
다른 층의 선택이라 플래그를 따로 둠 — `--limb all` 처럼 단수 플래그에 복수 의미를
얹으면, 나중에 팔이 붙었을 때 "다리만? 팔까지?" 로 다시 애매해짐. 둘을 같이 주면
거부함.

프로세스를 둘 띄우는 것과 다름 — 명령이 같은 주기에 나가고, 한쪽 통신이 끊기면
**양쪽이 같이** 멈춤. 따로 띄우면 한쪽이 죽어도 다른 쪽은 계속 움직여 넘어짐.

나머지 진입점은 팔다리 하나만 받음. `commission` 은 사람이 한 다리를 손으로
만지는 절차이고, `bringup` 은 관절 하나씩 확인하는 화면이라 묶을 이유가 없음.

---

## `commission.py`

```bash
python -m huphy.scripts.commission --limb right_leg scan
huphy-commission --limb right_leg state
huphy-commission --limb right_leg nudge knee --delta 5
huphy-commission --limb right_leg zero knee --note "다리 편 상태" --yes
```

| | 무엇 |
|---|---|
| `scan` | 어느 모터가 응답하는지 |
| `state` | raw 와 cal 을 나란히. 속도·토크·온도 |
| `fault` | 고장 상태 조회 |
| `clear-fault` | 고장 상태 지우기 |
| `sweep` | 토크를 끄고 손으로 밀어 가동 범위 측정 |
| `nudge` | 조금 움직였다 되돌림 |
| `zero` | **[영구]** 지금 자세를 기계 영점으로 |
| `mode` | 제어 모드 변경 |
| `can-id` | **[영구]** CAN id 변경 |
| `protocol` | **[영구]** 프로토콜 전환 |

`mode` `can-id` `protocol` 은 모터를 새로 사거나 갈아 끼울 때만 씀. 한 관절씩만 다룸.

### 관절과 옵션을 생략하면 물어봄

관절을 받는 명령(`clear-fault` `sweep` `nudge` `zero` `mode` `can-id` `protocol`)은
이름을 빼고 실행할 수 있음. 터미널이면 목록을 띄우고 번호·이름·`a`(전부) 로 받음.

```
  right_leg -- 무엇을 영점 잡을까요?

    1) hip_pitch  id=7   RS02
    ...
    a) 전부

  선택 [a]:
```

`clear-fault` `sweep` `zero` 는 전부가 기본임. `nudge` `mode` `can-id` `protocol` 은
전부를 고를 수 없음.

관절을 고르고 나면 옵션을 한 번에 보여주고 한 줄로 받음. 쉼표로 구분하고 빈 칸은
기본값임. 옵션이 하나뿐인 명령은 줄 전체를 값으로 씀.

```
  옵션 -- 쉼표로 구분, 비우면 기본값

    1) delta   5.0      몇 도 움직였다 되돌릴지. 20도까지
    2) kp      5.0      위치 게인. 안 움직이면 조금씩 올릴 것
    3) kd      0.5      속도 게인

  입력 [5.0, 5.0, 0.5]: 3

  실행: huphy-commission --limb right_leg nudge knee --delta 3.0 --kp 5.0 --kd 0.5
```

끝에 나오는 명령줄은 그대로 다시 쳐도 같이 동작함. `(필수)` 로 나오는 것은
`zero --note`, `can-id --to`, `protocol --to` 임.

옵션을 물어보는 것은 관절을 생략했을 때뿐임. 관절을 명령줄에 적었으면 묻지 않음.

터미널이 아니면 물어볼 수 없음. 전부가 기본인 명령은 전부로 가고, 하나만 받는 명령은
거부함.

### `--yes` 를 요구하는 것

```
zero        기계 영점. 모터에 저장되고 좌표계가 통째로 옮겨감
can-id      CAN id 변경. 바꾼 뒤 robot.yaml 도 고쳐야 함
protocol    프로토콜 전환. 전원 재투입 필요
```

승인 확인이 버스를 열기 전에 일어남. `--yes` 는 이 세 명령에만 붙음.

---

## 명령별 동작

### `scan`

```
right_leg  can1  모터 6개

  hip_pitch  id=7   RS02   응답
  ankle_b    id=12  RS00   ----

응답 없음: ['ankle_b']
```

### `state`

```
  관절               raw       cal        속도       토크     온도
  knee       29.99     29.99     -0.46     0.39   32.5

  미실측 상태라 cal 이 raw 와 같음.
```

### `sweep`

```bash
huphy-commission --limb right_leg sweep          # 전 관절
huphy-commission --limb right_leg sweep knee     # 하나만
huphy-commission --limb right_leg sweep 10       # id 로도 됨
```

토크를 끄고 관절마다 오프셋과 한계각을 정함. 관절(묶음) 하나가 한 단계임. 측정은
초당 20번, `--hz` 로 바꿀 수 있음.

```
right_leg 가동 범위 측정 -- 5단계, 초당 20번 잽니다.

  [4/5] knee
       0도 자세로 두고 Enter:
       knee       offset   -33.40

       양쪽 끝까지 미세요. 끝나면 Enter.

       관절                최소        지금        최대        범위
       knee       -20.61     30.12     74.76     95.37
```

0도가 먼저임. `measure_offset` 이 `offset = -raw` 를 내고, 그 뒤 `cal = raw + offset`
으로 최대·최소를 굴림. `ankle_a` 과 `ankle_b` 는 한 단계로 묶임(`LINKED`).

단계가 끝날 때마다 캘리브레이션 파일에 적음. 오프셋과 한계각이 같이 들어가고,
`sign` 과 `zero_reference` 는 건드리지 않음.

```json
"knee": {"sign": 1.0, "offset_deg": -33.40,
         "zero_reference": "다리 편 상태", "limits_deg": [-20.61, 74.76]}
```

움직인 폭이 0인 관절은 빼고 나머지를 적음. 뺀 관절과 그것만 다시 재는 명령을 화면에
냄. 캘리브레이션 파일이 설정되어 있지 않으면 재기 전에 멈춤.

단계마다 Enter 를 받으므로 화면에서만 실행됨. 범위가 5도도 안 되는 관절이 있으면
종료 코드 `1`.

### `nudge`

```
right_leg.knee (id=10) 를 +5.0도 움직였다 되돌림.
  게인 kp=5.0 kd=0.5
  다리를 받쳐 두고, 실제로 어느 관절이 움직이는지 볼 것.

  시작    29.99
  최대    34.72   (움직인 양 +4.73)
  끝      30.23
```

명령한 양의 30% 미만이면 알림. 응답 없는 모터에는 토크를 넣지 않음.

### `zero`

```bash
... zero --yes                                            # 전부, 관절마다 Enter
... zero knee --note "다리 편 상태, 발바닥 평면 접촉" --yes   # 하나만
```

```json
"knee": {"sign": 1.0, "offset_deg": 0.0, "zero_reference": "다리 편 상태, 발바닥 평면 접촉"}
```

토크를 맨 앞에서 한 번에 끊고 영점을 잡음. 관절을 생략하면 한 번 실행에 전부를 잡되
관절마다 Enter 를 받음.

```
  자세를 잡은 채로 관절마다 Enter.

  hip_pitch  Enter:
  hip_pitch  잡음
  hip_roll   실패 -- 응답 없음
```

프레임이 실제로 나간 관절만 메모가 저장됨. 하나라도 실패하면 종료 코드 `1`.

### 채널이 안 열리면

```
can1 (socketcan) 를 열 수 없음: ...
채널이 올라와 있는지 확인할 것:
  sudo ip link set can1 up type can bitrate 1000000
  ip -details link show can1
```

### 종료 코드

```
0   정상
1   응답 없는 모터가 있거나 고장이 있음
```

`scan`, `state`, `fault` 는 문제가 있으면 `1` 을 냄.

---

## 실물에서의 순서

```
1  CAN 채널 올리기          어댑터 종류에 따라 다름. 루트 README 4번
2  commission scan          6개 다 응답하나
3  commission fault         고장 없나
4  commission nudge <관절>   설정대로 움직이나. 관절마다, 눈으로
5  commission zero <관절>    영점. 자세를 잡아 놓고 관절 전부
6  commission sweep         가동 범위. 영점을 전부 끝낸 뒤 한 번에
7  게인 튜닝               kp, kd 를 robot.yaml 에 적음
8  bringup                  게인 튜닝. 그래프를 보며
```

`sweep` 이 재는 값은 영점 기준의 각도임. 영점을 다시 잡으면 범위도 다시 재야 함.

---

## `bringup.py`

```bash
python -m huphy.scripts.bringup --limb right_leg
huphy-bringup --limb right_leg
huphy-bringup --limb right_leg --gain-scale 0.1 --allow-uncalibrated
```

설정·실측값·버스·기구학·안전·텔레메트리·제어 루프를 한데 묶어 씀.

| 옵션 | 기본값 | 무엇 |
|---|---|---|
| `--limb` | 없으면 오류 | 어느 팔다리. 팔다리마다 CAN 채널이 다름 |
| `--gain-scale` | `1.0` | `robot.yaml` 의 `kp`/`kd` 를 이 비율로 낮춰 씀. 설정 파일은 안 바뀜 |
| `--allow-uncalibrated` | 꺼짐 | 미실측이어도 토크를 넣음. 경고만 내고 진행함 |
| `--config` | 위로 올라가며 찾음 | 다른 `robot.yaml` 을 쓸 때 |
| `--hz` | 설정의 `control_hz` | 제어 주기 |
| `--no-precise` | 켜짐 | 마감 직전 스핀을 끔 |
| `-v` | | DEBUG 로그 |

`--gain-scale 0.1` 이면 `kp: 20.0, kd: 1.0` 이 `kp: 2.0, kd: 0.1` 로 감. 실물에서
튜닝하기 전의 시작값이라 처음 만질 때는 낮춰야 함. 낮추면 시작할 때 화면에 냄.

`--allow-uncalibrated` 가 없으면 `Leg.enable` 이 `is_calibrated` 를 보고 막음 —
`[토크]` 항목이 전부 거부됨. 상태 보기와 카운터 보기는 그대로 됨.

```
1. 상태 보기
2. 카운터 보기
3. 자세 유지 [토크]
4. 한 관절 옮기기 [토크]
5. 계단 응답 [토크]
6. 사인파 왕복 [토크]
```

`[토크]` 항목은 미실측이면 거부함. 아래가 다 차야 `is_calibrated` 가 참이 됨.

| 파일 | 채울 값 | 무엇으로 |
|---|---|---|
| `config/calibration/<팔다리>.json` | `limits_deg` | `commission sweep` |
| `config/calibration/<팔다리>.json` | `zero_reference` | `commission zero --note` |
| `config/robot.yaml` | `kp` (0보다 커야 함) | 사람이 적음 |

하나라도 비면 메뉴 머리에 `미실측 (allow_uncalibrated 필요)` 이 뜨고 `[토크]` 항목이
거부됨. 어느 관절이 빈지는 상태 보기 끝에 나옴.

`--allow-uncalibrated` 를 붙이면 경고만 내고 진행함. 실측을 하려면 움직여야 하므로
커미셔닝 단계에서 필요함.

움직이는 것은 전부 제어 루프를 탐. 메뉴는 동작만 정하고 루프에 넘김.

```
menu -> motion -> ControlLoop -> Leg -> bus
```

### 상태 보기

```
  관절             raw       cal      속도     토크   온도     응답  마지막 응답
  hip_pitch  -60.01    -60.01     -0.46     0.32   30.0     받음       0.02ms
  knee       -60.01    -60.01     -0.46     0.32   30.0     씹힘      254.9초
  ankle_b    -60.01    -60.01     -0.46     0.32   30.0 명령안함         없음

  발목  pitch   -0.00   roll    0.01
```

| 칸 | 무엇 |
|---|---|
| `raw` | 모터가 보고한 각도 그대로 |
| `cal` | 관절 좌표계 각도. `sign`·`offset` 을 적용한 값 |
| `속도` | 도/초 |
| `토크` | 모터가 보고한 토크 |
| `온도` | ℃ |
| `응답` | 직전 주기에 명령하고 `받음` / 명령했는데 `씹힘` / `명령안함` |
| `마지막 응답` | 그 이후 경과 시간. 1초가 넘으면 초로 씀. 한 번도 못 받았으면 `없음` |

100Hz 면 `마지막 응답` 이 한 자리 ms 임. `254.9초` 처럼 크면 한 번은 받았다가 끊긴
것이고, `없음` 이면 처음부터 응답이 없던 것임.

응답이 끊기면 가드가 현재 위치를 몰라 명령을 거부하므로 `응답` 이 `명령안함` 으로
조용해짐. 진짜 신호는 `마지막 응답` 임.

발목은 모터 각도가 아니라 기구학으로 푼 pitch/roll 로 보여줌.

IMU 가 붙어 있으면 이어서 나옴. 없으면 이 표가 아예 안 나옴.

```
  IMU             roll     pitch       yaw    마지막 값
  main            1.50     -2.50     30.00      4.21ms
```

`마지막 값` 이 진짜 신호임. 센서가 멈춰도 자세는 그럴듯한 숫자로 남아 있음.

한계 밖에 있는 관절이 있으면 경고함.

```
  ** 한계 밖에 있는 관절 **
  토크를 넣으면 가드가 한계 안으로 끌어당김 -- 그 방향으로 움직임.
    knee       200.00  한계 -20.65 ~ 74.79
```

### 카운터 보기

시작부터 지금까지 쌓인 사건 수임. 상태 보기가 "지금 어떤가"면 이쪽은 "그동안 몇 번".

```
  가드
    clips                12
    rejects               0
    clips_limit          12
    clips_jump            0
    rejects_nan           0
    rejects_nostate       0

  CAN
    can.tx_errors         0
    can.rx_errors         0
    can.frames_sent    1204
    can.frames_received 1198
    can.drain_timeouts    2

  마지막 클리핑 이후 3.4초   마지막 거부 이후 -1.0초

  루프  340주기 3.4초 (평균 100.1Hz / 목표 100Hz), 밀림 2회, 최악 12.3ms, 무응답 주기 0회
```

| 칸 | 무엇 |
|---|---|
| `clips` | 잘렸지만 전송은 된 횟수 |
| `rejects` | 아예 안 나간 횟수 |
| `clips_limit` | 목표가 관절 한계 밖이라 한계값으로 잘림 |
| `clips_jump` | 한 주기에 너무 많이 움직이려 해서 잘림 |
| `rejects_nan` | 목표가 NaN/Inf. 자를 수 있는 값이 아님 |
| `rejects_nostate` | 현재 위치를 몰라 점프 폭을 못 잼 — 그 모터가 무응답임 |
| `can.frames_sent` / `_received` | 보낸 프레임과 받은 프레임. 차이가 벌어지면 응답을 못 받는 것 |
| `can.drain_timeouts` | 기대한 개수를 못 채우고 시간을 다 쓴 횟수 |
| `마지막 ... 이후` | 초. 없었으면 `-1.0` |

누적 수만 보면 12번이 방금 몰린 것인지 10분 전 것인지 모름. 그래서 마지막 사건 이후
경과를 같이 냄.

루프 줄은 한 번이라도 돌았을 때만 나옴.

| 칸 | 무엇 |
|---|---|
| `340주기 3.4초` | 돈 주기 수와 걸린 시간 |
| `평균 100.1Hz / 목표 100Hz` | 실제 주파수와 `--hz` |
| `밀림` | 마감을 넘긴 주기 수 |
| `최악` | 가장 오래 걸린 주기 |
| `무응답 주기` | 응답이 하나도 안 온 주기 수 |

못 지키면 끝에 `** 주기를 못 지킴 **` 이 붙음.

### 자세 유지 [토크]

지금 자세를 그대로 붙잡음. 게인 튜닝의 출발점임 — 여기서 떨리면 어떤 동작을 시켜도
떨림.

```
  얼마나 (초) [3.0]:

  지금 자세를 붙잡음. 처지나, 떨리나 볼 것

  300주기 3.0초 (평균 100.0Hz / 목표 100Hz), 밀림 0회, 최악 10.4ms, 무응답 주기 0회
```

처지면 `kp` 부족, 떨리면 `kp` 과함.

관절을 고르지 않고 여섯 개를 전부 붙잡음. 목표는 시작할 때 한 번 정하고 그대로 둠.
발목은 FK 로 풀어 `ankle_pitch`/`ankle_roll` 목표를 만듦. 목표를 못 정한 관절은
이름을 내고 빠짐.

### 한 관절 옮기기 [토크]

한 관절만 지금 자리에서 조금 옮김. 나머지는 지금 자리에 붙잡아 둠.

```
  관절: hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll
  이름: knee
  얼마나 (도) [5.0]:

  knee       29.99 -> 34.99 도

  250주기 2.5초 (평균 100.0Hz / 목표 100Hz), 밀림 0회, 최악 10.4ms, 무응답 주기 0회
```

`29.99 -> 34.99` 는 지금 각도와 목표 각도임. 1초에 걸쳐 옮김. 발목은 모터가 아니라
`ankle_pitch` / `ankle_roll` 로 받음.

### 계단 응답 [토크]

목표를 한 번에 뛰게 하고 따라오는 모양을 봄. 게인 튜닝에서 가장 많은 것을 알려줌.

```
  이름: knee
  계단 크기 (도) [10.0]:

  knee       29.99 -> 39.99 도
  그래프에서 볼 것:
    못 미침    kp 부족      지나쳤다 돌아옴  kd 부족
    떨림       kp 과함      느리게 도달      kp 올릴 여지

  350주기 3.5초 (평균 100.0Hz / 목표 100Hz), 밀림 0회, 최악 10.4ms, 무응답 주기 0회
```

0.5초 붙잡고 있다가 뛰므로 그래프에서 뛰기 전후가 갈림. 모양은 텔레메트리로 봄.

### 사인파 왕복 [토크]

지금 자리를 중심으로 왕복시킴. 추종 지연과 진폭 감쇠를 봄.

```
  이름: knee
  진폭 (도) [5.0]:
  주파수 (Hz) [0.5]:
  길이 (초) [3.0]:

  knee       29.99 도를 중심으로 ±5.0 도, 0.5 Hz

  350주기 3.5초 (평균 100.0Hz / 목표 100Hz), 밀림 0회, 최악 10.4ms, 무응답 주기 0회
```

| 물음 | 무엇 |
|---|---|
| `진폭` | 중심에서 한쪽으로 몇 도 |
| `주파수` | 초당 몇 번 오가는지 |
| `길이` | 몇 초 동안 |

주파수를 올릴수록 뒤처짐과 진폭 감쇠가 커짐. 어디서 못 따라오는지가 게인의 한계임.

### 안전

```
시작할 때    관찰 모드. 토크가 꺼져 있음
움직이기 전  freeze 로 지금 자세를 잡음
끝날 때      hold 후 토크 차단
Ctrl-C       루프의 finally 를 지나므로 같은 순서
```

모든 동작이 `freeze` 로 시작하고, 흔들지 않는 관절은 지금 자리에 붙잡아 둠.

---

## `selftest.py`

```bash
huphy-test --limb right_leg zero
huphy-test --limb right_leg range --period 10 --margin 8
```

정해진 패턴으로 Ctrl-Q 를 누를 때까지 계속 움직임.

| | 무엇 | 무엇을 보나 |
|---|---|---|
| `zero` | 관절 전부를 0도로 두고 붙잡음 | 자세가 유지되는가. 처지면 `kp` 부족, 떨면 과함 |
| `range` | 관절마다 최소~최대를 오감 | 끝까지 가는가. 걸리는 데는 없는가 |

지금 자세에서 목표까지 `--approach` 초(기본 3)에 걸쳐 옮긴 뒤 패턴을 시작함.
왕복은 사인파이고 `--period`(기본 6초)에 한 번 오감.

관절 한계의 출처가 둘임.

```
hip_pitch hip_roll hip_yaw knee     캘리브레이션 파일의 limits_deg
ankle_pitch ankle_roll  AnkleEnvelope 의 시험 범위
```

한계가 없는 관절은 빠지고, 어느 관절을 뺐는지 화면에 냄. `--margin`(기본 5도)만큼
한계 안쪽까지만 감.

Ctrl-Q 는 `QuitWatcher` 가 별도 스레드에서 봄. 터미널을 cbreak 로 바꾸고 빠져나올 때
되돌림. 화면이 아니면 켜지 않고 Ctrl-C 로 끊음.

---

## 셋의 차이

| | `commission.py` | `bringup.py` | `selftest.py` |
|---|---|---|---|
| 무엇 | 조립할 때 한 번 | 하나씩 골라 움직여 봄 | 정해진 패턴을 계속 |
| 주기 | 없음 | 제어 루프 | 제어 루프 |
| 되돌리기 | 어려움 | 쉬움 | 쉬움 |
| 사람이 하는 일 | 관절을 손으로 잡음 | 메뉴를 고름 | 보고만 있음 |

---

## 테스트

```bash
PYTHONPATH=src python3 -m pytest tests/test_commission_cli.py tests/test_bringup.py tests/test_selftest.py -q
```

`commission` 71개, `bringup` 30개, `selftest` 27개. 가짜 모터가 붙은 가짜 CAN 버스에
대고 명령줄과 메뉴를 그대로 돌림. 설정 파일은 임시 폴더에 만들어 씀.

---

## 조립 함수 — `bringup.build_leg` / `bringup.build_biped`

설정을 실행 객체로 바꾸는 곳. 진입점 넷이 전부 이 둘 중 하나를 씀.

```python
build_leg(robot, limb)      # 다리 하나.       ControlLoop 에 그대로 들어감
build_biped(robot)          # 로봇 전체.       kind: leg 인 팔다리를 전부
```

둘 다 `Robot` 계약을 채우므로 제어 루프 입장에서는 차이가 없음.

### `build_biped` 가 다르게 하는 것

| | 왜 |
|---|---|
| 수신 스레드를 켬 | 버스가 둘이면 순차 수거의 총 대기가 두 배가 됨 |
| IMU 를 다리에 안 붙임 | 로봇이 들고 있음. 몸통 센서는 어느 다리의 것도 아님 |
| 관절 이름에 팔다리가 붙음 | `right_leg/knee`. 무릎이 둘이라 구분이 필요함 |

나머지 인자(`gain_scale` 등)는 `build_leg` 로 그대로 넘어가 **양다리에 같이**
걸림.

### 순서가 곧 열 순서임

`limbs` 를 생략하면 `robot.yaml` 에 적힌 순서를 씀. 그 순서가 관절 이름 순서와
텔레메트리 열 순서를 정하므로, 설정에서 다리 순서를 바꾸면 로그 열 순서도 바뀜.
