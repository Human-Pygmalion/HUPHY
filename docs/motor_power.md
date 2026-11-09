# 모터 전류·전압·전력

모터에서 무엇이 읽히고, 안 읽히는 전력을 읽히는 값으로 어떻게 계산하는지.
지금 코드가 어디까지 하는지. 기록 열 이름은
[`telemetry/track_log.py`](../src/huphy/telemetry/track_log.py).

---

## 1. 모터에서 읽히는 값

RobStride 모터는 두 경로로 값을 냄.

```
상태 응답 프레임 (매 주기)      위치 / 속도 / 토크 / 온도
파라미터 읽기 (Command 14)      인덱스 하나당 float 하나
```

파라미터 읽기로 가져올 수 있는 것은 **네 개뿐임.** RS04 매뉴얼 `0x70xx` 표
(PDF 52~53쪽 = 인쇄 50~51쪽, 4.1.14절) 에서 권한이 `R` 인 항목:

```
0x7019  mechPos   부하측 기계 각도 (rad)
0x701A  iqf       q축 상전류 필터값 (A)
0x701B  mechVel   부하측 기계 속도 (rad/s)
0x701C  VBUS      버스 전압 (V)
```

`W` 는 쓰기만, `W/R` 은 읽기·쓰기 둘 다 되는 항목임. `R` 네 개가 위 목록임.

**DC 전류와 전력은 표에 없음.** `0x70xx` 표에도, 상태 프레임이 실어 오는
`0x30xx` 표 (PDF 27~29쪽 = 인쇄 25~27쪽, 3.3.4절) 에도 없음. 모터가 측정해 주지
않으므로 계산으로 구해야 함.

### `iqf × VBUS` 는 전력이 아님

둘은 인버터의 **반대쪽**에 있음.

```
배터리 ──[VBUS]── 인버터(MOSFET 6개) ──[상전류 iq]── 코일
```

`VBUS` 는 인버터 입력 전압, `iq` 는 인버터 출력 전류임. 곱해도 어느 지점의
전력도 아님. 2026-10-28 보행 기록에서 무릎 하나의 `iq` 최대가 15.83 A 이고
`vbus` 가 48 V 라 곱하면 760 W 가 되는데, 같은 기록의 온도가 31도로 일정해서
그만한 전력이 들어갔을 수 없음.

---

## 2. 상저항

### 의미

모터 코일 한 상의 직류 저항임 (단위 Ω). 전류가 코일을 지날 때 열로 버리는
전력(구리 손실)을 이 값으로 계산함.

사양서는 **선저항**(line resistance, 상-상)으로 적음. Y 결선이면 단자 두 개
사이에 코일이 두 개 직렬이라:

```
선저항 = 2 x 상저항
상저항 = 선저항 / 2
```

### 측정

상 단자 두 개(U, V)에 멀티미터 프로브를 대고 저항 모드로 읽음. 읽은 값이
선저항임.

```
RS03 실측 0.4 ohm  ->  상저항 0.20     사양 선저항 0.39 ->  0.195    일치
RS04 실측 0.2 ohm  ->  상저항 0.10     사양 선저항 0.16 ->  0.08     분해능 안
```

멀티미터 분해능이 0.1 Ω 이라 RS04 는 반올림 폭 안에서만 일치함. 계산에는
사양값(±10%)을 씀.

리드선 저항은 프로브 둘을 맞대 읽은 값을 빼서 지움. 리드선은 보통 0.1~0.2 Ω 를
더함 ([Knova KN-8056 매뉴얼, Multimeter Basic
Testing](https://www.manualslib.com/manual/2358711/Knova-Kn-8056.html?page=6)).

업계 관행은 마이크로옴미터로 4선(켈빈) 결선해 상-상으로 재고, 절대값보다 세 상의
**편차**(5% 이내)를 보는 것임 ([fabrico.io, Motor Circuit
Analysis](https://www.fabrico.io/blog/motor-circuit-analysis/)).

### MOSFET 이 무시되는 이유

이 모터는 드라이버가 안에 붙어 있어서 상 단자가 인버터에 연결된 상태로 측정함.
3상 인버터는 MOSFET 6개임.

```
    VBAT+ ──┬───────┬───────┬──
            │       │       │
         하이사이드  하이     하이
            │       │       │
    상단자 ─U───────V───────W──    코일이 여기 붙음
            │       │       │
         로우사이드  로우     로우
            │       │       │
    GND  ───┴───────┴───────┴──     VBAT+ 와 GND 사이에 버스 커패시터
```

MOSFET 은 꺼져 있어도 구조상 다이오드가 하나 남음 (**바디 다이오드**). 기판과
도핑 영역 사이의 PN 접합이고, 소스를 바디에 금속으로 단락시켜 드레인-소스에
역병렬 다이오드가 됨 ([DigiKey, The Significance of the Intrinsic Body Diodes
Inside MOSFETs](https://www.digikey.fi/fi/articles/the-significance-of-the-intrinsic-body-diodes-inside-mosfets)):

> There are P-N junctions in the FET structure between the substrate and the
> doped regions. ... a typical N-channel FET cannot block current flow from its
> source to its drain.

방향은 이렇게 고정됨.

```
하이사이드 바디 다이오드    상단자 -> VBAT+    한 방향만
로우사이드 바디 다이오드    GND    -> 상단자    한 방향만
```

U-V 에 프로브를 대면 전류가 갈 길이 둘임. 읽히는 값은 둘의 병렬임.

```
경로 1 (코일)     U -> 코일 -> 중성점 -> 코일 -> V
경로 2 (MOSFET)   U -> 다이오드 -> VBAT+ -> 커패시터 -> GND -> 다이오드 -> V
```

경로 2 가 무한대인 이유가 둘임.

**(1) 문턱 전압.** 실리콘 접합은 순방향 0.5~0.8 V 를 넘어야 전류가 흐름
([Fluke, How to Test Diodes with a Digital
Multimeter](https://www.fluke.com/en-th/learn/blog/digital-multimeters/how-to-test-diodes),
[learnabout-electronics.org](https://learnabout-electronics.org/Semiconductors/diodes_28.php)).
경로 2 는 다이오드 두 개를 직렬로 지나므로 약 1.4 V 가 필요함. 멀티미터가 저항
모드에서 거는 개방 전압은 보통 0.2~1.2 V 라 안 켜짐.

**(2) 커패시터.** 커패시터는 직류 전류를 흘리지 않음. 멀티미터는 직류를 걸어
재므로 충전이 끝나면 경로 2 의 전류가 0 이 됨.

그 밖의 경로(`U -> 하이사이드 -> VBAT+ -> 하이사이드 -> V` 등)는 두 번째
다이오드를 역방향으로 지나야 해서 막힘.

그래서 **상-상 측정은 코일 두 개만 읽음.** 세 상이 다 같게 나오고, 프로브를
바꿔 껴도 같게 나오는 것이 이 구조와 일치함 — 다이오드로 흐르면 방향에 따라
값이 달라짐.

**영향을 받는 측정은 이것임.**

```
상단자 - GND      로우사이드 다이오드 하나만 지남 (0.7 V) -> 켜짐
상단자 - VBAT+    하이사이드 다이오드 하나만 지남        -> 켜짐
```

이 조합은 방향에 따라 값이 다르고, 커패시터가 충전되는 동안 값이 계속 변함.
코일 저항이 아님.

출처에 대해: 위 세 출처는 바디 다이오드의 존재·방향과 접합 문턱 전압을 말함.
**인버터 브리지를 통과하는 저항 측정 자체를 다룬 출처는 찾지 못했음** — 경로
두 개와 직렬 다이오드 두 개는 위 회로에서 직접 따진 것임.

---

## 3. 전력 계산식

### 모터 상수

사양서값 (RS03 ±10%, RS04 ±10%):

```
             Kt (Nm/Arms)   선저항 (ohm)   상저항 (ohm)   인덕턴스 (mH)
RS03             2.36          0.39          0.195          0.275
RS04             2.10          0.16          0.080          0.211
```

`robot.yaml` 의 관절별 `model` 로 가름 ([config/robot.yaml:82-103](../config/robot.yaml#L82-L103)):

```
RS04    hip_pitch, hip_roll, knee
RS03    hip_yaw, ankle_a, ankle_b
```

감속비는 9:1.

### 구리 손실

FOC 는 `id ≈ 0` 으로 제어하므로 상전류 크기가 `iq` 임. 3상 합계는:

```
P_cu = 1.5 x iq^2 x R_phase        (iq 는 진폭, 단위 A)
```

`1.5` 는 3상(x3)과 진폭-실효값 변환(x1/2)에서 옴. `iqf` 는 진폭으로 옴.

### 기계 출력

```
P_mech = tau x omega               (tau: Nm, omega: rad/s, 부하측)
```

### DC 전류

```
P_dc = P_cu + P_mech + P_fe + P_inv + P_gear
I_dc = P_dc / VBUS
```

`P_fe`(철손), `P_inv`(인버터 손실), `P_gear`(감속기 손실) 은 상수가 없어 넣을 수
없음. 그래서 **`P_cu + P_mech` 만으로 낸 값은 하한임.**

### 2026-10-28 보행 기록 (65초, 12관절)

```
구리 손실     51.2 W     (RS04 6개 27.9 + RS03 6개 23.3)
기계 출력      2.0 W
합            53.3 W
VBUS 중위값   48.39 V
DC 전류 하한   1.10 A

사양 +-10% 를 넣으면    48.1 ~ 58.3 W  ->  1.00 ~ 1.21 A
```

### 미확정

```
결선        Y 인지 델타인지 미확인. 델타면 상저항 = 선저항 x 1.5 라 구리 손실이 3배
무부하 전류  RS03 2 Arms, RS04 0.7 Arms 로 출처마다 다름. 철손을 못 넣는 이유
감속기 효율  9:1 의 효율값이 사양서에 없음
드라이버 대기 전류  사양 40 mA 이하. 12대면 23 W 까지 될 수 있음
```

---

## 4. 지금 코드에서 어떻게 나오나

ROS `log_node` 가 구리 손실을 계산해 기록 열로 냄. 기계 출력과 나머지 손실은
안 들어감 -- 그래서 기록의 `dc_a_lo` (모터별·합) 는 하한임. 3절의 53.3 W 는 기계 출력까지
넣어 손으로 낸 값이고, 그것도 하한임.

### 값이 오는 경로

```
Leg._next_reads        한 주기에 extra 개씩 읽기 요청을 고름 (슬롯 회전)
  leg.py:455
Bus.send_mit           명령 프레임과 함께 CAN 으로 보냄 (reads=...)
Bus.params             모터 id -> 인덱스 -> (값, 도착 시각) 으로 보관
  bus.py:163
Leg.get_observation    관찰에 <모터>.iq / .vbus 로 넣음. 못 받았으면 NaN
  leg.py:575-579
```

읽을 인덱스는 [`leg.py:156`](../src/huphy/robots/leg.py#L156) 의 `EXTRA_PARAMS`
두 개임 (`0x701A`, `0x701C`). 온도는 상태 프레임에 있어 요청이 필요 없음.

`extra` 는 **한 주기에 다리마다 보낼 읽기 요청 수**임. 슬롯이 모터 6개 x 값
2개 = 12칸이라, 100Hz 에 `extra=2` 면 값 하나의 갱신 주기는 100 / 6 ≈ 17Hz 임.
`extra=0` 이면 요청을 안 보내고 관찰에 `iq`/`vbus` 가 아예 없음.

`extra` 는 burst 송신에서만 됨 — 읽기 응답 프레임에 모터 id 가 없어서 CAN id 로
누구 것인지 가름.

### 기록 열

```
extra/<모터>/iq     A
extra/<모터>/temp   C
extra/<모터>/vbus   V
```

순서는 [`track_log.py:111`](../src/huphy/telemetry/track_log.py#L111) 의
`EXTRA_VALUE_ORDER` 로 고정됨. 값을 넣는 쪽
([`extra_values`](../src/huphy/telemetry/track_log.py#L127)) 과 열을 미리 만드는
쪽 ([`extra_columns`](../src/huphy/telemetry/track_log.py#L117)) 이 같은 순서를
써야 열이 안 밀림.

### 두 실행 경로

`huphy-run --extra 2` 는 `extra_values` 가 관찰에서 바로 열을 만듦.

ROS 는 노드 두 개를 거침:

```
control_node   관찰 -> MotorExtra 세 배열 (iq, vbus, temp)
               convert.observation_to_extra   convert.py:44
               /huphy/debug/motor_extra 에 state_hz(기본 100Hz) 로 발행
               _publish_extra                 control_node.py:513
               구독자가 없으면 발행하지 않음

log_node       MotorExtra 를 구독해 마지막 것만 보관
               정책 줄이 올 때 그 줄에 넣음
               convert.extra_to_columns       convert.py:70
               _extra_row                     log_node.py:185
```

기록 주기는 정책 주기임 (정책 줄 한 개당 한 줄). 전기값은 그보다 느리게
갱신되므로 **연속한 줄에 같은 `iq` 값이 반복됨.**

열은 `ControlInfo.extra` 가 0 이 아닐 때 `PolicyLog(extra_motors=...)` 로 미리
만듦 — 첫 정책 줄이 첫 `MotorExtra` 보다 먼저 올 수 있어서임.

### 전력 열

`extra` 가 0 이 아닐 때만 생김. 따로 켜는 인자는 없음.

```
power/<모터>/copper_w   그 모터의 구리 손실 (W)
power/<모터>/dc_a_lo    그 모터의 DC 전류 하한 (A) = 그 모터의 구리 손실 / 그 모터의 vbus
power/copper_w          모터 전부의 합 (W)
power/dc_a_lo           DC 전류 하한 합 (A) = 합 / 그 줄 vbus 평균
```

계산은 [`telemetry/power.py`](../src/huphy/telemetry/power.py) 에 있고 `rclpy` 를
import 하지 않음. 상저항은
[`tables.phase_resistance_ohm`](../src/huphy/motors/robstride/tables.py) 이
모델명에서 냄 (선저항 / 2).

모델명이 `log_node` 까지 오는 경로:

```
robot.yaml 의 model           RS04 / RS03
control_node                  convert.motor_models(robot) 로 관절별 모델명
  ControlInfo.motor_models    1회, transient_local
log_node                      tables.phase_resistance_ohm 으로 상저항
  self.r_phase
  power.power_values          줄마다 계산
```

모터가 다른 기계에 있어서 `log_node` 가 `robot.yaml` 을 못 읽음. 토픽이 모델명을
얻는 유일한 경로임.

빈 칸 규칙:

```
상전류를 아직 못 받은 모터        그 모터 열이 null, 합도 null
사양을 안 넣은 모델 (RS00·RS02)   그 모터 열이 null, 합도 null
전부 모르는 모델                  열을 아예 안 만듦
extra=0                         열을 아예 안 만듦
```

합을 `null` 로 두는 이유: 그 모터를 빼고 더하면 일부만 더한 값이 전체 합으로 보임.

### 아직 안 들어간 것

```
기계 출력    토크·속도가 관찰에 있음 (leg.py:572-573). 기록 열에도 MotorExtra 에도
            없어서 log_node 가 못 받음
철손        상수가 사양서에 없음
인버터 손실  상수가 사양서에 없음
감속기 손실  9:1 의 효율값이 사양서에 없음
huphy-run   --extra 기록에는 전력 열이 없음. 모델별 상저항을 꺼내는 길이 없음
```
