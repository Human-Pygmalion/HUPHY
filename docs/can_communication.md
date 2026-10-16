# CAN 통신 — 무엇이 문제였고 지금은 어떻게 하나

**1.0 양다리의 모터 통신을 2026-10-12 ~ 10-15 에 걸쳐 진단하고 고친 기록임.** 응답이
빠지고(미수신), 주기가 밀리고, 실행마다 결과가 3% ~ 23% 로 들쭉날쭉하던 문제의 원인이
**세 겹**이었음. 순서대로 하나씩 벗겨 냄.

| | 원인 | 고친 것 | 커밋 |
|---|---|---|---|
| 1 | 어댑터 펌웨어가 slcan -- 느리고 오류를 숨김 | candleLight(gs_usb) 로 교체 | `395952c` `32b156a` |
| 2 | 직접 빌드한 커널 5.15 판 gs_usb 드라이버가 USB 패킷을 어긋나게 읽음 | 커널 6.1 판 드라이버를 백포트해 교체 | (이 문서 3절. 저장소 밖) |
| 3 | **모든 모터가 같은 CAN id 로 응답** -- 몰아서 보내면 응답끼리 부딪힘 | 하나씩 보내기 `--send-mode seq` | `24ef2a0` |

곁가지로 고친 것: gs_usb 의 오류 프레임을 응답으로 세던 것 (`fc32e78`), 지연 기록에서
감쇠 구간을 빼고 정책 구간만 남긴 것 (`55384ae`).

관련 문서: 펌웨어 전환 절차는 [candlelight.md](candlelight.md), 한 주기 흐름은
[cycle.md](cycle.md).

---

## 1. 구성

```
Jetson (L4T R36.4.3, 커널 5.15.148-tegra)
 ├─ USB ─ EM Tech USB-CAN (LC_CANABLE_PRO, STM32F072, 절연형) ─ canr ─ 오른다리 모터 7~12
 └─ USB ─ EM Tech USB-CAN (같은 것)                          ─ canl ─ 왼다리 모터 1~6
```

| | 지금 | 예전 |
|---|---|---|
| 어댑터 펌웨어 | candleLight `85e5b33` (`canable` 대상) | canable-fw (slcan) |
| 리눅스 드라이버 | **gs_usb, 커널 6.1.115 판을 5.15 로 백포트** | slcan (`slcand`) -> gs_usb 5.15 판 |
| 채널 이름 | `canr` / `canl` (udev, 어댑터 시리얼로 고정) | `slcan1` / `slcan0` |
| 비트레이트 | 1 Mbps, sample point 0.75, SJW 1 (커널 기본) | `slcand -s8` |
| 버스 하나 | 모터 6개 (RS02/RS03/RS04), MIT 프로토콜 | 같음 |

### MIT 프로토콜의 응답 id

    명령   11비트 id = 모터 CAN id (7, 8, ... 12)        데이터 8바이트
    응답   11비트 id = **호스트 id (0xFD) -- 모든 모터 공통**   데이터[0] = 모터 CAN id

**응답 프레임의 CAN id 가 모터마다 다르지 않음.** 누가 보낸 응답인지는 데이터 첫 바이트로만
구분함. 실측 캡처 (오른다리, `candump -x canr`):

```
TX  007  7F FF 7F F0 00 00 07 FF        <- 명령 6개를 0.6 ms 안에 연달아
TX  008  ...
TX  00C  ...
RX  0FD  07 7F E3 7F 77 FF 01 22        <- 응답은 전부 0FD
RX  0FD  08 80 12 7F 67 FF 01 04
RX  0FD  0B ...                          <- 순서는 모터 처리 시간 따라 섞임
```

호스트 id 는 모터 세팅 때 넣은 값임 ([motor_setup.md](motor_setup.md) 의 `HH` = `FD`).

---

## 2. 문제 1 -- slcan 펌웨어

slcan 은 CAN 프레임을 ASCII 문자열로 바꿔 USB 시리얼(가상 COM)로 주고받고, `slcand`
데몬이 그걸 socketcan 장치로 바꿈.

| 실측 (양다리 12모터, 토크 0) | |
|---|---|
| 12 프레임 송신 | 약 1.6 ms (프레임마다 직렬로 씀) |
| 마지막 응답 도착 | 송신 뒤 약 4.2 ms. 응답이 0.3~0.4 ms 간격으로 띄엄띄엄 옴 |
| 프레임 유실 | 3~6 %, CAN id 가 클수록 많음 |

- **느림.** CAN 선 위에서 프레임 하나는 0.13 ms 인데 어댑터 <-> USB 구간이 병목임
- **오류를 안 알려 줌.** 버스 오류가 나도 오류 프레임이 안 올라와서, 빠진 이유가 안 보였음
- **속도를 `slcand -sN` 으로 줌.** 처음 이 기기로 옮겼을 때 `-s6`(500k) 으로 떠 있어서 채널은
  UP 인데 받는 것이 0 이었음. `-s8` 이 1 Mbps

candleLight 로 바꾸는 절차는 [candlelight.md](candlelight.md). 오른다리 10-13, 왼다리 10-15.

---

## 3. 문제 2 -- 옛 gs_usb 드라이버가 패킷을 어긋나게 읽음

JetPack 6 커널에는 gs_usb 가 없어서 커널 5.15.148 소스의 `gs_usb.c` 를 따로 빌드해
넣었음. 그런데 **이 옛 드라이버와 최신 candleLight 펌웨어가 USB 패킷 형식에서 어긋남.**

### 증상

```
gs_usb 1-2.2.4.4:1.0 canr: Unexpected out of range echo id 117440512
gs_usb 1-2.2.4.4:1.0 canr: Unexpected out of range echo id 134217728
gs_usb 1-2.2.4.3:1.0 canl: __can_get_echo_skb: BUG! Trying to access can_priv::echo_skb out of bounds (262144/max 10)
gs_usb 1-2.2.4.3:1.0 canl: can_put_echo_skb: BUG! echo_skb 5 is occupied!
```

드라이버가 "내가 보낸 프레임의 확인 번호(echo id, 정상 0~9)" 칸에서 읽은 값:

| echo id | 16진수 |
|---|---|
| 117440512 | `0x07000000` |
| 134217728 | `0x08000000` |
| 150994944 | `0x09000000` |
| 167772160 | `0x0A000000` |
| 184549376 | `0x0B000000` |

**맨 앞 바이트가 오른다리 모터 번호 7~11 임.** 모터 응답 데이터(첫 바이트 = 모터 번호)를
바이트가 어긋난 채로 읽은 것. 결과:

- 응답 프레임이 깨져서 버려짐 -> 미수신
- 송신 확인 슬롯(10개)이 반납되지 않고 막힘 -> **어느 순간 송신이 통째로 멈춤**

토크 0 상태 조회로 재현한 것 (오른다리, 옛 드라이버):

| | 보냄 | 받음 | |
|---|---|---|---|
| 몰아서 1회째 (300주기) | 1800 | 926 | 48.6 % 유실 |
| 하나씩 | 1800 | 0 | 100 % -- 이미 송신이 멈춤 |
| 몰아서 2회째 | 1800 | 0 | 100 % |

이때 어댑터 송신 카운터(`ip -s link`)는 **972 에서 멈춰 있었음** (프로그램은 5400 을 보냄).

### 어댑터 자체는 정상이었음 -- 루프백 시험

통신선을 뺀 왼다리 어댑터를 `loopback on` 으로 올려(자기가 보낸 것을 자기가 받음) 모터·배선
없이 어댑터 + USB + 드라이버만 시험함.

| | 보냄 | 돌아옴 | 오류 | 커널 echo 오류 |
|---|---|---|---|---|
| 꾸준히 초당 5000 개 | 20,000 | 20,000 | 0 | 0 |
| 10 ms 마다 6 개 묶음 (실제 패턴) | 5,916 | 5,916 | 0 | 0 |

모터 6개의 응답이 짧은 간격으로 몰려 들어올 때만 어긋남이 생겼음.

### 오류 프레임도 상당수가 이것이었을 수 있음

gs_usb 로 바꾼 뒤 이런 오류 프레임이 초당 약 135 개씩 (두 다리 똑같이) 올라왔음.

```
class 0x28C   버스 오류 + 프로토콜 오류 + 컨트롤러 상태 + 카운터
data[2] 0x04  스터프 오류
data[1]       수신 error-passive <-> warning 반복
data[6] TEC   0         <- 어댑터가 보낸 것은 문제없음
data[7] REC   255 <-> 126
```

sample point 0.875, SJW 4 로 바꿔도 그대로였음 (비트 타이밍 문제가 아니었음). 드라이버를
바꾼 뒤에는 하나씩 보낼 때 오류 프레임이 1800 개 송신에 1 개로 줄었음.

### 고친 것 -- 커널 6.1 판 gs_usb 백포트

| | |
|---|---|
| 소스 | `gregkh/linux` `v6.1.115` 의 `drivers/net/can/usb/gs_usb.c` |
| 빌드 폴더 | `~/gs_usb_build_v6.1/` (`gs_usb.c`, `Makefile`, `gs_usb.ko`) |
| 설치 위치 | `/lib/modules/5.15.148-tegra/extra/gs_usb.ko` |
| 옛 모듈 백업 | `~/gs_usb_build/gs_usb_5.15_installed.ko.bak` |

6.6 판은 5.15 에 없는 rx-offload 함수를 많이 써서 안 됨. 6.1 판은 5.15 에 없는 보조 함수
**셋만** 빠져 있어서, `#include <linux/can/error.h>` 바로 뒤에 같은 동작을 넣음.

```c
#include <linux/version.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 0, 0)
static inline bool can_dev_dropped_skb(struct net_device *dev, struct sk_buff *skb)
{
	return can_dropped_invalid_skb(dev, skb);          /* 5.15 에 있는 같은 검사 */
}
static inline int can_eth_ioctl_hwts(struct net_device *netdev, struct ifreq *ifr, int cmd)
{
	return -EOPNOTSUPP;                                 /* HW 타임스탬프 설정. 안 씀 */
}
static inline int can_ethtool_op_get_ts_info_hwts(struct net_device *netdev,
						  struct ethtool_ts_info *info)
{
	return ethtool_op_get_ts_info(netdev, info);
}
#endif
```

프레임 송수신 경로는 6.1 판 그대로임.

```bash
# 빌드
mkdir -p ~/gs_usb_build_v6.1 && cd ~/gs_usb_build_v6.1
curl -fsSL -o gs_usb.c https://raw.githubusercontent.com/gregkh/linux/v6.1.115/drivers/net/can/usb/gs_usb.c
# (위 보조 함수 블록을 넣음)
echo 'obj-m += gs_usb.o' > Makefile
make -C /lib/modules/$(uname -r)/build M=$PWD modules
modinfo ./gs_usb.ko | grep vermagic          # 5.15.148-tegra 여야 함

# 교체
sudo ip link set canr down; sudo ip link set canl down
sudo rmmod gs_usb
cp /lib/modules/$(uname -r)/extra/gs_usb.ko ~/gs_usb_build/gs_usb_5.15_installed.ko.bak
sudo cp ~/gs_usb_build_v6.1/gs_usb.ko /lib/modules/$(uname -r)/extra/gs_usb.ko
sudo depmod -a && sudo modprobe gs_usb

# 되돌리기
sudo rmmod gs_usb
sudo cp ~/gs_usb_build/gs_usb_5.15_installed.ko.bak /lib/modules/$(uname -r)/extra/gs_usb.ko
sudo depmod -a && sudo modprobe gs_usb
```

**커널을 업데이트하면 다시 빌드해야 함** (vermagic 이 커널 버전에 묶임). 드라이버는 커널
모듈 하나라 두 어댑터에 같이 걸림.

확인:

```bash
md5sum /lib/modules/$(uname -r)/extra/gs_usb.ko ~/gs_usb_build_v6.1/gs_usb.ko   # 같아야 함
journalctl -k --since "-10min" | grep -c "echo id"                              # 0 이어야 함
```

---

## 4. 문제 3 -- 같은 응답 id 를 몰아서 받음

드라이버를 바꾸고 나니 다른 원인이 다 빠지고 이것 하나가 깨끗하게 남음.

### 원리

CAN 은 여러 노드가 동시에 보내기 시작하면 **id 비트를 비교해(중재) 한 노드만 남기고**
나머지는 물러났다가 다음에 보냄. 그래서 원래는 충돌로 데이터가 깨지지 않음. 이게 성립하려면
**동시에 보낼 수 있는 노드끼리 id 가 달라야 함** (ISO 11898 이 전제함).

id 가 같으면:

1. 중재 단계에서 둘 다 자기가 이겼다고 봄
2. 데이터까지 보내다가 모터 번호 바이트에서 비트가 부딪힘
3. CAN 이 오류로 감지 -> 오류 프레임 -> 둘 다 재전송
4. 같은 순간에 다시 시작하면 또 부딪힘. 한쪽이 오류 카운터가 쌓여 물러날 때까지 반복

그 사이 응답이 늦어지거나, 수거 대기 안에 못 들어와 빠짐. **CAN 이 재전송은 해 주지만 다
살리지는 못함.**

기존 방식은 한 다리 6개에 명령을 0.6 ms 안에 연달아 보내고, 모터들은 각자 처리 시간
(1~3.5 ms)이 지나면 바로 응답함. 그래서 **두 모터의 응답 시작이 겹치는 일이 구조적으로
생김.** 즉 기존 방식은 "이 하드웨어에서 우연히 나빠진" 게 아니라, 공용 응답 id 와 같이 쓰면
**원래 조금씩 깨지게 되어 있는 방식**이었음.

### 증거

**빠질 때 둘씩 빠짐.** 오른다리 696 주기 캡처 (몰아서):

| 한 주기에 받은 응답 수 | 주기 수 |
|---|---|
| 6 (다 받음) | 622 |
| **4 (둘이 같이 빠짐)** | **71** |
| 5 (하나만 빠짐) | 1 |
| 2 / 8 (중복 수신) | 각 1 |

**보내는 방식만 바꾸면 사라짐.** 새 드라이버, 토크 0 상태 조회, 다리마다 300 주기 x 6 모터:

| | 몰아서 (burst) | 하나씩 (seq) |
|---|---|---|
| 오른다리 유실 | 7.6 % / 9.0 % | **0 %** |
| 왼다리 유실 | 8.3 % / 8.3 % | **0 %** |
| 오류 프레임 | 약 600 | **1** |
| 빠질 때 둘 이상 같이 빠진 주기 | 68~80 | -- |

모터별로는 번호가 큰 모터(무릎·발목)가 더 많이 빠짐 -- 뒤에 응답할수록 겹칠 기회가 많음.

### 고친 것 -- 하나씩 보내기

```
burst (기존)                              seq (지금)
  왼1~6, 오른7~12 를 연달아 보냄             왼1·오른7 보냄 -> 둘 응답 받음
  응답 12 개가 다 오거나                     왼2·오른8 보냄 -> 둘 응답 받음
  수거 대기가 끝날 때까지 모음               ...
                                           왼6·오른12 보냄 -> 둘 응답 받음
```

- **한 버스 안에서는 한 번에 한 모터만 응답함** -> 같은 id 가 겹칠 일이 없음
- 두 다리는 버스가 따로라 i 번째끼리 동시에 진행함 -> 한 주기가 다리 수만큼 길어지지 않음
- 응답이 안 오는 모터는 그 모터만 `--drain-ms` 까지 기다리고 다음으로 넘어감
- 기다리는 동안 다른 모터의 늦은 응답이 오면 그것도 받아 둠 (버리면 그 모터가 무응답이 됨)

| 비용 (왼다리, 토크 0) | p50 | p95 | 최대 |
|---|---|---|---|
| 모터 하나 왕복 (보내고 응답) | 0.69 ms | 0.96 ms | 4.39 ms |
| 다리 하나 6 모터 순차 | 4.33 ms | 5.27 ms | 8.78 ms |
| 참고: 몰아서 보낼 때 마지막 응답 | 약 3.5 ms | | (8 % 유실) |

1 ms 남짓 늘지만 50Hz(20 ms) 는 물론 100Hz(10 ms) 에도 들어감. 양다리 관찰 모드 100Hz 5초:

| | burst | seq |
|---|---|---|
| 무응답 주기 | 43.4 % / 39.3 % | **0 % / 0 %** |
| 프레임 유실 | 5.8 % / 5.0 % | **0.2 % / 0.2 %** (시작할 때 토크 끄는 명령 몫) |
| 버스 오류 프레임 | 1,367 / 1,314 | **4 / 4** |
| 평균 주기 | 89.4 / 89.5 Hz | **95.6 / 94.5 Hz** |

빠진 응답을 기다리느라 수거 대기를 끝까지 쓰는 주기가 없어져서 **주기도 빨라짐.**

토크를 켠 실제 동작 (`huphy-test --robot --hz 100 --drain-ms 4 --send-mode seq zero --log`,
약 25초, 응답 27,840 개): 늦음 0 %, 유실 0 %, 지연 p50 0.70 / p95 1.19 / p99 1.73 ms.

### 코드

| 곳 | 무엇 |
|---|---|
| `RobStrideBus.collect_until(mid)` | 그 모터 응답이 올 때까지만 받음. 받은 모든 모터 id 를 돌려줌 |
| `RobStrideBus._absorb(frames)` | 응답 해석. `collect` 와 `collect_until` 이 같이 씀 |
| `Leg.send_mode` | `burst` / `seq`. **seq 면 `send()` 안에서 송신과 수거를 다 끝냄** -- `collect()` 는 결과만 정리하므로, `collect()` 를 안 부르는 경로가 생겨도 명령은 나감 |
| `Biped._seq_run` | 두 다리가 다 `seq` 면 i 번째끼리 엇갈려 보냄. 한 다리가 실패해도 나머지는 계속 |
| `--send-mode` | `huphy-test`, `huphy-run`. `build_leg(send_mode=...)` |

**기본은 `seq` 임** (2026-10-16 부터). `huphy-test`, `huphy-run`, `huphy-bringup` 이 지나는
`build_leg` 의 기본값이고, 예전 방식은 `--send-mode burst` 로 고름. `Leg` 클래스 자체의
기본은 `burst` 로 남겨 둠 -- 가짜 버스로 도는 테스트들이 그것을 전제로 짜여 있음.
`huphy-commission` 은 `RobStrideBus.refresh_states` 를 바로 써서 여전히 몰아서 보냄
(한 번씩 상태를 읽는 용도라 주기 문제는 없지만, 여러 모터를 한꺼번에 scan 하면 가끔
빠질 수 있음).

### 다른 해법 -- 모터마다 응답 id 를 다르게

모터의 호스트 id 를 모터마다 다르게 설정(`CMD_SET_HOST_ID`)하면 응답이 겹쳐도 CAN 중재가
정상 동작해서 몰아서 보내도 안전함. 주기 손해도 없음. 다만 모터 12 개 설정을 바꿔야 하고,
응답 id 를 0xFD 로 가정하는 곳이 있는지 점검이 필요해서 지금은 안 함.

---

## 5. 지금 쓰는 방법

### 채널 올리기

```bash
sudo ip link set canr up type can bitrate 1000000
sudo ip link set canl up type can bitrate 1000000
ip -details link show canr        # can state ERROR-ACTIVE, bitrate 1000000
```

- gs_usb 는 `ip link` 에 **bitrate 를 꼭 넘겨야 함.** 빠뜨리면 `RTNETLINK answers: Invalid argument`
- **어댑터를 뽑았다 꽂으면 채널이 새로 만들어져 `STOPPED` 로 돌아옴.** 다시 올려야 함. 안
  올린 채로 돌리면 소켓만 열리고 아무것도 안 오감 (`huphy-commission ... scan` 이 멈춘 것처럼 보임)
- slcan 이 남아 있으면 `sudo pkill slcand`

### 실행

```bash
huphy-test --robot --hz 100 --drain-ms 4 zero --log          # seq 가 기본
huphy-run  --robot --policy stand --ankle-space ab --weights config/policies/stand_bi.pt \
           --hz 50 --drain-ms 4 --log
```

### 끝 집계에서 볼 것

| 칸 | 정상 | 아니면 |
|---|---|---|
| 무응답 주기 | 0 % 근처 | `--send-mode burst` 로 돌리지 않았는지, 채널이 올라왔는지 |
| 버스오류 | 0 근처 | burst 로 돌았는지, 배선·종단(60 Ω)·절연 어댑터 GND |
| 밀림 | 0 | 주기에 비해 `--drain-ms` 가 큰지 |

```bash
journalctl -k --since "-10min" | grep -E "gs_usb.*(echo|BUG)"    # 드라이버 문제. 나오면 3절 확인
```

---

## 6. 지연 기록 -- 무엇을 어떻게 세나

`huphy-run --log`, `huphy-test --log` 이 `latency_<이름>_<시각>.csv` 를 남기고 끝날 때
관절별 표를 찍음. `tools/latency_viewer.html` 로 히스토그램을 봄. 코드는
`telemetry/latency_log.py` 와 `RobStrideBus._note_sent` / `_note_received`.

### 시각

| | 찍는 곳 | |
|---|---|---|
| 송신 시각 | `send_mit()` 가 프레임을 내보내기 직전, 모터마다 | burst 는 6 개가 같은 시각 (묶음 직렬 시간까지 지연에 들어감). seq 는 모터마다 자기 송신 시각 |
| 직전 송신 시각 | 같은 모터의 한 번 전 송신 | 늦은 응답을 원래 명령에 맞추는 데 씀 |
| 도착 시각 | 수신 스레드가 USB 에서 프레임을 꺼낸 순간 | 제어 루프가 나중에 수거해도 지연은 실제 도착 기준 |

### 응답 하나의 판정

| 조건 | 판정 | 기록하는 지연 |
|---|---|---|
| 도착 ≥ 이번 송신 | **제때 (ontime)** | 도착 − 이번 송신 |
| 직전 송신 ≤ 도착 < 이번 송신 | **늦음 (late)** | 도착 − 직전 송신 |
| 도착 < 직전 송신 | 버림 (두 주기 넘게 묵음) | 없음 |

MIT 응답에는 순번이 없어서 시각으로만 맞춤. 도착 시각을 수신 스레드가 찍으므로 이 구분이
성립함.

**"늦음" 은 "그 모터에 다음 명령이 나간 뒤에야 응답이 들어온 것" 임.** `--drain-ms` 를
넘었는지가 기준이 아님.

- **burst**: 대기를 넘긴 응답은 큐에 남았다가 다음 주기 수거 때 해석됨. 그때는 다음 명령이
  나간 뒤라 **늦음**
- **seq**: 대기를 넘긴 응답이 같은 주기 안에서 다음 모터를 기다리는 동안 들어오면 **제때**.
  지연은 대기보다 큰 값으로 남음 -- "늦음 0 % 인데 최대 12 ms" 가 이 경우임

### 세는 규칙

- 송신 하나마다 "이 송신을 셀지" 를 정함 -- 기록이 켜져 있고 건너뛰기가 지났으면 셈
- **센 송신 하나당 응답은 최대 하나만 셈.** 같은 송신의 두 번째 응답은 버림
- 늦은 응답은 그 사이 나간 다음 송신의 칸을 쓰고 들어감 -> 그 다음 송신의 진짜 응답은 따로
  안 셈. 송신 하나에 표본이 하나를 넘지 않게 하려는 것
- 꺼진 동안 나간 송신은 송신도 응답도 안 셈 -> 켜고 끄는 경계에서 가짜 유실이 안 생김

### 비율

```
보냄    = 센 송신 수
늦음    = late 표본 수
제때    = 전체 표본 수 − 늦음
유실    = max(0, 보냄 − 전체 표본 수)

늦음 %  = 늦음 / 보냄
유실 %  = 유실 / 보냄
p50·p95·p99·최대 = 받은 표본(제때 + 늦음)의 지연 분포
```

유실에 들어가는 것: 끝내 안 온 응답, 두 주기 넘게 늦어 버린 응답, 실행이 끝나는 순간 아직
해석되지 않은 마지막 응답 몇 개 (짧은 실행에서 아주 조금 생김).

### 어느 구간을 세나

| | `huphy-run --log` | `huphy-test --log` |
|---|---|---|
| 시작 | **정책이 처음 불린 순간** | 첫 송신 |
| 앞부분 버리기 (`--log-skip`) | 기본 0 -- 정책 시작은 안 자름 | 기본 2 초 -- 연결 직후 첫 주기들이 느림 |
| 빠지는 구간 | 접근 자세, Enter 대기, 감쇠 정지 | 감쇠 정지 (studio 의 관찰 전환 포함) |

감쇠는 `ControlLoop.on_settle` 이 시작·끝을 알려 `LatencyLog.on_settle` 이 끄고 켬.
정책 입출력 기록(`policy_*.csv`) 은 원래 정책이 불릴 때만 남김.

### 다른 숫자와 헷갈리지 말 것

| 숫자 | 어디서 | 세는 것 |
|---|---|---|
| 지연 표의 늦음·유실 | `latency_log` | 위 방식. MIT 명령 응답만, 기록이 켜진 구간만 |
| 끝 집계의 "미수신" | `CanBus` 카운터 | 연결 뒤 **모든** 송신 프레임 − 받은 프레임. 토크 켜기·끄기, 접근·감쇠 구간도 들어감 |
| "무응답 주기", 모터별 "빠짐" | `Leg` | 그 주기 **수거 시점까지** 응답이 없던 것. 늦게 온 것도 빠짐으로 셈 |
| "버스오류" | `CanBus.counters.error_frames` | gs_usb 가 올린 CAN 오류 프레임. 응답으로 세지 않음. slcan 은 늘 0 |

그래서 같은 실행이라도 셋이 다르게 나올 수 있음. 특히 burst 에서는 지연 표의 "늦음" 이 끝
집계에서는 "빠짐" 으로 잡힘.

---

## 7. 진단에 쓴 방법

다시 볼 일이 생기면 이 순서로 봄.

```bash
# 오류 프레임만 (종류를 디코드). 필터의 #FFFFFFFF 가 오류 마스크 -- 없으면 오류 프레임이 안 옴
candump -e -t a canr,0~0,#FFFFFFFF canl,0~0,#FFFFFFFF

# 데이터 프레임 전부 (응답 id, 순서, 빠진 짝)
candump -t a -x canr

# 어댑터 오류 상태와 카운터
ip -details -statistics link show canr     # can state, error-warn, error-pass, bus-off

# 드라이버
journalctl -k | grep gs_usb

# 어댑터만 시험 (통신선을 뺀 어댑터에서만! 루프백도 버스로 프레임이 나감)
sudo ip link set canl down
sudo ip link set canl type can bitrate 1000000 loopback on
sudo ip link set canl up
cangen canl -I 123 -L 8 -D i -g 0.2 -n 20000 -i     # 보낸 수 = 받은 수 인지
sudo ip link set canl down && sudo ip link set canl type can bitrate 1000000 loopback off && sudo ip link set canl up
```

틀렸던 가설도 남김 -- 같은 길을 다시 안 가려고.

| 가설 | 결과 |
|---|---|
| 왼다리를 candleLight 로 바꿔서 나빠짐 | 아님. 오른다리만 바꿨을 때도 실행마다 3 ~ 23 % 로 들쭉날쭉했고, 나쁜 실행에서는 두 다리(slcan / gs_usb) 가 같이 나빠졌음 |
| sample point (0.75 -> 0.875) | 변화 없음 |
| 어댑터 클럭 오차 -> SJW 1 -> 4 | 변화 없음 |
| 어댑터 하드웨어 불량 | 아님. 루프백 시험 깨끗함 |

---

## 8. 남은 일

- [x] `--send-mode` 기본값을 `seq` 로 (10-16)
- [ ] udev 로 어댑터를 꽂으면 1 Mbps 로 자동으로 올리기 (지금은 꽂을 때마다 `ip link set ... up`)
- [ ] 커널 업데이트 때 gs_usb 6.1 백포트 다시 빌드 (3절)
- [ ] (선택) 모터마다 호스트 id 를 다르게 -- 몰아서 보내도 안전해짐
