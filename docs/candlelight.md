# USB-CAN 어댑터 전환 — slcan 에서 candleLight(gs_usb) 로

**양다리 CAN 통신의 지연과 유실을 줄이려고 어댑터 펌웨어를 바꾸는 절차임.**

    지금   EM Tech USB to CAN (Isolated, SLCAN)   펌웨어 canable-fw (slcan)
    목표   같은 어댑터                             펌웨어 candleLight (gs_usb)

이름 둘을 구분할 것.

| 이름 | 무엇 | 어디에 있나 |
|---|---|---|
| **candleLight** | 어댑터에 올리는 펌웨어 | 어댑터 STM32 플래시 |
| **gs_usb** | 그 펌웨어를 받는 리눅스 커널 드라이버 | Jetson 커널 모듈 |

**코드는 안 바뀜.** 지금도 socketcan 으로 열고 있어서 `robot.yaml` 의 `channel`
(`canr` / `canl`)만 맞으면 됨. 바뀌는 것은 채널을 올리는 방법뿐임.

> **상태: 양다리 전환 끝 (10-15).** `canr`/`canl` 둘 다 gs_usb 로 올라옴. 모터 응답과
> 효과 확인(6절)이 남음. 진행한 것은 바로 아래 "진행 기록".

---

## 진행 기록

| 날짜 | 단계 | 결과 |
|---|---|---|
| 10-13 | gs_usb 빌드 (2-1) | `~/gs_usb_build/gs_usb.ko`. vermagic `5.15.148-tegra` 일치 |
| 10-13 | gs_usb 설치 (2-2) | `lsmod` 에 `gs_usb` 보임 |
| 10-13 | `dfu-util` 설치 | `sudo apt install -y dfu-util` (0.9) |
| 10-13 | 오른다리 DFU 진입 (3-2) | `0483:df11 STM Device in DFU Mode` |
| 10-13 | 오른다리 칩 확인 (3-3) | **STM32F072** (128K). 아래 출력 |
| 10-13 | 오른다리 백업 (4-1) | `firmware/canable/backup_right.bin` 131072 바이트 |
| 10-13 | candleLight 빌드 (4-2) | `canable` 대상, 소스 `85e5b33`. `firmware/canable/candlelight_canable_85e5b33.bin` |
| 10-13 | 오른다리 candleLight 쓰기 (4-2) | `1d50:606f canable.io canable gs_usb`. **시리얼 그대로** |
| 10-13 | 오른다리 이름 고정 (5) | `/etc/udev/rules.d/70-candlelight.rules` -> `canr` |
| 10-13 | 오른다리 채널 올리기 (6) | `state ERROR-ACTIVE`, `bitrate 1000000` |
| 10-15 | 왼다리 칩 확인 (3-3) | **STM32F072** (128K). 오른다리와 같음 |
| 10-15 | 왼다리 백업 (4-1) | `firmware/canable/backup_left.bin` 131072 바이트 |
| 10-15 | 왼다리 candleLight 쓰기 (4-2) | `1d50:606f`. **시리얼 그대로** (`004100375034570520323530`) |
| 10-15 | 왼다리 이름 고정 (5) | `70-candlelight.rules` 에 줄 추가 -> `canl` |
| 10-15 | 양다리 채널 올리기 (6) | `canr`/`canl` 둘 다 `state ERROR-ACTIVE`, `bitrate 1000000`. `slcand` 안 씀 |
| | 모터 응답 (`scan`) | 다음 차례 |
| | 효과 확인 (6) | |

처음 쓰기는 들어가지 않았음 -- 다시 꽂아 보니 `ad50:60c4 ... canable-fw.git` 그대로였음.
원인은 확인 못 함. 4-2 (나) `dfu-util` 로 다시 해서 됨.

### 지금까지 친 명령

```bash
# Jetson -- gs_usb
mkdir -p ~/gs_usb_build && cd ~/gs_usb_build
curl -fsSL -o gs_usb.c \
  https://raw.githubusercontent.com/gregkh/linux/v5.15.148/drivers/net/can/usb/gs_usb.c
echo 'obj-m += gs_usb.o' > Makefile
make -C /lib/modules/$(uname -r)/build M=$PWD modules
sudo cp ~/gs_usb_build/gs_usb.ko /lib/modules/$(uname -r)/extra/
sudo depmod -a
sudo modprobe gs_usb

# 도구
sudo apt install -y dfu-util

# 오른다리 어댑터 -- BOOT0 구멍 두 개를 이은 채로 꽂은 뒤
lsusb | grep 0483:df11
sudo dfu-util -l              # 일반 사용자로는 "Cannot open DFU device" -- sudo 필요

# 오른다리 백업 (레포 루트에서)
sudo dfu-util -a 0 -s 0x08000000:131072 -U firmware/canable/backup_right.bin
sudo chown $USER: firmware/canable/backup_right.bin

# candleLight 빌드 (한 번만)
sudo apt install -y gcc-arm-none-eabi libnewlib-arm-none-eabi
cd ~ && git clone https://github.com/candle-usb/candleLight_fw
cd ~/candleLight_fw && mkdir -p build && cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE=../cmake/arm-none-eabi-gcc.cmake
make canable_fw

# 오른다리 쓰기 -- 다시 BOOT0 를 이어 DFU 로 꽂은 뒤
sudo dfu-util -d 0483:df11 -a 0 -s 0x08000000:leave -D ~/candleLight_fw/build/canable_fw.bin

# 확인 -- BOOT0 를 잇지 않고 다시 꽂은 뒤
lsusb | grep 1d50:606f        # 1d50:606f OpenMoko, Inc. Geschwister Schneider CAN adapter
ip -br link show type can     # can1 이 새로 생김 (can0 은 Jetson 내장)

# 이름 고정
echo 'SUBSYSTEM=="net", ACTION=="add", ATTRS{idVendor}=="1d50", ATTRS{idProduct}=="606f", ATTRS{serial}=="003E00315550530B20303731", NAME="canr"' | sudo tee /etc/udev/rules.d/70-candlelight.rules
sudo udevadm control --reload-rules
# 뽑았다 다시 꽂음 -> can1 대신 canr

# 채널 올리기
sudo pkill slcand
sudo ip link set canr up type can bitrate 1000000
ip -details link show canr    # state ERROR-ACTIVE, bitrate 1000000
```

오른다리 `dfu-util -l` 출력.

```
Found DFU: [0483:df11] ver=2200, devnum=11, cfg=1, intf=0, path="1-2.3", alt=1, name="@Option Bytes  /0x1FFFF800/01*016 e", serial="FFFFFFFEFFFF"
Found DFU: [0483:df11] ver=2200, devnum=11, cfg=1, intf=0, path="1-2.3", alt=0, name="@Internal Flash  /0x08000000/064*0002Kg", serial="FFFFFFFEFFFF"
```

`064*0002Kg` = 2KB 페이지 64개 = **128KB -> STM32F072.** F042 였다면 `032*0001Kg`.
지금 들어 있는 slcan 펌웨어(F042 용 빌드)가 이 F072 에서 돌고 있었음 -- F042 용
`canable` candleLight 빌드도 돌 것으로 봄. 안 되면 백업으로 되돌림.

### 백업 파일

레포 [`firmware/canable/`](../firmware/canable/) 에 둠. 어댑터 두 개의 원래 펌웨어임.

| 파일 | 무엇 | 크기 | 상태 |
|---|---|---|---|
| `backup_right.bin` | 오른다리 원래 펌웨어 (시리얼 `003E00315550530B20303731`) | 131072 | 뜸 |
| `backup_left.bin` | 왼다리 원래 펌웨어 (시리얼 `004100375034570520323530`) | 131072 | 뜸 |
| `candlelight_canable_85e5b33.bin` | 쓴 candleLight. `canable` 대상, 소스 커밋 `85e5b33` | 23624 | 양다리에 씀 |

백업은 전체 플래시(128K)를 그대로 뜬 것임. 되돌릴 때 그대로 씀 (7절). candleLight
파일이 있으면 왼다리나 재설치 때 다시 빌드할 필요 없음.

---

## 1. 왜 바꾸나

2026-10-12 `stand` 실행 지연 기록 3개 (`logs/latency_stand_*`, 50Hz, `--drain-ms 4`).

| | 실측 | 선 위에서 걸리는 시간 |
|---|---|---|
| 첫 모터 응답 | 1.3 ms | 0.3 ms 안팎 |
| 모터 하나 뒤로 갈 때마다 | +0.35~0.4 ms | +0.26 ms (명령+응답 프레임) |
| 마지막 모터(발목) | p50 3.0~3.3 ms, p95 5 ms대 | 1.5~2 ms |
| 유실 | 3~22% | 거의 0 |
| 버스 사용률 | 20 ms 주기 중 2 ms 미만 | — |

**버스는 비어 있음.** 원인은 선이 아니라 slcan 경로임.

- 프레임마다 ASCII 문자열로 바꿔 USB 시리얼(가상 COM 포트)로 보냄. `slcand` 데몬을
  한 번 더 거침 -> 기본 지연 약 1 ms, 프레임마다 조금씩 더 쌓임
- 흐름 제어가 없어 몰리면 그냥 버림 -> 들쭉날쭉한 유실

candleLight 는 CAN 프레임을 USB 로 그대로 넘기고 커널 드라이버(gs_usb)가 바로
socketcan 장치를 만듦. 문자열 변환, tty, `slcand` 가 전부 빠짐.

다른 방법과 비교.

| 방법 | 해결되는 것 | 남는 것 |
|---|---|---|
| **candleLight 로 전환** (이 문서) | 문자열 변환 지연, 유실 | USB 지연 (보통 1 ms 이하) |
| Jetson 내장 CAN + 트랜시버 | 전부 | 내장 CAN 이 `can0` 하나뿐 -- 다리 하나만 됨 |
| `--drain-ms` 늘리기 | 늦게 온 응답을 받음 | 지연·유실 자체는 그대로 |
| 비트레이트 올리기 / CAN FD | -- | RobStride 가 일반 CAN 1 Mbps 만 지원. 불가 |

---

## 2. Jetson 쪽 — gs_usb 드라이버

### 2-0. 왜 직접 빌드하나

JetPack 6 (L4T R36.4.3, 커널 `5.15.148-tegra`) 커널에는 gs_usb 가 **빠져 있음.**

```bash
$ zcat /proc/config.gz | grep -E 'GS_USB|SLCAN'
# CONFIG_CAN_SLCAN is not set
# CONFIG_CAN_GS_USB is not set
```

slcan 도 원래 없어서 `~/slcan_build` 에서 따로 빌드해 `/lib/modules/.../extra/` 에
넣어 둔 상태임. gs_usb 도 **같은 방식**으로 넣음. 커널 전체를 다시 빌드할 필요 없음.

### 2-1. 빌드

커널 헤더는 이미 깔려 있음 (`/lib/modules/$(uname -r)/build`).

```bash
mkdir -p ~/gs_usb_build && cd ~/gs_usb_build
curl -fsSL -o gs_usb.c \
  https://raw.githubusercontent.com/gregkh/linux/v5.15.148/drivers/net/can/usb/gs_usb.c
echo 'obj-m += gs_usb.o' > Makefile
make -C /lib/modules/$(uname -r)/build M=$PWD modules
```

**소스 버전은 커널과 같아야 함** (`v5.15.148`). 다르면 모듈이 안 올라감.

확인.

```bash
modinfo ./gs_usb.ko | grep -E 'vermagic|alias'
# vermagic:  5.15.148-tegra SMP preempt mod_unload modversions aarch64
# alias:     usb:v1D50p606F...     <- candleLight 의 USB id
```

2026-10-13 에 이대로 빌드해서 위 출력까지 확인했음.

### 2-2. 설치

```bash
sudo cp ~/gs_usb_build/gs_usb.ko /lib/modules/$(uname -r)/extra/
sudo depmod -a
sudo modprobe gs_usb
lsmod | grep gs_usb
```

`depmod` 를 해 두면 candleLight 어댑터를 꽂을 때 자동으로 올라옴.

---

## 3. 어댑터 쪽 — 펌웨어 쓰기 모드로 들어가기

### 지금 어댑터

```
$ lsusb
ID ad50:60c4 Protofusion Labs CANable 9fddea4 github.com/normaldotcom/canable-fw.git
```

| | 시리얼 | udev 링크 | 채널 | 칩 |
|---|---|---|---|---|
| 오른다리 | `003E00315550530B20303731` | `/dev/canable1` | `canr` | STM32F072 (128K) |
| 왼다리 | `004100375034570520323530` | `/dev/canable0` | `canl` | STM32F072 (128K) |

기판 뒷면 이름은 `LC_CANABLE_PRO` 임. CANable Pro(절연형)를 따라 만든 보드이고,
원래 CANable 펌웨어(`canable-fw`, F042 용 빌드)가 F072 칩에서 그대로 돌고 있음.

### 3-1. BOOT0 점퍼 자리

뒷면에 `BOOT0` 표시와 **구멍 두 개**가 있음. 점퍼 핀을 안 꽂고 자리만 남긴 것임.

```
 뒷면
 BOOT0                 3V3 SWDIO SWCLK GND
 (O)(O)  <- 이것        [ ]  [ ]  [ ]  [ ]   <- SWD 디버그 패드. 이번엔 안 씀
  │  └── 3.3V
  └───── 칩의 BOOT0 핀
```

| 두 구멍 | 켜질 때 |
|---|---|
| 떨어져 있음 (평소) | BOOT0 = 0V -> 지금 펌웨어로 켜짐 |
| 이어져 있음 | BOOT0 = 3.3V -> **칩 ROM 의 DFU 부트로더**로 켜짐 |

구멍 없는 `3V3` 패드는 SWD 단자의 전원임. 이번 작업에는 안 씀.

**불안하면 먼저 잼.** 이어 두지 않고 USB 를 꽂은 채, 각 구멍과 GND (CAN 단자의
GND) 사이 전압을 멀티미터로 잼. 한쪽 약 3.3V, 다른 쪽 약 0V 면 맞는 구조임. 다르게
나오면 잇지 말 것.

### 3-2. 들어가기

1. 어댑터를 USB 에서 뽑음. CAN 선도 뺌
2. BOOT0 구멍 두 개를 이음 -- 클립 끝, 짧은 전선, 또는 2.54mm 핀헤더 + 점퍼 캡
3. 이은 채로 USB 를 꽂음
4. 확인

```bash
lsusb | grep 0483:df11
# ID 0483:df11 STMicroelectronics STM Device in DFU Mode
```

이게 보이면 이은 것을 떼도 됨. 다시 꽂을 때까지 이 모드가 유지됨. 여전히
`ad50:60c4 CANable` 로 나오면 두 구멍이 덜 닿은 것임.

### 3-3. 칩 확인

```bash
sudo apt install -y dfu-util
sudo dfu-util -l              # sudo 없으면 "Cannot open DFU device"
```

`Internal Flash` 줄의 크기가 **`032*0001Kg`(32K) 면 STM32F042, `064*0002Kg`(128K) 면
STM32F072** 임. 오른다리는 F072 였음. 둘 다
candleLight 지원 대상임. 그 외(F103, F303 등)가 나오면 **여기서 멈출 것** -- 해당
칩은 USB 와 CAN 을 동시에 못 써서 candleLight 가 안 돌아감.

---

## 4. 백업과 펌웨어 쓰기

### 4-1. 지금 펌웨어 백업

**쓰기 전에 반드시 뜸.** 이 파일이 있으면 지금과 완전히 같은 상태로 되돌릴 수 있음.

레포의 `firmware/canable/` 에 뜸. 레포 루트에서 침.

```bash
cd ~/Desktop/chanhuegit/HUPHY

# 오른다리. F072 (128K) 라 131072
sudo dfu-util -a 0 -s 0x08000000:131072 -U firmware/canable/backup_right.bin
sudo chown $USER: firmware/canable/backup_right.bin    # sudo 로 만들어져 root 소유임
ls -l firmware/canable/backup_right.bin                 # 131072 바이트여야 함

# 왼다리. 3-3 에서 칩을 먼저 확인하고 크기를 맞출 것 (F042 면 32768)
sudo dfu-util -a 0 -s 0x08000000:131072 -U firmware/canable/backup_left.bin
sudo chown $USER: firmware/canable/backup_left.bin
```

- 같은 이름의 파일이 이미 있으면 `dfu-util` 이 덮어쓰지 않고 멈춤. 다시 뜰 거면 지우고 침
- 칩에 읽기 보호가 걸려 있으면 `Error during upload` 로 실패함 -- 그때는 7절의 공개
  펌웨어로 되돌리면 됨. 지금 펌웨어가 공개 `canable-fw` 라 사실상 같음

### 4-2. candleLight 쓰기

둘 중 하나.

**(가) 브라우저** -- 가장 쉬움. 크롬에서 [canable.io 업데이트 페이지 (CANable 1.x)](https://canable.io/updater/canable1.html)
를 열고, 펌웨어를 candleLight 로 골라 "Connect and Update". 목록에 장치가 안 뜨면
udev 규칙이 필요하거나 크롬을 root 로 띄워야 한다고 페이지에 적혀 있음.

**(나) 명령줄** -- 오른다리는 이쪽으로 됨. 레포에 빌드해 둔 파일을 씀.

```bash
cd ~/Desktop/chanhuegit/HUPHY
sudo dfu-util -d 0483:df11 -a 0 -s 0x08000000:leave -D firmware/canable/candlelight_canable_85e5b33.bin
```

끝에 `File downloaded successfully` 가 나와야 함.

새로 빌드하려면 ([candleLight_fw](https://github.com/candle-usb/candleLight_fw) README).

```bash
sudo apt install -y gcc-arm-none-eabi libnewlib-arm-none-eabi
cd ~ && git clone https://github.com/candle-usb/candleLight_fw
cd ~/candleLight_fw && mkdir -p build && cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE=../cmake/arm-none-eabi-gcc.cmake
make canable_fw               # -> canable_fw.bin
```

**`canable` 대상(F042 용)을 고른 이유**: 원래 slcan 펌웨어도 F042 용 빌드인데 이 F072
보드에서 돌았음 -- 같은 핀 배치(LED PB0/PB1)를 쓰는 `canable` 이 가장 안전함. F072 용
대상(`CANable_MKS` 등)은 LED 핀이 다름.

### 4-3. 확인

뽑았다가 **BOOT0 를 잇지 않은 채** 다시 꽂음.

```bash
lsusb | grep 1d50:606f              # candleLight 로 잡힘
ip -br link show type can           # can1 같은 새 장치가 생김 (이름은 5절에서 고정)
```

**어댑터는 하나씩 바꿀 것.** 한쪽을 4-3 까지 확인한 다음에 나머지를 함. 문제가 생겨도
다른 하나는 그대로라 작업이 안 막힘.

---

## 5. 장치 이름 고정 — `canr` / `canl`

gs_usb 는 꽂는 순서대로 `can1`, `can2` 처럼 번호 이름을 붙임. 번호 이름은 **쓰지 않음**
-- Jetson 내장 CAN 이 이미 `can0` 을 차지하고 있어서 나중에 겹침.

시리얼 번호로 이름을 붙임. **펌웨어를 바꾸면 시리얼 표기가 달라질 수 있으므로** 바꾼
뒤에 다시 읽음.

```bash
udevadm info -a -p /sys/class/net/can1 | grep -m1 'ATTRS{serial}'
```

두 어댑터 모두 펌웨어를 바꾼 뒤에도 시리얼이 **그대로**였음. candleLight 도 칩 고유
번호로 시리얼을 만들기 때문으로 봄.

```
# /etc/udev/rules.d/70-candlelight.rules
SUBSYSTEM=="net", ACTION=="add", ATTRS{idVendor}=="1d50", ATTRS{idProduct}=="606f", ATTRS{serial}=="003E00315550530B20303731", NAME="canr"
SUBSYSTEM=="net", ACTION=="add", ATTRS{idVendor}=="1d50", ATTRS{idProduct}=="606f", ATTRS{serial}=="004100375034570520323530", NAME="canl"
```

두 줄 다 넣었음 (10-15). 아래는 왼다리 줄을 넣은 명령임.

```bash
echo 'SUBSYSTEM=="net", ACTION=="add", ATTRS{idVendor}=="1d50", ATTRS{idProduct}=="606f", ATTRS{serial}=="004100375034570520323530", NAME="canl"' | sudo tee -a /etc/udev/rules.d/70-candlelight.rules
```

```bash
sudo udevadm control --reload-rules
# 어댑터를 뽑았다 다시 꽂음
ip -br link show type can           # canr, canl 이 보여야 함
```

기존 `99-mydevice.rules` 의 `canable0` / `canable1` 줄은 tty 용이라 전환 후에는 안
맞음. 지워도 되고 남겨 둬도 해는 없음.

---

## 6. 채널 올리기 — 바뀐 방법

`slcand` 가 필요 없음. 속도를 커널이 들고 있으므로 Jetson 내장 CAN 처럼 `ip link` 에
넘김.

```bash
# 전 (slcan)
sudo slcand -o -c -s8 /dev/canable1 canr && sudo ip link set canr up

# 후 (gs_usb)
sudo ip link set canr up type can bitrate 1000000
sudo ip link set canl up type can bitrate 1000000
ip -details link show canr          # state UP, bitrate 1000000
```

`robot.yaml`, 코드는 그대로임.

### 효과 확인

전환 전과 같은 명령으로 지연 기록을 남겨 1절 표와 비교함.

```bash
huphy-run --robot --policy stand --ankle-space ab --hz 50 --drain-ms 4 --log
```

`tools/latency_viewer.html` 로 관절별 p50/p95 와 유실을 봄.

---

## 7. 되돌리기

**펌웨어는 언제든 되돌릴 수 있음.** DFU 부트로더는 칩 ROM 에 있어서 플래시에 무엇을
쓰든, 쓰다가 USB 가 빠지든 지워지지 않음. 3-2 대로 다시 들어가서 씀.

1. BOOT0 구멍 두 개를 이은 채로 꽂음 (3-2). `lsusb | grep 0483:df11` 로 확인
2. 레포 루트에서 백업을 씀

```bash
cd ~/Desktop/chanhuegit/HUPHY

# 오른다리
sudo dfu-util -d 0483:df11 -a 0 -s 0x08000000:leave -D firmware/canable/backup_right.bin

# 왼다리
sudo dfu-util -d 0483:df11 -a 0 -s 0x08000000:leave -D firmware/canable/backup_left.bin
```

`:leave` 는 쓰고 나서 바로 새 펌웨어로 재시작시킴. 안 되면 뽑았다가 **BOOT0 를 잇지
않고** 다시 꽂음.

3. 확인 -- 원래대로 돌아왔으면 slcan 어댑터로 잡힘

```bash
lsusb | grep ad50:60c4      # Protofusion Labs CANable ... canable-fw.git
ls -l /dev/canable0 /dev/canable1
```

백업이 없으면 canable.io 업데이트 페이지에서 slcan 을 고르거나
[normaldotcom/canable-fw](https://github.com/normaldotcom/canable-fw) 를 씀.

되돌린 뒤에는 6절 "전" 방법으로 채널을 올림.

**되돌릴 수 없는 것**은 하드웨어 손상뿐임 -- BOOT0 두 구멍 말고 다른 패드를 잇지 말
것. 전원이 꺼진 상태에서 이음.

---

## 참고

- [candleLight_fw](https://github.com/candle-usb/candleLight_fw) -- 펌웨어 원본, 지원 칩, dfu-util 명령
- [canable.io 업데이트 페이지](https://canable.io/updater/canable1.html) -- 브라우저로 쓰기
- [normaldotcom/canable-fw](https://github.com/normaldotcom/canable-fw) -- 지금 들어 있는 slcan 펌웨어
- [VantTec — Jetson 에서 커널 모듈 따로 빌드](https://vanttec-documentation.readthedocs.io/en/latest/troubleshooting/jetson_oot_driver.html)
- [NVIDIA 포럼 — JetPack 6 에 gs_usb 가 없음](https://forums.developer.nvidia.com/t/missing-gs-usb-kernel-module-for-jetpack-6/275287)
