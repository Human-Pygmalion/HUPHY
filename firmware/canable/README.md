# `firmware/canable/`

USB-CAN 어댑터(EM Tech, `LC_CANABLE_PRO`) 원래 펌웨어 백업. slcan (`canable-fw`).

candleLight 로 바꾸기 전에 뜬 것이고, 되돌릴 때 그대로 씀. 절차는
[docs/candlelight.md](../../docs/candlelight.md) 4-1(백업), 7(되돌리기).

```
backup_right.bin                  오른다리 (canr) 원래 펌웨어. 시리얼 003E00315550530B20303731. STM32F072, 128K 전체
backup_left.bin                   왼다리 (canl) 원래 펌웨어.   시리얼 004100375034570520323530
candlelight_canable_85e5b33.bin   새로 쓰는 candleLight. canable 대상, candle-usb/candleLight_fw 85e5b33
```

candleLight 쓰기 (DFU 모드로 꽂은 뒤, 레포 루트에서):

```bash
sudo dfu-util -d 0483:df11 -a 0 -s 0x08000000:leave -D firmware/canable/candlelight_canable_85e5b33.bin
```

되돌리기 (BOOT0 를 이어 DFU 모드로 꽂은 뒤, 레포 루트에서):

```bash
sudo dfu-util -d 0483:df11 -a 0 -s 0x08000000:leave -D firmware/canable/backup_right.bin
```
