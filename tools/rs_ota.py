"""RobStride 펌웨어 업데이트 (OTA) -- private 프로토콜, 29비트 확장 프레임.

RobStride 의 "OTA Agreement Description" 문서와 공식 OTA.py 를 따름
(github.com/RobStride/Product_Information/tree/main/OTA). 펌웨어 파일은 같은 저장소의
Releases 에 있음. 절차 전체는 docs/motor_setup.md 15절.

**모터가 private 프로토콜이어야 함.** MIT 모드에서는 이 프레임들에 아예 답하지 않음.

    python3 tools/rs_ota.py --channel canr --motor 8 --info              # UID·버전만
    python3 tools/rs_ota.py --channel canr --motor 8 --bin rs04_0.4.1.38.bin

**모델 확인을 함.** 모델이 다른 펌웨어 파일도 앞부분이 똑같아서 파일만으로는 구분이
안 되고, 모터가 거부한다는 보장도 없음. 그래서 쓰기 전에 지금 버전을 읽어 두 번째
자리(0.3.x = RS03, 0.4.x = RS04)가 파일 이름의 모델과 다르면 멈춤.

2026-10-27 RS03 6대, RS04 1대에 썼음. 한 대에 약 10초, 재전송 0.
"""

import argparse
import sys
import time

import can

ANNOUNCE, FEEDBACK, STOP = 0, 2, 4
OTA_START, OTA_INFO, OTA_ING, OTA_END = 11, 12, 13, 14


def ext_id(mode, data, dev):
    return ((mode & 0x1F) << 24) | ((data & 0xFFFF) << 8) | (dev & 0xFF)


def split(cid):
    return (cid >> 24) & 0x1F, (cid >> 8) & 0xFFFF, cid & 0xFF


class Ota:
    def __init__(self, channel, motor, host):
        self.bus = can.interface.Bus(channel=channel, interface="socketcan")
        self.motor, self.host = motor, host

    def flush(self):
        while self.bus.recv(timeout=0) is not None:
            pass

    def send(self, mode, data, payload):
        self.bus.send(can.Message(arbitration_id=ext_id(mode, data, self.motor),
                                  data=bytes(payload), is_extended_id=True))

    def wait(self, pred, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rx = self.bus.recv(timeout=max(0.0, end - time.monotonic()))
            if rx is not None and rx.is_extended_id and pred(rx):
                return rx
        return None

    def uid(self):
        self.flush()
        self.send(ANNOUNCE, self.host, [0] * 8)
        rx = self.wait(lambda r: split(r.arbitration_id)[0] == ANNOUNCE
                       and split(r.arbitration_id)[2] == 0xFE
                       and (split(r.arbitration_id)[1] & 0xFF) == self.motor, 1.0)
        return None if rx is None else bytes(rx.data)

    def version(self):
        self.flush()
        self.send(STOP, self.host, [0x00, 0xC4, 0, 0, 0, 0, 0, 0])
        rx = self.wait(lambda r: split(r.arbitration_id)[0] == FEEDBACK
                       and r.data[0] == 0x00 and r.data[1] == 0xC4, 1.0)
        if rx is None:
            return None
        return ".".join(str(b) for b in rx.data[3:7]), bytes(rx.data).hex(" ")

    def step(self, mode, data, payload, timeout=0.5, retries=3):
        """Send one frame, wait for its ack. Returns (ok, rx)."""
        modes = (mode, OTA_START) if mode == OTA_ING else (mode,)
        for _ in range(retries):
            self.send(mode, data, payload)
            rx = self.wait(lambda r: split(r.arbitration_id)[0] in modes
                           and split(r.arbitration_id)[2] == self.host
                           and (split(r.arbitration_id)[1] & 0xFF) == self.motor, timeout)
            if rx is None:
                continue
            result = (split(rx.arbitration_id)[1] >> 8) & 0xFF
            return result == 0x00, rx
        return False, None

    def flash(self, blob):
        size = len(blob)
        if not 0 < size <= 0x80000:
            raise SystemExit(f"bad bin size {size}")
        packs = (size + 7) // 8
        uid = self.uid()
        if uid is None:
            raise SystemExit("no device-ID reply (is the motor in private protocol?)")
        print(f"uid {uid.hex(' ')}  size {size}  packs {packs}")

        self.flush()
        ok, _ = self.step(OTA_START, self.host, uid, timeout=2.0)
        if not ok:
            raise SystemExit("START failed")
        print("entered upgrade mode")
        ok, _ = self.step(OTA_INFO, self.host,
                          list(size.to_bytes(4, "little")) + list(packs.to_bytes(4, "little")),
                          timeout=2.0)
        if not ok:
            raise SystemExit("INFO failed")

        cnt, t0, resends = 0, time.monotonic(), 0
        while cnt < packs:
            chunk = blob[cnt * 8:cnt * 8 + 8]
            ok, rx = self.step(OTA_ING, cnt, list(chunk) + [0xFF] * (8 - len(chunk)))
            if ok:
                cnt += 1
                if cnt % 1000 == 0 or cnt == packs:
                    print(f"  {cnt}/{packs}  {time.monotonic() - t0:.1f}s", flush=True)
            elif rx is not None:
                resends += 1
                cnt = int.from_bytes(rx.data[0:2], "little")
                if resends > 200:
                    raise SystemExit(f"too many resend requests (at {cnt})")
            else:
                raise SystemExit(f"no ack at pack {cnt}/{packs} -- power-cycle and retry")

        ok, rx = self.step(OTA_END, 0, list(packs.to_bytes(4, "little")) + [0] * 4, timeout=3.0)
        if not ok:
            raise SystemExit(f"END failed {rx}")
        print(f"upgrade OK in {time.monotonic() - t0:.1f}s (resends {resends})")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--channel", required=True)
    p.add_argument("--motor", type=lambda s: int(s, 0), required=True)
    p.add_argument("--host", type=lambda s: int(s, 0), default=0xFD)
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument("--info", action="store_true")
    g.add_argument("--bin")
    a = p.parse_args()
    ota = Ota(a.channel, a.motor, a.host)
    try:
        if a.info:
            uid = ota.uid()
            print("uid", uid.hex(" ") if uid else None)
            print("version", ota.version())
            return 0 if uid else 1
        # Model guard: rs03_*.bin must go to a motor whose version is 0.3.x, etc.
        # The .bin headers of different models are identical, so check by version.
        import os
        import re
        m = re.match(r"rs0?(\d+)[-_]", os.path.basename(a.bin).lower())
        if not m:
            raise SystemExit(f"cannot tell model from file name {a.bin}")
        ver = ota.version()
        if ver is None:
            raise SystemExit("no version reply -- refusing to flash")
        cur_model = int(ver[0].split(".")[1])
        if cur_model != int(m.group(1)):
            raise SystemExit(f"model mismatch: motor runs {ver[0]} (RS0{cur_model}), "
                             f"file is RS0{m.group(1)} -- refusing to flash")
        print(f"current {ver[0]} -> {os.path.basename(a.bin)}")
        with open(a.bin, "rb") as f:
            ota.flash(f.read())
        return 0
    finally:
        ota.bus.shutdown()


if __name__ == "__main__":
    sys.exit(main())
