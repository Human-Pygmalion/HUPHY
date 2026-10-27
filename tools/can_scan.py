"""CAN 라인을 훑어 모터마다 호스트 id·상태·전압·전류·zero_sta 를 표로 찍음.

    python3 tools/can_scan.py                 # canl, canr 둘 다. id 1~12 + 0x7F
    python3 tools/can_scan.py canr --ids 7-12

**제어가 돌고 있을 때 쓰지 말 것.** 모터마다 정지(Command 2)를 보내 상태를 받으므로
토크가 켜져 있으면 끊김. 그리고 이 스크립트의 응답이 돌고 있는 루프의 수거에 섞임.

모터 하나에 하는 것 (docs/motor_setup.md 14~15절):

    1  MIT 정지(Command 2)      답하면 MIT. 응답의 CAN id 가 그 모터의 호스트 id
    2  답이 없으면 private Type 0  답하면 아직 private (펌웨어 업데이트 중이었거나 출하 상태)
    3  MIT 이면 Command 14 로 VBUS(0x701C) / iqf(0x701A) / zero_sta(0x7029)
       답이 없으면 펌웨어가 Command 14 이전 판임 -> 업데이트 대상

호스트 id 를 바꾼 직후 첫 Command 14 는 가끔 답이 없음 (실물에서 여러 번 봄). 그래서
한 번 더 물음.
"""

from __future__ import annotations

import argparse
import struct
import sys
import time

import can

HOST_ID_BASE = 0xF0      # 호스트 id 규칙: 0xF0 + 모터 id (docs/motor_setup.md 14절)
FACTORY_ID = 0x7F
PRIVATE_HOST = 0xFD      # private 요청에 실어 보내는 호스트 id

PARAMS = (("vbus", 0x701C), ("iqf", 0x701A), ("zero_sta", 0x7029))


def parse_ids(text: str):
    out = []
    for part in text.split(","):
        if "-" in part:
            a, b = part.split("-")
            out += range(int(a, 0), int(b, 0) + 1)
        else:
            out.append(int(part, 0))
    return out


def recv_until(bus, pred, timeout_s):
    end = time.monotonic() + timeout_s
    while True:
        left = end - time.monotonic()
        if left <= 0:
            return None
        msg = bus.recv(timeout=left)
        if msg is not None and pred(msg):
            return msg


def flush(bus):
    while bus.recv(timeout=0) is not None:
        pass


def mit_status(bus, mid, timeout_s):
    """정지 명령을 보내 상태 프레임을 받음. (호스트 id, 데이터) 또는 None."""
    flush(bus)
    bus.send(can.Message(arbitration_id=mid, data=bytes([0xFF] * 7 + [0xFD]),
                         is_extended_id=False))
    msg = recv_until(bus, lambda m: not m.is_extended_id and len(m.data) == 8
                     and m.data[0] == mid, timeout_s)
    return None if msg is None else (msg.arbitration_id, bytes(msg.data))


def private_alive(bus, mid, timeout_s):
    """private Type 0 에 답하는지. 답하면 아직 private 임."""
    flush(bus)
    bus.send(can.Message(arbitration_id=(PRIVATE_HOST << 8) | mid, data=bytes(8),
                         is_extended_id=True))
    return recv_until(bus, lambda m: m.is_extended_id and (m.arbitration_id & 0xFF) == 0xFE
                      and ((m.arbitration_id >> 8) & 0xFF) == mid, timeout_s) is not None


def read_param(bus, mid, host, index, timeout_s, tries=2):
    """Command 14 로 하나를 읽음. 답이 없으면 None."""
    key = bytes([index & 0xFF, index >> 8, 0, 0])
    for _ in range(tries):
        flush(bus)
        bus.send(can.Message(arbitration_id=(3 << 8) | mid,
                             data=key + bytes(4), is_extended_id=False))
        msg = recv_until(bus, lambda m: not m.is_extended_id and m.arbitration_id == host
                         and bytes(m.data[:4]) == key, timeout_s)
        if msg is not None:
            return bytes(msg.data[4:8])
    return None


def scan(channel, ids, timeout_s):
    rows = []
    try:
        bus = can.interface.Bus(channel=channel, interface="socketcan")
    except OSError as e:
        print(f"{channel}: 열 수 없음 ({e}) -- huphy-can --channel {channel} up")
        return rows
    try:
        for mid in ids:
            st = mit_status(bus, mid, timeout_s)
            if st is None:
                if private_alive(bus, mid, timeout_s):
                    rows.append((channel, mid, "private", None, None, {}))
                continue
            host, d = st
            vals = {}
            for name, index in PARAMS:
                raw = read_param(bus, mid, host, index, timeout_s)
                if raw is None:
                    vals[name] = None
                elif name == "zero_sta":
                    vals[name] = raw[0]
                else:
                    vals[name] = struct.unpack("<f", raw)[0]
            rows.append((channel, mid, "mit", host, d, vals))
    finally:
        bus.shutdown()
    return rows


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("channels", nargs="*", default=["canl", "canr"])
    p.add_argument("--ids", default="1-12", help="훑을 모터 id. 예: 1-12, 7,8,10. 0x7F 는 늘 붙음")
    p.add_argument("--timeout-ms", type=float, default=20.0)
    a = p.parse_args()

    ids = parse_ids(a.ids)
    if FACTORY_ID not in ids:
        ids.append(FACTORY_ID)
    timeout_s = a.timeout_ms / 1000.0

    rows = []
    for ch in a.channels:
        rows += scan(ch, ids, timeout_s)

    print(f"{'ch':5} {'id':>3} {'proto':7} {'host':5} {'mode':5} {'fault':5} {'temp':>6} "
          f"{'VBUS':>7} {'iqf':>6} {'zero':>4}  note")
    bad = 0
    for ch, mid, proto, host, d, vals in rows:
        notes = []
        if proto == "private":
            print(f"{ch:5} {mid:3d} {'private':7}  -- MIT 이 아님. 업데이트 중이었거나 출하 상태")
            bad += 1
            continue
        mode = d[6] >> 6
        fault = bool(d[6] & 0x20)
        temp = ((d[6] & 0x0F) << 8 | d[7]) / 10
        if host != HOST_ID_BASE + mid:
            notes.append(f"호스트 id 가 0x{HOST_ID_BASE + mid:02X} 이어야 함")
        if vals.get("vbus") is None:
            notes.append("Command 14 무응답 -> 펌웨어 업데이트 대상")
        if fault:
            notes.append("고장 비트")
        if notes:
            bad += 1
        fmt = lambda v, f: "-" if v is None else format(v, f)
        print(f"{ch:5} {mid:3d} {proto:7} 0x{host:02X}  {mode:<5} {str(fault):5} {temp:6.1f} "
              f"{fmt(vals.get('vbus'), '7.2f')} {fmt(vals.get('iqf'), '6.2f')} "
              f"{fmt(vals.get('zero_sta'), '4d')}  {'; '.join(notes)}")
    print(f"\n모터 {len(rows)}개" + (f", 확인 필요 {bad}개" if bad else ", 이상 없음"))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
