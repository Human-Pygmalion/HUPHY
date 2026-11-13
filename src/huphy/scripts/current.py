"""`huphy-current` — 전류계로 로봇 전체 DC 전류를 봄.

    huphy-current                       포트를 찾아 계속 보여 줌
    huphy-current --port /dev/ttyUSB1   포트를 직접
    huphy-current --quiet --seconds 10  10초 동안 조용히 재고 요약만

설정 파일을 안 읽음. 전류계는 `config/robot.yaml` 의 항목이 아니고 **필요할 때만
꽂는 장비**임 (`sensors/wcm1600/`).

종료할 때 요약을 냄. **기준선을 잴 때 쓰는 화면임** -- 로봇을 세워 둔 채(모터는
자세 유지 전류만) 평균을 재 두면, 나중에 로그의 `current/total_a` 에서 그만큼을
빼고 `power/dc_a_lo` 와 비교할 수 있음 (`telemetry/power.py` 4절).
"""

from __future__ import annotations

import argparse
import logging
import time
from typing import List, Optional

from ..sensors.wcm1600 import BAUDRATE, Wcm1600

PRINT_HZ = 10.0
"""화면에 찍는 주기. 장비가 더 빨리 보내도 눈으로 읽을 수 있게 줄임.

값은 버리지 않음 -- 요약(개수·평균·최대)은 스레드가 받은 전부에서 나옴.
"""


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="huphy-current",
        description="전류계(WCM1600)로 로봇 전체 DC 전류를 봄",
    )
    p.add_argument("--port", default=None,
                   help="직렬 포트. 생략하면 USB 직렬 장치를 찾음 "
                        "(여럿이면 멈춤 -- IMU 도 USB 면 직접 줄 것)")
    p.add_argument("--seconds", type=float, default=None, metavar="S",
                   help="이 시간이 지나면 끝냄. 생략하면 Ctrl-C 까지")
    p.add_argument("--quiet", action="store_true",
                   help="줄마다 안 찍고 요약만 냄")
    p.add_argument("--verbose", action="store_true", help="디버그 로그")
    return p


def _summary(samples: List[float], meter: Wcm1600, elapsed: float) -> str:
    if not samples:
        return (f"  프레임 0개 ({elapsed:.1f}초). 버린 줄 {meter.dropped}개 -- "
                f"Continuous 모드인지 확인할 것")
    n = len(samples)
    lines = [
        f"  {elapsed:.1f}초 동안 프레임 {n}개  ({n / elapsed:.1f}/s)"
        + (f"  버린 줄 {meter.dropped}개" if meter.dropped else ""),
        f"  평균 {sum(samples) / n:+.2f} A   최소 {min(samples):+.2f} A   "
        f"최대 {max(samples):+.2f} A",
    ]
    return "\n".join(lines)


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(levelname)s %(name)s: %(message)s",
    )

    meter = Wcm1600(port=args.port)
    try:
        meter.connect()
    except Exception as e:
        print(f"[!] {e}")
        return 2

    print(f"\n  포트 {meter.port}  ({BAUDRATE} 8N1)   "
          f"{'조용히 잼' if args.quiet else '종료: Ctrl-C'}\n")

    samples: List[float] = []
    ac_seen = False
    seen = 0                                    # 마지막으로 요약에 넣은 프레임 수
    t0 = time.monotonic()
    deadline = t0 + args.seconds if args.seconds else None
    next_print = t0
    try:
        while deadline is None or time.monotonic() < deadline:
            time.sleep(1.0 / PRINT_HZ / 2)      # 장비보다 촘촘히 깨어 값을 놓치지 않음
            state = meter.read()
            if not state.is_valid:
                continue
            if meter.frames != seen:
                seen = meter.frames
                samples.append(state.amps)
                ac_seen = ac_seen or state.is_ac
            now = time.monotonic()
            if args.quiet or now < next_print:
                continue
            next_print = now + 1.0 / PRINT_HZ
            kind = "AC" if state.is_ac else "DC"
            print(f"  {now - t0:7.2f}s  {kind}  {state.amps:+7.2f} A  "
                  f"(값 들어온 뒤 {state.age_ms():.0f}ms)")
    except KeyboardInterrupt:
        pass
    finally:
        meter.disconnect()

    print("\n" + _summary(samples, meter, time.monotonic() - t0) + "\n")
    if ac_seen:
        print("  [!] AC 프레임이 있었음. 이 로봇은 DC 이므로 장비 설정을 확인할 것\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
