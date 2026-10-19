"""CAN 채널 관리 진입점 -- 올리고, 내리고, 상태를 봄.

    huphy-can up           채널을 내렸다가 1 Mbps 로 올림 (막힌 송신 큐도 비워짐)
    huphy-can down         채널을 내림
    huphy-can status       어댑터·채널·오류 카운터·막힌 큐·드라이버 오류를 한 번에

채널 이름은 `config/robot.yaml` 의 `channel` 을 읽음 (1.0 이면 canr / canl). 하드웨어
버전을 바꾸면 그대로 따라감 (`--config`).


## sudo

`ip link` 는 root 가 필요함. root 가 아니면 이 명령이 `sudo` 를 붙여
부르고, 암호는 터미널에서 sudo 가 물음. 이 코드는 암호를 받거나 들고 있지 않음.


## 왜 `up` 이 먼저 내리나

gs_usb 어댑터는 버스에서 ACK 를 못 받으면(모터 전원이 꺼짐, 선이 빠짐) 같은 프레임을
계속 재전송하다가 error-passive 가 되고 송신 큐가 막힘 (`No buffer space available`).
그 상태에서는 위의 원인을 고쳐도 큐가 안 풀림 -- 내렸다 올리면 비워짐. 이미 정상인
채널도 내렸다 올려서 손해가 없음.

"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path
from typing import List, Optional, Sequence

from ..config import ConfigError, load_robot
from .commission import CONFIG_NAME, _find_config

CAN_BITRATE = 1_000_000
"""모터가 쓰는 비트레이트. RobStride 는 일반 CAN 1 Mbps 만 받음."""

IP = shutil.which("ip") or "/usr/sbin/ip"

CANDLELIGHT_USB = "1d50:606f"
SLCAN_USB = "ad50:60c4"


# ---------------------------------------------------------------------------
# 부르기
# ---------------------------------------------------------------------------
def _sudo(cmd: Sequence[str]) -> List[str]:
    return list(cmd) if os.geteuid() == 0 else ["sudo", *cmd]


def _run(cmd: Sequence[str], *, root: bool = False, check: bool = True) -> subprocess.CompletedProcess:
    full = _sudo(cmd) if root else list(cmd)
    print("  $ " + " ".join(full))
    result = subprocess.run(full, text=True, capture_output=True)
    if result.returncode != 0 and check:
        msg = (result.stderr or result.stdout).strip()
        raise SystemExit(f"  실패: {msg}")
    return result


def _quiet(cmd: Sequence[str]) -> subprocess.CompletedProcess:
    return subprocess.run(list(cmd), text=True, capture_output=True)


def _exists(channel: str) -> bool:
    return Path(f"/sys/class/net/{channel}").exists()


def _channels(args) -> List[str]:
    path = args.config or _find_config()
    if path is None:
        raise SystemExit(f"{CONFIG_NAME} 을 찾지 못했음. 저장소 안에서 실행하거나 --config 로 지정할 것")
    try:
        robot = load_robot(path)
    except ConfigError as e:
        raise SystemExit(f"{e}") from e
    channels = list(robot.channels)
    if args.channel:
        unknown = [c for c in args.channel if c not in channels]
        if unknown:
            raise SystemExit(f"설정에 없는 채널: {unknown} (있는 것: {channels})")
        channels = list(args.channel)
    return channels


# ---------------------------------------------------------------------------
# 명령
# ---------------------------------------------------------------------------
def cmd_up(args) -> int:
    channels = _channels(args)
    if _quiet(["pgrep", "-x", "slcand"]).returncode == 0:
        print("  slcand 가 떠 있음 -- 같은 이름을 잡고 있을 수 있어 먼저 끔")
        _run(["pkill", "-x", "slcand"], root=True, check=False)
    missing = [c for c in channels if not _exists(c)]
    for c in channels:
        if c in missing:
            continue
        _run([IP, "link", "set", c, "down"], root=True)
        _run([IP, "link", "set", c, "up", "type", "can", "bitrate", str(CAN_BITRATE)], root=True)
    print()
    _print_status(channels)
    if missing:
        print(f"\n  ** 장치가 없음: {', '.join(missing)} -- 어댑터가 꽂혀 있는지, 이름 고정 규칙"
              f"(70-candlelight.rules)에 시리얼이 있는지 볼 것. `huphy-can status` **")
        return 1
    return 0


def cmd_down(args) -> int:
    for c in _channels(args):
        if _exists(c):
            _run([IP, "link", "set", c, "down"], root=True)
        else:
            print(f"  {c}: 장치 없음 (건너뜀)")
    return 0


def cmd_status(args) -> int:
    channels = _channels(args)
    lsusb = _quiet(["lsusb"]).stdout
    n_cl = lsusb.count(CANDLELIGHT_USB)
    n_sl = lsusb.count(SLCAN_USB)
    print(f"  어댑터     candleLight(gs_usb) {n_cl}개" + (f", slcan {n_sl}개" if n_sl else ""))
    driver = "로드됨" if Path("/sys/module/gs_usb").exists() else "** 안 올라옴 (sudo modprobe gs_usb) **"
    print(f"  드라이버   gs_usb {driver}")
    print()
    bad = _print_status(channels)
    log = _quiet(["journalctl", "-k", "--since", "-10min", "--no-pager"]).stdout
    echo = sum(1 for line in log.splitlines() if "gs_usb" in line and ("echo" in line or "BUG" in line))
    if echo:
        print(f"\n  ** 최근 10분 커널 로그에 gs_usb echo 오류 {echo}건 -- 옛 드라이버인지 볼 것 "
              f"(docs/can_communication.md 3절) **")
        bad = True
    return 1 if bad else 0


def _w(text: str, width: int, right: bool = False) -> str:
    """한글은 터미널에서 두 칸이라 폭을 손으로 맞춤."""
    pad = max(0, width - sum(2 if ord(ch) > 0x1100 else 1 for ch in str(text)))
    return (" " * pad + str(text)) if right else (str(text) + " " * pad)


def _print_status(channels: Sequence[str]) -> bool:
    """채널마다 한 줄. 손볼 것이 있으면 참."""
    bad = False
    print("  " + _w("채널", 8) + _w("링크", 7) + _w("CAN 상태", 15) + _w("비트레이트", 11, True)
          + _w("경고", 7, True) + _w("수동", 7, True) + _w("버스오프", 9, True) + _w("막힌 큐", 9, True))
    for c in channels:
        if not _exists(c):
            print(f"  {c:8s} 장치 없음")
            bad = True
            continue
        raw = _quiet([IP, "-details", "-statistics", "-json", "link", "show", c]).stdout
        try:
            d = json.loads(raw)[0]
        except (ValueError, IndexError):
            print(f"  {c:8s} 읽기 실패")
            bad = True
            continue
        info = d.get("linkinfo", {}).get("info_data", {})
        xs = d.get("linkinfo", {}).get("info_xstats", {})
        state = info.get("state", "?")
        bitrate = info.get("bittiming", {}).get("bitrate")
        backlog = _backlog(c)
        link = d.get("operstate", "?")
        print("  " + _w(c, 8) + _w(link, 7) + _w(state, 15) + _w(bitrate or "-", 11, True)
              + _w(xs.get("error_warning", 0), 7, True) + _w(xs.get("error_passive", 0), 7, True)
              + _w(xs.get("bus_off", 0), 9, True) + _w(backlog, 9, True))
        if link == "DOWN" or state in ("STOPPED", "BUS-OFF") or (bitrate and bitrate != CAN_BITRATE):
            bad = True
        if state in ("ERROR-PASSIVE", "BUS-OFF") or backlog:
            print(f"           ** {c}: 버스에서 응답을 못 받음 -- 모터 전원·CAN 선을 보고 `huphy-can up` **")
            bad = True
    if bad:
        print("\n  올라오지 않은 채널이 있으면 `huphy-can up`.")
    return bad


def _backlog(channel: str) -> int:
    out = _quiet(["tc", "-s", "-j", "qdisc", "show", "dev", channel]).stdout
    try:
        return int(sum(q.get("qlen", 0) for q in json.loads(out or "[]")))
    except (ValueError, TypeError):
        return 0


COMMANDS = {"up": cmd_up, "down": cmd_down, "status": cmd_status}


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="huphy-can",
        description="CAN 채널을 올리고 내리고 상태를 봄. 채널은 robot.yaml 에서 읽음.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "예시:\n"
            "  huphy-can up              로봇 돌리기 전에\n"
            "  huphy-can down            끝나고\n"
            "  huphy-can status          통신이 이상할 때 먼저\n"
        ),
    )
    p.add_argument("--config", type=Path, help=f"기본값: 위로 올라가며 {CONFIG_NAME} 을 찾음")
    p.add_argument("--channel", action="append", help="이 채널만 (여러 번 줄 수 있음). 기본은 설정의 전부")
    sub = p.add_subparsers(dest="command", required=True)
    sub.add_parser("up", help="내렸다가 1 Mbps 로 올림")
    sub.add_parser("down", help="내림")
    sub.add_parser("status", help="어댑터·채널·오류 상태")
    return p


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    if not shutil.which("ip") and not Path(IP).exists():
        raise SystemExit("ip 명령이 없음 (iproute2)")
    return COMMANDS[args.command](args)


if __name__ == "__main__":  # pragma: no cover
    sys.exit(main())
