"""패드 입력을 브라우저로 실시간 확인 -- 이 파일 하나로 끝. ROS·런치·노드 안 씀.

    python3 tools/gamepad_viewer.py                       # http://127.0.0.1:8780
    python3 tools/gamepad_viewer.py --host 0.0.0.0        # 같은 망의 다른 기계에서도
    python3 tools/gamepad_viewer.py --device /dev/input/js1

리눅스 조이스틱 장치(`/dev/input/js*`)를 직접 읽음. 파이썬 표준 라이브러리만 씀.
읽기만 함 -- 장치는 여러 프로그램이 같이 열 수 있어서 `gamepad.launch.py` 가 돌고 있어도 됨.

## 이름은 장치가 알려 줌

축·버튼마다 리눅스 입력 코드를 장치에 물어서(`JSIOCGAXMAP`, `JSIOCGBTNMAP`) 이름을 붙임.
그래서 USB 든 블루투스든 따로 맞출 필요가 없음.

    버튼  SOUTH(A)  EAST(B)  NORTH/WEST(X·Y)  TL(LB)  TR(RB)  SELECT(View)  START(Menu)
          MODE(Xbox)  THUMBL/THUMBR(스틱 누름)
    축    X/Y(왼쪽 스틱)  RX/RY 또는 Z/RZ(오른쪽 스틱)  Z/RZ 또는 GAS/BRAKE(트리거)  HAT0X/HAT0Y(십자키)
          Z/RZ 는 드라이버마다 뜻이 다름 -- USB(xpad)는 트리거, 블루투스는 오른쪽 스틱

**여기 번호는 커널 번호임.** ROS `joy_node` 가 내는 `/joy` 번호와는 다를 수 있음 (블루투스에서
커널은 버튼 15개, joy_node 는 16개였음). gamepad_node 의 번호를 정할 때는 이름을 보고 맞출 것.

## 끊겼을 때

블루투스 패드는 안 쓰면 잠들어 장치가 사라짐. 그러면 화면에 "연결 안 됨" 이 뜨고, 패드를
깨우면(Xbox 버튼) 1초 안에 다시 엶.
"""

from __future__ import annotations

import argparse
import array
import fcntl
import glob
import json
import os
import select
import struct
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Dict, List, Optional

# linux/joystick.h 의 ioctl 번호
JSIOCGAXES = 0x80016A11
JSIOCGBUTTONS = 0x80016A12
JSIOCGNAME = lambda n: 0x80006A13 + (n << 16)  # noqa: E731
JSIOCGAXMAP = 0x80406A32
JSIOCGBTNMAP = 0x84006A34
JS_EVENT_BUTTON, JS_EVENT_AXIS, JS_EVENT_INIT = 0x01, 0x02, 0x80

AXIS_NAMES = {
    0x00: "X 왼쪽 가로", 0x01: "Y 왼쪽 세로", 0x02: "Z", 0x03: "RX 오른쪽 가로",
    0x04: "RY 오른쪽 세로", 0x05: "RZ", 0x09: "GAS (RT 오른쪽 트리거)", 0x0A: "BRAKE (LT 왼쪽 트리거)",
    0x10: "HAT0X 십자 가로", 0x11: "HAT0Y 십자 세로",
}
BUTTON_NAMES = {
    # C·Z 는 옛 게임패드 형식의 자리라 Xbox 에는 실제 버튼이 없음 (블루투스에서 칸만 있음).
    # 그래서 블루투스에서는 X 3, Y 4, LB 6, RB 7, View 10 으로 한 칸씩 밀림.
    # 0x133·0x134 는 리눅스에서 BTN_X/BTN_NORTH, BTN_Y/BTN_WEST 로 이름이 둘씩인데, 별칭의
    # 방향(NORTH 위, WEST 왼쪽)은 이 패드와 **반대**임. Xbox 드라이버는 X 버튼(왼쪽)에 BTN_X,
    # Y 버튼(위)에 BTN_Y 를 씀 -- 2026-10-29 실측 3=X(왼쪽), 4=Y(위). 그래서 별칭을 안 씀.
    0x130: "A (아래)", 0x131: "B (오른쪽)", 0x132: "C (빈 칸 -- 버튼 없음)", 0x133: "X (왼쪽)",
    0x134: "Y (위)", 0x135: "Z (빈 칸 -- 버튼 없음)", 0x136: "LB", 0x137: "RB",
    0x138: "LT 버튼 (트리거는 축으로 옴)", 0x139: "RT 버튼 (트리거는 축으로 옴)",
    0x13A: "View", 0x13B: "Menu", 0x13C: "Xbox",
    0x13D: "왼쪽 스틱 누름", 0x13E: "오른쪽 스틱 누름",
    0xA7: "Share", 0x220: "십자 위", 0x221: "십자 아래", 0x222: "십자 왼쪽", 0x223: "십자 오른쪽",
}
ABS_Z, ABS_RX, ABS_RY, ABS_RZ, ABS_GAS, ABS_BRAKE = 0x02, 0x03, 0x04, 0x05, 0x09, 0x0A


def axis_name(code: int, codes: List[int]) -> str:
    """Z/RZ 는 드라이버마다 뜻이 다름 -- USB(xpad)는 트리거, 블루투스는 오른쪽 스틱.
    트리거가 GAS/BRAKE 로 따로 있으면 Z/RZ 를 오른쪽 스틱으로 봄."""
    if code in (ABS_Z, ABS_RZ):
        stick = (ABS_GAS in codes or ABS_BRAKE in codes) and ABS_RX not in codes
        side = "가로" if code == ABS_Z else "세로"
        return f"{'Z' if code == ABS_Z else 'RZ'} {'오른쪽 ' + side if stick else '트리거'}"
    return AXIS_NAMES.get(code, f"0x{code:02X}")


def right_stick(codes: List[int]):
    """오른쪽 스틱 축 코드 (가로, 세로). RX/RY 가 없으면 Z/RZ."""
    if ABS_RX in codes and ABS_RY in codes:
        return ABS_RX, ABS_RY
    return ABS_Z, ABS_RZ


class Pad:
    """장치 하나를 읽는 스레드. 끊기면 1초마다 다시 열어 봄."""

    def __init__(self, device: Optional[str]) -> None:
        self.wanted = device
        self.lock = threading.Lock()
        self.path: Optional[str] = None
        self.name = ""
        self.axis_codes: List[int] = []
        self.button_codes: List[int] = []
        self.axes: List[int] = []
        self.buttons: List[int] = []
        self.changed: Dict[str, float] = {}
        self.last_event = 0.0
        self.error = ""
        threading.Thread(target=self._run, daemon=True).start()

    def _candidates(self) -> List[str]:
        return [self.wanted] if self.wanted else sorted(glob.glob("/dev/input/js*"))

    def _open(self, path: str) -> int:
        fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
        buf = array.array("B", [0])
        fcntl.ioctl(fd, JSIOCGAXES, buf)
        n_axes = buf[0]
        fcntl.ioctl(fd, JSIOCGBUTTONS, buf)
        n_buttons = buf[0]
        name = array.array("B", [0] * 128)
        fcntl.ioctl(fd, JSIOCGNAME(128), name)
        axmap = array.array("B", [0] * 0x40)
        fcntl.ioctl(fd, JSIOCGAXMAP, axmap)
        btnmap = array.array("H", [0] * 0x200)
        fcntl.ioctl(fd, JSIOCGBTNMAP, btnmap)
        with self.lock:
            self.path = path
            self.name = name.tobytes().split(b"\0")[0].decode(errors="replace")
            self.axis_codes = list(axmap[:n_axes])
            self.button_codes = list(btnmap[:n_buttons])
            self.axes = [0] * n_axes
            self.buttons = [0] * n_buttons
            self.changed = {}
            self.error = ""
        return fd

    def _run(self) -> None:
        while True:
            fd = None
            for path in self._candidates():
                try:
                    fd = self._open(path)
                    break
                except OSError as e:
                    with self.lock:
                        self.error = f"{path}: {e.strerror}"
            if fd is None:
                with self.lock:
                    self.path = None
                    if not self.error:
                        self.error = "/dev/input/js* 가 없음 -- 패드가 연결 안 됨"
                time.sleep(1.0)
                continue
            try:
                self._read(fd)
            except OSError as e:                         # 블루투스가 잠들면 여기로 옴
                with self.lock:
                    self.error = f"{self.path}: 끊김 ({e.strerror})"
                    self.path = None
            finally:
                os.close(fd)
            time.sleep(1.0)

    def _read(self, fd: int) -> None:
        while True:
            ready, _, _ = select.select([fd], [], [], 0.5)
            if not ready:
                continue
            data = os.read(fd, 8 * 64)
            if not data:
                raise OSError(0, "장치가 닫힘")
            now = time.monotonic()
            with self.lock:
                for i in range(0, len(data) - 7, 8):
                    _, value, kind, number = struct.unpack("<IhBB", data[i:i + 8])
                    init = bool(kind & JS_EVENT_INIT)
                    kind &= ~JS_EVENT_INIT
                    if kind == JS_EVENT_AXIS and number < len(self.axes):
                        if abs(value - self.axes[number]) > 600 and not init:
                            self.changed[f"a{number}"] = now
                        self.axes[number] = value
                    elif kind == JS_EVENT_BUTTON and number < len(self.buttons):
                        if value != self.buttons[number] and not init:
                            self.changed[f"b{number}"] = now
                        self.buttons[number] = value
                    if not init:
                        self.last_event = now

    def snapshot(self) -> Dict[str, Any]:
        now = time.monotonic()
        with self.lock:
            axes = [round(v / 32767.0, 3) for v in self.axes]
            sticks = {}
            for side, (cx, cy) in {"left": (0x00, 0x01), "right": right_stick(self.axis_codes)}.items():
                if cx in self.axis_codes and cy in self.axis_codes:
                    x = axes[self.axis_codes.index(cx)]
                    y = axes[self.axis_codes.index(cy)]
                    sticks[side] = [x, y]
            return {
                "connected": self.path is not None,
                "path": self.path, "name": self.name, "error": self.error,
                "since_event": None if not self.last_event else round(now - self.last_event, 1),
                "axes": [{"i": i, "code": f"0x{c:02X}", "name": axis_name(c, self.axis_codes),
                          "value": axes[i]} for i, c in enumerate(self.axis_codes)],
                "buttons": [{"i": i, "code": f"0x{c:03X}", "name": BUTTON_NAMES.get(c, f"0x{c:03X}"),
                             "value": self.buttons[i]} for i, c in enumerate(self.button_codes)],
                "recent": {k: round(now - t, 2) for k, t in self.changed.items() if now - t < 1.0},
                "sticks": sticks,
            }


PAGE = """<!doctype html>
<html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>패드 입력</title>
<style>
:root { --bg:#fafafa; --fg:#1d1d1f; --muted:#6e6e73; --card:#fff; --line:#e3e3e8;
        --on:#2f9e44; --hot:#f59f00; --bad:#e03131; --barbg:#ececf0; }
@media (prefers-color-scheme: dark) {
  :root { --bg:#141416; --fg:#ececf0; --muted:#9a9aa3; --card:#1d1d21; --line:#2e2e35;
          --on:#51cf66; --hot:#fcc419; --bad:#ff6b6b; --barbg:#2a2a30; } }
* { box-sizing:border-box; }
body { margin:0; padding:16px; background:var(--bg); color:var(--fg);
       font:14px/1.45 system-ui,-apple-system,"Noto Sans KR",sans-serif; }
h1 { font-size:18px; margin:0 0 4px; } h2 { font-size:14px; margin:0 0 8px; color:var(--muted); }
.grid { display:grid; grid-template-columns:repeat(auto-fit,minmax(300px,1fr)); gap:12px; margin-top:12px; }
.card { background:var(--card); border:1px solid var(--line); border-radius:10px; padding:12px; }
.muted { color:var(--muted); } .bad { color:var(--bad); font-weight:600; } .ok { color:var(--on); font-weight:600; }
table { border-collapse:collapse; width:100%; font-variant-numeric:tabular-nums; }
td { padding:3px 6px; border-bottom:1px solid var(--line); vertical-align:middle; }
td.n { color:var(--muted); width:2.2em; } td.v { text-align:right; width:4.5em; }
tr.hot td { background:color-mix(in srgb, var(--hot) 28%, transparent); }
tr.on td.v { color:var(--on); font-weight:700; }
.bar { position:relative; height:8px; background:var(--barbg); border-radius:4px; min-width:80px; }
.bar i { position:absolute; top:0; bottom:0; background:var(--hot); border-radius:4px; }
.bar b { position:absolute; left:50%; top:-2px; bottom:-2px; width:1px; background:var(--muted); }
.sticks { display:flex; gap:20px; flex-wrap:wrap; }
.stick { width:120px; height:120px; border:1px solid var(--line); border-radius:50%; position:relative; }
.stick i { position:absolute; width:14px; height:14px; border-radius:50%; background:var(--hot); left:53px; top:53px; }
.cap { text-align:center; color:var(--muted); margin-top:4px; font-variant-numeric:tabular-nums; }
</style></head><body>
<h1>패드 입력</h1>
<div id="st" class="muted">연결 중…</div>
<div class="grid">
  <div class="card"><h2>스틱</h2><div class="sticks">
    <div><div class="stick"><i id="ls"></i></div><div class="cap" id="lsv">왼쪽</div></div>
    <div><div class="stick"><i id="rs"></i></div><div class="cap" id="rsv">오른쪽</div></div>
  </div><p class="muted">값은 커널 그대로임: 위로 밀면 세로가 − (ROS joy 는 뒤집어서 +).</p></div>
  <div class="card"><h2>버튼 (번호 · 장치가 알려 준 이름)</h2><table id="buttons"></table></div>
  <div class="card"><h2>축 (번호 · 이름 · −1 ~ +1)</h2><table id="axes"></table></div>
</div>
<script>
"use strict";
const $ = (id) => document.getElementById(id);
const f = (v) => (v >= 0 ? "+" : "") + v.toFixed(2);
function dot(el, cap, v, label) {
  const [x, y] = v || [0, 0];
  el.style.left = (53 + 46 * x) + "px"; el.style.top = (53 + 46 * y) + "px";
  cap.textContent = v ? `${label}  ${f(x)} , ${f(y)}` : `${label}  —`;
}
function bar(v) {
  const left = Math.min(50, 50 + 50 * v), width = Math.abs(50 * v);
  return `<div class="bar"><b></b><i style="left:${left}%;width:${width}%"></i></div>`;
}
async function tick() {
  try {
    const d = await (await fetch("/data", { cache: "no-store" })).json();
    $("st").innerHTML = d.connected
      ? `<span class="ok">${d.name}</span> · ${d.path} · 버튼 ${d.buttons.length}개 · 축 ${d.axes.length}개` +
        (d.since_event != null ? ` · 마지막 입력 ${d.since_event}s 전` : "")
      : `<span class="bad">연결 안 됨</span> -- ${d.error || ""} (Xbox 버튼으로 깨우면 다시 엶)`;
    $("buttons").innerHTML = d.buttons.map(b =>
      `<tr class="${("b" + b.i) in d.recent ? "hot " : ""}${b.value ? "on" : ""}"><td class="n">${b.i}</td>` +
      `<td>${b.name}</td><td class="v">${b.value ? "눌림" : "·"}</td></tr>`).join("") || "<tr><td>—</td></tr>";
    $("axes").innerHTML = d.axes.map(a =>
      `<tr class="${("a" + a.i) in d.recent ? "hot" : ""}"><td class="n">${a.i}</td><td>${a.name}</td>` +
      `<td>${bar(a.value)}</td><td class="v">${f(a.value)}</td></tr>`).join("") || "<tr><td>—</td></tr>";
    dot($("ls"), $("lsv"), d.sticks.left, "왼쪽"); dot($("rs"), $("rsv"), d.sticks.right, "오른쪽");
  } catch (e) { $("st").innerHTML = '<span class="bad">뷰어가 꺼짐</span>'; }
  setTimeout(tick, 80);
}
tick();
</script></body></html>
"""


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--device", help="기본: /dev/input/js* 중 처음 열리는 것")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8780)
    args = ap.parse_args(argv)

    pad = Pad(args.device)

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):  # noqa: N802
            if self.path.startswith("/data"):
                body, kind = json.dumps(pad.snapshot()).encode(), "application/json"
            else:
                body, kind = PAGE.encode(), "text/html; charset=utf-8"
            self.send_response(200)
            self.send_header("Content-Type", kind)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *a):  # 요청마다 찍지 않음
            pass

    server = ThreadingHTTPServer((args.host, args.port), Handler)
    print(f"패드 뷰어 -> http://{args.host}:{args.port}   (읽기만 함. Ctrl-C 로 끝)", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
