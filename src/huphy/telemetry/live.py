"""실시간 추종 뷰어 -- 제어하는 동안 **관절 목표각과 실측각**을 브라우저로 봄.

    huphy-test --robot --hz 50 zero --live          http://<로봇 IP>:8770
    huphy-run  --robot --policy stand ... --live 8771

`track_log` 와 같은 줄(`t`, `tgt/<관절>`, `pos/<관절>`, `imu/<이름>/<값>`)을 파일 대신 메모리 고리에
쌓고 (목표는 가드를 거쳐 **실제로 나간 값** -- `track_log` 참조), 같은 프로세스의 작은 HTTP 서버가 내줌. 화면은 `live_viewer.html` (이 파일 옆).


## 제어를 막지 않음

    제어 루프   motion 을 감싼 자리에서 줄 하나를 고리에 넣음 (락 한 번, 수 µs)
    HTTP 스레드 브라우저가 물으면 그 뒤에 쌓인 줄만 꺼내 JSON 으로 줌

브라우저가 없거나 느려도 루프는 모름 -- 고리는 오래된 것부터 밀려남. 기록 중 예외는
세기만 함.

브라우저는 0.1초마다 `/data?since=<번호>` 로 물음. 50Hz 면 한 번에 5줄 남짓이라 GIL
을 잡는 시간이 짧음. 그래도 **주기가 빠듯하면 끄고 돌릴 것** -- 끝 집계의 밀림을 볼 것.


## 녹화

화면의 **녹화** 를 누른 때부터 **정지** 까지의 줄을 따로 모아 CSV 로 씀. 형식은
`track_log` 와 같아서 `tools/policy_viewer.html` 로 열림.

    <폴더>/track/<월일>_track_live_<이름>.json   폴더는 --log 를 줬으면 거기, 아니면 logs/

고리(60초)와 따로 모으므로 길게 녹화해도 잘리지 않음. 파일은 HTTP 스레드가 씀 --
제어 루프는 줄을 하나 더 붙일 뿐임. 녹화 중에 실행이 끝나면 거기까지 저장함.


## 열리는 주소

`0.0.0.0` 에 열어서 같은 망의 노트북에서도 볼 수 있음. 바깥에서 할 수 있는 것은
보기와 녹화뿐임 -- 제어에는 닿지 않음.
"""

from __future__ import annotations

import json
import logging
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional
from urllib.parse import parse_qs, urlparse

logger = logging.getLogger(__name__)

DEFAULT_PORT = 8770
KEEP_S = 60.0
"""고리에 남기는 시간. 브라우저가 처음 붙으면 이만큼을 한 번에 받음."""

PAGE = Path(__file__).with_name("live_viewer.html")


class LiveFeed:
    """매 주기 목표·실측 한 줄을 고리에 쌓음. 관절 순서는 첫 목표의 키 순서."""

    def __init__(
        self, *, hz: float, title: str = "", name: str = "live",
        save_dir: "str | Path" = "logs", robot: Any = None,
    ) -> None:
        self.hz = float(hz)
        self.robot = robot                   # 있으면 목표를 실제로 나간 값으로 (track_log)
        from .track_log import joint_limits    # 순환 import 를 피함

        self.limits = joint_limits(robot)
        """관절 가동범위. 화면이 관절 그래프를 정규화하는 데 씀. 로봇이 없으면 빔."""
        self.title = title
        self.name = name
        self.save_dir = Path(save_dir)
        self.order: Optional[tuple] = None   # 관찰만 받을 때(watch)는 미리 줌
        self.imu_cols: Optional[tuple] = None
        self.has_tgt = True                   # 첫 줄에서 정해짐. 관찰만 받으면 False
        self.errors = 0
        self._rows: deque = deque(maxlen=max(100, int(KEEP_S * self.hz)))
        self._seq = 0                         # 지금까지 넣은 줄 수. 마지막 줄의 번호
        self._lock = threading.Lock()
        self._t0: Optional[float] = None
        self._rec: Optional[List[list]] = None   # 녹화 중이면 모으는 곳
        self._rec_since = 0.0
        self.saved: List[str] = []           # 이번 실행에서 저장한 파일
        self.save_error = ""

    # ---- 제어 루프 쪽 -----------------------------------------------------
    def wrap_motion(self, motion: Callable[[float, Dict[str, Any]], Optional[Dict[str, float]]]):
        def logged(t: float, observation: Dict[str, Any]):
            targets = motion(t, observation)
            self.sample(observation, targets or {})
            return targets

        return logged

    def sample(
        self, observation: Dict[str, Any], targets: Optional[Dict[str, float]] = None,
        *, sent: Optional[Dict[str, float]] = None,
    ) -> None:
        """한 줄. `targets` 가 None 이면 관찰만 (`ObservationTap` 이 부름). `sent` 는
        `track_log.TrackLog.sample` 과 같음. 예외는 세기만."""
        try:
            self._record(observation, targets, sent)
        except Exception as e:
            self.errors += 1
            if self.errors == 1:
                logger.warning("실시간 뷰어 기록 실패 (제어는 계속함): %s", e)

    def _record(self, observation, targets, sent=None) -> None:
        if targets is not None and self.order is None and targets:
            self.order = tuple(targets)
        if not self.order:
            return
        from .track_log import build_row       # 순환 import 를 피함

        row, imu_cols = build_row(self.robot, self.order, observation, targets, self.imu_cols, sent)
        if row is None:
            return
        if self._t0 is None:
            self.has_tgt = targets is not None or sent is not None
            self._t0 = time.monotonic()
        self.imu_cols = imu_cols
        row = [round(time.monotonic() - self._t0, 4)] + [_num(v) for v in row[1:]]
        with self._lock:
            self._seq += 1
            self._rows.append((self._seq, row))
            if self._rec is not None:
                self._rec.append(row)

    def columns(self) -> List[str]:
        from .track_log import columns       # 순환 import 를 피함

        return list(columns(self.order, self.imu_cols, self.has_tgt))

    # ---- 녹화 -------------------------------------------------------------
    def start_recording(self) -> None:
        with self._lock:
            if self._rec is None:
                self._rec = []
                self._rec_since = time.monotonic()

    def stop_recording(self) -> Optional[Path]:
        """모은 줄을 CSV 로 씀. 녹화 중이 아니거나 모은 줄이 없으면 None."""
        with self._lock:
            rows, self._rec = self._rec, None
            order = self.order or ()
        if not rows:
            return None
        from . import track_log                # 순환 import 를 피함

        log = track_log.TrackLog(order)
        log.limits = self.limits
        log.imu_cols = self.imu_cols
        log.has_tgt = self.has_tgt
        t0 = rows[0][0]
        nan = float("nan")
        log.rows = [[r[0] - t0] + [nan if v is None else v for v in r[1:]] for r in rows]
        # 겹치면 `_2` 를 붙이는 것은 `track_log.default_path` 가 함
        # (`telemetry/run_name.py` 의 `path`).
        path = track_log.default_path(self.save_dir, f"live_{self.name}")
        try:
            path = log.save(path)
        except OSError as e:
            self.save_error = str(e)
            logger.warning("녹화 저장 실패: %s", e)
            return None
        self.saved.append(str(path))
        self.save_error = ""
        return path

    def recording(self) -> Dict[str, Any]:
        with self._lock:
            on = self._rec is not None
            rows = len(self._rec) if on else 0
            since = self._rec_since
        return {
            "on": on,
            "rows": rows,
            "seconds": round(time.monotonic() - since, 1) if on else 0.0,
            "saved": self.saved[-1] if self.saved else "",
            "count": len(self.saved),
            "error": self.save_error,
        }

    # ---- HTTP 쪽 ----------------------------------------------------------
    def since(self, seq: int) -> Dict[str, Any]:
        """`seq` 뒤에 쌓인 줄. 브라우저가 처음이면 0 을 줌 -> 고리 전체."""
        with self._lock:
            rows = [row for s, row in self._rows if s > seq]
            last = self._seq
        return {
            "title": self.title,
            "hz": self.hz,
            "seq": last,
            "cols": self.columns(),
            "limits": self.limits,
            "rows": rows,
            "rec": self.recording(),
        }


def _num(value: Any) -> Optional[float]:
    """JSON 에 NaN 을 못 넣으므로 없는 값은 null."""
    try:
        v = float(value)
    except (TypeError, ValueError):
        return None
    return round(v, 3) if v == v else None


class LiveServer:
    """`LiveFeed` 를 내주는 HTTP 서버. 데몬 스레드에서 돎."""

    def __init__(
        self, feed: LiveFeed, *, port: int = DEFAULT_PORT, host: str = "0.0.0.0",
        studio: Any = None,
    ) -> None:
        self.feed = feed
        self.studio = studio
        """주면 `/api/...` 로 제어 요청을 받음 (`control.studio.Studio`). 화면이 제어 패널을
        켬. **제어가 붙으므로 host 를 0.0.0.0 으로 열 때는 같은 망 누구나 움직일 수 있음.**"""
        self.host = host
        self.port = int(port)
        self._server: Optional[ThreadingHTTPServer] = None
        self._thread: Optional[threading.Thread] = None

    def start(self) -> "LiveServer":
        page = PAGE.read_bytes()
        feed = self.feed
        studio = self.studio

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                url = urlparse(self.path)
                if url.path == "/data":
                    try:
                        seq = int(parse_qs(url.query).get("since", ["0"])[0])
                    except ValueError:
                        seq = 0
                    body = json.dumps(feed.since(seq), separators=(",", ":")).encode()
                    kind = "application/json"
                    if studio is not None:
                        studio.touch()          # 화면이 살아 있음 (끊김 감시)
                elif url.path in ("/", "/index.html"):
                    body, kind = page, "text/html; charset=utf-8"
                elif url.path.startswith("/api/") and studio is not None:
                    self._api_get(url)
                    return
                else:
                    self.send_error(404)
                    return
                self._reply(body, kind)

            def _api_get(self, url):
                query = parse_qs(url.query)
                try:
                    if url.path == "/api/status":
                        out = studio.status()
                    elif url.path == "/api/layout":
                        out = studio.layout()
                    elif url.path == "/api/sequences":
                        out = {"names": studio.list_sequences()}
                    elif url.path == "/api/sequence":
                        out = studio.load_sequence(query.get("name", [""])[0])
                    else:
                        self.send_error(404)
                        return
                except Exception as e:
                    out = {"error": str(e)}
                self._json(out)

            def _api_post(self, path):
                try:
                    length = int(self.headers.get("Content-Length") or 0)
                    data = json.loads(self.rfile.read(length) or b"{}")
                    if path == "/api/command":
                        studio.submit(data)
                        out = {"ok": True}
                    elif path == "/api/sequence/save":
                        out = {"ok": True, "path": studio.save_sequence(data.get("name", ""), data)}
                    else:
                        self.send_error(404)
                        return
                except Exception as e:
                    out = {"ok": False, "error": str(e)}
                self._json(out)

            def _json(self, out) -> None:
                self._reply(json.dumps(out, ensure_ascii=False).encode(), "application/json")

            def do_POST(self):
                path = urlparse(self.path).path
                if path.startswith("/api/") and studio is not None:
                    self._api_post(path)
                    return
                if path == "/record/start":
                    feed.start_recording()
                    print("\r\n  녹화 시작 (뷰어)\r")
                elif path == "/record/stop":
                    saved = feed.stop_recording()
                    if saved is not None:
                        print(f"\r\n  녹화 저장: {saved}\r")
                else:
                    self.send_error(404)
                    return
                body = json.dumps(feed.recording()).encode()
                self._reply(body, "application/json")

            def _reply(self, body: bytes, kind: str) -> None:
                self.send_response(200)
                self.send_header("Content-Type", kind)
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def log_message(self, *args):     # 요청마다 터미널에 찍지 않음
                pass

        self._server = ThreadingHTTPServer((self.host, self.port), Handler)
        self._server.daemon_threads = True
        self._thread = threading.Thread(
            target=self._server.serve_forever, name="huphy-live", daemon=True
        )
        self._thread.start()
        return self

    def stop(self) -> None:
        """서버를 내림. 녹화 중이었으면 거기까지 저장하고 경로를 찍음."""
        if self.feed.recording()["on"]:
            path = self.feed.stop_recording()
            if path is not None:
                print(f"  녹화 저장 (실행이 끝나 멈춤): {path}")
        if self._server is not None:
            self._server.shutdown()
            self._server.server_close()
            self._server = None

    def urls(self) -> List[str]:
        """화면에 찍을 주소. 0.0.0.0 이면 이 기계의 IP 들로."""
        if self.host not in ("0.0.0.0", ""):
            return [f"http://{self.host}:{self.port}"]
        out = [f"http://127.0.0.1:{self.port}"]
        out += [f"http://{ip}:{self.port}" for ip in _local_ips()]
        return out


def _local_ips() -> List[str]:
    """밖으로 나가는 쪽 IP. 망이 없으면 빈 목록. 실제로 패킷을 보내지 않음."""
    import socket

    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("10.255.255.255", 1))
            ip = s.getsockname()[0]
    except OSError:
        return []
    return [] if ip.startswith("127.") else [ip]


def start(
    feed: LiveFeed, port: int, *, studio: Any = None, host: str = "0.0.0.0",
) -> Optional[LiveServer]:
    """서버를 띄우고 주소를 찍음. 포트가 막혀 있으면 경고만 하고 None -- 제어는 계속함."""
    try:
        server = LiveServer(feed, port=port, studio=studio, host=host).start()
    except OSError as e:
        print(f"  실시간 뷰어를 못 띄움 (포트 {port}): {e}. 다른 포트는 --live <번호>")
        return None
    print("  실시간 뷰어  " + "   ".join(server.urls()))
    return server
