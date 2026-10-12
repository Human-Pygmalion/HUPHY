"""IMU 가 모델에 들어가는 값(obs 0-5)을 보여주고, 단위와 좌표계를 검사함.

    python3 tools/imu_viewer.py serve [--port 8765] [--host 127.0.0.1]
    python3 tools/imu_viewer.py check [--seconds 45] [--url http://127.0.0.1:8765]

읽기 전용임. 모터는 열지 않고 IMU 포트만 씀. 포트는 한 프로세스만 열 수 있으므로
`serve` 가 떠 있으면 `check` 는 `--url` 로 거기서 값을 받아야 함 (아니면 `serve` 를
끄고 직접 열 것).


## serve

웹에 6개 값을 소수 첫째 자리까지 실시간으로 보여줌.

    roll / pitch / yaw   도. 쿼터니언에서 나온 것 (화면용)
    gyro x / y / z       rad/s. **모델 obs 0-2 칸 그대로**

`stand.observation_vector` 를 그대로 거쳐서 만들므로 모델이 받는 숫자와 같음.
`/data` 로 JSON 도 나가는데 거기에는 중력(`ax ay az`, obs 3-5)도 들어 있음.


## check

IMU 를 손으로 돌리는 동안 기록해서 두 가지를 판정함. 서로 다른 축을 돌려 보되 한
축씩 천천히 90도쯤 돌릴 것.

    크기   obs 의 gyro 와 중력의 변화율이 맞는지. 맞는 배율이 1 이면 rad/s 임.
           0.0175 면 도/초를 또 라디안으로 바꾼 것이고 57 이면 도/초 그대로임.
    부호   각 축 부호를 뒤집어 보며 어느 조합이 중력과 맞는지. **(1,1,1) 이
           아니면 gyro 와 중력이 서로 다른 좌표계에 있는 것임.**

판정은 `g_dot = -w x g` 임 (몸체 좌표에서 본 중력의 변화율). 둘 다 정책이 보는 값이라
어느 쪽이 "진짜" 인지가 아니라 **서로 맞는지** 를 봄. 어긋나면 정책은 실제로는 없는
자세를 보게 됨.
"""
from __future__ import annotations

import argparse
import itertools
import json
import sys
import threading
import time
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from huphy.config import load_robot  # noqa: E402
from huphy.control import stand  # noqa: E402
from huphy.sensors import make_imu  # noqa: E402

PAGE = """<!doctype html>
<html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>IMU</title>
<style>
:root{--bg:#fff;--fg:#111;--mut:#667;--card:#f3f4f6}
@media(prefers-color-scheme:dark){:root{--bg:#111318;--fg:#f2f3f5;--mut:#9aa0ab;--card:#1c1f26}}
body{margin:0;padding:16px;background:var(--bg);color:var(--fg);font:16px system-ui,sans-serif}
h2{margin:18px 0 8px;font-size:14px;color:var(--mut);font-weight:600}
.row{display:grid;grid-template-columns:repeat(3,1fr);gap:10px;max-width:560px}
.c{background:var(--card);border-radius:10px;padding:12px}
.n{font-size:12px;color:var(--mut)}
.v{font-size:32px;font-variant-numeric:tabular-nums;font-weight:600}
#s{margin-top:14px;font-size:12px;color:var(--mut)}
</style></head><body>
<h2>자세 (deg)</h2>
<div class="row">
<div class="c"><div class="n">roll</div><div class="v" id="roll">-</div></div>
<div class="c"><div class="n">pitch</div><div class="v" id="pitch">-</div></div>
<div class="c"><div class="n">yaw</div><div class="v" id="yaw">-</div></div>
</div>
<h2>gyro (rad/s, 모델 obs 0-2)</h2>
<div class="row">
<div class="c"><div class="n">gyro x</div><div class="v" id="gx">-</div></div>
<div class="c"><div class="n">gyro y</div><div class="v" id="gy">-</div></div>
<div class="c"><div class="n">gyro z</div><div class="v" id="gz">-</div></div>
</div>
<div id="s">연결 중…</div>
<script>
const f=v=>{const s=(Math.round(v*10)/10).toFixed(1);return s==="-0.0"?"0.0":s};
const ids=["roll","pitch","yaw","gx","gy","gz"];
async function tick(){
  try{
    const d=await (await fetch("/data",{cache:"no-store"})).json();
    if(d.ok){ids.forEach(k=>document.getElementById(k).textContent=f(d[k]));
      document.getElementById("s").textContent="수신 중 · 지연 "+Math.round(d.age_ms)+" ms";}
    else document.getElementById("s").textContent="IMU 값 대기 중…";
  }catch(e){document.getElementById("s").textContent="서버 연결 끊김";}
  setTimeout(tick,50);
}
tick();
</script></body></html>
"""

FIELDS = ("stamp", "roll", "pitch", "yaw", "gx", "gy", "gz", "ax", "ay", "az")


# ---- 읽기 ----------------------------------------------------------------
def open_imu(config_path: Path):
    robot = load_robot(config_path)
    if not robot.imus:
        raise SystemExit(f"{config_path} 에 imus 가 없음")
    if len(robot.imus) != 1:
        raise SystemExit(f"IMU 가 여럿임: {sorted(robot.imus)}")
    config = next(iter(robot.imus.values()))
    imu = make_imu(config)
    imu.connect()
    return config, imu


class Reader:
    """IMU 를 읽어 **모델이 받는 obs 0-5** 와 화면용 자세를 만듦."""

    def __init__(self, imu) -> None:
        self.imu = imu
        self._zeros = {f"{j}.pos": stand.DEFAULT_POSE_DEG[j] for j in stand.ORDER}
        self._prev = [stand.DEFAULT_POSE_DEG[j] for j in stand.ORDER]
        self._last = [0.0] * len(stand.ORDER)
        self._stamp: Optional[float] = None

    def poll(self) -> Optional[Dict[str, float]]:
        """새 패킷이 있으면 한 줄, 없으면 `None`."""
        s = self.imu.read()
        if not s.is_valid or s.stamp == self._stamp:
            return None
        self._stamp = s.stamp
        # 관절 칸은 안 봄. 모델 입력 경로를 그대로 타려고 같은 함수를 부름.
        obs = stand.observation_vector(self._zeros, s, self._last, self._prev)
        return dict(
            stamp=s.stamp,
            roll=s.extra["roll"], pitch=s.extra["pitch"], yaw=s.extra["yaw"],
            gx=float(obs[0]), gy=float(obs[1]), gz=float(obs[2]),
            ax=float(obs[3]), ay=float(obs[4]), az=float(obs[5]),
        )


# ---- serve ---------------------------------------------------------------
def serve(args) -> int:
    config, imu = open_imu(Path(args.config))
    reader = Reader(imu)
    latest: Dict[str, float] = {"ok": False}
    lock = threading.Lock()

    def loop() -> None:
        while True:
            time.sleep(0.005)
            row = reader.poll()
            if row is not None:
                with lock:
                    latest.clear()
                    latest.update(row, ok=True)

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self) -> None:  # noqa: N802
            if self.path.startswith("/data"):
                with lock:
                    d = dict(latest)
                if d.get("ok"):
                    d["age_ms"] = (time.monotonic() - d["stamp"]) * 1000.0
                body, ctype = json.dumps(d).encode(), "application/json"
            else:
                body, ctype = PAGE.encode(), "text/html; charset=utf-8"
            self.send_response(200)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *a) -> None:
            pass

    threading.Thread(target=loop, daemon=True).start()
    server = ThreadingHTTPServer((args.host, args.port), Handler)
    print(f"IMU {config.name} {config.port} 연결됨 -> http://{args.host}:{args.port}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        imu.disconnect()
    return 0


# ---- check ---------------------------------------------------------------
def record(seconds: float, config_path: Path, url: Optional[str]) -> np.ndarray:
    """`FIELDS` 순서의 행렬. 새 패킷만 쌓음."""
    rows: List[List[float]] = []
    imu = None
    reader = None
    last = None
    if url is None:
        _, imu = open_imu(config_path)
        reader = Reader(imu)
    t0 = time.monotonic()
    print(f"{seconds:.0f}초 기록합니다. 한 축씩 천천히 90도쯤 돌리세요.", flush=True)
    try:
        while time.monotonic() - t0 < seconds:
            if reader is not None:
                row = reader.poll()
            else:
                try:
                    d = json.load(urllib.request.urlopen(f"{url}/data", timeout=1))
                except OSError:
                    d = {}
                row = d if d.get("ok") and d["stamp"] != last else None
                if row is not None:
                    last = d["stamp"]
            if row is not None:
                rows.append([row[k] for k in FIELDS])
            time.sleep(0.004)
    finally:
        if imu is not None:
            imu.disconnect()
    return np.array(rows)


def analyze(data: np.ndarray, *, moved: float = 0.1) -> Dict[str, object]:
    """`g_dot = -w x g` 로 크기와 부호를 판정함.

    반환: 부호 조합별 (상대오차, 상관) 와 최적 조합의 배율 등. 움직임이 모자라면
    `enough` 가 거짓임.
    """
    t = data[:, 0] - data[0, 0]
    grid = np.arange(0.0, t[-1], 0.05)
    g = np.stack([np.interp(grid, t, data[:, 7 + i]) for i in range(3)], 1)
    w = np.stack([np.interp(grid, t, data[:, 4 + i]) for i in range(3)], 1)
    gdot = np.gradient(g, grid, axis=0)
    mask = np.linalg.norm(gdot, axis=1) > moved
    out: Dict[str, object] = {"samples": len(data), "seconds": float(t[-1]), "moving": int(mask.sum())}
    if mask.sum() < 10:
        out["enough"] = False
        return out
    out["enough"] = True
    results = []
    for signs in itertools.product((1, -1), repeat=3):
        pred = -np.cross(w * np.array(signs), g)
        p, q = pred[mask], gdot[mask]
        scale = float(np.sum(p * q) / np.sum(p * p))        # gdot ~ scale * pred
        err = float(np.linalg.norm(scale * p - q) / np.linalg.norm(q))
        if scale <= 0:
            # 세 축을 다 뒤집은 해는 배율이 음수로 같은 오차가 나옴. 배율은 단위
            # 변환이라 양수여야 하므로 후보에서 뺌.
            err = float("inf")
        corr = float(np.corrcoef(p.ravel(), q.ravel())[0, 1])
        results.append((err, corr, signs, scale))
    results.sort()
    out["results"] = results
    out["best"] = results[0]
    out["identity"] = next(r for r in results if r[2] == (1, 1, 1))
    # 각 축이 충분히 돌았는지 (한 축만 돌렸으면 나머지 축 부호는 못 정함)
    out["axis_motion"] = np.abs(w).max(axis=0).tolist()
    return out


def report(res: Dict[str, object]) -> None:
    print(f"\n샘플 {res['samples']}개 / {res['seconds']:.1f}s / 움직인 구간 {res['moving']}개")
    if not res["enough"]:
        print("움직임이 모자라 판정 못 함. 더 크게, 한 축씩 돌려서 다시 할 것.")
        return
    best, ident = res["best"], res["identity"]
    err, corr, signs, scale = best
    print("\n부호 조합 (x, y, z)   상대오차   상관")
    for e, c, s, _ in res["results"][:3]:
        print(f"  {str(s):14s}     {e:5.2f}    {c:+.2f}")
    print(f"  현재 (1, 1, 1)         {ident[0]:5.2f}    {ident[1]:+.2f}")

    print("\n판정")
    if err > 0.35:
        print("  맞는 조합이 없음. 한 축씩 더 크게 돌려 보거나, 센서 출력이 이상한지 볼 것.")
        return
    if 0.8 < scale < 1.25:
        print(f"  크기: OK. gyro 가 rad/s 임 (배율 {scale:.2f}).")
    elif scale < 0.05:
        print(f"  크기: gyro 가 너무 큼 (배율 {scale:.4f}). 도/초가 그대로 들어가고 있음.")
    elif scale > 20:
        print(f"  크기: gyro 가 너무 작음 (배율 {scale:.1f}). 라디안이 한 번 더 변환됨.")
    else:
        print(f"  크기: 배율 {scale:.2f} 로 어긋남. 단위가 아니라 센서 설정을 의심할 것.")
    if signs == (1, 1, 1):
        print("  부호: OK. gyro 와 중력이 같은 좌표계에 있음.")
    else:
        flip = [n for n, s in zip("xyz", signs) if s == -1]
        print(f"  부호: ** {', '.join(flip)} 축이 중력과 반대임 ** -- gyro 에서 이 축의 부호를 뒤집어야 맞음.")
        print("        지금 상태로 정책에 넣으면 gyro 와 중력이 서로 모순됨.")
        weak = [n for n, m in zip("xyz", res["axis_motion"]) if m < 0.3]
        if weak:
            print(f"        주의: {', '.join(weak)} 축은 거의 안 돌아서 부호가 덜 확실함.")


def check(args) -> int:
    data = record(args.seconds, Path(args.config), args.url)
    if len(data) < 20:
        print("받은 샘플이 너무 적음. IMU 연결을 확인할 것.")
        return 1
    report(analyze(data))
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=str(ROOT / "config" / "robot.yaml"))
    sub = ap.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("serve", help="웹으로 6개 값 표시")
    s.add_argument("--port", type=int, default=8765)
    s.add_argument("--host", default="127.0.0.1")
    c = sub.add_parser("check", help="돌려 보며 단위·부호 판정")
    c.add_argument("--seconds", type=float, default=45.0)
    c.add_argument("--url", default=None, help="떠 있는 serve 의 주소. 없으면 IMU 를 직접 엶")
    args = ap.parse_args(argv)
    return serve(args) if args.cmd == "serve" else check(args)


if __name__ == "__main__":
    raise SystemExit(main())
