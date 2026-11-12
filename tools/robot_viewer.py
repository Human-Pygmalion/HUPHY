"""기록을 3D 로봇으로 재생 -- ROS 안 씀. 브라우저에 띄움.

    python3 tools/robot_viewer.py                                   # http://127.0.0.1:8790 , 화면에서 기록을 고름
    python3 tools/robot_viewer.py logs/policy/1102_policy_huphy_squat.json
    python3 tools/robot_viewer.py --host 0.0.0.0                    # 같은 망의 다른 기계에서도

`huphy-run --log`, `huphy-test --log`, ROS `log_node` 가 남긴 `logs/policy/`, `logs/track/` 의
JSON 을 읽음. **실측(`pos/`)은 진하게, 목표(`tgt/`)는 반투명**으로 겹쳐 그림 -- 정책이 낸 목표를
실물이 얼마나 따라갔는지 봄.

## 무엇을 씀

    모양     ros/huphy_ros/urdf/huphy.urdf + ros/huphy_ros/meshes/   (RViz 와 같은 모델)
    발목     ros/huphy_ros/huphy_ros/linkage.py -- 크랭크각에서 발 자세·막대 각도를 풂
    그리기   three.js (CDN 에서 받음 -- 브라우저에 인터넷이 있어야 함)

기록에는 모터 12개 각도만 있음 (발목은 크랭크 `ankle_a/b`). 발과 막대는 여기서 URDF 치수로
풀어서 그림. 몸통은 공중에 고정 (몸통 위치·자세는 기록에 없음).

사람이 준 명령(`obs/cmd/`)은 **오른쪽 위**에 그 프레임 값으로 띄움 -- 걷기는 `vx`·`wz`,
스쿼트는 `h_star`. 정책이 정한 이름을 그대로 쓰므로 여기에 이름을 박아 두지 않음.

## 화면

    Space        재생 / 정지
    ← →          한 프레임        Shift+← →   1초
    Home End     처음 / 끝
    마우스       왼쪽 끌기 돌리기, 휠 확대, 오른쪽 끌기 옮기기
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
import sys
import threading
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse
import xml.etree.ElementTree as ET

import numpy as np

REPO = Path(__file__).resolve().parent.parent
PKG = REPO / "ros" / "huphy_ros"
URDF = PKG / "urdf" / "huphy.urdf"
MESHES = PKG / "meshes"
LOG_DIRS = (REPO / "logs", REPO / "ros" / "logs")
FPS = 30.0
"""화면 프레임. 기록은 50~100Hz 지만 다 풀 필요가 없음 -- 발목 풀이가 한 번에 수 ms."""
ROS_SEP = "__"  # URDF 관절 이름 (right_leg__knee) -- huphy_ros/convert.py 와 같음
CMD_PREFIX = "obs/cmd/"
"""사람이 준 명령이 실린 열의 머리. 뒤에 붙는 이름은 정책이 정함."""
CMD_UNITS = {"vx": "m/s", "vy": "m/s", "wz": "rad/s", "h_star": "m"}
"""화면에 붙일 단위. 모르는 이름이면 단위 없이 숫자만 띄움."""


def _load_linkage():
    """`linkage.py` 를 파일로 직접 읽음. huphy_ros 패키지(rclpy)를 안 거침."""
    spec = importlib.util.spec_from_file_location("huphy_linkage", PKG / "huphy_ros" / "linkage.py")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod   # dataclass 가 자기 모듈을 찾음
    spec.loader.exec_module(mod)
    return mod


linkage_mod = _load_linkage()


# ---------------------------------------------------------------------------
# 모델
# ---------------------------------------------------------------------------
def model_json(urdf_text: str) -> dict:
    """브라우저가 그릴 것: 링크마다 visual (메시 또는 원기둥) 과 그 자리."""
    root = ET.fromstring(urdf_text)
    visuals = []
    for link in root.iter("link"):
        for v in link.findall("visual"):
            o = v.find("origin")
            xyz = [float(x) for x in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
            rpy = [float(x) for x in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
            g = v.find("geometry")
            mesh, cyl = g.find("mesh"), g.find("cylinder")
            item = {"link": link.get("name"), "xyz": xyz, "rpy": rpy}
            if mesh is not None:
                item["mesh"] = Path(mesh.get("filename")).name
            elif cyl is not None:
                item["cylinder"] = [float(cyl.get("radius")), float(cyl.get("length"))]
            else:
                continue
            visuals.append(item)
    return {"visuals": visuals, "links": sorted({v["link"] for v in visuals})}


class Kinematics:
    """관절각 -> 링크마다 base_link 기준 (위치, 쿼터니언)."""

    def __init__(self, urdf_text: str, links):
        self.joints = linkage_mod.parse(urdf_text)   # 자식 링크 -> 관절
        # 발판 각도 범위를 안 봄 -- 정책이 낸 목표가 범위를 넘어도 그대로 보여 줌
        self.linkage = linkage_mod.Linkage(urdf_text, ankle_limits=False)
        self.links = list(links)
        self.chains = {l: self.linkage._chain(l) for l in self.links}

    def poses(self, q: dict) -> list:
        q = dict(q)
        q.update(self.linkage.solve(q))
        out = []
        cache = {}
        for link in self.links:
            T = np.eye(4)
            for j in self.chains[link]:
                key = j.child
                if key in cache:
                    T = cache[key]
                    continue
                T = T @ j.origin
                if j.moving:
                    R = np.eye(4)
                    R[:3, :3] = linkage_mod._axis_angle(j.axis, q.get(j.name, 0.0))
                    T = T @ R
                cache[key] = T
            out.extend(round(float(v), 5) for v in (*T[:3, 3], *_quat(T[:3, :3])))
        return out


def _quat(R: np.ndarray):
    """회전행렬 -> (x, y, z, w). three.js 순서."""
    w = math.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return x, y, z, w


# ---------------------------------------------------------------------------
# 기록
# ---------------------------------------------------------------------------
def list_logs():
    out = []
    for d in LOG_DIRS:
        for kind in ("policy", "track"):
            for p in sorted((d / kind).glob("*.json"), reverse=True):
                out.append(str(p.relative_to(REPO)))
    return out


def frames(path: Path, make_kin, progress) -> dict:
    """기록 -> 화면 프레임. `pos/` 와 `tgt/` 를 둘 다 풂 (있으면)."""
    d = json.loads(path.read_text())
    cols = d["columns"]
    rows = d["rows"]
    ti = cols.index("t")

    kin = make_kin()

    def picks(prefix):
        out = []
        for i, c in enumerate(cols):
            if c.startswith(prefix):
                name = c[len(prefix):].replace("/", ROS_SEP)
                if name in {j.name for j in kin.joints.values()}:
                    out.append((i, name))
        return out

    kinds = {k: picks(f"{k}/") for k in ("pos", "tgt")}
    kinds = {k: v for k, v in kinds.items() if v}
    if not kinds:
        raise ValueError("pos/ · tgt/ 관절 열이 없음 -- policy·track 기록이어야 함")

    # 사람이 준 명령. 정책마다 다름 -- 걷기는 vx·wz, 스쿼트는 h_star. 이름을 박지
    # 않고 `obs/cmd/` 뒤에 붙은 것을 그대로 씀. 계약의 단위가 m/s · rad/s · m 라
    # 배율 없이 그대로 보여줌 (deploy_contract.json 의 velocity_command).
    cmd_cols = [(i, c[len(CMD_PREFIX):])
                for i, c in enumerate(cols) if c.startswith(CMD_PREFIX)]

    # 화면 주기로 줄을 고름
    picked, next_t = [], -1.0
    for r in rows:
        t = r[ti]
        if t is None or t < next_t:
            continue
        picked.append(r)
        next_t = t + 1.0 / FPS

    # 실측·목표를 따로 풂 -- 발목 풀이가 직전 해에서 시작하므로 둘이 섞이면 느리고 튐
    kins = {k: make_kin() for k in kinds}
    out = {"t": [], **{k: [] for k in kinds}, "cmd": []}
    for n, r in enumerate(picked):
        out["t"].append(round(float(r[ti]), 4))
        out["cmd"].append([None if r[i] is None else round(float(r[i]), 3)
                           for i, _ in cmd_cols])
        for k, cols_k in kinds.items():
            q = {}
            for i, name in cols_k:
                v = r[i]
                if v is not None and not (isinstance(v, float) and math.isnan(v)):
                    q[name] = math.radians(float(v))
            out[k].append(kins[k].poses(q))
        if n % 30 == 0:
            progress(n / len(picked))
    progress(1.0)
    out["run"] = d.get("run", {})
    worst = max(k.linkage.worst_m for k in kins.values())
    # inf·nan 은 JSON 이 아님 -- 브라우저의 res.json() 이 터져 화면이 멈춤
    out["worst_mm"] = round(worst * 1000, 3) if math.isfinite(worst) else None
    out["kinds"] = list(kinds)
    out["cmd_names"] = [n for _, n in cmd_cols]
    out["cmd_units"] = [CMD_UNITS.get(n, "") for _, n in cmd_cols]
    return out


# ---------------------------------------------------------------------------
# 서버
# ---------------------------------------------------------------------------
class State:
    def __init__(self):
        self.urdf = URDF.read_text()
        self.model = model_json(self.urdf)
        self.lock = threading.Lock()
        self.cache = {}       # 기록 경로 -> 프레임 (JSON 바이트)
        self.progress = {}    # 기록 경로 -> 0..1
        self.errors = {}

    def load(self, rel: str):
        path = (REPO / rel).resolve()
        if REPO not in path.parents or not path.is_file():
            raise FileNotFoundError(rel)
        with self.lock:
            if rel in self.cache or rel in self.progress:
                return
            self.progress[rel] = 0.0

        def work():
            try:
                data = frames(path, lambda: Kinematics(self.urdf, self.model["links"]),
                              lambda p: self.progress.__setitem__(rel, p))
                body = json.dumps(data, separators=(",", ":")).encode()
                with self.lock:
                    self.cache[rel] = body
                print(f"  {rel}: 프레임 {len(data['t'])}개 준비됨")
            except Exception as e:  # 화면에 띄움
                self.errors[rel] = f"{type(e).__name__}: {e}"
                print(f"  {rel}: {self.errors[rel]}", file=sys.stderr)
            finally:
                self.progress.pop(rel, None)

        threading.Thread(target=work, daemon=True).start()


def make_handler(state: State):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _send(self, code, body: bytes, ctype="application/json"):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            u = urlparse(self.path)
            qs = parse_qs(u.query)
            if u.path == "/":
                self._send(200, PAGE.encode(), "text/html; charset=utf-8")
            elif u.path == "/model":
                self._send(200, json.dumps(state.model).encode())
            elif u.path == "/logs":
                self._send(200, json.dumps(list_logs()).encode())
            elif u.path.startswith("/mesh/"):
                p = MESHES / Path(u.path[6:]).name
                if p.is_file():
                    self._send(200, p.read_bytes(), "application/octet-stream")
                else:
                    self._send(404, b"{}")
            elif u.path == "/frames":
                rel = qs.get("log", [""])[0]
                try:
                    state.load(rel)
                except FileNotFoundError:
                    self._send(404, json.dumps({"error": f"없는 기록: {rel}"}).encode())
                    return
                with state.lock:
                    body = state.cache.get(rel)
                if body is not None:
                    self._send(200, body)
                elif rel in state.errors:
                    self._send(500, json.dumps({"error": state.errors.pop(rel)}).encode())
                else:
                    self._send(202, json.dumps({"progress": state.progress.get(rel, 0.0)}).encode())
            else:
                self._send(404, b"{}")

    return H


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("log", nargs="?", help="처음에 열 기록 (레포 기준 경로). 없으면 화면에서 고름")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8790)
    ap.add_argument("--no-browser", action="store_true", help="브라우저를 자동으로 안 엶")
    a = ap.parse_args()

    state = State()
    first = ""
    if a.log:
        p = Path(a.log).resolve()
        first = str(p.relative_to(REPO)) if REPO in p.parents else a.log
        state.load(first)
    srv = ThreadingHTTPServer((a.host, a.port), make_handler(state))
    url = f"http://{'127.0.0.1' if a.host == '0.0.0.0' else a.host}:{a.port}/"
    if first:
        url += f"#{first}"
    print(f"{url}   Ctrl-C 로 끝")
    if not a.no_browser:
        threading.Timer(0.5, lambda: webbrowser.open(url)).start()
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


PAGE = r"""<!doctype html>
<html lang="ko"><head><meta charset="utf-8"><title>robot_viewer</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
  html,body{margin:0;height:100%;background:#202124;color:#ddd;font:13px system-ui,sans-serif;overflow:hidden}
  #bar{position:absolute;left:0;right:0;bottom:0;padding:8px 12px;background:#000a;display:flex;gap:10px;align-items:center;flex-wrap:wrap}
  #top{position:absolute;left:0;right:0;top:0;padding:8px 12px;background:#000a;display:flex;gap:10px;align-items:center;flex-wrap:wrap}
  #seek{flex:1;min-width:200px}
  select,button{background:#333;color:#ddd;border:1px solid #555;border-radius:4px;padding:3px 8px}
  #msg{color:#fc6}
  /* 명령은 오른쪽 끝. margin-left:auto 가 flex 에서 남은 자리를 다 밀어냄 */
  #cmd{margin-left:auto;font-variant-numeric:tabular-nums;white-space:nowrap}
  #cmd b{color:#9f9;font-weight:600}
  #cmd i{color:#888;font-style:normal}
  label{white-space:nowrap}
  .pos{color:#9cf}.tgt{color:#f9a}
</style>
<script type="importmap">
{"imports":{"three":"https://cdn.jsdelivr.net/npm/three@0.160.0/build/three.module.js",
 "three/addons/":"https://cdn.jsdelivr.net/npm/three@0.160.0/examples/jsm/"}}
</script></head><body>
<div id="top">
  <select id="logs"></select>
  <label class="pos"><input type="checkbox" id="showPos" checked> 실측</label>
  <label class="tgt"><input type="checkbox" id="showTgt" checked> 목표</label>
  <span id="run"></span><span id="msg"></span>
  <span id="cmd"></span>
</div>
<div id="bar">
  <button id="play">▶</button>
  <input id="seek" type="range" min="0" max="0" value="0">
  <span id="time">-</span>
  <select id="speed"><option>0.25</option><option>0.5</option><option selected>1</option><option>2</option><option>4</option></select>x
</div>
<script type="module">
import * as THREE from "three";
import {OrbitControls} from "three/addons/controls/OrbitControls.js";
import {STLLoader} from "three/addons/loaders/STLLoader.js";

const $ = id => document.getElementById(id);
const renderer = new THREE.WebGLRenderer({antialias:true});
renderer.setPixelRatio(devicePixelRatio);
document.body.prepend(renderer.domElement);
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x202124);
THREE.Object3D.DEFAULT_UP.set(0,0,1);
const camera = new THREE.PerspectiveCamera(40, 1, 0.01, 50);
camera.up.set(0,0,1);
camera.position.set(1.3, 1.1, 0.1);
const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(0, 0, -0.45);
controls.update();
scene.add(new THREE.HemisphereLight(0xffffff, 0x444444, 1.6));
const sun = new THREE.DirectionalLight(0xffffff, 1.4); sun.position.set(2,1,3); scene.add(sun);
const grid = new THREE.GridHelper(2, 20, 0x666666, 0x3a3a3a);
grid.rotation.x = Math.PI/2; grid.position.z = -0.9; scene.add(grid);

function resize(){ renderer.setSize(innerWidth, innerHeight); camera.aspect = innerWidth/innerHeight; camera.updateProjectionMatrix(); }
addEventListener("resize", resize); resize();

// ---- 모델: 실측·목표 두 벌 ----
const model = await (await fetch("/model")).json();
const loader = new STLLoader();
const geoms = {};
async function geom(name){
  if (!geoms[name]) geoms[name] = loader.loadAsync("/mesh/" + name);
  return geoms[name];
}
function makeRobot(color, opacity){
  const root = new THREE.Group(); const links = {};
  for (const l of model.links){ const g = new THREE.Group(); links[l] = g; root.add(g); }
  const mat = new THREE.MeshStandardMaterial({color, metalness:0.1, roughness:0.7,
                transparent: opacity < 1, opacity, depthWrite: opacity >= 1});
  const dark = new THREE.MeshStandardMaterial({color:0x222222, transparent: opacity<1, opacity, depthWrite: opacity>=1});
  for (const v of model.visuals){
    const holder = new THREE.Group();
    holder.position.set(...v.xyz);
    holder.rotation.set(v.rpy[0], v.rpy[1], v.rpy[2], "ZYX");
    links[v.link].add(holder);
    if (v.mesh){
      geom(v.mesh).then(g => holder.add(new THREE.Mesh(g, mat)));
    } else {
      const [r, len] = v.cylinder;
      const m = new THREE.Mesh(new THREE.CylinderGeometry(r, r, len, 24), dark);
      m.rotation.x = Math.PI/2;   // three 원기둥은 y 축, URDF 는 z 축
      holder.add(m);
    }
  }
  scene.add(root);
  return {root, links};
}
const robots = {pos: makeRobot(0xd8dde6, 1.0), tgt: makeRobot(0xff7799, 0.35)};

// ---- 기록 ----
let data = null, idx = 0, playing = false, playT = 0, last = 0;
function apply(){
  if (!data) return;
  for (const k of ["pos","tgt"]){
    const r = robots[k], arr = data[k];
    r.root.visible = !!arr && $(k === "pos" ? "showPos" : "showTgt").checked;
    if (!arr) continue;
    const f = arr[idx];
    model.links.forEach((l, i) => {
      const o = i*7, g = r.links[l];
      g.position.set(f[o], f[o+1], f[o+2]);
      g.quaternion.set(f[o+3], f[o+4], f[o+5], f[o+6]);
    });
  }
  // 사람이 준 명령. 이름은 기록이 들고 옴 (걷기 vx·wz, 스쿼트 h_star)
  const c = data.cmd && data.cmd[idx];
  $("cmd").innerHTML = !c ? "" : data.cmd_names.map((nm, i) =>
    c[i] === null ? "" :
    `${nm} <b>${c[i].toFixed(2)}</b> <i>${data.cmd_units[i]}</i>`).filter(Boolean).join("&nbsp;&nbsp; ");
  const t = data.t, n = t.length;
  $("seek").value = idx;
  $("time").textContent = `${(t[idx]-t[0]).toFixed(2)} / ${(t[n-1]-t[0]).toFixed(2)} s`;
}
function seekTo(i){ if (!data) return; idx = Math.max(0, Math.min(data.t.length-1, i)); playT = data.t[idx]; apply(); }
function seekTime(dt){ if (!data) return; const want = data.t[idx] + dt; let i = idx;
  if (dt > 0) while (i < data.t.length-1 && data.t[i] < want) i++; else while (i > 0 && data.t[i] > want) i--; seekTo(i); }

async function open(rel){
  if (!rel) return;
  playing = false; $("play").textContent = "▶"; data = null; $("cmd").textContent = "";
  location.hash = rel; $("logs").value = rel;
  while (true){
    const res = await fetch("/frames?log=" + encodeURIComponent(rel));
    const j = await res.json();
    if (res.status === 202){ $("msg").textContent = `발목 푸는 중 ${Math.round(j.progress*100)}%`; await new Promise(r=>setTimeout(r,300)); continue; }
    if (!res.ok){ $("msg").textContent = j.error; return; }
    data = j; break;
  }
  $("msg").textContent = data.kinds.includes("tgt") ? "" : "목표 열 없음 -- 실측만";
  const run = data.run || {};
  $("run").textContent = [run.policy, run.checkpoint && "ckpt " + run.checkpoint, run.at].filter(Boolean).join("  ·  ");
  $("seek").max = data.t.length - 1;
  seekTo(0);
}

const logs = await (await fetch("/logs")).json();
$("logs").innerHTML = '<option value="">기록 고르기</option>' + logs.map(l => `<option>${l}</option>`).join("");
$("logs").onchange = e => open(e.target.value);
$("seek").oninput = e => seekTo(+e.target.value);
$("showPos").onchange = $("showTgt").onchange = apply;
$("play").onclick = () => { if (!data) return; playing = !playing; $("play").textContent = playing ? "⏸" : "▶";
  if (playing && idx >= data.t.length-1) seekTo(0); playT = data.t[idx]; last = performance.now(); };
addEventListener("keydown", e => {
  if (e.target.tagName === "SELECT") return;
  if (e.code === "Space"){ e.preventDefault(); $("play").click(); }
  else if (e.key === "ArrowRight") e.shiftKey ? seekTime(1) : seekTo(idx+1);
  else if (e.key === "ArrowLeft") e.shiftKey ? seekTime(-1) : seekTo(idx-1);
  else if (e.key === "Home") seekTo(0);
  else if (e.key === "End") data && seekTo(data.t.length-1);
});

function loop(now){
  requestAnimationFrame(loop);
  if (playing && data){
    playT += (now - last) / 1000 * parseFloat($("speed").value);
    while (idx < data.t.length-1 && data.t[idx+1] <= playT) idx++;
    if (idx >= data.t.length-1){ playing = false; $("play").textContent = "▶"; }
    apply();
  }
  last = now;
  controls.update();
  renderer.render(scene, camera);
}
requestAnimationFrame(loop);
if (location.hash.length > 1) open(decodeURIComponent(location.hash.slice(1)));
</script></body></html>
"""

if __name__ == "__main__":
    main()
