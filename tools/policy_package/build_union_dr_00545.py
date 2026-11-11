"""`union_dr_00545` 꾸러미를 릴리스에서 만듦. **한 번 돌리고 결과를 커밋함.**

    python3 tools/policy_package/build_union_dr_00545.py --release <릴리스 폴더> \
        [--out config/policies/union_dr_00545]

릴리스 폴더는 HuphyChan-dev 커밋 `ee7770d7` 의
`mjlab_policy/checkpoints/huphy_00545_uniondr_30000` 임. 다음 네 파일을 읽음.

    deploy.yaml                              계약 원본
    env.yaml                                 24칸 기본자세 (init_state.joint_pos)
    model_30000.pt                           가중치. 그대로 복사함
    policy.onnx                              참조 입출력을 만드는 데 씀
    motions/00545_interactionmesh_50hz.npz   기준 모션. 쓰는 배열만 추려 복사함

내는 것:

    deploy_contract.json   이 레포 실행기가 읽는 계약 (`control/deploy.py`)
    model_30000.pt         릴리스 그대로
    motion_00545.npz       joint_pos / joint_vel / joint_names / fps 만


## 참조 입출력을 ONNX 에서 만드는 이유

`.pt` 를 읽는 쪽(`control/rsl_rl.py`)을 검사하려면 **다른 파일, 다른 코드**로 낸
값과 맞춰야 함. 같은 `.pt` 로 만든 값을 쓰면 읽기 코드가 틀려도 통과함.

릴리스의 `policy.onnx` 는 정규화가 안에 든 actor 임. 이 스크립트는 그 파일의
초기값(가중치)을 protobuf 에서 직접 꺼내 순전파를 따로 구현해 참조값을 만듦
(`onnxruntime` 없이). 두 파일의 가중치가 바이트 단위로 같은지도 같이 확인함.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import struct
import sys
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from huphy.control import rsl_rl                                # noqa: E402

GOLDEN_FRAMES = 16
"""참조 입출력 쌍 수. 기준 모션에서 고르게 뽑은 프레임임."""

MOTION_KEEP = ("joint_pos", "joint_vel", "joint_names", "fps")
"""기준 모션에서 꾸러미로 옮기는 배열. 나머지(body_*, qpos 등)는 실물에서 안 씀."""


# ---------------------------------------------------------------------------
# ONNX 최소 해독 — onnxruntime 없이 초기값만 꺼냄
# ---------------------------------------------------------------------------
def _varint(buf: bytes, i: int) -> Tuple[int, int]:
    r = sh = 0
    while True:
        b = buf[i]
        i += 1
        r |= (b & 0x7F) << sh
        if not b & 0x80:
            return r, i
        sh += 7


def _fields(buf: bytes):
    i = 0
    while i < len(buf):
        key, i = _varint(buf, i)
        fn, wt = key >> 3, key & 7
        if wt == 0:
            v, i = _varint(buf, i)
            yield fn, wt, v
        elif wt == 2:
            n, i = _varint(buf, i)
            yield fn, wt, buf[i:i + n]
            i += n
        elif wt == 5:
            yield fn, wt, buf[i:i + 4]
            i += 4
        elif wt == 1:
            yield fn, wt, buf[i:i + 8]
            i += 8
        else:
            raise ValueError(f"ONNX protobuf wire type {wt}")


def _tensor(buf: bytes) -> Tuple[str, np.ndarray]:
    dims: List[int] = []
    name, raw = "", b""
    for fn, wt, v in _fields(buf):
        if fn == 1 and wt == 0:
            dims.append(v)
        elif fn == 1 and wt == 2:
            j = 0
            while j < len(v):
                x, j = _varint(v, j)
                dims.append(x)
        elif fn == 8:
            name = v.decode()
        elif fn == 9:
            raw = v
    arr = np.frombuffer(raw, dtype=np.float32)
    return name, arr.reshape(dims) if dims else arr


def onnx_initializers(path: Path) -> Dict[str, np.ndarray]:
    graph = next(v for fn, wt, v in _fields(path.read_bytes()) if fn == 7)
    out: Dict[str, np.ndarray] = {}
    for fn, wt, v in _fields(graph):
        if fn == 5:
            n, a = _tensor(v)
            out[n] = a
    return out


def onnx_forward(init: Dict[str, np.ndarray], obs: np.ndarray) -> np.ndarray:
    """`policy.onnx` 그래프를 그대로 따라감: Sub -> Div -> (Gemm, Elu) x3 -> Gemm."""
    x = (obs - init["obs_normalizer._mean"].reshape(-1)) / init["onnx::Div_24"].reshape(-1)
    for n in (0, 2, 4):
        x = init[f"mlp.{n}.weight"] @ x + init[f"mlp.{n}.bias"]
        x = np.where(x > 0.0, x, np.expm1(np.minimum(x, 0.0)))
    return init["mlp.6.weight"] @ x + init["mlp.6.bias"]


# ---------------------------------------------------------------------------
def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for block in iter(lambda: f.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def gravity_and_gyro(qpos: np.ndarray, qvel: np.ndarray) -> Tuple[List[float], List[float]]:
    """루트 자유관절 -> (몸통 프레임 중력방향, 몸통 프레임 각속도).

    MuJoCo 자유관절은 `qpos[3:7]` 이 쿼터니언 (w,x,y,z), `qvel[3:6]` 이 **몸통
    프레임** 각속도임. 중력방향은 세계 (0,0,-1) 을 몸통 프레임으로 돌린 것.
    """
    w, x, y, z = (float(v) for v in qpos[3:7])
    r = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    grav = r.T @ np.array([0.0, 0.0, -1.0])
    return [float(v) for v in grav], [float(v) for v in qvel[3:6]]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--release", required=True, type=Path)
    ap.add_argument("--out", type=Path, default=ROOT / "config/policies/union_dr_00545")
    args = ap.parse_args()

    rel: Path = args.release
    out: Path = args.out
    out.mkdir(parents=True, exist_ok=True)

    deploy_yaml = yaml.safe_load((rel / "deploy.yaml").read_text())
    env_text = (rel / "env.yaml").read_text()
    motion_src = rel / "motions" / "00545_interactionmesh_50hz.npz"
    pt_src = rel / "model_30000.pt"
    onnx_src = rel / "policy.onnx"

    # ---- 24칸 기본자세: env.yaml 의 init_state.joint_pos ----
    # `!!python/name:` 태그가 섞여 있어 전체를 safe_load 할 수 없음. 해당 블록만 뽑음.
    lines = env_text.splitlines()
    start = next(i for i, l in enumerate(lines)
                 if l.strip() == "joint_pos:" and lines[i + 1].lstrip().startswith("L_hip_pitch"))
    block = []
    indent = len(lines[start]) - len(lines[start].lstrip())
    for line in lines[start + 1:]:
        if line.strip() and (len(line) - len(line.lstrip())) <= indent:
            break
        block.append(line.strip())
    full_default = {k.strip(): float(v) for k, v in (b.split(":", 1) for b in block)}

    actor = deploy_yaml["observations"]["actor"]
    full_order = [str(n) for n in actor["terms"][0]["joint_order"]]
    motor_order = [str(n) for n in deploy_yaml["actions"]["joint_order"]]
    motor_default = [float(v) for v in deploy_yaml["actions"]["default_position_rad"]]
    scale = float(deploy_yaml["actions"]["scale_rad_per_unit"])
    for j in full_order:
        full_default.setdefault(j, 0.0)      # env.yaml 이 0 인 관절을 안 적음

    # ---- 기준 모션: 쓰는 배열만 추려 씀 ----
    src = np.load(motion_src)                 # allow_pickle 꺼짐
    names = [str(n) for n in src["joint_names"]]
    if names != full_order:
        raise SystemExit(f"모션 관절 순서가 계약과 다름:\n{names}\n{full_order}")
    motion_path = out / "motion_00545.npz"
    np.savez(motion_path, **{k: src[k] for k in MOTION_KEEP})
    frames = int(src["joint_pos"].shape[0])

    # ---- 가중치: 릴리스 그대로 복사 ----
    pt_path = out / pt_src.name
    pt_path.write_bytes(pt_src.read_bytes())
    if sha256(pt_path) != deploy_yaml["artifacts"]["model_30000.pt"]["sha256"]:
        raise SystemExit("model_30000.pt 의 sha256 이 릴리스 계약과 다름")

    # ---- .pt 와 ONNX 의 가중치가 같은지 ----
    init = onnx_initializers(onnx_src)
    state, archive, root = rsl_rl._read(pt_path)
    for key in ("mlp.0", "mlp.2", "mlp.4", "mlp.6"):
        for part in ("weight", "bias"):
            a = rsl_rl._array(state[f"{key}.{part}"], archive, root)
            if not np.array_equal(a, init[f"{key}.{part}"]):
                raise SystemExit(f"{key}.{part} 가 .pt 와 ONNX 에서 다름")
    std = rsl_rl._array(state["obs_normalizer._std"], archive, root).reshape(-1)
    if not np.array_equal(init["onnx::Div_24"].reshape(-1), std + rsl_rl.NORM_EPS):
        raise SystemExit(
            f"ONNX 의 나눗셈 값이 std + {rsl_rl.NORM_EPS} 와 다름 -- 정규화 식을 다시 볼 것")

    # ---- 참조 입출력: 기준 모션으로 만든 실제 자세 ----
    jp, jv, qpos, qvel = (np.asarray(src[k]) for k in ("joint_pos", "joint_vel", "qpos", "qvel"))
    default24 = np.array([full_default[j] for j in full_order], dtype=np.float32)
    golden_obs, golden_act = [], []
    for i in np.linspace(0, frames - 1, GOLDEN_FRAMES).astype(int):
        grav, gyro = gravity_and_gyro(qpos[i], qvel[i])
        vector = np.concatenate([
            jp[i], jv[i],                   # 기준 프레임 (절대값)
            np.asarray(grav, dtype=np.float32),
            np.asarray(gyro, dtype=np.float32),
            jp[i] - default24,              # 로봇 관절 (기본자세 기준)
            jv[i],
            np.zeros(len(motor_order), dtype=np.float32),
        ]).astype(np.float32)
        golden_obs.append([float(v) for v in vector])
        golden_act.append([float(v) for v in onnx_forward(init, vector)])

    width = len(golden_obs[0])
    if width != int(deploy_yaml["policy"]["actor_input_dim"]):
        raise SystemExit(f"관찰 벡터가 {width}칸인데 계약은 "
                         f"{deploy_yaml['policy']['actor_input_dim']}칸임")

    # ---- 계약 ----
    pd = deploy_yaml["pd_gains"]
    contract = {
        "package": "union_dr_00545",
        "task": "HUPHY 00545 Union-DR 모션 추종 (기준 궤적 659프레임 재생)",
        "what": (
            "Human-Pygmalion/HuphyChan-dev ee7770d7 의 "
            "mjlab_policy/checkpoints/huphy_00545_uniondr_30000 을 이 레포 계약 형식으로 옮긴 것. "
            "tools/policy_package/build_union_dr_00545.py 가 만듦. "
            "actions/observations/pd_gains 는 릴리스 deploy.yaml, 24칸 기본자세는 "
            "env.yaml 의 init_state.joint_pos, golden 은 policy.onnx 의 가중치로 따로 순전파한 값."
        ),
        "control_rate_hz": float(deploy_yaml["policy"]["control_rate_hz"]),
        "obs_dim": width,
        "action_dim": len(motor_order),
        "default_checkpoint": "30000",
        "target_pos_rad": "target = offset_rad + scale_rad_per_unit * action, then clipped to clip_rad",
        "actions": [],
        "default_pose_rad": {},
        "loop_joints": {
            "what": (
                "관찰이 쓰는 관절 24개 순서. 다리마다 6개는 모터가 없고 크랭크각에서 "
                "계산해 냄 (kinematics/mjcf_loop.py)."
            ),
            "order": full_order,
            "default_rad": {j: full_default[j] for j in full_order},
        },
        "motion": {
            "what": "기준 궤적. 한 정책 주기에 한 프레임 나아감. 끝에서 멈춤 (되돌려 반복하지 않음)",
            "file": motion_path.name,
            "frames": frames,
            "fps": float(np.asarray(src["fps"]).reshape(-1)[0]),
            "joint_order": full_order,
            "sha256": sha256(motion_path),
            "source": "motions/00545_interactionmesh_50hz.npz",
            "source_sha256": deploy_yaml["artifacts"][
                "motions/00545_interactionmesh_50hz.npz"]["sha256"],
        },
        "observations": [],
        "pd_gains_nm_per_rad": {
            j: {"kp": float(k), "kd": float(d)}
            for j, k, d in zip(motor_order, pd["stiffness_nm_per_rad"], pd["damping_nm_s_per_rad"])
        },
        "effort_limit_nm": {
            j: float(v) for j, v in zip(motor_order, pd["effort_limit_nm"])
        },
        "checkpoints": {
            "30000": {
                "iteration": int(deploy_yaml["policy"]["iteration"]),
                "pt": pt_path.name,
                "pt_sha256": sha256(pt_path),
                "golden": {"obs": golden_obs, "actions": golden_act},
                "golden_source": "policy.onnx (sha256 " +
                                 deploy_yaml["artifacts"]["policy.onnx"]["sha256"] + ")",
            }
        },
    }

    # 행동 12칸. 자르기 범위는 MJCF 관절 범위 -- 릴리스가 따로 안 줌
    ranges = {
        "hip_pitch": (-1.9198621771937625, 0.5235987755982988),
        "hip_roll": (-0.3490658503988659, 1.3962634015954636),
        "hip_yaw": (-0.7853981633974483, 0.7853981633974483),
        "knee": (0.0, 2.0943951023931953),
        "crank_A": (-1.2, 1.2),
        "crank_B": (-1.2, 1.2),
    }
    for i, (joint, default) in enumerate(zip(motor_order, motor_default)):
        base = joint[2:].replace("_joint", "")
        lo, hi = ranges[base]
        if joint.startswith("L_") and base in ("hip_pitch", "hip_roll", "knee"):
            lo, hi = -hi, -lo
        contract["actions"].append({
            "index": i, "joint": joint, "scale_rad_per_unit": scale,
            "offset_rad": default, "clip_rad": [lo, hi],
        })
        contract["default_pose_rad"][joint] = default
        if abs(full_default[joint] - default) > 1e-9:
            raise SystemExit(f"{joint} 기본자세가 env.yaml 과 deploy.yaml 에서 다름")

    func = {
        "command": "reference_motion",
        "projected_gravity": "imu_gravity",
        "base_ang_vel": "imu_gyro",
        "joint_pos": "loop_joint_pos_rel",
        "joint_vel": "loop_joint_vel",
        "previous_action": "last_action",
    }
    unit = {
        "command": "rad, rad/s", "projected_gravity": "unit vector", "base_ang_vel": "rad/s",
        "joint_pos": "rad", "joint_vel": "rad/s", "previous_action": "none",
    }
    for term in actor["terms"]:
        name = str(term["name"])
        lo, hi = (int(v) for v in term["slice"])
        contract["observations"].append({
            "term": name, "slice": [lo, hi], "width": hi - lo,
            "func": func[name], "unit": unit[name], "history_frames": 0,
        })

    path = out / "deploy_contract.json"
    path.write_text(json.dumps(contract, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"{path}  ({path.stat().st_size / 1024:.0f} KB)")
    print(f"{motion_path}  ({motion_path.stat().st_size / 1024:.0f} KB, {frames} 프레임)")
    print(f"{pt_path}  ({pt_path.stat().st_size / 1024 / 1024:.1f} MB)")
    print(f"참조 입출력 {GOLDEN_FRAMES}쌍, 관찰 {width}칸, 행동 {len(motor_order)}칸")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
