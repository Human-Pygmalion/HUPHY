"""기준 모션을 기록 파일 모양으로 바꿈 -- 3D 뷰어로 보려는 것.

    python3 tools/policy_package/reference_to_log.py
    python3 tools/policy_package/reference_to_log.py --policy config/policies/motion545
    python3 tools/robot_viewer.py logs/track/motion545_reference.json

`tools/robot_viewer.py` 는 `logs/policy/`·`logs/track/` 의 JSON 을 읽어 URDF 로봇을
재생함 (`columns` 에 `t` 와 `pos/<관절>`, 단위는 도). 꾸러미의 기준 모션은 npz 라
그대로 못 엶 -- 같은 모양의 JSON 으로 바꿔 둠.

**로봇이 돈 기록이 아님.** 정책이 따라가려는 궤적일 뿐이고 실측이 아님. 그래서
`tgt/` 열을 안 만듦 (뷰어가 `tgt/` 를 반투명 겹치기로 쓰는데 겹칠 것이 없음).
`run.source` 에 `reference` 를 적어 둠.


## 관절 24개를 다 씀

뷰어의 URDF(`ros/huphy_ros/urdf/huphy.urdf`)는 발목 크랭크·로드·발목 2축을 다 들고
있고, **그 기하가 학습 MJCF 와 같음** (크랭크·로드·볼 점 좌표가 0 차이. URDF 가
같은 Fusion 모델의 `_loop` 변형에서 나왔음). 그래서 npz 의 24관절을 그대로 넣으면
뷰어가 학습이 본 자세를 그대로 그림 -- 풀 것이 없음.

모터 12개만 넣어도 뷰어가 `linkage.py` 로 로드 각도를 풀지만, 그때는 발목 2축을
`ankle.py`(실측 링키지) 로 푼 값이 아니라 **기록에 있는 값**을 씀. 기준 모션에는
MJCF 값이 들어 있으니 그것을 그대로 주는 쪽이 정확함.


## 몸통은 공중에 고정됨

뷰어는 몸통 위치·자세를 안 그림 (기록에 없음). 이 궤적은 몸통이 1.03m 에서 0.55m
로 내려가는 동작인데 **화면에서는 다리만 움직임.** npz 의 `qpos` 에 루트 자유관절이
있지만 꾸러미에 안 넣었고 뷰어도 받을 자리가 없음.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from huphy.control import deploy                                 # noqa: E402

MOTOR = {
    "hip_pitch": "hip_pitch", "hip_roll": "hip_roll", "hip_yaw": "hip_yaw",
    "knee": "knee", "crank_A": "ankle_a", "crank_B": "ankle_b",
    "ankle_pitch": "ankle_pitch", "ankle_roll": "ankle_roll",
}
"""학습 쪽 관절 이름 -> 로봇 이름. 뷰어는 `/` 를 `__` 로 바꿔 URDF 이름으로 씀."""

SIDE = {"R": "right_leg", "L": "left_leg"}

MOTORS = ("hip_pitch", "hip_roll", "hip_yaw", "knee", "crank_A", "crank_B")
"""인코더가 있는 관절. `--motors-only` 가 이것만 남김."""


def is_motor(contract_joint: str) -> bool:
    return contract_joint[2:].replace("_joint", "") in MOTORS


def column_name(contract_joint: str) -> str:
    """`R_knee_joint` -> `pos/right_leg/knee`, `R_rod_A_u1` -> `pos/R_rod_A_u1`.

    로드 관절은 URDF 에도 팔다리 이름이 안 붙어 있어 그대로 씀
    (`urdf/make_urdf.py` 의 이름 규칙).
    """
    base = contract_joint[2:].replace("_joint", "")
    if base in MOTOR:
        return f"pos/{SIDE[contract_joint[0]]}/{MOTOR[base]}"
    return f"pos/{contract_joint}"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--policy", type=Path, default=ROOT / "config/policies/motion545")
    ap.add_argument("--out", type=Path, default=None,
                    help="기본 logs/track/<꾸러미>_reference.json")
    ap.add_argument("--motion", type=Path, default=None,
                    help="꾸러미가 아닌 npz 하나를 직접 읽음. 학습 쪽 모션 파일을 "
                         "그대로 볼 때 씀 (joint_pos · joint_names · fps 만 봄)")
    ap.add_argument("--name", default=None, help="--motion 과 함께. 기록 이름")
    ap.add_argument("--motors-only", action="store_true",
                    help="모터 12개만 넣음. 발목 2축·로드 4개는 뷰어가 역산함 "
                         "(ROS 기록과 같은 모양). 역산이 파일 값과 맞는지 보려는 것")
    args = ap.parse_args()

    if args.motion is not None:
        # 계약 없이 읽음 -- 해시·관절 순서를 맞춰 볼 상대가 없음. allow_pickle 은 기본값(False)
        data = np.load(args.motion)
        for key in ("joint_pos", "joint_names"):
            if key not in data:
                raise SystemExit(f"{args.motion.name}: {key} 가 없음")
        motion = SimpleNamespace(
            path=args.motion,
            joint_pos=np.asarray(data["joint_pos"], dtype=np.float64),
            names=[str(n) for n in data["joint_names"]],
            fps=float(np.asarray(data["fps"]).reshape(-1)[0]) if "fps" in data else 50.0,
        )
        motion.frames = int(motion.joint_pos.shape[0])
        motion.duration_s = motion.frames / motion.fps if motion.fps else 0.0
        name = args.name or args.motion.stem
    else:
        policy = deploy.load(args.policy)
        if policy.reference is None:
            raise SystemExit(f"{policy.name}: 기준 모션이 없는 꾸러미임")
        motion = policy.reference
        motion.names = list(motion.names) or [s.name for s in policy.loop_slots]
        name = policy.name
    names = list(motion.names)

    keep = [i for i, n in enumerate(names) if not args.motors_only or is_motor(n)]
    suffix = "_reference_motors" if args.motors_only else "_reference"
    out = args.out or ROOT / "logs" / "track" / f"{name}{suffix}.json"
    out.parent.mkdir(parents=True, exist_ok=True)

    columns = ["t"] + [column_name(names[i]) for i in keep]
    rows = []
    for frame in range(motion.frames):
        row = [round(frame / motion.fps, 4)]
        row += [round(math.degrees(float(motion.joint_pos[frame][i])), 4) for i in keep]
        rows.append(row)

    doc = {
        "run": {
            "at": "",
            "source": "reference",
            "policy": name,
            "what": (f"{name} 의 기준 모션 {motion.path.name} 을 "
                     f"뷰어용으로 바꾼 것. 로봇이 돈 기록이 아님 -- 몸통은 안 그려짐"
                     + (". 모터 12개만 넣음 -- 발목은 뷰어가 역산함"
                        if args.motors_only else "")),
            "hz": motion.fps,
            "frames": motion.frames,
            "robot": "huphy",
        },
        "columns": columns,
        "rows": rows,
    }
    out.write_text(json.dumps(doc, ensure_ascii=False) + "\n", encoding="utf-8")

    jp = np.degrees(motion.joint_pos)
    print(f"{out.relative_to(ROOT)}  ({out.stat().st_size / 1024:.0f} KB)")
    print(f"프레임 {motion.frames}  {motion.fps:g}fps  {motion.duration_s:.2f}초  열 {len(columns)}개")
    print(f"관절 범위 {jp.min():+.1f} ~ {jp.max():+.1f} 도")
    print(f"\n보기:  python3 tools/robot_viewer.py {out.relative_to(ROOT)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
