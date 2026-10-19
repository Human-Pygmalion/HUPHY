"""studio 시퀀스를 계산으로 만듦 -- 스쿼트와 제자리 걷기 (정적 보행).

    python3 tools/make_sequences.py            config/sequences/sim_*.json 을 씀
    python3 tools/make_sequences.py --check    쓰지 않고 표만 찍음
    python3 tools/make_sequences.py --preview logs/seq_preview.html
                                               같은 모델로 막대 그림 애니메이션 (브라우저로 열기)

만든 시퀀스는 `huphy-test --robot studio` 의 시퀀스 목록에 뜸. 키프레임마다 `label` 을
붙여 둠 (studio 는 읽을 때 무시함).


## 모델

시상면 2D 다리 + 발판 수평 + 좌우는 골반 기울기로만 봄. 물리 시뮬이 아니라 **기구학**임
-- 관절이 한계 안인지, 발이 어디에 놓이는지, 발판이 수평인지를 계산으로 확인함.
넘어지지 않는지(동역학)는 이 계산이 보장하지 않음. **처음에는 매달거나 잡은 채로**.

    허벅지 = 정강이 = LEG_L          발목 기하 실측(무릎 -> 발목 393mm) 기준, 허벅지는 같다고 가정
    hip, knee, ankle_pitch           굽힘을 양수로 둔 크기 (부호는 아래 `_joint` 가 다리별로 붙임)
    발판 수평                         ankle_pitch = knee - hip
    발 위치 (골반 기준)               x = L sin(hip) - L sin(knee - hip)
                                     z = L cos(hip) + L cos(knee - hip)   (아래로)

검증: `stand.DEFAULT_POSE_DEG` (엉덩이 10, 무릎 20) 의 발목 모터각을 FK 로 풀면 pitch
8.8 도 -- 이 모델의 10 도와 거의 맞음. 즉 기준자세는 "발이 엉덩이 바로 아래, 발판 수평".


## 관절 부호 (다리별)

    오른다리   hip_pitch = -hip    knee = +knee    (robot_v1.0 범위: hip -112~17, knee 6~114)
    왼다리     hip_pitch = +hip    knee = -knee    (거울)
    hip_roll   양 다리 같은 부호가 같은 쪽으로 기움 -- 오른다리 +79 / 왼다리 -79 가
               벌림(외전)이라, +phi 면 골반이 왼쪽으로 감 (오른다리 벌림, 왼다리 모음)
               다리 벌림 spread 는 반대 부호: 오른다리 +spread, 왼다리 -spread
    발목       pitch/roll 을 `AnkleKinematics.solve_ik` 로 모터각 a/b 로 바꿈. 왼다리는
               거울상 기하 (`Leg` 과 같음)

**발목 roll 의 부호는 확인되지 않았음** (기구학 좌표 x 축이 안쪽인지 바깥쪽인지 모름,
docs/issues.md #13). 골반이 기울 때 발판을 수평으로 되돌리는 방향을 `ANKLE_ROLL_SIGN`
하나로 둠. `sim_roll_check` 를 먼저 돌려 발바닥이 바닥에 붙은 채 있는지 볼 것 -- 발
바깥이나 안쪽 모서리가 들리면 부호를 뒤집고 다시 만듦 (`--roll-sign -1`).
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Dict, List, Tuple

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from huphy.kinematics.ankle import AnkleGeometry, AnkleKinematics  # noqa: E402

LEG_L = 0.39
"""허벅지·정강이 길이 (m). 발목 기하의 무릎 -> 발목 393mm 에서 반올림."""

HIP_WIDTH = 0.20
"""두 발 사이 (m). 실측 아님 -- 골반을 한쪽 발 위로 옮기는 각도를 정하는 데만 씀."""

MARGIN_DEG = 3.0
"""robot_v1.0 의 `command_margin_deg`. 이만큼 안쪽까지만 씀."""

LIMITS: Dict[str, Tuple[float, float]] = {
    "right_leg/hip_pitch": (-112.75, 17.75), "right_leg/hip_roll": (-19.5, 79.5),
    "right_leg/hip_yaw": (-40.5, 40.5), "right_leg/knee": (6.0, 114.0),
    "right_leg/ankle_a": (-61.88, 61.88), "right_leg/ankle_b": (-61.88, 61.88),
    "left_leg/hip_pitch": (-17.75, 112.75), "left_leg/hip_roll": (-79.5, 19.5),
    "left_leg/hip_yaw": (-40.5, 40.5), "left_leg/knee": (-114.0, -6.0),
    "left_leg/ankle_a": (-61.88, 61.88), "left_leg/ankle_b": (-61.88, 61.88),
}
"""`config/calibration/*_leg_v1.0.json` 의 limits_deg (10-16 기준)."""

ANKLE_ENVELOPE = {"pitch": (-40.0, 40.0), "roll": (-25.0, 25.0)}
"""`AnkleEnvelope` 기본값. 이 밖은 IK 가 거부함."""

A_SIDE_DOWN_DEG = 3.0
"""발판의 ankle_a 로드 쪽(기하값 C1)을 b 쪽보다 낮추는 roll 보정 (도). 모든 시퀀스에 걸림.

실물에서 양다리 다 a 쪽이 떠 있었음 (10-16). 3 도면 a 쪽 모서리가 약 4.6 mm 낮아짐.
a 쪽을 내리는 roll 은 오른다리 +, 왼다리 - (왼다리는 거울상 기하). pitch 는 안 바꿈.
"""

ORDER = (
    "left_leg/hip_pitch", "left_leg/hip_roll", "left_leg/hip_yaw",
    "left_leg/knee", "left_leg/ankle_a", "left_leg/ankle_b",
    "right_leg/hip_pitch", "right_leg/hip_roll", "right_leg/hip_yaw",
    "right_leg/knee", "right_leg/ankle_a", "right_leg/ankle_b",
)

_IK = {"right_leg": AnkleKinematics(AnkleGeometry()), "left_leg": AnkleKinematics(AnkleGeometry().mirrored())}


# ---------------------------------------------------------------------------
# 다리 하나
# ---------------------------------------------------------------------------
def leg_ik(x: float, height: float) -> Tuple[float, float, float]:
    """골반 기준 발 위치 (앞 x, 아래 height) -> (hip, knee, ankle_pitch) 굽힘 크기 (도).

    발판 수평. 허벅지·정강이 길이가 같아 닫힌 해임.
    """
    d = math.hypot(x, height)
    if d > 2 * LEG_L:
        raise ValueError(f"발이 닿지 않음: x={x:.3f} height={height:.3f}")
    knee = 2.0 * math.acos(d / (2 * LEG_L))
    hip = math.atan2(x, height) + knee / 2.0
    return math.degrees(hip), math.degrees(knee), math.degrees(knee - hip)


def leg_fk(hip: float, knee: float) -> Tuple[float, float]:
    """(hip, knee) 굽힘 크기 -> 골반 기준 발 위치 (x, height)."""
    h, k = math.radians(hip), math.radians(knee)
    return (LEG_L * math.sin(h) - LEG_L * math.sin(k - h),
            LEG_L * math.cos(h) + LEG_L * math.cos(k - h))


def _joint(leg: str, hip: float, knee: float, pitch: float, roll: float, hip_roll: float) -> Dict[str, float]:
    """굽힘 크기 -> 그 다리 관절 명령 (도, cal 공간)."""
    sign = -1.0 if leg == "right_leg" else 1.0
    a, b = _IK[leg].solve_ik(pitch, roll)
    return {
        f"{leg}/hip_pitch": sign * hip,
        f"{leg}/hip_roll": hip_roll,
        f"{leg}/hip_yaw": 0.0,
        f"{leg}/knee": -sign * knee,
        f"{leg}/ankle_a": a,
        f"{leg}/ankle_b": b,
    }


# ---------------------------------------------------------------------------
# 자세 = 두 발 위치 + 골반 기울기
# ---------------------------------------------------------------------------
class Pose:
    """골반 기준 두 발의 (앞, 높이) 와 골반의 좌우 기울기 phi (도, + = 골반이 왼쪽으로)."""

    def __init__(self, height: float, left=(0.0, 0.0), right=(0.0, 0.0), phi: float = 0.0,
                 ankle_down: float = 0.0, spread: float = 0.0, ankle_roll=None,
                 ankle_pitch=None):
        self.height = height
        self.feet = {"left_leg": left, "right_leg": right}     # (x, 들어올림)
        self.phi = phi
        self.ankle_down = ankle_down
        """발판 수평에서 발목 pitch 를 이만큼 덜 젖힘 (도). 0 이면 발판 수평."""
        self.spread = spread
        """다리마다 바깥으로 벌리는 hip_roll (도). 발목 roll 로 되돌려 발판은 수평."""
        self.ankle_roll = ankle_roll
        """주면 발목 roll 을 이 값(도)으로 고정함 -- 골반 기울기·벌림 보정을 안 함."""
        self.ankle_pitch = ankle_pitch
        """주면 발목 pitch 를 이 값(도)으로 고정함 -- 다리 자세와 상관없이."""
        self.ankle_up: Dict[str, float] = {}
        """다리별로 고정 발목 pitch 에 더 젖히는 양 (도, + = 발끝이 올라감)."""

    def joints(self, roll_sign: float) -> Tuple[Dict[str, float], Dict[str, dict]]:
        out: Dict[str, float] = {}
        info: Dict[str, dict] = {}
        for leg, (x, lift) in self.feet.items():
            # 벌린 만큼 다리 평면이 기울어, 같은 골반 높이에 다리가 1/cos 만큼 길어야 함.
            reach = (self.height - lift) / math.cos(math.radians(self.spread))
            hip, knee, pitch = leg_ik(x, reach)
            pitch -= self.ankle_down
            if self.ankle_pitch is not None:
                pitch = self.ankle_pitch + self.ankle_up.get(leg, 0.0)
            # 골반이 phi 만큼 기울면 다리도 같이 기움 -> 발판을 반대로 되돌림. 왼다리는
            # 거울상 기하라 같은 물리 방향이 반대 부호임.
            #
            # 벌림은 좌우 거울 동작이라 hip_roll 은 오른 +, 왼 - 이고, 되돌리는 발목 roll
            # 도 거울이어야 함 -- 발목 roll 도 거울 규약이라 (A_SIDE_DOWN_DEG 와 같음)
            # leg_sign 이 한 번만 붙음.
            leg_sign = 1.0 if leg == "right_leg" else -1.0
            hip_roll = self.phi + leg_sign * self.spread
            roll = (-roll_sign * leg_sign * (self.phi + self.spread)
                    + leg_sign * A_SIDE_DOWN_DEG)
            if self.ankle_roll is not None:
                roll = leg_sign * self.ankle_roll
            out.update(_joint(leg, hip, knee, pitch, roll, hip_roll))
            info[leg] = {"x": x, "lift": lift, "hip": hip, "knee": knee, "pitch": pitch, "roll": roll}
        return out, info


def lateral_phi(shift: float, height: float) -> float:
    """골반을 옆으로 shift (m) 옮기는 기울기 (도)."""
    return math.degrees(math.atan2(shift, height))


# ---------------------------------------------------------------------------
# 시퀀스
# ---------------------------------------------------------------------------
SQUAT_ANKLE_DOWN = 15.0
"""스쿼트 가장 깊은 자세에서 발목을 덜 젖히는 양 (도). 10 에서 올림 (10-19, 몸이 앞으로 쏠림)."""

FLAT_ANKLE_DOWN = 5.0
"""모델의 발판 수평보다 발목을 이만큼 펴야 실물 발바닥이 평평함 (도). 스쿼트 준비 자세에서
찾은 값 (10-19). 모델 각도가 실물과 어긋나는 몫이라 걷기에도 모든 키프레임에 같이 걸음."""

SQUAT_ANKLE_DOWN_START = FLAT_ANKLE_DOWN
"""스쿼트 준비 자세(무릎 20)에서 발목을 펴는 양 (도). 0 이면 발판 수평으로 당겨져 있음 (10-19)."""

SQUAT_SPREAD = 3.0
"""스쿼트에서 다리마다 바깥으로 벌리는 hip_roll (도). 0 에서 올림 (10-19).
3 도면 발이 한쪽에 약 4 cm 씩 바깥으로 감."""


def squat(ankle_down: float = SQUAT_ANKLE_DOWN, spread: float = SQUAT_SPREAD,
          ankle_down_start: float = SQUAT_ANKLE_DOWN_START,
          ) -> Tuple[str, int, List[Tuple[str, Pose, float, float]]]:
    """기준자세 -> 반 -> 깊게 -> 반 -> 기준자세. 엉덩이는 늘 발 바로 위 (앞뒤).

    `ankle_down` 은 가장 깊은 자세(무릎 70)에서 발목을 발판 수평보다 덜 젖히는 양 (도).
    준비 자세(무릎 20)에서 `ankle_down_start` 이고 깊이에 비례해 `ankle_down` 까지 늘어남
    -- 발끝이 그만큼 내려가는 쪽, 즉 정강이가 뒤로 서서 몸이 뒤로 감.

    `spread` 는 다리마다 바깥으로 벌리는 hip_roll (도). 모든 키프레임에 같게 걸림.
    """
    def at(knee: float) -> Pose:
        down = ankle_down_start + (ankle_down - ankle_down_start) * (knee - 20.0) / (70.0 - 20.0)
        return Pose(2 * LEG_L * math.cos(math.radians(knee / 2)), ankle_down=down, spread=spread)
    return "sim_squat", 3, [
        ("1 준비 (기준자세, 무릎 20)", at(20), 2.5, 1.0),
        ("2 내려가기 반 (무릎 45)", at(45), 2.0, 0.3),
        ("3 깊게 (무릎 70)", at(70), 2.0, 1.0),
        ("4 올라오기 반 (무릎 45)", at(45), 2.0, 0.3),
        ("5 기준자세 (무릎 20)", at(20), 2.0, 1.0),
    ]


def squat_start_ankle_pitch() -> float:
    """스쿼트 준비 자세(무릎 20)의 발목 pitch (도). 실물에서 발바닥이 평평한 각도."""
    _, info = squat()[2][0][1].joints(1.0)
    return info["right_leg"]["pitch"]


WALK_LIFT = 0.15
"""걸을 때 발을 드는 높이 (m). 0.08 에서 올림 (10-19, 무릎을 앞으로 더 들어야 함)."""


WALK_TOE_UP = 5.0
"""내딛기(D) 때 흔들발 발목을 더 젖히는 양 (도). 발끝이 올라감 (10-19)."""

WALK_SPREAD = 4.0
"""걸을 때 다리마다 바깥으로 벌리는 hip_roll (도). 0 에서 올림 (10-19)."""


WALK_SWAY = 4.0
"""체중 이동 때 골반 좌우 기울기 (도). 예전에는 골반을 디딤발 바로 위까지 옮기도록
계산해 11.6 도였음 -- 너무 흔들어서 줄임 (10-19)."""


def walk(step: float = 0.13, lift: float = WALK_LIFT, knee0: float = 30.0,
         spread: float = WALK_SPREAD, sway: float = WALK_SWAY, toe_up: float = WALK_TOE_UP):
    """정적 보행 세 개. 늘 한 발로 선 채 다른 발을 옮김 -- 느리고 단계가 분명함.

        sim_walk_start   두 발 모음 -> 왼발 한 걸음. 1번        (A, 왼발 B~F)
        sim_walk         오른발 한 걸음 + 왼발 한 걸음. 무한 반복 (B~F x 2)
        sim_walk_stop    뒷발을 끌어와 두 발 모음. 1번          (Z)

    한 걸음은

        B 체중 이동   골반을 디딤발 쪽으로 (hip_roll)
        C 발 들기     흔들발을 lift 만큼
        D 내딛기      흔들발을 앞으로 (공중)
        E 내려놓기    흔들발을 바닥에
        F 가운데로    골반을 두 발 사이로 (앞으로 반 걸음, 기울기 0)

    **`sim_walk` 는 끝 자세가 첫 자세의 바로 앞 자세임** -- 왼발이 반 걸음 앞, 오른발이
    반 걸음 뒤. 그래서 studio 가 처음으로 돌아가도 발이 끌리지 않음. 두 발 모음에서
    바로 반복을 틀면 첫 체중 이동에서 오른발이 바닥을 끌고 뒤로 감 -- `sim_walk_start`
    를 먼저 틀 것.

    `spread` 는 다리마다 바깥으로 벌리는 hip_roll (도).

    `sway` 는 체중 이동 때 골반 좌우 기울기 (도). 골반이 디딤발 바로 위까지 가지는
    않음 -- 거기까지 가려면 `lateral_phi(HIP_WIDTH / 2, H) + spread` 가 필요함.

    **발목은 모든 키프레임에서 스쿼트 준비 자세와 같은 모터각으로 고정함** (pitch
    `squat_start_ankle_pitch()`, roll 0). 다리 자세에 따라 바꾸지 않음. 예외는 내딛기(D)
    의 흔들발 하나 -- `toe_up` 만큼 더 젖혀 발끝을 올림. 내려놓기(E)에서 고정 각도로 돌아옴.
    """
    H = 2 * LEG_L * math.cos(math.radians(knee0 / 2))
    ankle = squat_start_ankle_pitch()

    def base() -> Pose:
        return Pose(H, ankle_pitch=ankle, ankle_roll=0.0, spread=spread)

    def one_step(frames, feet, stance, swing, tag):
        """디딤발 stance 로 서서 swing 을 한 걸음 옮김. 걸음 뒤의 발 위치를 냄 (골반 기준 앞)."""
        side = 1.0 if stance == "left_leg" else -1.0       # 골반을 디딤발 쪽으로
        # 골반을 디딤발 위로 앞뒤도 맞춤: 디딤발 x 를 0 으로 만들도록 전체를 옮김
        f = {k: v - feet[stance] for k, v in feet.items()}

        def pose(sw_x, sw_lift, up=0.0):
            p = base()
            p.phi = side * sway
            p.ankle_up[swing] = up
            p.feet[stance] = (f[stance], 0.0)
            p.feet[swing] = (sw_x, sw_lift)
            return p
        frames.append((f"{tag} B 체중 이동", pose(f[swing], 0.0), 1.5, 0.3))
        frames.append((f"{tag} C 발 들기", pose(f[swing], lift), 1.0, 0.0))
        frames.append((f"{tag} D 내딛기", pose(step, lift, toe_up), 1.2, 0.0))
        frames.append((f"{tag} E 내려놓기", pose(step, 0.0), 1.0, 0.2))
        # 가운데로: 골반을 두 발 가운데로, 기울기 0
        mid = step / 2
        p = base()
        p.feet[stance] = (f[stance] - mid, 0.0)
        p.feet[swing] = (step - mid, 0.0)
        frames.append((f"{tag} F 가운데로", p, 1.5, 0.3))
        return {stance: f[stance] - mid, swing: step - mid}

    # 시작: 두 발 모음 -> 오른발로 서서 왼발 한 걸음. 끝나면 왼발 앞, 오른발 뒤.
    start: List[Tuple[str, Pose, float, float]] = [
        ("A 준비 (두 발 모음, 무릎 %.0f)" % knee0, base(), 2.5, 1.0)]
    looped = one_step(start, {"left_leg": 0.0, "right_leg": 0.0}, "right_leg", "left_leg", "시작 왼발")

    # 반복: 오른발 -> 왼발. 끝 발 위치가 시작과 같아야 이어짐.
    loop: List[Tuple[str, Pose, float, float]] = []
    feet = one_step(loop, looped, "left_leg", "right_leg", "오른발")
    feet = one_step(loop, feet, "right_leg", "left_leg", "왼발")
    assert all(abs(feet[k] - looped[k]) < 1e-9 for k in feet), "반복 끝 자세가 시작과 다름"

    # 멈춤: 앞발(왼발)로 서서 뒷발(오른발)을 끌어와 두 발 모음.
    stance, swing = "left_leg", "right_leg"
    back = feet[swing] - feet[stance]

    def pose_end(sw_x, sw_lift):
        p = base()
        p.phi = sway
        p.feet[swing] = (sw_x, sw_lift)
        return p
    stop = [
        ("Z 체중 이동", pose_end(back, 0.0), 1.5, 0.3),
        ("Z 뒷발 들기", pose_end(back, lift), 1.0, 0.0),
        ("Z 뒷발 모으기", pose_end(0.0, lift), 1.2, 0.0),
        ("Z 내려놓기", pose_end(0.0, 0.0), 1.0, 0.2),
        ("Z 가운데로 (두 발 모음)", base(), 1.5, 1.0),
    ]
    return [("sim_walk_start", 1, start), ("sim_walk", 0, loop), ("sim_walk_stop", 1, stop)]


def roll_check(knee0: float = 30.0, deg: float = 3.0):
    """발목 roll 부호 확인. 골반을 좌우로 deg 만 기울임 -- 발바닥이 붙어 있어야 맞음."""
    H = 2 * LEG_L * math.cos(math.radians(knee0 / 2))
    return "sim_roll_check", 1, [
        ("1 준비 (무릎 %.0f)" % knee0, Pose(H), 2.5, 1.0),
        ("2 골반 왼쪽 %.0f도 -- 발바닥 확인" % deg, Pose(H, phi=deg), 2.0, 3.0),
        ("3 가운데", Pose(H), 2.0, 1.0),
        ("4 골반 오른쪽 %.0f도 -- 발바닥 확인" % deg, Pose(H, phi=-deg), 2.0, 3.0),
        ("5 가운데", Pose(H), 2.0, 1.0),
    ]


# ---------------------------------------------------------------------------
# 검사와 출력
# ---------------------------------------------------------------------------
def check(joints: Dict[str, float], info: Dict[str, dict]) -> List[str]:
    bad = []
    for j, v in joints.items():
        lo, hi = LIMITS[j]
        if not (lo + MARGIN_DEG <= v <= hi - MARGIN_DEG):
            bad.append(f"{j} {v:.1f} (허용 {lo + MARGIN_DEG:.1f}~{hi - MARGIN_DEG:.1f})")
    for leg, d in info.items():
        for k in ("pitch", "roll"):
            lo, hi = ANKLE_ENVELOPE[k]
            if not lo <= d[k] <= hi:
                bad.append(f"{leg} 발목 {k} {d[k]:.1f} (범위 {lo:g}~{hi:g})")
    return bad


def build(seq, roll_sign: float, verbose: bool, preview: list = None) -> dict:
    name, repeat, frames = seq
    keyframes, problems = [], []
    pv = {"name": name, "frames": []}
    if verbose:
        print(f"\n== {name}  (키프레임 {len(frames)}개, 반복 {repeat or '무한'})")
        print("   단계                              엉덩이높이  발(앞,들기 m)  왼 hip/knee/pitch/roll    오른 hip/knee/pitch/roll   기울기  여유")
    for label, pose, move, hold in frames:
        joints, info = pose.joints(roll_sign)
        bad = check(joints, info)
        problems += [f"{label}: {b}" for b in bad]
        margin = min(
            min(v - (LIMITS[j][0] + MARGIN_DEG), (LIMITS[j][1] - MARGIN_DEG) - v) for j, v in joints.items()
        )
        pv["frames"].append({
            "label": label, "move": move, "hold": hold, "phi": pose.phi, "height": pose.height,
            "spread": pose.spread,
            "legs": {leg: {k: info[leg][k] for k in ("hip", "knee", "pitch", "lift", "x")} for leg in info},
        })
        keyframes.append({
            "label": label,
            "pose": {j: round(joints[j], 2) for j in ORDER},
            "move_s": move,
            "hold_s": hold,
        })
        if verbose:
            L, R = info["left_leg"], info["right_leg"]
            feet = f"L({L['x']:+.3f},{L['lift']:.2f}) R({R['x']:+.3f},{R['lift']:.2f})"
            lt = f"{L['hip']:5.1f}/{L['knee']:5.1f}/{L['pitch']:5.1f}/{L['roll']:+4.1f}"
            rt = f"{R['hip']:5.1f}/{R['knee']:5.1f}/{R['pitch']:5.1f}/{R['roll']:+4.1f}"
            print(f"   {label[:32]:32s}  {pose.height:.3f}m  {feet}  {lt}  {rt}  {pose.phi:+5.1f}  {margin:5.1f}{'  !!' if bad else ''}")
    total = sum(f[2] + f[3] for f in frames)
    if verbose:
        print(f"   한 바퀴 {total:.1f}초" + ("" if not problems else f"  ** 한계 문제 {len(problems)}개 **"))
        for p in problems:
            print("   !!", p)
    if problems:
        raise SystemExit(f"{name}: 한계 밖 자세가 있어 쓰지 않음")
    if preview is not None:
        preview.append(pv)
    return {"name": name, "ankle_space": "ab", "repeat": repeat, "keyframes": keyframes}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--check", action="store_true", help="파일을 쓰지 않고 표만 찍음")
    ap.add_argument("--roll-sign", type=float, choices=(1.0, -1.0), default=1.0,
                    help="발목 roll 보정 방향. sim_roll_check 에서 발 모서리가 들리면 -1")
    ap.add_argument("--out", type=Path, default=ROOT / "config" / "sequences")
    ap.add_argument("--preview", type=Path, help="막대 그림 미리보기 html 을 이 경로에 씀")
    ap.add_argument("--squat-ankle-down", type=float, default=SQUAT_ANKLE_DOWN,
                    help=f"스쿼트 가장 깊은 자세에서 발목을 펴는 양 (도). 클수록 몸이 뒤로. 기본 {SQUAT_ANKLE_DOWN:g}")
    ap.add_argument("--squat-ankle-down-start", type=float, default=SQUAT_ANKLE_DOWN_START,
                    help=f"스쿼트 준비 자세(무릎 20)에서 발목을 펴는 양 (도). 기본 {SQUAT_ANKLE_DOWN_START:g}")
    ap.add_argument("--squat-spread", type=float, default=SQUAT_SPREAD,
                    help=f"스쿼트에서 다리마다 바깥으로 벌리는 hip_roll (도). 기본 {SQUAT_SPREAD:g}")
    ap.add_argument("--walk-toe-up", type=float, default=WALK_TOE_UP,
                    help=f"내딛을 때 흔들발 발목을 더 젖히는 양 (도). 기본 {WALK_TOE_UP:g}")
    ap.add_argument("--walk-spread", type=float, default=WALK_SPREAD,
                    help=f"걸을 때 다리마다 바깥으로 벌리는 hip_roll (도). 기본 {WALK_SPREAD:g}")
    ap.add_argument("--walk-sway", type=float, default=WALK_SWAY,
                    help=f"걸을 때 체중 이동의 골반 기울기 (도). 기본 {WALK_SWAY:g}")
    ap.add_argument("--lift", type=float, default=WALK_LIFT,
                    help=f"걸을 때 발을 드는 높이 (m). 클수록 무릎이 높이 올라감. 기본 {WALK_LIFT:g}")
    ap.add_argument("--step", type=float, default=0.13,
                    help="한 걸음에 나아가는 거리 (m). 기본 0.13")
    args = ap.parse_args(argv)

    # 모델 확인: 기준자세의 발 위치와 발목
    x, z = leg_fk(10.0, 20.0)
    print(f"모델 확인  기준자세(엉덩이 10, 무릎 20) -> 발 앞 {x * 1000:+.1f}mm, 엉덩이 높이 {z:.3f}m, "
          f"발판 수평 발목 pitch {20 - 10:.1f}도 (정책 기준자세의 FK: 8.8도)")
    print(f"걷기 골반 기울기 {args.walk_sway:g}도, 다리 벌림 {args.walk_spread:g}도, "
          f"발목 roll 부호 {args.roll_sign:+.0f}")

    previews: list = []
    seqs = [roll_check(), squat(args.squat_ankle_down, args.squat_spread, args.squat_ankle_down_start)]
    seqs += walk(step=args.step, lift=args.lift, spread=args.walk_spread, sway=args.walk_sway, toe_up=args.walk_toe_up)
    for seq in seqs:
        body = build(seq, args.roll_sign, verbose=True, preview=previews)
        if not args.check:
            path = args.out / f"{body['name']}.json"
            path.write_text(json.dumps(body, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
            print(f"   -> {path.relative_to(ROOT)}")
    if args.preview:
        template = (Path(__file__).with_name("sequence_preview.html")).read_text(encoding="utf-8")
        data = json.dumps({"leg_l": LEG_L, "hip_w": HIP_WIDTH, "sequences": previews}, ensure_ascii=False)
        args.preview.parent.mkdir(parents=True, exist_ok=True)
        args.preview.write_text(template.replace("/*DATA*/null", data), encoding="utf-8")
        print(f"\n미리보기: {args.preview}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
