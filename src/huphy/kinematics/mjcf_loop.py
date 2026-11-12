"""학습 MJCF 의 발목 폐루프 — 크랭크각에서 종속 관절 6개를 냄.

`ankle.py` 와 **다른 모델임.** 둘을 섞어 쓰면 안 됨.

    ankle.py        실측 링키지. 발목 목표각 -> 모터각 (`solve_ik`). 제어가 씀
    이 파일         학습 MJCF 링키지. 모터각 -> 종속 관절각. 관찰 벡터가 씀

## 왜 따로 있나

모션 추종 정책(`motion545`·`motion399`)의 관찰은 관절 24개를 받음. 그중
다리마다 6개는 모터가 없음.

    rod_A_u1, rod_A_u2      로드 A 의 유니버설 조인트 2축
    rod_B_u1, rod_B_u2      로드 B 의 유니버설 조인트 2축
    ankle_pitch, ankle_roll  발목 2축

인코더가 없지만 **크랭크각 2개로 정해짐** -- 로드 끝이 발판에 붙어 있어 루프가
닫혀 있음. 미지수 6개(위 6개)에 구속 6개(로드 끝 == 발판 점, 로드마다 3개)임.

그 값을 `ankle.py` 로 내면 안 되는 이유: 두 모델의 링키지 수치가 다름. 같은
크랭크각에 대해 피치가 약 1.1배, 롤이 약 1.4배 다르고 롤 부호 규약도 다름
(docs/policies/motion.md 3절). 정책은 **MJCF 가 내는 값**으로 학습했으므로
관찰에는 이 파일의 값을 넣어야 함.

아래 수치는 이 레포의 RViz URDF (`ros/huphy_ros/urdf/huphy.urdf`) 와도 **같음** --
크랭크·로드·볼 점 좌표가 0 차이임. 둘이 같은 Fusion 모델에서 나왔음
(`urdf/make_urdf.py`). 다른 것은 `ankle.py` 의 실측 좌표 하나임.

`ros/huphy_ros/huphy_ros/linkage.py` 와도 하는 일이 다름. 그쪽은 크랭크각과 **발목
2축을 다 받아** 로드 각도만 풂 (RViz 가 나무 구조라 로드 값을 누가 줘야 함).
이쪽은 크랭크각 2개만 받아 발목 2축까지 풂.

## 수치의 출처

`pygmalion_locomotion/assets/pygmalion_v2/LegOnly_prototype-tempmass-motormeasured-
armfix_v30_proxyfix_footv5_d455_reinf_alparts2_capsule7plus5_loop.xml`
(sha256 `a34631c3...`, 꾸러미 계약의 `robot.mjcf_sha256`). body `pos`, joint `axis`,
site `pos` 를 그대로 옮긴 것임. 길이는 m, 각도는 rad.

## 푸는 방법

미지수를 (pitch, roll) 둘로 줄임. 로드 회전은 로드 원점 주위의 순회전이라 로드
원점에서 로드 끝까지 거리가 늘 `rod_len` 임. 그래서

    잔차_rod = ||발판점(pitch, roll) - 로드원점(크랭크각)|| - rod_len

둘이 0 이면 그 자세가 답임. 뉴턴 2차원, 직전 자세를 추정으로 주면 2~3회 반복임.
자세가 나오면 u1·u2 는 닫힌 해로 나옴 (`_solve_rod`).

## 검증

학습에 쓴 reference motion 659 프레임의 24관절 값이 참값임. 크랭크각 2개만 주고
푼 결과가 나머지 6개와 최대 6.8e-8 rad (3.9e-6 도) 안에서 같음
(`tests/test_mjcf_loop.py`). MJCF 자체의 폐루프 잔차는 4.2e-9 m 임 (계약의
`motion.max_closed_loop_residual_m`).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Tuple

import numpy as np

Vec3 = Tuple[float, float, float]

TOL = 1e-10
"""뉴턴 종료 기준. 잔차(m) 의 최대절대값."""

MAX_ITER = 12
"""뉴턴 최대 반복. 직전 자세를 추정으로 주면 실제로는 2~3회임."""

JAC_STEP = 1e-6
"""속도용 야코비안의 중앙차분 폭 (rad)."""


class LoopUnsolvedError(ValueError):
    """주어진 크랭크각에서 루프가 닫히는 자세를 못 찾음.

    크랭크각 둘이 서로 모순되는 조합일 때 남 -- 실물에서는 한쪽 인코더가 튀었거나
    캘리브레이션이 어긋난 것임. 관찰에 엉뚱한 값을 넣는 것보다 멈추는 쪽이 나음.
    """


# ===========================================================================
# 기하 — MJCF 수치
# ===========================================================================
@dataclass(frozen=True)
class RodGeometry:
    """로드 하나. 크랭크 -> 로드 -> 발판 순서."""

    crank_pos: Vec3
    """크랭크 body 의 위치. 정강이 프레임."""
    crank_sign: float
    """크랭크 joint 축. MJCF 가 `(0, ±1, 0)` 이라 부호만 둠 (+1 = +y)."""
    rod_pos: Vec3
    """로드 body 의 위치. **크랭크가 돈 뒤의 크랭크 프레임.**"""
    u1_sign: float
    """로드 u1 축. MJCF 가 `(0, ±1, 0)` 이라 부호만 둠."""
    u2_axis: Vec3
    """로드 u2 축. y 성분이 0 인 단위벡터임 -- `_solve_rod` 가 그것에 기댐."""
    rod_end: Vec3
    """로드 끝 site. 로드 프레임."""
    ball: Vec3
    """로드 끝이 붙는 발판 site. 발 프레임."""

    @property
    def rod_len(self) -> float:
        return float(np.linalg.norm(self.rod_end))


@dataclass(frozen=True)
class LoopGeometry:
    """다리 하나의 발목 폐루프."""

    rod_a: RodGeometry
    rod_b: RodGeometry
    ankle_pos: Vec3
    """발목 피치 body 의 위치. 정강이 프레임."""
    pitch_sign: float
    roll_sign: float
    default: Dict[str, float]
    """기본자세의 종속 관절값 (rad). 뉴턴 추정의 시작값이고 관찰의 기준값임.

    꾸러미 계약의 `joint_default_rad` 와 같아야 함 -- 학습 `env.yaml` 의
    `init_state.joint_pos` 에서 온 값임.
    """


RIGHT = LoopGeometry(
    rod_a=RodGeometry(
        crank_pos=(-0.029945, -0.001718, -0.190034), crank_sign=-1.0,
        rod_pos=(-0.061494, 0.042718, -0.021185), u1_sign=-1.0,
        u2_axis=(0.999282, 0.0, 0.0378993),
        rod_end=(0.010939, 0.0026, -0.288781), ball=(-0.0505, 0.0436, -0.01)),
    rod_b=RodGeometry(
        crank_pos=(-0.030055, 0.001717, -0.290034), crank_sign=1.0,
        rod_pos=(-0.060039, -0.042717, -0.01522), u1_sign=1.0,
        u2_axis=(-0.998789, 0.0, -0.0491995),
        rod_end=(0.009594, -0.0026, -0.194746), ball=(-0.0505, -0.0436, -0.01)),
    ankle_pos=(-0.03, 0.0, -0.49), pitch_sign=-1.0, roll_sign=-1.0,
    default={
        "rod_a_u1": -0.14797270816558003, "rod_a_u2": 2.6819833875474064e-06,
        "rod_b_u1": 0.14892320455004768, "rod_b_u2": -3.976385353225781e-06,
        "ankle_pitch": 0.17516470240179116, "ankle_roll": -7.751934743886312e-05,
    },
)

LEFT = LoopGeometry(
    rod_a=RodGeometry(
        crank_pos=(-0.029945, 0.001718, -0.190034), crank_sign=1.0,
        rod_pos=(-0.061494, -0.042718, -0.021185), u1_sign=1.0,
        u2_axis=(-0.999282, 0.0, -0.0378993),
        rod_end=(0.010939, -0.0026, -0.288781), ball=(-0.0505, -0.0436, -0.01)),
    rod_b=RodGeometry(
        crank_pos=(-0.030055, -0.001717, -0.290034), crank_sign=-1.0,
        rod_pos=(-0.060039, 0.042717, -0.01522), u1_sign=-1.0,
        u2_axis=(0.998789, 0.0, 0.0491995),
        rod_end=(0.009594, 0.0026, -0.194746), ball=(-0.0505, 0.0436, -0.01)),
    ankle_pos=(-0.03, 0.0, -0.49), pitch_sign=-1.0, roll_sign=1.0,
    default={
        "rod_a_u1": 0.1479036260054202, "rod_a_u2": -1.6276083554340244e-07,
        "rod_b_u1": -0.1487249952991747, "rod_b_u2": 2.412252153998891e-07,
        "ankle_pitch": 0.1750072557823706, "ankle_roll": 4.703560734910088e-06,
    },
)
"""왼다리는 MJCF 에서 y 와 축 부호만 뒤집힌 거울상임. **가정이 아니라 MJCF 수치임.**"""

SIDES = {"right_leg": RIGHT, "left_leg": LEFT}

DEPENDENT = ("rod_a_u1", "rod_a_u2", "rod_b_u1", "rod_b_u2", "ankle_pitch", "ankle_roll")
"""이 파일이 내는 관절. 모터가 없음."""


# ===========================================================================
# 기본 회전 — 축이 ±y / ±x 라 행렬을 안 만듦
# ===========================================================================
def _rot_y(v: Vec3, sign: float, q: float) -> Vec3:
    """`(0, sign, 0)` 축 회전. 크랭크·u1·발목 피치가 전부 이 꼴임."""
    c, s = math.cos(sign * q), math.sin(sign * q)
    return (v[0] * c + v[2] * s, v[1], -v[0] * s + v[2] * c)


def _rot_x(v: Vec3, sign: float, q: float) -> Vec3:
    """`(sign, 0, 0)` 축 회전. 발목 롤이 이 꼴임."""
    c, s = math.cos(sign * q), math.sin(sign * q)
    return (v[0], v[1] * c - v[2] * s, v[1] * s + v[2] * c)


def _rot_axis(v: Vec3, axis: Vec3, q: float) -> Vec3:
    """임의 축 회전 (로드레그). u2 에만 씀 -- 한 주기에 다리마다 2번."""
    c, s = math.cos(q), math.sin(q)
    ax, ay, az = axis
    dot = ax * v[0] + ay * v[1] + az * v[2]
    cx = ay * v[2] - az * v[1]
    cy = az * v[0] - ax * v[2]
    cz = ax * v[1] - ay * v[0]
    return (
        v[0] * c + cx * s + ax * dot * (1.0 - c),
        v[1] * c + cy * s + ay * dot * (1.0 - c),
        v[2] * c + cz * s + az * dot * (1.0 - c),
    )


def _ball_point(g: LoopGeometry, rod: RodGeometry, pitch: float, roll: float) -> Vec3:
    """발판 site 의 정강이 프레임 위치."""
    v = _rot_x(rod.ball, g.roll_sign, roll)
    v = _rot_y(v, g.pitch_sign, pitch)
    return (v[0] + g.ankle_pos[0], v[1] + g.ankle_pos[1], v[2] + g.ankle_pos[2])


def _rod_origin(rod: RodGeometry, crank: float) -> Vec3:
    """로드 회전 중심의 정강이 프레임 위치."""
    v = _rot_y(rod.rod_pos, rod.crank_sign, crank)
    return (v[0] + rod.crank_pos[0], v[1] + rod.crank_pos[1], v[2] + rod.crank_pos[2])


def _residual(g: LoopGeometry, crank_a: float, crank_b: float,
              pitch: float, roll: float) -> Tuple[float, float]:
    """로드 길이 오차 둘 (m). 둘 다 0 이면 그 자세가 답임."""
    out = []
    for rod, crank in ((g.rod_a, crank_a), (g.rod_b, crank_b)):
        p = _ball_point(g, rod, pitch, roll)
        o = _rod_origin(rod, crank)
        d = math.sqrt((p[0] - o[0]) ** 2 + (p[1] - o[1]) ** 2 + (p[2] - o[2]) ** 2)
        out.append(d - rod.rod_len)
    return out[0], out[1]


# ===========================================================================
# 자세 풀기
# ===========================================================================
def solve_pose(g: LoopGeometry, crank_a: float, crank_b: float,
               guess: Tuple[float, float]) -> Tuple[float, float]:
    """(크랭크 A, 크랭크 B) -> (발목 피치, 발목 롤). rad.

    `guess` 는 (피치, 롤) 추정임. **직전 주기 값을 줄 것** -- 같은 크랭크각에
    대응하는 자세가 여럿이라 추정이 멀면 다른 자세로 수렴함.
    """
    pitch, roll = float(guess[0]), float(guess[1])
    h = 1e-7
    for _ in range(MAX_ITER):
        r0, r1 = _residual(g, crank_a, crank_b, pitch, roll)
        if max(abs(r0), abs(r1)) < TOL:
            return pitch, roll
        p0, p1 = _residual(g, crank_a, crank_b, pitch + h, roll)
        q0, q1 = _residual(g, crank_a, crank_b, pitch, roll + h)
        j00, j10 = (p0 - r0) / h, (p1 - r1) / h
        j01, j11 = (q0 - r0) / h, (q1 - r1) / h
        det = j00 * j11 - j01 * j10
        if det == 0.0:
            raise LoopUnsolvedError(
                f"야코비안이 특이함 (크랭크 {crank_a:.4f}, {crank_b:.4f} rad)")
        pitch -= (j11 * r0 - j01 * r1) / det
        roll -= (-j10 * r0 + j00 * r1) / det
    r0, r1 = _residual(g, crank_a, crank_b, pitch, roll)
    raise LoopUnsolvedError(
        f"{MAX_ITER}회 안에 안 수렴함 (크랭크 {crank_a:.4f}, {crank_b:.4f} rad, "
        f"남은 잔차 {max(abs(r0), abs(r1)):.2e} m). 인코더나 캘리브레이션을 볼 것")


def _solve_rod(g: LoopGeometry, rod: RodGeometry, crank: float,
               pitch: float, roll: float, hint: Tuple[float, float]) -> Tuple[float, float]:
    """로드 하나의 (u1, u2). 자세가 정해진 뒤 닫힌 해로 나옴.

    `R(u1축, u1) R(u2축, u2) rod_end = p` 를 푸는 것임. u1 축이 ±y 이고 u2 축의
    y 성분이 0 이라 **y 성분만으로 u2 가 나옴** -- 축 회전은 그 축 방향 성분을
    안 바꾸고, u1 회전은 y 성분을 안 바꿈.

        p_y = A cos(u2) + B sin(u2)

    해가 둘이라 `hint`(직전 주기 값) 에 가까운 쪽을 고름.
    """
    p = _ball_point(g, rod, pitch, roll)
    p = (p[0] - rod.crank_pos[0], p[1] - rod.crank_pos[1], p[2] - rod.crank_pos[2])
    p = _rot_y(p, rod.crank_sign, -crank)
    p = (p[0] - rod.rod_pos[0], p[1] - rod.rod_pos[1], p[2] - rod.rod_pos[2])

    e = rod.rod_end
    ax = rod.u2_axis
    dot = ax[0] * e[0] + ax[1] * e[1] + ax[2] * e[2]
    A = e[1] - ax[1] * dot
    B = ax[2] * e[0] - ax[0] * e[2]          # (u2축 x rod_end) 의 y 성분
    C = p[1] - ax[1] * dot
    hyp = math.hypot(A, B)
    if hyp == 0.0:
        raise LoopUnsolvedError("로드 u2 가 y 성분을 안 바꿈 -- 기하 수치를 볼 것")
    base = math.atan2(B, A)
    arg = max(-1.0, min(1.0, C / hyp))
    best = None
    for branch in (1.0, -1.0):
        u2 = base + branch * math.acos(arg)
        w = _rot_axis(e, ax, u2)
        # u1: u1축 회전으로 w 를 p 에 맞춤. 축이 (0, s, 0) 이라 xz 평면 회전임
        wx, wz = w[0], w[2]
        u1 = rod.u1_sign * (math.atan2(p[0], p[2]) - math.atan2(wx, wz))
        u1 = (u1 + math.pi) % (2.0 * math.pi) - math.pi
        cost = abs(u1 - hint[0]) + abs(u2 - hint[1])
        if best is None or cost < best[0]:
            best = (cost, u1, u2)
    assert best is not None
    return best[1], best[2]


def solve(g: LoopGeometry, crank_a: float, crank_b: float,
          guess: Dict[str, float]) -> Dict[str, float]:
    """(크랭크 A, 크랭크 B) -> 종속 관절 6개. rad.

        guess   직전 주기의 같은 사전. 시작할 때는 `g.default` 를 줄 것

    돌려주는 키는 `DEPENDENT` 임.
    """
    pitch, roll = solve_pose(
        g, crank_a, crank_b, (guess["ankle_pitch"], guess["ankle_roll"]))
    a1, a2 = _solve_rod(g, g.rod_a, crank_a, pitch, roll,
                        (guess["rod_a_u1"], guess["rod_a_u2"]))
    b1, b2 = _solve_rod(g, g.rod_b, crank_b, pitch, roll,
                        (guess["rod_b_u1"], guess["rod_b_u2"]))
    return {"rod_a_u1": a1, "rod_a_u2": a2, "rod_b_u1": b1, "rod_b_u2": b2,
            "ankle_pitch": pitch, "ankle_roll": roll}


def velocities(g: LoopGeometry, crank_a: float, crank_b: float,
               crank_a_vel: float, crank_b_vel: float,
               pose: Dict[str, float]) -> Dict[str, float]:
    """종속 관절 6개의 속도. rad/s.

        pose   같은 크랭크각에서 `solve` 가 낸 자세. 중앙차분의 추정으로 씀

    야코비안을 중앙차분으로 냄 (`JAC_STEP`). 크랭크각을 흔들어 다시 푸는 것이라
    풀이 4번이 더 들지만, `pose` 에서 시작하므로 각 풀이가 1~2회 반복으로 끝남.

    인코더 속도를 그대로 쓰므로 **차분으로 만든 속도가 아님** -- 모터가 보고하는
    속도를 그대로 넣음.
    """
    h = JAC_STEP
    cols = []
    for da, db in ((h, 0.0), (0.0, h)):
        plus = solve(g, crank_a + da, crank_b + db, pose)
        minus = solve(g, crank_a - da, crank_b - db, pose)
        cols.append({k: (plus[k] - minus[k]) / (2.0 * h) for k in DEPENDENT})
    return {
        k: cols[0][k] * crank_a_vel + cols[1][k] * crank_b_vel
        for k in DEPENDENT
    }
