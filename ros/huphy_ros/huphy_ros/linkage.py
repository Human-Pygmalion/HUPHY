"""발목 막대 각도를 풂. RViz 용. `rclpy` 를 안 씀.

URDF 는 나무 구조라 크랭크 -> 막대 -> 발 고리를 못 적음. 발은 `ankle_pitch/roll` 에, 막대는
크랭크에 따로 매달려 있고 막대 끝이 발에 닿는다는 조건이 빠져 있음. robot_state_publisher
는 계산을 안 하므로 막대 관절(`R_rod_A_u1`, `_u2`) 값을 누가 줘야 함.

크랭크각 (`ankle_a/b`) 과 발판 자세 (`ankle_pitch/roll`) 는 `joint_states` 에 이미 있음.
남은 미지수는 막대 하나에 두 개 (유니버설 조인트 u1, u2) 이고, 조건은 "막대 끝 점 =
발의 볼 점" (3식) 임. 막대 길이가 맞으면 해가 있음 -- 가우스-뉴턴 몇 번으로 풂.

    shin ─ crank ─ u1 ─ u2 ─ rod ─ (rod_X_end)      이 두 점이
    shin ─ ankle_pitch ─ ankle_roll ─ foot ─ (ball_X)   같은 곳에 오게

기하는 전부 URDF 에서 읽음 (`urdf/make_urdf.py` 가 두 점을 빈 링크로 넣어 둠). 숫자를
코드에 박지 않음 -- 모델이 바뀌면 URDF 만 다시 만들면 됨.
"""

from __future__ import annotations

import math
import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Dict, List, Mapping, Optional, Tuple

import numpy as np

_ROD = re.compile(r"^([RL])_rod_([AB])_u1$")


@dataclass(frozen=True)
class _Joint:
    name: str
    parent: str
    child: str
    origin: np.ndarray  # 4x4
    axis: np.ndarray    # 3
    moving: bool
    lower: float = -math.pi
    upper: float = math.pi


def _rpy(r: float, p: float, y: float) -> np.ndarray:
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def _axis_angle(axis: np.ndarray, q: float) -> np.ndarray:
    x, y, z = axis
    c, s, t = math.cos(q), math.sin(q), 1 - math.cos(q)
    return np.array([
        [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
        [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
        [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
    ])


def _floats(text: Optional[str], default: str) -> List[float]:
    return [float(v) for v in (text or default).split()]


def parse(urdf: str) -> Dict[str, _Joint]:
    """URDF 문자열 -> 자식 링크 이름별 관절."""
    by_child: Dict[str, _Joint] = {}
    for j in ET.fromstring(urdf).iter("joint"):
        o = j.find("origin")
        T = np.eye(4)
        if o is not None:
            T[:3, :3] = _rpy(*_floats(o.get("rpy"), "0 0 0"))
            T[:3, 3] = _floats(o.get("xyz"), "0 0 0")
        a = j.find("axis")
        axis = np.array(_floats(a.get("xyz") if a is not None else None, "1 0 0"))
        axis = axis / np.linalg.norm(axis)
        child = j.find("child").get("link")
        lim = j.find("limit")
        lower = float(lim.get("lower", -math.pi)) if lim is not None else -math.pi
        upper = float(lim.get("upper", math.pi)) if lim is not None else math.pi
        by_child[child] = _Joint(
            lower=lower,
            upper=upper,
            name=j.get("name"),
            parent=j.find("parent").get("link"),
            child=child,
            origin=T,
            axis=axis,
            moving=j.get("type") in ("revolute", "continuous"),
        )
    return by_child


@dataclass(frozen=True)
class _Foot:
    unknowns: Tuple[str, ...]                 # ankle_pitch, ankle_roll, 막대 u1·u2 둘씩
    chains: Tuple[Tuple[Tuple[_Joint, ...], Tuple[_Joint, ...]], ...]  # (막대 끝, 볼) 짝 둘
    lower: np.ndarray   # 받아들일 해의 범위. 링키지는 같은 크랭크각에 조립 모양이 여럿
    upper: np.ndarray   # (발이 뒤집힌 해 등) -- 범위 밖이면 버림


class Linkage:
    """URDF 를 읽고, 크랭크각을 받아 발판 자세와 막대 각도를 돌려줌.

    발 하나에 미지수 6개 (ankle_pitch, ankle_roll, 막대 둘의 u1·u2) 와 조건 6식 (막대 끝 두 개가
    각각 볼에 닿음). 크랭크각만 있으면 됨 -- **발판 자세를 `huphy` 의 `ankle.py` 에서 받지
    않음.** 그쪽 링키지 치수가 학습 모델(MJCF)과 달라서, 그 값을 넣으면 막대 끝이 발에서
    수 mm ~ 2cm 어긋남. 여기서는 URDF(= 학습 모델) 치수로 풀어서 화면이 모델과 맞음.
    """

    def __init__(self, urdf: str, tol_m: float = 0.002) -> None:
        self.joints = parse(urdf)
        self.tol_m = tol_m
        by_name = {j.name: j for j in self.joints.values()}
        sides: Dict[str, list] = {}
        for name in sorted(by_name):
            m = _ROD.match(name)
            if not m:
                continue
            side, x = m.groups()
            u2 = f"{side}_rod_{x}_u2"
            if u2 not in by_name:
                raise ValueError(f"{u2} 가 URDF 에 없음")
            end, ball = f"{side}_rod_{x}_end", f"{side}_ball_{x}"
            for link in (end, ball):
                if link not in self.joints:
                    raise ValueError(
                        f"고리 점 {link} 가 URDF 에 없음 -- urdf/make_urdf.py 로 다시 만들 것")
            a, b = self._chain(end), self._chain(ball)
            # 공통 부분(뿌리 -> 정강이)은 양쪽에 같으므로 떼어 냄
            k = 0
            while k < min(len(a), len(b)) and a[k] is b[k]:
                k += 1
            sides.setdefault(side, []).append((name, u2, tuple(a[k:]), tuple(b[k:])))
        self.feet: List[_Foot] = []
        for side, rods in sorted(sides.items()):
            ankle = tuple(j.name for j in rods[0][3] if j.moving)  # 볼 쪽 사슬의 관절 = pitch, roll
            unknowns = ankle + tuple(n for r in rods for n in (r[0], r[1]))
            lo, hi = [], []
            for n in unknowns:
                j = by_name[n]
                if n in ankle:      # 발판: URDF 범위 + 17도 여유 (뒤집힌 해만 거름)
                    lo.append(j.lower - 0.3)
                    hi.append(j.upper + 0.3)
                else:               # 막대: 크랭크에서 직각 넘게 꺾이는 일은 없음
                    lo.append(-math.pi / 2)
                    hi.append(math.pi / 2)
            self.feet.append(_Foot(unknowns, tuple((r[2], r[3]) for r in rods),
                                   np.array(lo), np.array(hi)))
        self._guess: Dict[str, float] = {}
        self.worst_m = 0.0
        """마지막 `solve` 에서 가장 많이 어긋난 거리 (m). 크면 못 푼 것 -- 크랭크각이 모델 범위 밖."""

    @property
    def names(self) -> List[str]:
        """`solve` 가 채우는 관절 이름. 발목 pitch/roll 4개 + 막대 8개."""
        return [n for f in self.feet for n in f.unknowns]

    def _chain(self, link: str) -> List[_Joint]:
        out = []
        while link in self.joints:
            j = self.joints[link]
            out.append(j)
            link = j.parent
        return out[::-1]

    @staticmethod
    def _point(chain, q: Mapping[str, float]) -> np.ndarray:
        T = np.eye(4)
        for j in chain:
            T = T @ j.origin
            if j.moving:
                R = np.eye(4)
                R[:3, :3] = _axis_angle(j.axis, q.get(j.name, 0.0))
                T = T @ R
        return T[:3, 3]

    def solve(self, q: Mapping[str, float]) -> Dict[str, float]:
        """`q`: URDF 관절 이름 -> rad. 크랭크(`*__ankle_a/b`) 가 있어야 함.

        못 풀면 그 발은 직전 성공 값을 돌려줌 (화면이 튀지 않게).
        """
        out: Dict[str, float] = {}
        self.worst_m = 0.0
        qq = dict(q)
        for foot in self.feet:
            names = foot.unknowns

            def resid(v):
                qq.update(zip(names, v))
                return np.concatenate([
                    self._point(end, qq) - self._point(ball, qq) for end, ball in foot.chains])

            # 직전 해에서 시작. 못 풀거나 다른 조립 모양으로 가면 0 자세에서 한 번 더.
            for start in ([self._guess.get(n, 0.0) for n in names], [0.0] * len(names)):
                x, err = self._newton(resid, np.array(start))
                if np.any(x < foot.lower) or np.any(x > foot.upper):
                    err = math.inf
                if err <= self.tol_m:
                    break
            self.worst_m = max(self.worst_m, err)
            if err <= self.tol_m and np.all(np.isfinite(x)):
                self._guess.update(zip(names, (float(v) for v in x)))
            out.update({n: self._guess.get(n, 0.0) for n in names})
        return out

    @staticmethod
    def _newton(resid, x: np.ndarray) -> Tuple[np.ndarray, float]:
        """감쇠 가우스-뉴턴 (레벤버그-마콰트). (해, 가장 많이 어긋난 점의 거리 m).

        한 번에 움직이는 양을 줄여 멀리 튀지 않게 함. 각도는 -pi ~ pi 로 접음 --
        안 접으면 몇 바퀴 돈 같은 자세로 수렴할 수 있음.
        """
        r = resid(x)
        cost = float(r @ r)
        lam = 1e-3
        n = len(x)
        for _ in range(60):
            if cost < 1e-14:
                break
            J = np.empty((len(r), n))
            for i in range(n):
                d = np.zeros(n)
                d[i] = 1e-6
                J[:, i] = (resid(x + d) - r) / 1e-6
            A, g = J.T @ J, J.T @ r
            while True:
                step = np.linalg.solve(A + lam * np.eye(n), -g)
                step = np.clip(step, -0.3, 0.3)          # 한 번에 17도 넘게 안 감
                cand = (x + step + math.pi) % (2 * math.pi) - math.pi
                rc = resid(cand)
                cc = float(rc @ rc)
                if cc < cost:
                    x, r, cost = cand, rc, cc
                    lam = max(lam / 3, 1e-9)
                    break
                lam *= 4
                if lam > 1e6:
                    break
            if lam > 1e6:
                break
        if not np.all(np.isfinite(r)):
            return x, math.inf
        return x, float(np.max(np.linalg.norm(r.reshape(-1, 3), axis=1)))
