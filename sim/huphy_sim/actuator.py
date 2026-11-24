"""모터 한 대가 내는 토크. **MuJoCo 를 모름** -- 순수 계산이라 시험하기 쉬움.

실물 RobStride 는 MIT 프레임 다섯 칸(`q`, `dq`, `kp`, `kd`, `tau_ff`)을 받아
펌웨어가 토크를 만듦 (`motors/robstride/bus.py` 의 `MitCommand`):

    tau = kp*(q - 현재각) + kd*(dq - 현재속도) + tau_ff

그래서 여기도 **명령에 실려 온 kp/kd** 로 계산함. MJCF 액추에이터의 게인을 쓰지
않는 이유: 그 값은 학습 설정의 것이고, 실물은 `robot.yaml` 의 게인으로 돎. 실물
게인으로 정책을 따라갈 수 있는지를 보려면 명령에 온 값을 써야 함.

## 토크-속도 곡선

모터는 빨리 돌 때 토크를 덜 냄. 48 V 실측 곡선이
`sim/assets/motor_spec/RS0*_TN_curve_48V.csv` 에 있음 (출처: 학습 레포
`pygmalion_mjlab/assets/motor_spec/`, 값은 출력축 기준).

구동(토크와 속도가 같은 부호)일 때만 곡선이 상한이 됨. 제동(부호가 반대)일 때는
피크 토크까지 낼 수 있음 -- 모터가 발전기로 돌아 전기를 되돌리는 쪽이라 속도가
제약이 되지 않음. 학습 쪽 `tn_actuator.py` 의 `tn_clamp` 와 같은 규칙임.
"""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Tuple

Curve = Tuple[Tuple[float, float], ...]
"""((속도 rad/s, 최대 토크 Nm), ...). 속도 오름차순. 첫 토크가 피크."""

NO_LOAD_RPM = 200.0
"""곡선 끝에 붙이는 무부하 점. 이 속도에서 토크 0.

CSV 는 측정 구간(120 rpm 부터)만 담고 있어서 그 위가 비어 있음. 학습 쪽
`robot.py:tn_curve` 가 같은 점을 붙임 -- 안 붙이면 측정 구간 밖에서 상한이
마지막 측정값으로 평탄해져 실제보다 센 모터가 됨.
"""

RPM_TO_RAD_S = 2.0 * math.pi / 60.0


def load_curve(path: Path) -> Curve:
    """`speed_rpm,torque_Nm` CSV 를 읽어 (rad/s, Nm) 로 바꿈. 무부하 점을 붙임."""
    with open(path, newline="") as f:
        rows = [(float(r["speed_rpm"]), float(r["torque_Nm"])) for r in csv.DictReader(f)]
    rows.append((NO_LOAD_RPM, 0.0))
    curve = tuple((rpm * RPM_TO_RAD_S, torque) for rpm, torque in rows)
    speeds = [w for w, _ in curve]
    if any(b <= a for a, b in zip(speeds, speeds[1:])):
        raise ValueError(f"{path}: 속도가 오름차순이어야 함")
    return curve


def load_curves(folder: Path) -> Dict[str, Curve]:
    """모델 이름 -> 곡선. 파일 이름의 앞머리가 모델 이름임 (`RS03_TN_...`)."""
    return {
        p.name.split("_", 1)[0]: load_curve(p)
        for p in sorted(folder.glob("*_TN_curve_*.csv"))
    }


def tn_limit(curve: Curve, speed: float) -> float:
    """그 속도에서 낼 수 있는 최대 토크 (Nm). `speed` 는 **크기**(0 이상).

    첫 측정 속도 아래는 평탄하게 피크를 씀 (측정이 없음). 마지막 점 이상은 0 임.
    """
    speed = max(float(speed), curve[0][0])
    if speed >= curve[-1][0]:
        return 0.0
    for (w0, t0), (w1, t1) in zip(curve, curve[1:]):
        if speed <= w1:
            f = (speed - w0) / (w1 - w0) if w1 > w0 else 0.0
            return t0 + f * (t1 - t0)
    return 0.0


def clamp_to_curve(tau: float, speed: float, curve: Curve) -> float:
    """토크를 토크-속도 곡선 안으로 깎음. `speed` 는 **부호 있는** 속도.

    구동 쪽만 곡선이 걸림. 제동 쪽은 피크까지 허용함.
    """
    peak = curve[0][1]
    upper = tn_limit(curve, speed) if speed >= 0.0 else peak
    lower = -tn_limit(curve, -speed) if speed < 0.0 else -peak
    return min(max(float(tau), lower), upper)


@dataclass(frozen=True)
class Torque:
    """토크 한 번의 내역. **자른 뒤 값만 보면 포화를 알 수 없어서** 상한도 같이 냄."""

    raw: float
    """자르기 전 (Nm). PD 와 `tau_ff` 를 더한 값."""

    applied: float
    """자른 뒤 (Nm). `data.ctrl` 에 들어가는 값."""

    limit: float
    """`raw` 가 미는 방향에서의 상한 (Nm, **크기**). 고정 상한과 곡선 중 낮은 쪽."""

    source: str
    """무엇에 잘렸나. `""` 안 잘림, `"effort"` 고정 상한, `"curve"` 토크-속도 곡선."""

    @property
    def clipped(self) -> bool:
        return bool(self.source)


def torque_detail(
    *,
    kp: float,
    kd: float,
    target_rad: float,
    target_vel_rad_s: float,
    pos_rad: float,
    vel_rad_s: float,
    tau_ff: float,
    curve: Curve,
    effort_limit: float,
) -> Torque:
    """`torque_nm` 과 같은 계산인데 **상한과 잘린 이유까지** 냄.

    `torque_nm` 은 이것의 `applied` 임. 두 벌로 두지 않으려고 그쪽이 이것을 부름.

    상한을 `raw` 가 미는 방향에서만 내는 이유: 곡선은 비대칭임 (구동 쪽은 곡선,
    제동 쪽은 피크). 반대 방향 상한은 그 주기에 쓰이지 않은 값이라 같이 내면
    기록에서 어느 쪽이 걸린 것인지 되짚을 수 없음.
    """
    raw = (
        float(kp) * (float(target_rad) - float(pos_rad))
        + float(kd) * (float(target_vel_rad_s) - float(vel_rad_s))
        + float(tau_ff)
    )
    effort = abs(float(effort_limit))
    speed = float(vel_rad_s)
    peak = curve[0][1]
    if raw >= 0.0:
        from_curve = tn_limit(curve, speed) if speed >= 0.0 else peak
    else:
        from_curve = tn_limit(curve, -speed) if speed < 0.0 else peak
    limit = min(effort, from_curve)
    applied = min(max(raw, -limit), limit)
    source = "" if applied == raw else ("effort" if effort <= from_curve else "curve")
    return Torque(raw=raw, applied=applied, limit=limit, source=source)


def torque_nm(
    *,
    kp: float,
    kd: float,
    target_rad: float,
    target_vel_rad_s: float,
    pos_rad: float,
    vel_rad_s: float,
    tau_ff: float,
    curve: Curve,
    effort_limit: float,
) -> float:
    """MIT 프레임 한 장이 만드는 토크 (Nm).

    `kp`/`kd` 는 프레임에 실려 온 값임 (Nm/rad, Nm*s/rad -- 출력축 기준).
    `effort_limit` 은 모델의 토크 상한(`motors/robstride/tables.py` 의 `tmax_nm`)임.
    """
    return torque_detail(
        kp=kp, kd=kd, target_rad=target_rad, target_vel_rad_s=target_vel_rad_s,
        pos_rad=pos_rad, vel_rad_s=vel_rad_s, tau_ff=tau_ff,
        curve=curve, effort_limit=effort_limit,
    ).applied
