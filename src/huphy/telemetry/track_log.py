"""추종 기록 -- 매 주기 **관절 목표각과 실측각**을 CSV 로 남김. `huphy-test --log`.

    t              첫 주기부터 흐른 초
    tgt/<관절>     실제로 나간 관절 목표각 (도). 가드가 자른 뒤 값
    pos/<관절>     그 명령에 대한 실측 관절각 (도)
    imu/<이름>/<값> 그 주기의 IMU 값. 붙은 IMU 가 없으면 열이 없음 (`IMU_FIELDS`)

`policy_log` 에서 정책 칸(obs/act)을 뺀 것과 같은 모양이라 `tools/policy_viewer.html`
로 그대로 열림. 목표가 고정(`zero`, `pose`)이면 실측이 거기로 어떻게 다가가고 얼마나
처지는지가 보임.

## 가동범위 줄

첫 줄이 `#` 로 시작하는 범위 줄임 (`# limits right_leg/knee=6:114 ...`). 뷰어가 관절
그래프를 가동범위로 정규화하는 데 씀. pandas 로 읽을 때는 `comment="#"` 을 줄 것.

    모터 관절   캘리브레이션의 limits_deg
    발목 관절   ankle_pitch/ankle_roll 은 IK 시험 범위 (`AnkleEnvelope`)


## 목표는 실제로 나간 값

`robot` 을 주면 `tgt` 는 motion 이 낸 값이 아니라 `robot.last_sent` 임 -- 안전 가드가
한계 안쪽으로 자른 뒤 **모터에 실제로 나간** 관절 목표. motion 의 값을 쓰면 가드가
자른 관절(예: 한계 6~114도인 무릎에 0도를 시킴 -> 9도로 나감)이 실제로는 잘 따라가는데
오차가 9도로 보임.

한 줄은 매 주기 motion 이 불릴 때 만듦. 그때 `last_sent` 는 **직전 주기에 나간 명령**
이고 관찰(`pos`)은 **그 명령 뒤에 수거한 응답**이라, 한 줄 안의 둘이 같은 명령에 대한
짝임. 명령이 아직 한 번도 안 나간 첫 주기는 남기지 않음. 가드가 버린 관절은 그
주기에 빈 값(NaN). 응답이 빠진 주기는 마지막으로 받은 실측이 그대로 남음.

`robot` 이 없으면 (테스트) motion 이 낸 목표를 그대로 씀.

## IMU

`robot` 에 IMU 가 붙어 있으면 줄 끝에 `imu/<이름>/<값>` 을 붙임. 값은 텔레메트리
(`snapshot.build_imu`)와 같고 그중 `IMU_FIELDS` 만 고름. 벤더 수신 스레드가 받아 둔
것을 꺼낼 뿐이라 새로 통신하지 않음. 열은 첫 줄에서 정해짐.

## 목표가 없는 줄 -- 손으로 움직일 때 (`huphy-test watch`)

토크를 끈 관찰 모드에서는 motion 이 안 불림. 그때는 `ObservationTap` 을 제어 루프의
텔레메트리 자리에 꽂아 매 주기 관찰을 `sample(observation)` 으로 넘김. 목표가 없으니
`tgt/` 열이 없고 `t`, `pos/<관절>`, `imu/...` 만 남음. 관절 순서는 `order` 로 미리 줌.

메모리에 쌓고 끝날 때 씀. 제어 루프 안에서 파일을 쓰지 않음. 기록 중 예외는 세기만
함 -- 관측이 제어를 멈추면 안 됨.
"""

from __future__ import annotations

import csv
import logging
import time
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

logger = logging.getLogger(__name__)


IMU_FIELDS = (
    "roll", "pitch", "yaw",
    "gx", "gy", "gz",
    "grav_x", "grav_y", "grav_z",
    "ax", "ay", "az",
    "age",
)
"""IMU 에서 남기는 값. 이름과 단위는 `snapshot.IMU_FIELDS` 와 같음.

    roll pitch yaw      자세 (도). 벤더가 extra 로 냄 -- 표시용
    gx gy gz            각속도 (도/초)
    grav_x/y/z          중력방향 단위벡터. 정책이 보는 값
    ax ay az            가속도 (m/s^2)
    age                 받은 지 얼마나 됐나 (ms). 못 받았으면 -1
"""


def imu_values(robot: Any) -> Dict[str, float]:
    """`imu/<이름>/<값>` -> 값. IMU 가 없거나 못 읽으면 빈 사전."""
    if robot is None:
        return {}
    from .snapshot import build_imu        # 순환 import 를 피함

    try:
        values = build_imu(robot, t=0.0)
    except Exception:
        return {}
    return {
        key: float(value)
        for key, value in values.items()
        if key.startswith("imu/") and key.rsplit("/", 1)[-1] in IMU_FIELDS
    }


class TrackLog:
    """한 번 실행의 목표·실측 기록. 관절 순서는 첫 목표의 키 순서를 따름."""

    def __init__(self, order: Optional[Sequence[str]] = None, *, robot: Any = None) -> None:
        self.order: Optional[tuple] = tuple(order) if order is not None else None
        self.robot = robot
        self.limits = joint_limits(robot)
        self.imu_cols: Optional[tuple] = None
        self.has_tgt = True                  # 첫 줄에서 정해짐. 관찰만 받으면 False
        self.rows: List[List[float]] = []
        self.errors = 0
        self._t0: Optional[float] = None

    def __len__(self) -> int:
        return len(self.rows)

    @property
    def columns(self) -> tuple:
        return columns(self.order, self.imu_cols, self.has_tgt)

    def wrap_motion(self, motion: Callable[[float, Dict[str, Any]], Optional[Dict[str, float]]]):
        """motion 을 감쌈. 낸 목표와 받은 관찰로 한 줄을 만듦."""

        def logged(t: float, observation: Dict[str, Any]):
            targets = motion(t, observation)
            self.sample(observation, targets or {})
            return targets

        return logged

    def sample(
        self, observation: Dict[str, Any], targets: Optional[Dict[str, float]] = None,
        *, sent: Optional[Dict[str, float]] = None,
    ) -> None:
        """한 줄. `targets` 가 None 이면 관찰만 (목표 열 없음). 예외는 세기만 함.

        `sent` 를 주면 그것을 목표 칸에 그대로 씀 (빈 사전이면 빈 칸) -- 토크를 켰다
        껐다 하는 `studio` 처럼 목표 칸은 늘 있되 없을 때가 있는 경우.
        """
        try:
            self._record(observation, targets, sent)
        except Exception as e:
            self._fail(e)

    def _record(self, observation, targets, sent=None) -> None:
        if targets is not None and self.order is None and targets:
            self.order = tuple(targets)
        if not self.order:
            return
        row, imu_cols = build_row(self.robot, self.order, observation, targets, self.imu_cols, sent)
        if row is None:
            return
        if not self.rows and self._t0 is None:
            self.has_tgt = targets is not None or sent is not None
        self.imu_cols = imu_cols
        now = time.monotonic()
        if self._t0 is None:
            self._t0 = now
        row[0] = now - self._t0
        self.rows.append(row)

    def _fail(self, e: Exception) -> None:
        self.errors += 1
        if self.errors == 1:
            logger.warning("추종 기록 실패 (제어는 계속함): %s", e)

    def save(self, path: "str | Path") -> Path:
        """CSV 로 씀. 폴더가 없으면 만듦."""
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", newline="") as f:
            write_limits(f, self.limits)
            writer = csv.writer(f)
            writer.writerow(self.columns)
            for row in self.rows:
                writer.writerow(f"{v:.6g}" for v in row)
        return path


def joint_limits(robot: Any) -> Dict[str, Tuple[float, float]]:
    """관절 이름 -> (최소, 최대) 도. 기록의 열 이름과 같은 이름을 씀.

    양다리(`Biped`)면 `right_leg/knee`, 다리 하나면 `knee`. 한계가 안 정해진 관절은
    빠짐. 실패해도 빈 사전 -- 범위 줄이 없을 뿐 기록은 그대로 남아야 함.
    """
    out: Dict[str, Tuple[float, float]] = {}
    if robot is None:
        return out
    try:
        parts = getattr(robot, "parts", None)
        legs = [(p.id + "/", p) for p in parts] if parts else [("", robot)]
        for prefix, leg in legs:
            config = getattr(leg, "config", None)
            for name, motor in (getattr(config, "motors", None) or {}).items():
                if motor.limits_deg is not None:
                    out[prefix + name] = (float(motor.limits_deg[0]), float(motor.limits_deg[1]))
            envelope = getattr(getattr(leg, "kinematics", None), "envelope", None)
            if envelope is not None:
                out[prefix + "ankle_pitch"] = tuple(map(float, envelope.pitch_deg))
                out[prefix + "ankle_roll"] = tuple(map(float, envelope.roll_deg))
    except Exception as e:                    # 관측이 기록을 막으면 안 됨
        logger.warning("가동범위를 못 읽음 (범위 줄 없이 씀): %s", e)
        return {}
    return out


def write_limits(f, limits: Dict[str, Tuple[float, float]]) -> None:
    """`# limits 이름=최소:최대 ...` 한 줄. 범위가 없으면 안 씀."""
    if limits:
        f.write("# limits " + " ".join(f"{k}={lo:g}:{hi:g}" for k, (lo, hi) in limits.items()) + "\n")


def columns(order: Optional[Sequence[str]], imu_cols: Optional[Sequence[str]], has_tgt: bool) -> tuple:
    """줄의 열 이름. `t`, (`tgt/...`), `pos/...`, `imu/...`."""
    order = tuple(order or ())
    tgt = tuple(f"tgt/{j}" for j in order) if has_tgt else ()
    return ("t",) + tgt + tuple(f"pos/{j}" for j in order) + tuple(imu_cols or ())


def build_row(
    robot: Any,
    order: Sequence[str],
    observation: Dict[str, Any],
    targets: Optional[Dict[str, float]],
    imu_cols: Optional[Sequence[str]],
    sent: Optional[Dict[str, float]] = None,
):
    """`(줄, imu 열)`. 줄의 첫 칸(t)은 0 으로 비워 둠 -- 시각은 부르는 쪽이 채움.

    `targets` 가 None 이면 관찰만 받는 줄 (목표 칸 없음). 목표가 있는데 아직 아무것도
    안 나갔으면 줄이 None. `imu_cols` 가 None 이면 이번 IMU 값으로 열을 정함.
    """
    nan = float("nan")
    row: List[float] = [0.0]
    if sent is not None:
        row += [float(sent.get(j, nan)) for j in order]
    elif targets is not None:
        if not targets:
            return None, imu_cols
        sent = sent_targets(robot, targets)
        if sent is None:
            return None, imu_cols            # 아직 한 번도 안 나감
        row += [float(sent.get(j, nan)) for j in order]
    row += [float(observation.get(f"{j}.pos", nan)) for j in order]
    imu = imu_values(robot)
    if imu_cols is None:
        imu_cols = tuple(imu)
    row += [imu.get(c, nan) for c in imu_cols]
    return row, imu_cols


class ObservationTap:
    """제어 루프의 `telemetry` 자리에 꽂아 매 주기 관찰을 `sample()` 로 넘김.

    토크를 끈 관찰 모드(`Mode.OBSERVE`)에서는 motion 이 안 불려서 `wrap_motion` 을 쓸
    수 없음. 루프는 모드와 무관하게 매 주기 `telemetry.record()` 를 부르므로 거기에
    붙음. 기존 텔레메트리가 있으면 `inner` 로 받아 같이 부름.
    """

    def __init__(
        self, robot: Any, sinks: Sequence[Any], inner: Any = None,
        targets: Optional[Callable[[], Optional[Dict[str, float]]]] = None,
    ) -> None:
        self.robot = robot
        self.sinks = tuple(sinks)
        self.inner = inner
        self.targets = targets
        """주면 매 주기 목표 칸을 이것으로 채움 (None 이면 빈 칸). `studio` 가 씀."""

    def open(self) -> None:
        if self.inner is not None:
            self.inner.open()

    def close(self) -> None:
        if self.inner is not None:
            self.inner.close()

    def record(self, **kwargs) -> None:
        observation = self.robot.get_observation()
        sent = None
        if self.targets is not None:
            sent = self.targets() or {}        # 목표 칸은 두되 없으면 빈 값 (토크 없음)
        for sink in self.sinks:
            if sent is None:
                sink.sample(observation)
            else:
                sink.sample(observation, sent=sent)
        if self.inner is not None:
            self.inner.record(**kwargs)


def sent_targets(robot: Any, targets: Dict[str, float]) -> Optional[Dict[str, float]]:
    """줄에 적을 목표. `robot` 이 있으면 직전 주기에 실제로 나간 것, 없으면 `targets`.

    `robot` 은 있는데 아직 아무것도 안 나갔으면 None -- 그 줄은 남기지 않음.
    """
    if robot is None:
        return targets
    sent = getattr(robot, "last_sent", None)
    return dict(sent) if sent else None


def default_path(directory: "str | Path", name: str) -> Path:
    """`<폴더>/track_<이름>_<날짜_시각>.csv`. 실행마다 새 파일."""
    stamp = time.strftime("%Y%m%d_%H%M%S")
    return Path(directory) / f"track_{name}_{stamp}.csv"
