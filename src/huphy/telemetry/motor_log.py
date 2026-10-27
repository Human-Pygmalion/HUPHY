"""모터 전류·전압·온도 기록 -- `--probe` 를 켰을 때만.

`Leg.probe` 가 주기마다 명령에 파라미터 읽기(Command 14)를 몇 개씩 실어 보내고,
`RobStrideBus` 가 응답을 받으면 여기로 넘김 (`bus.param_sink`). 읽는 것은 둘:

    iq     0x701A  iq 필터값 (A). 토크를 만드는 전류
    vbus   0x701C  모터가 보는 버스 전압 (V)

전류를 읽을 때마다 그 순간의 **온도와 토크**(상태 프레임에 매번 실려 오는 값)도 같이
남김 -- 온도는 그동안 어디에도 기록되지 않았고, 토크는 모터가 iq 에서 계산한 값이라
둘을 같이 봐야 토크 상수가 맞는지 보임.


## JSON

    {"run": {...}, "meta": {"hz": 100, "probe": 2, "counts": {...}},
     "columns": ["t", "joint", "kind", "value"],
     "rows": [[0.0123, "right_leg/knee", "iq", 1.52],
              [0.0123, "right_leg/knee", "temp", 31.2], ...]}

`t` 는 응답이 **도착한 시각**임 (기록을 붙인 때부터 초). 늦게 온 응답도 그 시각으로
들어감 -- CAN id 로 누구 것인지 알아서 버릴 이유가 없음.

`counts` 는 관절마다 `{sent, ok, late, lost}`. lost 가 많으면 `--drain-ms` 를 늘리거나
`--probe` 를 줄일 자리임.
"""

from __future__ import annotations

import time
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Tuple

from . import run_name

KINDS = {0x701A: "iq", 0x701C: "vbus"}


class MotorLog:
    """여러 버스의 파라미터 읽기를 관절 이름으로 모음."""

    def __init__(self, *, hz: Optional[float] = None, probe: Optional[int] = None) -> None:
        self.hz = hz
        self.probe = probe
        self.joints: List[str] = []
        self.rows: List[Tuple[float, str, str, float]] = []
        self.errors = 0
        self._buses: List[Tuple[Any, Dict[int, str]]] = []
        self._t0 = time.monotonic()

    # ---- 붙이기 -----------------------------------------------------------
    def attach(self, robot: Any) -> "MotorLog":
        """다리 하나든 `Biped` 든 그 안의 버스에 전부 붙임."""
        for part in getattr(robot, "parts", None) or (robot,):
            bus = part.bus
            names = {
                mid: f"{part.id}/{part.config.joint_of(mid) or mid}"
                for mid in bus.motor_ids
            }
            for name in names.values():
                if name not in self.joints:
                    self.joints.append(name)
            bus.param_sink = _BusSink(self, bus, names)
            self._buses.append((bus, names))
        return self

    def detach(self) -> None:
        for bus, _ in self._buses:
            bus.param_sink = None

    def _add(self, stamp: float, joint: str, kind: str, value: float) -> None:
        self.rows.append((stamp - self._t0, joint, kind, float(value)))

    # ---- 집계 -------------------------------------------------------------
    def counts(self) -> Dict[str, Dict[str, int]]:
        """관절 -> `{sent, ok, late, lost}`. 버스가 센 것을 그대로 옮김."""
        out: Dict[str, Dict[str, int]] = {}
        for bus, names in self._buses:
            for mid, name in names.items():
                c = dict(bus.param_counts.get(mid, {}))
                c["lost"] = max(0, c.get("sent", 0) - c.get("ok", 0) - c.get("late", 0))
                out[name] = c
        return out

    def summary(self) -> str:
        """관절별 한 줄. 최대 |iq|, 평균 |iq|, 최고 온도, 전압 범위, 읽기 집계."""
        by: Dict[str, Dict[str, List[float]]] = {j: {} for j in self.joints}
        for _, joint, kind, value in self.rows:
            by.setdefault(joint, {}).setdefault(kind, []).append(value)
        counts = self.counts()

        head = (
            f"  관절{'':18s} {'|iq|최대':>8s} {'|iq|평균':>8s} {'온도최고':>8s}"
            f" {'전압최저':>8s} {'전압최고':>8s}   읽기 ok/늦음/유실"
        )
        lines = ["  모터 전류·전압·온도 (--probe)", head]
        for joint in self.joints:
            v = by.get(joint, {})
            iq = [abs(x) for x in v.get("iq", [])]
            temp = v.get("temp", [])
            vb = v.get("vbus", [])
            c = counts.get(joint, {})
            f = lambda xs, fn, unit: f"{fn(xs):7.2f}{unit}" if xs else f"{'-':>8s}"
            pad = 22 - sum(2 if ord(ch) > 0x1100 else 1 for ch in joint)
            lines.append(
                f"  {joint}{' ' * max(1, pad)} {f(iq, max, 'A')} "
                f"{f(iq, lambda a: sum(a) / len(a), 'A')} {f(temp, max, 'C')} "
                f"{f(vb, min, 'V')} {f(vb, max, 'V')}   "
                f"{c.get('ok', 0)}/{c.get('late', 0)}/{c.get('lost', 0)}"
            )
        return "\n".join(lines)

    # ---- 저장 -------------------------------------------------------------
    def save(self, path: "str | Path", run: Mapping[str, Any] = {}) -> Path:
        """JSON 으로 씀. `run` 은 `run_name.header(...)` 가 만든 사전."""
        meta: Dict[str, Any] = {"counts": self.counts()}
        if self.hz is not None:
            meta["hz"] = self.hz
        if self.probe is not None:
            meta["probe"] = self.probe
        rows = [[round(t, 4), joint, kind, round(value, 4)]
                for t, joint, kind, value in self.rows]
        return run_name.write(
            path, run,
            meta=meta,
            columns=["t", "joint", "kind", "value"],
            rows=rows,
        )

    def __len__(self) -> int:
        return len(self.rows)


class _BusSink:
    """버스 하나의 모터 id 를 관절 이름으로 바꿔 넘김. 버스가 부르는 쪽."""

    __slots__ = ("log", "bus", "names")

    def __init__(self, log: MotorLog, bus: Any, names: Dict[int, str]) -> None:
        self.log = log
        self.bus = bus
        self.names = names

    def sample(self, mid: int, index: int, value: float, stamp: float, late: bool) -> None:
        name = self.names.get(mid)
        kind = KINDS.get(index)
        if name is None or kind is None:
            return
        try:
            self.log._add(stamp, name, kind, value)
            if kind == "iq":
                # 같은 순간의 온도·토크. 상태 프레임에서 매 주기 갱신된 값.
                state = self.bus.state(mid)
                if state.is_valid:
                    self.log._add(stamp, name, "temp", state.temp_c)
                    self.log._add(stamp, name, "tau", state.torque_nm)
        except Exception:  # noqa: BLE001 - 기록 때문에 제어가 멈추면 안 됨
            self.log.errors += 1


def default_path(directory: "str | Path", name: str, parts=()) -> Path:
    """`<폴더>/motor/<월일>_motor_<이름>[_<인자들>].json`. 규칙은 `run_name` 에 있음."""
    return run_name.path(directory, "motor", name, parts)
