"""관절별 응답 지연 기록 -- **명령을 보낸 때부터 응답이 도착한 때까지**.

    지연 = 응답 도착 시각(수신 스레드가 찍음) - 그 명령 묶음을 보내기 직전 시각

`RobStrideBus.latency` 에 붙어서 모터마다 보낸 횟수와 응답의 지연을 모음. 끝나면
CSV 로 쓰고 터미널에 요약을 찍음. `tools/latency_viewer.html` 로 열면 관절별·전체
히스토그램이 나옴.


## 세 가지로 나눔

    ontime   이번 주기 수거에서 받은 것
    late     수거 대기를 넘겨 다음 주기에 들어온 것. 지연은 원래 명령 기준임
    lost     끝내 안 온 것. 보낸 수 - 받은 수

late 가 많으면 `--drain-ms` 를 늘릴 자리이고, lost 가 많으면 어댑터·배선 쪽임.


## CSV

    # drain_ms=4 hz=100            주석 줄. 뷰어가 기준선을 그리는 데 씀
    t,joint,latency_ms,status      한 줄이 응답 하나
    ,right_leg/knee,,lost          lost 는 개수만큼 빈 지연으로 씀

**수신 스레드가 없는 버스**(다리 하나)는 도착이 아니라 꺼낸 시각이 찍혀 지연이 조금
크게 나옴. 양다리(`--robot`)는 스레드가 켜져 있음.
"""

from __future__ import annotations

import csv
import time
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple

import numpy as np


class LatencyLog:
    """여러 버스의 지연을 관절 이름으로 모음."""

    def __init__(
        self,
        *,
        drain_ms: Optional[float] = None,
        hz: Optional[float] = None,
        skip_s: float = 0.0,
        active: bool = True,
    ) -> None:
        self.drain_ms = drain_ms
        self.hz = hz
        self.skip_s = max(0.0, float(skip_s))
        """켜진 뒤 첫 송신부터 이 초만큼은 세지 않음. 연결 직후 첫 주기들이 느려서
        (수신 스레드·USB 가 막 돌기 시작함) 최대값을 끌어올리는 것을 뺌."""
        self.joints: List[str] = []
        self.sent: Dict[str, int] = {}
        self.samples: Dict[str, List[Tuple[float, float, bool]]] = {}
        self._t0 = time.monotonic()
        self._active = bool(active)
        self._armed_at: Optional[float] = None
        self._counted: Dict[str, bool] = {}
        """관절 -> 마지막 송신을 셌는지. **센 송신의 응답만 셈** -- 켜고 끄는 경계에서
        송신만 세고 응답은 안 세면 가짜 유실이 생김."""

    # ---- 켜고 끄기 ---------------------------------------------------------
    def resume(self) -> None:
        """기록을 켬. 그 뒤 첫 송신부터 `skip_s` 를 다시 셈."""
        if not self._active:
            self._active = True
            self._armed_at = None

    def pause(self) -> None:
        """기록을 끔. 감쇠 정지처럼 재고 싶지 않은 구간 앞에서 부름."""
        self._active = False

    def on_settle(self, starting: bool) -> None:
        """`ControlLoop.on_settle` 에 그대로 붙임. 감쇠 동안만 끔."""
        self.pause() if starting else self.resume()

    def _recording(self) -> bool:
        if not self._active:
            return False
        now = time.monotonic()
        if self._armed_at is None:
            self._armed_at = now
        return now - self._armed_at >= self.skip_s

    # ---- 붙이기 -----------------------------------------------------------
    def attach(self, robot: Any) -> "LatencyLog":
        """다리 하나든 `Biped` 든 그 안의 버스에 전부 붙임."""
        for part in getattr(robot, "parts", None) or (robot,):
            bus = part.bus
            names = {
                mid: f"{part.id}/{part.config.joint_of(mid) or mid}"
                for mid in bus.motor_ids
            }
            for name in names.values():
                if name not in self.sent:
                    self.joints.append(name)
                    self.sent[name] = 0
                    self.samples[name] = []
            bus.latency = _BusSink(self, names)
        return self

    def detach(self, robot: Any) -> None:
        for part in getattr(robot, "parts", None) or (robot,):
            part.bus.latency = None

    # ---- 집계 -------------------------------------------------------------
    def counts(self, joint: str) -> Tuple[int, int, int]:
        """(제때, 늦음, 유실). 유실은 보낸 수에서 받은 수를 뺀 것."""
        rows = self.samples[joint]
        late = sum(1 for _, _, is_late in rows if is_late)
        ontime = len(rows) - late
        return ontime, late, max(0, self.sent[joint] - len(rows))

    def summary(self) -> str:
        """관절별 한 줄. 지연은 받은 것만으로 셈."""
        # 한글은 터미널에서 두 칸이라 폭을 손으로 맞춤.
        head = (
            f"  관절{'':18s} {'p50':>6s} {'p95':>6s} {'p99':>6s}   최대"
            f"    늦음   유실   (ms, 비율은 보낸 것 대비)"
        )
        lines = ["  응답 지연 (보낸 때부터 도착까지)", head]
        everything: List[float] = []
        totals = [0, 0, 0, 0]
        for joint in self.joints:
            lat = [ms for _, ms, _ in self.samples[joint]]
            everything += lat
            ontime, late, lost = self.counts(joint)
            sent = self.sent[joint]
            totals = [totals[0] + sent, totals[1] + ontime, totals[2] + late, totals[3] + lost]
            lines.append(_row(joint, lat, sent, late, lost))
        lines.append(_row("전체", everything, totals[0], totals[2], totals[3]))
        return "\n".join(lines)

    # ---- 저장 -------------------------------------------------------------
    def save(self, path: "str | Path") -> Path:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        meta = []
        if self.drain_ms is not None:
            meta.append(f"drain_ms={self.drain_ms:g}")
        if self.hz is not None:
            meta.append(f"hz={self.hz:g}")
        with path.open("w", newline="") as f:
            if meta:
                f.write("# " + " ".join(meta) + "\n")
            writer = csv.writer(f)
            writer.writerow(("t", "joint", "latency_ms", "status"))
            for joint in self.joints:
                for t, ms, late in self.samples[joint]:
                    writer.writerow((f"{t:.4f}", joint, f"{ms:.4f}", "late" if late else "ontime"))
                for _ in range(self.counts(joint)[2]):
                    writer.writerow(("", joint, "", "lost"))
        return path

    def __len__(self) -> int:
        return sum(self.sent.values())


class _BusSink:
    """버스 하나의 모터 id 를 관절 이름으로 바꿔 넘김. 버스가 부르는 쪽."""

    __slots__ = ("log", "names")

    def __init__(self, log: LatencyLog, names: Dict[int, str]) -> None:
        self.log = log
        self.names = names

    def sent(self, mid: int) -> None:
        name = self.names.get(mid)
        if name is None:
            return
        counted = self.log._recording()
        self.log._counted[name] = counted
        if counted:
            self.log.sent[name] += 1

    def received(self, mid: int, latency_s: float, late: bool) -> None:
        name = self.names.get(mid)
        if name is None or not self.log._counted.get(name):
            return
        self.log._counted[name] = False      # 응답 하나만 셈. 중복 수신은 버림
        t = time.monotonic() - self.log._t0
        self.log.samples[name].append((t, latency_s * 1000.0, late))


def _row(name: str, lat: Iterable[float], sent: int, late: int, lost: int) -> str:
    a = np.asarray(list(lat), dtype=float)
    if a.size:
        p50, p95, p99 = np.percentile(a, (50, 95, 99))
        body = f"{p50:6.2f} {p95:6.2f} {p99:6.2f} {a.max():6.2f}"
    else:
        body = f"{'–':>6s} {'–':>6s} {'–':>6s} {'–':>6s}"
    pct = (lambda n: f"{100.0 * n / sent:5.1f}%") if sent else (lambda n: "    –")
    pad = 22 - sum(2 if ord(ch) > 0x1100 else 1 for ch in name)
    return f"  {name}{' ' * max(1, pad)} {body}  {pct(late):>6s} {pct(lost):>6s}"


def default_path(directory: "str | Path", name: str) -> Path:
    """`<폴더>/latency_<이름>_<날짜_시각>.csv`."""
    stamp = time.strftime("%Y%m%d_%H%M%S")
    return Path(directory) / f"latency_{name}_{stamp}.csv"
