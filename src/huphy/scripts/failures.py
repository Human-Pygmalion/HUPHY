"""실행이 끝난 뒤 무엇이 실패했는지 한 번에 찍음.

텔레메트리를 켜지 않고도 "이 주기로 돌렸을 때 몇 번 빠졌나" 를 보려는 것임.
`huphy-test` 와 `huphy-run` 이 루프가 끝난 뒤에 부름.

    주기        평균 Hz, 밀린 횟수, 가장 늦은 주기
    수거 대기   응답을 기다린 최대 시간과 주기 대비 비율
    무응답      응답이 하나라도 빠진 주기 수
    CAN         채널별 송신 실패, 수신 실패, 대기 시간 초과, 큐가 차서 버린 것
    모터별      명령한 횟수 대비 응답이 없던 횟수. 빠진 모터만
    가드        한계·점프로 자른 횟수, 거부한 횟수


## 새로 재지 않음

전부 이미 세고 있던 값을 모아 읽기만 함. 루프는 `LoopStats`, 버스는 `CanCounters`,
다리는 `link_counts()` 와 `GuardCounters` 를 들고 있음.

**연결한 뒤부터의 누적**임. 연결 직후의 상태 조회와 영자세로 옮기는 구간이 같이
들어감 -- 실행 하나를 통째로 본 결과임.


## 다리 하나든 양다리든

합성 로봇이면 `parts` 로 팔다리를 돌고, 아니면 자기 자신 하나로 봄. 텔레메트리의
`snapshot.parts` 와 같은 방식임.
"""

from __future__ import annotations

from typing import Any, Dict, List, Optional, Tuple

from . import table

CAN_FIELDS: Tuple[Tuple[str, str], ...] = (
    ("frames_sent", "보냄"),
    ("frames_received", "받음"),
    ("tx_errors", "송신실패"),
    ("rx_errors", "수신실패"),
    ("drain_timeouts", "대기초과"),
    ("rx_dropped", "버림"),
)
"""`CanCounters` 필드 -> 화면 이름. 순서가 열 순서임.

    보냄/받음  나간 프레임과 들어온 프레임. **둘을 비교하는 것이 요점임**
    대기초과   기다린 개수를 못 채우고 수거 대기 시간을 다 쓴 횟수
    버림       수신 스레드 큐가 차서 버린 프레임 (수신 스레드를 켰을 때만)
"""

MISSING_LABEL = "미수신"
"""보냄 - 받음. 표에 계산해서 넣는 칸.

MIT 모드는 명령을 받으면 반드시 응답하므로 **정상이면 0 에 가까움.**

    0 에 가까움   유실은 없음. 늦게 온 것이라 대기초과·무응답이 집계에 잡힌 것
    크게 양수     실제로 안 돌아옴. 배선·전원·프로토콜 쪽

끝날 때 큐에 남아 있던 것과 정지 절차에서 수거 없이 나간 것은 안 세므로, 실행
길이와 무관하게 **몇 개는 늘 남음.** 그래서 개수가 아니라 비율로 판정함.
"""

LOST_WARN_RATIO = 0.01
"""미수신이 보낸 것의 이 비율을 넘으면 유실로 봄.

끝자락의 몇 개는 늘 남으므로 짧은 실행에서 비율이 튐 -- `LOST_WARN_FLOOR` 개는
넘어야 판정함.
"""

LOST_WARN_FLOOR = 32
"""이 개수 이하는 비율과 무관하게 정상으로 봄. 정지 절차에서 나가는 몫."""

DRAIN_WARN_RATIO = 0.5
"""수거 대기가 주기의 이 비율을 넘으면 경고함.

양다리는 다리마다 따로 기다려서 최악 두 배가 되고, 계산할 시간도 남아야 함.
"""


def parts(robot: Any) -> Tuple[Any, ...]:
    """합성 로봇이면 팔다리들, 아니면 자기 자신 하나."""
    return tuple(getattr(robot, "parts", ())) or (robot,)


def _name(robot: Any, part: Any, motor: str) -> str:
    """모터 이름. 합성 로봇이면 앞에 팔다리를 붙임."""
    return f"{part.id}/{motor}" if getattr(robot, "parts", None) else motor


def drain_line(drain_s: float, hz: float, *, legs: int = 1) -> str:
    """수거 대기 한 줄. 주기 대비 비율과, 넘치면 경고를 같이 냄."""
    period_ms = 1000.0 / hz if hz > 0 else 0.0
    drain_ms = drain_s * 1000.0
    worst_ms = drain_ms * legs
    ratio = worst_ms / period_ms if period_ms > 0 else 0.0
    text = f"{drain_ms:g}ms"
    if legs > 1:
        text += f" x {legs}다리 = 최악 {worst_ms:g}ms"
    text += f"  (주기 {period_ms:.1f}ms 의 {ratio * 100:.0f}%)"
    if ratio > DRAIN_WARN_RATIO:
        text += "  ** 크게 잡음. 응답이 늦으면 주기가 밀림 **"
    return text


def report(robot: Any, stats: Any, *, drain_s: Optional[float] = None) -> str:
    """실패 집계를 여러 줄 문자열로.

    `drain_s` 를 안 주면 첫 팔다리 버스가 든 값을 읽음 (`RobStrideBus.drain_s`).
    """
    legs = parts(robot)
    lines: List[str] = []
    if drain_s is None:
        drain_s = float(getattr(getattr(legs[0], "bus", None), "drain_s", 0.0))

    target = f" / 목표 {stats.target_hz:.0f}Hz" if stats.target_hz else ""
    lines.append(
        f"  주기       {stats.cycles}회 {stats.total_s:.1f}초  "
        f"평균 {stats.mean_hz:.1f}Hz{target}  "
        f"밀림 {stats.overruns}회  최악 {stats.worst_dt_ms:.1f}ms"
    )
    lines.append(
        "  수거 대기  "
        + drain_line(drain_s, stats.target_hz or stats.mean_hz, legs=len(legs))
    )
    share = (stats.missing_cycles / stats.cycles * 100.0) if stats.cycles else 0.0
    lines.append(f"  무응답     {stats.missing_cycles}주기 ({share:.1f}%)")

    # ---- CAN ---------------------------------------------------------------
    lines.append("")
    labels = [label for _, label in CAN_FIELDS]
    labels.insert(2, MISSING_LABEL)          # 보냄·받음 바로 뒤
    lines.append(
        "  " + table.header(("채널", 10, "<"), *[(label, 9) for label in labels])
    )
    lost = 0
    for part in legs:
        counters = getattr(getattr(getattr(part, "bus", None), "bus", None), "counters", None)
        channel = getattr(getattr(part, "config", None), "channel", part.id)
        values = [getattr(counters, field, 0) for field, _ in CAN_FIELDS]
        values.insert(2, values[0] - values[1])
        lost += values[2]
        lines.append(
            "  " + table.cell(channel, 10, align="<")
            + "".join(f" {v:9d}" for v in values)
        )
    sent = sum(
        getattr(
            getattr(getattr(getattr(p, "bus", None), "bus", None), "counters", None),
            "frames_sent", 0,
        )
        for p in legs
    )
    heavy = lost > LOST_WARN_FLOOR and lost > sent * LOST_WARN_RATIO
    lines.append(
        "  ** 미수신이 큼. 응답이 실제로 안 돌아옴 -- 배선·전원·프로토콜 **"
        if heavy
        else "  미수신이 작으면 유실은 없고 늦게 온 것임 (대기초과를 볼 것)."
    )

    # ---- 모터별 무응답 -----------------------------------------------------
    missed = []
    for part in legs:
        counts = getattr(part, "link_counts", None)
        if not callable(counts):
            continue
        for motor, value in counts().items():
            if value.get("missed", 0):
                missed.append((_name(robot, part, motor), value["missed"], value["asked"]))
    lines.append("")
    if missed:
        lines.append("  " + table.header(("모터", 22, "<"), ("빠짐", 7), ("명령", 7), ("비율", 7)))
        for name, miss, asked in sorted(missed, key=lambda row: -row[1]):
            share = miss / asked * 100.0 if asked else 0.0
            lines.append(f"  {table.cell(name, 22, align='<')} {miss:7d} {asked:7d} {share:6.1f}%")
    else:
        lines.append("  모터별 무응답  없음")

    # ---- 가드 --------------------------------------------------------------
    clips: Dict[str, int] = {}
    rejects: Dict[str, int] = {}
    for part in legs:
        guard = getattr(part, "counters", None)
        for key, value in getattr(guard, "clips", {}).items():
            clips[key] = clips.get(key, 0) + value
        for key, value in getattr(guard, "rejects", {}).items():
            rejects[key] = rejects.get(key, 0) + value
    lines.append("")
    lines.append(f"  가드 자름   {_pairs(clips)}")
    lines.append(f"  가드 거부   {_pairs(rejects)}")

    return "\n".join(lines)


def _pairs(counts: Dict[str, int]) -> str:
    nonzero = {k: v for k, v in counts.items() if v}
    if not nonzero:
        return "없음"
    return "  ".join(f"{k} {v}" for k, v in sorted(nonzero.items()))
