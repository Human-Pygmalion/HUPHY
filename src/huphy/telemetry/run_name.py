"""로그 파일 이름과 첫 줄 `# run ...` 을 만듦.

`policy_log` 와 `latency_log` 가 같은 규칙을 쓰게 한 곳에 모음. 한쪽만 고치면 같은
실행의 두 파일이 다른 이름으로 나가 짝을 못 찾음.


## 이름

    <월일>_<무엇>_<정책이름>_<인자들>.csv

    1029_policy_generalized_walk.csv                      기본값만
    1029_policy_generalized_walk_g0.3_vx0.3_wz0.2.csv
    1029_policy_generalized_walk_g0.3_teleop.csv
    1029_latency_huphy_squat_h0.80.csv

**기본값인 인자는 이름에서 뺌.** 전부 기본값이면 월일·무엇·정책만 남음. 뭘 바꿔
돌린 것인지가 이름에서 바로 보이게 하려는 것임.

`teleop` 이면 명령값 자리에 `teleop` 을 적음 -- 값이 매 주기 바뀌어 이름에 넣을 수
없음. 실제 값은 `obs/cmd/<이름>` 열에 주기마다 들어가 있음 (`policy_log`).

시각은 안 넣음. 같은 날 같은 인자로 또 돌리면 `_2`, `_3` 을 붙임. **덮어쓰지
않음.**


## 첫 줄

    # run 2026-10-29 14:30:52 policy=generalized_walk checkpoint=37400 gain=0.3 ...

이름에서 뺀 것이 전부 여기 들어감 -- 초 단위 시각, 체크포인트, 기본값인 인자들.
**이름은 찾기용, 첫 줄은 정확한 기록**임.

뷰어(`tools/policy_viewer.html`, `tools/latency_viewer.html`)는 `#` 로 시작하는
줄을 건너뛰므로 줄이 늘어도 안 깨짐.
"""

from __future__ import annotations

import time
from pathlib import Path
from typing import Any, Dict, Mapping, Optional, Sequence

SHORT = {"h_star": "h"}
"""이름에 쓸 때만 줄이는 명령. 첫 줄에는 원래 이름을 그대로 씀.

`h_star` 가 `h_star0.80` 이면 숫자와 붙어 읽기 나빠서임. 나머지(`vx`, `wz`)는
이미 짧아 그대로 둠.
"""


def _num(value: float) -> str:
    """`0.3` -> `0.3`, `1.0` -> `1`, `0.30` -> `0.3`. 파일 이름에 넣을 모양."""
    return f"{float(value):g}"


def tags(
    *,
    gain_scale: Any = None,
    commands: Optional[Mapping[str, float]] = None,
    defaults: Optional[Mapping[str, float]] = None,
    teleop: bool = False,
    extra: Optional[Mapping[str, Any]] = None,
) -> list:
    """이름에 붙일 조각들. **기본값인 것은 안 넣음.**

    `gain_scale`   숫자 하나 또는 둘(`나머지 발목`). `1.0` 이면 생략
    `commands`     지금 쓰는 명령값
    `defaults`     계약의 기본값. 같으면 그 명령을 생략함
    `teleop`       참이면 명령값 대신 `teleop` 하나만 넣음
    `extra`        `{"hz": 100, "drain": 4, "send": "burst"}` 처럼. 값이 `None`
                   이면 그 항목을 건너뜀 -- 호출부가 기본값일 때 `None` 을 줌
    """
    out = []

    if gain_scale is not None:
        values = list(gain_scale) if isinstance(gain_scale, (list, tuple)) else [gain_scale]
        if any(abs(float(v) - 1.0) > 1e-9 for v in values):
            out.append("g" + "-".join(_num(v) for v in values))

    if teleop:
        out.append("teleop")
    elif commands:
        base = defaults or {}
        for name, value in commands.items():
            if abs(float(value)) < 1e-9:
                continue                      # 0 은 "안 줌" 과 같음
            if name in base and abs(float(value) - float(base[name])) < 1e-9:
                continue                      # 계약 기본값이면 바꾼 것이 아님
            out.append(f"{SHORT.get(name, name)}{_num(value)}")

    for key, value in (extra or {}).items():
        if value is not None:
            out.append(f"{key}{value}" if not isinstance(value, bool) else key)

    return out


def path(directory: "str | Path", kind: str, name: str, parts: Sequence[str]) -> Path:
    """`<폴더>/<월일>_<kind>_<name>[_<parts...>].csv`.

    `kind` 는 `policy` / `latency`. `name` 은 정책 이름이나 동작 이름.

    같은 이름이 이미 있으면 `_2`, `_3` 을 붙임. **덮어쓰지 않음** -- 같은 날 같은
    인자로 두 번 돌린 것이 둘 다 남아야 비교가 됨.
    """
    stem = "_".join([time.strftime("%m%d"), kind, name, *parts])
    base = Path(directory)
    candidate = base / f"{stem}.csv"
    n = 2
    while candidate.exists():
        candidate = base / f"{stem}_{n}.csv"
        n += 1
    return candidate


def header(**fields: Any) -> str:
    """첫 줄 `# run <날짜 시각> key=value ...`. 줄바꿈까지 포함함.

    값이 `None` 인 항목은 건너뜀. 참/거짓은 `1`/`0` 으로 적음 -- `teleop=0` 이
    `teleop=False` 보다 짧고 읽기 같음.
    """
    stamp = time.strftime("%Y-%m-%d %H:%M:%S")
    out = []
    for key, value in fields.items():
        if value is None:
            continue
        if isinstance(value, bool):
            out.append(f"{key}={1 if value else 0}")
        elif isinstance(value, float):
            out.append(f"{key}={value:g}")
        elif isinstance(value, (list, tuple)):
            out.append(f"{key}=" + ",".join(f"{v:g}" if isinstance(v, float) else str(v)
                                            for v in value))
        else:
            out.append(f"{key}={value}")
    return f"# run {stamp} " + " ".join(out) + "\n"
