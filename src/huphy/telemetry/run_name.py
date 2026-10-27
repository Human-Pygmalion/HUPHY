"""로그 파일의 이름·`run` 칸·쓰기를 한 곳에 모음.

`policy_log`·`latency_log`·`track_log` 가 같은 규칙을 쓰게 함. 한쪽만 고치면 같은
실행의 파일들이 다른 이름으로 나가, 어느 `latency` 가 어느 `policy` 와 같은 실행
것인지 이름으로 못 가림.


## 이름

    <폴더>/<종류>/<월일>_<종류>_<이름>[_<인자들>].json

    logs/policy/1027_policy_generalized_walk.json              기본값만
    logs/policy/1027_policy_generalized_walk_g0.3_vx0.3.json
    logs/policy/1027_policy_generalized_walk_g0.5-0.8_teleop.json
    logs/latency/1027_latency_huphy_squat_h0.8.json
    logs/track/1027_track_zero_g0.3.json

**기본값인 인자는 이름에서 뺌.** 전부 기본값이면 월일·종류·이름만 남음. 뭘 바꿔
돌린 것인지가 이름에서 바로 보이게 하려는 것임.

`teleop` 이면 명령값 자리에 `teleop` 을 적음 -- 값이 매 주기 바뀌어 이름에 넣을 수
없음. 실제 값은 `obs/cmd/<이름>` 열에 주기마다 들어가 있음 (`policy_log`).

시각은 안 넣음. 같은 날 같은 인자로 또 돌리면 `_2`, `_3` 을 붙임. **덮어쓰지
않음.**


## 파일 안

    {"run":     {"at": "2026-10-27 14:30:52", "policy": "generalized_walk", ...},
     "limits":  {"right_leg/knee": [6, 114], ...},
     "columns": ["t", "obs/gyro/x", ...],
     "rows":    [[0.0, 0.01, ...], [0.02, ...]]}

`run` 에 **이름에서 뺀 것이 전부** 들어감 -- 초 단위 시각, 체크포인트, 기본값인
인자들. 이름은 찾기용, `run` 은 정확한 기록임.

CSV 였다가 JSON 으로 바꿨음. 전에는 `# limits ...` 와 `# run ...` 을 문자열로
쪼개 읽어야 했는데, 지금은 뷰어가 `data.run.gain` 으로 바로 읽음.
"""

from __future__ import annotations

import json
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
    """`<폴더>/<kind>/<월일>_<kind>_<name>[_<parts...>].json`.

    `kind` 는 `policy` / `latency` / `track`. `name` 은 정책 이름이나 동작 이름.

        logs/policy/1027_policy_generalized_walk_g0.3_vx0.3.json
        logs/latency/1027_latency_generalized_walk_g0.3_vx0.3.json
        logs/track/1027_track_zero_g0.3.json

    **종류별로 폴더를 나눔.** 한 폴더에 섞여 있으면 실행을 몇 번 하는 순간 목록이
    길어져 같은 실행의 짝을 눈으로 찾기 어려움.

    폴더에 이미 `kind` 가 있는데 파일 이름에도 넣는 이유: 파일 하나만 떼어
    뷰어에 끌어다 놓거나 남에게 보낼 때 이름만으로 뭔지 알 수 있어야 함.

    같은 이름이 이미 있으면 `_2`, `_3` 을 붙임. **덮어쓰지 않음** -- 같은 날 같은
    인자로 두 번 돌린 것이 둘 다 남아야 비교가 됨.
    """
    stem = "_".join([time.strftime("%m%d"), kind, name, *parts])
    base = Path(directory) / kind
    candidate = base / f"{stem}.json"
    n = 2
    while candidate.exists():
        candidate = base / f"{stem}_{n}.json"
        n += 1
    return candidate


def header(**fields: Any) -> Dict[str, Any]:
    """파일의 `run` 칸. 이 실행이 무엇이었는지.

        {"at": "2026-10-27 14:30:52", "policy": "generalized_walk",
         "checkpoint": "37400", "gain": 0.3, "vx": 0.3, ...}

    **이름에서 뺀 것이 전부 여기 들어감** -- 초 단위 시각, 체크포인트, 기본값인
    인자들. 이름은 찾기용, 이 칸은 정확한 기록임.

    값이 `None` 인 항목은 건너뜀. 참/거짓은 그대로 `true`/`false` 로 나감.
    """
    out: Dict[str, Any] = {"at": time.strftime("%Y-%m-%d %H:%M:%S")}
    for key, value in fields.items():
        if value is not None:
            out[key] = list(value) if isinstance(value, tuple) else value
    return out


def write(path: "str | Path", run: Mapping[str, Any], **sections: Any) -> Path:
    """JSON 한 파일로 씀. 폴더가 없으면 만듦.

        {"run": {...}, "limits": {...}, "columns": [...], "rows": [[...], ...]}

    `run` 은 `header(...)` 가 만든 사전. 나머지는 기록 종류마다 다름 --
    `policy`/`track` 은 `limits`·`columns`·`rows`, `latency` 는 `meta`·`columns`·
    `rows`.

    **열 이름을 한 번만 적고 값은 배열로** 둠. 줄마다 `{"t": ..., "obs/...": ...}`
    로 쓰면 같은 기록이 1.9배로 커짐 (50Hz 60초, 열 81개 기준 2.2MB -> 4.0MB).

    `separators` 로 공백을 뺌. 사람이 읽는 파일이 아니라 뷰어가 읽는 파일임.
    """
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    body: Dict[str, Any] = {"run": dict(run)}
    body.update({k: v for k, v in sections.items() if v is not None})
    with path.open("w") as f:
        json.dump(body, f, separators=(",", ":"), allow_nan=False)
    return path
