"""전력 열 -- 읽어 온 상전류로 구리 손실을 계산해 기록에 붙임.

    power/<모터>/copper_w   그 모터의 구리 손실 (W)
    power/copper_w          모터 전부의 합 (W)
    power/dc_a_lo           DC 전류 **하한** (A)

`extra/` 열과 짝임 -- 상전류(`iq`)와 버스 전압(`vbus`)을 읽어 올 때만 계산됨
(`--extra`, ROS `control_node` 의 `extra` 파라미터). 안 읽으면 열이 없음.

## 계산

FOC 는 `id` 를 0 으로 제어하므로 상전류 크기가 `iq` 임. 3상 합계:

    P_cu = 1.5 * iq**2 * R_phase        iq 는 진폭 (A), R_phase 는 상저항 (ohm)

`1.5` 는 3상(x3) 과 진폭->실효값(x1/2) 에서 옴. 모터가 내는 `iqf` 는 진폭임.

`R_phase` 는 모델별 사양값임 (`motors/robstride/tables.phase_resistance_ohm`).

## 하한인 이유

    들어가는 것 = 구리 손실 + 기계 출력 + 철손 + 인버터 손실 + 감속기 손실

이 모듈은 **구리 손실만** 계산함. 나머지 네 항목이 빠진 이유:

    기계 출력    토크와 속도가 필요함. 상태 프레임에 있지만 기록 열에 없음
    철손        철손 상수가 사양서에 없음
    인버터 손실  스위칭 손실 상수가 사양서에 없음
    감속기 손실  9:1 감속기의 효율값이 사양서에 없음

그래서 `power/dc_a_lo` 는 실제 DC 전류보다 작음. 이름의 `_lo` 가 그 뜻임.
docs/motor_power.md 3절.

## 모르는 값은 NaN

상전류를 아직 못 받은 모터(파라미터 읽기 응답이 안 옴)와 사양을 안 넣은
모델(RS00·RS02)은 `NaN` 이 됨. 합은 **한 모터라도 NaN 이면 NaN** 임 -- 그 모터를
빼고 더하면 일부만 더한 값이 전체 합으로 보임.
"""

from __future__ import annotations

import math
from typing import Dict, Mapping, Sequence, Tuple

PHASE_SUM = 1.5
"""3상 합계 계수. 3 (상 개수) x 1/2 (진폭 -> 실효값 제곱)."""

TOTAL_COLUMNS = ("power/copper_w", "power/dc_a_lo")
"""모터별 열 뒤에 붙는 합계 열. 순서 고정."""


def power_columns(motors: Sequence[str]) -> Tuple[str, ...]:
    """모터 이름 목록 -> 전력 열 이름. `power_values` 와 같은 순서.

    `telemetry/track_log.py` 의 `extra_columns` 와 같은 이유로 열을 미리 만들 수
    있어야 함 -- ROS `log_node` 는 첫 정책 줄이 전기값보다 먼저 올 수 있음.
    """
    return tuple(f"power/{m}/copper_w" for m in motors) + TOTAL_COLUMNS


def copper_loss_w(iq_a: float, r_phase_ohm: float) -> float:
    """구리 손실 (W). `iq` 는 진폭 (A), `r_phase` 는 상저항 (ohm)."""
    return PHASE_SUM * float(iq_a) * float(iq_a) * float(r_phase_ohm)


def power_values(
    motors: Sequence[str],
    iq: Mapping[str, float],
    vbus: Mapping[str, float],
    r_phase: Mapping[str, float],
) -> Dict[str, float]:
    """전력 열 -> 값. `power_columns(motors)` 와 같은 열을 다 채움.

        motors    모터 이름. 열 순서를 정함
        iq        모터 이름 -> 상전류 진폭 (A). 없거나 NaN 이면 그 모터가 NaN
        vbus      모터 이름 -> 버스 전압 (V). `dc_a_lo` 의 분모(평균)에 씀
        r_phase   모터 이름 -> 상저항 (ohm). 모르는 모델은 NaN 으로 줄 것

    `dc_a_lo` 는 합을 `vbus` 평균으로 나눈 값임. 모터마다 재는 전압이 같은
    버스라 평균을 씀 -- 2026-10-28 기록에서 12대의 폭이 48.1~48.5 V 였음.
    """
    nan = float("nan")
    out: Dict[str, float] = {}
    total = 0.0
    for name in motors:
        w = copper_loss_w(iq.get(name, nan), r_phase.get(name, nan))
        out[f"power/{name}/copper_w"] = w
        total += w                      # NaN 이 하나라도 섞이면 합이 NaN 이 됨
    volts = [float(v) for v in (vbus.get(n, nan) for n in motors) if not math.isnan(v)]
    out["power/copper_w"] = total
    out["power/dc_a_lo"] = total / (sum(volts) / len(volts)) if volts else nan
    return out
