"""전류·전력 열 -- 모터에서 계산한 것과 전류계로 **실측한 것**.

    power/<모터>/copper_w   그 모터의 구리 손실 (W)
    power/<모터>/dc_a_lo    그 모터의 DC 전류 **하한** (A). 구리 손실 / 그 모터의 VBUS
    power/copper_w          모터 전부의 합 (W)
    power/dc_a_lo           DC 전류 **하한** 합 (A). 합 / VBUS 평균

    current/total_a         전류계가 잰 로봇 전체 DC 전류 (A). 실측
    current/age             그 값이 들어온 뒤 흐른 시간 (ms). 못 받았으면 -1

앞의 넷은 계산, 뒤의 둘은 실측임. **한 파일에 둔 이유는 둘이 같아질 수 없다는 것을
한 군데에 적어 두려는 것임** -- 4절.

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

그래서 `dc_a_lo` (모터별·합 모두) 는 실제 DC 전류보다 작음. 이름의 `_lo` 가 그 뜻임.
docs/motor_power.md 3절.

## 계산값과 실측값은 같아질 수 없음

차이가 두 가지에서 옴.

    빠진 손실    철손 + 인버터 + 감속기 + 기계 출력    -> 계산값이 작음
    범위 차이    실측은 연산보드·팬·통신까지 다 포함   -> 실측이 큼

그래서 `current/total_a - power/dc_a_lo` 를 바로 "손실" 로 읽으면 안 됨. 로봇을
세워 둔 채(모터는 자세 유지 전류만) 잰 값을 기준선으로 빼면 비교가 됨.

전류계는 `sensors/wcm1600/` 임. 값이 없는 실행에서는 두 열이 아예 안 생김 --
`current/age` 가 `-1` 인 것은 전류계를 켰는데 아직 못 받은 것이고, 열 자체가 없는
것은 안 켠 것임.


## 모르는 값은 NaN

상전류를 아직 못 받은 모터(파라미터 읽기 응답이 안 옴)와 사양을 안 넣은
모델(RS00·RS02)은 `NaN` 이 됨. 합은 **한 모터라도 NaN 이면 NaN** 임 -- 그 모터를
빼고 더하면 일부만 더한 값이 전체 합으로 보임.
"""

from __future__ import annotations

import math
from typing import Any, Dict, Mapping, Sequence, Tuple

PHASE_SUM = 1.5
"""3상 합계 계수. 3 (상 개수) x 1/2 (진폭 -> 실효값 제곱)."""

TOTAL_COLUMNS = ("power/copper_w", "power/dc_a_lo")
"""모터별 열 뒤에 붙는 합계 열. 순서 고정."""


def power_columns(motors: Sequence[str]) -> Tuple[str, ...]:
    """모터 이름 목록 -> 전력 열 이름. `power_values` 와 같은 순서.

        모터별 copper_w  ->  모터별 dc_a_lo  ->  합계 (TOTAL_COLUMNS)

    `telemetry/track_log.py` 의 `extra_columns` 와 같은 이유로 열을 미리 만들 수
    있어야 함 -- ROS `log_node` 는 첫 정책 줄이 전기값보다 먼저 올 수 있음.
    """
    return (tuple(f"power/{m}/copper_w" for m in motors)
            + tuple(f"power/{m}/dc_a_lo" for m in motors)
            + TOTAL_COLUMNS)


CURRENT_COLUMNS = ("current/total_a", "current/age")
"""전류계 실측 열. 순서 고정. `current_values` 와 같은 순서임.

`power/...` 뒤에 붙음 (`telemetry/policy_log.py`). 모터 수와 무관하게 두 칸임 --
전류계는 로봇 전체를 하나로 잼.
"""


def current_values(state: Any) -> Dict[str, float]:
    """`CurrentState` -> 실측 열. `CURRENT_COLUMNS` 를 다 채움.

    `None` 을 주면 두 칸이 NaN·-1 이 됨 -- 전류계를 켰는데 아직 아무 프레임도 안
    온 경우임. 열을 만들지 말지는 부르는 쪽이 정함 (`PolicyLog(current=...)`).

    `age` 를 같이 내는 이유: 장비 출력 주기가 로그 주기(50Hz)보다 느려 같은 값이
    여러 줄에 반복됨. `age` 가 없으면 그것이 "전류가 안 변함" 인지 "장비가 멈춤"
    인지 구분이 안 됨 (`sensors/base.py` 의 IMU `age` 와 같은 이유).
    """
    if state is None or not getattr(state, "is_valid", False):
        return {"current/total_a": float("nan"), "current/age": -1.0}
    return {"current/total_a": float(state.amps), "current/age": float(state.age_ms())}


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
        vbus      모터 이름 -> 버스 전압 (V). `dc_a_lo` 의 분모
        r_phase   모터 이름 -> 상저항 (ohm). 모르는 모델은 NaN 으로 줄 것

    모터별 `dc_a_lo` 는 그 모터의 구리 손실을 그 모터가 잰 `vbus` 로 나눈 값임.
    전압을 못 받은 모터는 NaN.

    합계 `dc_a_lo` 는 손실 합을 `vbus` 평균으로 나눈 값임. 모터마다 재는 전압이 같은
    버스라 평균을 씀 -- 2026-10-28 기록에서 12대의 폭이 48.1~48.5 V 였음. 모터별
    값을 더해도 거의 같지만, 전압을 못 받은 모터가 하나 있어도 합은 나오게 함.
    """
    nan = float("nan")
    out: Dict[str, float] = {}
    total = 0.0
    losses = {}
    for name in motors:
        w = copper_loss_w(iq.get(name, nan), r_phase.get(name, nan))
        out[f"power/{name}/copper_w"] = w
        losses[name] = w
        total += w                      # NaN 이 하나라도 섞이면 합이 NaN 이 됨
    for name in motors:
        v = float(vbus.get(name, nan))
        out[f"power/{name}/dc_a_lo"] = losses[name] / v if v > 0 else nan   # NaN > 0 은 거짓
    volts = [float(v) for v in (vbus.get(n, nan) for n in motors) if not math.isnan(v)]
    out["power/copper_w"] = total
    out["power/dc_a_lo"] = total / (sum(volts) / len(volts)) if volts else nan
    return out
