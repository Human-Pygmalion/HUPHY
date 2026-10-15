"""EBIMU 패킷 한 줄 <-> 값.

센서도 시리얼도 모름. 문자열과 숫자만 다룸. `robstride/codec/mit.py` 와 같은
자리임 -- 순수 함수라 하드웨어 없이 테스트됨.


## 한 줄의 모양

    *<자세><각속도><가속도><거리><온도><시각>(CR)(LF)

`*` 로 시작하고 값을 `,` 로 구분함. **어떤 항목이 들어 있는지는 안 적혀 있음.**
그래서 `output` 을 받아 그 순서대로 잘라 읽음.

    output = [quat, gyro, accel, dist, temp, time]      15개

개수가 안 맞으면 그 줄을 통째로 버림. 앞에서부터 채우고 남는 것을 무시하면, 항목
하나가 빠졌을 때 뒤의 값이 전부 한 칸씩 당겨져 **각속도 자리에 가속도가 들어감.**
값이 그럴듯해서 실물에서 안 잡히는 종류의 오류임.


## 여기서 바꾸는 것

    쿼터니언        센서는 (z, y, x, w) 로 보냄 -> (w, x, -y, -z). 순서와 축 부호
    가속도 단위     센서는 g 로 보냄            -> m/s^2
    중력방향        쿼터니언에서 계산. 정책이 쓰는 값
    roll/pitch/yaw  쿼터니언에서 계산. 그래프에만 나감

마지막 둘이 성격이 다름. 중력방향은 제어 경로라 형식(쿼터니언)에서 바로 나오고,
오일러는 사람이 보는 표시값이라 순서 규약을 우리가 정함 (`sensors/base.py` 참조).
"""

from __future__ import annotations

import time
from typing import Dict, List, Optional, Sequence, Tuple

from ..base import (
    ZERO3,
    ImuState,
    Quaternion,
    gravity_from_quat,
    quat_to_euler,
)
from . import commands

PREFIX = "*"
"""패킷 시작 문자. 이걸로 시작 안 하면 설정 응답이거나 깨진 줄임."""

G_TO_MPS2 = 9.80665
"""표준 중력. 센서는 가속도를 g 로 보냄."""

EXTRA_FIELDS: Tuple[str, ...] = (
    "qw", "qx", "qy", "qz",
    "roll", "pitch", "yaw",
    "dx", "dy", "dz",
    "temp", "sensor_ms",
)
"""텔레메트리로 나가는 EBIMU 고유 값. **`output` 과 무관하게 고정임.**

`dist` 나 `temp` 를 안 켰어도 키는 냄. 열이 나타났다 사라지면 CSV 가 밀리고
PlotJuggler 레이아웃이 깨짐 -- `Leg.torque_motors` 가 모드와 무관하게 고정 목록을
내는 것과 같은 이유임.

`qw..qz` 와 `roll..yaw` 를 둘 다 내는 이유: 그래프가 이상할 때 센서가 이상한 것인지
우리 변환이 이상한 것인지 구분하려면 원본이 옆에 있어야 함.
"""


def _floats(text: str) -> List[float]:
    """`,` 로 나눠 숫자만 뽑음. 숫자가 아닌 것이 나오면 거기서 멈춤.

    끝에 체크섬이 붙는 펌웨어가 있어 **뒤쪽 비숫자는 버림.** 중간에 있으면 그
    뒤를 다 버리므로 개수가 안 맞아 줄이 통째로 탈락함 -- 조용히 밀려 들어가는
    것보다 나음.
    """
    out: List[float] = []
    for token in text.split(","):
        token = token.strip()
        if not token:
            continue
        try:
            out.append(float(token))
        except ValueError:
            break
    return out


def parse_fields(
    line: str, output: Sequence[str]
) -> Optional[Dict[str, Tuple[float, ...]]]:
    """한 줄을 블록별 값으로 자름. 못 읽으면 `None`.

    예외를 던지지 않음 -- 100Hz 로 들어오는 줄 하나가 깨졌다고 스레드가 죽으면
    안 됨. 몇 줄이 버려졌는지는 호출부가 셈.
    """
    if not line.startswith(PREFIX):
        return None

    values = _floats(line[len(PREFIX):])
    if len(values) != commands.field_count(output):
        return None

    out: Dict[str, Tuple[float, ...]] = {}
    index = 0
    for block in output:
        size = commands.BLOCK_SIZE[block]
        out[block] = tuple(values[index:index + size])
        index += size
    return out


def to_quaternion(packed: Sequence[float]) -> Quaternion:
    """센서가 보낸 `(z, y, x, w)` 를 도면 축 기준 오른손 `(w, x, y, z)` 로.

    **뒤집는 곳은 여기 하나임.** 나머지 코드는 EBIMU 순서와 축 규약을 모름.

    순서만 바꾸면 안 되고 **y, z 부호도 뒤집어야 함.** EBIMU 의 회전 부호 규약(매뉴얼
    4-1)은 롤 Y->Z, 피치 X->Z, 요 Y->X 인데, 이는 도면의 (X, Y, Z) 가 아니라
    `(X, -Y, -Z)` 축(X 앞 / Y 오른쪽 / Z 아래, 항공 기체 좌표계)에 오른손 법칙을 쓴
    것과 같음. 각속도·가속도는 도면 축으로 나오므로 쿼터니언만 도면 축으로 옮김.

    뒤집지 않으면 중력방향의 **x 성분만** 부호가 반대로 나옴 (y·z 성분은 y, z 를
    같이 뒤집어도 안 바뀜) -- 앞으로 기울면 정책은 뒤로 기운 것으로 봄. 표시용
    pitch·yaw 도 부호가 반대가 됨.

    가속도계로 검증됨 (Test-repo `bf2978a`, 정지·15도 기울임): 뒤집으면 쿼터니언의
    중력과 가속도계가 0.002 안에서 맞고, 뒤집기 전에는 0.5 벌어짐.
    """
    z, y, x, w = (float(v) for v in packed)
    return (w, x, -y, -z)


def to_state(
    fields: Dict[str, Tuple[float, ...]],
    *,
    stamp: Optional[float] = None,
) -> ImuState:
    """블록별 값을 `ImuState` 로. 단위와 형식을 여기서 맞춤.

    `output` 에 없던 항목은 0 으로 나감 -- `temp` 를 안 켰으면 0 이고, `time` 을
    안 켰으면 `sensor_ms` 가 -1 임.
    """
    if "quat" not in fields:
        raise ValueError(
            "자세가 쿼터니언이 아님. `huphy-imu apply` 로 센서를 맞출 것 "
            "(오일러 출력은 회전 순서 규약을 알아야 해서 쓰지 않음)"
        )

    quat = to_quaternion(fields["quat"])
    roll, pitch, yaw = quat_to_euler(quat)
    dist = fields.get("dist", ZERO3)
    sensor_time = fields.get("time")

    return ImuState(
        gravity=gravity_from_quat(quat),
        gyro_dps=tuple(fields.get("gyro", ZERO3)),
        accel_mps2=tuple(v * G_TO_MPS2 for v in fields.get("accel", ZERO3)),
        extra={
            "qw": quat[0], "qx": quat[1], "qy": quat[2], "qz": quat[3],
            "roll": roll, "pitch": pitch, "yaw": yaw,
            "dx": float(dist[0]), "dy": float(dist[1]), "dz": float(dist[2]),
            "temp": float(fields["temp"][0]) if "temp" in fields else 0.0,
            "sensor_ms": float(sensor_time[0]) if sensor_time else -1.0,
        },
        stamp=time.monotonic() if stamp is None else float(stamp),
        is_valid=True,
    )


def decode(
    line: str,
    output: Sequence[str],
    *,
    stamp: Optional[float] = None,
) -> Optional[ImuState]:
    """한 줄을 바로 `ImuState` 로. 못 읽으면 `None`."""
    fields = parse_fields(line, output)
    if fields is None:
        return None
    return to_state(fields, stamp=stamp)
