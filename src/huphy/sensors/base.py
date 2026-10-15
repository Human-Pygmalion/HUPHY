"""센서 계층의 중립 자료형 — **벤더를 모름.**

`motors/base.py` 와 같은 자리임. 위쪽(로봇, 텔레메트리)은 여기 있는 것만 보고,
어느 회사 IMU 인지는 `sensors/<벤더>/` 가 앎.


## 상자에 무엇을 두나

**제어가 반드시 쓰는 것만 필수로 두고, 나머지는 센서가 정함.**

    필수   gravity  gyro_dps  accel_mps2  stamp  is_valid
    선택   extra    센서마다 다름. 텔레메트리로만 나감

자세를 원본 형식(오일러/쿼터니언)으로 올리지 않는 이유가 여기 있음. 센서마다 주는
형식이 다른데 그걸 필수 칸으로 두면, **한 센서를 붙이려고 형식을 정하는 순간 다른
센서가 전부 따라와야 함.** 정책이 실제로 쓰는 것은 중력방향 3개뿐이므로 그것만
필수로 둠.

원본 자세는 `extra` 로 감. EBIMU 는 `qw qx qy qz`, Xsens 는 `roll pitch yaw` 를
넣고, 둘 다 그래프에 그대로 나옴.


## 단위

    중력방향  단위벡터. 수평이면 (0, 0, -1)
    각속도    도/초 (deg/s)
    가속도    m/s^2

센서가 다른 단위로 주면 벤더 모듈이 바꿔서 올림. 프로젝트 전체가 도를 쓰므로 여기서
섞이면 안 됨.


## 중력방향은 벤더가 계산해 올림

센서가 오일러를 주든 쿼터니언을 주든 **그 형식을 아는 쪽에서** 중력방향을 만듦.
식은 아래 `gravity_from_quat` / `gravity_from_euler` 에 있고, 벤더 모듈이 자기에게
맞는 것을 부름.

위쪽이 "이 센서는 뭘 주지" 를 묻지 않게 하려는 것임. 물어야 하면 정책이 센서마다
분기하게 되고, 센서를 늘릴 때마다 정책을 고쳐야 함.

부착 방향이 맞는지는 확인해야 함. `huphy-imu check` 가 가속도계와 대조함 -- 정지
상태의 가속도계는 중력방향을 직접 재므로, 자세에서 계산한 값과 같아야 함.


## `read()` 는 통신하지 않음

제어 루프 안에서 시리얼을 기다리면 주기가 통째로 밀림. 벤더 구현이 백그라운드로
받아 두고, `read()` 는 **가장 최근 값을 꺼내기만** 함.


## 시각이 둘임

    stamp                우리가 그 줄을 파싱한 시각
    extra["sensor_ms"]   센서가 측정한 시각. 센서가 찍어 보냄

`stamp` 만으로는 커널 버퍼에 머문 시간을 못 봄. 스레드가 밀리면 50ms 전에 측정된
값도 지금 파싱하니 `stamp` 가 지금으로 찍히고 `age_ms` 가 0을 냄.

둘을 같이 보면 원인이 갈림.

    sensor_ms 는 규칙적인데 stamp 간격이 튐    우리 쪽이 밀림
    sensor_ms 증가량이 주기의 배수로 뜀        패킷이 빠지는 중
    둘 다 안 변하고 age 만 자람                센서가 멈춤
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Dict, Optional, Protocol, Tuple, runtime_checkable

Vector3 = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]
"""자세 쿼터니언 `(w, x, y, z)`.

**벤더 모듈 안에서만 쓰는 형식임.** 센서가 다른 순서로 보내면(EBIMU 는 `z, y, x, w`)
그 모듈이 뒤집어서 이 순서로 만든 뒤 중력방향을 계산함. **축 규약도 그 모듈이
맞춤** -- 여기 들어오는 쿼터니언은 각속도·가속도와 같은 축(센서 도면 축) 기준의
오른손 쿼터니언이어야 함. EBIMU 는 쿼터니언만 `(X, -Y, -Z)` 축 기준이라 y, z 부호를
뒤집어 들어옴 (`ebimu/protocol.py` 의 `to_quaternion`).
"""

ZERO3: Vector3 = (0.0, 0.0, 0.0)

LEVEL_GRAVITY: Vector3 = (0.0, 0.0, -1.0)
"""수평일 때의 중력방향. 값을 못 받았을 때의 기본값임.

0벡터로 두지 않는 이유: 크기가 0인 중력방향은 물리적으로 없는 값이라, 정책에
들어가면 "어느 쪽이 아래인지 모름" 이 아니라 "중력이 없음" 으로 해석됨. 받았는지는
`is_valid` 로 판단할 것.
"""


@dataclass
class ImuState:
    """IMU 가 보고한 한 시점의 값.

    **못 받았어도 객체는 나옴.** `is_valid` 가 거짓일 뿐임 — 호출부가 `None` 검사를
    하지 않아도 되게 함. 그때 값은 수평·정지 상태와 같은데, 그것이 사실이라는 뜻이
    아니라 모른다는 뜻임. 구분은 `is_valid` 와 `stamp` 로 함.
    """

    gravity: Vector3 = field(default=LEVEL_GRAVITY)
    """몸체 좌표에서 본 중력 단위벡터. 수평이면 `(0, 0, -1)`.

    **정책이 쓰는 유일한 자세 값임.** 원본 형식은 센서마다 다르므로 `extra` 로 감.
    """

    gyro_dps: Vector3 = field(default=ZERO3)
    """각속도. 벤더가 rad/s 로 주면 바꿔서 올림."""

    accel_mps2: Vector3 = field(default=ZERO3)
    """몸체 좌표계 가속도. **중력이 포함된 값이어야 함.**

    정지 상태에서 중력방향을 그대로 잰 값이 되어, `gravity` 와 대조해 부착 방향을
    확인하는 근거가 됨 (`huphy-imu check`). 중력 제거 모드로 세팅하면 그 검사가
    무의미해짐.
    """

    extra: Dict[str, float] = field(default_factory=dict)
    """센서마다 다른 값. 텔레메트리로만 나가고 제어에는 안 씀.

    키 목록은 벤더가 `Imu.extra_fields` 로 미리 냄 -- CSV 헤더를 실행 전에 써야
    하고, 열이 나타났다 사라지면 파일이 밀림.
    """

    stamp: float = 0.0
    """이 값을 파싱한 시각. `time.monotonic()` 기준.

    벽시계를 쓰지 않는 이유: 시각 동기화가 뒤로 점프하면 경과 시간이 음수가 됨.
    """

    is_valid: bool = False
    """한 번이라도 패킷을 받았는지. 거짓이면 나머지 필드는 의미 없음."""

    def age_ms(self, now: Optional[float] = None) -> float:
        """마지막 값 이후 경과. **한 번도 못 받았으면 -1.**

        `-1` 을 쓰는 이유: 무한대는 JSON 으로 못 보내고 CSV 에서도 읽기 어려움
        (`telemetry/snapshot.py` 의 규약과 같음).
        """
        if not self.is_valid:
            return -1.0
        return ((now if now is not None else time.monotonic()) - self.stamp) * 1000.0


# ---------------------------------------------------------------------------
# 자세 -> 중력방향
#
# 벤더 모듈이 자기 센서 형식에 맞는 것을 부름. 두 식은 같은 값을 냄 -- 오일러가
# ZYX 순서라는 전제 아래 대수적으로 동일함.
# ---------------------------------------------------------------------------
def gravity_from_quat(quat: Quaternion) -> Vector3:
    """쿼터니언 `(w, x, y, z)` 에서. 수평이면 `(0, 0, -1)`.

        g = R^T (0, 0, -1)

    `R` 은 몸체에서 월드로 가는 회전이고 세 번째 행만 있으면 되므로, 행렬을 만들지
    않고 바로 씀.
    """
    w, x, y, z = (float(v) for v in quat)
    return (
        2.0 * (w * y - x * z),
        -2.0 * (y * z + w * x),
        2.0 * (x * x + y * y) - 1.0,
    )


def quat_to_euler(quat: Quaternion) -> Vector3:
    """쿼터니언에서 `(roll, pitch, yaw)` 도. **그래프에 보여주기 위한 것임.**

    쿼터니언을 받는 센서라도 사람은 자세를 도로 봄. 그래서 벤더 모듈이 이것을
    `extra` 에 같이 넣음.

    **제어 경로가 아님.** 정책은 `gravity` 를 쓰고 그 값은 쿼터니언에서 바로 나옴.
    여기서 순서(ZYX)를 잘못 골라도 그래프 숫자만 달라지고 토크는 안 바뀜.

    순서를 우리가 정할 수 있는 이유: 센서가 준 값을 해석하는 것이 아니라 우리가
    만들어 내는 표시값임. 맞출 상대가 없음.

    pitch 가 +-90 도 근처에서는 roll 과 yaw 가 서로 섞여 값이 불안정해짐 -- 자세
    자체는 쿼터니언에 그대로 있으므로 표시만 흔들림.
    """
    w, x, y, z = (float(v) for v in quat)
    sin_pitch = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    return (
        math.degrees(math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))),
        math.degrees(math.asin(sin_pitch)),
        math.degrees(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))),
    )


def gravity_from_euler(roll_deg: float, pitch_deg: float) -> Vector3:
    """오일러각에서. **ZYX(yaw -> pitch -> roll) 순서를 전제함.**

        g = (sin p, -sin r cos p, -cos r cos p)

    yaw 는 안 씀 -- 중력이 z 축이라 z 축 회전으로는 안 바뀜.

    **순서 규약은 센서 펌웨어가 정하는 값임.** ZXY 로 보고하는 센서에 이 식을 쓰면
    중력방향이 조용히 틀어짐 -- 크기는 여전히 1이라 검사로 안 잡힘. 오일러를 주는
    센서를 붙일 때는 `huphy-imu check` 로 가속도계와 대조해 확인할 것.
    """
    roll, pitch = math.radians(roll_deg), math.radians(pitch_deg)
    return (
        math.sin(pitch),
        -math.sin(roll) * math.cos(pitch),
        -math.cos(roll) * math.cos(pitch),
    )


@runtime_checkable
class Imu(Protocol):
    """IMU 하나가 갖춰야 할 것.

    `Robot` 계약과 같은 모양임 — 열고, 읽고, 닫음. 구현체는 이 프로토콜만 맞추면
    되고 상속하지 않아도 됨.
    """

    name: str
    """이 로봇에서 이 IMU 를 부르는 이름. **텔레메트리 필드 앞에 붙음.**

    붙는 자리(다리, 몸통)가 아니라 개체 이름을 쓰는 이유: 센서를 다리에서 몸통으로
    옮겨도 필드 이름이 그대로라 예전 로그·그래프 레이아웃과 맞음.
    """

    extra_fields: Tuple[str, ...]
    """이 센서가 `ImuState.extra` 에 넣는 키 목록. **고정이어야 함.**

    실행 전에 알 수 있어야 CSV 헤더를 쓰고 PlotJuggler 레이아웃을 미리 만들 수 있음.
    값이 없는 주기에도 키는 내보내고 0을 채움 -- 열이 중간에 사라지면 파일이 밀림.
    """

    @property
    def is_connected(self) -> bool: ...

    def connect(self) -> None: ...

    def disconnect(self) -> None: ...

    def read(self) -> ImuState:
        """가장 최근 값. **새로 통신하지 않음.**"""
        ...
