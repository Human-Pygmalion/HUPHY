"""전류계의 중립 자료형 — **벤더를 모름.**

`base.py` 가 IMU 에 대해 하는 일을 전류계에 대해 함. 위쪽(기록, ROS 노드)은 여기
있는 것만 보고, 어느 회사 장비인지는 `sensors/<벤더>/` 가 앎.


## IMU 와 같은 계약, 다른 자료형

`ImuState` 에 암페어 칸을 하나 더 붙이지 않음. 전류계는 자세도 각속도도 안 내므로
그 칸이 전부 "모름" 으로 차고, `Imu` 프로토콜을 구현하게 만들면 `huphy-imu check`
같은 IMU 전용 검사에 끌려 들어옴. `registry.MODELS` 에도 넣지 않음 -- 그 표는
설정의 `imus` 항목을 구현체로 바꾸는 것임.

같은 것은 세 가지임.

    read() 는 통신하지 않음      백그라운드로 받아 두고 최근 값만 꺼냄
    못 받아도 객체는 나옴        is_valid 가 거짓. 값은 0 이지만 "0 A" 가 아니라 "모름"
    age_ms() 는 못 받으면 -1     무한대는 JSON 으로 못 보냄


## 이 값이 로그에서 무엇과 짝인가

`telemetry/power.py` 가 모터 상전류로 계산해 내는 `power/dc_a_lo` 와 **나란히**
남음. 둘은 같아질 수 없음 -- 계산값은 구리 손실만 보므로 작고, 실측은 모터 말고
연산보드·팬·통신까지 다 포함하므로 큼. 차를 바로 "손실" 로 읽으면 안 되고, 로봇을
세워 둔 채 잰 값을 기준선으로 빼야 비교가 됨.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional, Protocol, runtime_checkable


@dataclass(frozen=True)
class CurrentState:
    """전류계가 보고한 한 시점의 값.

    **못 받았어도 객체는 나옴.** `is_valid` 가 거짓일 뿐임 -- 호출부가 `None` 검사를
    하지 않아도 되게 함. 그때 `amps` 는 0 인데, 그것이 "전류가 0" 이라는 뜻이 아니라
    모른다는 뜻임.
    """

    amps: float = 0.0
    """전류 (A). DC 에서 부호가 방향임. AC 는 크기이므로 항상 0 이상."""

    is_ac: bool = False
    """AC 프레임이었는지. 이 로봇은 DC 라 참이면 측정 설정이 틀린 것임."""

    stamp: float = 0.0
    """이 프레임을 파싱한 시각. `time.monotonic()` 기준."""

    is_valid: bool = False
    """한 번이라도 프레임을 받았는지. 거짓이면 `amps` 는 의미 없음."""

    def age_ms(self, now: Optional[float] = None) -> float:
        """마지막 값 이후 경과. **한 번도 못 받았으면 -1.**"""
        if not self.is_valid:
            return -1.0
        return ((now if now is not None else time.monotonic()) - self.stamp) * 1000.0


@runtime_checkable
class CurrentMeter(Protocol):
    """전류계 하나. 상속하지 않아도 이 꼴이면 됨 (`sensors/base.py` 의 `Imu` 와 같음)."""

    name: str

    @property
    def is_connected(self) -> bool: ...

    def connect(self) -> None: ...

    def disconnect(self) -> None: ...

    def read(self) -> CurrentState: ...
