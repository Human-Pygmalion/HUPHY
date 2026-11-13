"""WCM1600 — 로봇 전체의 DC 전류를 재는 전류계 (UART 9600 8N1, Continuous 모드).

    protocol.py   프레임 한 줄 <-> 값. 순수 함수
    meter.py      Wcm1600. 위쪽이 쓰는 것

`sensors/ebimu/` 와 같은 나눔임. 다만 **IMU 가 아니므로** `registry.MODELS` 에
없음 -- 설정의 `imus` 항목이 아니고, 쓸 때만 포트를 줘서 엶
(`huphy-current`, `huphy-run --current`, ROS `current_node`).

계약과 자료형은 `sensors/current.py` 에 있음.
"""

from .meter import BAUDRATE, Wcm1600, find_port
from .protocol import parse

__all__ = ["Wcm1600", "find_port", "parse", "BAUDRATE"]
