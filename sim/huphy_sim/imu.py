"""MuJoCo 센서를 IMU 로 내보냄. `sensors/base.py` 의 `Imu` 프로토콜을 채움.

실물 IMU(`sensors/ebimu/`)는 시리얼로 쿼터니언을 받아 중력방향을 계산함. 여기서는
MJCF 의 센서를 그대로 씀 -- `imu_in_base` site 에 자이로·가속도계·`framezaxis` 가
붙어 있음 (MJCF 270~275줄).

    gravity    -(월드 z축을 IMU 좌표에서 본 것). 수평이면 (0, 0, -1)
    gyro_dps   MJCF 는 rad/s 로 줌. **도/초로 바꿔 올림** -- `ImuState` 규약임
    accel      중력 포함. MJCF `accelerometer` 가 그렇게 냄

노이즈·바이어스·지연을 넣지 않음. 실물에 있는 그 세 가지는 이 경로로 검증되지
않음 (`sim/README.md`).
"""

from __future__ import annotations

import math
import time
from typing import Tuple

from huphy.sensors.base import ImuState

from .world import IMU_ACCEL, IMU_GYRO, IMU_UPVECTOR, SimWorld


class SimImu:
    """IMU 하나. 세계를 읽기만 함.

    `name` 이 `robot.yaml` 의 `imus` 키와 같아야 함 -- 텔레메트리 필드 앞에 그
    이름이 붙어서, 다르면 실물 로그와 열 이름이 어긋남.
    """

    def __init__(self, world: SimWorld, *, name: str = "main") -> None:
        self.world = world
        self.name = str(name)
        self.extra_fields: Tuple[str, ...] = ()
        self._connected = False

    def __str__(self) -> str:
        return f"{self.name} (SimImu)"

    @property
    def is_connected(self) -> bool:
        return self._connected

    def connect(self) -> None:
        self._connected = True

    def disconnect(self) -> None:
        self._connected = False

    def read(self) -> ImuState:
        """지금 값. **새로 통신하지 않음** (실물도 최신 캐시를 냄)."""
        up = self.world.sensor(IMU_UPVECTOR)
        gyro = self.world.sensor(IMU_GYRO)
        accel = self.world.sensor(IMU_ACCEL)
        return ImuState(
            gravity=(-up[0], -up[1], -up[2]),
            gyro_dps=tuple(math.degrees(v) for v in gyro),
            accel_mps2=accel,
            stamp=time.monotonic(),
            is_valid=True,
        )
