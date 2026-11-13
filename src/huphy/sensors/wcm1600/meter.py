"""WCM1600 — `CurrentMeter` 프로토콜 구현.

백그라운드 스레드가 시리얼을 계속 읽어 최신 `CurrentState` 만 들고 있고, `read()` 는
그것을 꺼내기만 함. `sensors/ebimu/imu.py` 와 같은 구조임.


## 제어 루프에서 기다리면 안 됨

9600 baud 에 8바이트 프레임이므로 한 프레임이 선로에 8.3ms 걸림 (시작·정지 비트
포함 10비트/바이트). 50Hz 루프 한 주기가 20ms 라, 루프 안에서 한 줄을 기다리면
주기의 절반을 전류계에 씀. 그래서 스레드가 받아 두고 `read()` 는 사전만 읽음.


## 못 열려도 멈추지 않음

`connect()` 는 실패하면 예외를 올리지만, **부르는 쪽이 그것으로 로봇을 멈추지
않음.** 전류계는 관측이지 제어가 아님 (`sensors/README.md` 의 IMU 와 같은 이유).
`huphy-run --current`, ROS `current_node` 는 경고만 남기고 계속함.


## 핸드셰이크가 없음

IMU 는 첫 패킷의 필드 수로 구성을 확인함. 여기는 확인할 구성이 없음 -- 프레임이
고정 8바이트 한 가지뿐임. 대신 `connect()` 가 첫 프레임을 기다려 **포트가 맞는지만**
봄. 기다려도 안 오면 보레이트나 포트가 틀린 것임.

시작 직후 첫 줄은 프레임 중간부터 잘려 들어오므로 버림.
"""

from __future__ import annotations

import glob
import logging
import threading
import time
from typing import Any, List, Optional

from ..current import CurrentState
from . import protocol

logger = logging.getLogger(__name__)

BAUDRATE = 9600
"""매뉴얼의 Continuous 모드 설정. 8N1. 장비에 설정이 없으므로 고정임."""

READ_TIMEOUT_S = 0.2
"""`readline()` 한 번의 대기. 스레드가 `stop()` 에 반응하는 데 걸리는 시간이기도 함."""

HANDSHAKE_S = 1.5
"""`connect()` 가 첫 프레임을 기다리는 시간.

프레임 하나가 8.3ms 이고 장비의 출력 주기는 그보다 느림. 1.5초를 다 쓰면 포트나
보레이트가 틀린 것임.
"""

PORT_GLOBS = ("/dev/serial/by-id/*FTDI*", "/dev/ttyUSB*")
"""`find_port()` 가 보는 순서. by-id 가 먼저임 -- `ttyUSB*` 는 꽂는 순서대로 붙어
**재부팅마다 달라짐.**

여러 개가 잡히면 고르지 않고 멈춤. USB 직렬 장치가 하나뿐일 때만 생략할 수 있고,
IMU 도 USB 로 붙어 있으면 `port` 를 직접 줄 것 (udev 로 고정한 링크를 권함).
"""


def find_port() -> str:
    """USB 직렬 장치를 하나 찾음. 없거나 여럿이면 멈춤."""
    for pattern in PORT_GLOBS:
        found = sorted(glob.glob(pattern))
        if len(found) == 1:
            return found[0]
        if found:
            raise ConnectionError(
                f"직렬 장치가 여럿임 {found} -- 어느 것인지 알 수 없으므로 "
                f"포트를 직접 줄 것")
    raise ConnectionError(
        "직렬 장치를 찾을 수 없음. USB 연결과 "
        "권한(sudo usermod -aG dialout $USER, 재로그인)을 확인할 것")


class Wcm1600:
    """WCM1600 전류계 하나. 로봇 전체의 DC 전류를 잼.

    `port` 를 생략하면 `find_port()` 가 찾음. 전류계와 IMU 가 같이 USB 로 붙어
    있으면 **반드시 직접 줄 것** -- 둘 다 FTDI 면 어느 것인지 알 수 없음.
    """

    def __init__(self, name: str = "bus", port: Optional[str] = None) -> None:
        self.name = name
        self.port = port
        self._serial: Any = None
        self._thread: Optional[threading.Thread] = None
        self._running = False
        self._lock = threading.Lock()
        self._latest = CurrentState()
        self.dropped = 0
        """읽었지만 버린 줄 수. 길이·부호·자리수가 매뉴얼과 다른 줄임."""
        self.frames = 0
        """제대로 읽은 프레임 수."""

    def __repr__(self) -> str:
        where = self.port or "(포트 미지정)"
        return f"Wcm1600({self.name}, {where if self.is_connected else where + ' 안 열림'})"

    @property
    def is_connected(self) -> bool:
        return self._serial is not None

    # ---- 수명 -------------------------------------------------------------
    def connect(self) -> None:
        """포트를 열고 첫 프레임을 기다림. 여러 번 불려도 안전함."""
        if self._serial is not None:
            return

        try:
            import serial  # noqa: PLC0415
        except ImportError as e:
            raise ImportError(
                f"전류계를 읽으려면 pyserial 이 필요함. `pip install pyserial` ({e})"
            ) from e

        if self.port is None:
            self.port = find_port()
        try:
            port = serial.Serial(
                port=self.port, baudrate=BAUDRATE, timeout=READ_TIMEOUT_S)
        except Exception as e:
            raise ConnectionError(
                f"{self.name}: {self.port} 를 열 수 없음: {e}\n"
                f"  포트 확인   ls /dev/ttyUSB* /dev/serial/by-id/*\n"
                f"  권한        sudo usermod -aG dialout $USER  (재로그인)"
            ) from e

        try:
            self._handshake(port)
        except Exception:
            port.close()
            raise

        self._serial = port
        self._running = True
        self._thread = threading.Thread(
            target=self._read_loop, name=f"wcm1600-{self.name}", daemon=True)
        self._thread.start()
        logger.info("전류계 연결됨: %s", self)

    def _handshake(self, port: Any) -> None:
        """첫 프레임이 오는지만 봄. 확인할 구성은 없음."""
        port.reset_input_buffer()
        deadline = time.monotonic() + HANDSHAKE_S
        seen: List[bytes] = []
        first = True
        while time.monotonic() < deadline:
            try:
                raw = port.readline()
            except Exception as e:
                raise ConnectionError(f"{self.name}: 읽기 실패: {e}") from e
            if not raw.strip():
                continue
            if first:
                first = False           # 중간부터 잘려 들어온 줄
                continue
            if protocol.parse(raw) is not None:
                return
            seen.append(raw.strip())
            if len(seen) >= 3:
                break

        if not seen:
            raise ConnectionError(
                f"{self.name}: {HANDSHAKE_S:.1f}초 동안 프레임이 없음. "
                f"포트({self.port})와 Continuous 모드 설정을 확인할 것")
        raise ConnectionError(
            f"{self.name}: 프레임 형식이 매뉴얼과 다름 -- 받은 것 {seen}. "
            f"8바이트 <부호><다섯 자리>(CR)(LF) 여야 함")

    def disconnect(self) -> None:
        """스레드를 멈추고 포트를 닫음. 여러 번 불려도 안전함."""
        self._running = False
        thread, self._thread = self._thread, None
        if thread is not None:
            thread.join(timeout=READ_TIMEOUT_S * 3)

        port, self._serial = self._serial, None
        if port is None:
            return
        try:
            port.close()
        except Exception as e:
            logger.warning("%s 종료 실패 (무시함): %s", self.name, e)

    def __enter__(self) -> "Wcm1600":
        self.connect()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.disconnect()

    # ---- 읽기 -------------------------------------------------------------
    def _read_loop(self) -> None:
        """계속 읽어 최신 값만 덮어씀. **예외로 죽지 않음.**

        큐를 두지 않는 이유는 `ebimu/imu.py` 와 같음 -- 쌓아 두면 루프가 한 번
        밀렸을 때 묵은 값부터 꺼내게 됨. 몇 개가 버려졌는지는 `age_ms` 로 드러남.
        """
        port = self._serial
        while self._running:
            try:
                raw = port.readline()
            except Exception as e:
                if self._running:
                    logger.warning("%s 시리얼 읽기 실패: %s", self.name, e)
                    self._running = False
                return
            if not raw.strip():
                continue
            self.feed(raw)

    def feed(self, raw: bytes) -> bool:
        """한 줄을 해석해 최신 값으로 둠. 버렸으면 거짓.

        스레드가 부르는 것이지만 **시리얼 없이 테스트할 수 있게** 공개함.
        """
        parsed = protocol.parse(raw)
        if parsed is None:
            self.dropped += 1
            if self.dropped == 1:
                logger.warning("%s 알 수 없는 프레임 (계속함): %r", self.name, raw.strip())
            return False
        is_ac, amps = parsed
        self.frames += 1
        with self._lock:
            self._latest = CurrentState(
                amps=amps, is_ac=is_ac, stamp=time.monotonic(), is_valid=True)
        return True

    def read(self) -> CurrentState:
        """가장 최근 값. **새로 통신하지 않음.** 한 번도 못 받았으면 `is_valid=False`."""
        with self._lock:
            return self._latest
