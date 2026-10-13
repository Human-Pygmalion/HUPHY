"""CAN 전송 계층.

프레임을 **주고받기만** 함. 안에 든 바이트가 무슨 뜻인지 모름 — 그건 벤더 코덱이 앎.

`python-can` 을 import하는 **유일한 파일**임. 위 계층은 `CanFrame` 만 다루므로,
드라이버를 바꾸거나 테스트에서 가짜 버스를 끼울 때 이 파일만 보면 됨.
`codec/mit.py` 가 라디안 변환을 혼자 떠안는 것과 같은 방식임.


## 채널은 이 코드 밖에서 올라옴

socketcan 채널은 커널이 관리함. 속도를 포함한 설정이 채널을 올릴 때 정해지고,
이 코드는 이미 올라와 있는 채널에 붙기만 함. 올리는 법은 어댑터에 따라 다름.

    네이티브        sudo ip link set can0 up type can bitrate 1000000
    USB (slcand)    sudo slcand -o -c -s8 /dev/canable1 canr
                    sudo ip link set canr up

slcan 은 속도를 어댑터가 들고 있어서 `ip link` 에 `bitrate` 를 못 넘김.
채널이 없거나 내려가 있으면 `connect()` 가 실패함. 진입점들은 `bring_up_hint()` 로
채널 이름에 맞는 명령을 알려 줌.


## 전송과 수거를 나눔

한 번에 묶어 두면 버스가 둘일 때 두 다리의 명령 시각이 벌어짐 (이슈 #10).

    보내기(can0) -> 보내기(can1) -> 수거(can0) -> 수거(can1)

이 순서가 되려면 `send_many` 와 `drain` 이 별개여야 함. 두 버스는 물리적으로
독립이라 전송이 진짜로 겹침.

수거가 더 비쌈. `recv()` 는 큐가 비면 타임아웃만큼 블로킹하므로, 순차로 수거하면
그 시간이 버스 수만큼 곱해짐.


## 드레인 비용

100Hz 루프의 예산은 10ms 임. 그 안에서 `recv(timeout=2ms)` 를 한 번 부르고 큐가
비어 있으면 **2ms 를 그냥 버림.**

그래서 짧게 여러 번 폴링하고, 기다릴 총 시간을 따로 둠. 기대한 개수가 다 오면 즉시
빠져나감 — 정상 상황에서는 총 시간을 다 쓰지 않음.


## 스레드

지금은 제어 루프 한 곳에서만 부름. 송신에만 락을 걸어 프레임이 섞이지 않게 함.

## 수신 스레드

버스마다 하나씩 둘 수 있음 (`reader=True`). 켜면 `recv` 를 기다리는 일이 제어
루프에서 빠짐 — 프레임은 백그라운드에서 큐에 쌓이고, `drain` 은 큐에서 꺼내기만
함.

    켜기 전    수거 대기가 버스 수만큼 곱해짐      can0 기다림 -> can1 기다림
    켜고 나서  두 버스가 동시에 받음               큐에서 꺼내기만

**수신 스레드는 송신 락을 잡지 않음.** 잡으면 `recv` 에서 블로킹하는 동안 송신이
밀려, 스레드를 둔 이유가 사라짐. 공유하는 것은 프레임 큐뿐이고, `deque` 의
`append`/`popleft` 가 원자적이라 락이 필요 없음.

닫을 때는 **스레드를 먼저 세우고** 채널을 닫음. 순서가 반대면 닫힌 소켓에서
`recv` 를 부르게 됨.
"""

from __future__ import annotations

import logging
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence

logger = logging.getLogger(__name__)

DEFAULT_INTERFACE = "socketcan"
"""라즈베리파이 + 리눅스 커널 CAN. 다른 값은 테스트용 `virtual` 정도임."""

def bring_up_hint(channels: Iterable[str]) -> str:
    """채널을 올리는 명령. 연결에 실패했을 때 진입점들이 찍음.

    **두 방식을 같이 찍음.** 채널 이름(`can1`, `canr`)으로는 USB 어댑터인지 네이티브
    인지 알 수 없음 -- 0.5 는 USB 어댑터를 `can1` 로 올렸고 1.0 은 `canr` 로 올림.
    USB 쪽은 `ip link` 에 `bitrate` 를 못 넘기고 속도를 `slcand -s8` 로 줌. 어느
    어댑터(`/dev/canableN`)가 어느 채널인지는 루트 README 의 1.0 명령을 볼 것.
    """
    names = list(dict.fromkeys(channels))
    lines = ["채널이 올라와 있는지 확인할 것:", "  USB 어댑터 (slcand, -s8 = 1 Mbps):"]
    lines += [
        f"    sudo slcand -o -c -s8 /dev/<어댑터> {ch} && sudo ip link set {ch} up"
        for ch in names
    ]
    lines.append("  네이티브 CAN:")
    lines += [f"    sudo ip link set {ch} up type can bitrate 1000000" for ch in names]
    lines.append("  확인: " + "  ".join(f"ip -details link show {ch}" for ch in names))
    return "\n".join(lines)


DEFAULT_POLL_S = 0.0002
"""recv 한 번의 대기 시간. 짧게 여러 번 도는 쪽이 한 번 길게 기다리는 것보다 나음."""

DEFAULT_DRAIN_S = 0.002
"""수거에 쓸 총 시간. 100Hz 예산 10ms 의 20%."""

RX_QUEUE_MAX = 4096
"""수신 스레드가 쌓아 둘 최대 프레임 수. 넘으면 **오래된 것부터 버림.**

버리는 쪽이 맞음 -- 소비가 밀렸다는 뜻이고, 그때 남은 옛 프레임은 이미 지난 주기의
상태임. 몇 개 버렸는지는 `rx_dropped` 로 셈.
"""

READER_JOIN_S = 0.5
"""수신 스레드가 끝나기를 기다리는 시간. `recv` 한 번의 대기보다 넉넉해야 함."""


@dataclass(frozen=True)
class CanFrame:
    """CAN 프레임 하나. 바이트의 뜻은 모름.

    `python-can` 의 `Message` 를 그대로 쓰지 않는 이유: 그걸 위 계층에 흘리면
    `python-can` 이 없는 환경에서 import가 깨지고 테스트도 못 돌림.
    """

    can_id: int
    data: bytes
    is_extended: bool = False
    """MIT 프로토콜은 11-bit 표준 프레임이라 `False`. private 은 29-bit 확장임."""

    stamp: float = 0.0
    """수신 시각. 보낼 때는 쓰지 않음."""


@dataclass
class CanCounters:
    """무슨 일이 있었는지 세는 곳. 텔레메트리로 나감.

    한 번의 실패는 흔한 일이라 예외를 던지지 않음. **몇 번 연속인지**가 의미 있고,
    그 판단은 호출부가 함.
    """

    tx_errors: int = 0
    rx_errors: int = 0
    frames_sent: int = 0
    frames_received: int = 0
    drain_timeouts: int = 0
    """기대한 개수를 못 채우고 총 시간을 다 쓴 횟수."""

    rx_dropped: int = 0
    """수신 큐가 차서 버린 프레임 수. **수신 스레드를 켰을 때만 늘어남.**

    0 이 아니면 소비가 밀린 것임 -- 제어 루프가 주기를 못 지키고 있다는 신호로,
    `loop_dt` 와 같이 봐야 함.
    """

    error_frames: int = 0
    """들어온 CAN 오류 프레임 수. **수신 큐에 넣지 않고 세기만 함.**

    `python-can` 은 오류 프레임을 기본으로 받음. 큐에 섞이면 `drain(expect=6)` 이
    그것을 응답 한 개로 세어, 진짜 응답이 다음 주기로 밀리고 그 밀림이 쌓임 -- 지연이
    늘고 모터별 무응답이 고르게 높아짐. slcan 은 오류 프레임을 안 만들어 안 보였고
    gs_usb 에서 드러남. 0 이 아니면 버스 오류가 실제로 있는 것임 (배선·노이즈·타이밍).
    """

    def reset(self) -> None:
        self.tx_errors = 0
        self.rx_errors = 0
        self.frames_sent = 0
        self.frames_received = 0
        self.drain_timeouts = 0
        self.rx_dropped = 0
        self.error_frames = 0

    def as_fields(self) -> Dict[str, int]:
        """0이어도 모든 키를 냄. 필드가 중간에 생기면 그래프가 끊김."""
        return {
            "can.tx_errors": self.tx_errors,
            "can.rx_errors": self.rx_errors,
            "can.frames_sent": self.frames_sent,
            "can.frames_received": self.frames_received,
            "can.drain_timeouts": self.drain_timeouts,
            "can.rx_dropped": self.rx_dropped,
            "can.error_frames": self.error_frames,
        }


class CanBus:
    """CAN 채널 하나. 다리 하나가 버스 하나를 씀.

    되돌리기 어려운 조작은 여기 없음 — 이 계층은 바이트의 뜻을 모르므로 애초에
    그런 명령을 만들 수 없음.
    """

    def __init__(
        self,
        channel: str,
        *,
        interface: str = DEFAULT_INTERFACE,
        reader: bool = False,
    ) -> None:
        self.channel = channel
        self.interface = interface
        self.use_reader = bool(reader)
        """수신 스레드를 둘지. `connect()` 가 이 값을 보고 시작함.

        **기본은 끔.** 스레드가 없으면 `drain` 이 직접 `recv` 를 부름 -- 다리
        하나만 돌릴 때는 기다릴 버스가 하나뿐이라 얻는 것이 없음.
        """

        self._bus = None
        self._tx_lock = threading.Lock()
        self.counters = CanCounters()

        self._rx: "deque[CanFrame]" = deque(maxlen=RX_QUEUE_MAX)
        """수신 스레드가 쌓는 곳. **락이 없음** -- `deque` 의 `append`/`popleft`
        가 원자적이라 필요 없고, 송신 락과 얽히지 않게 하려는 것이기도 함.
        """

        self._reader: Optional[threading.Thread] = None
        self._reader_stop = threading.Event()

    def __repr__(self) -> str:
        state = "연결됨" if self.is_connected else "끊김"
        return f"CanBus({self.channel!r}, {self.interface}, {state})"

    # ---- 수명 -------------------------------------------------------------
    @property
    def is_connected(self) -> bool:
        return self._bus is not None

    def connect(self) -> None:
        """이미 올라와 있는 채널에 붙음.

        속도는 채널을 올릴 때 정해짐 -- 명령은 `bring_up_hint()` 참조.

        `python-can` 은 여기서 처음 import됨. 모듈 최상단에서 하면 이 파일을
        import하는 것만으로 `python-can` 이 필요해지고, 순수 계산 계층의 테스트까지
        같이 막힘.
        """
        if self.is_connected:
            return

        try:
            import can  # noqa: PLC0415
        except ImportError as e:
            raise ImportError(
                "CAN 통신에는 python-can 이 필요함. `pip install python-can` "
                "(순수 계산 계층은 이것 없이도 동작함)"
            ) from e

        try:
            self._bus = can.interface.Bus(
                channel=self.channel, interface=self.interface
            )
        except Exception as e:
            raise ConnectionError(
                f"{self.channel} ({self.interface}) 를 열 수 없음: {e}"
            ) from e

        logger.info("CAN 연결됨: %s", self)

        if self.use_reader:
            self.start_reader()

    def disconnect(self) -> None:
        """채널을 닫음. 여러 번 불려도 안전함.

        **수신 스레드를 먼저 세움.** 순서가 반대면 닫힌 소켓에서 `recv` 를 부름.
        """
        self.stop_reader()
        if self._bus is None:
            return
        try:
            self._bus.shutdown()
        except Exception as e:
            logger.warning("%s 종료 중 예외 (무시함): %s", self.channel, e)
        finally:
            self._bus = None

    def __enter__(self) -> "CanBus":
        self.connect()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.disconnect()

    def _require(self):
        if self._bus is None:
            raise ConnectionError(f"{self.channel} 이 연결되지 않음. connect() 를 먼저 부를 것")
        return self._bus

    # ---- 수신 스레드 -------------------------------------------------------
    @property
    def reader_running(self) -> bool:
        return self._reader is not None and self._reader.is_alive()

    def start_reader(self, *, poll_s: float = DEFAULT_POLL_S) -> None:
        """백그라운드로 프레임을 받아 큐에 쌓기 시작함. 여러 번 불려도 안전함.

        **제어 루프에서 기다림을 뺌.** 버스가 둘이면 순차 수거의 총 대기가 두 배가
        되는데, 스레드를 두면 두 채널이 동시에 받고 `drain` 은 큐에서 꺼내기만 함.

        `daemon=True` 로 둠 -- 프로세스가 죽을 때 이 스레드가 붙잡지 않게 함.
        정상 종료 경로는 `stop_reader` 임.
        """
        if self.reader_running:
            return
        self._require()
        self._reader_stop.clear()
        self._reader = threading.Thread(
            target=self._read_loop,
            args=(float(poll_s),),
            name=f"can-rx-{self.channel}",
            daemon=True,
        )
        self._reader.start()
        logger.info("%s 수신 스레드 시작", self.channel)

    def stop_reader(self) -> None:
        """수신 스레드를 세움. 여러 번 불려도 안전함.

        기다려 주는 이유: 스레드가 아직 `recv` 안에 있는데 채널을 닫으면 닫힌
        소켓을 건드림.
        """
        if self._reader is None:
            return
        self._reader_stop.set()
        self._reader.join(timeout=READER_JOIN_S)
        if self._reader.is_alive():
            logger.warning("%s 수신 스레드가 제때 안 끝남", self.channel)
        self._reader = None

    def _read_loop(self, poll_s: float) -> None:
        """스레드 본체. **송신 락을 잡지 않음.**

        잡으면 `recv` 에서 블로킹하는 동안 송신이 밀려, 스레드를 둔 이유가 사라짐.

        예외가 나도 멈추지 않음 -- 채널이 닫히는 중일 수도 있고, 한 번의 실패로
        수신이 통째로 죽으면 그 뒤 모든 모터가 무응답으로 보임. 멈춤 신호는
        `stop_reader` 뿐임.
        """
        bus = self._bus
        while not self._reader_stop.is_set():
            try:
                msg = bus.recv(timeout=poll_s)
            except Exception as e:
                if self._reader_stop.is_set():
                    break
                self.counters.rx_errors += 1
                logger.debug("%s 수신 스레드 예외: %s", self.channel, e)
                time.sleep(poll_s)
                continue
            if msg is None:
                continue
            if getattr(msg, "is_error_frame", False):
                self.counters.error_frames += 1
                continue
            if len(self._rx) == self._rx.maxlen:
                # 큐가 참 -- 소비가 밀린 것임. 오래된 것부터 밀려나감.
                self.counters.rx_dropped += 1
            self._rx.append(_frame_of(msg))

    def _take(self, limit: int) -> List[CanFrame]:
        """큐에 있는 것을 최대 `limit` 개 꺼냄. 없으면 빈 목록."""
        out: List[CanFrame] = []
        while len(out) < limit:
            try:
                out.append(self._rx.popleft())
            except IndexError:
                break
        return out

    # ---- 송신 -------------------------------------------------------------
    def send(self, frame: CanFrame) -> bool:
        """한 프레임을 보냄. 실패하면 세고 `False` 를 반환함."""
        return self.send_many([frame]) == 1

    def send_many(self, frames: Sequence[CanFrame]) -> int:
        """여러 프레임을 **연속으로** 보냄. 보낸 개수를 반환함.

        락을 한 번만 잡음. 프레임마다 잡았다 놓으면 그 사이에 다른 프레임이 끼어들어
        한 관절의 명령들이 버스에서 흩어짐.

        중간에 실패해도 나머지를 계속 보냄. 다리 하나가 6모터인데 첫 프레임이
        실패했다고 멈추면 나머지 5개가 직전 명령을 유지해 자세가 어긋남.
        """
        bus = self._require()
        import can  # noqa: PLC0415

        sent = 0
        with self._tx_lock:
            for f in frames:
                msg = can.Message(
                    arbitration_id=f.can_id,
                    data=f.data,
                    is_extended_id=f.is_extended,
                )
                try:
                    bus.send(msg)
                    sent += 1
                except Exception as e:
                    self.counters.tx_errors += 1
                    logger.debug("%s 송신 실패 id=0x%X: %s", self.channel, f.can_id, e)
        self.counters.frames_sent += sent
        return sent

    # ---- 수신 -------------------------------------------------------------
    def drain(
        self,
        *,
        expect: Optional[int] = None,
        timeout_s: float = DEFAULT_DRAIN_S,
        poll_s: float = DEFAULT_POLL_S,
        max_frames: int = 4096,
    ) -> List[CanFrame]:
        """도착한 프레임을 모음.

        `expect` 개를 채우면 **즉시** 빠져나감 — 정상 상황에서는 `timeout_s` 를 다
        쓰지 않음. `None` 이면 버스가 조용해질 때까지 읽음.

        예외를 던지지 않음. 한 주기에 프레임이 덜 오는 것은 흔한 일이고, 그때마다
        제어 루프가 죽으면 안 됨. 몇 개 왔는지는 반환값 길이로 알 수 있음.

        **수신 스레드가 돌고 있으면 큐에서 꺼내기만 함.** `recv` 를 부르지 않으므로
        두 버스의 대기가 겹침.
        """
        if self.reader_running:
            return self._drain_queue(
                expect=expect, timeout_s=timeout_s, poll_s=poll_s, max_frames=max_frames
            )

        bus = self._require()
        out: List[CanFrame] = []
        deadline = time.monotonic() + max(0.0, float(timeout_s))

        # 마감은 **기다릴 때만** 봄. 이미 큐에 있는 것은 마감과 무관하게 다 읽음.
        # 읽기 전에 마감을 보면 timeout_s=0 일 때 한 프레임도 못 꺼냄.
        while len(out) < max_frames:
            if expect is not None and len(out) >= expect:
                break                       # expect=0 이면 recv 자체를 부르지 않음
            try:
                msg = bus.recv(timeout=poll_s)
            except Exception as e:
                self.counters.rx_errors += 1
                logger.debug("%s 수신 실패: %s", self.channel, e)
                break

            if msg is not None:
                if getattr(msg, "is_error_frame", False):
                    self.counters.error_frames += 1
                else:
                    out.append(_frame_of(msg))
                continue

            # 큐가 비었음
            if expect is None:
                break                       # 조용해짐 -- 다 받은 것으로 봄
            if time.monotonic() >= deadline:
                self.counters.drain_timeouts += 1
                break

        self.counters.frames_received += len(out)
        return out

    def flush_rx(self, *, max_frames: int = 4096) -> int:
        """큐에 남은 것을 버리고 개수를 반환함.

        직전 주기의 응답이 남아 있으면 이번 주기의 것으로 오해함. 특히 고장 조회는
        일반 상태 프레임과 CAN ID 가 같아 구분되지 않으므로, 보내기 전에 비워야 함.
        """
        return len(self.drain(timeout_s=0.0, poll_s=0.0, max_frames=max_frames))

    def _drain_queue(
        self,
        *,
        expect: Optional[int],
        timeout_s: float,
        poll_s: float,
        max_frames: int,
    ) -> List[CanFrame]:
        """수신 스레드가 쌓아 둔 것을 꺼냄. `recv` 를 부르지 않음.

        `expect` 를 채울 때까지 짧게 자며 기다림. 자는 동안 다른 버스도 자기
        스레드로 받고 있으므로, 순차 수거처럼 대기가 곱해지지 않음.
        """
        out = self._take(max_frames if expect is None else expect)
        if expect is None or len(out) >= expect:
            self.counters.frames_received += len(out)
            return out

        deadline = time.monotonic() + max(0.0, float(timeout_s))
        while len(out) < expect and time.monotonic() < deadline:
            time.sleep(max(0.0, float(poll_s)))
            out.extend(self._take(expect - len(out)))

        if len(out) < expect:
            self.counters.drain_timeouts += 1
        self.counters.frames_received += len(out)
        return out


def _frame_of(msg) -> CanFrame:
    """`python-can` 의 `Message` 를 중립 프레임으로. **여기서만 변환함.**"""
    return CanFrame(
        can_id=int(msg.arbitration_id),
        data=bytes(msg.data),
        is_extended=bool(msg.is_extended_id),
        stamp=time.monotonic(),
    )


def drain_all(
    buses: Iterable[CanBus],
    *,
    expect_per_bus: Optional[int] = None,
    timeout_s: float = DEFAULT_DRAIN_S,
) -> Dict[str, List[CanFrame]]:
    """여러 버스를 수거함. 채널 이름으로 묶어 반환함.

    **수신 스레드를 켜 두면 대기가 겹침** — 각 버스가 자기 스레드로 이미 받아 뒀고
    여기서는 큐에서 꺼내기만 함. 끄면 순차라 총 대기가 버스 수만큼 곱해짐
    (이슈 #10).

    이 함수를 두는 이유: 호출부가 "전부 보내고 전부 수거" 순서를 지키게 됨.
    버스별로 보내고 바로 수거하면 두 다리의 명령 시각이 벌어짐.
    """
    return {
        b.channel: b.drain(expect=expect_per_bus, timeout_s=timeout_s) for b in buses
    }
