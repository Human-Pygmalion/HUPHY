"""CAN 전송 계층 테스트 — 하드웨어 없이 실행됨.

    PYTHONPATH=src python3 -m pytest tests -q

`python-can` 을 가짜 모듈로 갈아끼워 확인함. `canbus.py` 가 `import can` 을 함수
안에서 하기 때문에 가능함 — 최상단에서 했다면 이 파일 자체가 import되지 않음.

가짜 버스는 실제 타이밍을 재현하지 않음. 여기서 확인하는 것은 **순서·개수·집계**임.
전송 지연이나 중재는 실물에서만 확인됨.
"""

import sys
import types
from collections import deque

import pytest

from huphy.motors.canbus import (
    CanBus,
    CanCounters,
    CanFrame,
    drain_all,
)


# ===========================================================================
# 가짜 python-can
# ===========================================================================
class FakeMessage:
    def __init__(self, arbitration_id, data, is_extended_id=False, is_error_frame=False):
        self.arbitration_id = arbitration_id
        self.data = data
        self.is_extended_id = is_extended_id
        self.is_error_frame = is_error_frame


def error_frame():
    """gs_usb 가 버스 오류 때 올려 보내는 것. id 와 데이터는 오류 종류임."""
    return FakeMessage(0x20000004, bytes([0, 0x08, 0, 0, 0, 0, 0, 0]), True, True)


class FakeCanBus:
    """보낸 것을 기록하고, 미리 넣어둔 것을 돌려주는 버스."""

    instances = []
    events = []
    """모든 버스가 공유하는 시간 순서. 버스 간 순서를 확인하는 데 씀."""

    def __init__(self, **kwargs):
        self.kwargs = kwargs
        self.channel = kwargs.get("channel")
        self.sent = []
        self.rx = deque()
        self.fail_ids = set()
        self.recv_calls = 0
        self.shutdown_called = 0
        FakeCanBus.instances.append(self)

    def send(self, msg):
        FakeCanBus.events.append(("send", self.channel))
        if msg.arbitration_id in self.fail_ids:
            raise OSError("전송 실패 (가짜)")
        self.sent.append(msg)

    def recv(self, timeout=None):
        FakeCanBus.events.append(("recv", self.channel))
        self.recv_calls += 1
        return self.rx.popleft() if self.rx else None

    def shutdown(self):
        self.shutdown_called += 1


@pytest.fixture
def fake_can(monkeypatch):
    FakeCanBus.instances = []
    FakeCanBus.events = []
    mod = types.ModuleType("can")
    mod.Message = FakeMessage
    mod.interface = types.SimpleNamespace(Bus=FakeCanBus)
    monkeypatch.setitem(sys.modules, "can", mod)
    return mod


@pytest.fixture
def bus(fake_can):
    b = CanBus("can1")
    b.connect()
    return b


def frame(i, byte=0):
    return CanFrame(can_id=i, data=bytes([byte] * 8))


# ===========================================================================
# CanFrame
# ===========================================================================
class TestCanFrame:
    def test_defaults_to_standard_frame(self):
        """MIT 프로토콜은 11-bit 표준 프레임임. private 만 확장을 씀."""
        assert frame(10).is_extended is False

    def test_is_frozen(self):
        """보낸 뒤에 내용이 바뀌면 로그와 실제가 어긋남."""
        with pytest.raises(Exception):
            frame(10).can_id = 11

    def test_does_not_need_python_can(self):
        """위 계층이 CanFrame 만 다루므로 python-can 없이 프레임을 만들 수 있음."""
        f = CanFrame(can_id=10, data=b"\x00" * 8)
        assert len(f.data) == 8


# ===========================================================================
# 인터페이스
# ===========================================================================
class TestInterface:
    def test_defaults_to_socketcan(self, fake_can):
        """라즈베리파이 + 리눅스 커널 CAN. 이 프로젝트의 유일한 실사용 경로임."""
        assert CanBus("can0").interface == "socketcan"

    def test_explicit_wins(self, fake_can):
        """테스트에서 virtual 을 끼울 때 씀."""
        assert CanBus("can0", interface="virtual").interface == "virtual"


# ===========================================================================
# 수명
# ===========================================================================
class TestLifecycle:
    def test_not_connected_initially(self, fake_can):
        assert CanBus("can1").is_connected is False

    def test_connect_passes_channel_and_interface(self, bus):
        """채널과 인터페이스만 넘김. 속도는 커널이 관리함.

            sudo ip link set can0 up type can bitrate 1000000
        """
        assert FakeCanBus.instances[-1].kwargs == {
            "channel": "can1",
            "interface": "socketcan",
        }

    def test_connect_twice_is_noop(self, bus):
        """두 번 열면 소켓이 두 개 생겨 프레임이 중복 수신됨."""
        bus.connect()
        assert len(FakeCanBus.instances) == 1

    def test_disconnect_twice_is_safe(self, bus):
        bus.disconnect()
        bus.disconnect()
        assert bus.is_connected is False

    def test_disconnect_survives_driver_exception(self, bus):
        """종료 중 예외가 나도 끊긴 것으로 처리함.

        여기서 예외를 올리면 정리 경로가 중간에 멈춰 다음 정리가 실행되지 않음.
        """
        FakeCanBus.instances[-1].shutdown = lambda: (_ for _ in ()).throw(OSError("bye"))
        bus.disconnect()
        assert bus.is_connected is False

    def test_context_manager_closes_on_exception(self, fake_can):
        b = CanBus("can1")
        with pytest.raises(RuntimeError):
            with b:
                raise RuntimeError("제어 루프에서 터진 예외")
        assert b.is_connected is False
        assert FakeCanBus.instances[-1].shutdown_called == 1

    def test_operations_require_connection(self, fake_can):
        """연결 전에 보내면 조용히 실패하지 않고 에러가 남."""
        b = CanBus("can1")
        with pytest.raises(ConnectionError, match="연결되지 않음"):
            b.send(frame(10))
        with pytest.raises(ConnectionError):
            b.drain()

    def test_open_failure_names_the_channel(self, fake_can, monkeypatch):
        """어느 채널인지 알아야 배선을 확인할 수 있음."""
        def boom(**kwargs):
            raise OSError("no such device")

        monkeypatch.setattr(fake_can.interface, "Bus", boom)
        with pytest.raises(ConnectionError, match="can1"):
            CanBus("can1").connect()


# ===========================================================================
# 송신
# ===========================================================================
class TestSend:
    def test_send_many_preserves_order(self, bus):
        """관절 순서대로 나가야 함. 버스에서 흩어지면 자세가 어긋남."""
        bus.send_many([frame(i) for i in (7, 8, 9, 10, 11, 12)])
        assert [m.arbitration_id for m in FakeCanBus.instances[-1].sent] == [7, 8, 9, 10, 11, 12]

    def test_send_many_continues_after_failure(self, bus):
        """첫 프레임이 실패했다고 멈추면 나머지 5개가 직전 명령을 유지해 자세가 어긋남."""
        FakeCanBus.instances[-1].fail_ids = {7}
        sent = bus.send_many([frame(i) for i in (7, 8, 9, 10, 11, 12)])
        assert sent == 5
        assert [m.arbitration_id for m in FakeCanBus.instances[-1].sent] == [8, 9, 10, 11, 12]
        assert bus.counters.tx_errors == 1

    def test_counters_track_only_successes(self, bus):
        FakeCanBus.instances[-1].fail_ids = {8}
        bus.send_many([frame(7), frame(8)])
        assert bus.counters.frames_sent == 1
        assert bus.counters.tx_errors == 1

    def test_send_one_reports_failure(self, bus):
        FakeCanBus.instances[-1].fail_ids = {10}
        assert bus.send(frame(10)) is False
        assert bus.send(frame(11)) is True

    def test_payload_and_frame_type_are_passed_through(self, bus):
        """전송 계층은 바이트의 뜻을 모름. 받은 그대로 실어 보냄."""
        payload = bytes([0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08])
        bus.send(CanFrame(can_id=0x1234, data=payload, is_extended=True))
        msg = FakeCanBus.instances[-1].sent[0]
        assert msg.data == payload
        assert msg.is_extended_id is True

    def test_empty_send_is_harmless(self, bus):
        assert bus.send_many([]) == 0


# ===========================================================================
# 수신
# ===========================================================================
class TestDrain:
    def test_collects_what_arrived(self, bus):
        raw = FakeCanBus.instances[-1]
        raw.rx.extend(FakeMessage(i, bytes(8)) for i in (10, 11, 12))
        got = bus.drain()
        assert [f.can_id for f in got] == [10, 11, 12]
        assert bus.counters.frames_received == 3

    def test_expect_exits_early(self, bus):
        """기대한 개수가 다 오면 총 시간을 다 쓰지 않음.

        정상 주기에서 예산을 쓰지 않는 것이 핵심임 -- 100Hz 에서 2ms 를 매번 버리면
        20% 를 잃음.
        """
        raw = FakeCanBus.instances[-1]
        raw.rx.extend(FakeMessage(i, bytes(8)) for i in range(6))
        got = bus.drain(expect=6, timeout_s=1.0)
        assert len(got) == 6
        assert raw.recv_calls == 6      # 더 읽지 않음
        assert bus.counters.drain_timeouts == 0

    def test_missing_response_is_not_an_error(self, bus):
        """한 모터가 한 주기 빠지는 것은 흔한 일임. 그때마다 루프가 죽으면 안 됨.

        예외 대신 개수로 알림. 몇 주기 연속인지는 호출부가 판단함.
        """
        raw = FakeCanBus.instances[-1]
        raw.rx.extend(FakeMessage(i, bytes(8)) for i in (10, 11))
        got = bus.drain(expect=6, timeout_s=0.005, poll_s=0.0)
        assert len(got) == 2
        assert bus.counters.drain_timeouts == 1

    def test_error_frames_do_not_count_as_responses(self, bus):
        """오류 프레임이 `expect` 칸을 차지하면 진짜 응답이 다음 주기로 밀림.

        gs_usb 로 바꾼 뒤 모터별 무응답이 25% 넘게 고르게 나왔던 원인임.
        """
        raw = FakeCanBus.instances[-1]
        raw.rx.extend([error_frame(), FakeMessage(10, bytes(8)), error_frame()])
        raw.rx.extend(FakeMessage(i, bytes(8)) for i in (11, 12))
        got = bus.drain(expect=3, timeout_s=0.005, poll_s=0.0)
        assert [f.can_id for f in got] == [10, 11, 12]
        assert bus.counters.error_frames == 2
        assert bus.counters.frames_received == 3

    def test_stops_when_quiet_without_expect(self, bus):
        """expect 가 없으면 조용해진 시점을 끝으로 봄."""
        raw = FakeCanBus.instances[-1]
        raw.rx.append(FakeMessage(10, bytes(8)))
        got = bus.drain(timeout_s=1.0)
        assert len(got) == 1
        assert raw.recv_calls == 2      # 하나 읽고, 한 번 비어서 멈춤

    def test_recv_error_stops_without_raising(self, bus):
        raw = FakeCanBus.instances[-1]
        raw.recv = lambda timeout=None: (_ for _ in ()).throw(OSError("버스 오프"))
        assert bus.drain() == []
        assert bus.counters.rx_errors == 1

    def test_max_frames_caps_the_loop(self, bus):
        """폭주하는 노드가 있어도 루프가 갇히지 않음."""
        raw = FakeCanBus.instances[-1]
        raw.rx.extend(FakeMessage(1, bytes(8)) for _ in range(100))
        assert len(bus.drain(timeout_s=1.0, max_frames=10)) == 10

    def test_frame_fields_are_carried_over(self, bus):
        raw = FakeCanBus.instances[-1]
        raw.rx.append(FakeMessage(0x1F, bytes([9] * 8), is_extended_id=True))
        f = bus.drain()[0]
        assert f.can_id == 0x1F
        assert f.data == bytes([9] * 8)
        assert f.is_extended is True
        assert f.stamp > 0.0

    def test_flush_discards_leftovers(self, bus):
        """직전 주기의 응답이 남아 있으면 이번 주기의 것으로 오해함.

        고장 조회는 일반 상태 프레임과 CAN ID 가 같아 구분되지 않으므로 특히 중요함.
        """
        raw = FakeCanBus.instances[-1]
        raw.rx.extend(FakeMessage(i, bytes(8)) for i in range(4))
        assert bus.flush_rx() == 4
        assert bus.drain() == []


# ===========================================================================
# 여러 버스
# ===========================================================================
class TestDrainAll:
    def test_groups_by_channel(self, fake_can):
        left, right = CanBus("can0"), CanBus("can1")
        left.connect()
        right.connect()
        FakeCanBus.instances[-2].rx.append(FakeMessage(1, bytes(8)))
        FakeCanBus.instances[-1].rx.extend(FakeMessage(i, bytes(8)) for i in (7, 8))

        got = drain_all([left, right])
        assert [f.can_id for f in got["can0"]] == [1]
        assert [f.can_id for f in got["can1"]] == [7, 8]

    def test_send_all_precedes_drain_all(self, fake_can):
        """두 다리의 명령 시각을 붙이려면 전송을 먼저 몰아야 함 (이슈 #10).

        버스별로 보내고 바로 수거하면 오른다리 전송이 왼다리 수거 뒤로 밀림.
        수거는 큐가 빌 때까지 기다리므로 그 지연이 그대로 전송 지연이 됨.

        `send_many` 와 `drain` 이 별개이기 때문에 이 순서를 짤 수 있음.
        """
        left, right = CanBus("can0"), CanBus("can1")
        left.connect()
        right.connect()
        FakeCanBus.events.clear()

        left.send_many([frame(i) for i in (1, 2, 3)])
        right.send_many([frame(i) for i in (7, 8, 9)])
        drain_all([left, right], expect_per_bus=0)

        kinds = [kind for kind, _ in FakeCanBus.events]
        assert kinds == ["send"] * 6              # 수거가 전송 사이에 끼지 않음
        assert [ch for _, ch in FakeCanBus.events] == ["can0"] * 3 + ["can1"] * 3

    def test_expect_zero_does_not_wait(self, fake_can):
        """수거할 것이 없으면 recv 자체를 부르지 않음.

        expect=0 인데도 폴링을 돌면 예산을 그냥 버림.
        """
        b = CanBus("can1")
        b.connect()
        b.drain(expect=0, timeout_s=1.0)
        assert FakeCanBus.instances[-1].recv_calls == 0


# ===========================================================================
# 집계
# ===========================================================================
class TestCounters:
    def test_all_keys_present_when_zero(self):
        """0이어도 모든 키를 냄. 필드가 중간에 생기면 그래프가 끊김."""
        fields = CanCounters().as_fields()
        assert set(fields) == {
            "can.tx_errors",
            "can.rx_errors",
            "can.frames_sent",
            "can.frames_received",
            "can.drain_timeouts",
            "can.rx_dropped",
            "can.error_frames",
        }
        assert all(v == 0 for v in fields.values())

    def test_reset_clears_everything(self):
        c = CanCounters(tx_errors=3, frames_sent=100, drain_timeouts=2)
        c.reset()
        assert all(v == 0 for v in c.as_fields().values())

    def test_prefixed_for_telemetry(self):
        """다른 계층의 카운터와 섞이지 않게 접두사를 붙임."""
        assert all(k.startswith("can.") for k in CanCounters().as_fields())


# ===========================================================================
# 수신 스레드
# ===========================================================================
class TestReader:
    def test_off_by_default(self, bus):
        """다리 하나면 기다릴 버스가 하나뿐이라 얻는 것이 없음."""
        assert not bus.reader_running

    def test_connect_starts_it_when_asked(self, fake_can):
        b = CanBus("can1", reader=True)
        b.connect()
        try:
            assert b.reader_running
        finally:
            b.disconnect()

    def test_disconnect_stops_it(self, fake_can):
        b = CanBus("can1", reader=True)
        b.connect()
        b.disconnect()
        assert not b.reader_running

    def test_the_thread_stops_before_the_channel_closes(self, fake_can):
        """순서가 반대면 닫힌 소켓에서 recv 를 부름."""
        b = CanBus("can1", reader=True)
        b.connect()
        raw = FakeCanBus.instances[-1]
        b.disconnect()
        assert not b.reader_running
        assert raw.shutdown_called == 1

    def test_start_twice_is_noop(self, fake_can):
        b = CanBus("can1", reader=True)
        b.connect()
        try:
            first = b._reader
            b.start_reader()
            assert b._reader is first
        finally:
            b.disconnect()

    def test_stop_twice_is_safe(self, fake_can):
        b = CanBus("can1", reader=True)
        b.connect()
        b.stop_reader()
        b.stop_reader()
        assert not b.reader_running

    def test_drain_reads_from_the_queue(self, fake_can):
        """스레드가 돌면 drain 은 recv 를 부르지 않음 -- 두 버스의 대기가 겹침."""
        b = CanBus("can1", reader=True)
        b.connect()
        try:
            raw = FakeCanBus.instances[-1]
            raw.rx.extend(FakeMessage(i, bytes(8)) for i in (7, 8))
            got = _wait_for(b, 2)
            assert [f.can_id for f in got] == [7, 8]
        finally:
            b.disconnect()

    def test_the_reader_drops_error_frames(self, fake_can):
        """큐에 넣지 않고 세기만 함. 수신 스레드 경로도 같아야 함."""
        b = CanBus("can1", reader=True)
        b.connect()
        try:
            raw = FakeCanBus.instances[-1]
            raw.rx.extend([error_frame(), FakeMessage(7, bytes(8)), error_frame()])
            raw.rx.append(FakeMessage(8, bytes(8)))
            got = _wait_for(b, 2)
            assert [f.can_id for f in got] == [7, 8]
            assert b.counters.error_frames == 2
        finally:
            b.disconnect()

    def test_frames_keep_their_payload(self, fake_can):
        b = CanBus("can1", reader=True)
        b.connect()
        try:
            payload = bytes([1, 2, 3, 4, 5, 6, 7, 8])
            FakeCanBus.instances[-1].rx.append(FakeMessage(0x1234, payload, True))
            got = _wait_for(b, 1)
            assert got[0].data == payload
            assert got[0].is_extended is True
        finally:
            b.disconnect()

    def test_a_short_drain_counts_a_timeout(self, fake_can):
        """기대한 개수를 못 채우면 세어 둠. 원인 좁히기의 시작점임."""
        b = CanBus("can1", reader=True)
        b.connect()
        try:
            b.drain(expect=6, timeout_s=0.005)
            assert b.counters.drain_timeouts == 1
        finally:
            b.disconnect()

    def test_a_full_queue_drops_the_oldest(self, fake_can):
        """소비가 밀린 것임. 남은 옛 프레임은 이미 지난 주기의 상태임."""
        b = CanBus("can1", reader=True)
        b.connect()
        try:
            b.stop_reader()
            for i in range(b._rx.maxlen + 3):
                b._rx.append(frame(i))
            assert len(b._rx) == b._rx.maxlen
        finally:
            b.disconnect()

    def test_flush_empties_the_queue(self, fake_can):
        """직전 주기의 응답이 남아 있으면 이번 주기의 것으로 오해함."""
        b = CanBus("can1", reader=True)
        b.connect()
        try:
            FakeCanBus.instances[-1].rx.extend(FakeMessage(i, bytes(8)) for i in (7, 8))
            _wait_for(b, 2, take=False)
            assert b.flush_rx() == 2
            assert b.drain(expect=1, timeout_s=0.001) == []
        finally:
            b.disconnect()

    def test_sending_does_not_wait_for_the_reader(self, fake_can):
        """수신 스레드가 송신 락을 잡으면 스레드를 둔 이유가 사라짐."""
        b = CanBus("can1", reader=True)
        b.connect()
        try:
            assert b.send_many([frame(7), frame(8)]) == 2
        finally:
            b.disconnect()


def _wait_for(bus, count, *, take=True, timeout_s=1.0):
    """수신 스레드가 큐를 채울 때까지 기다림. 가짜 버스는 즉시 주지 않음."""
    import time as _time

    deadline = _time.monotonic() + timeout_s
    while len(bus._rx) < count and _time.monotonic() < deadline:
        _time.sleep(0.001)
    return bus.drain(expect=count, timeout_s=0.01) if take else None
