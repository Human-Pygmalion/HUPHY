"""응답 지연 기록 테스트 — 하드웨어 없이 실행됨.

    PYTHONPATH=src python3 -m pytest tests/test_latency_log.py -q

시각을 손으로 정한 프레임을 버스에 넣어 봄. 확인하는 것은 **지연을 어느 송신에
맞추는지**(제때 / 늦음 / 유실)와 관절 이름, 파일 형식임.
"""

import types

import pytest

from huphy.motors.base import Motor
from huphy.motors.canbus import CanFrame
from huphy.motors.robstride.bus import PASSIVE, RobStrideBus
from huphy.telemetry.latency_log import LatencyLog

from test_robstride_bus import state_frame


class StubBus:
    """보내기는 기록만 하고, 수거는 미리 넣어 둔 프레임을 내줌."""

    channel = "canl"

    def __init__(self):
        self.queue = []

    def send_many(self, frames):
        return len(frames)

    def drain(self, *, expect=None, timeout_s=0.0):
        out, self.queue = self.queue, []
        return out


def frame(mid, stamp):
    msg = state_frame(mid, 0.0)
    return CanFrame(can_id=mid, data=bytes(msg.data), stamp=stamp)


@pytest.fixture
def rig(monkeypatch):
    """다리 하나 (모터 1, 2). 송신 시각을 고정할 수 있게 시계를 갈아 끼움."""
    clock = {"now": 0.0}
    monkeypatch.setattr(
        "huphy.motors.robstride.bus.time.monotonic", lambda: clock["now"]
    )
    stub = StubBus()
    bus = RobStrideBus(stub, {1: Motor(id=1, model="RS02"), 2: Motor(id=2, model="RS02")})
    names = {1: "knee", 2: "hip_pitch"}
    part = types.SimpleNamespace(
        id="left_leg", bus=bus,
        config=types.SimpleNamespace(joint_of=lambda mid: names.get(mid)),
    )
    log = LatencyLog(drain_ms=4, hz=100).attach(part)
    return clock, stub, bus, log


def send(bus, clock, t):
    clock["now"] = t
    bus.send_mit({1: PASSIVE, 2: PASSIVE})


class TestMatching:
    def test_names_are_leg_and_joint(self, rig):
        _, _, _, log = rig
        assert log.joints == ["left_leg/knee", "left_leg/hip_pitch"]

    def test_an_on_time_reply_is_measured_from_its_send(self, rig):
        clock, stub, bus, log = rig
        send(bus, clock, 10.000)
        stub.queue = [frame(1, 10.0025)]
        bus.collect()
        [(_, ms, late)] = log.samples["left_leg/knee"]
        assert ms == pytest.approx(2.5)
        assert late is False

    def test_a_reply_older_than_the_send_goes_to_the_previous_command(self, rig):
        """다음 송신 뒤에 수거된 응답. 도착은 그 송신보다 앞임."""
        clock, stub, bus, log = rig
        send(bus, clock, 10.000)
        bus.collect()                               # 아무것도 안 옴
        send(bus, clock, 10.010)
        stub.queue = [frame(1, 10.006)]             # 첫 명령 뒤 6ms 에 도착했던 것
        bus.collect()
        [(_, ms, late)] = log.samples["left_leg/knee"]
        assert ms == pytest.approx(6.0)
        assert late is True

    def test_what_never_came_is_lost(self, rig):
        clock, stub, bus, log = rig
        for k in range(3):
            send(bus, clock, 10.0 + 0.01 * k)
            stub.queue = [frame(1, 10.0 + 0.01 * k + 0.002)]
            bus.collect()
        assert log.counts("left_leg/knee") == (3, 0, 0)
        assert log.counts("left_leg/hip_pitch") == (0, 0, 3)

    def test_nothing_is_timed_without_a_recorder(self, rig):
        clock, stub, bus, log = rig
        log.detach(types.SimpleNamespace(bus=bus))
        send(bus, clock, 10.0)
        assert bus._sent_at == {}


class TestOutput:
    def test_the_file_has_meta_and_all_three_kinds(self, rig, tmp_path):
        clock, stub, bus, log = rig
        send(bus, clock, 10.0)
        stub.queue = [frame(1, 10.001)]
        bus.collect()
        send(bus, clock, 10.01)
        stub.queue = [frame(2, 10.005)]             # 늦게 온 hip_pitch
        bus.collect()

        text = log.save(tmp_path / "lat.csv").read_text().splitlines()
        assert text[0] == "# drain_ms=4 hz=100"
        assert text[1] == "t,joint,latency_ms,status"
        statuses = [line.split(",")[3] for line in text[2:]]
        assert set(statuses) == {"ontime", "late", "lost"}

    def test_the_summary_has_a_row_per_joint_and_a_total(self, rig):
        clock, stub, bus, log = rig
        send(bus, clock, 10.0)
        stub.queue = [frame(1, 10.002), frame(2, 10.003)]
        bus.collect()
        out = log.summary()
        assert "left_leg/knee" in out and "left_leg/hip_pitch" in out and "전체" in out
