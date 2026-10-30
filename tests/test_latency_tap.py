"""`ros/huphy_ros/huphy_ros/latency_tap.py` 테스트 — ROS 없이 실행됨.

    PYTHONPATH=src:ros/huphy_ros python3 -m pytest tests/test_latency_tap.py -q

`latency_tap.py` 가 `rclpy` 를 import 하지 않으므로 ROS 가 없는 기계에서도 돎.
`convert.py`·`freshness.py` 와 같은 자리임.

확인하는 것은 **관절 색인이 `LatencyLog` 가 쓰는 이름과 같은지**와 **큐가 사건을
빠뜨리지 않는지**임. 색인이 어긋나면 기록의 지연이 다른 관절에 붙고, 사건이 빠지면
보낸 수가 줄어 유실 집계가 틀어짐.
"""

import math
import threading

import pytest

from huphy_ros.latency_tap import LATE, ONTIME, SENT, TapSet


class FakeBus:
    def __init__(self, motor_ids):
        self.motor_ids = list(motor_ids)
        self.latency = None


class FakeConfig:
    def __init__(self, names):
        self.names = names

    def joint_of(self, mid):
        return self.names.get(mid)


class FakeLeg:
    def __init__(self, leg_id, motors):
        self.id = leg_id
        self.bus = FakeBus(motors)
        self.config = FakeConfig(motors)


class FakeBiped:
    def __init__(self, parts):
        self.parts = parts
        self.id = "biped"


@pytest.fixture
def right():
    return FakeLeg("right_leg", {1: "hip_pitch", 2: "knee"})


@pytest.fixture
def biped(right):
    left = FakeLeg("left_leg", {1: "hip_pitch", 2: "knee"})
    return FakeBiped([right, left])


class TestNames:
    def test_joint_names_carry_the_limb(self, biped):
        taps = TapSet(biped)
        assert taps.joints == [
            "right_leg/hip_pitch", "right_leg/knee",
            "left_leg/hip_pitch", "left_leg/knee",
        ]

    def test_the_same_motor_id_on_two_legs_gets_two_indices(self, biped):
        """양다리는 모터 id 가 겹침 (1, 2 가 양쪽에). 버스마다 다른 색인이어야 함."""
        taps = TapSet(biped)
        r, l = biped.parts
        assert r.bus.latency.names == {1: 0, 2: 1}
        assert l.bus.latency.names == {1: 2, 2: 3}

    def test_one_leg_has_no_parts(self, right):
        taps = TapSet(right)
        assert taps.joints == ["right_leg/hip_pitch", "right_leg/knee"]
        assert right.bus.latency is not None

    def test_detach_unhooks_every_bus(self, biped):
        taps = TapSet(biped)
        taps.detach(biped)
        assert all(p.bus.latency is None for p in biped.parts)


class TestEvents:
    def test_a_send_has_no_latency(self, right):
        taps = TapSet(right)
        right.bus.latency.sent(2)
        (t, idx, ms, kind), = taps.drain()
        assert idx == 1 and kind == SENT and math.isnan(ms)
        assert t >= 0.0

    def test_a_reply_carries_milliseconds(self, right):
        taps = TapSet(right)
        right.bus.latency.received(2, 0.0026, False)
        (_, idx, ms, kind), = taps.drain()
        assert idx == 1 and kind == ONTIME
        assert abs(ms - 2.6) < 1e-9, "초를 밀리초로 바꿔 보냄"

    def test_a_late_reply_is_marked(self, right):
        taps = TapSet(right)
        right.bus.latency.received(1, 0.012, True)
        assert taps.drain()[0][3] == LATE

    def test_an_unknown_motor_id_is_ignored(self, right):
        taps = TapSet(right)
        right.bus.latency.sent(9)
        right.bus.latency.received(9, 0.001, False)
        assert taps.drain() == []

    def test_t_is_measured_from_one_origin_for_every_bus(self, biped):
        """양다리가 같은 t0 를 씀. 한 토픽으로 나가므로 기준이 같아야 함."""
        taps = TapSet(biped)
        r, l = biped.parts
        assert r.bus.latency.t0 == l.bus.latency.t0 == taps.t0


class TestQueue:
    def test_draining_keeps_the_order(self, right):
        taps = TapSet(right)
        tap = right.bus.latency
        tap.sent(1)
        tap.sent(2)
        tap.received(1, 0.001, False)
        assert [e[3] for e in taps.drain()] == [SENT, SENT, ONTIME]

    def test_draining_empties_the_queue(self, right):
        taps = TapSet(right)
        right.bus.latency.sent(1)
        assert len(taps.drain()) == 1
        assert taps.drain() == []

    def test_pending_shows_the_backlog(self, right):
        taps = TapSet(right)
        assert taps.pending == 0
        right.bus.latency.sent(1)
        right.bus.latency.sent(2)
        assert taps.pending == 2
        taps.drain()
        assert taps.pending == 0

    def test_the_oldest_goes_when_the_queue_is_full(self, right):
        """발행이 멈춰도 메모리가 안 늘어야 함. 대신 오래된 것이 버려짐."""
        taps = TapSet(right, max_events=3)
        tap = right.bus.latency
        for _ in range(5):
            tap.sent(1)
        assert taps.full
        assert len(taps.drain()) == 3

    def test_events_added_while_draining_are_not_lost(self, right):
        """제어·수신 스레드가 꺼내는 중에도 넣음. 다음 번에 나가야 함."""
        taps = TapSet(right)
        tap = right.bus.latency
        tap.sent(1)
        done = threading.Event()

        def writer():
            done.wait(1.0)
            for _ in range(50):
                tap.received(1, 0.001, False)

        th = threading.Thread(target=writer)
        th.start()
        done.set()
        th.join()
        total = len(taps.drain())
        total += len(taps.drain())
        assert total == 51
