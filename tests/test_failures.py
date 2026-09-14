"""실행 끝 실패 집계 테스트 — 하드웨어 없이 실행됨.

새로 재는 것이 없고 이미 세던 값을 모아 읽기만 함. 여기서 확인하는 것은 **무엇을
어디서 읽어 어떻게 보여 주나** 임.
"""

from types import SimpleNamespace

from huphy.control.loop import LoopStats
from huphy.motors.canbus import CanCounters
from huphy.safety.guards import GuardCounters
from huphy.scripts import failures


def fake_leg(name="right_leg", channel="can1", *, missed=None, drain_s=0.002):
    counters = CanCounters()
    guard = GuardCounters()
    counts = {m: {"asked": 100, "missed": 0} for m in ("knee", "ankle_a")}
    for motor, value in (missed or {}).items():
        counts[motor]["missed"] = value
    return SimpleNamespace(
        id=name,
        config=SimpleNamespace(channel=channel),
        bus=SimpleNamespace(bus=SimpleNamespace(counters=counters), drain_s=drain_s),
        counters=guard,
        link_counts=lambda: counts,
    )


def stats(**over):
    values = dict(cycles=100, total_s=1.0, target_hz=100.0, overruns=0,
                  worst_dt_ms=10.0, missing_cycles=0)
    values.update(over)
    s = LoopStats(target_hz=values.pop("target_hz"))
    for key, value in values.items():
        setattr(s, key, value)
    return s


class TestReport:
    def test_a_clean_run(self):
        text = failures.report(fake_leg(), stats())
        assert "무응답     0주기 (0.0%)" in text
        assert "모터별 무응답  없음" in text
        assert "가드 자름   없음" in text

    def test_missing_cycles_as_a_share(self):
        text = failures.report(fake_leg(), stats(missing_cycles=5))
        assert "5주기 (5.0%)" in text

    def test_can_counters_per_channel(self):
        leg = fake_leg()
        leg.bus.bus.counters.frames_sent = 100
        leg.bus.bus.counters.frames_received = 98
        leg.bus.bus.counters.drain_timeouts = 7
        leg.bus.bus.counters.tx_errors = 2
        line = [l for l in failures.report(leg, stats()).splitlines() if "can1" in l][0]
        #                보냄  받음  미수신 송신  수신  대기초과 버림
        assert line.split()[1:] == ["100", "98", "2", "2", "0", "7", "0"]

    def test_missing_frames_are_sent_minus_received(self):
        """MIT 는 명령을 받으면 반드시 응답함. 차이가 곧 안 돌아온 수임."""
        leg = fake_leg()
        leg.bus.bus.counters.frames_sent = 1000
        leg.bus.bus.counters.frames_received = 1000
        assert "유실은 없고 늦게 온 것임" in failures.report(leg, stats())

    def test_a_short_run_does_not_cry_wolf(self):
        """정지 절차에서 수거 없이 나가는 몫이 있어 짧은 실행은 비율이 튐."""
        leg = fake_leg()
        leg.bus.bus.counters.frames_sent = 100
        leg.bus.bus.counters.frames_received = 88
        assert "유실은 없고" in failures.report(leg, stats())

    def test_a_big_gap_is_called_out(self):
        leg = fake_leg()
        leg.bus.bus.counters.frames_sent = 1000
        leg.bus.bus.counters.frames_received = 600
        assert "응답이 실제로 안 돌아옴" in failures.report(leg, stats())

    def test_a_few_left_in_the_queue_is_still_fine(self):
        """끝날 때 큐에 남은 것은 안 세므로 모터 수 정도는 정상임."""
        leg = fake_leg()
        leg.bus.bus.counters.frames_sent = 1000
        leg.bus.bus.counters.frames_received = 994
        assert "유실은 없고" in failures.report(leg, stats())

    def test_a_quarter_lost_is_called_out(self):
        """실물에서 본 비율. 이건 늦은 것이 아니라 유실임."""
        leg = fake_leg()
        leg.bus.bus.counters.frames_sent = 191000
        leg.bus.bus.counters.frames_received = 145000
        assert "실제로 안 돌아옴" in failures.report(leg, stats())

    def test_only_motors_that_missed_are_listed(self):
        text = failures.report(fake_leg(missed={"knee": 3}), stats())
        assert "knee" in text
        assert "ankle_a" not in text

    def test_worst_motor_comes_first(self):
        text = failures.report(fake_leg(missed={"knee": 1, "ankle_a": 9}), stats())
        assert text.index("ankle_a") < text.index("knee")

    def test_guard_counts_are_summed(self):
        leg = fake_leg()
        leg.counters.clips["limit"] = 4
        leg.counters.rejects["nostate"] = 2
        text = failures.report(leg, stats())
        assert "limit 4" in text
        assert "nostate 2" in text

    def test_drain_comes_from_the_bus(self):
        text = failures.report(fake_leg(drain_s=0.003), stats())
        assert "3ms" in text


class TestComposite:
    def robot(self, left_missed=None):
        right = fake_leg("right_leg", "can1")
        left = fake_leg("left_leg", "can0", missed=left_missed)
        return SimpleNamespace(id="huphy", parts=(right, left))

    def test_both_channels(self):
        text = failures.report(self.robot(), stats())
        assert "can1" in text and "can0" in text

    def test_motor_names_carry_the_leg(self):
        """어느 다리의 무릎인지 알아야 손을 씀."""
        text = failures.report(self.robot(left_missed={"knee": 2}), stats())
        assert "left_leg/knee" in text

    def test_drain_is_counted_per_leg(self):
        """다리마다 따로 기다려서 최악 두 배임."""
        text = failures.report(self.robot(), stats())
        assert "x 2다리 = 최악 4ms" in text


class TestDrainLine:
    def test_share_of_the_period(self):
        assert "주기 10.0ms 의 20%" in failures.drain_line(0.002, 100.0)

    def test_warns_when_it_eats_half_the_period(self):
        assert "크게 잡음" in failures.drain_line(0.006, 100.0)

    def test_two_legs_double_it(self):
        assert "크게 잡음" in failures.drain_line(0.003, 100.0, legs=2)

    def test_quiet_when_small(self):
        assert "크게 잡음" not in failures.drain_line(0.002, 100.0)
