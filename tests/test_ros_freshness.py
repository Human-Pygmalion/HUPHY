"""받은 값의 나이 판정 — **`rclpy` 없이 돎.**

    PYTHONPATH=src python3 -m pytest tests/test_ros_freshness.py -q

여기 있는 판단이 틀리면 로봇이 잘못 움직임.

    너무 느슨   끊긴 줄 모르고 마지막 목표를 계속 붙잡음
    너무 빡빡   네트워크가 조금만 흔들려도 토크가 끊김
"""

import pytest

pytest.importorskip("huphy_ros", reason="ros/huphy_ros 가 경로에 없음")

from huphy_ros.freshness import (                       # noqa: E402
    CONTROL_TARGET_STALE_S,
    POLICY_STATE_STALE_S,
    Freshness,
    Latest,
)


class Clock:
    """테스트용 시계. `time.monotonic` 자리에 들어감."""

    def __init__(self, t=0.0):
        self.t = t

    def __call__(self):
        return self.t


# ===========================================================================
# 임계값
# ===========================================================================
class TestThresholds:
    def test_control_waits_longer_than_policy(self):
        """순서가 뒤바뀌면 안 됨 -- 정책이 먼저 멈추고 그 뒤에 제어가 멈춤.

        반대로 두면 네트워크가 끊겼을 때 제어가 먼저 토크를 끊고, 정책은 그것도
        모른 채 목표각을 계속 냄.
        """
        assert CONTROL_TARGET_STALE_S > POLICY_STATE_STALE_S

    def test_a_non_positive_threshold_is_refused(self):
        for bad in (0.0, -0.1):
            with pytest.raises(ValueError):
                Freshness(bad)


# ===========================================================================
# Freshness
# ===========================================================================
class TestFreshness:
    def test_nothing_received_yet(self):
        value, fresh = Freshness(0.1, clock=Clock()).take()
        assert value is None and fresh is False

    def test_a_fresh_value_comes_back(self):
        f = Freshness(0.1, clock=Clock())
        f.put({"knee": 30.0})
        assert f.take() == ({"knee": 30.0}, True)

    def test_right_at_the_threshold_is_still_fresh(self):
        clock = Clock()
        f = Freshness(0.1, clock=clock)
        f.put("x")
        clock.t = 0.1
        assert f.take()[1] is True

    def test_past_the_threshold_is_stale(self):
        clock = Clock()
        f = Freshness(0.1, clock=clock)
        f.put("x")
        clock.t = 0.101
        value, fresh = f.take()
        assert fresh is False
        assert value == "x", "낡아도 값은 같이 냄 -- 로그에 찍을 수 있게"
        assert f.stale_count == 1

    def test_a_new_value_revives_it(self):
        clock = Clock()
        f = Freshness(0.1, clock=clock)
        f.put("old")
        clock.t = 0.5
        assert f.take()[1] is False
        f.put("new")
        assert f.take() == ("new", True)

    def test_clock_zero_is_a_real_time(self):
        """'아직 없음' 을 0.0 으로 표시하면 안 되는 이유.

        clock() 이 정말로 0.0 을 낼 수 있음 -- monotonic 은 기준점이 정해져 있지
        않고, 테스트 시계는 0 에서 시작함.
        """
        f = Freshness(0.1, clock=Clock(0.0))
        f.put("x")
        assert f.take() == ("x", True)
        assert f.age() == 0.0

    def test_clear_forgets(self):
        """켤 때 직전 값이 한 주기 나가지 않게."""
        f = Freshness(0.1, clock=Clock())
        f.put("x")
        f.clear()
        assert f.take() == (None, False)
        assert f.age() == -1.0

    def test_age_before_anything(self):
        assert Freshness(0.1, clock=Clock()).age() == -1.0

    def test_age_grows(self):
        clock = Clock()
        f = Freshness(0.1, clock=clock)
        f.put("x")
        clock.t = 0.3
        assert f.age() == pytest.approx(0.3)

    def test_the_value_is_not_copied_but_the_caller_sees_what_was_put(self):
        """사전을 통째로 바꾸므로 절반만 갱신된 값이 안 나감."""
        f = Freshness(0.1, clock=Clock())
        first = {"a": 1}
        f.put(first)
        f.put({"a": 2})
        assert f.take()[0] == {"a": 2}


# ===========================================================================
# Latest
# ===========================================================================
class TestLatest:
    def test_nothing_yet(self):
        assert Latest().take() is None

    def test_keeps_the_last(self):
        latest = Latest()
        latest.put("a")
        latest.put("b")
        assert latest.take() == "b"

    def test_does_not_go_stale(self):
        """반대 방향에 씀 -- 발행이 한 번 늦는 것은 문제가 아님."""
        latest = Latest()
        latest.put({"knee.pos": 1.0})
        assert latest.take() == {"knee.pos": 1.0}


# ===========================================================================
# 끊김 순서
# ===========================================================================
class TestOutage:
    def test_policy_stops_before_control_does(self):
        """네트워크가 끊겼을 때 두 노드가 걸리는 순서.

        t=0     마지막 메시지
        t=60ms  정책이 상태를 낡았다고 봄 -> 목표각을 안 냄
        t=100ms 제어가 목표각을 낡았다고 봄 -> 감쇠
        """
        clock = Clock()
        state = Freshness(POLICY_STATE_STALE_S, clock=clock)
        target = Freshness(CONTROL_TARGET_STALE_S, clock=clock)
        state.put("상태")
        target.put("목표각")

        clock.t = 0.05
        assert state.take()[1] is True
        assert target.take()[1] is True

        clock.t = 0.08
        assert state.take()[1] is False, "정책이 먼저 멈춰야 함"
        assert target.take()[1] is True, "제어는 아직 돌아야 함"

        clock.t = 0.12
        assert target.take()[1] is False, "그 뒤에 제어가 멈춤"
