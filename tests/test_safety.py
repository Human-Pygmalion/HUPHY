"""safety/ 순수 함수 테스트 — 하드웨어 없이 실행됨.

원본에서는 이 로직이 컨트롤러의 락과 CAN 버스에 얽혀 있어 검증 불가였음.
한계 여유, 클리핑 순서, NaN 처리는 미묘한데 실물에서 디버깅하면 비쌈.

    PYTHONPATH=src python3 -m pytest tests -q      # python-can 불필요
"""

import math

import pytest

from huphy.safety import guards, limits
from huphy.safety import LinkWatch

# 실제 무릎(m10) 한계. 비대칭이라 좌우 구분이 필요한 검사에 적합함.
KNEE = (-20.65, 74.79)

NAN = float("nan")
INF = float("inf")


# ===========================================================================
# limits — 한계와의 관계 계산
# ===========================================================================
class TestSafeWindow:
    def test_shrinks_both_sides(self):
        """여유는 양쪽에서 빠짐."""
        lo, hi = limits.safe_window(KNEE, 3)
        assert lo == pytest.approx(-17.65)
        assert hi == pytest.approx(71.79)

    def test_zero_margin_is_identity(self):
        assert limits.safe_window(KNEE, 0) == pytest.approx(KNEE)

    def test_negative_margin_treated_as_magnitude(self):
        """부호를 실수로 반대로 줘도 안쪽으로 좁혀야 함."""
        assert limits.safe_window(KNEE, -3) == pytest.approx(limits.safe_window(KNEE, 3))


class TestClamp:
    def test_inside_passes_unchanged(self):
        assert limits.clamp(50, KNEE, margin_deg=3) == (50.0, False)

    def test_above_is_clipped_to_upper(self):
        value, clipped = limits.clamp(90, KNEE, margin_deg=3)
        assert value == pytest.approx(71.79)
        assert clipped is True

    def test_below_is_clipped_to_lower(self):
        value, clipped = limits.clamp(-50, KNEE, margin_deg=3)
        assert value == pytest.approx(-17.65)
        assert clipped is True

    def test_none_limits_passes(self):
        """한계가 없는 모터(캘리브레이션 미완)는 통과시킴."""
        assert limits.clamp(9999, None, margin_deg=3) == (9999.0, False)

    def test_reports_clipping(self):
        """잘림 여부를 반환하는 것이 핵심 — 클리핑은 조용한 변조임."""
        assert limits.clamp(71.79, KNEE, margin_deg=3)[1] is False
        assert limits.clamp(71.80, KNEE, margin_deg=3)[1] is True


class TestMarginToLimit:
    def test_uses_nearer_side(self):
        """비대칭 한계에서 가까운 쪽을 봄. 50은 위쪽(74.79)이 더 가까움."""
        assert limits.margin_to_limit(50, KNEE) == pytest.approx(24.79)

    def test_lower_side(self):
        """0은 아래쪽(-20.65)이 더 가까움."""
        assert limits.margin_to_limit(0, KNEE) == pytest.approx(20.65)

    def test_negative_when_above(self):
        assert limits.margin_to_limit(80, KNEE) == pytest.approx(-5.21)

    def test_negative_when_below(self):
        assert limits.margin_to_limit(-30, KNEE) == pytest.approx(-9.35)

    def test_zero_at_limit(self):
        assert limits.margin_to_limit(74.79, KNEE) == pytest.approx(0.0)


class TestClosestToLimit:
    def test_returns_motor_id_not_bool(self):
        """id를 함께 반환해야 원인 관절을 추적할 수 있음."""
        values = {7: 0.0, 8: 40.0, 10: 70.0}
        lims = {7: (-117.07, 21.07), 8: (-5.51, 79.64), 10: KNEE}
        motor_id, margin = limits.closest_to_limit(values, lims)
        assert motor_id == 10
        assert margin == pytest.approx(4.79)

    def test_skips_motors_without_limits(self):
        values = {7: 0.0, 10: 70.0}
        lims = {7: None, 10: KNEE}
        assert limits.closest_to_limit(values, lims)[0] == 10

    def test_none_when_no_candidates(self):
        motor_id, margin = limits.closest_to_limit({7: 0.0}, {7: None})
        assert motor_id is None
        assert margin == INF


# ===========================================================================
# guards — 명령의 최종 관문
# ===========================================================================
class TestIsFinite:
    def test_rejects_nan_and_inf(self):
        assert guards.is_finite(NAN) is False
        assert guards.is_finite(INF) is False
        assert guards.is_finite(-INF) is False

    def test_accepts_normal(self):
        assert guards.is_finite(0.0, -100.0, 1e6) is True

    def test_skips_none(self):
        """None은 '상태 없음'이라는 별개 사유이므로 여기서 걸러내지 않음."""
        assert guards.is_finite(None, 5.0) is True

    def test_any_bad_fails(self):
        assert guards.is_finite(1.0, NAN, 3.0) is False


class TestNanIsDangerous:
    """유한값 검사가 왜 최우선이어야 하는지 고정함.

    파이썬의 min/max는 NaN을 비교할 수 없어 그대로 통과시킴. 따라서 인코딩
    단계의 클램프가 무력화되고, NaN 하나가 최대값 명령이 됨.
    """

    def test_python_minmax_passes_nan_through(self):
        assert min(10, NAN) == 10
        assert max(0, min(10, NAN)) == 10

    def test_clamp_does_not_catch_nan(self):
        """limits.clamp도 NaN을 못 잡음 — 비교가 전부 False라 원본이 반환됨."""
        value, clipped = limits.clamp(NAN, KNEE, margin_deg=3)
        assert math.isnan(value)
        assert clipped is False

    def test_guards_catches_it(self):
        r = guards.apply(NAN, 0.0, limits=KNEE, command_margin_deg=3, max_delta_deg=50)
        assert r.sendable is False
        assert r.reject is guards.RejectReason.NOT_FINITE


class TestClampJump:
    def test_within_limit_unchanged(self):
        assert guards.clamp_jump(30, 0, 50) == (30.0, False)

    def test_positive_is_capped(self):
        assert guards.clamp_jump(100, 0, 50) == (50.0, True)

    def test_negative_is_capped(self):
        assert guards.clamp_jump(-100, 0, 50) == (-50.0, True)

    def test_relative_to_current_not_zero(self):
        """현재 위치 기준임. 20에서 100이면 80 차이라 70까지만."""
        assert guards.clamp_jump(100, 20, 50) == (70.0, True)

    def test_reaches_far_target_over_cycles(self):
        """버리지 않고 자르므로 먼 목표에도 결국 도달함.

        버리는 방식이면 영영 도달하지 못함. 즉 클리핑 = 속도 제한임.
        """
        cur = 0.0
        for _ in range(3):
            cur, _ = guards.clamp_jump(100, cur, 50)
        assert cur == pytest.approx(100.0)


class TestApply:
    def kw(self, **over):
        base = dict(limits=KNEE, command_margin_deg=3.0, max_delta_deg=50.0)
        base.update(over)
        return base

    def test_normal_passes(self):
        r = guards.apply(50, 45, **self.kw())
        assert r.sendable
        assert r.value == pytest.approx(50.0)
        assert r.clips == ()

    def test_no_state_is_rejected(self):
        r = guards.apply(50, None, **self.kw())
        assert r.sendable is False
        assert r.reject is guards.RejectReason.NO_STATE

    def test_nan_current_is_rejected(self):
        """현재값이 NaN이면 점프 계산이 NaN을 뱉으므로 함께 검사함."""
        r = guards.apply(50, NAN, **self.kw())
        assert r.reject is guards.RejectReason.NOT_FINITE

    def test_limit_clip(self):
        r = guards.apply(90, 70, **self.kw())
        assert r.value == pytest.approx(71.79)
        assert r.clips == (guards.ClipReason.LIMIT,)

    def test_jump_clip(self):
        r = guards.apply(60, 0, **self.kw(max_delta_deg=10))
        assert r.value == pytest.approx(10.0)
        assert r.clips == (guards.ClipReason.JUMP,)

    def test_both_can_clip(self):
        """한계와 점프가 동시에 걸릴 수 있으므로 clips가 튜플임."""
        r = guards.apply(100, 0, **self.kw())
        assert r.value == pytest.approx(50.0)
        assert r.clips == (guards.ClipReason.LIMIT, guards.ClipReason.JUMP)

    def test_limit_applied_before_jump(self):
        """순서 고정 — 안전한 목표를 정하고 거기로 가는 속도를 제한함.

        목표 200, 현재 60, max_delta 50 인 경우 순서에 따라 결과가 달라짐:

          한계 먼저 (현재 구현)
            clamp(200) -> 71.79
            clamp_jump(71.79, 60, 50) -> 차이 11.79 < 50 이므로 그대로
            결과 71.79, clips = (LIMIT,)          <- 점프는 걸리지 않음

          점프 먼저 (잘못된 순서)
            clamp_jump(200, 60, 50) -> 110        <- 한계(74.79)를 넘은 값
            clamp(110) -> 71.79
            결과 71.79, clips = (JUMP, LIMIT)     <- 불필요한 점프 클리핑이 기록됨

        값은 같지만 경로가 다름. 역순은 중간에 한계 밖 값을 만들고, 실제로는
        걸릴 필요가 없는 점프 클리핑을 카운터에 남김.
        """
        r = guards.apply(200, 60, **self.kw())
        assert r.value == pytest.approx(71.79)
        assert r.clips == (guards.ClipReason.LIMIT,)

    def test_output_may_be_outside_limits_while_recovering(self):
        """현재가 이미 한계 밖이면 한 번에 복귀하지 않음.

        의도된 동작임 — 한 번에 뛰면 위험하므로 max_delta씩 돌아옴.
        """
        r = guards.apply(0, 200, **self.kw())
        assert r.value == pytest.approx(150.0)
        assert r.value > KNEE[1]          # 아직 한계 밖
        assert guards.ClipReason.JUMP in r.clips

    def test_enforce_limits_off(self):
        r = guards.apply(90, 70, **self.kw(enforce_limits=False))
        assert r.value == pytest.approx(90.0)
        assert r.clips == ()

    def test_none_limits_only_jump_applies(self):
        r = guards.apply(1000, 0, **self.kw(limits=None))
        assert r.value == pytest.approx(50.0)
        assert r.clips == (guards.ClipReason.JUMP,)


class TestGuardCounters:
    def make(self, target, current, **over):
        kw = dict(limits=KNEE, command_margin_deg=3.0, max_delta_deg=50.0)
        kw.update(over)
        return guards.apply(target, current, **kw)

    def test_counts_by_kind(self):
        c = guards.GuardCounters()
        c.record(self.make(90, 70))       # limit
        c.record(self.make(100, 0))       # limit + jump
        c.record(self.make(NAN, 0))       # reject
        assert c.total_clips == 3
        assert c.total_rejects == 1

    def test_separates_clip_and_reject(self):
        """'잘림'과 '미전송'은 다른 사건이므로 분리해 집계함."""
        c = guards.GuardCounters()
        c.record(self.make(100, 0))
        c.record(self.make(NAN, 0))
        f = c.as_fields()
        assert f["clips_limit"] == 1
        assert f["clips_jump"] == 1
        assert f["rejects_nan"] == 1
        assert f["rejects_nostate"] == 0

    def test_all_keys_always_present(self):
        """0이어도 모든 키를 출력함.

        필드가 나타났다 사라지면 PlotJuggler 레이아웃과 CSV 헤더가 깨짐.
        """
        expected = {
            "clips", "rejects",
            # clips_rate 는 2026-09-07 추가된 속도 제한(clamp_rate)의 것. 폭주 상한
            # (clips_jump) 과 반드시 따로 세어야 한다 - 둘은 기준점이 다르고, 어느
            # 쪽이 자르고 있는지가 곧 "느린 이유" 의 답이기 때문이다.
            "clips_limit", "clips_jump", "clips_rate",
            "rejects_nan", "rejects_nostate",
        }
        assert set(guards.GuardCounters().as_fields()) == expected

    def test_reset(self):
        c = guards.GuardCounters()
        c.record(self.make(100, 0))
        c.reset()
        assert c.total_clips == 0
        assert c.total_rejects == 0


# ===========================================================================
# 통신 두절 판정
# ===========================================================================
def status(**miss):
    """`link_status` 모양의 사전. 값은 연속 무응답 주기 수."""
    return {name: {"age": 1.0, "ack": 0.0, "miss": float(n)} for name, n in miss.items()}


class TestLinkWatch:
    def test_quiet_link_is_fine(self):
        assert LinkWatch(5).check(status(knee=0)) is None

    def test_one_dropped_cycle_is_normal(self):
        """한 주기 빠지는 것은 흔함. 다음 주기에 옴."""
        assert LinkWatch(5).check(status(knee=1)) is None

    def test_sustained_silence_trips(self):
        loss = LinkWatch(5).check(status(knee=5))
        assert loss is not None
        assert loss.motors == ("knee",)

    def test_one_motor_is_enough(self):
        """무릎 하나가 굳어도 그 다리는 의도한 자세를 못 만듦."""
        loss = LinkWatch(3).check(status(hip_pitch=0, knee=3))
        assert loss.motors == ("knee",)

    def test_it_reports_the_worst(self):
        loss = LinkWatch(3).check(status(knee=4, hip_roll=9))
        assert loss.cycles == 9
        assert loss.motors == ("hip_roll", "knee")

    def test_zero_turns_it_off(self):
        """커미셔닝처럼 응답이 원래 드문 상황용."""
        assert not LinkWatch(0).enabled
        assert LinkWatch(0).check(status(knee=100)) is None

    def test_composite_names_carry_the_limb(self):
        """어느 다리가 죽었는지 알아야 사람이 손을 씀."""
        loss = LinkWatch(3).check(status(**{"left_leg/knee": 5}))
        assert loss.motors == ("left_leg/knee",)

    def test_missing_field_counts_as_answering(self):
        """`miss` 를 안 내는 로봇을 끊긴 것으로 보지 않음."""
        assert LinkWatch(1).check({"knee": {"age": 1.0}}) is None

    def test_message_says_what_and_how_long(self):
        assert "3주기째" in str(LinkWatch(3).check(status(knee=3)))
