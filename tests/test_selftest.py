"""동작 확인 진입점 테스트 — 하드웨어 없이 실행됨.

    PYTHONPATH=src python3 -m pytest tests -q

동작 함수는 순수 함수임. 시간과 관찰을 받아 관절 목표를 내므로 그냥 부르면 됨.
루프도 CAN 도 필요 없음.
"""

import math

import pytest

from huphy.config import LimbConfig
from huphy.motors.base import Gains, Motor
from huphy.scripts import selftest

LIMITS = {"knee": (-20.0, 70.0), "hip_pitch": (-110.0, -20.0)}


class FakeKinematics:
    class envelope:
        pitch_deg = (-40.0, 40.0)
        roll_deg = (-25.0, 25.0)


class FakeLeg:
    """`joint_limits` 가 보는 것만 흉내 냄 — 설정과 기구학."""

    id = "right_leg"

    def __init__(self, limits=None):
        limits = LIMITS if limits is None else limits
        motors = {
            name: Motor(
                id=index,
                model="RS02",
                limits_deg=limits.get(name),
                gains=Gains(kp=1.0, kd=0.1),
            )
            for index, name in enumerate(
                ("hip_pitch", "hip_roll", "hip_yaw", "knee", "ankle_a", "ankle_b"), start=7
            )
        }
        self.config = LimbConfig(name="right_leg", kind="leg", motors=motors)
        self.kinematics = FakeKinematics()


# ===========================================================================
# 관절 한계
# ===========================================================================
class TestJointLimits:
    def test_single_joints_come_from_the_config(self):
        """모터와 관절이 1:1 이라 robot.yaml 의 limits_deg 를 그대로 씀."""
        limits = selftest.joint_limits(FakeLeg())
        assert limits["knee"] == (-20.0, 70.0)
        assert limits["hip_pitch"] == (-110.0, -20.0)

    def test_unmeasured_joints_are_left_out(self):
        """어디까지 가도 되는지 모르는 관절을 흔들 수는 없음."""
        limits = selftest.joint_limits(FakeLeg())
        assert "hip_roll" not in limits and "hip_yaw" not in limits

    def test_ankle_comes_from_the_envelope(self):
        """모터 두 개가 물려 있어 모터 한계를 관절 한계로 옮길 수 없음."""
        limits = selftest.joint_limits(FakeLeg())
        assert limits["ankle_pitch"] == (-40.0, 40.0)
        assert limits["ankle_roll"] == (-25.0, 25.0)

    def test_motor_names_never_appear(self):
        """관절 공간으로 명령함. ankle_a 은 관절이 아님."""
        limits = selftest.joint_limits(FakeLeg())
        assert "ankle_a" not in limits and "ankle_b" not in limits


class TestInset:
    def test_pulls_in_from_both_ends(self):
        """한계는 하드스톱을 잰 값임. 그대로 명령하면 스톱에 부딪힘."""
        assert selftest._inset((-20.0, 70.0), 5.0) == (-15.0, 65.0)

    def test_narrow_span_collapses_to_the_middle(self):
        """여유가 폭보다 크면 최소가 최대보다 커짐. 그때는 안 움직임."""
        assert selftest._inset((0.0, 6.0), 5.0) == (3.0, 3.0)

    def test_zero_margin_keeps_the_limits(self):
        assert selftest._inset((-20.0, 70.0), 0.0) == (-20.0, 70.0)


# ===========================================================================
# 동작
# ===========================================================================
class TestApproach:
    def test_starts_where_the_joint_is(self):
        """토크를 넣는 순간 목표가 멀리 있으면 관절이 튐."""
        motion = selftest.approach({"knee": 0.0}, {"knee": 30.0}, 3.0)
        assert motion(0.0, {}) == {"knee": 30.0}

    def test_reaches_the_target(self):
        motion = selftest.approach({"knee": 0.0}, {"knee": 30.0}, 3.0)
        assert motion(3.0, {}) == {"knee": 0.0}

    def test_holds_after_arriving(self):
        """도착한 뒤에도 계속 불림. 넘어가면 목표를 지나침."""
        motion = selftest.approach({"knee": 0.0}, {"knee": 30.0}, 3.0)
        assert motion(99.0, {}) == {"knee": 0.0}

    def test_moves_at_a_constant_rate(self):
        motion = selftest.approach({"knee": 0.0}, {"knee": 30.0}, 3.0)
        assert motion(1.5, {})["knee"] == pytest.approx(15.0)

    def test_unknown_start_holds_the_target(self):
        """상태를 못 읽은 관절이 있어도 다른 관절이 멈추면 안 됨."""
        motion = selftest.approach({"knee": 5.0}, {}, 3.0)
        assert motion(0.0, {}) == {"knee": 5.0}

    def test_zero_seconds_is_refused(self):
        with pytest.raises(ValueError, match="0보다 커야 함"):
            selftest.approach({"knee": 0.0}, {"knee": 0.0}, 0.0)


class TestCycle:
    def test_starts_in_the_middle(self):
        """접근이 데려다 놓은 자리에서 이어져야 함."""
        motion = selftest.cycle({"knee": (-20.0, 70.0)}, period_s=4.0)
        assert motion(0.0, {})["knee"] == pytest.approx(25.0)

    def test_reaches_both_ends(self):
        motion = selftest.cycle({"knee": (-20.0, 70.0)}, period_s=4.0)
        assert motion(1.0, {})["knee"] == pytest.approx(70.0)
        assert motion(3.0, {})["knee"] == pytest.approx(-20.0)

    def test_never_leaves_the_limits(self):
        """사인파라 진폭이 반폭임. 한계를 넘을 수 없음."""
        motion = selftest.cycle({"knee": (-20.0, 70.0)}, period_s=4.0)
        values = [motion(t / 100.0, {})["knee"] for t in range(800)]
        assert min(values) >= -20.0 - 1e-9
        assert max(values) <= 70.0 + 1e-9

    def test_every_joint_moves(self):
        motion = selftest.cycle(
            {"knee": (-20.0, 70.0), "ankle_roll": (-25.0, 25.0)}, period_s=4.0
        )
        assert set(motion(1.0, {})) == {"knee", "ankle_roll"}

    def test_period_sets_the_speed(self):
        """주기가 길수록 천천히 감. 점프 가드에 걸리지 않게 조절하는 손잡이임."""
        slow = selftest.cycle({"knee": (-20.0, 70.0)}, period_s=8.0)
        assert slow(2.0, {})["knee"] == pytest.approx(70.0)


class TestThen:
    def test_first_runs_until_the_handover(self):
        motion = selftest.then(lambda t, o: {"a": t}, 2.0, lambda t, o: {"b": t})
        assert motion(1.0, {}) == {"a": 1.0}

    def test_second_starts_from_its_own_zero(self):
        """사인파가 가운데에서 시작해야 접근한 자리와 이어짐."""
        motion = selftest.then(lambda t, o: {"a": t}, 2.0, lambda t, o: {"b": t})
        assert motion(2.0, {}) == {"b": 0.0}
        assert motion(5.0, {}) == {"b": 3.0}

    def test_second_never_ends(self):
        """Ctrl-Q 까지 계속함. chain 과 달리 뒤 구간에 길이가 없음."""
        motion = selftest.then(lambda t, o: {"a": t}, 2.0, lambda t, o: {"b": t})
        assert motion(1e6, {}) is not None


class TestMidpoints:
    def test_center_of_each_span(self):
        assert selftest.midpoints({"knee": (-20.0, 70.0)}) == {"knee": 25.0}


# ===========================================================================
# 명령줄
# ===========================================================================
class TestParser:
    def test_common_options_work_before_the_subcommand(self):
        args = selftest.build_parser().parse_args(
            ["--limb", "right_leg", "--approach", "1", "zero"]
        )
        assert args.limb == "right_leg" and args.approach == 1.0

    def test_common_options_work_after_the_subcommand(self):
        """zero --approach 5 라고 쓰는 것이 자연스러움."""
        args = selftest.build_parser().parse_args(
            ["zero", "--limb", "right_leg", "--approach", "1"]
        )
        assert args.limb == "right_leg" and args.approach == 1.0

    def test_a_value_given_early_is_not_overwritten(self):
        """서브명령의 기본값이 앞에 적은 값을 덮으면 안 됨."""
        args = selftest.build_parser().parse_args(["--limb", "right_leg", "zero"])
        assert args.limb == "right_leg"

    def test_range_has_its_own_options(self):
        args = selftest.build_parser().parse_args(["range", "--period", "9"])
        assert args.period == 9.0 and args.margin == selftest.DEFAULT_MARGIN_DEG

    def test_a_command_is_required(self):
        with pytest.raises(SystemExit):
            selftest.build_parser().parse_args(["--limb", "right_leg"])

    def test_robot_works_before_the_subcommand(self):
        args = selftest.build_parser().parse_args(["--robot", "zero"])
        assert args.robot is True

    def test_robot_works_after_the_subcommand(self):
        args = selftest.build_parser().parse_args(["zero", "--robot"])
        assert args.robot is True

    def test_robot_is_not_set_by_default(self):
        """다리 하나가 기본임. 로봇 전체는 명시해야 함."""
        assert selftest.build_parser().parse_args(["zero"]).robot is False

    def test_robot_given_early_is_not_overwritten(self):
        """서브명령의 기본값이 앞에 적은 값을 덮으면 안 됨."""
        args = selftest.build_parser().parse_args(["--robot", "zero", "--approach", "1"])
        assert args.robot is True


# ===========================================================================
# 팔다리가 여럿일 때
# ===========================================================================
class FakeBiped:
    """`parts` 를 가진 로봇. 이름 붙이기와 한계 모으기만 봄."""

    id = "huphy"

    def __init__(self, parts):
        self.parts = tuple(parts)


def leg_named(name, limits=None):
    leg = FakeLeg(limits)
    leg.id = name
    leg.config = LimbConfig(name=name, kind="leg", motors=leg.config.motors)
    return leg


class TestParts:
    def test_a_single_leg_is_its_own_part(self):
        leg = FakeLeg()
        assert selftest.parts(leg) == (leg,)

    def test_a_composite_lists_its_limbs(self):
        left, right = leg_named("left_leg"), leg_named("right_leg")
        assert selftest.parts(FakeBiped([left, right])) == (left, right)

    def test_a_single_leg_keeps_bare_names(self):
        """구분할 대상이 없으면 접두어가 얻는 것 없이 화면만 바꿈."""
        leg = FakeLeg()
        assert selftest.named(leg, leg, "knee") == "knee"

    def test_a_composite_prefixes(self):
        right = leg_named("right_leg")
        assert selftest.named(FakeBiped([right]), right, "knee") == "right_leg/knee"


class TestCompositeLimits:
    def test_both_legs_appear(self):
        biped = FakeBiped([leg_named("right_leg"), leg_named("left_leg")])
        limits = selftest.joint_limits(biped)
        assert "right_leg/knee" in limits
        assert "left_leg/knee" in limits

    def test_each_leg_keeps_its_own_numbers(self):
        """좌우가 거울상이라도 실측값이라 같지 않음."""
        biped = FakeBiped([
            leg_named("right_leg", {"knee": (-20.0, 70.0)}),
            leg_named("left_leg", {"knee": (-18.0, 72.0)}),
        ])
        limits = selftest.joint_limits(biped)
        assert limits["right_leg/knee"] == (-20.0, 70.0)
        assert limits["left_leg/knee"] == (-18.0, 72.0)

    def test_an_unmeasured_joint_is_dropped_per_leg(self):
        """한쪽만 안 잰 상태가 있을 수 있음."""
        biped = FakeBiped([
            leg_named("right_leg", {"knee": (-20.0, 70.0)}),
            leg_named("left_leg", {}),
        ])
        limits = selftest.joint_limits(biped)
        assert "right_leg/knee" in limits
        assert "left_leg/knee" not in limits

    def test_ankles_come_from_kinematics_per_leg(self):
        biped = FakeBiped([leg_named("right_leg"), leg_named("left_leg")])
        limits = selftest.joint_limits(biped)
        assert limits["right_leg/ankle_pitch"] == (-40.0, 40.0)
        assert limits["left_leg/ankle_roll"] == (-25.0, 25.0)


# ===========================================================================
# 무엇을 조종할지
# ===========================================================================
TWO_LEG_YAML = """
name: t
limbs:
  right_leg:
    kind: leg
    side: right
    channel: can1
    motors:
      hip_pitch: {id: 7,  model: RS02, kp: 1.0, kd: 0.1}
      hip_roll:  {id: 8,  model: RS02, kp: 1.0, kd: 0.1}
      hip_yaw:   {id: 9,  model: RS02, kp: 1.0, kd: 0.1}
      knee:      {id: 10, model: RS02, kp: 1.0, kd: 0.1}
      ankle_a:   {id: 11, model: RS00, kp: 1.0, kd: 0.1}
      ankle_b:   {id: 12, model: RS00, kp: 1.0, kd: 0.1}
  left_leg:
    kind: leg
    side: left
    channel: can0
    motors:
      hip_pitch: {id: 1, model: RS02, kp: 1.0, kd: 0.1}
      hip_roll:  {id: 2, model: RS02, kp: 1.0, kd: 0.1}
      hip_yaw:   {id: 3, model: RS02, kp: 1.0, kd: 0.1}
      knee:      {id: 4, model: RS02, kp: 1.0, kd: 0.1}
      ankle_a:   {id: 5, model: RS00, kp: 1.0, kd: 0.1}
      ankle_b:   {id: 6, model: RS00, kp: 1.0, kd: 0.1}
"""


@pytest.fixture
def two_legs(tmp_path):
    path = tmp_path / "config" / "robot.yaml"
    path.parent.mkdir(parents=True)
    path.write_text(TWO_LEG_YAML, encoding="utf-8")
    return path


class TestTarget:
    def test_robot_and_limb_together_is_an_error(self, two_legs):
        """--robot 은 팔다리 전부이고 --limb 은 그중 골라 쓰는 것임."""
        with pytest.raises(SystemExit, match="같이 줄 수 없음"):
            selftest.main(["--config", str(two_legs), "--robot",
                           "--limb", "right_leg", "zero"])

    def test_two_legs_need_a_choice(self, two_legs):
        """팔다리마다 CAN 채널이 달라 잘못 고르면 엉뚱한 쪽이 움직임."""
        with pytest.raises(SystemExit, match="--limb"):
            selftest.main(["--config", str(two_legs), "zero"])
