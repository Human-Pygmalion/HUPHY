"""정책 어댑터 테스트 — 하드웨어도 모델도 없이 실행됨.

    PYTHONPATH=src python3 -m pytest tests -q

모델은 관절 이름을 모르고 **순서만** 봄. 여기서 확인하는 것은 그 순서와 단위임.
"""

import math

import numpy as np
import pytest

from huphy.control import policy
from huphy.control.policy import (
    ACTION_DIM,
    BALANCE,
    HOPPING,
    JOINT_ORDER,
    MOTOR_ORDER,
    ORDERS,
    hop_phase,
    joint_targets,
    observation_vector,
    policy_motion,
)
from huphy.sensors.base import ImuState, gravity_from_euler


def observation(**over):
    out = {}
    for joint in JOINT_ORDER:
        out[f"{joint}.pos"] = 0.0
        out[f"{joint}.vel"] = 0.0
    out.update(over)
    return out


def imu(**over):
    """센서 모듈이 이미 중력방향을 만들어 올린 상태임."""
    values = {"gravity": (0.0, 0.0, -1.0), "gyro_dps": (0.0, 0.0, 0.0), "is_valid": True}
    values.update(over)
    return ImuState(**values)


# ===========================================================================
# 관찰 벡터
# ===========================================================================
class TestObservationVector:
    """가중치 파일의 입력 차원과 맞아야 함 (balance 24, hopping 26)."""

    def test_balance_is_24(self):
        got = observation_vector(observation(), imu(), [0.0] * 6, spec=BALANCE)
        assert got.shape == (24,)

    def test_hopping_is_26(self):
        got = observation_vector(observation(), imu(), [0.0] * 6, spec=HOPPING)
        assert got.shape == (26,)

    def test_the_order_is_fixed(self):
        """각속도 3, 중력 3, 각도 6, 속도 6, 직전 행동 6."""
        got = observation_vector(
            observation(**{f"{j}.pos": 10.0 for j in JOINT_ORDER}),
            imu(gyro_dps=(1.0, 2.0, 3.0)),
            [0.5] * 6,
            spec=BALANCE,
        )
        assert got[0] == pytest.approx(math.radians(1.0))      # 각속도
        assert got[3:6] == pytest.approx([0.0, 0.0, -1.0])     # 중력
        assert got[6] == pytest.approx(math.radians(10.0))     # 각도
        assert got[12] == pytest.approx(0.0)                   # 속도
        assert got[18] == pytest.approx(0.5)                   # 직전 행동

    def test_degrees_become_radians(self):
        got = observation_vector(
            observation(**{"knee.pos": 90.0}), imu(), [0.0] * 6, spec=BALANCE
        )
        assert got[6 + JOINT_ORDER.index("knee")] == pytest.approx(math.pi / 2)

    def test_ankle_joints_come_from_the_observation(self):
        """모터각이 아니라 FK 로 푼 관절각임."""
        got = observation_vector(
            observation(**{"ankle_pitch.pos": 5.0, "ankle_roll.pos": -3.0}),
            imu(), [0.0] * 6, spec=BALANCE,
        )
        assert got[6 + 4] == pytest.approx(math.radians(5.0))
        assert got[6 + 5] == pytest.approx(math.radians(-3.0))

    def test_a_wrong_length_is_refused(self):
        """길이가 어긋나면 모델이 조용히 다른 값을 읽음."""
        broken = policy.PolicySpec(name="x", action_scale=0.25, obs_dim=99)
        with pytest.raises(ValueError, match="24개인데 99개"):
            observation_vector(observation(), imu(), [0.0] * 6, spec=broken)

    def test_it_is_float32(self):
        got = observation_vector(observation(), imu(), [0.0] * 6, spec=BALANCE)
        assert got.dtype == np.float32


# ===========================================================================
# 중력 투영
#
# 계산 자체는 sensors/base.py 에 있고 test_sensors.py 가 확인함. 여기서는
# **정책이 그 값을 그대로 쓰는지**만 봄 -- 센서가 어떤 형식으로 받았는지 정책은
# 몰라야 함.
# ===========================================================================
class TestProjectedGravity:
    def test_it_goes_into_the_vector_unchanged(self):
        got = observation_vector(
            observation(), imu(gravity=(0.1, -0.2, -0.97)), [0.0] * ACTION_DIM, spec=BALANCE
        )
        assert got[3:6] == pytest.approx((0.1, -0.2, -0.97))

    def test_the_policy_never_sees_the_raw_format(self):
        """오일러로 받은 센서든 쿼터니언으로 받은 센서든 같은 3칸이 들어감."""
        state = imu(gravity=gravity_from_euler(10.0, 20.0))
        state.extra = {"roll": 10.0, "pitch": 20.0, "yaw": 0.0}
        got = observation_vector(observation(), state, [0.0] * ACTION_DIM, spec=BALANCE)
        assert got[3:6] == pytest.approx(gravity_from_euler(10.0, 20.0))


# ===========================================================================
# 뛰는 위상
# ===========================================================================
class TestHopPhase:
    def test_it_starts_at_zero_phase(self):
        assert hop_phase(0.0, 0.6) == pytest.approx((0.0, 1.0))

    def test_it_wraps_without_a_jump(self):
        """각도로 주면 한 바퀴에서 값이 튐. sin/cos 는 이어짐."""
        before = hop_phase(0.599, 0.6)
        after = hop_phase(0.601, 0.6)
        assert abs(before[0] - after[0]) < 0.05
        assert abs(before[1] - after[1]) < 0.05

    def test_half_period_is_opposite(self):
        assert hop_phase(0.3, 0.6) == pytest.approx((0.0, -1.0), abs=1e-9)

    def test_zero_period_is_safe(self):
        assert hop_phase(1.0, 0.0) == (0.0, 1.0)


# ===========================================================================
# 행동 -> 관절 목표
# ===========================================================================
class TestJointTargets:
    def test_scale_is_applied(self):
        got = joint_targets([1.0] * 6, spec=BALANCE)
        assert got["knee"] == pytest.approx(math.degrees(0.25))

    def test_hopping_scales_more(self):
        """raw +-1 이 balance 는 +-14도, hopping 은 +-29도."""
        assert joint_targets([1.0] * 6, spec=HOPPING)["knee"] == pytest.approx(
            math.degrees(0.5)
        )

    def test_every_joint_is_named(self):
        assert set(joint_targets([0.0] * 6, spec=BALANCE)) == set(JOINT_ORDER)

    def test_the_order_matches_the_joints(self):
        got = joint_targets([0.0, 0.0, 0.0, 1.0, 0.0, 0.0], spec=BALANCE)
        assert got["knee"] != 0.0
        assert got["hip_pitch"] == 0.0

    def test_a_wrong_length_is_refused(self):
        with pytest.raises(ValueError, match="6개여야 함"):
            joint_targets([0.0] * 5, spec=BALANCE)


# ===========================================================================
# Motion 으로 쓰기
# ===========================================================================
class FakeImu:
    def __init__(self, state=None):
        self.state = state if state is not None else imu()

    def read(self):
        return self.state


class TestPolicyMotion:
    def test_it_has_the_motion_shape(self):
        """`(t, observation) -> action`. 제어 루프가 그대로 받음."""
        motion = policy_motion(lambda v: [0.0] * 6, FakeImu(), spec=BALANCE)
        got = motion(0.0, observation())
        assert set(got) == set(JOINT_ORDER)

    def test_the_model_sees_the_right_length(self):
        seen = []
        motion = policy_motion(
            lambda v: (seen.append(v.shape), [0.0] * 6)[1], FakeImu(), spec=HOPPING
        )
        motion(0.0, observation())
        assert seen == [(26,)]

    def test_the_last_action_comes_back_in_the_observation(self):
        """관찰 24개 중 6개가 직전 행동임. 들고 있어야 함."""
        seen = []

        def model(vector):
            seen.append(vector.copy())
            return [0.7] * 6

        motion = policy_motion(model, FakeImu(), spec=BALANCE)
        motion(0.0, observation())
        motion(0.02, observation())
        assert seen[0][18:24] == pytest.approx([0.0] * 6)      # 첫 주기는 0
        assert seen[1][18:24] == pytest.approx([0.7] * 6)

    def test_the_last_action_is_before_scaling(self):
        """시뮬의 last_action 은 관절 목표가 아니라 모델 출력 그대로임."""
        seen = []

        def model(vector):
            seen.append(vector.copy())
            return [1.0] * 6

        motion = policy_motion(model, FakeImu(), spec=BALANCE)
        motion(0.0, observation())
        motion(0.02, observation())
        assert seen[1][18] == pytest.approx(1.0)               # 0.25 가 아님

    def test_it_reads_the_imu_every_cycle(self):
        sensor = FakeImu()
        calls = []
        original = sensor.read
        sensor.read = lambda: (calls.append(1), original())[1]

        motion = policy_motion(lambda v: [0.0] * 6, sensor, spec=BALANCE)
        motion(0.0, observation())
        motion(0.02, observation())
        assert len(calls) == 2


# ===========================================================================
# 규격
# ===========================================================================
class TestSpecs:
    def test_balance_matches_the_weights(self):
        """가중치 파일의 입력이 24, 출력이 6 임."""
        assert BALANCE.obs_dim == 24
        assert ACTION_DIM == 6

    def test_hopping_matches_the_weights(self):
        assert HOPPING.obs_dim == 26

    def test_the_scales_come_from_the_sim(self):
        assert BALANCE.action_scale == 0.25
        assert HOPPING.action_scale == 0.5

    def test_only_hopping_has_a_phase(self):
        assert not BALANCE.uses_hop_phase
        assert HOPPING.uses_hop_phase


# ===========================================================================
# 발목이 두 갈래
# ===========================================================================
def motor_observation(**over):
    out = {}
    for joint in MOTOR_ORDER:
        out[f"{joint}.pos"] = 0.0
        out[f"{joint}.vel"] = 0.0
    out.update(over)
    return out


class TestMotorSpace:
    def test_only_the_ankle_differs(self):
        """앞 네 칸이 같아야 함. 순서가 어긋나면 값은 다 정상인데 로봇만 엉뚱해짐."""
        assert MOTOR_ORDER[:4] == JOINT_ORDER[:4]
        assert MOTOR_ORDER[4:] == ("ankle_a", "ankle_b")

    def test_same_length(self):
        """행동 개수로는 두 공간이 구분되지 않음. 사람이 골라야 하는 이유임."""
        assert len(MOTOR_ORDER) == len(JOINT_ORDER)

    def test_orders_maps_the_cli_values(self):
        assert ORDERS == {"rp": JOINT_ORDER, "ab": MOTOR_ORDER}

    def test_observation_reads_the_motor_axes(self):
        """모터 값은 Leg 관찰에 이미 있음. 관찰 쪽은 손댈 것이 없음."""
        vector = observation_vector(
            motor_observation(**{"ankle_a.pos": 30.0}),
            imu(),
            [0.0] * 6,
            spec=BALANCE,
            order=MOTOR_ORDER,
        )
        assert vector[6 + 4] == pytest.approx(math.radians(30.0))

    def test_observation_length_is_unchanged(self):
        """공간이 관찰 길이를 안 바꿈. obs_dim 검사로는 못 거름."""
        vector = observation_vector(
            motor_observation(), imu(), [0.0] * 6,
            spec=BALANCE, order=MOTOR_ORDER,
        )
        assert vector.size == BALANCE.obs_dim

    def test_targets_carry_motor_names(self):
        targets = joint_targets([0.0] * 6, spec=BALANCE, order=MOTOR_ORDER)
        assert set(targets) == set(MOTOR_ORDER)
        assert "ankle_pitch" not in targets

    def test_scale_is_the_same(self):
        """공간이 바뀌어도 action_scale 의 뜻은 같음."""
        rp = joint_targets([1.0] * 6, spec=BALANCE, order=JOINT_ORDER)
        ab = joint_targets([1.0] * 6, spec=BALANCE, order=MOTOR_ORDER)
        assert ab["ankle_a"] == pytest.approx(rp["ankle_pitch"])

    def test_wrong_action_count_is_measured_against_the_order(self):
        with pytest.raises(ValueError, match="6개여야 함"):
            joint_targets([0.0] * 5, spec=BALANCE, order=MOTOR_ORDER)

    def test_motion_uses_the_order_on_both_sides(self):
        """관찰과 목표에 같은 순서가 걸려야 함. 한쪽만 바꾸면 축이 어긋남."""
        seen = {}

        def model(vector):
            seen["size"] = vector.size
            return [0.0] * 6

        motion = policy_motion(model, FakeImu(), spec=BALANCE, order=MOTOR_ORDER)
        targets = motion(0.0, motor_observation())

        assert seen["size"] == BALANCE.obs_dim
        assert set(targets) == set(MOTOR_ORDER)
