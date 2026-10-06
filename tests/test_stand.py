"""서기 정책 어댑터 테스트 — 하드웨어도 가중치 파일도 없이 실행됨.

    PYTHONPATH=src python3 -m pytest tests/test_stand.py -q

확인하는 것은 **칸 배치와 단위**임. 둘 다 틀려도 길이가 같아서 코드로는 안 잡히는
자리라, 테스트로 고정해 둠.

학습 쪽 꾸러미의 참조 입출력으로 하는 대조는 여기 없음 -- 그 파일이 저장소 밖에
있어서임. 그 대조는 `docs/policy_runner.md` 에 절차로 적어 둠.
"""

import math

import numpy as np
import pytest

from huphy.control import stand
from huphy.sensors.base import ImuState


def imu_state(gyro_dps=(0.0, 0.0, 0.0), gravity=(0.0, 0.0, -1.0)) -> ImuState:
    """`observation_vector` 가 받는 것은 센서가 아니라 읽어 둔 값임."""
    return ImuState(gyro_dps=tuple(gyro_dps), gravity=tuple(gravity))


class FakeImu:
    """`read()` 만 있는 센서. `motion` 이 이것밖에 안 부름."""

    def __init__(self, gyro_dps=(0.0, 0.0, 0.0), gravity=(0.0, 0.0, -1.0)):
        self.state = imu_state(gyro_dps, gravity)

    def read(self) -> ImuState:
        return self.state


def observation(pos=None):
    """관절 이름 -> 각도(도). 안 주면 기준 자세."""
    source = stand.DEFAULT_POSE_DEG if pos is None else pos
    return {f"{j}.pos": float(source[j]) for j in stand.ORDER}


def default_list():
    return [stand.DEFAULT_POSE_DEG[j] for j in stand.ORDER]


# ===========================================================================
# 규격
# ===========================================================================
class TestSpec:
    def test_twelve_joints_right_leg_first(self):
        """0.5 의 BIPED_LEGS 는 왼다리가 먼저인데 이 정책은 반대임."""
        assert len(stand.ORDER) == 12
        assert stand.ORDER[0].startswith("right_leg/")
        assert stand.ORDER[6].startswith("left_leg/")

    def test_ankle_is_motor_space(self):
        """발판 자세가 아니라 크랭크 두 개를 직접 냄."""
        assert stand.ORDER[4].endswith("/ankle_a")
        assert stand.ORDER[5].endswith("/ankle_b")
        assert not any(name.endswith("/ankle_pitch") for name in stand.ORDER)

    def test_obs_dim_is_45(self):
        """3 + 3 + 12x2 + 12 + 3."""
        assert stand.OBS_DIM == 45
        assert stand.SPEC.obs_dim == 45

    def test_every_joint_has_a_default_and_a_limit(self):
        for joint in stand.ORDER:
            assert joint in stand.DEFAULT_POSE_DEG
            low, high = stand.LIMITS_DEG[joint]
            assert low < high

    def test_default_pose_is_not_zero(self):
        """살짝 웅크린 자세임. 0 이면 0.5 와 같은 가정으로 돌아간 것임."""
        assert stand.DEFAULT_POSE_DEG["right_leg/knee"] == pytest.approx(20.05, abs=0.1)
        assert stand.DEFAULT_POSE_DEG["left_leg/knee"] == pytest.approx(-20.05, abs=0.1)

    def test_left_and_right_mirror(self):
        """뒤집으면 로봇이 뒤로 감."""
        for joint in ("hip_pitch", "knee"):
            right = stand.DEFAULT_POSE_DEG[f"right_leg/{joint}"]
            left = stand.DEFAULT_POSE_DEG[f"left_leg/{joint}"]
            assert right == pytest.approx(-left)


# ===========================================================================
# 관찰
# ===========================================================================
class TestObservation:
    def test_length(self):
        v = stand.observation_vector(
            observation(), imu_state(), [0.0] * 12, default_list()
        )
        assert v.shape == (45,)
        assert v.dtype == np.float32

    def test_gyro_is_converted_to_radians(self):
        """센서는 도/초로 올리고 모델은 rad/s 를 받음."""
        v = stand.observation_vector(
            observation(), imu_state(gyro_dps=(90.0, 0.0, 0.0)), [0.0] * 12, default_list()
        )
        assert v[0] == pytest.approx(math.pi / 2, abs=1e-5)

    def test_gravity_goes_in_as_is(self):
        """센서 모듈이 이미 만들어 둔 값임. 서 있으면 z 가 -1."""
        v = stand.observation_vector(
            observation(), imu_state(gravity=(0.1, 0.2, -0.97)), [0.0] * 12, default_list()
        )
        assert v[3:6] == pytest.approx([0.1, 0.2, -0.97], abs=1e-6)

    def test_default_pose_is_subtracted(self):
        """기준 자세에 가만히 있으면 위치 24칸이 전부 0 임."""
        v = stand.observation_vector(
            observation(), imu_state(), [0.0] * 12, default_list()
        )
        assert v[6:30] == pytest.approx([0.0] * 24, abs=1e-6)

    def test_history_is_oldest_first_and_not_interleaved(self):
        """[q(t-1) 전체 12][q(t) 전체 12] 임. 관절별로 묶이지 않음."""
        previous = default_list()
        previous[0] += 10.0                      # 직전 주기에 오른쪽 hip_pitch 만 달랐음
        now = dict(stand.DEFAULT_POSE_DEG)
        now["left_leg/knee"] -= 10.0             # 지금은 왼쪽 knee 만 다름

        v = stand.observation_vector(
            observation(now), imu_state(), [0.0] * 12, previous
        )
        old, new = v[6:18], v[18:30]
        assert old[0] == pytest.approx(math.radians(10.0), abs=1e-6)
        assert old[9] == pytest.approx(0.0, abs=1e-6)
        assert new[0] == pytest.approx(0.0, abs=1e-6)
        assert new[9] == pytest.approx(math.radians(-10.0), abs=1e-6)

    def test_last_action_goes_in_raw(self):
        """목표각이 아니라 모델이 낸 날것임."""
        action = [float(i) for i in range(12)]
        v = stand.observation_vector(
            observation(), imu_state(), action, default_list()
        )
        assert v[30:42] == pytest.approx(action)

    def test_command_is_zero_for_standing(self):
        v = stand.observation_vector(
            observation(), imu_state(), [0.0] * 12, default_list()
        )
        assert v[42:45] == pytest.approx([0.0, 0.0, 0.0])

    def test_short_history_is_refused(self):
        with pytest.raises(ValueError):
            stand.observation_vector(
                observation(), imu_state(), [0.0] * 12, [0.0] * 6
            )


# ===========================================================================
# 행동
# ===========================================================================
class TestTargets:
    def test_zero_action_is_the_default_pose(self):
        targets = stand.joint_targets([0.0] * 12)
        for joint in stand.ORDER:
            assert targets[joint] == pytest.approx(stand.DEFAULT_POSE_DEG[joint])

    def test_scale_is_quarter_radian_per_unit(self):
        """행동 1 이 기준자세에서 0.25rad = 14.324도."""
        targets = stand.joint_targets([1.0] + [0.0] * 11)
        joint = stand.ORDER[0]
        moved = targets[joint] - stand.DEFAULT_POSE_DEG[joint]
        assert moved == pytest.approx(math.degrees(0.25), abs=1e-3)

    def test_targets_are_clipped(self):
        """자르지 않으면 학습 모형 범위 밖을 목표로 삼음."""
        targets = stand.joint_targets([100.0] * 12)
        for joint, value in targets.items():
            low, high = stand.LIMITS_DEG[joint]
            assert low <= value <= high

    def test_wrong_action_count_is_refused(self):
        with pytest.raises(ValueError):
            stand.joint_targets([0.0] * 6)

    def test_keys_are_motor_names(self):
        """Leg 이 이 이름을 보고 기구학을 건너뜀."""
        targets = stand.joint_targets([0.0] * 12)
        assert set(targets) == set(stand.ORDER)


# ===========================================================================
# Motion
# ===========================================================================
class TestMotion:
    def test_first_cycle_backfills_history(self):
        """이력이 없을 때 0 으로 두면 기준자세만큼 움직인 것으로 읽힘."""
        seen = []

        def model(vector):
            seen.append(np.array(vector))
            return [0.0] * 12

        pose = dict(stand.DEFAULT_POSE_DEG)
        pose["right_leg/knee"] += 5.0
        step = stand.motion(model, FakeImu())
        step(0.0, observation(pose))

        old, new = seen[0][6:18], seen[0][18:30]
        assert old == pytest.approx(new, abs=1e-6)
        assert new[3] == pytest.approx(math.radians(5.0), abs=1e-6)

    def test_history_shifts_by_one_call(self):
        """두 번째 호출의 '오래된 프레임' 이 첫 호출의 위치임."""
        seen = []

        def model(vector):
            seen.append(np.array(vector))
            return [0.0] * 12

        first = dict(stand.DEFAULT_POSE_DEG)
        first["right_leg/knee"] += 5.0
        second = dict(stand.DEFAULT_POSE_DEG)
        second["right_leg/knee"] += 9.0

        step = stand.motion(model, FakeImu())
        step(0.0, observation(first))
        step(0.02, observation(second))

        assert seen[1][6 + 3] == pytest.approx(math.radians(5.0), abs=1e-6)
        assert seen[1][18 + 3] == pytest.approx(math.radians(9.0), abs=1e-6)

    def test_last_action_is_fed_back_raw(self):
        """목표각이 아니라 모델 출력 그대로 돌아가야 함."""
        seen = []

        def model(vector):
            seen.append(np.array(vector))
            return [0.5] * 12

        step = stand.motion(model, FakeImu())
        step(0.0, observation())
        step(0.02, observation())

        assert seen[0][30:42] == pytest.approx([0.0] * 12)
        assert seen[1][30:42] == pytest.approx([0.5] * 12)

    def test_targets_come_out_in_degrees(self):
        step = stand.motion(lambda v: [0.0] * 12, FakeImu())
        targets = step(0.0, observation())
        assert targets["right_leg/knee"] == pytest.approx(20.05, abs=0.1)
