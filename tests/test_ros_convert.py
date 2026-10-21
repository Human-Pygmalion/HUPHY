"""ROS 경계의 단위·이름 변환 — **`rclpy` 없이 돎.**

    PYTHONPATH=src python3 -m pytest tests/test_ros_convert.py -q

`huphy_ros/convert.py` 가 `rclpy` 를 import 하지 않게 둔 이유가 이것임. 여기서
고정하는 것은 **틀려도 길이와 자릿수가 그대로라 코드로는 안 잡히는 것**임.

    단위   도 <-> 라디안
    이름   right_leg/knee <-> right_leg__knee
    순서   사전이 아니라 ORDER 가 정함
"""

import math

import pytest

pytest.importorskip("huphy_ros", reason="ros/huphy_ros 가 경로에 없음")

from huphy.control import generalized_walk as gw        # noqa: E402
from huphy.robots.biped import SEP                      # noqa: E402
from huphy_ros import convert                           # noqa: E402


def default_observation():
    return {f"{n}.pos": gw.DEFAULT_POSE_DEG[n] for n in gw.ORDER}


# ===========================================================================
# 이름
# ===========================================================================
class TestNames:
    def test_limb_separator_becomes_double_underscore(self):
        assert convert.to_ros_name("right_leg/knee") == "right_leg__knee"
        assert convert.to_huphy_name("right_leg__knee") == "right_leg/knee"

    def test_a_bare_joint_name_is_untouched(self):
        """다리 하나로 돌리면 접두어가 없음."""
        assert convert.to_ros_name("knee") == "knee"
        assert convert.to_huphy_name("knee") == "knee"

    def test_round_trip_for_every_joint(self):
        for name in gw.ORDER:
            assert convert.to_huphy_name(convert.to_ros_name(name)) == name

    def test_ros_names_carry_no_slash(self):
        """ROS 에서 / 는 네임스페이스 구분자라 사람이 헷갈림."""
        for name in gw.ORDER:
            assert SEP not in convert.to_ros_name(name)


# ===========================================================================
# 관절 이름 뽑기
# ===========================================================================
class TestJointNames:
    def test_scalar_entries_are_not_joints(self):
        """stale_motors 는 관절이 아님 (robots/leg.py 의 get_observation)."""
        observation = {"knee.pos": 0.0, "knee.vel": 0.0, "stale_motors": 2}
        assert convert.joint_names(observation) == ("knee",)

    def test_order_follows_the_observation(self):
        assert convert.joint_names(default_observation()) == gw.ORDER


# ===========================================================================
# 관찰 -> JointState
# ===========================================================================
class TestObservationOut:
    def test_position_goes_out_in_radians(self):
        """저장소는 도, ROS 는 라디안 (REP 103)."""
        _, position, _, _ = convert.observation_to_arrays(
            {"knee.pos": 90.0}, ["knee"])
        assert position[0] == pytest.approx(math.pi / 2)

    def test_velocity_too(self):
        _, _, velocity, _ = convert.observation_to_arrays(
            {"knee.pos": 0.0, "knee.vel": 180.0}, ["knee"])
        assert velocity[0] == pytest.approx(math.pi)

    def test_torque_is_not_converted(self):
        """Nm 은 양쪽이 같은 단위임."""
        _, _, _, effort = convert.observation_to_arrays(
            {"knee.pos": 0.0, "knee.torque": 1.75}, ["knee"])
        assert effort[0] == pytest.approx(1.75)

    def test_missing_fields_become_zero_not_short_arrays(self):
        """JointState 는 네 배열 길이가 같아야 함. 짧으면 짝을 못 맞춤."""
        names, position, velocity, effort = convert.observation_to_arrays(
            {"knee.pos": 10.0}, ["knee", "hip_pitch"])
        assert len(names) == len(position) == len(velocity) == len(effort) == 2
        assert position[1] == 0.0

    def test_names_come_out_in_ros_form(self):
        names, _, _, _ = convert.observation_to_arrays(
            default_observation(), gw.ORDER)
        assert names[0] == "right_leg__hip_pitch"


# ===========================================================================
# JointState -> 관찰
# ===========================================================================
class TestObservationIn:
    def test_round_trip_through_the_default_pose(self):
        names, position, _, _ = convert.observation_to_arrays(
            default_observation(), gw.ORDER)
        back = convert.arrays_to_observation(names, position)
        for name in gw.ORDER:
            assert back[f"{name}.pos"] == pytest.approx(
                gw.DEFAULT_POSE_DEG[name], abs=1e-9)

    def test_velocity_is_optional(self):
        """정책 관찰에 관절 속도 칸이 없어서 안 줘도 됨."""
        back = convert.arrays_to_observation(["knee"], [0.0])
        assert back == {"knee.pos": 0.0}

    def test_mismatched_lengths_are_refused(self):
        with pytest.raises(ValueError):
            convert.arrays_to_observation(["a", "b"], [0.0])


# ===========================================================================
# 목표각
# ===========================================================================
class TestTargets:
    def test_order_comes_from_ORDER_not_the_dict(self):
        """사전 순서에 기대면 보내는 쪽이 바뀌었을 때 열이 섞임."""
        targets = {n: 0.0 for n in reversed(gw.ORDER)}
        names, _ = convert.targets_to_arrays(targets, gw.ORDER)
        assert names == [convert.to_ros_name(n) for n in gw.ORDER]

    def test_a_missing_joint_is_refused(self):
        targets = {n: 0.0 for n in gw.ORDER[:-1]}
        with pytest.raises(ValueError):
            convert.targets_to_arrays(targets, gw.ORDER)

    def test_round_trip(self):
        names, position = convert.targets_to_arrays(gw.default_pose(), gw.ORDER)
        back = convert.arrays_to_targets(names, position)
        for name in gw.ORDER:
            assert back[name] == pytest.approx(gw.DEFAULT_POSE_DEG[name], abs=1e-9)

    def test_incoming_is_radians(self):
        assert convert.arrays_to_targets(
            ["right_leg__knee"], [math.pi / 2]) == {
                "right_leg/knee": pytest.approx(90.0)}

    def test_mismatched_lengths_are_refused(self):
        """zip 은 짧은 쪽에 맞춰 조용히 자름. 관절 하나가 빠진 명령이 통과함."""
        with pytest.raises(ValueError):
            convert.arrays_to_targets(["a", "b"], [0.0])


# ===========================================================================
# IMU
# ===========================================================================
class TestImu:
    def test_gyro_goes_out_in_radians(self):
        from huphy.sensors.base import ImuState
        gyro, _, _ = convert.imu_arrays(ImuState(gyro_dps=(90.0, 0.0, 0.0)))
        assert gyro[0] == pytest.approx(math.pi / 2)

    def test_gravity_is_not_converted(self):
        """이미 단위벡터임."""
        from huphy.sensors.base import ImuState
        _, _, gravity = convert.imu_arrays(ImuState(gravity=(0.1, 0.2, -0.97)))
        assert gravity == pytest.approx([0.1, 0.2, -0.97])


# ===========================================================================
# 한 바퀴
# ===========================================================================
class TestLoop:
    def test_the_policy_sees_what_control_measured(self):
        """control_node 가 내보낸 것을 policy_node 가 그대로 읽어야 함.

        도 -> rad -> 네트워크 -> rad -> 도 를 지나도 값이 안 변해야 함.
        """
        measured = {f"{n}.pos": gw.DEFAULT_POSE_DEG[n] + 3.0 for n in gw.ORDER}
        names, position, _, _ = convert.observation_to_arrays(measured, gw.ORDER)
        seen = convert.arrays_to_observation(names, position)
        for name in gw.ORDER:
            assert seen[f"{name}.pos"] == pytest.approx(
                measured[f"{name}.pos"], abs=1e-9)

    def test_control_gets_what_the_policy_meant(self):
        wanted = gw.joint_targets([0.3] * len(gw.ORDER))
        names, position = convert.targets_to_arrays(wanted, gw.ORDER)
        got = convert.arrays_to_targets(names, position)
        for name in gw.ORDER:
            assert got[name] == pytest.approx(wanted[name], abs=1e-9)
