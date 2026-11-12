"""ROS 경계의 단위·이름 변환 — **`rclpy` 없이 돎.**

    PYTHONPATH=src python3 -m pytest tests/test_ros_convert.py -q

`huphy_ros/convert.py` 가 `rclpy` 를 import 하지 않게 둔 이유가 이것임. 여기서
고정하는 것은 **틀려도 길이와 자릿수가 그대로라 코드로는 안 잡히는 것**임.

    단위   도 <-> 라디안
    이름   right_leg/knee <-> right_leg__knee
    순서   사전이 아니라 ORDER 가 정함
"""

import math
from pathlib import Path

import pytest

pytest.importorskip("huphy_ros", reason="ros/huphy_ros 가 경로에 없음")

from huphy.control import deploy                       # noqa: E402
from huphy.robots.biped import SEP                      # noqa: E402
from huphy_ros import convert                           # noqa: E402


WALK = deploy.load(Path(__file__).resolve().parent.parent / "config" / "policies" / "walk")


def default_observation():
    return {f"{n}.pos": WALK.default_pose_deg[n] for n in WALK.order}


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
        for name in WALK.order:
            assert convert.to_huphy_name(convert.to_ros_name(name)) == name

    def test_ros_names_carry_no_slash(self):
        """ROS 에서 / 는 네임스페이스 구분자라 사람이 헷갈림."""
        for name in WALK.order:
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
        assert convert.joint_names(default_observation()) == WALK.order


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
            default_observation(), WALK.order)
        assert names[0] == "right_leg__hip_pitch"


# ===========================================================================
# JointState -> 관찰
# ===========================================================================
class TestObservationIn:
    def test_round_trip_through_the_default_pose(self):
        names, position, _, _ = convert.observation_to_arrays(
            default_observation(), WALK.order)
        back = convert.arrays_to_observation(names, position)
        for name in WALK.order:
            assert back[f"{name}.pos"] == pytest.approx(
                WALK.default_pose_deg[name], abs=1e-9)

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
        targets = {n: 0.0 for n in reversed(WALK.order)}
        names, _ = convert.targets_to_arrays(targets, WALK.order)
        assert names == [convert.to_ros_name(n) for n in WALK.order]

    def test_a_missing_joint_is_refused(self):
        targets = {n: 0.0 for n in WALK.order[:-1]}
        with pytest.raises(ValueError):
            convert.targets_to_arrays(targets, WALK.order)

    def test_round_trip(self):
        names, position = convert.targets_to_arrays(WALK.default_pose(), WALK.order)
        back = convert.arrays_to_targets(names, position)
        for name in WALK.order:
            assert back[name] == pytest.approx(WALK.default_pose_deg[name], abs=1e-9)

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
        measured = {f"{n}.pos": WALK.default_pose_deg[n] + 3.0 for n in WALK.order}
        names, position, _, _ = convert.observation_to_arrays(measured, WALK.order)
        seen = convert.arrays_to_observation(names, position)
        for name in WALK.order:
            assert seen[f"{name}.pos"] == pytest.approx(
                measured[f"{name}.pos"], abs=1e-9)

    def test_control_gets_what_the_policy_meant(self):
        wanted = WALK.joint_targets([0.3] * len(WALK.order))
        names, position = convert.targets_to_arrays(wanted, WALK.order)
        got = convert.arrays_to_targets(names, position)
        for name in WALK.order:
            assert got[name] == pytest.approx(wanted[name], abs=1e-9)


# ===========================================================================
# 모터 전기값 (MotorExtra)
# ===========================================================================
class TestExtra:
    """`control_node` -> `MotorExtra` -> `log_node` -> 기록 열.

    고정하는 것은 **열 이름과 순서**임. 틀려도 열 수가 같아서 코드로는 안 잡히고,
    뷰어가 다른 모터의 그래프를 그림.
    """

    OBS = {
        "right_leg/knee.iq": 2.5,
        "right_leg/knee.vbus": 48.1,
        "right_leg/knee.temp": 31.0,
        "left_leg/knee.iq": -1.5,
        "left_leg/knee.vbus": 47.9,
        "left_leg/knee.temp": 33.0,
    }
    NAMES = ("right_leg/knee", "left_leg/knee")

    def test_arrays_follow_the_given_order(self):
        iq, vbus, temp = convert.observation_to_extra(self.OBS, self.NAMES)
        assert iq == pytest.approx([2.5, -1.5])
        assert vbus == pytest.approx([48.1, 47.9])
        assert temp == pytest.approx([31.0, 33.0])

    def test_missing_values_are_nan_not_zero(self):
        """`extra` 가 0 이면 관찰에 `iq`·`vbus` 가 없음. 0 으로 채우면 0A 로 기록됨."""
        obs = {"right_leg/knee.temp": 31.0}
        iq, vbus, temp = convert.observation_to_extra(obs, ("right_leg/knee",))
        assert math.isnan(iq[0]) and math.isnan(vbus[0])
        assert temp == pytest.approx([31.0])

    def test_columns_match_the_non_ros_recording(self):
        """`huphy-run --extra` 기록과 열 이름·순서가 같아야 같은 뷰어로 봄."""
        from huphy.telemetry.track_log import extra_columns, extra_values

        iq, vbus, temp = convert.observation_to_extra(self.OBS, self.NAMES)
        got = convert.extra_to_columns(self.NAMES, iq, vbus, temp)
        assert tuple(got) == extra_columns(self.NAMES)
        assert got == pytest.approx(extra_values(self.OBS))

    def test_short_arrays_stop_instead_of_filling_with_zero(self):
        from huphy.telemetry.track_log import extra_columns

        got = convert.extra_to_columns(self.NAMES, [2.5], [48.1], [31.0])
        assert tuple(got) == extra_columns(self.NAMES[:1])

    def test_a_row_keeps_its_width_when_the_columns_are_fixed_up_front(self):
        """`log_node` 가 열을 미리 박는 길. 첫 줄에 전기값이 없어도 줄 폭이 같아야 함."""
        from huphy.telemetry.policy_log import PolicyLog

        log = PolicyLog(("obs/a",), self.NAMES, limits={}, extra_motors=self.NAMES)
        log.add_row(0.00, [1.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0])
        iq, vbus, temp = convert.observation_to_extra(self.OBS, self.NAMES)
        log.add_row(0.02, [1.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0],
                    convert.extra_to_columns(self.NAMES, iq, vbus, temp))

        assert len(log.columns) == 1 + 1 + 2 * 3 + 2 * 3
        assert all(len(r) == len(log.columns) for r in log.rows)
        i = log.columns.index("extra/right_leg/knee/iq")
        assert math.isnan(log.rows[0][i])
        assert log.rows[1][i] == pytest.approx(2.5)


# ===========================================================================
# 모터 모델명 (ControlInfo.motor_models)
# ===========================================================================
class TestMotorModels:
    """`log_node` 가 전력 열의 상저항을 가르는 값.

    이름이 `latency_tap.TapSet.joints` 와 같아야 함 -- 어긋나면 `log_node` 가
    상저항을 못 찾아 전력 열이 전부 `null` 이 됨.
    """

    class _Motor:
        def __init__(self, model):
            self.model = model

    class _Config:
        def __init__(self, motors, ids):
            self.motors = motors
            self._ids = ids

        def joint_of(self, mid):
            return self._ids.get(mid)

    class _Bus:
        def __init__(self, ids):
            self.motor_ids = tuple(ids)

    class _Part:
        def __init__(self, part_id, motors, ids):
            self.id = part_id
            self.bus = TestMotorModels._Bus(ids)
            self.config = TestMotorModels._Config(motors, ids)

    def _leg(self, part_id):
        motors = {"knee": self._Motor("RS04"), "hip_yaw": self._Motor("RS03")}
        return self._Part(part_id, motors, {10: "knee", 9: "hip_yaw"})

    def test_names_are_prefixed_with_the_limb_id(self):
        got = convert.motor_models(self._leg("right_leg"))
        assert got == {"right_leg/knee": "RS04", "right_leg/hip_yaw": "RS03"}

    def test_both_legs_are_collected(self):
        class Biped:
            pass

        robot = Biped()
        robot.parts = (self._leg("right_leg"), self._leg("left_leg"))
        got = convert.motor_models(robot)
        assert set(got) == {
            "right_leg/knee", "right_leg/hip_yaw",
            "left_leg/knee", "left_leg/hip_yaw",
        }

    def test_a_can_id_without_a_joint_is_left_empty(self):
        """관절이 안 붙은 모터. 빈 문자열이면 `log_node` 에서 상저항이 NaN 이 됨."""
        part = self._Part("right_leg", {"knee": self._Motor("RS04")}, {10: "knee"})
        part.bus.motor_ids = (10, 99)
        part.config._ids = {10: "knee"}
        got = convert.motor_models(part)
        assert got == {"right_leg/knee": "RS04", "right_leg/99": ""}

    def test_the_model_resolves_to_a_phase_resistance(self):
        from huphy.motors.robstride import tables

        got = convert.motor_models(self._leg("right_leg"))
        assert tables.phase_resistance_ohm(got["right_leg/knee"]) == pytest.approx(0.08)
        assert tables.phase_resistance_ohm(got["right_leg/hip_yaw"]) == pytest.approx(0.195)
