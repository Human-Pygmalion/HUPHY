"""ROS 경계의 순수 모듈 테스트 — **`rclpy` 없이 돎.**

    PYTHONPATH=src:ros/huphy_ros python3 -m pytest tests/test_ros_bridge.py -q

`huphy_ros/convert.py` 와 `huphy_ros/bridge.py` 가 `rclpy` 를 import 하지 않게 둔
이유가 이것임. 노드(`control_node.py`, `telemetry_node.py`)는 배선뿐이라 여기서
안 봄 -- 그쪽은 ROS 가 깔린 기계에서 `launch_testing` 으로 봄.

여기서 고정하는 것은 **틀려도 길이와 자릿수가 그대로라 코드로는 안 잡히는 것**임.

    단위    도 <-> 라디안
    이름    right_leg/knee <-> right_leg__knee
    오래됨  명령이 안 오면 전송을 멈추는가
"""

import math

import pytest

pytest.importorskip("huphy_ros", reason="PYTHONPATH 에 ros/huphy_ros 가 없음")

from huphy.control import generalized_walk as stand                       # noqa: E402
from huphy_ros import convert                         # noqa: E402
from huphy_ros.bridge import CommandBridge            # noqa: E402


class Clock:
    """테스트용 시계. `time.monotonic` 자리에 들어감."""

    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


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

    def test_round_trip_for_every_stand_joint(self):
        for name in stand.ORDER:
            assert convert.to_huphy_name(convert.to_ros_name(name)) == name

    def test_ros_names_carry_no_slash(self):
        """ROS 에서 `/` 는 네임스페이스 구분자라 사람이 헷갈림."""
        for name in stand.ORDER:
            assert "/" not in convert.to_ros_name(name)


# ===========================================================================
# 단위
# ===========================================================================
class TestUnits:
    def test_observation_goes_out_in_radians(self):
        """저장소는 도, ROS 는 라디안 (REP 103)."""
        observation = {"right_leg/knee.pos": 90.0, "right_leg/knee.vel": 180.0}
        names, position, velocity, _ = convert.observation_to_arrays(
            observation, ["right_leg/knee"]
        )
        assert names == ["right_leg__knee"]
        assert position[0] == pytest.approx(math.pi / 2)
        assert velocity[0] == pytest.approx(math.pi)

    def test_torque_is_not_converted(self):
        """Nm 은 양쪽이 같은 단위임."""
        _, _, _, effort = convert.observation_to_arrays(
            {"knee.pos": 0.0, "knee.torque": 1.75}, ["knee"]
        )
        assert effort[0] == pytest.approx(1.75)

    def test_missing_fields_become_zero_not_short_arrays(self):
        """JointState 는 세 배열 길이가 같아야 함. 짧으면 받는 쪽이 짝을 못 맞춤."""
        names, position, velocity, effort = convert.observation_to_arrays(
            {"knee.pos": 10.0}, ["knee", "hip_pitch"]
        )
        assert len(names) == len(position) == len(velocity) == len(effort) == 2
        assert position[1] == 0.0

    def test_command_comes_in_as_radians(self):
        action = convert.command_to_action(["right_leg__knee"], [math.pi / 2])
        assert action == {"right_leg/knee": pytest.approx(90.0)}

    def test_command_round_trips_through_the_default_pose(self):
        """기준 자세를 내보냈다 받으면 같은 값이어야 함."""
        names = list(stand.ORDER)
        observation = {f"{n}.pos": stand.DEFAULT_POSE_DEG[n] for n in names}
        ros_names, position, _, _ = convert.observation_to_arrays(observation, names)
        back = convert.command_to_action(ros_names, position)
        for name in names:
            assert back[name] == pytest.approx(stand.DEFAULT_POSE_DEG[name], abs=1e-9)

    def test_mismatched_lengths_are_refused(self):
        """zip 은 짧은 쪽에 맞춰 조용히 자름. 관절 하나가 빠진 명령이 통과함."""
        with pytest.raises(ValueError):
            convert.command_to_action(["a", "b"], [0.0])


# ===========================================================================
# 관절 이름 뽑기
# ===========================================================================
class TestJointNames:
    def test_scalar_entries_are_not_joints(self):
        """`stale_motors` 는 관절이 아님."""
        observation = {"knee.pos": 0.0, "knee.vel": 0.0, "stale_motors": 2}
        assert convert.joint_names(observation) == ("knee",)

    def test_order_follows_the_observation(self):
        observation = {f"{n}.pos": 0.0 for n in stand.ORDER}
        assert convert.joint_names(observation) == stand.ORDER


# ===========================================================================
# 다리
# ===========================================================================
class TestBridge:
    def test_no_command_sends_nothing(self):
        """`None` 을 내면 그 주기는 아무것도 안 나감 (ControlLoop.step)."""
        bridge = CommandBridge(clock=Clock())
        assert bridge.motion(0.0, {"knee.pos": 0.0}) is None

    def test_a_fresh_command_goes_through(self):
        bridge = CommandBridge(clock=Clock())
        bridge.put_command({"knee": 30.0})
        assert bridge.motion(0.0, {}) == {"knee": 30.0}

    def test_a_stale_command_is_dropped(self):
        """명령 보내던 노드가 죽으면 토픽이 그냥 안 옴. 에러가 안 남."""
        clock = Clock()
        bridge = CommandBridge(stale_s=0.2, clock=clock)
        bridge.put_command({"knee": 30.0})

        clock.t = 0.19
        assert bridge.motion(0.0, {}) == {"knee": 30.0}

        clock.t = 0.21
        assert bridge.motion(0.0, {}) is None
        assert bridge.stale_drops == 1

    def test_a_new_command_revives_it(self):
        clock = Clock()
        bridge = CommandBridge(stale_s=0.2, clock=clock)
        bridge.put_command({"knee": 30.0})
        clock.t = 0.5
        assert bridge.motion(0.0, {}) is None

        bridge.put_command({"knee": 40.0})
        assert bridge.motion(0.0, {}) == {"knee": 40.0}

    def test_clearing_stops_the_command(self):
        """토크를 끊을 때 안 비우면 다음에 켤 때 옛 목표가 한 주기 나감."""
        bridge = CommandBridge(clock=Clock())
        bridge.put_command({"knee": 30.0})
        bridge.clear_command()
        assert bridge.motion(0.0, {}) is None

    def test_the_observation_is_handed_over(self):
        bridge = CommandBridge(clock=Clock())
        bridge.motion(0.0, {"knee.pos": 12.5})
        assert bridge.take_observation() == {"knee.pos": 12.5}

    def test_the_observation_is_a_copy(self):
        """제어 스레드가 들고 있는 사전을 ROS 쪽이 고치면 안 됨."""
        bridge = CommandBridge(clock=Clock())
        source = {"knee.pos": 12.5}
        bridge.motion(0.0, source)
        taken = bridge.take_observation()
        taken["knee.pos"] = 99.0
        assert bridge.take_observation()["knee.pos"] == 12.5

    def test_the_command_is_a_copy(self):
        """부르는 쪽이 들고 있는 사전을 고쳐도 안에 든 것이 안 바뀌어야 함."""
        bridge = CommandBridge(clock=Clock())
        source = {"knee": 30.0}
        bridge.put_command(source)
        source["knee"] = 99.0
        assert bridge.motion(0.0, {}) == {"knee": 30.0}

    def test_observation_age_before_anything(self):
        bridge = CommandBridge(clock=Clock())
        assert bridge.observation_age() == -1.0

    def test_observation_age_grows(self):
        clock = Clock()
        bridge = CommandBridge(clock=clock)
        bridge.motion(0.0, {"knee.pos": 0.0})
        clock.t = 0.3
        assert bridge.observation_age() == pytest.approx(0.3)
