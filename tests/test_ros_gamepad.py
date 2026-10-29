"""패드 조작 노드 — **`rclpy` 가 있어야 돎** (없으면 통째로 건너뜀).

    source /opt/ros/humble/setup.bash
    PYTHONPATH=src python3 -m pytest tests/test_ros_gamepad.py -q

`gamepad_node` 는 `rclpy` 를 import 하므로 `convert`/`freshness` 와 달리 ROS 없는
기계에서는 못 돎. 그래도 시험하는 이유는 여기 있는 판단이 **토크와 정책을 켜는 조건**
이기 때문임.

    데드맨(RB)을 떼면 속도가 0 인가
    RB 없이 Menu 를 누르면 정책이 안 켜지는가
    View 를 짧게 누르면 토크가 안 들어가는가
    /joy 가 끊기면 속도가 0 이 되는가
"""

import time

import pytest

pytest.importorskip("rclpy", reason="ROS 2 가 source 되어 있지 않음")
pytest.importorskip("huphy_ros", reason="ros/huphy_ros 가 경로에 없음")

import rclpy                                            # noqa: E402
from sensor_msgs.msg import Joy                         # noqa: E402

from huphy_ros.gamepad_node import AXES, BUTTONS, GamepadNode   # noqa: E402

WALK = "config/policies/generalized_walk"
SQUAT = "config/policies/huphy_squat"
HOLD_S = 0.2
"""시험에서 쓰는 `enable_hold_s`. 기본 1.0초를 그대로 기다리지 않으려는 것임."""


class Pad:
    """노드 하나 + 합성 `Joy` 메시지. 서비스 호출과 발행은 가로채 목록에 모음."""

    def __init__(self, policy):
        rclpy.init(args=["--ros-args", "-p", f"policy:={policy}",
                         "-p", f"enable_hold_s:={HOLD_S}"])
        self.node = GamepadNode()
        self.calls = []
        self.sent = []
        self.node._call = self.calls.append
        self.node._publish = lambda values: self.sent.append(dict(values))

    def close(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def press(self, *buttons, **axes):
        """버튼과 스틱 상태를 하나 밀어 넣음. 안 적은 것은 놓은 것으로 봄."""
        msg = Joy()
        msg.axes = [0.0] * 8
        for name, value in axes.items():
            msg.axes[AXES[name]] = value
        msg.buttons = [0] * 11
        for name in buttons:
            msg.buttons[BUTTONS[name]] = 1
        self.node._on_joy(msg)

    def tick(self, dt=1.0 / 20.0):
        """한 주기 돌리고 (부른 서비스, 내보낸 명령 값) 을 냄.

        `dt` 는 **직전 주기에서 흐른 것으로 칠 시간**임. `GamepadNode._tick` 이
        `time.monotonic()` 에서 `self.last_tick` 을 빼서 쓰는데, 시험에서는 주기를
        연달아 부르므로 실제로는 수십 마이크로초밖에 안 흐름. 높이 명령이 그 값에
        비례해 움직이므로 실제 주기(20Hz)를 넣어 줌.
        """
        self.node.last_tick = time.monotonic() - dt
        self.calls.clear()
        self.sent.clear()
        self.node._tick()
        return list(self.calls), (self.sent[-1] if self.sent else None)

    def torque_on(self):
        self.node.mode, self.node.mode_at = "control", time.monotonic()


@pytest.fixture
def walk():
    pad = Pad(WALK)
    yield pad
    pad.close()


@pytest.fixture
def squat():
    pad = Pad(SQUAT)
    yield pad
    pad.close()


class TestDeadman:
    """RB 를 떼면 속도가 안 나가야 함."""

    def test_sticks_do_nothing_without_the_deadman(self, walk):
        walk.press(vx=1.0, wz=1.0)
        _, values = walk.tick()
        assert values == {"vx": 0.0, "wz": 0.0}

    def test_held_deadman_scales_the_stick_by_max(self, walk):
        walk.press("deadman", vx=1.0, wz=-0.5)
        _, values = walk.tick()
        assert values["vx"] == pytest.approx(walk.node.axes["vx"].max)
        assert values["wz"] == pytest.approx(-0.5 * walk.node.axes["wz"].max)

    def test_releasing_the_deadman_zeroes_the_speed(self, walk):
        walk.press("deadman", vx=1.0)
        walk.tick()
        walk.press(vx=1.0)
        _, values = walk.tick()
        assert values["vx"] == 0.0

    def test_small_stick_values_are_cut(self, walk):
        walk.press("deadman", vx=walk.node.deadzone / 2.0)
        _, values = walk.tick()
        assert values["vx"] == 0.0

    def test_height_keeps_its_value_when_the_deadman_is_released(self, squat):
        """높이를 0 으로 돌리면 바닥까지 내려가라는 뜻이 됨. 마지막 값을 유지해야 함."""
        squat.press("deadman", h_star=-1.0)
        for _ in range(5):
            squat.tick()
        _, before = squat.tick()
        squat.press(h_star=-1.0)
        _, after = squat.tick()
        assert after["h_star"] == pytest.approx(before["h_star"])

    def test_height_stays_inside_the_contract_range(self, squat):
        low, high = squat.node.axes["h_star"].range
        squat.press("deadman", h_star=-1.0)
        for _ in range(200):
            squat.tick()
        _, values = squat.tick()
        assert values["h_star"] == pytest.approx(low)


class TestStartPolicy:
    """정책 시작은 RB 를 쥐고 있을 때만."""

    def test_menu_alone_does_not_start(self, walk):
        walk.press("start_policy")
        calls, _ = walk.tick()
        assert calls == []

    def test_menu_with_the_deadman_starts(self, walk):
        walk.press("deadman", "start_policy")
        calls, _ = walk.tick()
        assert calls == ["start_policy"]

    def test_holding_menu_sends_once(self, walk):
        walk.press("deadman", "start_policy")
        walk.tick()
        calls, _ = walk.tick()
        assert calls == []

    def test_adding_the_deadman_while_menu_is_held_does_nothing(self, walk):
        """버튼이 안 눌림 -> 눌림 으로 바뀐 주기에만 나감. Menu 를 뗐다 눌러야 함."""
        walk.press("start_policy")
        walk.tick()
        walk.press("deadman", "start_policy")
        calls, _ = walk.tick()
        assert calls == []

        walk.press("deadman")
        walk.tick()
        walk.press("deadman", "start_policy")
        calls, _ = walk.tick()
        assert calls == ["start_policy"]


class TestEnable:
    """토크 넣기는 View 를 `enable_hold_s` 동안 누르고 있을 때만."""

    def test_a_short_press_does_not_enable(self, walk):
        walk.press("enable")
        calls, _ = walk.tick()
        assert calls == []
        walk.press()
        calls, _ = walk.tick()
        assert calls == []

    def test_holding_long_enough_enables_once(self, walk):
        walk.press("enable")
        walk.tick()
        time.sleep(HOLD_S * 1.5)
        calls, _ = walk.tick()
        assert calls == ["enable"]
        calls, _ = walk.tick()
        assert calls == []

    def test_releasing_and_holding_again_enables_again(self, walk):
        walk.press("enable")
        walk.tick()
        time.sleep(HOLD_S * 1.5)
        walk.tick()
        walk.press()
        walk.tick()
        walk.press("enable")
        walk.tick()
        time.sleep(HOLD_S * 1.5)
        calls, _ = walk.tick()
        assert calls == ["enable"]


class TestApproach:
    def test_a_is_refused_while_the_torque_is_off(self, walk):
        walk.press("approach")
        calls, _ = walk.tick()
        assert calls == []

    def test_a_goes_through_once_the_torque_is_on(self, walk):
        walk.torque_on()
        walk.press("approach")
        calls, _ = walk.tick()
        assert calls == ["approach"]

    def test_a_is_refused_when_the_control_status_is_stale(self, walk):
        walk.torque_on()
        walk.node.mode_at -= 10.0
        walk.press("approach")
        calls, _ = walk.tick()
        assert calls == []


class TestLostPad:
    def test_a_silent_joy_topic_zeroes_the_speed(self, walk):
        walk.press("deadman", vx=1.0)
        walk.tick()
        walk.node.joy_at -= 10.0
        _, values = walk.tick()
        assert values["vx"] == 0.0

    def test_a_silent_joy_topic_forgets_the_held_buttons(self, walk):
        """패드가 끊긴 뒤 다시 붙었을 때, 눌린 채였던 버튼이 곧바로 나가면 안 됨."""
        walk.press("deadman", "start_policy")
        walk.tick()
        walk.node.joy_at -= 10.0
        walk.tick()
        assert not any(walk.node.pressed.values())


class TestStop:
    def test_b_asks_the_main_loop_to_shut_down(self, walk):
        walk.press("stop")
        walk.tick()
        assert walk.node.stop_reason is not None
