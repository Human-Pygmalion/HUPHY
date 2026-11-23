"""명령 입력 노드 — **`rclpy` 가 있어야 돎** (없으면 통째로 건너뜀).

    source /opt/ros/humble/setup.bash
    PYTHONPATH=ros/huphy_ros:src python3 -m pytest tests/test_ros_cmd.py -q

창(`curses`)은 안 띄움. `CmdNode.key` 에 키 이름을 넣고 `lines`·발행 값을 봄 --
`run_ui` 가 하는 일이 키 번호를 그 이름으로 바꿔 넣는 것뿐임.

디버깅용 노드지만 **친 값이 그대로 로봇으로 가는 길**이라 걸러 내는 자리를 시험함.

    Enter 전에는 값이 안 바뀌는가 (치는 중에도 전 값이 나가야 함)
    학습 범위 밖의 값을 막는가
    Space 가 속도만 0 으로 하고 높이는 그대로 두는가
    0 으로 시작하는 값(0.3)을 칠 수 있는가 -- Space 를 쓰는 이유임
"""

import pytest

pytest.importorskip("rclpy", reason="ROS 2 가 source 되어 있지 않음")
pytest.importorskip("huphy_ros", reason="ros/huphy_ros 가 경로에 없음")

import rclpy                                        # noqa: E402

from huphy_ros.cmd_node import CmdNode, _key_name   # noqa: E402

NEWWALK = "config/policies/newwalk"
SQUAT = "config/policies/squat"


class Typed:
    """노드 하나. 발행과 서비스 호출은 가로채 목록에 모음."""

    def __init__(self, policy):
        rclpy.init(args=["--ros-args", "-p", f"policy:={policy}"])
        self.node = CmdNode()
        self.calls = []
        self.sent = []
        self.node._call = lambda name: self.calls.append(name)
        self.node._publish = lambda values: self.sent.append(dict(values))

    def close(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def type(self, *keys):
        """키를 차례로 넣고 한 주기 돌림. (부른 서비스, 내보낸 값) 을 냄."""
        self.calls.clear()
        self.sent.clear()
        for key in keys:
            self.node.key(key)
        self.node._tick()
        return list(self.calls), (self.sent[-1] if self.sent else None)

    def number(self, text, enter=True):
        """`"0.3"` 처럼 숫자를 한 글자씩 침. `enter=False` 면 치는 중인 상태로 둠."""
        keys = list(text) + (["enter"] if enter else [])
        return self.type(*keys)


@pytest.fixture
def walk():
    t = Typed(NEWWALK)
    yield t
    t.close()


@pytest.fixture
def squat():
    t = Typed(SQUAT)
    yield t
    t.close()


class TestTyping:
    def test_a_typed_number_lands_on_enter(self, walk):
        _, values = walk.number("0.3")
        assert values == {"vx": 0.3, "vy": 0.0, "wz": 0.0}

    def test_nothing_changes_before_enter(self, walk):
        """치는 중에도 전에 적용된 값이 계속 나가야 함."""
        _, values = walk.number("0.3", enter=False)
        assert values["vx"] == 0.0
        assert walk.node.buffer == "0.3"

    def test_a_number_starting_with_zero_is_typable(self, walk):
        """`0` 을 '전부 0' 으로 쓰지 않는 이유. 첫 글자가 급정지면 0.3 을 못 침."""
        walk.number("0.3", enter=False)
        assert walk.node.buffer == "0.3"

    def test_three_decimals_go_in_whole(self, walk):
        _, values = walk.number("0.005")
        assert values["vx"] == 0.005

    def test_a_bare_integer_is_a_float(self, walk):
        _, values = walk.number("1")
        assert values["vx"] == 1.0

    def test_a_minus_sign_works(self, walk):
        _, values = walk.number("-0.4")
        assert values["vx"] == -0.4

    def test_backspace_drops_the_last_key(self, walk):
        walk.type(*"0.35", "backspace")
        _, values = walk.type("enter")
        assert values["vx"] == 0.3

    def test_tab_throws_away_what_was_being_typed(self, walk):
        walk.number("0.3", enter=False)
        walk.type("tab")
        assert walk.node.buffer == ""
        assert walk.node.field == "vy"
        _, values = walk.type("enter")
        assert values == {"vx": 0.0, "vy": 0.0, "wz": 0.0}

    def test_tab_wraps_around(self, walk):
        walk.type("tab", "tab", "tab")
        assert walk.node.field == "vx"

    def test_enter_on_an_empty_field_does_nothing(self, walk):
        _, values = walk.type("enter")
        assert values["vx"] == 0.0

    def test_not_a_number_is_refused(self, walk):
        _, values = walk.type("-", "-", "enter")
        assert values["vx"] == 0.0
        assert "숫자가 아님" in walk.node.message


class TestRange:
    def test_a_value_outside_the_trained_range_is_refused(self, walk):
        """`newwalk` 의 vx 는 [-1.5, 1.5]. 보내도 관찰 전에 잘리므로 막음."""
        _, values = walk.number("9")
        assert values["vx"] == 0.0
        assert "학습 범위" in walk.node.message

    def test_the_edge_of_the_range_is_allowed(self, walk):
        _, values = walk.number("1.5")
        assert values["vx"] == 1.5

    def test_the_nudge_stops_at_the_range(self, walk):
        walk.number("1.5")
        _, values = walk.type("k")          # +0.05 -> 1.55, 범위 밖
        assert values["vx"] == 1.5


class TestNudge:
    def test_k_adds_a_step_and_j_takes_one_away(self, walk):
        _, up = walk.type("k")
        assert up["vx"] == pytest.approx(walk.node.step)
        _, down = walk.type("j", "j")
        assert down["vx"] == pytest.approx(-walk.node.step)

    def test_the_nudge_needs_no_enter(self, walk):
        _, values = walk.type("k")
        assert values["vx"] == pytest.approx(walk.node.step)

    def test_the_nudge_throws_away_what_was_being_typed(self, walk):
        walk.number("1.2", enter=False)
        walk.type("k")
        assert walk.node.buffer == ""

    def test_height_has_its_own_step(self, squat):
        """`squat` 의 기본 높이 0.8914 는 범위 [0.65, 0.895] 의 거의 천장임 -- 내려 봄."""
        before = squat.node.values["h_star"]
        _, values = squat.type("j")
        assert values["h_star"] == pytest.approx(before - squat.node.height_step)
        assert squat.node.height_step != squat.node.step

    def test_the_nudge_follows_the_focused_field(self, walk):
        walk.type("tab", "k")
        assert walk.node.values["vy"] == pytest.approx(walk.node.step)
        assert walk.node.values["vx"] == 0.0


class TestSpace:
    def test_space_zeroes_every_speed(self, walk):
        walk.number("0.3")
        walk.type("tab")
        walk.number("0.2")
        _, values = walk.type("space")
        assert values == {"vx": 0.0, "vy": 0.0, "wz": 0.0}

    def test_space_leaves_the_height_alone(self, squat):
        """높이를 0 으로 돌리면 바닥까지 내려가라는 뜻이 됨."""
        _, before = squat.number("0.8")
        _, after = squat.type("space")
        assert after["h_star"] == before["h_star"] == 0.8


class TestServices:
    @pytest.mark.parametrize("key,name", [
        ("e", "enable"),
        ("r", "approach"),
        ("s", "start_policy"),
    ])
    def test_a_letter_calls_its_service(self, walk, key, name):
        calls, _ = walk.type(key)
        assert calls == [name]

    def test_a_service_key_does_not_change_the_values(self, walk):
        walk.number("0.3")
        _, values = walk.type("s")
        assert values["vx"] == 0.3

    def test_an_unused_key_only_leaves_a_message(self, walk):
        calls, values = walk.type("z")
        assert calls == []
        assert values["vx"] == 0.0
        assert "쓰는 키가 아님" in walk.node.message

    def test_finish_zeroes_the_speed_and_stops_both(self, walk):
        walk.number("0.3")
        walk.calls.clear()
        walk.node.finish()
        assert walk.calls == ["stop_policy", "disable"]
        assert walk.node.values["vx"] == 0.0
        assert walk.node.finished


class TestLines:
    def test_the_focused_field_is_marked(self, walk):
        walk.type("tab")
        focused = [line for line in walk.node.lines() if line.startswith(" >")]
        assert len(focused) == 1
        assert "vy" in focused[0]

    def test_what_is_being_typed_shows_next_to_the_value(self, walk):
        walk.number("1.05", enter=False)
        line = next(line for line in walk.node.lines() if line.startswith(" >"))
        assert "0.000" in line and "< 1.05_" in line

    def test_the_value_is_drawn_with_three_decimals(self, walk):
        walk.number("1")
        assert any("1.000" in line for line in walk.node.lines())

    def test_a_missing_status_topic_shows_as_none(self, walk):
        line = next(line for line in walk.node.lines() if "control_node" in line)
        assert "없음" in line

    def test_the_last_message_is_the_bottom_line(self, walk):
        walk.number("9")
        assert walk.node.lines()[-1].strip() == walk.node.message

    def test_the_step_shown_is_the_focused_field(self, squat):
        """높이 칸은 0.01 로 움직임. 속도 폭(0.05)을 띄우면 안 됨."""
        line = next(line for line in squat.node.lines() if "j/k" in line)
        assert f"∓{squat.node.height_step:g}" in line

    def test_a_policy_without_speed_has_no_space_line(self, squat):
        assert not any("Space" in line for line in squat.node.lines())


class TestKeyNames:
    """`curses` 의 키 번호 -> `key` 가 받는 이름."""

    def test_the_numbers_map_to_names(self):
        curses = pytest.importorskip("curses")
        assert _key_name(9, curses) == "tab"
        assert _key_name(10, curses) == "enter"
        assert _key_name(127, curses) == "backspace"
        assert _key_name(32, curses) == "space"
        assert _key_name(ord("j"), curses) == "j"

    def test_keys_we_do_not_use_are_dropped(self):
        curses = pytest.importorskip("curses")
        assert _key_name(-1, curses) is None          # 아무 키도 안 눌림
        assert _key_name(curses.KEY_LEFT, curses) is None
