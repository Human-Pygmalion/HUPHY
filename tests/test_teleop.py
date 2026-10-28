"""pygame 조작(`control/teleop.py`) 테스트 — 화면 없이 돎.

    PYTHONPATH=src python3 -m pytest tests/test_teleop.py -q

계산(`TeleopState`)과 키 처리(`Keys`)는 pygame 없이 시험함. 프로세스 시험은 pygame 이
있을 때만, SDL 더미 화면으로 돌림.
"""

import time
from pathlib import Path

import pytest

from huphy.control import deploy, teleop

POLICIES = Path(__file__).resolve().parent.parent / "config" / "policies"
WALK = deploy.load(POLICIES / "generalized_walk")
SQUAT = deploy.load(POLICIES / "huphy_squat")


def walk_state(**overrides):
    axes = teleop.make_axes(WALK.commands.values(), overrides)
    return teleop.TeleopState(axes)


def run_for(state, held, seconds, dt=0.01):
    for _ in range(int(round(seconds / dt))):
        values = state.update(set(held), dt)
    return values


# ===========================================================================
# 축 만들기
# ===========================================================================
class TestAxes:
    def test_walk_gets_velocity_axes_with_defaults(self):
        axes = {a.name: a for a in teleop.make_axes(WALK.commands.values())}
        assert set(axes) == {"vx", "wz"}
        assert axes["vx"].keys == ("w", "s") and axes["wz"].keys == ("a", "d")
        assert (axes["vx"].max, axes["vx"].accel, axes["vx"].decel) == (1.0, 0.5, 1.0)

    def test_overrides(self):
        axes = {a.name: a for a in teleop.make_axes(
            WALK.commands.values(), {"max": {"vx": 0.8}, "accel": {"wz": 2.0}})}
        assert axes["vx"].max == 0.8
        assert axes["wz"].accel == 2.0

    def test_squat_gets_a_height_axis(self):
        (axis,) = teleop.make_axes(SQUAT.commands.values())
        assert axis.kind == "height" and axis.keys == ("up", "down")
        assert axis.range == (0.65, 0.895) and axis.start == pytest.approx(0.8914)

    def test_unknown_name_is_refused(self):
        with pytest.raises(teleop.TeleopError, match="이 정책에 없는 명령"):
            teleop.make_axes(WALK.commands.values(), {"max": {"h_star": 1.0}})

    @pytest.mark.parametrize("value", [0.0, -1.0])
    def test_non_positive_is_refused(self, value):
        with pytest.raises(teleop.TeleopError, match="0 보다 커야"):
            teleop.make_axes(WALK.commands.values(), {"accel": {"vx": value}})

    def test_height_takes_only_accel(self):
        with pytest.raises(teleop.TeleopError, match="--accel 만"):
            teleop.make_axes(SQUAT.commands.values(), {"max": {"h_star": 0.8}})


# ===========================================================================
# 속도 — 누르는 동안 적분, 떼면 감속해 0, 상한
# ===========================================================================
class TestVelocity:
    def test_holding_integrates_at_accel(self):
        values = run_for(walk_state(), {"w"}, 0.4)
        assert values["vx"] == pytest.approx(0.5 * 0.4)

    def test_it_stops_at_max(self):
        values = run_for(walk_state(), {"w"}, 10.0)
        assert values["vx"] == pytest.approx(1.0)

    def test_backward(self):
        values = run_for(walk_state(), {"s"}, 0.4)
        assert values["vx"] == pytest.approx(-0.2)

    def test_release_decays_to_exactly_zero(self):
        """1.0 m/s 에서 감속 1.0 m/s² 이면 1초 뒤 정확히 0, 그 뒤로도 0."""
        state = walk_state()
        run_for(state, {"w"}, 10.0)
        assert run_for(state, set(), 0.5)["vx"] == pytest.approx(0.5)
        assert run_for(state, set(), 0.5)["vx"] == pytest.approx(0.0, abs=1e-9)
        assert run_for(state, set(), 1.0)["vx"] == 0.0

    def test_reversing_brakes_with_decel(self):
        """반대 키는 decel(1.0) 로 멈춘 뒤 accel(0.5) 로 가속."""
        state = walk_state()
        run_for(state, {"w"}, 10.0)                         # +1.0
        assert run_for(state, {"s"}, 1.0)["vx"] == pytest.approx(0.0, abs=1e-6)
        assert run_for(state, {"s"}, 0.2)["vx"] == pytest.approx(-0.1, abs=1e-6)

    def test_both_axes_at_once(self):
        """W 와 A 를 같이 누르면 앞으로 가면서 돎."""
        values = run_for(walk_state(), {"w", "a"}, 0.4)
        assert values["vx"] == pytest.approx(0.2) and values["wz"] == pytest.approx(0.2)

    def test_settings_change_the_dynamics(self):
        state = walk_state(max={"vx": 1.0}, accel={"vx": 2.0}, decel={"vx": 4.0})
        assert run_for(state, {"w"}, 0.25)["vx"] == pytest.approx(0.5)
        assert run_for(state, {"w"}, 5.0)["vx"] == pytest.approx(1.0)
        assert run_for(state, set(), 0.25)["vx"] == pytest.approx(0.0, abs=1e-9)

    def test_space_stops_at_once(self):
        state = walk_state()
        keys = teleop.Keys(state)
        keys.handle("down", "w")
        run_for(state, keys.held, 5.0)
        keys.handle("down", "space")
        assert state.values["vx"] == 0.0 and keys.held == set()


# ===========================================================================
# 높이
# ===========================================================================
class TestHeight:
    def _state(self):
        return teleop.TeleopState(teleop.make_axes(SQUAT.commands.values()))

    def test_down_moves_at_the_rate_and_stays_after_release(self):
        state = self._state()
        assert run_for(state, {"down"}, 1.0)["h_star"] == pytest.approx(0.8914 - 0.05)
        assert run_for(state, set(), 1.0)["h_star"] == pytest.approx(0.8914 - 0.05)

    def test_it_stays_in_the_training_range(self):
        state = self._state()
        assert run_for(state, {"down"}, 10.0)["h_star"] == pytest.approx(0.65)
        assert run_for(state, {"up"}, 10.0)["h_star"] == pytest.approx(0.895)

    def test_space_does_not_touch_height(self):
        state = self._state()
        run_for(state, {"down"}, 1.0)
        state.stop_velocities()
        assert state.values["h_star"] == pytest.approx(0.8414)


# ===========================================================================
# 키 이벤트
# ===========================================================================
class TestKeys:
    def test_down_and_up(self):
        keys = teleop.Keys(walk_state())
        keys.handle("down", "w")
        keys.handle("down", "a")
        keys.handle("up", "w")
        assert keys.held == {"a"}

    def test_focus_lost_releases_everything(self):
        keys = teleop.Keys(walk_state())
        keys.handle("down", "w")
        keys.handle("focus_lost")
        assert keys.held == set()

    @pytest.mark.parametrize("kind, name, action", [
        ("down", "return", "start"), ("down", "enter", "start"),
        ("down", "escape", "stop"), ("quit", "", "stop"), ("down", "w", None),
    ])
    def test_actions(self, kind, name, action):
        assert teleop.Keys(walk_state()).handle(kind, name) == action

    @pytest.mark.parametrize("name", ["c", "q"])
    def test_ctrl_c_and_ctrl_q_in_the_window_stop(self, name):
        """창이 선택돼 있으면 Ctrl-C/Ctrl-Q 가 터미널로 안 감. 창에서도 멈춰야 함."""
        keys = teleop.Keys(walk_state())
        assert keys.handle("down", name, ctrl=True) == "stop"
        assert keys.held == set()

    def test_c_without_ctrl_is_just_a_key(self):
        assert teleop.Keys(walk_state()).handle("down", "c") is None


# ===========================================================================
# 제어 루프 쪽
# ===========================================================================
class TestCommandsSource:
    def test_commands_read_the_source_every_step(self):
        cmds = deploy.Commands(WALK.commands.values())
        feed = {"vx": 0.3, "wz": -0.1}
        cmds.source = lambda: feed
        assert cmds.step(0.02) == {"vx": 0.3, "wz": -0.1}
        feed["vx"] = 0.0
        assert cmds.step(0.02)["vx"] == 0.0

    def test_h_star_from_the_source_still_ramps(self):
        cmds = deploy.Commands(SQUAT.commands.values())
        cmds.source = lambda: {"h_star": 0.70}
        assert cmds.step(0.02)["h_star"] == pytest.approx(0.8914 - 0.002)


class TestTeleopMotion:
    class Loop:
        stopped = False

        def stop(self):
            self.stopped = True

    class Pad:
        def __init__(self, start=False, stop=False):
            self._start, self._stop = start, stop

        def start_requested(self):
            return self._start

        def stop_requested(self):
            return self._stop

    def _staged(self):
        from huphy.scripts import run
        return run.staged(lambda t, o: {}, 0.0, lambda t, o: {"p": 1}, hold_pose={})

    def test_enter_in_the_window_starts_the_policy(self):
        staged, loop = self._staged(), self.Loop()
        wrapped = teleop.teleop_motion(staged, self.Pad(start=True), loop, staged)
        wrapped(0.0, {})
        assert staged.is_started() and not loop.stopped

    def test_escape_in_the_window_stops_the_loop(self):
        staged, loop = self._staged(), self.Loop()
        teleop.teleop_motion(staged, self.Pad(stop=True), loop, staged)(0.0, {})
        assert loop.stopped and not staged.is_started()


# ===========================================================================
# 진짜 pygame 프로세스 (더미 화면)
# ===========================================================================
@pytest.fixture
def dummy_display(monkeypatch):
    pytest.importorskip("pygame")
    monkeypatch.setenv("SDL_VIDEODRIVER", "dummy")


class TestProcess:
    def test_it_starts_and_feeds_values(self, dummy_display):
        pad = teleop.Teleop(teleop.make_axes(WALK.commands.values())).start()
        try:
            time.sleep(0.1)
            assert pad.age() < teleop.STALE_S
            assert pad.targets() == {"vx": 0.0, "wz": 0.0}
            assert not pad.start_requested() and not pad.stop_requested()
        finally:
            pad.close()
        assert pad.process is None

    def test_a_dead_process_means_zero_velocity(self, dummy_display):
        pad = teleop.Teleop(teleop.make_axes(WALK.commands.values())).start()
        try:
            pad.shared[0] = 0.4                     # vx 가 0.4 인 채로
            pad.process.kill()
            pad.process.join()
            time.sleep(teleop.STALE_S + 0.05)
            assert pad.targets()["vx"] == 0.0
        finally:
            pad.close()

    def test_a_dead_process_keeps_the_last_height(self, dummy_display):
        pad = teleop.Teleop(teleop.make_axes(SQUAT.commands.values()),
                            start={"h_star": 0.8914}).start()
        try:
            time.sleep(0.05)
            assert pad.targets()["h_star"] == pytest.approx(0.8914)
            pad.process.kill()
            pad.process.join()
            time.sleep(teleop.STALE_S + 0.05)
            assert pad.targets()["h_star"] == pytest.approx(0.8914)
        finally:
            pad.close()

    def test_terminate_works(self, dummy_display):
        """SDL 이 SIGTERM 을 가로채면 안 죽음 -- SDL_NO_SIGNAL_HANDLERS."""
        pad = teleop.Teleop(teleop.make_axes(WALK.commands.values())).start()
        process = pad.process
        process.terminate()
        process.join(timeout=3.0)
        assert not process.is_alive()
        pad.close()

    def test_no_display_is_an_error_before_the_motors(self, monkeypatch):
        pytest.importorskip("pygame")
        monkeypatch.setenv("SDL_VIDEODRIVER", "no_such_driver")
        with pytest.raises(teleop.TeleopError, match="pygame 창을 못 띄움"):
            teleop.Teleop(teleop.make_axes(WALK.commands.values())).start(timeout=5.0)
