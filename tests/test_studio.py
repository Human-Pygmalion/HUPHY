"""studio 세션 테스트 -- 하드웨어 없이, 제어 루프 없이 실행됨.

웹 요청을 `submit` 으로 넣고 `before_step` / `motion` 을 손으로 불러 한 주기씩 진행함.
"""

import pytest

from huphy.control import studio as S

KNEE, HIP = "right_leg/knee", "right_leg/hip_pitch"


class FakeRobot:
    def __init__(self):
        self.pos = {KNEE: 20.0, HIP: 1.0}
        self.gains = None
        self.last_sent = {}

    def get_observation(self):
        return {f"{k}.pos": v for k, v in self.pos.items()}

    def set_gains(self, gains):
        self.gains = {m: (g.kp, g.kd) for m, g in gains.items()}


class FakeLoop:
    def __init__(self):
        self.events = []

    def switch_to_control(self):
        self.events.append("control")

    def switch_to_observe(self):
        self.events.append("observe")


@pytest.fixture
def session(tmp_path):
    robot = FakeRobot()
    st = S.Studio(
        robot,
        joints=[KNEE, HIP],
        limits={KNEE: (6.0, 114.0), HIP: (-20.0, 90.0)},
        base_gains={"knee": (220.0, 6.0), "ankle_a": (22.0, 1.4)},
        max_gains={"knee": (5000.0, 100.0), "ankle_a": (5000.0, 100.0)},
        gain_scale=0.5,
        sequences_dir=tmp_path,
    )
    return st, robot, FakeLoop()


def tick(st, loop, t):
    st.before_step(loop, t)
    return st.motion(t, {}) if st.state != S.FREE else None


class TestStates:
    def test_it_starts_free(self, session):
        st, robot, _ = session
        assert st.state == S.FREE
        assert robot.gains["knee"] == (110.0, 3.0)  # 처음 값 = yaml x 배율

    def test_torque_on_holds_the_measured_pose(self, session):
        """켜는 순간 당기는 힘이 없어야 함 -- 목표 = 지금 실측."""
        st, robot, loop = session
        st.submit({"op": "torque_on"})
        assert tick(st, loop, 0.0) == {KNEE: 20.0, HIP: 1.0}
        assert st.state == S.HOLD and loop.events == ["control"]

    def test_moves_need_hold(self, session):
        st, _, loop = session
        st.submit({"op": "move", "pose": {KNEE: 40}, "move_s": 1})
        tick(st, loop, 0.0)
        assert st.state == S.FREE and "거부" in st.message

    def test_a_move_starts_from_the_target_and_arrives(self, session):
        st, robot, loop = session
        st.submit({"op": "torque_on"})
        tick(st, loop, 0.0)
        robot.pos[KNEE] = 15.0                       # 처졌어도 출발은 목표(20)에서
        st.submit({"op": "move", "pose": {KNEE: 40}, "move_s": 1.0})
        tick(st, loop, 1.0)
        assert tick(st, loop, 1.5)[KNEE] == pytest.approx(30.0)
        assert tick(st, loop, 2.0)[KNEE] == pytest.approx(40.0)
        assert st.state == S.HOLD

    def test_outside_the_limits_is_refused(self, session):
        st, _, loop = session
        st.submit({"op": "torque_on"})
        tick(st, loop, 0.0)
        st.submit({"op": "move", "pose": {KNEE: 200}, "move_s": 1.0})
        tick(st, loop, 0.1)
        assert st.state == S.HOLD and "한계 밖" in st.message

    def test_torque_off_goes_free(self, session):
        st, _, loop = session
        st.submit({"op": "torque_on"})
        tick(st, loop, 0.0)
        st.submit({"op": "torque_off"})
        tick(st, loop, 0.1)
        assert st.state == S.FREE and loop.events == ["control", "observe"]


class TestGain:
    """게인은 모터 이름별 kp/kd 값. 양다리 공통."""

    def test_only_in_hold(self, session):
        """움직이는 중에는 게인을 못 바꿈."""
        st, _, loop = session
        st.submit({"op": "torque_on"})
        tick(st, loop, 0.0)
        st.submit({"op": "move", "pose": {KNEE: 40}, "move_s": 2.0})
        tick(st, loop, 0.1)
        st.submit({"op": "gain", "gains": {"knee": {"kp": 300, "kd": 8}}})
        tick(st, loop, 0.2)
        assert st.state == S.MOVING and "HOLD 에서만" in st.message

    def test_it_ramps_to_the_given_values(self, session):
        st, robot, loop = session
        st.submit({"op": "torque_on"})
        tick(st, loop, 0.0)
        st.submit({"op": "gain", "gains": {"knee": {"kp": 310, "kd": 9}}})
        tick(st, loop, 1.0)
        tick(st, loop, 1.0 + S.RAMP_S / 2)
        assert robot.gains["knee"] == pytest.approx((210.0, 6.0))
        tick(st, loop, 1.0 + S.RAMP_S)
        assert robot.gains["knee"] == pytest.approx((310.0, 9.0))
        assert robot.gains["ankle_a"] == pytest.approx((11.0, 0.7))   # 안 준 것은 그대로
        assert st.state == S.HOLD

    def test_above_yaml_is_allowed(self, session):
        """yaml 값은 기준일 뿐 상한이 아님."""
        st, robot, loop = session
        st.submit({"op": "gain", "gains": {"knee": {"kp": 1000, "kd": 20}}})
        tick(st, loop, 0.0)
        assert robot.gains["knee"] == (1000.0, 20.0)

    def test_free_sets_at_once(self, session):
        """힘이 없으니 바로 바꿔 둠 -- 토크를 켜면 그 값으로."""
        st, robot, loop = session
        st.submit({"op": "gain", "gains": {"knee": {"kp": 50, "kd": 1}}})
        tick(st, loop, 0.0)
        assert robot.gains["knee"] == (50.0, 1.0) and st.state == S.FREE

    def test_above_the_encoding_is_refused(self, session):
        """MIT 인코딩 최댓값을 넘으면 보내는 숫자가 잘림."""
        st, robot, loop = session
        st.submit({"op": "gain", "gains": {"knee": {"kp": 9000, "kd": 6}}})
        tick(st, loop, 0.0)
        assert robot.gains["knee"] == (110.0, 3.0) and "범위 밖" in st.message

    def test_an_unknown_motor_is_refused(self, session):
        st, robot, loop = session
        st.submit({"op": "gain", "gains": {"elbow": {"kp": 10, "kd": 1}}})
        tick(st, loop, 0.0)
        assert "모르는 모터" in st.message


class TestPlay:
    def test_frames_in_order_then_hold(self, session):
        st, _, loop = session
        st.submit({"op": "torque_on"})
        tick(st, loop, 0.0)
        frames = [
            {"pose": {KNEE: 40}, "move_s": 1.0, "hold_s": 0.5},
            {"pose": {KNEE: 30}, "move_s": 1.0},
        ]
        st.submit({"op": "play", "keyframes": frames, "repeat": 1})
        tick(st, loop, 0.0)
        assert tick(st, loop, 1.0)[KNEE] == pytest.approx(40.0)
        assert tick(st, loop, 1.4)[KNEE] == pytest.approx(40.0)   # 머무름
        tick(st, loop, 1.5)
        assert tick(st, loop, 2.0)[KNEE] == pytest.approx(35.0)
        tick(st, loop, 2.5)
        assert st.state == S.HOLD and st.target[KNEE] == pytest.approx(30.0)

    def test_the_watchdog_holds_when_the_screen_is_gone(self, session, monkeypatch):
        st, _, loop = session
        st.submit({"op": "torque_on"})
        tick(st, loop, 0.0)
        st.submit({"op": "play", "keyframes": [{"pose": {KNEE: 40}, "move_s": 5.0}], "repeat": 0})
        tick(st, loop, 0.1)
        monkeypatch.setattr(st, "_seen", st._seen - S.WATCHDOG_S - 1)
        tick(st, loop, 0.2)
        assert st.state == S.HOLD and "화면" in st.message


class TestSequences:
    def test_save_and_load(self, session):
        st, _, _ = session
        st.save_sequence("wave_1", {"repeat": 0, "keyframes": [{"pose": {KNEE: 40}, "move_s": 2}]})
        assert st.list_sequences() == ["wave_1"]
        data = st.load_sequence("wave_1")
        assert data["ankle_space"] == "ab" and data["keyframes"][0]["pose"] == {KNEE: 40.0}

    def test_a_path_is_not_a_name(self, session):
        st, _, _ = session
        with pytest.raises(ValueError):
            st.save_sequence("../x", {"keyframes": [{"pose": {KNEE: 40}, "move_s": 2}]})

    def test_a_bad_pose_is_not_saved(self, session):
        st, _, _ = session
        with pytest.raises(ValueError, match="한계 밖"):
            st.save_sequence("bad", {"keyframes": [{"pose": {KNEE: 500}, "move_s": 2}]})
        assert st.list_sequences() == []
