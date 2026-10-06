"""제어 루프와 동작 테스트 — 하드웨어 없이 실행됨.

    PYTHONPATH=src python3 -m pytest tests -q

동작은 순수 함수라 그냥 부름. 루프는 가짜 로봇으로 **순서와 안전**을 봄 —
실제 주기 정밀도는 `time.sleep` 정밀도에 달려 있어 여기서 확인되지 않음.
"""

import pytest

import time

from huphy.config.schema import SafetyConfig
from huphy.control import ControlLoop, LoopStats, Mode, motions
from huphy.control.loop import precise_sleep


# ===========================================================================
# 가짜 로봇
# ===========================================================================
class FakeRobot:
    """`Robot` 계약 중 루프가 쓰는 부분만 채움. 무슨 순서로 불렸는지 기록함."""

    name = "leg"

    def __init__(self, robot_id="right_leg", missing=()):
        self.id = robot_id
        self.log = []
        self.connected = False
        self.torque = False
        self._missing = tuple(missing)
        self._pos = {"knee": 0.0, "hip_pitch": 0.0}
        self.last_sent = {}
        self.sent_actions = []

    # 수명
    @property
    def is_connected(self):
        return self.connected

    def connect(self):
        self.connected = True
        self.log.append("connect")

    def disconnect(self):
        self.connected = False
        self.log.append("disconnect")

    def enable(self):
        self.torque = True
        self.log.append("enable")

    def disable(self):
        self.torque = False
        self.log.append("disable")

    # 관찰·명령
    def get_observation(self):
        return {f"{j}.pos": v for j, v in self._pos.items()}

    def build_commands(self, action):
        self.sent_actions.append(dict(action))
        self.last_sent = dict(action)
        return {10: action}

    def send(self, commands):
        self.log.append("send")
        for joint, value in self.last_sent.items():
            self._pos[joint] = value
        return len(commands)

    def collect(self):
        self.log.append("collect")
        return self._missing

    def refresh(self):
        self.log.append("refresh")
        return self._missing

    def hold(self):
        self.log.append("hold")
        return {10: "hold"}

    def damp(self):
        self.log.append("damp")
        return {10: "damp"}


class FakeTelemetry:
    def __init__(self):
        self.rows = []
        self.opened = 0
        self.closed = 0

    def open(self):
        self.opened += 1

    def close(self):
        self.closed += 1

    def record(self, *, loop_dt_ms=0.0):
        self.rows.append(loop_dt_ms)
        return {}


@pytest.fixture
def robot():
    return FakeRobot()


# ===========================================================================
# 동작
# ===========================================================================
class TestMotions:
    def test_hold_is_constant(self):
        m = motions.hold({"knee": 30.0})
        assert m(0.0, {}) == {"knee": 30.0}
        assert m(99.0, {}) == {"knee": 30.0}

    def test_hold_returns_a_copy(self):
        """호출부가 반환값을 고쳐도 다음 주기에 영향이 없어야 함."""
        m = motions.hold({"knee": 30.0})
        m(0.0, {})["knee"] = 999.0
        assert m(1.0, {})["knee"] == 30.0

    def test_freeze_captures_the_first_reading(self):
        """토크를 넣는 순간 관절이 튀지 않게 함.

        목표를 0으로 잡고 시작하면 다리가 0을 향해 한 번에 움직임.
        """
        m = motions.freeze(["knee"])
        assert m(0.0, {"knee.pos": 12.5}) == {"knee": 12.5}
        assert m(1.0, {"knee.pos": 40.0}) == {"knee": 12.5}   # 처음 값을 계속 씀

    def test_freeze_waits_for_a_reading(self):
        """상태를 못 받았으면 그 주기는 명령하지 않음."""
        assert motions.freeze(["knee"])(0.0, {}) is None

    def test_freeze_keeps_the_joints_it_could_read(self):
        """관찰에 없는 관절 하나 때문에 전부를 버리지 않음.

        전부를 버리면 토크가 켜진 채 명령이 한 개도 안 나가고, 그때 모터는 자기
        내부 목표를 붙잡으므로 다리가 그쪽으로 움직임.
        """
        m = motions.freeze(["knee", "ankle_pitch"])
        assert m(0.0, {"knee.pos": 12.5}) == {"knee": 12.5}

    def test_freeze_holds_what_it_captured_first(self):
        """뒤늦게 나타난 관절을 나중에 끼워 넣지 않음. 목표가 도중에 바뀌면 튐."""
        m = motions.freeze(["knee", "ankle_pitch"])
        m(0.0, {"knee.pos": 12.5})
        assert m(1.0, {"knee.pos": 12.5, "ankle_pitch.pos": 3.0}) == {"knee": 12.5}

    def test_step_jumps_at_the_given_time(self):
        m = motions.step("knee", start=0.0, end=30.0, at_s=1.0)
        assert m(0.5, {})["knee"] == 0.0
        assert m(1.0, {})["knee"] == 30.0
        assert m(5.0, {})["knee"] == 30.0

    def test_step_holds_the_others(self):
        """한 관절만 흔들며 나머지는 붙잡아 둠. 아니면 원인이 섞임."""
        m = motions.step("knee", start=0.0, end=30.0, hold_others={"hip_pitch": -50.0})
        assert m(0.0, {})["hip_pitch"] == -50.0

    def test_sine_starts_at_the_center(self):
        """토크를 넣는 순간 튀지 않게 함."""
        m = motions.sine("knee", center=10.0, amplitude=5.0, hz=1.0)
        assert m(0.0, {})["knee"] == pytest.approx(10.0)

    def test_sine_reaches_the_amplitude(self):
        m = motions.sine("knee", center=0.0, amplitude=5.0, hz=1.0)
        assert m(0.25, {})["knee"] == pytest.approx(5.0)      # 1/4 주기
        assert m(0.75, {})["knee"] == pytest.approx(-5.0)

    def test_ramp_is_linear_then_holds(self):
        m = motions.ramp("knee", start=0.0, end=10.0, seconds=2.0)
        assert m(0.0, {})["knee"] == pytest.approx(0.0)
        assert m(1.0, {})["knee"] == pytest.approx(5.0)
        assert m(2.0, {})["knee"] == pytest.approx(10.0)
        assert m(9.0, {})["knee"] == pytest.approx(10.0)

    def test_ramp_rejects_zero_seconds(self):
        with pytest.raises(ValueError, match="0보다 커야 함"):
            motions.ramp("knee", start=0.0, end=10.0, seconds=0.0)

    def test_chain_restarts_time_each_segment(self):
        """각 동작이 자기 구간의 0초부터 받음. 이어 붙일 때 시간 계산을 안 해도 됨."""
        m = motions.chain(
            (motions.ramp("knee", start=0.0, end=10.0, seconds=2.0), 2.0),
            (motions.step("knee", start=100.0, end=200.0, at_s=1.0), 2.0),
        )
        assert m(1.0, {})["knee"] == pytest.approx(5.0)     # 첫 구간 절반
        assert m(2.5, {})["knee"] == pytest.approx(100.0)   # 둘째 구간 0.5초
        assert m(3.5, {})["knee"] == pytest.approx(200.0)   # 둘째 구간 1.5초

    def test_chain_ends_with_none(self):
        """궤적이 끝나면 명령을 멈춤. 마지막 값을 계속 보내지 않음."""
        m = motions.chain((motions.hold({"knee": 1.0}), 1.0))
        assert m(0.5, {}) is not None
        assert m(1.5, {}) is None

    def test_motions_do_not_touch_hardware(self):
        """전부 순수 함수임. 하드웨어 없이 시험됨."""
        assert motions.sine("knee")(0.3, {}) is not None


# ===========================================================================
# 모드
# ===========================================================================
class TestMode:
    def test_observe_cuts_torque_on_entry(self, robot):
        """설정이 잘못돼 있어도 다리가 안 움직이게 하는 하드 안전장치임."""
        ControlLoop(robot, hz=1000.0, mode=Mode.OBSERVE).run(max_cycles=2)
        assert robot.log[:2] == ["connect", "disable"]
        assert "enable" not in robot.log

    def test_observe_still_reads(self, robot):
        """아무것도 보내지 않으면 아무것도 오지 않음.

        MIT 모드에는 읽기 전용 명령이 없어서, 힘이 나가지 않는 명령을 보내고 그
        응답을 받음.
        """
        ControlLoop(robot, hz=1000.0, mode=Mode.OBSERVE).run(max_cycles=3)
        assert robot.log.count("refresh") == 3
        assert "send" not in robot.log

    def test_observe_ignores_the_motion(self, robot):
        ControlLoop(robot, hz=1000.0, mode=Mode.OBSERVE).run(
            motions.hold({"knee": 30.0}), max_cycles=2
        )
        assert robot.sent_actions == []

    def test_control_enables_torque(self, robot):
        ControlLoop(robot, hz=1000.0, mode=Mode.CONTROL).run(
            motions.hold({"knee": 1.0}), max_cycles=2
        )
        assert robot.log[:2] == ["connect", "enable"]

    def test_control_sends_then_collects(self, robot):
        """계산·전송·수거 순서를 지킴 (이슈 #10)."""
        ControlLoop(robot, hz=1000.0, mode=Mode.CONTROL).run(
            motions.hold({"knee": 1.0}), max_cycles=1
        )
        assert robot.log.index("send") < robot.log.index("collect")

    def test_none_action_sends_nothing(self, robot):
        """궤적이 끝났거나 아직 시작 전일 때 명령을 멈춤.

        종료할 때의 감쇠는 별개임 — 실행 구간만 봄.
        """
        ControlLoop(robot, hz=1000.0, mode=Mode.CONTROL).run(
            lambda t, obs: None, max_cycles=3
        )
        during_run = robot.log[: robot.log.index("damp")]
        assert "send" not in during_run
        assert during_run.count("collect") == 3


# ===========================================================================
# 멈출 때
# ===========================================================================
class TestShutdown:
    def test_it_damps_before_cutting_torque(self, robot):
        """서 있는 다리에서 힘이 갑자기 빠지면 주저앉음.

        붙잡는 것이 아니라 `kp=0` 으로 속도만 죽임 -- 위치로 당기지 않으므로
        중력·관성에 맡겨져 급정지보다 완만하게 멈춤 (`Leg.damp`).
        """
        ControlLoop(robot, hz=1000.0, mode=Mode.CONTROL).run(
            motions.hold({"knee": 1.0}), max_cycles=2
        )
        assert "damp" in robot.log
        assert robot.log.index("damp") < len(robot.log) - robot.log[::-1].index("disable") - 1

    def test_torque_is_cut_last(self, robot):
        ControlLoop(robot, hz=1000.0, mode=Mode.CONTROL).run(
            motions.hold({"knee": 1.0}), max_cycles=1
        )
        assert robot.log[-1] == "disable"
        assert robot.torque is False

    def test_observe_does_not_hold(self, robot):
        """토크가 애초에 꺼져 있으므로 붙잡을 것이 없음."""
        ControlLoop(robot, hz=1000.0, mode=Mode.OBSERVE).run(max_cycles=2)
        assert "hold" not in robot.log

    def test_exception_still_cuts_torque(self, robot):
        """제어 중 예외가 나면 모터가 마지막 명령을 계속 유지함."""
        def boom(t, obs):
            raise RuntimeError("동작 계산에서 터짐")

        with pytest.raises(RuntimeError):
            ControlLoop(robot, hz=1000.0, mode=Mode.CONTROL).run(boom)
        assert robot.torque is False
        assert robot.log[-1] == "disable"

    def test_hold_failure_does_not_block_torque_cut(self, robot):
        """자세 유지가 실패해도 힘은 반드시 빠져야 함."""
        robot.hold = lambda: (_ for _ in ()).throw(OSError("버스 오프"))
        ControlLoop(robot, hz=1000.0, mode=Mode.CONTROL).run(
            motions.hold({"knee": 1.0}), max_cycles=1
        )
        assert robot.torque is False

    def test_stop_ends_the_loop(self, robot):
        """다른 스레드나 시그널 처리기에서 부름.

        관찰 모드에서는 동작이 아예 불리지 않으므로 여기서 멈출 수 없음 --
        제어 모드로 확인함.
        """
        loop = ControlLoop(robot, hz=1000.0, mode=Mode.CONTROL)

        def stop_after_two(t, obs):
            if loop.stats.cycles >= 2:
                loop.stop()
            return None

        loop.run(stop_after_two)
        assert loop.stats.cycles <= 4


# ===========================================================================
# 주기
# ===========================================================================
class TestTiming:
    def test_rejects_zero_hz(self, robot):
        with pytest.raises(ValueError, match="0보다 커야 함"):
            ControlLoop(robot, hz=0.0)

    def test_counts_cycles(self, robot):
        stats = ControlLoop(robot, hz=1000.0, mode=Mode.OBSERVE).run(max_cycles=5)
        assert stats.cycles == 5

    def test_duration_ends_the_loop(self, robot):
        stats = ControlLoop(robot, hz=200.0, mode=Mode.OBSERVE).run(duration_s=0.05)
        assert 0 < stats.cycles < 30
        assert stats.total_s >= 0.05

    def test_kept_up_flags_a_sustained_shortfall(self):
        """overruns 는 튀는 주기를 세지만 꾸준히 느린 것은 못 잡음.

        매 주기 24%씩 넘으면 한 번도 밀림으로 세지 않으면서 주파수만 떨어짐.
        """
        assert LoopStats(cycles=100, total_s=1.0, target_hz=100.0).kept_up is True
        assert LoopStats(cycles=80, total_s=1.0, target_hz=100.0).kept_up is False

    def test_summary_marks_the_shortfall(self):
        assert "주기를 못 지킴" in LoopStats(cycles=80, total_s=1.0, target_hz=100.0).summary()
        assert "주기를 못 지킴" not in LoopStats(cycles=100, total_s=1.0, target_hz=100.0).summary()

    def test_no_target_never_complains(self):
        """목표가 없으면 비교할 것도 없음."""
        assert LoopStats(cycles=1, total_s=99.0).kept_up is True

    def test_missing_cycles_are_counted(self):
        robot = FakeRobot(missing=(12,))
        stats = ControlLoop(robot, hz=1000.0, mode=Mode.OBSERVE).run(max_cycles=4)
        assert stats.missing_cycles == 4

    def test_precise_sleep_beats_plain_sleep(self):
        """`time.sleep` 은 요청한 만큼 정확히 자지 않음.

        100Hz 면 한 주기가 10ms 인데 그 오차가 몇 ms 씩 섞이면 주파수가 눈에 띄게
        떨어짐. 마감 직전을 돌면서 기다려 이것을 없앰.

        환경에 따라 `time.sleep` 이 이미 정확할 수 있으므로 **더 낫거나 같음**만
        확인함 -- 정확한 환경에서는 스핀 구간이 저절로 짧아짐.
        """
        target = 0.005

        def measure(fn):
            worst = 0.0
            for _ in range(20):
                start = time.perf_counter()
                fn(target)
                worst = max(worst, time.perf_counter() - start)
            return worst

        assert measure(precise_sleep) <= measure(time.sleep) + 1e-4

    def test_precise_sleep_returns_immediately_when_late(self):
        """이미 늦었으면 따라잡으려 하지 않음."""
        start = time.perf_counter()
        precise_sleep(-1.0)
        assert time.perf_counter() - start < 0.001

    def test_precise_can_be_turned_off(self, robot):
        """스핀은 CPU 를 태움. 끌 수 있어야 함."""
        loop = ControlLoop(robot, hz=1000.0, mode=Mode.OBSERVE, precise=False)
        assert loop.precise is False
        assert loop.run(max_cycles=3).cycles == 3

    def test_does_not_catch_up_after_a_late_cycle(self, robot):
        """밀린 만큼 다음 주기를 줄여 따라잡으면 그 주기가 더 짧아져 또 밀림.

        한 주기를 늦게 시작하고 마는 편이 나음.
        """
        loop = ControlLoop(robot, hz=1000.0, mode=Mode.OBSERVE)
        loop._sleep_until(0.0)              # 이미 지난 마감. 그냥 돌아와야 함


# ===========================================================================
# 텔레메트리
# ===========================================================================
class TestTelemetryHookup:
    def test_records_every_cycle(self, robot):
        tm = FakeTelemetry()
        ControlLoop(robot, hz=1000.0, telemetry=tm, mode=Mode.OBSERVE).run(max_cycles=5)
        assert len(tm.rows) == 5

    def test_opens_and_closes(self, robot):
        tm = FakeTelemetry()
        ControlLoop(robot, hz=1000.0, telemetry=tm, mode=Mode.OBSERVE).run(max_cycles=2)
        assert (tm.opened, tm.closed) == (1, 1)

    def test_closes_on_exception(self, robot):
        """버퍼에 남은 몇 줄이 사라지면 하필 사고 직전 부분을 잃음."""
        tm = FakeTelemetry()

        def boom(t, obs):
            raise RuntimeError("터짐")

        with pytest.raises(RuntimeError):
            ControlLoop(robot, hz=1000.0, telemetry=tm, mode=Mode.CONTROL).run(boom)
        assert tm.closed == 1

    def test_record_failure_does_not_stop_the_loop(self, robot):
        """관측이 제어를 멈추면 관측할 대상이 없어짐."""
        tm = FakeTelemetry()
        tm.record = lambda **kw: (_ for _ in ()).throw(OSError("디스크 가득 참"))
        stats = ControlLoop(robot, hz=1000.0, telemetry=tm, mode=Mode.OBSERVE).run(max_cycles=3)
        assert stats.cycles == 3

    def test_works_without_telemetry(self, robot):
        assert ControlLoop(robot, hz=1000.0, mode=Mode.OBSERVE).run(max_cycles=2).cycles == 2


# ===========================================================================
# 한 걸음씩
# ===========================================================================
class TestStep:
    def test_step_does_not_wait(self, robot):
        """주기 유지는 run 이 함. 한 사이클만 떼어 시험하거나 대화형으로 씀."""
        loop = ControlLoop(robot, hz=1.0, mode=Mode.CONTROL)
        loop.step(motions.hold({"knee": 5.0}), t=0.0)
        assert robot.sent_actions == [{"knee": 5.0}]

    def test_step_returns_the_observation(self, robot):
        loop = ControlLoop(robot, hz=100.0, mode=Mode.OBSERVE)
        assert "knee.pos" in loop.step(None, t=0.0)


# ===========================================================================
# 통신 두절 시 정지
# ===========================================================================
class SilentRobot(FakeRobot):
    """응답이 없는 로봇. `miss` 가 주기마다 늘어감."""

    def __init__(self, *args, silent_from=0, **kwargs):
        super().__init__(*args, **kwargs)
        self.silent_from = silent_from
        self.cycle = 0
        self.safety = SafetyConfig(link_loss_cycles=3)

    def collect(self):
        self.cycle += 1
        return super().collect()

    def link_status(self, now=None):
        miss = max(0, self.cycle - self.silent_from)
        return {"knee": {"age": -1.0, "ack": 0.0, "miss": float(miss)}}


class TestLinkLoss:
    def test_it_stops_the_loop(self):
        """이어지는 무응답은 그 모터가 마지막 명령을 유지하고 있다는 뜻임."""
        robot = SilentRobot()
        loop = ControlLoop(robot, hz=1000.0, mode=Mode.CONTROL)

        stats = loop.run(motions.hold({"knee": 0.0}), max_cycles=50)
        assert stats.link_loss is not None
        assert stats.cycles < 50

    def test_it_names_what_died(self):
        stats = ControlLoop(SilentRobot(), hz=1000.0, mode=Mode.CONTROL).run(
            motions.hold({"knee": 0.0}), max_cycles=50
        )
        assert stats.link_loss.motors == ("knee",)

    def test_it_settles_before_cutting_torque(self):
        """바로 끊으면 서 있는 다리가 주저앉음.

        통신 두절은 예외로 빠져나가는데, 그래도 `finally` 가 감쇠 단계를 지남.
        """
        robot = SilentRobot()
        ControlLoop(robot, hz=1000.0, mode=Mode.CONTROL).run(
            motions.hold({"knee": 0.0}), max_cycles=50
        )
        assert "damp" in robot.log
        assert robot.log.index("damp") < robot.log.index("disable")

    def test_a_healthy_robot_runs_to_the_end(self):
        robot = SilentRobot(silent_from=10_000)
        stats = ControlLoop(robot, hz=1000.0, mode=Mode.CONTROL).run(
            motions.hold({"knee": 0.0}), max_cycles=20
        )
        assert stats.link_loss is None
        assert stats.cycles == 20

    def test_observe_mode_does_not_stop(self):
        """토크가 없어서 세울 것이 없음. 커미셔닝에서 진행이 끊기면 곤란함."""
        robot = SilentRobot()
        stats = ControlLoop(robot, hz=1000.0, mode=Mode.OBSERVE).run(max_cycles=20)
        assert stats.link_loss is None
        assert stats.cycles == 20

    def test_config_can_turn_it_off(self):
        robot = SilentRobot()
        robot.safety = SafetyConfig(link_loss_cycles=0)
        stats = ControlLoop(robot, hz=1000.0, mode=Mode.CONTROL).run(
            motions.hold({"knee": 0.0}), max_cycles=20
        )
        assert stats.link_loss is None
        assert stats.cycles == 20

    def test_a_robot_without_link_status_is_left_alone(self, robot):
        """응답 개념이 없는 로봇도 있음."""
        stats = ControlLoop(robot, hz=1000.0, mode=Mode.CONTROL).run(
            motions.hold({"knee": 0.0}), max_cycles=20
        )
        assert stats.link_loss is None

    def test_the_summary_says_why_it_stopped(self):
        """정상 종료와 구분되어야 함. 둘 다 조용히 끝남."""
        stats = ControlLoop(SilentRobot(), hz=1000.0, mode=Mode.CONTROL).run(
            motions.hold({"knee": 0.0}), max_cycles=50
        )
        assert "통신 두절" in stats.summary()
