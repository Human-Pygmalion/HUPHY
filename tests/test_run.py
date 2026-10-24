"""정책 실행 진입점 테스트 — 하드웨어 없이 실행됨.

    PYTHONPATH=src python3 -m pytest tests -q

가짜 모터와 가짜 IMU 를 붙이고 **명령줄을 그대로 돌림.** `config/policies/` 의 진짜
꾸러미(`generalized_walk/`, `huphy_squat/`)를 읽음.

**이 정책은 양다리 12관절 전용임** -- `--limb`/`--robot` 선택지가 없음. 설정에
`left_leg`/`right_leg` 가 둘 다 있어야 함.

여기서 확인하는 핵심은 **정책이 바로 시작하지 않는다**는 것임. 지금 자세에서 정책
목표로 한 번에 뛰면 관절이 튐.
"""

import json
import math
import sys
import time
import types
from collections import deque
from pathlib import Path

import pytest

from huphy.motors.robstride import tables as T
from huphy.motors.robstride.codec import mit
from huphy.control import deploy
from huphy.scripts import run
from huphy.scripts.run import main
from huphy.sensors.base import ImuState

MODELS = {
    7: T.Model.RS02, 8: T.Model.RS02, 9: T.Model.RS02,
    10: T.Model.RS02, 11: T.Model.RS00, 12: T.Model.RS00,
    1: T.Model.RS02, 2: T.Model.RS02, 3: T.Model.RS02,
    4: T.Model.RS02, 5: T.Model.RS00, 6: T.Model.RS00,
}

ROBOT_YAML = """
name: t
limbs:
  right_leg:
    kind: leg
    side: right
    channel: can1
    calibration: calibration/right_leg.json
    motors:
      hip_pitch: {id: 7,  model: RS02, kp: 30.0, kd: 1.0}
      hip_roll:  {id: 8,  model: RS02, kp: 30.0, kd: 1.0}
      hip_yaw:   {id: 9,  model: RS02, kp: 30.0, kd: 1.0}
      knee:      {id: 10, model: RS02, kp: 30.0, kd: 1.0}
      ankle_a:   {id: 11, model: RS00, kp: 30.0, kd: 1.0}
      ankle_b:   {id: 12, model: RS00, kp: 30.0, kd: 1.0}
  left_leg:
    kind: leg
    side: left
    channel: can0
    calibration: calibration/left_leg.json
    motors:
      hip_pitch: {id: 1, model: RS02, kp: 30.0, kd: 1.0}
      hip_roll:  {id: 2, model: RS02, kp: 30.0, kd: 1.0}
      hip_yaw:   {id: 3, model: RS02, kp: 30.0, kd: 1.0}
      knee:      {id: 4, model: RS02, kp: 30.0, kd: 1.0}
      ankle_a:   {id: 5, model: RS00, kp: 30.0, kd: 1.0}
      ankle_b:   {id: 6, model: RS00, kp: 30.0, kd: 1.0}
imus:
  main:
    model: xsens_mti
    port: /dev/fake_imu
    mount: torso
"""

LIMITS = {
    "hip_pitch": [-117.07, 117.07],
    "hip_roll": [-79.64, 79.64],
    "hip_yaw": [-41.90, 41.90],
    "knee": [-20.65, 114.79],
    "ankle_a": [-79.77, 79.77],
    "ankle_b": [-126.66, 126.66],
}


def _calibration(limb: str) -> dict:
    return {
        "schema_version": 2,
        "limb": limb,
        "note": "",
        "motors": {
            name: {"sign": 1.0, "offset_deg": 0.0, "zero_reference": "편 상태",
                   "limits_deg": limits}
            for name, limits in LIMITS.items()
        },
    }


POLICIES = Path(__file__).resolve().parent.parent / "config" / "policies"
WALK = deploy.load(POLICIES / "generalized_walk")
SQUAT = deploy.load(POLICIES / "huphy_squat")
WEIGHTS = WALK.weights_path()
"""진짜 가중치. 없으면 실행 테스트를 건너뜀."""


# ===========================================================================
# 가짜 하드웨어
# ===========================================================================
class FakeMessage:
    def __init__(self, arbitration_id, data, is_extended_id=False):
        self.arbitration_id = arbitration_id
        self.data = data
        self.is_extended_id = is_extended_id


def _state_frame(mid, pos_deg):
    enc = T.encoding_for(MODELS[mid])
    q = mit.float_to_uint(math.radians(pos_deg), -enc.pmax_rad, enc.pmax_rad, 16)
    v = mit.float_to_uint(0.0, -enc.vmax_rad_s, enc.vmax_rad_s, 12)
    tau = mit.float_to_uint(0.0, -enc.tmax_nm, enc.tmax_nm, 12)
    return FakeMessage(mid, bytes([
        mid, (q >> 8) & 0xFF, q & 0xFF, (v >> 4) & 0xFF,
        ((v & 0x0F) << 4) | ((tau >> 8) & 0x0F), tau & 0xFF, 1, 69,
    ]))


class FakeBus:
    position = {i: 0.0 for i in MODELS}
    instances = []

    def __init__(self, **kwargs):
        self.kwargs = kwargs
        self.rx = deque()
        self.sent = []
        FakeBus.instances.append(self)

    def send(self, msg):
        self.sent.append(msg)
        mid = msg.arbitration_id
        if mid in FakeBus.position:
            self.rx.append(_state_frame(mid, FakeBus.position[mid]))

    def recv(self, timeout=None):
        """큐가 비면 `timeout` 만큼 기다림. **실물 recv 가 그렇게 함.**

        바로 돌아오게 두면 양다리 경로에서 켜지는 수신 스레드가 쉬지 않고 돌며 GIL
        을 쥠. 실물(socketcan)은 select 로 기다리며 GIL 을 놓음.
        """
        if self.rx:
            return self.rx.popleft()
        if timeout:
            time.sleep(timeout)
        return None

    def shutdown(self):
        pass


class FakeImu:
    """`Imu` 프로토콜 자리. 수평으로 서 있다고 보고함."""

    def __init__(self, name="main"):
        self.name = name
        self.connected = False

    @property
    def is_connected(self):
        return self.connected

    def connect(self):
        self.connected = True

    def disconnect(self):
        self.connected = False

    extra_fields = ("roll", "pitch", "yaw")

    def read(self):
        return ImuState(gravity=(0.0, 0.0, -1.0), gyro_dps=(0.0, 0.0, 0.0),
                        extra={"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
                        stamp=1.0, is_valid=True)


@pytest.fixture
def fake_can(monkeypatch):
    FakeBus.instances = []
    FakeBus.position = {i: 0.0 for i in MODELS}
    mod = types.ModuleType("can")
    mod.Message = FakeMessage
    mod.interface = types.SimpleNamespace(Bus=FakeBus)
    monkeypatch.setitem(sys.modules, "can", mod)
    monkeypatch.setattr("huphy.sensors.registry.make_imu", lambda cfg: FakeImu(cfg.name))
    return mod


@pytest.fixture
def cfg(tmp_path):
    (tmp_path / "config" / "calibration").mkdir(parents=True)
    robot = tmp_path / "config" / "robot.yaml"
    robot.write_text(ROBOT_YAML, encoding="utf-8")
    for limb in ("right_leg", "left_leg"):
        (tmp_path / "config" / "calibration" / f"{limb}.json").write_text(
            json.dumps(_calibration(limb), ensure_ascii=False), encoding="utf-8"
        )
    return robot


@pytest.fixture
def go(fake_can, cfg, monkeypatch, capsys):
    """명령줄을 그대로 돌림. 화면이 아니라고 두어 Enter 없이 시작함."""
    monkeypatch.setattr(sys.stdin, "isatty", lambda: False)

    def _go(*argv):
        code = main(["--config", str(cfg), *argv])
        return code, capsys.readouterr().out
    return _go


REAL = ["--approach", "0.05", "--duration", "0.3", "--hz", "50", "--allow-uncalibrated",
        "--policy", "generalized_walk"]
"""walk 로 짧게 돌림. 다른 정책은 뒤에 --policy 를 한 번 더 주면 그쪽이 씀."""


def _position_commands(motor_id):
    """제어 중에 나간 위치 명령. 연결할 때 나가는 상태 조회(kp=0)는 뺌."""
    enc = T.encoding_for(MODELS[motor_id])
    out = []
    for bus in FakeBus.instances:
        for msg in bus.sent:
            if msg.arbitration_id != motor_id or msg.data[0] == 0xFF:
                continue
            d = msg.data
            kp = mit.uint_to_float(((d[3] & 0x0F) << 8) | d[4], 0.0, enc.kp_max, 12)
            if kp > 0.2:
                out.append(msg)
    return out


# ===========================================================================
# 실행 — 끝까지 돌려 봄
# ===========================================================================
@pytest.mark.skipif(not WEIGHTS.is_file(), reason="가중치 파일 없음")
class TestRun:
    def test_it_runs_to_the_end(self, go):
        code, out = go(*REAL)
        assert code == 0, out
        assert "종료. 토크가 끊겼음" in out

    def test_it_shows_what_it_will_do(self, go):
        _, out = go(*REAL)
        assert "generalized_walk" in out
        assert "kp=30" in out  # robot.yaml 값 그대로, 학습값으로 덮어쓰지 않음

    def test_it_warns_that_nothing_stops_it(self, go):
        """상태 기계도 토크 가드도 없음. 사람이 알고 있어야 함."""
        _, out = go(*REAL)
        assert "넘어져도 멈추지 않음" in out

    def test_the_start_screen_shows_the_default_pose(self, go):
        """0 이 아닌 기준 자세로 간다는 것이 화면에 보여야 함."""
        _, out = go(*REAL)
        assert "기준 자세" in out
        assert "기준자세 무릎 +20.1도" in out

    def test_it_runs_both_legs(self, go):
        _, out = go(*REAL)
        channels = {bus.kwargs.get("channel") for bus in FakeBus.instances}
        assert channels == {"can0", "can1"}
        assert "통신 두절" not in out, "한쪽 다리가 응답을 안 해서 멈춘 것임"

    def test_both_legs_receive_commands(self, go):
        """12칸이 이름을 달고 Biped 에서 다리별로 나뉘어 나감."""
        go(*REAL)
        assert _position_commands(4), "왼무릎에 제어 명령이 안 나감"
        assert _position_commands(10), "오른무릎에 제어 명령이 안 나감"

    def test_each_leg_gets_its_own_channel(self, go):
        """왼다리 명령이 오른다리 선으로 가면 안 됨."""
        go(*REAL)
        for bus in FakeBus.instances:
            ids = {m.arbitration_id for m in bus.sent}
            if bus.kwargs.get("channel") == "can0":
                assert ids <= set(range(1, 7))
            else:
                assert ids <= set(range(7, 13))

    def test_the_banner_names_the_channels(self, go):
        _, out = go(*REAL)
        assert "can0" in out and "can1" in out

    def test_the_banner_shows_the_command(self, go):
        _, out = go(*REAL, "--cmd", "vx=0.5", "--cmd", "wz=0.1")
        assert "vx=0.5" in out and "wz=0.1" in out

    def test_gain_scale_lowers_the_yaml_gains(self, go):
        """설정은 kp 30 / kd 1. 화면에 찍히는 값은 모터에 실제로 걸린 것임."""
        code, out = go(*REAL, "--gain-scale", "0.5")
        assert code == 0, out
        assert "kp=15 kd=0.5" in out
        assert "x0.5 로 낮춤" in out

    def test_ankle_gain_scale_is_separate(self, go):
        """설정은 전부 30/1. 발목은 0.8배, 나머지는 0.5배."""
        code, out = go(*REAL, "--gain-scale", "0.5", "0.8")
        assert code == 0, out
        assert "knee 15/0.5" in out
        assert "ankle_a 24/0.8" in out and "ankle_b 24/0.8" in out
        assert "발목 x0.8 / 나머지 x0.5" in out

    def test_log_writes_one_row_per_policy_call(self, go, tmp_path):
        """100Hz 전송이어도 정책은 50Hz 라 줄 수는 정책 호출 수임."""
        _, out = go(*REAL, "--log", str(tmp_path / "lg"))
        files = list((tmp_path / "lg").glob("policy_generalized_walk_*.csv"))
        assert len(files) == 1, out
        lines = files[0].read_text().splitlines()
        assert lines[0].startswith("# limits ")
        header = lines[1].split(",")
        assert len(header) == 1 + WALK.obs_dim + 12 * 3

    def test_log_also_writes_latency_for_all_twelve_joints(self, go, tmp_path):
        _, out = go(*REAL, "--log", str(tmp_path / "lg"))
        files = list((tmp_path / "lg").glob("latency_generalized_walk_*.csv"))
        assert len(files) == 1, out
        lines = files[0].read_text().splitlines()
        assert lines[0].startswith("# drain_ms=")
        joints = {line.split(",")[1] for line in lines[2:]}
        assert len(joints) == 12, joints
        assert "응답 지연" in out


# ===========================================================================
# squat — 같은 실행기, 다른 꾸러미
# ===========================================================================
@pytest.mark.skipif(not SQUAT.weights_path().is_file(), reason="가중치 파일 없음")
class TestSquat:
    SQ = [*REAL, "--policy", "huphy_squat"]

    def test_it_runs_to_the_end(self, go):
        code, out = go(*self.SQ, "--cmd", "h_star=0.80")
        assert code == 0, out
        assert "huphy_squat" in out and "(입력 43, 출력 12)" in out

    def test_the_golden_check_runs_before_the_motors(self, go):
        _, out = go(*self.SQ)
        assert "참조 입출력 통과" in out

    def test_h_star_ramps_from_the_default_height(self, go):
        _, out = go(*self.SQ, "--cmd", "h_star=0.80")
        assert "h_star=0.8914->0.8 (0.1/s)" in out

    def test_without_a_command_it_holds_the_default_height(self, go):
        _, out = go(*self.SQ)
        assert "h_star=0.8914" in out and "->" not in out.split("명령")[1].splitlines()[0]

    def test_the_start_pose_is_the_squat_default_pose(self, go):
        """walk 와 다른 기본 자세. 계약의 offset_rad 를 도로 바꾼 값."""
        _, out = go(*self.SQ)
        assert "기준자세 무릎 +20.1도" in out
        assert SQUAT.default_pose_deg["right_leg/hip_roll"] == pytest.approx(2.8648, abs=1e-3)

    def test_another_checkpoint(self, go):
        code, out = go(*self.SQ, "--checkpoint", "4500")
        assert code == 0, out
        assert "policy_4500.pt" in out

    def test_log_has_43_observation_columns(self, go, tmp_path):
        go(*self.SQ, "--log", str(tmp_path / "lg"))
        files = list((tmp_path / "lg").glob("policy_huphy_squat_*.csv"))
        assert len(files) == 1
        header = files[0].read_text().splitlines()[1].split(",")
        assert len(header) == 1 + 43 + 12 * 3
        assert "obs/cmd/h_star" in header


# ===========================================================================
# 거부 조건 — 모터를 켜기 전에 걸려야 함
# ===========================================================================
class TestRefusals:
    def test_policy_is_required(self, go, capsys):
        """기본 정책이 없음. 무엇을 돌릴지 사람이 꼭 적어야 함."""
        with pytest.raises(SystemExit):
            go("--approach", "0.05", "--duration", "0.3", "--allow-uncalibrated")
        assert "--policy" in capsys.readouterr().err
        assert FakeBus.instances == []

    def test_a_missing_policy_is_refused(self, go):
        with pytest.raises(SystemExit, match="정책 꾸러미를 못 찾음"):
            go("--policy", "없는정책")
        assert FakeBus.instances == []

    def test_a_command_the_policy_does_not_have_is_refused(self, go):
        """walk 에는 h_star 칸이 없음. 조용히 버리면 사람이 준 값이 안 들어간 줄 모름."""
        with pytest.raises(SystemExit, match="이 정책에 없는 명령"):
            go(*REAL, "--cmd", "h_star=0.8")
        assert FakeBus.instances == []

    def test_a_malformed_command_is_refused(self, go):
        with pytest.raises(SystemExit, match="이름=값"):
            go(*REAL, "--cmd", "vx")

    def test_an_unknown_checkpoint_is_refused(self, go):
        with pytest.raises(SystemExit, match="체크포인트 1 가 없음"):
            go(*REAL, "--policy", "huphy_squat", "--checkpoint", "1")
        assert FakeBus.instances == []

    @pytest.mark.parametrize("value", ["0.5", "85"])
    def test_h_star_outside_the_training_range_is_refused(self, go, value):
        """85 는 cm 로 친 실수. 범위 밖이면 CAN 을 열기 전에 멈춤."""
        with pytest.raises(SystemExit, match="학습 범위"):
            go(*REAL, "--policy", "huphy_squat", "--cmd", f"h_star={value}")
        assert FakeBus.instances == []

    @pytest.mark.parametrize("scales", [["0.5", "0"], ["0.5", "1.5"], ["0.5", "0.5", "0.5"]])
    def test_bad_ankle_gain_scale_is_refused(self, go, scales):
        with pytest.raises(SystemExit, match="--gain-scale"):
            go(*REAL, "--gain-scale", *scales)
        assert FakeBus.instances == []

    @pytest.mark.parametrize("scale", ["0", "-0.5", "1.5"])
    def test_gain_scale_out_of_range_is_refused(self, go, scale):
        with pytest.raises(SystemExit, match="--gain-scale"):
            go(*REAL, "--gain-scale", scale)
        assert FakeBus.instances == []

    def test_no_imu_is_refused(self, fake_can, tmp_path, monkeypatch):
        """관찰 44칸 중 6칸이 IMU 값임. 없으면 정책이 못 돎."""
        monkeypatch.setattr(sys.stdin, "isatty", lambda: False)
        (tmp_path / "config" / "calibration").mkdir(parents=True)
        robot = tmp_path / "config" / "robot.yaml"
        robot.write_text(ROBOT_YAML.split("imus:")[0], encoding="utf-8")
        for limb in ("right_leg", "left_leg"):
            (tmp_path / "config" / "calibration" / f"{limb}.json").write_text(
                json.dumps(_calibration(limb), ensure_ascii=False), encoding="utf-8"
            )
        with pytest.raises(SystemExit, match="IMU 가 없음"):
            main(["--config", str(robot), "--policy", "generalized_walk"])

    def test_missing_leg_names_are_refused(self, fake_can, tmp_path):
        """모델은 left_leg/right_leg 로 명령을 냄. 설정 이름이 다르면 못 나눔."""
        (tmp_path / "config" / "calibration").mkdir(parents=True)
        robot = tmp_path / "config" / "robot.yaml"
        right_only = """
name: t
limbs:
  right_leg:
    kind: leg
    side: right
    channel: can1
    calibration: calibration/right_leg.json
    motors:
      hip_pitch: {id: 7,  model: RS02, kp: 30.0, kd: 1.0}
      hip_roll:  {id: 8,  model: RS02, kp: 30.0, kd: 1.0}
      hip_yaw:   {id: 9,  model: RS02, kp: 30.0, kd: 1.0}
      knee:      {id: 10, model: RS02, kp: 30.0, kd: 1.0}
      ankle_a:   {id: 11, model: RS00, kp: 30.0, kd: 1.0}
      ankle_b:   {id: 12, model: RS00, kp: 30.0, kd: 1.0}
imus:
  main:
    model: xsens_mti
    port: /dev/fake_imu
    mount: torso
"""
        robot.write_text(right_only, encoding="utf-8")
        (tmp_path / "config" / "calibration" / "right_leg.json").write_text(
            json.dumps(_calibration("right_leg"), ensure_ascii=False), encoding="utf-8"
        )
        with pytest.raises(SystemExit, match="left_leg"):
            main(["--config", str(robot), *REAL])
        assert FakeBus.instances == []


# ===========================================================================
# 시작 순서 — 바로 정책으로 뛰지 않음
# ===========================================================================
class TestStaged:
    def _staged(self, started_at=None):
        approach = lambda t, obs: {"knee": 100.0 * t}          # noqa: E731
        policy_motion = lambda t, obs: {"knee": 999.0, "t": t}  # noqa: E731
        hold = {"knee": 0.0}
        return run.staged(approach, 1.0, policy_motion, hold_pose=hold)

    def test_it_approaches_first(self):
        """지금 자세에서 목표로 한 번에 뛰면 관절이 튐."""
        motion = self._staged()
        assert motion(0.5, {})["knee"] == pytest.approx(50.0)

    def test_it_holds_the_pose_until_started(self):
        motion = self._staged()
        assert motion(2.0, {}) == {"knee": 0.0}
        assert motion(9.0, {}) == {"knee": 0.0}

    def test_the_policy_runs_after_start(self):
        motion = self._staged()
        motion.start()
        assert motion(2.0, {})["knee"] == 999.0

    def test_the_policy_time_starts_at_zero(self):
        """뛰는 위상이 시작한 순간부터 세야 함."""
        motion = self._staged()
        motion.start()
        assert motion(5.0, {})["t"] == pytest.approx(0.0)
        assert motion(5.5, {})["t"] == pytest.approx(0.5)


class TestEnterWatcher:
    """Enter 로 시작, Ctrl-Q 로 멈춤. 진짜 터미널이 있어야 해서 pty 안에서 확인함."""

    class FakeLoop:
        def __init__(self):
            self.stopped = False

        def stop(self):
            self.stopped = True

    def test_it_starts_at_once_off_screen(self, monkeypatch):
        """누를 사람이 없는데 영원히 기다리면 안 됨."""
        monkeypatch.setattr(sys.stdin, "isatty", lambda: False)
        motion = run.staged(lambda t, o: {}, 0.0, lambda t, o: {}, hold_pose={})
        with run.EnterWatcher(motion, self.FakeLoop()):
            pass
        assert motion.is_started()

    def _press(self, monkeypatch, keys: bytes, until):
        import os
        import pty
        import time

        master, slave = pty.openpty()
        try:
            monkeypatch.setattr(sys, "stdin", os.fdopen(slave, "r"))
            loop = self.FakeLoop()
            motion = run.staged(lambda t, o: {}, 0.0, lambda t, o: {}, hold_pose={})
            with run.EnterWatcher(motion, loop):
                assert not motion.is_started()
                os.write(master, keys)
                deadline = time.monotonic() + 2.0
                while not until(motion, loop) and time.monotonic() < deadline:
                    time.sleep(0.01)
            return motion, loop
        finally:
            os.close(master)

    def test_enter_starts(self, monkeypatch):
        motion, loop = self._press(monkeypatch, b"\r", lambda m, l: m.is_started())
        assert motion.is_started()
        assert not loop.stopped

    def test_ctrl_q_stops(self, monkeypatch):
        motion, loop = self._press(monkeypatch, b"\x11", lambda m, l: l.stopped)
        assert loop.stopped
        assert not motion.is_started()


class TestDrainAndReport:
    def test_drain_reaches_the_bus(self, go, monkeypatch):
        seen = {}
        real = run.build_biped

        def spy(*args, **kwargs):
            seen["drain_s"] = kwargs.get("drain_s")
            return real(*args, **kwargs)

        monkeypatch.setattr(run, "build_biped", spy)
        go(*REAL, "--drain-ms", "5")
        assert seen["drain_s"] == 0.005

    def test_the_report_is_printed(self, go):
        """텔레메트리 없이도 이 주기로 몇 번 빠졌는지 보임."""
        _, out = go(*REAL)
        assert "수거 대기" in out
        assert "대기초과" in out
        assert "가드 자름" in out

    def test_negative_drain_is_refused(self, go):
        with pytest.raises(SystemExit, match="0 이상"):
            go(*REAL, "--drain-ms", "-1")


