"""정책 실행 진입점 테스트 — 하드웨어 없이 실행됨.

    PYTHONPATH=src python3 -m pytest tests -q

가짜 모터와 가짜 IMU 를 붙이고 **명령줄을 그대로 돌림.** `config/policies/` 의 진짜
가중치를 읽음.

여기서 확인하는 핵심은 **정책이 바로 시작하지 않는다**는 것임. 지금 자세에서 정책
목표로 한 번에 뛰면 관절이 튐.
"""

import json
import math
import sys
import time
import types
from collections import deque

import pytest

from huphy.motors.robstride import tables as T
from huphy.motors.robstride.codec import mit
from huphy.control import policy
from huphy.scripts import run
from huphy.scripts.run import main
from huphy.sensors.base import ImuState

MODELS = {7: T.Model.RS02, 8: T.Model.RS02, 9: T.Model.RS02,
          10: T.Model.RS02, 11: T.Model.RS00, 12: T.Model.RS00}

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
imus:
  main:
    model: xsens_mti
    port: /dev/fake_imu
    mount: right_leg
"""

LIMITS = {
    "hip_pitch": [-117.07, 117.07],
    "hip_roll": [-79.64, 79.64],
    "hip_yaw": [-41.90, 41.90],
    "knee": [-20.65, 114.79],
    "ankle_a": [-79.77, 79.77],
    "ankle_b": [-126.66, 126.66],
}

CALIBRATION = {
    "schema_version": 2,
    "limb": "right_leg",
    "note": "",
    "motors": {
        name: {"sign": 1.0, "offset_deg": 0.0, "zero_reference": "편 상태",
               "limits_deg": limits}
        for name, limits in LIMITS.items()
    },
}


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
        을 쥠. 파이썬은 5ms 마다 스레드를 바꾸는데 드레인 예산은 2ms 라, 메인
        스레드가 첫 상태 조회를 놓치고 그 다리가 끝까지 명령을 못 받음 -- 테스트가
        절반 확률로 흔들렸음. 실물(socketcan)은 select 로 기다리며 GIL 을 놓음.
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
    (tmp_path / "config" / "calibration" / "right_leg.json").write_text(
        json.dumps(CALIBRATION, ensure_ascii=False), encoding="utf-8"
    )
    return robot


@pytest.fixture
def go(fake_can, cfg, monkeypatch, capsys):
    """명령줄을 그대로 돌림. 화면이 아니라고 두어 Enter 없이 시작함."""
    monkeypatch.setattr(sys.stdin, "isatty", lambda: False)

    def _go(*argv):
        code = main(["--config", str(cfg), "--limb", "right_leg", *argv])
        return code, capsys.readouterr().out
    return _go


REAL = ["--policy", "balance", "--approach", "0.05", "--duration", "0.3",
        "--hz", "50", "--allow-uncalibrated"]


# ===========================================================================
# 거부 조건 — 모터를 켜기 전에 걸려야 함
# ===========================================================================
class TestRefusals:
    def test_an_unknown_policy_is_refused(self, go):
        with pytest.raises(SystemExit):
            go("--policy", "walk")

    def test_a_missing_weights_file_is_refused(self, go):
        with pytest.raises(SystemExit, match="가중치 파일이 없음"):
            go("--policy", "balance", "--weights", "없는파일.pt")

    def test_a_mismatched_policy_is_refused(self, go):
        """hopping 가중치를 balance 로 읽으면 신경망이 엉뚱한 자리를 읽음."""
        with pytest.raises(SystemExit, match="26개인데 balance"):
            go("--policy", "balance", "--weights", "config/policies/hopping.pt")

    def test_no_imu_is_refused(self, fake_can, tmp_path, monkeypatch, capsys):
        """관찰 24칸 중 6칸이 IMU 값임. 없으면 정책이 못 돎."""
        monkeypatch.setattr(sys.stdin, "isatty", lambda: False)
        (tmp_path / "config" / "calibration").mkdir(parents=True)
        robot = tmp_path / "config" / "robot.yaml"
        robot.write_text(ROBOT_YAML.split("imus:")[0], encoding="utf-8")
        (tmp_path / "config" / "calibration" / "right_leg.json").write_text(
            json.dumps(CALIBRATION, ensure_ascii=False), encoding="utf-8"
        )
        with pytest.raises(SystemExit, match="IMU 가 없음"):
            main(["--config", str(robot), "--limb", "right_leg", "--policy", "balance"])


# ===========================================================================
# 실행
# ===========================================================================
class TestRun:
    def test_it_runs(self, go):
        code, out = go(*REAL)
        assert code == 0
        assert "종료. 토크가 끊겼음" in out

    def test_it_shows_what_it_will_do(self, go):
        _, out = go(*REAL)
        assert "balance" in out
        assert "kp=20.0" in out

    def test_it_warns_that_nothing_stops_it(self, go):
        """상태 기계도 토크 가드도 없음. 사람이 알고 있어야 함."""
        _, out = go(*REAL)
        assert "넘어져도 멈추지 않음" in out

    def test_the_ankle_goes_out_as_torque(self, go):
        """--ankle-output torque 를 줬을 때. 기본은 위치임.

        시뮬은 발목 두 축이 독립 관절임. 실물은 모터 둘이 같이 만듦. 마지막 몇
        개는 정지 직전 `hold()` 라 위치 명령임. 도는 동안을 봄.
        """
        go(*REAL, "--ankle-output", "torque")
        raw = FakeBus.instances[-1]
        enc = T.encoding_for(T.Model.RS00)
        gains = []
        for msg in raw.sent:
            if msg.arbitration_id not in (11, 12) or msg.data[0] == 0xFF:
                continue
            d = msg.data
            gains.append(
                mit.uint_to_float(((d[3] & 0x0F) << 8) | d[4], 0.0, enc.kp_max, 12)
            )
        assert gains, "발목에 아무것도 안 나감"
        assert sum(1 for kp in gains if kp < 0.2) > 5, f"토크로 나간 것이 없음: {gains[:8]}"

    def test_the_ankle_carries_torque(self, go):
        """kp=0 이면 모터는 tau_ff 만 냄. 그 값이 0이면 발목이 떨어짐."""
        go(*REAL, "--ankle-output", "torque")
        raw = FakeBus.instances[-1]
        enc = T.encoding_for(T.Model.RS00)
        torques = []
        for msg in raw.sent:
            if msg.arbitration_id not in (11, 12) or msg.data[0] == 0xFF:
                continue
            d = msg.data
            kp = mit.uint_to_float(((d[3] & 0x0F) << 8) | d[4], 0.0, enc.kp_max, 12)
            if kp < 0.2:
                torques.append(
                    mit.uint_to_float(((d[6] & 0x0F) << 8) | d[7],
                                      -enc.tmax_nm, enc.tmax_nm, 12)
                )
        assert any(abs(v) > 0.01 for v in torques)

    def test_the_other_joints_use_the_training_gain(self, go):
        """설정 파일의 30 이 아니라 학습에 쓴 20 이 나가야 함."""
        go(*REAL)
        raw = FakeBus.instances[-1]
        knee = [m for m in raw.sent if m.arbitration_id == 10 and m.data[0] != 0xFF]
        enc = T.encoding_for(T.Model.RS02)
        d = knee[-1].data
        kp = mit.uint_to_float(((d[3] & 0x0F) << 8) | d[4], 0.0, enc.kp_max, 12)
        assert kp == pytest.approx(run.POLICY_KP, abs=0.5)


# ===========================================================================
# 시작 순서 — 바로 정책으로 뛰지 않음
# ===========================================================================
class TestStaged:
    def _staged(self, started_at=None):
        approach = lambda t, obs: {"knee": 100.0 * t}          # noqa: E731
        policy_motion = lambda t, obs: {"knee": 999.0, "t": t}  # noqa: E731
        return run.staged(approach, 1.0, policy_motion)

    def test_it_approaches_first(self):
        """지금 자세에서 목표로 한 번에 뛰면 관절이 튐."""
        motion = self._staged()
        assert motion(0.5, {})["knee"] == pytest.approx(50.0)

    def test_it_holds_zero_until_started(self):
        motion = self._staged()
        assert motion(2.0, {}) == run.ZERO_POSE
        assert motion(9.0, {}) == run.ZERO_POSE

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

    def test_zero_pose_is_every_joint(self):
        from huphy.control.policy import JOINT_ORDER

        assert set(run.ZERO_POSE) == set(JOINT_ORDER)
        assert all(v == 0.0 for v in run.ZERO_POSE.values())


class TestEnterWatcher:
    def test_it_starts_at_once_off_screen(self, monkeypatch):
        """누를 사람이 없는데 영원히 기다리면 안 됨."""
        monkeypatch.setattr(sys.stdin, "isatty", lambda: False)
        motion = run.staged(lambda t, o: {}, 0.0, lambda t, o: {})
        with run.EnterWatcher(motion):
            pass
        assert motion.is_started()

    def test_it_waits_on_screen(self, monkeypatch):
        monkeypatch.setattr(sys.stdin, "isatty", lambda: True)
        monkeypatch.setattr("builtins.input", lambda *a: (_ for _ in ()).throw(EOFError))
        motion = run.staged(lambda t, o: {}, 0.0, lambda t, o: {})
        with run.EnterWatcher(motion):
            pass
        assert not motion.is_started()


# ===========================================================================
# 발목 공간
# ===========================================================================
class TestAnkleSpace:
    def test_position_is_the_default(self, go):
        """인자 없이 돌리면 발목도 나머지 4관절처럼 위치로 나감."""
        go(*REAL)
        raw = FakeBus.instances[-1]
        enc = T.encoding_for(T.Model.RS00)
        gains = []
        for msg in raw.sent:
            if msg.arbitration_id not in (11, 12) or msg.data[0] == 0xFF:
                continue
            d = msg.data
            gains.append(
                mit.uint_to_float(((d[3] & 0x0F) << 8) | d[4], 0.0, enc.kp_max, 12)
            )
        assert gains, "발목에 아무것도 안 나감"
        # 앞쪽 kp=0 은 연결할 때 나가는 상태 조회(PASSIVE)임. 토크가 아님.
        assert sum(1 for kp in gains if kp > 0.2) > 5, (
            f"위치로 나간 것이 없음: {gains[:8]}"
        )

    def test_ab_with_torque_stops_before_opening_can(self, fake_can, cfg):
        """Leg 이 첫 주기에 거부하기는 하지만, 그때는 이미 토크가 들어간 뒤임."""
        FakeBus.instances = []
        with pytest.raises(SystemExit, match="같이 쓸 수 없음"):
            run.main(["--config", str(cfg), "--limb", "right_leg",
                      *REAL, "--ankle-space", "ab", "--ankle-output", "torque"])
        assert FakeBus.instances == [], "CAN 을 열기 전에 멈춰야 함"

    def test_ab_sends_motor_angles(self, go):
        """모터 공간 모델은 발목 각도를 직접 냄. IK 를 지나가지 않음."""
        go(*REAL, "--ankle-space", "ab")
        raw = FakeBus.instances[-1]
        ankle = [m for m in raw.sent if m.arbitration_id in (11, 12)
                 and m.data[0] != 0xFF]
        assert ankle, "발목에 아무것도 안 나감"

    def test_ab_zero_pose_uses_motor_names(self):
        assert set(run.zero_pose(policy.MOTOR_ORDER)) == set(policy.MOTOR_ORDER)
        assert set(run.zero_pose(policy.JOINT_ORDER)) == set(policy.JOINT_ORDER)

    def test_zero_pose_is_all_zero(self):
        """두 공간의 영자세가 같은 물리 자세임 -- solve_ik(0,0) == (0,0)."""
        assert set(run.zero_pose(policy.MOTOR_ORDER).values()) == {0.0}

    def test_the_banner_shows_the_space(self, go):
        _, out = go(*REAL, "--ankle-space", "ab")
        assert "ab" in out
        assert "ankle_a" in out

    def test_an_unknown_space_is_refused_by_argparse(self):
        with pytest.raises(SystemExit):
            run.build_parser().parse_args(
                ["--policy", "balance", "--ankle-space", "xy"]
            )


# ===========================================================================
# 양다리 — --robot
# ===========================================================================
BIPED_YAML = ROBOT_YAML.replace("""imus:""", """  left_leg:
    kind: leg
    side: left
    channel: can0
    motors:
      hip_pitch: {id: 1, model: RS02, kp: 30.0, kd: 1.0}
      hip_roll:  {id: 2, model: RS02, kp: 30.0, kd: 1.0}
      hip_yaw:   {id: 3, model: RS02, kp: 30.0, kd: 1.0}
      knee:      {id: 4, model: RS02, kp: 30.0, kd: 1.0}
      ankle_a:   {id: 5, model: RS00, kp: 30.0, kd: 1.0}
      ankle_b:   {id: 6, model: RS00, kp: 30.0, kd: 1.0}
imus:""")


@pytest.fixture
def biped_cfg(fake_can, tmp_path, monkeypatch):
    """오른다리가 먼저 적힌 설정 -- 지금 robot.yaml 과 같은 순서임.

    왼다리 모터(1-6)도 가짜 버스가 응답하게 함. 안 그러면 왼다리가 죽은 채로 돌아
    통신 두절로 멈추는데, 테스트는 그래도 통과해 버림.
    """
    for mid in range(1, 7):
        monkeypatch.setitem(MODELS, mid, T.Model.RS00 if mid in (5, 6) else T.Model.RS02)
        monkeypatch.setitem(FakeBus.position, mid, 0.0)
    (tmp_path / "config" / "calibration").mkdir(parents=True)
    robot = tmp_path / "config" / "robot.yaml"
    robot.write_text(BIPED_YAML, encoding="utf-8")
    (tmp_path / "config" / "calibration" / "right_leg.json").write_text(
        json.dumps(CALIBRATION, ensure_ascii=False), encoding="utf-8"
    )
    return robot


@pytest.fixture
def biped_model(monkeypatch):
    """42 -> 12 인 가짜 모델. 무엇을 받았는지 기록함."""
    seen = {}

    def fake_load(path, *, spec, action_dim=None):
        seen["obs_dim"] = spec.obs_dim
        seen["action_dim"] = action_dim

        def model(vector):
            seen["vector_size"] = len(vector)
            return [0.0] * 12

        model.obs_dim = spec.obs_dim
        model.action_dim = 12
        return model

    monkeypatch.setattr(run.rsl_rl, "load", fake_load)
    return seen


@pytest.fixture
def go_robot(fake_can, biped_cfg, monkeypatch, capsys):
    monkeypatch.setattr(sys.stdin, "isatty", lambda: False)

    def _go(*argv):
        code = main(["--config", str(biped_cfg), "--robot", *argv])
        return code, capsys.readouterr().out
    return _go


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


class TestRobot:
    def test_it_runs_both_legs(self, go_robot, biped_model):
        _, out = go_robot(*REAL)
        channels = {bus.kwargs.get("channel") for bus in FakeBus.instances}
        assert channels == {"can0", "can1"}
        assert "통신 두절" not in out, "한쪽 다리가 응답을 안 해서 멈춘 것임"

    def test_the_spec_grows_to_twelve_joints(self, go_robot, biped_model):
        """관찰 42칸, 출력 12칸으로 대조함."""
        go_robot(*REAL)
        assert biped_model["obs_dim"] == 42
        assert biped_model["action_dim"] == 12

    def test_the_model_gets_the_computed_vector(self, go_robot, biped_model):
        go_robot(*REAL)
        assert biped_model["vector_size"] == 42

    def test_both_legs_receive_commands(self, go_robot, biped_model):
        """12칸이 이름을 달고 Biped 에서 다리별로 나뉘어 나감."""
        go_robot(*REAL)
        assert _position_commands(4), "왼무릎에 제어 명령이 안 나감"
        assert _position_commands(10), "오른무릎에 제어 명령이 안 나감"

    def test_each_leg_gets_its_own_channel(self, go_robot, biped_model):
        """왼다리 명령이 오른다리 선으로 가면 안 됨."""
        go_robot(*REAL)
        for bus in FakeBus.instances:
            ids = {m.arbitration_id for m in bus.sent}
            if bus.kwargs.get("channel") == "can0":
                assert ids <= set(range(1, 7))
            else:
                assert ids <= set(range(7, 13))

    def test_the_banner_names_the_robot(self, go_robot, biped_model):
        _, out = go_robot(*REAL)
        assert "can0" in out and "can1" in out

    def test_robot_and_limb_together_is_refused(self, fake_can, biped_cfg):
        with pytest.raises(SystemExit, match="같이 줄 수 없음"):
            main(["--config", str(biped_cfg), "--robot", "--limb", "right_leg", *REAL])
        assert FakeBus.instances == []

    def test_a_single_leg_model_is_refused(self, fake_can, biped_cfg):
        """다리 하나 모델(입력 24)에 --robot 을 주면 모터를 켜기 전에 멈춤."""
        with pytest.raises(SystemExit, match="입력이 24개"):
            main(["--config", str(biped_cfg), "--robot", *REAL])
        assert FakeBus.instances == []

    def test_missing_leg_names_are_refused(self, fake_can, tmp_path, monkeypatch):
        """모델은 left_leg/right_leg 로 명령을 냄. 설정 이름이 다르면 못 나눔."""
        (tmp_path / "config").mkdir()
        robot = tmp_path / "config" / "robot.yaml"
        robot.write_text(ROBOT_YAML, encoding="utf-8")      # right_leg 하나뿐
        with pytest.raises(SystemExit, match="left_leg"):
            main(["--config", str(robot), "--robot", *REAL])
        assert FakeBus.instances == []
