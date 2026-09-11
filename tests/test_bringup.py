"""브링업 메뉴 테스트 — 하드웨어 없이 실행됨.

    PYTHONPATH=src python3 -m pytest tests -q

가짜 모터가 붙은 가짜 CAN 버스에 대고 메뉴를 실제로 돌림. 입력은 미리 넣음.

여기서 확인하는 핵심은 **움직이는 항목이 전부 제어 루프를 지나는지**임 (이슈 #4).
메뉴가 로봇을 직접 부르면 그 경로에서만 텔레메트리와 주기 측정이 빠짐.
"""

import json
from dataclasses import replace
import math
import sys
import types
from collections import deque

import pytest

from huphy.control import ControlLoop, Mode
from huphy.motors.robstride import tables as T
from huphy.motors.robstride.codec import mit
from huphy.scripts import bringup
from huphy.config import ConfigError, load_robot
from huphy.sensors import registry

MODELS = {7: T.Model.RS02, 8: T.Model.RS02, 9: T.Model.RS02,
          10: T.Model.RS02, 11: T.Model.RS00, 12: T.Model.RS00}

ROBOT_YAML = """
name: t
limbs:
  right_leg:
    kind: leg
    side: right
    channel: can1
    control_hz: 200.0
    calibration: calibration/right_leg.json
    motors:
      hip_pitch:     {id: 7,  model: RS02, kp: 30.0, kd: 1.0}
      hip_roll:     {id: 8,  model: RS02, kp: 30.0, kd: 1.0}
      hip_yaw:     {id: 9,  model: RS02, kp: 30.0, kd: 1.0}
      knee:     {id: 10, model: RS02, kp: 30.0, kd: 1.0}
      ankle_a: {id: 11, model: RS00, kp: 30.0, kd: 1.0}
      ankle_b: {id: 12, model: RS00, kp: 30.0, kd: 1.0}
  left_leg:
    kind: leg
    side: left
    channel: can0
    motors:
      hip_pitch:     {id: 1, model: RS02}
      hip_roll:     {id: 2, model: RS02}
      hip_yaw:     {id: 3, model: RS02}
      knee:     {id: 4, model: RS02}
      ankle_a: {id: 5, model: RS00}
      ankle_b: {id: 6, model: RS00}
"""

LIMITS = {
    "hip_pitch": [-117.07, -21.07],
    "hip_roll": [-5.51, 79.64],
    "hip_yaw": [-41.90, 31.09],
    "knee": [-20.65, 74.79],
    "ankle_a": [-79.77, 43.16],
    "ankle_b": [-12.50, 126.66],
}
"""한계각은 캘리브레이션에서 옴. `robot.yaml` 에는 없음 (이슈 #2)."""

CALIBRATION = {
    "schema_version": 2,
    "limb": "right_leg",
    "note": "",
    "motors": {
        name: {
            "sign": 1.0,
            "offset_deg": 0.0,
            "zero_reference": "편 상태",
            "limits_deg": limits,
        }
        for name, limits in LIMITS.items()
    },
}


# ===========================================================================
# 가짜 모터
# ===========================================================================
class FakeMessage:
    def __init__(self, arbitration_id, data, is_extended_id=False):
        self.arbitration_id = arbitration_id
        self.data = data
        self.is_extended_id = is_extended_id


class FakeBus:
    position = {}
    instances = []

    def __init__(self, **kwargs):
        self.rx = deque()
        self.sent = []
        FakeBus.instances.append(self)

    def send(self, msg):
        self.sent.append(msg)
        mid = msg.arbitration_id
        d = msg.data
        enc = T.encoding_for(MODELS[mid])
        if d[0] != 0xFF and ((d[3] & 0x0F) << 8) | d[4] > 0:
            target = math.degrees(mit.uint_to_float(
                (d[0] << 8) | d[1], -enc.pmax_rad, enc.pmax_rad, 16))
            FakeBus.position[mid] += (target - FakeBus.position[mid]) * 0.4
        pos = FakeBus.position.get(mid, 0.0)
        q = mit.float_to_uint(math.radians(pos), -enc.pmax_rad, enc.pmax_rad, 16)
        v = mit.float_to_uint(0.0, -enc.vmax_rad_s, enc.vmax_rad_s, 12)
        tau = mit.float_to_uint(0.3, -enc.tmax_nm, enc.tmax_nm, 12)
        self.rx.append(FakeMessage(mid, bytes([
            mid, (q >> 8) & 0xFF, q & 0xFF, (v >> 4) & 0xFF,
            ((v & 0x0F) << 4) | ((tau >> 8) & 0x0F), tau & 0xFF, 0x01, 0x2C,
        ])))

    def recv(self, timeout=None):
        return self.rx.popleft() if self.rx else None

    def shutdown(self):
        pass


@pytest.fixture
def fake_can(monkeypatch):
    FakeBus.instances = []
    FakeBus.position = {i: 0.0 for i in MODELS}
    FakeBus.position[7] = -60.0        # hip_pitch 는 한계 안
    mod = types.ModuleType("can")
    mod.Message = FakeMessage
    mod.interface = types.SimpleNamespace(Bus=FakeBus)
    monkeypatch.setitem(sys.modules, "can", mod)
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
def menu(fake_can, cfg, monkeypatch, capsys):
    """메뉴를 미리 넣은 입력으로 돌림. (종료코드, 출력) 을 돌려줌."""
    def _run(inputs, *argv):
        it = iter(inputs)
        monkeypatch.setattr("builtins.input", lambda prompt="": next(it))
        code = bringup.main(["--config", str(cfg), "--limb", "right_leg", *argv])
        return code, capsys.readouterr().out
    return _run


# ===========================================================================
# 조립
# ===========================================================================
class TestBuildLeg:
    def test_gain_scale_lowers_every_motor(self, fake_can, cfg):
        from huphy.config import load_robot

        robot = load_robot(cfg)
        leg = bringup.build_leg(robot, robot.limb("right_leg"), gain_scale=0.1)
        assert leg.config.motors["knee"].gains.kp == pytest.approx(3.0)
        assert leg.config.motors["ankle_a"].gains.kd == pytest.approx(0.1)

    def test_left_leg_gets_mirrored_kinematics(self, fake_can, cfg):
        """같은 관절 명령에 양다리가 같은 물리 동작을 하려면 필요함."""
        from huphy.config import load_robot

        robot = load_robot(cfg)
        right = bringup.build_leg(robot, robot.limb("right_leg"))
        left = bringup.build_leg(robot, robot.limb("left_leg"))

        r = right.kinematics.solve_ik(5.0, 2.0)
        l = left.kinematics.solve_ik(5.0, -2.0)
        assert l[0] == pytest.approx(-r[0])
        assert l[1] == pytest.approx(-r[1])

    def test_uses_the_robot_safety_config(self, fake_can, cfg):
        from huphy.config import load_robot

        robot = load_robot(cfg)
        leg = bringup.build_leg(robot, robot.limb("right_leg"))
        assert leg.safety is robot.safety


# ===========================================================================
# 메뉴가 루프를 탐 — 이슈 #4
# ===========================================================================
class TestGoesThroughTheLoop:
    def test_moving_items_use_the_loop(self, fake_can, cfg, monkeypatch, capsys):
        """메뉴가 로봇을 직접 부르지 않음.

        직접 부르면 그 경로에서만 텔레메트리·주기 측정·정지 순서가 빠짐 (이슈 #4).
        """
        runs = []
        original = ControlLoop.run

        def spy(self, motion=None, **kwargs):
            runs.append((self.mode, kwargs))
            return original(self, motion, **kwargs)

        monkeypatch.setattr(ControlLoop, "run", spy)
        it = iter(["3", "0.05", "q"])
        monkeypatch.setattr("builtins.input", lambda prompt="": next(it))
        bringup.main(["--config", str(cfg), "--limb", "right_leg"])

        assert runs, "움직이는 항목이 루프를 타지 않았음"
        assert all(mode is Mode.CONTROL for mode, _ in runs)

    def test_returns_to_observe_after_moving(self, fake_can, cfg, monkeypatch, capsys):
        """움직이고 나면 토크가 꺼진 상태로 돌아와야 함."""
        seen = []
        original = ControlLoop.run

        def spy(self, motion=None, **kwargs):
            result = original(self, motion, **kwargs)
            seen.append(self.mode)
            return result

        monkeypatch.setattr(ControlLoop, "run", spy)
        it = iter(["3", "0.05", "q"])
        monkeypatch.setattr("builtins.input", lambda prompt="": next(it))
        bringup.main(["--config", str(cfg), "--limb", "right_leg"])
        assert seen

    def test_stats_are_per_run(self, menu):
        """실행마다 통계를 새로 냄.

        누적하면 주기 수는 쌓이는데 시간은 이번 것이라 평균 주파수가 목표보다
        높게 나옴.
        """
        _, out = menu(["3", "0.05", "3", "0.05", "q"])
        rates = [
            float(line.split("평균 ")[1].split("Hz")[0])
            for line in out.splitlines() if "평균 " in line
        ]
        assert len(rates) >= 2
        assert all(rate <= 200.0 * 1.05 for rate in rates)


# ===========================================================================
# 화면
# ===========================================================================
class TestShowState:
    def test_shows_every_motor(self, menu):
        _, out = menu(["1", "q"])
        for name in ("hip_pitch", "hip_roll", "hip_yaw", "knee", "ankle_a", "ankle_b"):
            assert name in out

    def test_shows_both_spaces(self, menu):
        _, out = menu(["1", "q"])
        assert "raw" in out and "cal" in out

    def test_shows_link_status(self, menu):
        """명령이 씹혔는지 사람이 바로 보게 함."""
        _, out = menu(["1", "q"])
        assert "응답" in out and "마지막 응답" in out

    def test_shows_ankle_pose(self, menu):
        """모터 각도가 아니라 발이 몇 도 기울었는지를 봄."""
        _, out = menu(["1", "q"])
        assert "발목" in out and "pitch" in out

    def test_warns_when_outside_limits(self, fake_can, cfg, monkeypatch, capsys):
        """토크를 넣으면 가드가 한계 안으로 끌어당김 -- 그 방향으로 움직임.

        사람이 알고 있어야 함.
        """
        FakeBus.position[10] = 200.0        # 무릎 한계는 74.79
        it = iter(["1", "q"])
        monkeypatch.setattr("builtins.input", lambda prompt="": next(it))
        bringup.main(["--config", str(cfg), "--limb", "right_leg"])
        out = capsys.readouterr().out
        assert "한계 밖에 있는 관절" in out
        assert "knee" in out

    def test_quiet_when_inside_limits(self, menu):
        _, out = menu(["1", "q"])
        assert "한계 밖에 있는 관절" not in out

    def test_counters(self, menu):
        _, out = menu(["2", "q"])
        assert "clips_limit" in out
        assert "tx_errors" in out
        assert "마지막 클리핑 이후" in out


# ===========================================================================
# 토크가 필요한 항목
# ===========================================================================
class TestTorqueGate:
    def test_refuses_when_uncalibrated(self, fake_can, tmp_path, monkeypatch, capsys):
        """미실측 상태로 토크를 넣는 것이 가장 위험함."""
        (tmp_path / "config").mkdir()
        robot = tmp_path / "config" / "robot.yaml"
        robot.write_text(ROBOT_YAML, encoding="utf-8")   # 캘리브레이션 파일 없음

        it = iter(["3", "q"])
        monkeypatch.setattr("builtins.input", lambda prompt="": next(it))
        bringup.main(["--config", str(robot), "--limb", "right_leg"])
        out = capsys.readouterr().out
        assert "--allow-uncalibrated" in out

    def test_allows_with_the_flag(self, fake_can, tmp_path, monkeypatch, capsys):
        """실측을 하려면 움직여야 하므로 필요함."""
        (tmp_path / "config").mkdir()
        robot = tmp_path / "config" / "robot.yaml"
        robot.write_text(ROBOT_YAML, encoding="utf-8")

        it = iter(["3", "0.05", "q"])
        monkeypatch.setattr("builtins.input", lambda prompt="": next(it))
        bringup.main(["--config", str(robot), "--limb", "right_leg",
                      "--allow-uncalibrated"])
        assert "평균" in capsys.readouterr().out

    def test_read_only_items_never_gated(self, fake_can, tmp_path, monkeypatch, capsys):
        """상태 보기는 토크가 필요 없음."""
        (tmp_path / "config").mkdir()
        robot = tmp_path / "config" / "robot.yaml"
        robot.write_text(ROBOT_YAML, encoding="utf-8")

        it = iter(["1", "q"])
        monkeypatch.setattr("builtins.input", lambda prompt="": next(it))
        bringup.main(["--config", str(robot), "--limb", "right_leg"])
        assert "--allow-uncalibrated" not in capsys.readouterr().out


# ===========================================================================
# 동작 항목
# ===========================================================================
class TestHoldPose:
    """자세 유지는 관절 여섯 개를 **전부** 명령해야 함.

    발목은 관찰에 모터(`ankle_a`, `ankle_b`)로만 나오고 액션은 관절
    (`ankle_pitch`, `ankle_roll`)로 받음. 지금 자세를 목표로 삼으려면 FK 를 한 번
    거쳐야 하는데, 이걸 빠뜨리면 액션이 통째로 비어 **토크만 켜진 채 명령이 한 개도
    안 나감.** 그때 모터는 자기 내부 목표를 붙잡으므로 다리가 그쪽으로 움직임.
    """

    @pytest.fixture
    def commanded(self, fake_can, cfg, monkeypatch, capsys):
        """자세 유지를 한 번 돌리고, 로봇이 받은 액션들을 돌려줌."""
        from huphy.robots.leg import Leg

        seen = []
        original = Leg.build_commands

        def spy(self, action):
            seen.append(dict(action))
            return original(self, action)

        monkeypatch.setattr(Leg, "build_commands", spy)
        it = iter(["3", "0.05", "q"])
        monkeypatch.setattr("builtins.input", lambda prompt="": next(it))
        bringup.main(["--config", str(cfg), "--limb", "right_leg"])
        return seen, capsys.readouterr().out

    def test_it_commands_something(self, commanded):
        actions, _ = commanded
        assert actions, "명령이 한 개도 나가지 않았음 -- 토크만 켜진 상태임"

    def test_every_joint_is_held(self, commanded):
        actions, _ = commanded
        assert set(actions[0]) == set(bringup.SINGLE_JOINTS) | set(bringup.ANKLE_JOINTS)

    def test_the_target_is_where_the_leg_is(self, commanded):
        """지금 자세를 잡아야 함. 0을 목표로 잡으면 토크가 들어가는 순간 튐."""
        actions, _ = commanded
        assert actions[0]["hip_pitch"] == pytest.approx(-60.0, abs=0.5)

    def test_the_target_does_not_move(self, commanded):
        """목표가 다리를 따라다니면 오차가 늘 0이라 복원력이 안 생김."""
        actions, _ = commanded
        assert all(a == actions[0] for a in actions)


class TestMotionItems:
    def test_step_response_holds_the_others(self, menu):
        """여럿을 같이 흔들면 어느 관절이 원인인지 섞임."""
        _, out = menu(["5", "knee", "3", "q"])
        assert "kp 부족" in out          # 무엇을 볼지 안내함

    def test_sine(self, menu):
        _, out = menu(["6", "knee", "2", "1", "0.1", "q"])
        assert "평균" in out

    def test_move_joint(self, menu):
        _, out = menu(["4", "knee", "3", "q"])
        assert "->" in out

    def test_unknown_joint_is_rejected(self, menu):
        _, out = menu(["5", "elbow", "q"])
        assert "없음" in out

    def test_bad_number_is_rejected(self, menu):
        _, out = menu(["3", "빠르게", "q"])
        assert "숫자가 아님" in out


# ===========================================================================
# 메뉴 자체
# ===========================================================================
class TestMenu:
    def test_marks_items_that_need_torque(self, menu):
        _, out = menu(["q"])
        assert "[토크]" in out

    def test_unknown_choice(self, menu):
        _, out = menu(["99", "q"])
        assert "없는 항목" in out

    def test_shows_calibration_state(self, menu):
        _, out = menu(["q"])
        assert "캘리브레이션" in out

    def test_ends_with_torque_off(self, menu):
        _, out = menu(["q"])
        assert "토크가 끊겼음" in out

    def test_missing_config(self, fake_can, tmp_path):
        with pytest.raises(SystemExit, match="설정 파일이 없음"):
            bringup.main(["--config", str(tmp_path / "없음.yaml")])


# ===========================================================================
# IMU — 붙었을 때만 보임
# ===========================================================================
IMU_YAML = ROBOT_YAML + """
imus:
  main:
    model: xsens_mti
    port: /dev/fake_imu
    mount: right_leg
"""


class FakeImu:
    """`Imu` 프로토콜 자리. 시리얼을 열지 않음."""

    def __init__(self, name="main", valid=True):
        self.name = name
        self.valid = valid
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
        from huphy.sensors.base import ImuState, gravity_from_euler
        import time

        if not self.valid:
            return ImuState()
        return ImuState(
            gravity=gravity_from_euler(1.5, -2.5),
            extra={"roll": 1.5, "pitch": -2.5, "yaw": 30.0},
            stamp=time.monotonic(), is_valid=True,
        )


@pytest.fixture
def imu_cfg(tmp_path):
    (tmp_path / "config" / "calibration").mkdir(parents=True)
    robot = tmp_path / "config" / "robot.yaml"
    robot.write_text(IMU_YAML, encoding="utf-8")
    (tmp_path / "config" / "calibration" / "right_leg.json").write_text(
        json.dumps(CALIBRATION, ensure_ascii=False), encoding="utf-8"
    )
    return robot


@pytest.fixture
def imu_menu(fake_can, imu_cfg, monkeypatch, capsys):
    """설정의 IMU 를 가짜로 바꿔 메뉴를 돌림."""
    made = []

    def fake_make(config):
        imu = FakeImu(config.name)
        made.append(imu)
        return imu

    monkeypatch.setattr(registry, "make_imu", fake_make)

    def _run(inputs, *argv):
        it = iter(inputs)
        monkeypatch.setattr("builtins.input", lambda prompt="": next(it))
        code = bringup.main(["--config", str(imu_cfg), "--limb", "right_leg", *argv])
        return code, capsys.readouterr().out, made
    return _run


class TestImuOnTheLeg:
    def test_it_opens_with_the_leg(self, imu_menu):
        """설정에 적으면 따로 명령할 것 없이 같이 열려야 함."""
        _, _, made = imu_menu(["q"])
        assert len(made) == 1

    def test_the_state_screen_shows_it(self, imu_menu):
        _, out, _ = imu_menu(["1", "q"])
        assert "IMU" in out
        assert "main" in out

    def test_it_shows_the_attitude(self, imu_menu):
        _, out, _ = imu_menu(["1", "q"])
        assert "1.50" in out and "-2.50" in out and "30.00" in out

    def test_a_silent_sensor_says_so(self, imu_menu, monkeypatch):
        """자세는 멈춰도 그럴듯한 숫자로 남음. 살아 있는지는 따로 말해야 함."""
        monkeypatch.setattr(registry, "make_imu", lambda c: FakeImu(c.name, valid=False))
        _, out, _ = imu_menu(["1", "q"])
        assert "응답 없음" in out

    def test_no_imu_adds_no_table(self, menu):
        """IMU 가 없는 설정에서는 화면이 전과 같아야 함."""
        _, out = menu(["1", "q"])
        assert "IMU" not in out

    def test_a_broken_sensor_does_not_stop_the_leg(self, fake_can, imu_cfg, monkeypatch, capsys):
        """IMU 는 관측이지 제어가 아님. 못 열어도 다리는 써야 함."""
        def boom(config):
            raise RuntimeError("포트가 없음")

        monkeypatch.setattr(registry, "make_imu", boom)
        it = iter(["1", "q"])
        monkeypatch.setattr("builtins.input", lambda prompt="": next(it))
        code = bringup.main(["--config", str(imu_cfg), "--limb", "right_leg"])
        assert code == 0
        assert "knee" in capsys.readouterr().out


# ===========================================================================
# 로봇 전체 조립
# ===========================================================================
class TestBuildBiped:
    def test_it_takes_every_leg(self, cfg):
        robot = load_robot(cfg)
        biped = bringup.build_biped(robot)
        assert [p.id for p in biped.parts] == ["right_leg", "left_leg"]

    def test_joint_names_carry_the_limb(self, cfg):
        biped = bringup.build_biped(load_robot(cfg))
        assert "right_leg/knee" in biped.joint_names
        assert "left_leg/knee" in biped.joint_names

    def test_each_leg_keeps_its_own_channel(self, cfg):
        """팔다리마다 CAN 채널이 다름. 섞이면 엉뚱한 쪽이 움직임."""
        biped = bringup.build_biped(load_robot(cfg))
        assert [p.config.channel for p in biped.parts] == ["can1", "can0"]

    def test_the_reader_thread_is_on(self, cfg):
        """버스가 둘이면 순차 수거의 총 대기가 두 배가 됨."""
        biped = bringup.build_biped(load_robot(cfg))
        assert all(p.bus.bus.use_reader for p in biped.parts)

    def test_the_robot_id_comes_from_the_config(self, cfg):
        assert bringup.build_biped(load_robot(cfg)).id == "t"

    def test_safety_is_shared(self, cfg):
        robot = load_robot(cfg)
        biped = bringup.build_biped(robot)
        assert biped.safety == robot.safety
        assert all(p.safety == robot.safety for p in biped.parts)

    def test_explicit_limbs_win(self, cfg):
        robot = load_robot(cfg)
        biped = bringup.build_biped(robot, [robot.limb("left_leg")])
        assert [p.id for p in biped.parts] == ["left_leg"]

    def test_no_legs_is_an_error(self, cfg):
        robot = load_robot(cfg)
        arms = {n: replace(l, kind="arm") for n, l in robot.limbs.items()}
        with pytest.raises(ConfigError, match="kind: leg"):
            bringup.build_biped(replace(robot, limbs=arms))

    def test_options_reach_every_leg(self, cfg):
        """게인 스케일 같은 것이 양다리에 같이 걸려야 함."""
        biped = bringup.build_biped(load_robot(cfg), gain_scale=0.5)
        knee = biped.part("right_leg").config.motors["knee"]
        assert knee.gains.kp == 15.0

    def test_the_robot_owns_the_imus(self, imu_cfg, monkeypatch):
        """몸통 센서는 어느 다리의 것도 아님."""
        monkeypatch.setattr(registry, "make_imu", lambda c: FakeImu(c.name))
        biped = bringup.build_biped(load_robot(imu_cfg))

        assert biped.imus.names == ("main",)
        assert all(len(p.imus) == 0 for p in biped.parts)

    def test_leg_build_still_attaches_its_own(self, imu_cfg, monkeypatch):
        """다리 하나만 돌릴 때는 그 다리가 자기 센서를 들고 있음."""
        monkeypatch.setattr(registry, "make_imu", lambda c: FakeImu(c.name))
        robot = load_robot(imu_cfg)
        leg = bringup.build_leg(robot, robot.limb("right_leg"))

        assert len(leg.imus) == 1

    def test_a_single_leg_has_no_reader(self, cfg):
        """기다릴 버스가 하나뿐이라 얻는 것이 없음."""
        robot = load_robot(cfg)
        leg = bringup.build_leg(robot, robot.limb("right_leg"))
        assert not leg.bus.bus.use_reader


class TestDrainOption:
    def test_default_leaves_the_bus_default(self, cfg):
        from huphy.motors.canbus import DEFAULT_DRAIN_S

        robot = load_robot(cfg)
        leg = bringup.build_leg(robot, robot.limb("right_leg"))
        assert leg.bus.drain_s == DEFAULT_DRAIN_S

    def test_it_reaches_the_bus(self, cfg):
        robot = load_robot(cfg)
        leg = bringup.build_leg(robot, robot.limb("right_leg"), drain_s=0.005)
        assert leg.bus.drain_s == 0.005

    def test_it_reaches_both_legs(self, cfg):
        biped = bringup.build_biped(load_robot(cfg), drain_s=0.005)
        assert all(p.bus.drain_s == 0.005 for p in biped.parts)
