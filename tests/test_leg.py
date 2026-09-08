"""다리 테스트 — 하드웨어 없이 실행됨.

    PYTHONPATH=src python3 -m pytest tests -q

가짜 모터가 붙은 가짜 CAN 버스를 씀. 여기서 확인하는 것은 **경계에서 일어나는
변환**임 — 관절 이름 -> 모터 id, cal -> raw, 발목 pitch/roll -> a1/a2, 그리고 그
순서.
"""

import math
import time
import sys
import types
from collections import deque

import pytest

from huphy.calibration import identity
from huphy.config.schema import LimbConfig, SafetyConfig
from huphy.kinematics.ankle import AnkleGeometry, AnkleKinematics
from huphy.motors.base import Gains, Motor, MotorCalibration
from huphy.motors.canbus import CanBus
from huphy.motors.robstride import tables as T
from huphy.motors.robstride.bus import RobStrideBus
from huphy.motors.robstride.codec import mit
from huphy.robots.leg import (
    ANKLE_JOINTS, FIRST_STEP_DT_S, JOINT_NAMES, Leg, RESEED_GAP_S,
)

MODELS = {7: T.Model.RS02, 8: T.Model.RS02, 9: T.Model.RS02,
          10: T.Model.RS02, 11: T.Model.RS00, 12: T.Model.RS00}


LIMITS = {
    "hip_pitch": (-117.07, -21.07),
    "hip_roll": (-5.51, 79.64),
    "hip_yaw": (-41.90, 31.09),
    "knee": (-20.65, 74.79),
    "ankle_a": (-79.77, 43.16),
    "ankle_b": (-12.50, 126.66),
}
"""한계각은 캘리브레이션에서 옴. `robot.yaml` 에는 없음 (이슈 #2)."""


def leg_config(**over):
    gains = Gains(kp=30.0, kd=1.0)
    motors = {
        "hip_pitch": Motor(id=7, model="RS02", gains=gains),
        "hip_roll": Motor(id=8, model="RS02", gains=gains),
        "hip_yaw": Motor(id=9, model="RS02", gains=gains),
        "knee": Motor(id=10, model="RS02", gains=gains),
        "ankle_a": Motor(id=11, model="RS00", gains=gains),
        "ankle_b": Motor(id=12, model="RS00", gains=gains),
    }
    motors.update(over.pop("motors", {}))
    return LimbConfig(name="right_leg", kind="leg", side="right", channel="can1",
                      motors=motors, **over)


def measured(joints=None, **over):
    """한계각이 채워진 캘리브레이션. 재고 난 뒤의 상태임."""
    names = list(joints) if joints is not None else list(LIMITS)
    out = {
        name: MotorCalibration(motor_id=-1, limits_deg=LIMITS.get(name))
        for name in names
    }
    out.update(over)
    return out


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
    follow = True
    """True 면 명령을 따라감. False 면 꿈쩍 안 함."""
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
        if d[0] != 0xFF and FakeBus.follow:
            target = math.degrees(mit.uint_to_float(
                (d[0] << 8) | d[1], -enc.pmax_rad, enc.pmax_rad, 16))
            if ((d[3] & 0x0F) << 8) | d[4] > 0:      # kp 가 0이 아니면
                FakeBus.position[mid] = target
        pos = FakeBus.position.get(mid, 0.0)
        q = mit.float_to_uint(math.radians(pos), -enc.pmax_rad, enc.pmax_rad, 16)
        v = mit.float_to_uint(0.0, -enc.vmax_rad_s, enc.vmax_rad_s, 12)
        tau = mit.float_to_uint(0.0, -enc.tmax_nm, enc.tmax_nm, 12)
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
    FakeBus.follow = True
    mod = types.ModuleType("can")
    mod.Message = FakeMessage
    mod.interface = types.SimpleNamespace(Bus=FakeBus)
    monkeypatch.setitem(sys.modules, "can", mod)
    return mod


def build(config=None, **kwargs):
    cfg = config if config is not None else leg_config()
    bus = RobStrideBus(CanBus(cfg.channel), cfg.motors_by_id())
    kwargs.setdefault("calibration", measured(cfg.motors))
    kwargs.setdefault("allow_uncalibrated", True)
    leg = Leg(cfg, bus, **kwargs)
    leg.connect()
    return leg


@pytest.fixture
def leg(fake_can):
    return build()


@pytest.fixture
def raw(leg):
    r = FakeBus.instances[-1]
    r.sent.clear()
    return r


# ===========================================================================
# 구성
# ===========================================================================
class TestConstruction:
    def test_joint_names(self, leg):
        """명령은 관절로 받음. 발목은 pitch/roll 이지 a1/a2 가 아님."""
        assert leg.joint_names == ("hip_pitch", "hip_roll", "hip_yaw", "knee",
                                   "ankle_pitch", "ankle_roll")

    def test_motor_names_differ_from_joints(self, leg):
        """설정에는 모터가 a1/a2 로 있음. 사람이 다루는 것은 관절임."""
        assert "ankle_a" in leg.motor_names
        assert "ankle_a" not in leg.joint_names

    def test_missing_motor_is_rejected(self, fake_can):
        """다리에 필요한 모터가 빠지면 만들 수 없음."""
        cfg = leg_config()
        short = LimbConfig(name="x", kind="leg", channel="can1",
                           motors={k: v for k, v in cfg.motors.items() if k != "knee"})
        with pytest.raises(ValueError, match=r"필요한 모터가 없음 \['knee'\]"):
            build(short)

    def test_action_features_are_joints(self, leg):
        assert set(leg.action_features) == set(JOINT_NAMES)

    def test_observation_features_are_motors(self, leg):
        """관찰은 모터 단위임. 실제로 측정되는 것이 그것이기 때문임."""
        assert "knee.pos" in leg.observation_features
        assert "ankle_a.pos" in leg.observation_features
        assert "ankle_pitch" not in leg.observation_features

    def test_features_are_known_before_running(self, leg):
        """텔레메트리와 기록이 이 목록으로 열을 만듦.

        실행해 보고 나서야 필드를 알면 로그 형식이 매 실행마다 달라짐.
        """
        assert leg.observation_features == leg.observation_features
        assert set(leg.observation_features) == set(leg.get_observation())


# ===========================================================================
# 좌표 변환
# ===========================================================================
class TestCoordinates:
    def test_identity_when_unmeasured(self, leg):
        assert leg.raw_to_cal("knee", 45.0) == 45.0
        assert leg.cal_to_raw("knee", 45.0) == 45.0

    def test_offset_shifts(self, fake_can):
        cal = dict(identity(leg_config().motors))
        cal["knee"] = MotorCalibration(motor_id=-1, offset_deg=12.0)
        leg = build(calibration=cal)
        assert leg.raw_to_cal("knee", 0.0) == pytest.approx(12.0)
        assert leg.cal_to_raw("knee", 12.0) == pytest.approx(0.0)

    def test_mirrored_sign(self, fake_can):
        """sign 이 -1 이면 같은 물리 자세가 반대 부호의 raw 를 냄."""
        cal = measured(
            knee=MotorCalibration(motor_id=-1, sign=-1.0, limits_deg=LIMITS["knee"])
        )
        leg = build(calibration=cal)
        assert leg.raw_to_cal("knee", -45.0) == pytest.approx(45.0)

    def test_command_goes_out_in_raw(self, fake_can):
        """명령은 cal 로 받아 raw 로 나감. 그 변환이 여기서만 일어남."""
        cal = dict(identity(leg_config().motors))
        cal["knee"] = MotorCalibration(motor_id=-1, offset_deg=12.0)
        leg = build(calibration=cal)
        commands = leg.build_commands({"knee": 30.0})
        assert commands[10].position_deg == pytest.approx(18.0)   # 30 - 12


# ===========================================================================
# 발목
# ===========================================================================
class TestAnkle:
    def test_two_joints_become_two_motors(self, leg):
        commands = leg.build_commands({"ankle_pitch": 5.0, "ankle_roll": 2.0})
        assert set(commands) == {11, 12}

    def test_ik_matches_the_kinematics(self, leg):
        a1, a2 = leg.kinematics.solve_ik(5.0, 2.0)
        commands = leg.build_commands({"ankle_pitch": 5.0, "ankle_roll": 2.0})
        assert commands[11].position_deg == pytest.approx(a1)
        assert commands[12].position_deg == pytest.approx(a2)

    def test_both_are_required(self, leg):
        """모터 두 개가 두 자유도를 같이 만듦. 하나만 주면 나머지가 뭔지 알 수 없음."""
        with pytest.raises(ValueError, match="함께 줘야 함"):
            leg.build_commands({"ankle_pitch": 5.0})

    def test_unreachable_drops_both(self, leg):
        """IK 가 안 풀리면 두 모터 다 직전 명령을 유지함.

        한쪽만 새 명령을 받으면 두 로드가 서로 다른 자세를 요구해 관절이 비틀림.
        """
        commands = leg.build_commands({"ankle_pitch": 80.0, "ankle_roll": 0.0})
        assert 11 not in commands
        assert 12 not in commands

    def test_other_joints_survive_an_ankle_failure(self, leg):
        """나머지 관절은 서로 독립임. 발목이 실패해도 무릎은 나가야 함."""
        commands = leg.build_commands(
            {"knee": 30.0, "ankle_pitch": 80.0, "ankle_roll": 0.0}
        )
        assert 10 in commands

    def test_last_sent_reports_the_executed_pose(self, leg):
        """한계에 잘리면 명령한 자세와 다른 자세가 실행됨.

        무엇을 보냈는지가 아니라 무엇이 실행됐는지를 내야 로그를 믿을 수 있음.
        """
        leg.build_commands({"ankle_pitch": 10.0, "ankle_roll": 5.0})
        sent = leg.last_sent
        assert "ankle_pitch" in sent
        assert sent["ankle_pitch"] != pytest.approx(10.0)      # a2 가 한계에 걸림
        assert leg.counters.clips["limit"] >= 1

    def test_ankle_pose_uses_fk(self, leg):
        FakeBus.position[11] = 3.866
        FakeBus.position[12] = -15.220
        leg.refresh()
        pose = leg.ankle_pose()
        assert pose[0] == pytest.approx(10.0, abs=0.05)
        assert pose[1] == pytest.approx(5.0, abs=0.05)

    def test_ankle_pose_is_in_the_observation(self, leg):
        """모델이 관절각을 입력으로 받으므로 관찰에 같이 냄."""
        FakeBus.position[11] = 3.866
        FakeBus.position[12] = -15.220
        leg.refresh()
        observation = leg.get_observation()
        assert observation["ankle_pitch.pos"] == pytest.approx(10.0, abs=0.05)
        assert observation["ankle_roll.pos"] == pytest.approx(5.0, abs=0.05)

    def test_motor_angles_stay_in_the_observation(self, leg):
        """발목각이 늘어난 것이지 모터각이 빠진 것이 아님. 브링업이 그것을 씀."""
        observation = leg.get_observation()
        assert "ankle_a.pos" in observation
        assert "ankle_b.pos" in observation

    def test_the_pose_is_solved_once_per_cycle(self, leg, monkeypatch):
        """`get_observation()` 이 한 주기에 여러 번 불림. 부를 때마다 풀면 안 됨."""
        calls = []
        original = leg.kinematics.solve_fk
        monkeypatch.setattr(
            leg.kinematics, "solve_fk",
            lambda *a, **k: (calls.append(1), original(*a, **k))[1],
        )
        leg.refresh()
        before = len(calls)
        leg.get_observation()
        leg.get_observation()
        leg.get_observation()
        assert len(calls) == before

    def test_a_failed_fk_keeps_the_last_known_pose(self, leg):
        """모터각이 응답 없을 때 직전 상태를 그대로 두는 것과 같음."""
        FakeBus.position[11] = 3.866
        FakeBus.position[12] = -15.220
        leg.refresh()
        good = leg.get_observation()["ankle_pitch.pos"]

        FakeBus.position[11] = 170.0        # 로드 해가 없는 조합
        FakeBus.position[12] = -170.0
        leg.refresh()
        assert leg.ankle_pose() is None
        assert leg.get_observation()["ankle_pitch.pos"] == pytest.approx(good)


# ===========================================================================
# 안전
# ===========================================================================
class TestSafety:
    def test_limit_is_applied_in_cal_space(self, fake_can):
        """한계가 cal 공간에 있음. raw 로 내린 뒤 검사하면 sign 이 -1 인 관절에서
        부호가 뒤집혀 한계가 반대로 걸림 (이슈 #2).

        점프 가드를 크게 열어 한계만 보이게 함 -- 둘 다 걸리면 어느 쪽이 잘랐는지
        구분되지 않음.
        """
        cal = measured(
            knee=MotorCalibration(motor_id=-1, sign=-1.0, limits_deg=LIMITS["knee"])
        )
        leg = build(calibration=cal, safety=SafetyConfig(max_delta_deg=1000.0))
        commands = leg.build_commands({"knee": 200.0})       # 한계 74.79
        assert leg.last_sent["knee"] == pytest.approx(74.79 - 3.0)   # 여유 3도
        assert commands[10].position_deg == pytest.approx(-(74.79 - 3.0))

    def test_margin_is_taken_from_the_limit(self, fake_can):
        leg = build(safety=SafetyConfig(max_delta_deg=1000.0))
        leg.build_commands({"knee": 200.0})
        assert leg.last_sent["knee"] == pytest.approx(74.79 - 3.0)

    def test_both_guards_can_bite(self, leg):
        """한계가 71.79 이지만 한 주기에 50도까지만 감. 점프 쪽이 먼저 닿음."""
        leg.build_commands({"knee": 200.0})
        assert leg.last_sent["knee"] == pytest.approx(50.0, abs=0.05)
        assert leg.counters.clips["limit"] == 1
        assert leg.counters.clips["jump"] == 1

    def test_nan_is_rejected(self, leg):
        """NaN 하나가 720도 목표 명령이 됨. 클리핑으로 고칠 수 없어 버림."""
        commands = leg.build_commands({"knee": float("nan")})
        assert 10 not in commands
        assert leg.counters.rejects["nan"] == 1

    def test_jump_is_clipped(self, fake_can):
        narrow = SafetyConfig(max_delta_deg=5.0)
        leg = build(safety=narrow)
        leg.build_commands({"knee": 60.0})
        assert leg.last_sent["knee"] == pytest.approx(5.0, abs=0.05)
        assert leg.counters.clips["jump"] == 1

    def test_no_state_is_rejected(self, fake_can):
        """현재 위치를 모르면 점프를 잴 수 없음. 보내지 않음."""
        cfg = leg_config()
        bus = RobStrideBus(CanBus(cfg.channel), cfg.motors_by_id())
        leg = Leg(cfg, bus, calibration=identity(cfg.motors), allow_uncalibrated=True)
        bus.connect()                     # refresh_states 를 안 부름
        commands = leg.build_commands({"knee": 30.0, "hip_pitch": -50.0})
        assert commands == {}
        assert leg.counters.rejects["nostate"] == 2

    def test_counters_accumulate(self, leg):
        """한 주기에 무엇이 몇 번 잘렸는지 쌓임. 텔레메트리로 나감."""
        leg.build_commands({"knee": 200.0})
        leg.build_commands({"knee": 200.0})
        assert leg.counters.clips["limit"] == 2

    def test_only_requested_joints_are_touched(self, leg):
        """명령에 없는 관절은 건드리지 않음. 직전 명령을 그대로 유지함."""
        commands = leg.build_commands({"knee": 30.0})
        assert set(commands) == {10}

    def test_unknown_joint_is_rejected(self, leg):
        """오타를 조용히 무시하면 그 관절만 직전 명령을 유지해 자세가 어긋남."""
        with pytest.raises(ValueError, match=r"모르는 관절 \['elbow'\]"):
            leg.build_commands({"elbow": 10.0})


# ===========================================================================
# 계산·전송·수거
# ===========================================================================
class TestPipeline:
    def test_build_does_not_touch_can(self, leg, raw):
        """버스가 둘일 때 계산을 먼저 몰아야 함 (이슈 #10)."""
        leg.build_commands({"knee": 30.0})
        assert raw.sent == []

    def test_send_writes_frames(self, leg, raw):
        commands = leg.build_commands({"knee": 30.0, "hip_pitch": -50.0})
        assert leg.send(commands) == 2
        assert sorted(m.arbitration_id for m in raw.sent) == [7, 10]

    def test_collect_updates_state(self, leg):
        commands = leg.build_commands({"knee": 30.0})
        leg.send(commands)
        leg.collect()
        assert leg.get_observation()["knee.pos"] == pytest.approx(30.0, abs=0.05)

    def test_collect_reports_the_silent(self, leg, raw):
        """한 모터가 한 주기 빠지는 것은 흔한 일임. 예외 대신 목록으로 알림."""
        missing = leg.collect()
        assert set(missing) == set(leg.config.motor_ids)

    def test_send_action_does_all_three(self, leg, raw):
        sent = leg.send_action({"knee": 30.0})
        assert sent["knee"] == pytest.approx(30.0)
        assert raw.sent

    def test_send_action_returns_what_was_executed(self, leg):
        """명령한 것과 다를 수 있음. 잘린 값이 나와야 로그를 믿을 수 있음."""
        assert leg.send_action({"knee": 200.0})["knee"] == pytest.approx(50.0, abs=0.05)

    def test_hold_targets_the_current_position(self, leg):
        FakeBus.position[10] = 25.0
        leg.bus.refresh_states()
        commands = leg.hold()
        assert commands[10].position_deg == pytest.approx(25.0, abs=0.05)
        assert commands[10].kp == 30.0


# ===========================================================================
# 캘리브레이션
# ===========================================================================
class TestCalibration:
    def test_unmeasured_is_not_calibrated(self, leg):
        assert leg.is_calibrated is False

    def test_needs_both_config_and_measurement(self, fake_can):
        """게인만 있으면 어디까지 가도 되는지 모르고, 한계만 있으면 토크가 안 나감."""
        cal = measured()
        for entry in cal.values():
            entry.zero_reference = "편 상태"
        assert build(calibration=cal).is_calibrated is True

    def test_enable_refuses_when_uncalibrated(self, fake_can):
        cfg = leg_config()
        bus = RobStrideBus(CanBus(cfg.channel), cfg.motors_by_id())
        leg = Leg(cfg, bus, calibration=identity(cfg.motors))
        leg.connect()
        with pytest.raises(RuntimeError, match="allow_uncalibrated"):
            leg.enable()

    def test_enable_warns_when_allowed(self, leg, caplog):
        """커미셔닝 단계에서는 미실측 상태로 움직여야 함."""
        with caplog.at_level("WARNING"):
            leg.enable()
        assert "allow_uncalibrated" in caplog.text

    def test_error_says_what_is_missing(self, fake_can):
        cfg = leg_config()
        bus = RobStrideBus(CanBus(cfg.channel), cfg.motors_by_id())
        leg = Leg(cfg, bus, calibration=identity(cfg.motors))
        leg.connect()
        with pytest.raises(RuntimeError, match="미실측 관절"):
            leg.enable()


# ===========================================================================
# 수명
# ===========================================================================
class TestLifecycle:
    def test_context_manager_disconnects_on_exception(self, fake_can):
        cfg = leg_config()
        bus = RobStrideBus(CanBus(cfg.channel), cfg.motors_by_id())
        leg = Leg(cfg, bus, calibration=identity(cfg.motors), allow_uncalibrated=True)
        with pytest.raises(RuntimeError):
            with leg:
                leg.enable()
                raise RuntimeError("제어 루프에서 터진 예외")
        assert leg.is_connected is False
        assert FakeBus.instances[-1].sent[-1].data[7] == T.CMD_STOP

    def test_disconnect_twice_is_safe(self, leg):
        leg.disconnect()
        leg.disconnect()
        assert leg.is_connected is False


# ===========================================================================
# 거울상 다리
# ===========================================================================
class TestMirroredLeg:
    def test_left_leg_uses_mirrored_kinematics(self, fake_can):
        """양다리가 같은 관절 명령에 같은 물리 동작을 해야 함.

        기구학이 거울상이 아니면 좌우가 미묘하게 다르게 움직임.
        """
        right = build()
        left = build(kinematics=AnkleKinematics(AnkleGeometry().mirrored()))

        r = right.build_commands({"ankle_pitch": 5.0, "ankle_roll": 2.0})
        l = left.build_commands({"ankle_pitch": 5.0, "ankle_roll": -2.0})
        assert l[11].position_deg == pytest.approx(-r[11].position_deg)
        assert l[12].position_deg == pytest.approx(-r[12].position_deg)


# ===========================================================================
# 링크 상태 — 명령이 씹혔는지
# ===========================================================================
class TestLinkStatus:
    def test_ack_after_a_reply(self, leg):
        leg.send_action({"knee": 10.0})
        assert leg.link_status()["knee"]["ack"] == 1.0
        assert leg.link_status()["knee"]["miss"] == 0.0

    def test_ack_is_minus_one_when_not_commanded(self, leg):
        """명령하지 않은 모터를 1로 내면 거짓말이 되고 0으로 내면 없는 고장이 보임."""
        leg.send_action({"knee": 10.0})
        assert leg.link_status()["hip_pitch"]["ack"] == -1.0

    def test_silent_motor_is_counted(self, fake_can):
        """MIT 모드는 명령을 받으면 반드시 답함. 안 오면 처리하지 않은 것임."""
        leg = build()
        original = FakeBus.send

        def deaf(self, msg):
            if msg.arbitration_id == 10:
                self.sent.append(msg)
                return
            original(self, msg)

        FakeBus.send = deaf
        try:
            for _ in range(3):
                leg.send_action({"knee": 10.0, "hip_pitch": -50.0})
        finally:
            FakeBus.send = original

        status = leg.link_status()
        assert status["knee"]["ack"] == 0.0
        assert status["knee"]["miss"] == 3
        assert status["hip_pitch"]["ack"] == 1.0

    def test_age_is_the_reliable_signal(self, fake_can):
        """응답이 한 번도 없던 모터는 위치를 몰라 가드가 명령을 거부함.

        그러면 명령이 안 나가고 응답 대상에서도 빠져 ack 가 -1 로 가려짐.
        age 는 명령 여부와 무관하게 참임.
        """
        leg = build()
        assert leg.link_status()["knee"]["age"] >= 0.0     # connect 에서 받음

    def test_never_answered_reports_minus_one(self, fake_can):
        cfg = leg_config()
        bus = RobStrideBus(CanBus(cfg.channel), cfg.motors_by_id())
        leg = Leg(cfg, bus, calibration=identity(cfg.motors), allow_uncalibrated=True)
        bus.connect()                       # refresh_states 를 안 부름
        assert leg.link_status()["knee"]["age"] == -1.0

    def test_collect_waits_only_for_commanded_motors(self, leg):
        """응답은 명령을 받은 모터만 보냄.

        전체를 기다리면 명령하지 않은 모터가 무응답으로 잡혀 가짜 고장이 보임.
        """
        leg.send(leg.build_commands({"knee": 10.0}))
        assert leg.collect() == ()

    def test_since_clip(self, leg):
        assert leg.since_clip() == -1.0
        leg.build_commands({"knee": 200.0})
        assert 0.0 <= leg.since_clip() < 1.0

    def test_since_reject(self, leg):
        """누적 카운터만으로는 언제 일어났는지 계단을 찾아야 함."""
        assert leg.since_reject() == -1.0
        leg.build_commands({"knee": float("nan")})
        assert 0.0 <= leg.since_reject() < 1.0


# ===========================================================================
# 발목 출력 모드
# ===========================================================================
class TestAnkleOutputMode:
    """발목 두 모터에 무엇을 실을지. 나머지 4관절은 늘 위치임."""

    def test_position_is_the_default(self, leg):
        assert leg.ankle_output == "position"

    def test_an_unknown_mode_is_refused(self, fake_can):
        with pytest.raises(ValueError, match="ankle_output"):
            build(ankle_output="힘")

    def test_position_mode_sends_angles(self, leg):
        commands = leg.build_commands({"ankle_pitch": 10.0, "ankle_roll": 5.0})
        for motor in ("ankle_a", "ankle_b"):
            command = commands[leg.config.motors[motor].id]
            assert command.kp > 0.0
            assert command.torque_nm == 0.0

    def test_torque_mode_sends_torque(self, fake_can):
        torque_leg = build(ankle_output="torque", ankle_kp=(20.0, 20.0))
        commands = torque_leg.build_commands({"ankle_pitch": 10.0, "ankle_roll": 5.0})
        for motor in ("ankle_a", "ankle_b"):
            command = commands[torque_leg.config.motors[motor].id]
            assert command.kp == 0.0
            assert command.kd == 0.0
            assert command.torque_nm != 0.0

    def test_torque_mode_leaves_the_other_joints_alone(self, fake_can):
        """모터 1개 = 관절 1개인 곳은 야코비안이 필요 없음."""
        torque_leg = build(ankle_output="torque", ankle_kp=(20.0, 20.0))
        commands = torque_leg.build_commands({"knee": 10.0, "ankle_pitch": 0.0,
                                              "ankle_roll": 0.0})
        knee = commands[torque_leg.config.motors["knee"].id]
        assert knee.kp > 0.0
        assert knee.torque_nm == 0.0

    def test_no_error_means_no_torque(self, fake_can):
        """목표와 실측이 같으면 PD 가 0임. 중력 보상은 아직 없음."""
        torque_leg = build(ankle_output="torque", ankle_kp=(20.0, 20.0))
        pose = torque_leg.ankle_pose()
        commands = torque_leg.build_commands(
            {"ankle_pitch": pose[0], "ankle_roll": pose[1]}
        )
        for motor in ("ankle_a", "ankle_b"):
            command = commands[torque_leg.config.motors[motor].id]
            assert command.torque_nm == pytest.approx(0.0, abs=1e-9)

    def test_bigger_error_gives_bigger_torque(self, fake_can):
        torque_leg = build(ankle_output="torque", ankle_kp=(20.0, 20.0))
        small = torque_leg.build_commands({"ankle_pitch": 1.0, "ankle_roll": 0.0})
        big = torque_leg.build_commands({"ankle_pitch": 10.0, "ankle_roll": 0.0})
        a1 = torque_leg.config.motors["ankle_a"].id
        assert abs(big[a1].torque_nm) > abs(small[a1].torque_nm)

    def test_torque_mode_skips_the_ik(self, fake_can, monkeypatch):
        """모터 각도가 필요 없음. 목표를 각도로 풀 이유가 없음."""
        torque_leg = build(ankle_output="torque", ankle_kp=(20.0, 20.0))

        def boom(*a, **k):
            raise AssertionError("토크 모드에서는 solve_ik 를 부르지 않아야 함")

        monkeypatch.setattr(torque_leg.kinematics, "solve_ik", boom)
        torque_leg.build_commands({"ankle_pitch": 10.0, "ankle_roll": 5.0})

    def test_last_sent_is_still_joint_space(self, fake_can):
        torque_leg = build(ankle_output="torque", ankle_kp=(20.0, 20.0))
        torque_leg.build_commands({"ankle_pitch": 10.0, "ankle_roll": 5.0})
        assert torque_leg.last_sent["ankle_pitch"] == pytest.approx(10.0)

    def test_an_unknown_pose_sends_nothing(self, fake_can):
        """야코비안이 지금 자세에서 나옴. 자세를 모르면 토크를 만들 수 없음."""
        torque_leg = build(ankle_output="torque", ankle_kp=(20.0, 20.0))
        FakeBus.position[11] = 170.0
        FakeBus.position[12] = -170.0
        torque_leg.refresh()
        commands = torque_leg.build_commands({"ankle_pitch": 10.0, "ankle_roll": 5.0})
        assert torque_leg.config.motors["ankle_a"].id not in commands
        assert torque_leg.counters.rejects["ankle_nopose"] == 1


class TestAnkleVelocity:
    """모터 속도 -> 관절 속도. da = J dpr 이므로 dpr = J^-1 da."""

    def test_an_unknown_pose_gives_zero(self, leg):
        """감쇠가 0이 되는 것이라 부족하게 잡히지, 반대로 밀지는 않음."""
        FakeBus.position[11] = 170.0
        FakeBus.position[12] = -170.0
        leg.refresh()
        assert leg.ankle_pose() is None
        assert leg.ankle_velocity() == (0.0, 0.0)

    def test_it_matches_the_jacobian(self, leg):
        import numpy as np

        FakeBus.position[11] = 3.866
        FakeBus.position[12] = -15.220
        leg.refresh()
        pose = leg.ankle_pose()
        jac = leg.kinematics.jacobian(pose[0], pose[1])
        motor = np.array([
            leg.bus.state(11).velocity_deg_s,
            leg.bus.state(12).velocity_deg_s,
        ])
        assert leg.ankle_velocity() == pytest.approx(
            tuple(np.linalg.solve(jac, motor))
        )

    def test_the_commanded_torque_is_recorded(self, fake_can):
        """모터가 보고한 tau 와 짝임. 시킨 만큼 못 낸 것인지 봐야 함."""
        torque_leg = build(ankle_output="torque", ankle_kp=(20.0, 20.0))
        commands = torque_leg.build_commands({"ankle_pitch": 10.0, "ankle_roll": 5.0})
        a1 = torque_leg.config.motors["ankle_a"].id
        assert torque_leg.last_torque["ankle_a"] == commands[a1].torque_nm

    def test_position_mode_records_zero(self, leg):
        leg.build_commands({"ankle_pitch": 10.0, "ankle_roll": 5.0})
        assert leg.last_torque == {"ankle_a": 0.0, "ankle_b": 0.0}

    def test_a_dropped_command_records_zero(self, fake_can):
        """안 보냈으면 0임. 직전 값이 남으면 계속 보낸 것처럼 보임."""
        torque_leg = build(ankle_output="torque", ankle_kp=(20.0, 20.0))
        torque_leg.build_commands({"ankle_pitch": 10.0, "ankle_roll": 5.0})
        FakeBus.position[11] = 170.0
        FakeBus.position[12] = -170.0
        torque_leg.refresh()
        torque_leg.build_commands({"ankle_pitch": 10.0, "ankle_roll": 5.0})
        assert torque_leg.last_torque == {"ankle_a": 0.0, "ankle_b": 0.0}


class TestRateLimit:
    """속도 제한이 다리 제어 경로에서 실제로 동작하는가 (2026-09-07 벤치).

    guards 단위 시험(tests/test_rate_limit.py)이 램프 자체를 지키고, 이쪽은 다리가
    그것을 **쓰는 방식**을 지킨다: 실측 경과 시간으로 한 주기 몫을 만들고, 명령이
    끊겼다 재개되면 설정점을 측정값으로 다시 잡는가.
    """

    def test_the_command_climbs_while_the_motor_is_stuck(self, fake_can):
        """벤치의 실제 병증. 모터가 전혀 안 움직여도 명령은 계속 나아가야 한다 -
        그래야 오차가 쌓이고 토크가 커져 빠져나올 수 있다. 예전 방식은 여기서
        명령이 measured+max_delta 에 고정돼 영영 못 빠져나왔다."""
        FakeBus.follow = False                       # 모터가 꿈쩍도 안 함
        leg = build(safety=SafetyConfig(max_delta_deg=1000.0, max_vel_deg_s=60.0,
                                  enforce_limits=False))
        leg.enable()
        seen = []
        for _ in range(6):
            time.sleep(0.01)
            leg.build_commands({"knee": 90.0})
            seen.append(leg._prev_cmd["knee"])
        assert seen == sorted(seen), f"명령이 단조 증가해야 함: {seen}"
        assert seen[-1] > seen[0] + 0.5, f"6주기 동안 나아간 것이 없음: {seen}"
        assert seen[-1] < 90.0, "그렇다고 한 번에 목표까지 가면 속도 제한이 아님"

    def test_speed_matches_the_configured_limit(self, fake_can):
        """실측 경과 시간으로 환산하므로, 어떤 주기로 부르든 초당 이동량은 같다."""
        FakeBus.follow = False
        leg = build(safety=SafetyConfig(max_delta_deg=1000.0, max_vel_deg_s=60.0,
                                  enforce_limits=False))
        leg.enable()
        leg.build_commands({"knee": 90.0})           # 첫 호출은 기준 시각을 잡는 것
        t0 = time.monotonic()
        start = leg._prev_cmd["knee"]
        for _ in range(20):
            time.sleep(0.01)
            leg.build_commands({"knee": 90.0})
        elapsed = time.monotonic() - t0
        moved = leg._prev_cmd["knee"] - start
        assert moved == pytest.approx(60.0 * elapsed, rel=0.25), (
            f"{elapsed:.3f}s 에 {moved:.2f}도 = {moved/elapsed:.1f}도/s (한도 60)"
        )

    def test_a_gap_reseeds_the_setpoint_onto_the_measurement(self, fake_can):
        """사고 재발 방지. 전송이 끊겼다 재개되면 설정점이 **측정값**에서 다시
        출발해야 한다 - 끊긴 동안 벌어진 간격을 첫 명령이 한꺼번에 내보내면 안 된다.
        2026-09-07 벤치의 두 이동(40도·44도, 10도)이 전부 재개 첫 패킷이었다.

        `build_commands` 는 CAN 을 쓰지 않으므로(그 메서드의 docstring) 이 시험에서
        측정값은 0 도에 머문다. 그것이 오히려 시험을 선명하게 만든다: 램프가 명령을
        10 도까지 끌고 갔는데 관절은 0 도에 있으므로, 재개 첫 명령이 10 도 근처에서
        이어지면 재시딩이 안 된 것이고, 0 도에서 한 걸음이면 된 것이다."""
        leg = build(safety=SafetyConfig(max_delta_deg=1000.0, max_vel_deg_s=60.0,
                                  enforce_limits=False))
        leg.enable()
        for _ in range(40):                          # 60도/s 로 10도까지 램프
            time.sleep(0.005)
            leg.build_commands({"knee": 10.0})
        assert leg._prev_cmd["knee"] == pytest.approx(10.0, abs=1.0), (
            f"설정 단계에서 목표에 도달하지 못함: {leg._prev_cmd['knee']:.2f}"
        )

        measured = leg.raw_to_cal("knee", leg.bus.state(10).position_deg)
        assert abs(measured) < 1.0, f"이 시험은 측정값이 0도 근처임을 전제함: {measured:.2f}"

        time.sleep(RESEED_GAP_S + 0.02)              # 끊김으로 판정될 만큼의 공백
        leg.build_commands({"knee": 10.0})

        step = 60.0 * FIRST_STEP_DT_S                # 재개 첫 걸음의 크기
        resumed = leg._prev_cmd["knee"]
        assert abs(resumed - measured) <= step + 0.05, (
            f"끊김 뒤 설정점이 측정값({measured:.2f})이 아니라 직전 명령에서 이어짐: "
            f"{resumed:.2f}"
        )
        assert resumed < 5.0, "10도 근처에서 이어졌다면 재시딩이 안 된 것"

    def test_no_limit_configured_behaves_exactly_as_before(self, fake_can):
        """설정하지 않은 로봇은 예전 동작 그대로. 이 가지가 기존 장비를 바꾸지 않는다."""
        FakeBus.follow = False
        leg = build(safety=SafetyConfig(max_delta_deg=1000.0,   # max_vel_deg_s 없음
                                        enforce_limits=False))
        leg.enable()
        time.sleep(0.01)
        leg.build_commands({"knee": 90.0})
        assert leg._prev_cmd["knee"] == pytest.approx(90.0), (
            "속도 제한을 안 걸었으면 목표가 그대로 나가야 함"
        )

    def test_the_blowup_ceiling_still_applies(self, fake_can):
        """속도 제한을 넉넉히 줘도 폭주 상한은 살아 있어야 한다."""
        FakeBus.follow = False
        leg = build(safety=SafetyConfig(max_delta_deg=5.0, max_vel_deg_s=100000.0,
                                  enforce_limits=False))
        leg.enable()
        time.sleep(0.01)
        leg.build_commands({"knee": 90.0})
        # 측정값은 MIT 인코딩으로 양자화되어 정확히 0 이 아니므로 여유를 둔다
        assert leg._prev_cmd["knee"] == pytest.approx(5.0, abs=0.05), (
            "측정 0도에서 max_delta_deg 만큼만"
        )
