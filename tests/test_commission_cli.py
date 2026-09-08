"""커미셔닝 진입점 테스트 — 하드웨어 없이 실행됨.

    PYTHONPATH=src python3 -m pytest tests -q

가짜 모터가 붙은 가짜 CAN 버스에 대고 명령줄을 그대로 돌림. 확인하는 것은 **사람이
보는 출력과 종료 코드, 그리고 거부 조건**임.

설정 파일은 임시 폴더에 만들어 씀 — 테스트가 저장소의 실제 파일을 건드리면 안 됨.
"""

import argparse
import json
import math
import os
import sys
import types
from collections import deque

import pytest
from dataclasses import replace

from huphy.config import load_robot

from huphy.motors.robstride import tables as T
from huphy.motors.robstride.codec import mit
from huphy.scripts import commission
from huphy.scripts.commission import main

MODELS = {10: T.Model.RS02, 11: T.Model.RS00}

ROBOT_YAML = """
name: t
limbs:
  right_leg:
    kind: leg
    side: right
    channel: can1
    calibration: calibration/right_leg.json
    motors:
      knee:     {id: 10, model: RS02}
      ankle_a: {id: 11, model: RS00}
  left_leg:
    kind: leg
    side: left
    channel: can0
    motors:
      knee: {id: 4, model: RS02}
"""

CALIBRATION = {
    "schema_version": 2,
    "limb": "right_leg",
    "note": "",
    "motors": {
        "knee": {"sign": 1.0, "offset_deg": 0.0, "zero_reference": "",
                 "limits_deg": [-20.65, 74.79]},
        "ankle_a": {"sign": 1.0, "offset_deg": 0.0, "zero_reference": "",
                     "limits_deg": [-79.77, 43.16]},
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


def _state_frame(mid, pos_deg, temp=32.5):
    enc = T.encoding_for(MODELS[mid])
    q = mit.float_to_uint(math.radians(pos_deg), -enc.pmax_rad, enc.pmax_rad, 16)
    v = mit.float_to_uint(0.0, -enc.vmax_rad_s, enc.vmax_rad_s, 12)
    tau = mit.float_to_uint(0.0, -enc.tmax_nm, enc.tmax_nm, 12)
    t = int(temp * 10)
    return FakeMessage(mid, bytes([
        mid, (q >> 8) & 0xFF, q & 0xFF, (v >> 4) & 0xFF,
        ((v & 0x0F) << 4) | ((tau >> 8) & 0x0F), tau & 0xFF,
        (t >> 8) & 0xFF, t & 0xFF,
    ]))


class FakeBus:
    """모터 10번만 응답함. 11번은 조용함."""

    online = {10}
    faults = {}
    position = {10: 30.0, 11: 0.0}
    instances = []

    def __init__(self, **kwargs):
        self.rx = deque()
        self.sent = []
        FakeBus.instances.append(self)

    def send(self, msg):
        self.sent.append(msg)
        mid = msg.arbitration_id
        if mid not in FakeBus.online:
            return
        d = msg.data
        if d[0] == 0xFF and d[7] == T.CMD_FAULT and d[6] == T.F_CMD_FAULT_QUERY:
            word = FakeBus.faults.get(mid, 0)
            self.rx.append(FakeMessage(mid, bytes([          # 작은 자리 먼저 (6.7절)
                mid, word & 0xFF, (word >> 8) & 0xFF,
                (word >> 16) & 0xFF, (word >> 24) & 0xFF, 0, 0, 0,
            ])))
            return
        if d[0] != 0xFF:                        # MIT 명령이면 목표를 따라감
            enc = T.encoding_for(MODELS[mid])
            target = math.degrees(mit.uint_to_float(
                (d[0] << 8) | d[1], -enc.pmax_rad, enc.pmax_rad, 16))
            if ((d[3] & 0x0F) << 8) | d[4] > 0:     # kp 가 0이 아니면
                FakeBus.position[mid] += (target - FakeBus.position[mid]) * 0.5
        self.rx.append(_state_frame(mid, FakeBus.position[mid]))

    def recv(self, timeout=None):
        return self.rx.popleft() if self.rx else None

    def shutdown(self):
        pass


@pytest.fixture
def fake_can(monkeypatch):
    FakeBus.instances = []
    FakeBus.online = {10}
    FakeBus.faults = {}
    FakeBus.position = {10: 30.0, 11: 0.0}
    mod = types.ModuleType("can")
    mod.Message = FakeMessage
    mod.interface = types.SimpleNamespace(Bus=FakeBus)
    monkeypatch.setitem(sys.modules, "can", mod)
    return mod


@pytest.fixture
def cfg(tmp_path):
    """저장소 파일을 건드리지 않도록 임시 설정을 만듦."""
    (tmp_path / "config" / "calibration").mkdir(parents=True)
    robot = tmp_path / "config" / "robot.yaml"
    robot.write_text(ROBOT_YAML, encoding="utf-8")
    cal_path = tmp_path / "config" / "calibration" / "right_leg.json"
    cal_path.write_text(json.dumps(CALIBRATION, ensure_ascii=False), encoding="utf-8")
    return robot


@pytest.fixture
def run(fake_can, cfg, capsys):
    def _run(*argv):
        code = main(["--config", str(cfg), *argv])
        return code, capsys.readouterr()
    return _run


# ===========================================================================
# scan
# ===========================================================================
class TestScan:
    def test_lists_every_joint(self, run):
        code, out = run("--limb", "right_leg", "scan")
        assert "knee" in out.out
        assert "ankle_a" in out.out

    def test_marks_the_silent_one(self, run):
        code, out = run("--limb", "right_leg", "scan")
        assert "응답 없음: ['ankle_a']" in out.out
        assert code == 1

    def test_success_when_all_answer(self, run):
        FakeBus.online = {10, 11}
        code, out = run("--limb", "right_leg", "scan")
        assert code == 0
        assert "전부 응답함" in out.out


# ===========================================================================
# state
# ===========================================================================
class TestState:
    def test_shows_both_spaces(self, run):
        """raw 와 cal 을 나란히 냄. 지금은 미실측이라 같은 값임."""
        _, out = run("--limb", "right_leg", "state")
        assert "raw" in out.out and "cal" in out.out
        assert "30.0" in out.out or "29.9" in out.out

    def test_says_why_they_are_equal(self, run):
        """같은 값이 두 번 나오면 버그처럼 보임. 이유를 밝혀 둠."""
        _, out = run("--limb", "right_leg", "state")
        assert "미실측 상태라 cal 이 raw 와 같음" in out.out

    def test_marks_missing_motor(self, run):
        code, out = run("--limb", "right_leg", "state")
        assert "응답 없음" in out.out
        assert code == 1


# ===========================================================================
# fault
# ===========================================================================
class TestFault:
    def test_normal(self, run):
        FakeBus.online = {10, 11}
        code, out = run("--limb", "right_leg", "fault")
        assert code == 0
        assert out.out.count("정상") == 2

    def test_names_the_bit(self, run):
        FakeBus.online = {10, 11}
        FakeBus.faults = {10: 1 << T.FAULT_BITS["overtemperature"]}
        code, out = run("--limb", "right_leg", "fault")
        assert "overtemperature" in out.out
        assert code == 1

    def test_no_answer_is_not_normal(self, run):
        """응답이 없는 것과 고장이 없는 것은 다름."""
        code, out = run("--limb", "right_leg", "fault")
        assert "응답 없음" in out.out
        assert code == 1


# ===========================================================================
# nudge
# ===========================================================================
class TestNudge:
    def test_moves_and_returns(self, run):
        code, out = run("--limb", "right_leg", "nudge", "knee", "--delta", "5")
        assert code == 0
        assert "시작" in out.out and "최대" in out.out and "끝" in out.out

    def test_warns_when_it_barely_moved(self, run, monkeypatch):
        """게인이 낮거나, 중력을 못 이기거나, 걸린 것임. 사람이 알아야 함."""
        def stuck(self, msg):
            self.sent.append(msg)
            if msg.arbitration_id in FakeBus.online:
                self.rx.append(_state_frame(msg.arbitration_id, 30.0))   # 꿈쩍 안 함

        monkeypatch.setattr(FakeBus, "send", stuck)
        _, out = run("--limb", "right_leg", "nudge", "knee", "--delta", "5")
        assert "명령한 만큼 안 움직였음" in out.out
        assert "--kp 를 조금씩 올려볼 것" in out.out

    def test_refuses_a_silent_motor(self, run, capsys):
        """응답 없는 모터에 토크를 넣지 않음. 배선부터 확인할 일임."""
        code, out = run("--limb", "right_leg", "nudge", "ankle_a")
        assert code == 1
        assert "배선과 CAN id" in out.err

    def test_amplitude_is_capped(self, run):
        with pytest.raises(ValueError, match="20도까지만"):
            run("--limb", "right_leg", "nudge", "knee", "--delta", "45")

    def test_tells_the_operator_to_support_the_leg(self, run):
        _, out = run("--limb", "right_leg", "nudge", "knee")
        assert "받쳐" in out.out


class TestJointByCanId:
    """이름 대신 CAN id 로도 고를 수 있음.

    배선을 확인하는 중에는 어느 id 가 어느 관절인지 아직 모름 (이슈 #8). `scan` 과
    `state` 가 id 를 같이 내므로 그 시점에는 id 로 부르는 것이 자연스러움.
    """

    def test_an_id_picks_the_same_joint(self, run):
        code, out = run("--limb", "right_leg", "nudge", "10", "--delta", "5")
        assert code == 0
        assert "knee" in out.out          # 화면에는 이름으로 나옴

    def test_an_unknown_id_is_refused(self, run):
        with pytest.raises(SystemExit, match="관절이 없음"):
            run("--limb", "right_leg", "nudge", "99")

    def test_the_error_lists_both_ways(self, run):
        """이름만 알려주면 id 를 쳤을 때 무엇이 틀렸는지 모름."""
        with pytest.raises(SystemExit, match=r"id: \[10, 11\]"):
            run("--limb", "right_leg", "nudge", "99")

    def test_a_name_still_works(self, run):
        code, _ = run("--limb", "right_leg", "nudge", "knee", "--delta", "5")
        assert code == 0


# ===========================================================================
# 되돌리기 어려운 조작
# ===========================================================================
class TestDangerous:
    @pytest.mark.parametrize("argv", [
        ("zero", "knee", "--note", "편 상태"),
        ("can-id", "knee", "--to", "20"),
        ("protocol", "knee", "--to", "mit"),
    ])
    def test_requires_yes(self, run, argv):
        with pytest.raises(SystemExit, match="--yes"):
            run("--limb", "right_leg", *argv)

    @pytest.mark.parametrize("argv", [
        ("zero", "knee", "--note", "편 상태"),
        ("can-id", "knee", "--to", "20"),
    ])
    def test_nothing_is_sent_without_yes(self, run, argv):
        """승인 확인이 버스보다 먼저임. 모터에 아무것도 보내지 않은 채로 멈춤."""
        with pytest.raises(SystemExit):
            run("--limb", "right_leg", *argv)
        assert FakeBus.instances == []

    def test_explains_what_will_happen(self, run):
        with pytest.raises(SystemExit, match="모터에 저장"):
            run("--limb", "right_leg", "zero", "knee", "--note", "편 상태")


# ===========================================================================
# zero
# ===========================================================================
class TestZero:
    def test_saves_the_note(self, run, cfg):
        """모터는 영점 값을 저장하지만 "그때 어떤 자세였는지" 는 어디에도 안 남음."""
        code, _ = run("--limb", "right_leg", "zero", "knee",
                      "--note", "다리 편 상태, 발바닥 평면 접촉", "--yes")
        assert code == 0
        path = cfg.parent / "calibration" / "right_leg.json"
        data = json.loads(path.read_text(encoding="utf-8"))
        assert data["motors"]["knee"]["zero_reference"] == "다리 편 상태, 발바닥 평면 접촉"

    def test_leaves_other_joints_alone(self, run, cfg):
        run("--limb", "right_leg", "zero", "knee", "--note", "편 상태", "--yes")
        data = json.loads((cfg.parent / "calibration" / "right_leg.json").read_text(encoding="utf-8"))
        assert data["motors"]["ankle_a"]["zero_reference"] == ""

    def test_note_is_required(self, run):
        with pytest.raises(SystemExit):
            run("--limb", "right_leg", "zero", "knee", "--yes")

    def test_all_joints_when_name_omitted(self, run, cfg):
        """토크가 꺼진 채로 자세를 유지해야 하는데, 명령을 나눠 치면 다리가 무너짐."""
        FakeBus.online = {10, 11}
        code, out = run("--limb", "right_leg", "zero", "--note", "편 상태", "--yes")
        assert code == 0
        data = json.loads((cfg.parent / "calibration" / "right_leg.json").read_text(encoding="utf-8"))
        assert all(e["zero_reference"] == "편 상태" for e in data["motors"].values())

    def test_partial_failure_is_reported(self, run, cfg):
        """응답이 없는 모터는 메모가 비어 남음. 어느 것이 실패했는지 알려줌."""
        code, out = run("--limb", "right_leg", "zero", "--note", "편 상태", "--yes")
        assert code == 1
        assert "ankle_a" in out.out
        data = json.loads((cfg.parent / "calibration" / "right_leg.json").read_text(encoding="utf-8"))
        assert data["motors"]["knee"]["zero_reference"] == "편 상태"
        assert data["motors"]["ankle_a"]["zero_reference"] == ""

    def test_points_at_sweep_next(self, run):
        """영점 뒤에 범위를 재야 함. 자세를 그대로 두고."""
        FakeBus.online = {10, 11}
        _, out = run("--limb", "right_leg", "zero", "--note", "편 상태", "--yes")
        assert "sweep" in out.out

    def test_disables_torque_first(self, run):
        """영점을 잡으면 좌표계가 옮겨가는데 직전 목표각은 옛 좌표계 값임."""
        run("--limb", "right_leg", "zero", "knee", "--note", "편 상태", "--yes")
        sent = FakeBus.instances[-1].sent
        stop = next(i for i, m in enumerate(sent) if m.data[7] == T.CMD_STOP)
        zero = next(i for i, m in enumerate(sent) if m.data[7] == T.CMD_SET_ZERO)
        assert stop < zero


# ===========================================================================
# 대상 고르기
# ===========================================================================
class TestTargeting:
    def test_limb_is_required_when_there_are_several(self, run):
        """다리마다 CAN 채널이 달라 잘못 고르면 엉뚱한 쪽이 움직임."""
        with pytest.raises(SystemExit, match="--limb 을 지정할 것"):
            run("scan")

    def test_unknown_limb_lists_available(self, run):
        with pytest.raises(SystemExit, match=r"\['left_leg', 'right_leg'\]"):
            run("--limb", "right_arm", "scan")

    def test_unknown_joint_lists_available(self, run):
        """사람은 관절로 말함. 모터 id 를 외우게 하지 않음."""
        with pytest.raises(SystemExit, match="'elbow' 관절이 없음"):
            run("--limb", "right_leg", "nudge", "elbow")

    def test_omitted_joint_means_all_where_all_is_allowed(self, run):
        """화면이 아니면 물어볼 수 없음. 전부가 기본인 명령은 전부로 감."""
        FakeBus.online = {10, 11}
        _, out = run("--limb", "right_leg", "zero", "--note", "편 상태", "--yes")
        assert "knee" in out.out and "ankle_a" in out.out

    def test_omitted_joint_is_refused_where_one_is_required(self, run):
        """nudge 는 한 관절만 움직임. 물어볼 수 없으면 고를 방법이 없음."""
        with pytest.raises(SystemExit, match="관절을 지정할 것"):
            run("--limb", "right_leg", "nudge")

    def test_missing_config(self, fake_can, tmp_path):
        with pytest.raises(SystemExit, match="설정 파일이 없음"):
            main(["--config", str(tmp_path / "없음.yaml"), "scan"])

    def test_bad_config_stops_before_the_bus(self, fake_can, tmp_path):
        p = tmp_path / "robot.yaml"
        p.write_text("name: t\nlimbs:\n  a:\n    kind: leg\n    contorl_hz: 5\n", encoding="utf-8")
        with pytest.raises(SystemExit, match="모르는 키"):
            main(["--config", str(p), "scan"])
        assert FakeBus.instances == []


# ===========================================================================
# 옵션 고르기
# ===========================================================================
def _args(command, **given):
    """서브명령이 만들 이름공간. 안 준 옵션은 None 임."""
    fields = {o.name: None for o in commission.OPTIONS[command]}
    fields.update(given)
    return argparse.Namespace(command=command, joint="knee", **fields)


class TestOptions:
    def test_missing_options_take_the_default(self):
        """플래그를 하나도 안 줘도 굴러가야 함. 기본값 출처는 OPTIONS 하나뿐임."""
        args = _args("nudge")
        commission.choose_options("nudge", args, asked=False)
        assert (args.delta, args.kp, args.kd) == (5.0, 5.0, 0.5)

    def test_given_options_survive(self):
        args = _args("nudge", delta=12.0)
        commission.choose_options("nudge", args, asked=False)
        assert args.delta == 12.0 and args.kp == 5.0

    def test_required_option_is_refused_when_absent(self):
        """어느 자세에서 영점을 잡았는지는 대신 정해 줄 수 없음."""
        with pytest.raises(SystemExit, match="--note 을 지정할 것"):
            commission.choose_options("zero", _args("zero"), asked=False)

    def test_choices_are_checked(self):
        args = _args("protocol", to="usb")
        with pytest.raises(SystemExit, match="--to 은"):
            commission.choose_options("protocol", args, asked=False)

    def test_comma_line_fills_in_order(self):
        """표시 순서가 곧 쉼표 순서임."""
        args = _args("nudge")
        commission._apply_options(commission.OPTIONS["nudge"], args, "3, 8, 1.0")
        assert (args.delta, args.kp, args.kd) == (3.0, 8.0, 1.0)

    def test_blank_slot_keeps_the_default(self):
        args = _args("nudge")
        commission._apply_options(commission.OPTIONS["nudge"], args, "3, , 1.0")
        commission.choose_options("nudge", args, asked=False)
        assert (args.delta, args.kp, args.kd) == (3.0, 5.0, 1.0)

    def test_single_option_takes_the_whole_line(self):
        """메모에 쉼표가 들어감. 나누면 뒷부분이 잘림."""
        args = _args("zero")
        commission._apply_options(commission.OPTIONS["zero"], args, "다리 편 상태, 발바닥 접촉")
        assert args.note == "다리 편 상태, 발바닥 접촉"

    def test_too_many_values_is_refused(self):
        with pytest.raises(SystemExit, match="옵션은 3개인데 4개를 받음"):
            commission._apply_options(commission.OPTIONS["nudge"], _args("nudge"), "1,2,3,4")

    def test_unreadable_value_says_which_option(self):
        with pytest.raises(SystemExit, match="delta 값을 읽지 못함"):
            commission._apply_options(commission.OPTIONS["nudge"], _args("nudge"), "많이")

    def test_nothing_is_asked_when_the_joint_was_named(self, monkeypatch):
        """관절을 명령줄에 적었으면 플래그로 다 지정한 것으로 봄. 화면이어도 안 물음."""
        monkeypatch.setattr(commission.sys.stdin, "isatty", lambda: True, raising=False)
        monkeypatch.setattr("builtins.input", lambda p="": pytest.fail("물어보면 안 됨"))
        commission.choose_options("nudge", _args("nudge"), asked=False)

    def test_nothing_is_asked_off_screen(self, monkeypatch):
        """파이프·스크립트에서는 물어볼 수 없음. 기본값으로 감."""
        monkeypatch.setattr("builtins.input", lambda p="": pytest.fail("물어보면 안 됨"))
        args = _args("nudge")
        commission.choose_options("nudge", args, asked=True)
        assert args.delta == 5.0


# ===========================================================================
# sweep 단계 나누기
# ===========================================================================
class TestSweepSteps:
    def test_lone_joints_are_one_step_each(self):
        assert commission._steps(["hip_pitch", "knee"]) == [["hip_pitch"], ["knee"]]

    def test_ankle_motors_share_a_step(self):
        """로드로 발판에 물려 있어 한쪽만 손으로 돌릴 수 없음."""
        assert commission._steps(["ankle_a", "ankle_b"]) == [["ankle_a", "ankle_b"]]

    def test_order_follows_the_joint_list(self):
        steps = commission._steps(["hip_pitch", "ankle_a", "knee", "ankle_b"])
        assert steps == [["hip_pitch"], ["ankle_a", "ankle_b"], ["knee"]]

    def test_half_a_pair_stands_alone(self):
        """한쪽만 골랐으면 그 하나만 잼."""
        assert commission._steps(["ankle_b"]) == [["ankle_b"]]

    def test_every_joint_appears_once(self):
        joints = ["hip_pitch", "hip_roll", "hip_yaw", "knee", "ankle_a", "ankle_b"]
        flat = [j for step in commission._steps(joints) for j in step]
        assert sorted(flat) == sorted(joints)


class TestSweepNeedsAScreen:
    def test_refused_off_screen(self, run):
        """단계마다 Enter 를 받음. 파이프에서는 받을 데가 없음."""
        with pytest.raises(SystemExit, match="화면에서 실행할 것"):
            run("--limb", "right_leg", "sweep", "knee")


# ===========================================================================
# sweep 결과 저장
# ===========================================================================
NAMES = {10: "knee", 11: "ankle_a"}
IDS = {name: mid for mid, name in NAMES.items()}


def _results(**spans):
    """관절 이름 -> 폭 으로 `sweep` 결과를 만듦. 폭 0은 안 움직인 관절임."""
    from huphy.motors.robstride.commissioning import SweepResult

    return {
        IDS[joint]: SweepResult(
            motor_id=IDS[joint], lo_deg=0.0, hi_deg=span, samples=10, offset_deg=-1.5
        )
        for joint, span in spans.items()
    }


@pytest.fixture
def limb(cfg):
    from huphy.config import load_robot

    return load_robot(cfg).limb("right_leg")


@pytest.fixture
def saved(cfg):
    def _saved():
        path = cfg.parent / "calibration" / "right_leg.json"
        return json.loads(path.read_text(encoding="utf-8"))["motors"]
    return _saved


class TestSweepSavesEachStep:
    """단계가 끝날 때마다 씀. 도중에 끊겨도 앞서 잰 것이 남아야 함."""

    def test_a_finished_step_is_on_disk(self, limb, saved):
        commission._save_sweep(limb, _results(knee=30.0), NAMES)
        assert saved()["knee"]["limits_deg"] == [0.0, 30.0]

    def test_later_steps_do_not_erase_earlier_ones(self, limb, saved):
        """단계마다 파일을 다시 읽고 쓰므로 앞 단계가 남아 있어야 함."""
        commission._save_sweep(limb, _results(knee=30.0), NAMES)
        commission._save_sweep(limb, _results(ankle_a=25.0), NAMES)
        assert saved()["knee"]["limits_deg"] == [0.0, 30.0]
        assert saved()["ankle_a"]["limits_deg"] == [0.0, 25.0]

    def test_it_names_what_it_wrote(self, limb):
        assert commission._save_sweep(limb, _results(knee=30.0), NAMES) == ["knee"]

    def test_sign_and_zero_reference_survive(self, limb, saved):
        """다른 곳에서 정해지는 값임. sweep 이 건드리지 않음."""
        commission._save_sweep(limb, _results(knee=30.0), NAMES)
        assert saved()["knee"]["sign"] == 1.0


class TestSweepSkipsUnmovedJoints:
    """폭이 0이면 한계각으로 쓸 수 없음. 그 관절만 빼고 나머지는 저장함."""

    def test_an_unmoved_joint_does_not_block_the_others(self, limb, saved):
        written = commission._save_sweep(
            limb, _results(knee=30.0, ankle_a=0.0), NAMES
        )
        assert written == ["knee"]
        assert saved()["knee"]["limits_deg"] == [0.0, 30.0]

    def test_an_unmoved_joint_keeps_its_old_limits(self, limb, saved):
        commission._save_sweep(limb, _results(knee=30.0, ankle_a=0.0), NAMES)
        assert saved()["ankle_a"]["limits_deg"] == [-79.77, 43.16]

    def test_nothing_moved_writes_nothing(self, limb, cfg):
        path = cfg.parent / "calibration" / "right_leg.json"
        before = path.read_text(encoding="utf-8")
        assert commission._save_sweep(limb, _results(knee=0.0), NAMES) == []
        assert path.read_text(encoding="utf-8") == before

    def test_the_report_names_them(self, limb, capsys):
        code = commission._sweep_report(
            limb, _results(knee=30.0, ankle_a=0.0), NAMES
        )
        out = capsys.readouterr().out
        assert "ankle_a" in out
        assert "sweep ankle_a" in out          # 그것만 다시 재는 명령
        assert code == 1

    def test_the_report_is_clean_when_every_joint_moved(self, limb):
        code = commission._sweep_report(
            limb, _results(knee=30.0, ankle_a=25.0), NAMES
        )
        assert code == 0


class TestEnterKey:
    """Enter 를 길게 누르면 줄바꿈이 여러 개 들어옴. 남기면 다음 단계가 건너뛰어짐."""

    @pytest.fixture
    def keyboard(self, monkeypatch):
        read_fd, write_fd = os.pipe()
        monkeypatch.setattr(
            sys, "stdin",
            types.SimpleNamespace(isatty=lambda: True, fileno=lambda: read_fd),
        )
        yield lambda data: os.write(write_fd, data)
        os.close(read_fd)
        os.close(write_fd)

    def test_one_press_is_seen(self, keyboard):
        keyboard(b"\n")
        assert commission._enter_pressed() is True

    def test_nothing_typed_does_not_wait(self, keyboard):
        assert commission._enter_pressed() is False

    def test_a_long_press_counts_once(self, keyboard):
        """세 줄이 한꺼번에 들어와도 다음 단계로 넘어가지 않아야 함."""
        keyboard(b"\n\n\n")
        assert commission._enter_pressed() is True
        assert commission._enter_pressed() is False


class TestSweepRate:
    """측정 주기는 묻지 않고 기본값으로 감. 사람이 정할 일이 아님."""

    def test_it_is_not_asked(self):
        assert "sweep" not in commission.OPTIONS

    def test_the_default_is_filled_in(self):
        args = commission.build_parser().parse_args(["sweep"])
        assert args.hz == commission.DEFAULT_SWEEP_HZ

    def test_it_can_still_be_given(self):
        args = commission.build_parser().parse_args(["sweep", "--hz", "50"])
        assert args.hz == 50.0


class TestSweepNeedsSomewhereToSave:
    def test_refused_before_measuring(self, run, monkeypatch):
        """다 재고 나서 저장할 데가 없다고 하면 그 시간이 헛수고가 됨."""
        monkeypatch.setattr(sys.stdin, "isatty", lambda: True)
        with pytest.raises(SystemExit, match="저장할 데가 없음"):
            run("--limb", "left_leg", "sweep", "knee")


# ===========================================================================
# 팔다리 여럿 고르기
# ===========================================================================
class TestPickLimbs:
    def test_a_single_name_still_gives_a_list(self, cfg):
        robot = load_robot(cfg)
        assert [l.name for l in commission._pick_limbs(robot, "right_leg")] == [
            "right_leg"
        ]

    def test_commas_keep_the_written_order(self, cfg):
        """순서가 관절 이름 순서와 텔레메트리 열 순서를 정함."""
        robot = load_robot(cfg)
        picked = commission._pick_limbs(robot, "left_leg,right_leg")
        assert [l.name for l in picked] == ["left_leg", "right_leg"]

    def test_a_repeat_is_an_error(self, cfg):
        """같은 버스를 두 번 열게 됨."""
        robot = load_robot(cfg)
        with pytest.raises(SystemExit, match="겹침"):
            commission._pick_limbs(robot, "right_leg,right_leg")

    def test_an_unknown_name_is_an_error(self, cfg):
        robot = load_robot(cfg)
        with pytest.raises(SystemExit):
            commission._pick_limbs(robot, "middle_leg")

    def test_omitting_it_still_needs_one_limb(self, cfg):
        """팔다리마다 채널이 달라 잘못 고르면 엉뚱한 쪽이 움직임."""
        robot = load_robot(cfg)
        with pytest.raises(SystemExit, match="--limb"):
            commission._pick_limbs(robot, None)

    def test_all_is_not_a_limb_name(self, cfg):
        """로봇 전체는 --robot 이 맡음. --limb 은 팔다리를 짚는 플래그임."""
        robot = load_robot(cfg)
        with pytest.raises(SystemExit):
            commission._pick_limbs(robot, "all")


class TestAllLegs:
    def test_it_takes_every_leg(self, cfg):
        robot = load_robot(cfg)
        assert [l.name for l in commission.all_legs(robot)] == [
            "right_leg", "left_leg"
        ]

    def test_it_keeps_the_config_order(self, cfg):
        """순서가 관절 이름 순서와 텔레메트리 열 순서를 정함."""
        robot = load_robot(cfg)
        assert [l.name for l in commission.all_legs(robot)] == list(robot.limbs)

    def test_it_skips_other_kinds(self, cfg):
        """팔은 같은 제어 패턴으로 움직일 수 없음. 필요하면 --limb 에 나열함."""
        robot = load_robot(cfg)
        limbs = dict(robot.limbs)
        limbs["left_leg"] = replace(limbs["left_leg"], kind="arm")
        assert [l.name for l in commission.all_legs(replace(robot, limbs=limbs))] == [
            "right_leg"
        ]

    def test_no_legs_is_an_error(self, cfg):
        robot = load_robot(cfg)
        arms = {n: replace(l, kind="arm") for n, l in robot.limbs.items()}
        with pytest.raises(SystemExit, match="kind: leg"):
            commission.all_legs(replace(robot, limbs=arms))
