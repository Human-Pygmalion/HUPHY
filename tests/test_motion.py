"""모션 추종 꾸러미 — 계약·폐루프·한 주기. `motion545` 와 `motion399` 둘 다 돎.

    PYTHONPATH=src python3 -m pytest tests/test_motion.py -q

고정하는 것은 **틀려도 길이와 자릿수가 그대로라 코드로는 안 잡히는 것**임.

    관찰 114칸의 순서      계약 순서대로 이어 붙는지
    종속 관절 12개의 값    크랭크각에서 푼 값이 학습 MJCF 와 같은지
    기준 프레임            한 주기에 한 프레임 나아가고 끝에서 멈추는지
"""

import json
import math
from pathlib import Path

import numpy as np
import pytest

from huphy.control import deploy
from huphy.control.reference import Clock, ReferenceError, ReferenceMotion
from huphy.kinematics import mjcf_loop

POLICIES = Path(__file__).resolve().parent.parent / "config" / "policies"
PACKAGES = (POLICIES / "motion545", POLICIES / "motion399")
"""모션 추종 꾸러미 전부. 새 꾸러미는 여기 한 줄 -- 같은 검사를 그대로 받음."""

MOTOR = {
    "hip_pitch": "hip_pitch", "hip_roll": "hip_roll", "hip_yaw": "hip_yaw",
    "knee": "knee", "crank_A": "ankle_a", "crank_B": "ankle_b",
}


@pytest.fixture(scope="module", params=PACKAGES, ids=lambda p: p.name)
def package(request):
    return request.param


@pytest.fixture(scope="module")
def contract(package):
    return json.loads((package / "deploy_contract.json").read_text(encoding="utf-8"))


@pytest.fixture(scope="module")
def policy(package):
    return deploy.load(package)


@pytest.fixture(scope="module")
def motion(policy):
    data = np.load(policy.reference.path)
    return {k: data[k] for k in data.files}


class _Imu:
    """`ImuState` 자리. 중력방향과 각속도(도/초)만 봄."""

    def __init__(self, gravity, gyro_dps):
        self.gravity = gravity
        self.gyro_dps = gyro_dps


def _observation(motion, frame: int, names) -> dict:
    """기준 프레임을 **그대로 따라간** 로봇 관찰. 모터 12칸만 채움 (도)."""
    out = {}
    for i, contract_joint in enumerate(names):
        base = contract_joint[2:].replace("_joint", "")
        if base not in MOTOR:
            continue
        side = "right_leg" if contract_joint.startswith("R_") else "left_leg"
        key = f"{side}/{MOTOR[base]}"
        out[f"{key}.pos"] = math.degrees(float(motion["joint_pos"][frame, i]))
        out[f"{key}.vel"] = math.degrees(float(motion["joint_vel"][frame, i]))
    return out


# ===========================================================================
class TestContract:
    def test_the_package_loads_and_the_widths_add_up(self, policy):
        assert policy.obs_dim == 114
        assert len(policy.order) == 12
        assert policy.hz == 50.0
        assert sum(t.width for t in policy.terms) == 114
        assert len(policy.obs_names) == 114

    def test_the_term_order_is_the_training_order(self, policy):
        assert [t.func for t in policy.terms] == [
            "reference_motion", "imu_gravity", "imu_gyro",
            "loop_joint_pos_rel", "loop_joint_vel", "last_action",
        ]

    def test_twenty_four_joints_twelve_of_them_without_a_motor(self, policy):
        assert len(policy.loop_slots) == 24
        motors = [s for s in policy.loop_slots if s.motor is not None]
        dependent = [s for s in policy.loop_slots if s.dependent is not None]
        assert len(motors) == 12 and len(dependent) == 12
        assert {s.dependent for s in dependent} == set(mjcf_loop.DEPENDENT)

    def test_the_motor_default_matches_the_action_offset(self, policy):
        """관찰의 기준점과 행동의 기준점이 어긋나면 정책이 늘 치우친 값을 봄."""
        for slot, default in zip(policy.loop_slots, policy.loop_default_rad):
            if slot.motor is not None:
                assert default == pytest.approx(
                    math.radians(policy.default_pose_deg[slot.motor]), abs=1e-9)

    def test_the_dependent_default_matches_the_loop_model(self, policy):
        for slot, default in zip(policy.loop_slots, policy.loop_default_rad):
            if slot.dependent is not None:
                want = mjcf_loop.SIDES[slot.side].default[slot.dependent]
                assert default == pytest.approx(want, abs=1e-12)

    def test_the_weights_match_the_reference_pairs(self, policy):
        """참조값은 릴리스의 policy.onnx 가중치로 따로 순전파한 것임."""
        model = policy.load_model()
        assert model.golden_max_diff < deploy.GOLDEN_TOL

    def test_the_contract_records_where_it_came_from(self, contract):
        assert "HuphyChan-dev" in contract["what"]
        assert contract["motion"]["source_sha256"]
        ckpt = contract["checkpoints"][contract["default_checkpoint"]]
        assert ckpt["pt_sha256"]


# ===========================================================================
class TestLoop:
    """크랭크각 2개 -> 종속 관절 6개. 참값은 학습에 쓴 기준 모션임."""

    def test_the_solver_reproduces_the_training_values(self, motion):
        names = [str(n) for n in motion["joint_names"]]
        ix = {n: i for i, n in enumerate(names)}
        key = {"rod_a_u1": "rod_A_u1", "rod_a_u2": "rod_A_u2",
               "rod_b_u1": "rod_B_u1", "rod_b_u2": "rod_B_u2",
               "ankle_pitch": "ankle_pitch_joint", "ankle_roll": "ankle_roll_joint"}
        for side, prefix in (("right_leg", "R"), ("left_leg", "L")):
            g = mjcf_loop.SIDES[side]
            pose = dict(g.default)
            worst = 0.0
            for frame in range(0, motion["joint_pos"].shape[0], 3):
                row = motion["joint_pos"][frame]
                pose = mjcf_loop.solve(
                    g,
                    float(row[ix[f"{prefix}_crank_A_joint"]]),
                    float(row[ix[f"{prefix}_crank_B_joint"]]),
                    pose,
                )
                for ours, theirs in key.items():
                    worst = max(worst, abs(pose[ours] - float(row[ix[f"{prefix}_{theirs}"]])))
            assert worst < 1e-6, f"{side} 최대 {worst:.2e} rad"

    def test_the_velocities_follow_the_crank_velocities(self, motion):
        """기준 모션의 종속 관절 속도와 맞아야 함. 학습 노이즈가 0.5 rad/s 임."""
        names = [str(n) for n in motion["joint_names"]]
        ix = {n: i for i, n in enumerate(names)}
        g = mjcf_loop.SIDES["right_leg"]
        pose = dict(g.default)
        worst = 0.0
        for frame in range(0, motion["joint_pos"].shape[0], 3):
            p, v = motion["joint_pos"][frame], motion["joint_vel"][frame]
            a, b = float(p[ix["R_crank_A_joint"]]), float(p[ix["R_crank_B_joint"]])
            pose = mjcf_loop.solve(g, a, b, pose)
            vel = mjcf_loop.velocities(
                g, a, b, float(v[ix["R_crank_A_joint"]]), float(v[ix["R_crank_B_joint"]]), pose)
            worst = max(worst, abs(vel["ankle_pitch"] - float(v[ix["R_ankle_pitch_joint"]])))
            worst = max(worst, abs(vel["rod_a_u1"] - float(v[ix["R_rod_A_u1"]])))
        assert worst < 0.05, f"최대 {worst:.3e} rad/s"

    def test_the_two_ankle_models_are_not_interchangeable(self):
        """`ankle.py`(실측) 와 이 모델(학습 MJCF) 은 같은 크랭크각에 다른 발목각을 냄.

        둘을 섞어 쓰면 관찰이 학습과 어긋남. 차이가 사라지면 이 시험을 지우는 것이
        아니라 `kinematics/mjcf_loop.py` 첫머리의 설명을 고칠 것.
        """
        from huphy.kinematics.ankle import AnkleGeometry, AnkleKinematics

        g = mjcf_loop.SIDES["right_leg"]
        crank_a, crank_b = math.radians(8.0), math.radians(40.0)
        pose = mjcf_loop.solve(g, crank_a, crank_b, dict(g.default))
        measured = AnkleKinematics(AnkleGeometry()).solve_fk(
            math.degrees(crank_a), math.degrees(crank_b))
        assert abs(math.degrees(pose["ankle_roll"]) - measured[1]) > 5.0

    def test_an_impossible_crank_pair_stops(self):
        g = mjcf_loop.SIDES["right_leg"]
        with pytest.raises(mjcf_loop.LoopUnsolvedError):
            # 두 크랭크가 같은 쪽으로 끝까지 돌아간 조합. 로드 둘의 길이를 동시에
            # 맞추는 발 자세가 없음
            mjcf_loop.solve(g, 1.2, 1.2, dict(g.default))


# ===========================================================================
class TestReference:
    def test_the_clock_advances_one_frame_and_stops_at_the_end(self, policy):
        clock = Clock(policy.reference, start=policy.reference.frames - 3)
        assert not clock.done
        clock.step(); clock.step()
        assert clock.index == policy.reference.frames - 1 and not clock.done
        clock.step()
        assert clock.done and clock.index == policy.reference.frames - 1

    def test_the_command_is_absolute_not_default_relative(self, policy, motion):
        """학습 쪽은 기준 프레임을 그대로 넣음. 기본자세를 빼면 정책이 다른 목표를 봄."""
        got = policy.reference.command(7)
        assert got[:24] == pytest.approx(list(motion["joint_pos"][7]), abs=1e-6)
        assert got[24:] == pytest.approx(list(motion["joint_vel"][7]), abs=1e-6)

    def test_a_wrong_hash_stops(self, policy):
        with pytest.raises(ReferenceError):
            ReferenceMotion(policy.reference.path, sha256="0" * 64)

    def test_the_start_pose_is_the_first_frame_not_the_default(self, policy):
        """`approach` 가 데려갈 자세임. 기본자세로 가면 재생 시작에서 벌어져 있음."""
        start = policy.reference_pose(0)
        assert sorted(start) == sorted(policy.order)
        row = policy.reference.joint_pos[0]
        index = {s.name: i for i, s in enumerate(policy.loop_slots)}
        for joint, value in start.items():
            assert value == pytest.approx(math.degrees(float(row[index[joint]])), abs=1e-9)
        # 기본자세와 같지 않음 -- 그게 이 자세를 따로 내는 이유임
        gap = max(abs(value - policy.default_pose_deg[j]) for j, value in start.items())
        assert gap > 1.0

    def test_the_start_pose_is_inside_the_robot_range(self, policy):
        """시작 자세는 가드에 안 걸려야 함 -- 걸리면 거기서 못 멈춤."""
        import json as _json
        from pathlib import Path as _Path
        root = _Path(__file__).resolve().parent.parent
        limits = {}
        for side in ("right_leg", "left_leg"):
            cal = _json.loads((root / "config" / "calibration"
                               / f"{side}_v1.0.json").read_text(encoding="utf-8"))
            for name, motor in cal["motors"].items():
                limits[f"{side}/{name}"] = tuple(float(v) for v in motor["limits_deg"])
        for joint, value in policy.reference_pose(0).items():
            low, high = limits[joint]
            assert low <= value <= high, f"{joint} {value} 가 {low}~{high} 밖"

    def test_a_frame_outside_the_motion_stops(self, policy):
        with pytest.raises(ReferenceError):
            policy.reference.command(policy.reference.frames)


# ===========================================================================
class TestOneCycle:
    """한 주기 전체. 기준 프레임을 그대로 따라가는 로봇을 넣음."""

    def _runner(self, policy):
        return policy.runner(policy.load_model())

    def test_the_vector_matches_the_contract_reference_pair(self, policy, motion, contract):
        """계약의 참조 입력은 이 레포 코드 없이 만든 것임 -- 벡터 조립을 대조함."""
        c = contract
        obs_pairs = c["checkpoints"][c["default_checkpoint"]]["golden"]["obs"]
        names = [str(n) for n in motion["joint_names"]]
        frames = np.linspace(0, motion["joint_pos"].shape[0] - 1, len(obs_pairs)).astype(int)

        runner = self._runner(policy)
        for want, frame in zip(obs_pairs, frames):
            runner.reset()
            runner.clock.index = int(frame)
            imu = _Imu(want[48:51], [math.degrees(v) for v in want[51:54]])
            runner.step(_observation(motion, int(frame), names), imu)
            got = np.asarray(runner.last_vector)
            diff = np.abs(got - np.asarray(want, dtype=np.float32))
            # 속도 칸만 느슨함. 참조 입력의 속도는 모션 변환 스크립트가 낸 값이고
            # 여기 값은 야코비안 x 모터 속도라 종속 관절에서 어긋남 (학습 노이즈는
            # 0.5 rad/s 였음). 나머지 칸은 같은 숫자여야 함.
            for lo, hi, tol, label in ((0, 78, 2e-5, "기준·IMU·위치"),
                                       (78, 102, 0.02, "속도"),
                                       (102, 114, 2e-5, "직전 행동")):
                worst = float(diff[lo:hi].max())
                assert worst < tol, f"프레임 {frame} {label} 최대 {worst:.2e}"

    def test_a_step_moves_the_reference_one_frame(self, policy, motion):
        names = [str(n) for n in motion["joint_names"]]
        runner = self._runner(policy)
        assert runner.clock.index == 0
        runner.step(_observation(motion, 0, names), _Imu([0.0, 0.0, -1.0], [0.0, 0.0, 0.0]))
        assert runner.clock.index == 1

    def test_the_targets_stay_inside_the_clip_range(self, policy, motion):
        names = [str(n) for n in motion["joint_names"]]
        runner = self._runner(policy)
        for frame in range(0, 60):
            targets = runner.step(
                _observation(motion, frame, names), _Imu([0.0, 0.0, -1.0], [0.0, 0.0, 0.0]))
            for joint, value in targets.items():
                low, high = policy.limits_deg[joint]
                assert low - 1e-9 <= value <= high + 1e-9

    def test_reset_puts_the_reference_back_to_the_start(self, policy, motion):
        names = [str(n) for n in motion["joint_names"]]
        runner = self._runner(policy)
        for frame in range(5):
            runner.step(_observation(motion, frame, names), _Imu([0, 0, -1.0], [0, 0, 0]))
        runner.reset()
        assert runner.clock.index == 0 and not runner.clock.done
        assert runner.last_action == [0.0] * 12
