"""정책 꾸러미 공통 실행기(`control/deploy.py`) 테스트 — 하드웨어 없이 돎.

    PYTHONPATH=src python3 -m pytest tests/test_deploy.py -q

`config/policies/` 의 **진짜 꾸러미 둘**을 읽음.

    generalized_walk   계약을 이 저장소가 씀 (옛 generalized_walk.py 값 그대로)
    huphy_squat        학습 쪽 export_deploy.py 가 만든 계약 그대로. 참조 입출력 24쌍
"""

import json
import math
import shutil
from pathlib import Path

import numpy as np
import pytest

from huphy.control import deploy
from huphy.sensors.base import ImuState

POLICIES = Path(__file__).resolve().parent.parent / "config" / "policies"
WALK = deploy.load(POLICIES / "generalized_walk")
SQUAT = deploy.load(POLICIES / "huphy_squat")


class FakeImu:
    def __init__(self, state=None):
        self.state = state or ImuState(gyro_dps=(0.0, 0.0, 0.0), gravity=(0.0, 0.0, -1.0))

    def read(self):
        return self.state


def at_pose(policy, shift=0.0):
    return {f"{j}.pos": v + shift for j, v in policy.default_pose_deg.items()}


# ===========================================================================
# 관절 이름
# ===========================================================================
class TestNames:
    @pytest.mark.parametrize("contract, robot", [
        ("R_hip_pitch_joint", "right_leg/hip_pitch"),
        ("L_knee_joint", "left_leg/knee"),
        ("R_crank_A_joint", "right_leg/ankle_a"),
        ("L_crank_B_joint", "left_leg/ankle_b"),
    ])
    def test_contract_names_map_to_robot_names(self, contract, robot):
        assert deploy.robot_name(contract) == robot

    @pytest.mark.parametrize("bad", ["R_ankle_pitch_joint", "X_knee_joint", "R_knee"])
    def test_unknown_names_are_refused(self, bad):
        with pytest.raises(deploy.ContractError):
            deploy.robot_name(bad)


# ===========================================================================
# 계약 읽기
# ===========================================================================
class TestWalkContract:
    """옛 `control/generalized_walk.py` 상수와 같은 값이어야 함."""

    def test_shape(self):
        assert WALK.obs_dim == 44 and len(WALK.order) == 12 and WALK.hz == 50.0
        assert WALK.order[0] == "right_leg/hip_pitch" and WALK.order[6] == "left_leg/hip_pitch"

    def test_default_pose_is_the_old_table(self):
        assert WALK.default_pose_deg["right_leg/hip_pitch"] == pytest.approx(-10.0268)
        assert WALK.default_pose_deg["right_leg/knee"] == pytest.approx(20.0535)
        assert WALK.default_pose_deg["left_leg/ankle_b"] == pytest.approx(8.4150)

    def test_limits_are_the_old_table(self):
        assert WALK.limits_deg["right_leg/knee"] == pytest.approx((6.0, 114.0003))
        assert WALK.limits_deg["left_leg/ankle_a"] == pytest.approx((-61.8794, 61.8794))

    def test_commands_are_vx_wz_without_limits(self):
        assert sorted(WALK.commands) == ["vx", "wz"]
        assert all(c.max_rate is None and c.start == 0.0 for c in WALK.commands.values())

    def test_obs_names_keep_the_old_log_columns(self):
        assert WALK.obs_names[:3] == ("obs/gyro/x", "obs/gyro/y", "obs/gyro/z")
        assert WALK.obs_names[6] == "obs/q_prev/right_leg/hip_pitch"
        assert WALK.obs_names[18] == "obs/q/right_leg/hip_pitch"
        assert WALK.obs_names[-2:] == ("obs/cmd/vx", "obs/cmd/wz")
        assert len(set(WALK.obs_names)) == 44


class TestSquatContract:
    def test_shape(self):
        assert SQUAT.obs_dim == 43 and len(SQUAT.order) == 12 and SQUAT.hz == 50.0
        assert SQUAT.order == WALK.order

    def test_default_pose_differs_from_walk(self):
        """README 경고 표: hip roll/yaw, 크랭크가 다름. hip pitch/knee 는 같음."""
        assert SQUAT.default_pose_deg["right_leg/hip_roll"] == pytest.approx(math.degrees(0.05))
        assert SQUAT.default_pose_deg["right_leg/ankle_a"] == pytest.approx(math.degrees(0.106983))
        assert SQUAT.default_pose_deg["right_leg/knee"] == pytest.approx(
            WALK.default_pose_deg["right_leg/knee"], abs=1e-3)

    def test_crank_clip_is_narrower(self):
        assert SQUAT.limits_deg["right_leg/ankle_a"] == pytest.approx(
            (-math.degrees(0.95), math.degrees(0.95)))

    def test_h_star(self):
        h = SQUAT.commands["h_star"]
        assert h.start == pytest.approx(0.8914)
        assert h.range == (0.65, 0.895)
        assert h.max_rate == pytest.approx(0.1)


# ===========================================================================
# 가중치 — 모터를 켜기 전 검사
# ===========================================================================
class TestLoadModel:
    @pytest.mark.parametrize("checkpoint", ["5000", "4500", "4000"])
    def test_squat_matches_the_training_golden(self, checkpoint):
        model = SQUAT.load_model(checkpoint)
        assert model.golden_max_diff < deploy.GOLDEN_TOL

    def test_default_checkpoint(self):
        assert SQUAT.load_model().checkpoint == "5000"
        assert WALK.load_model().checkpoint == "37400"

    def _copy(self, tmp_path, name):
        dst = tmp_path / name
        shutil.copytree(POLICIES / name, dst)
        return dst

    def test_a_changed_weights_file_is_refused(self, tmp_path):
        folder = self._copy(tmp_path, "generalized_walk")
        pt = folder / "generalized_walk.pt"
        pt.write_bytes(pt.read_bytes() + b"\0")
        with pytest.raises(deploy.ContractError, match="sha256"):
            deploy.load(folder).load_model()

    def test_a_wrong_golden_is_refused(self, tmp_path):
        folder = self._copy(tmp_path, "huphy_squat")
        path = folder / deploy.CONTRACT_NAME
        c = json.loads(path.read_text())
        c["checkpoints"]["5000"]["golden"]["actions"][0][0] += 0.1
        path.write_text(json.dumps(c))
        with pytest.raises(deploy.ContractError, match="참조 입출력"):
            deploy.load(folder).load_model("5000")

    def test_an_unknown_observation_is_refused(self, tmp_path):
        """모르는 칸을 0 으로 채우지 않고 멈춤."""
        folder = self._copy(tmp_path, "generalized_walk")
        path = folder / deploy.CONTRACT_NAME
        c = json.loads(path.read_text())
        c["observations"][0]["func"] = "foot_contact"
        path.write_text(json.dumps(c))
        with pytest.raises(deploy.ContractError, match="모르는 관찰 항목"):
            deploy.load(folder)


# ===========================================================================
# 한 주기
# ===========================================================================
class TestStep:
    def test_zero_action_gives_the_default_pose(self):
        targets = SQUAT.joint_targets([0.0] * 12)
        for j, v in SQUAT.default_pose_deg.items():
            assert targets[j] == pytest.approx(v)

    def test_targets_are_clipped(self):
        targets = SQUAT.joint_targets([100.0] * 12)
        assert targets["right_leg/ankle_a"] == pytest.approx(math.degrees(0.95))

    def test_first_cycle_fills_history_with_the_current_pose(self):
        """`q(t-1) = q(t)` -- 학습의 리셋과 같음."""
        seen = []
        runner = SQUAT.runner(lambda v: seen.append(v) or np.zeros(12))
        runner.step(at_pose(SQUAT, 1.0), FakeImu().read())
        v = seen[0]
        assert v[6:18] == pytest.approx(v[18:30])
        assert v[18] == pytest.approx(math.radians(1.0), abs=1e-6)

    def test_second_cycle_shifts_history(self):
        seen = []
        runner = SQUAT.runner(lambda v: seen.append(v) or np.zeros(12))
        runner.step(at_pose(SQUAT, 1.0), FakeImu().read())
        runner.step(at_pose(SQUAT, 2.0), FakeImu().read())
        v = seen[1]
        assert v[6] == pytest.approx(math.radians(1.0), abs=1e-6)
        assert v[18] == pytest.approx(math.radians(2.0), abs=1e-6)

    def test_last_action_is_fed_back_raw(self):
        seen = []
        runner = SQUAT.runner(lambda v: seen.append(v) or np.full(12, 9.0))
        runner.step(at_pose(SQUAT), FakeImu().read())
        runner.step(at_pose(SQUAT), FakeImu().read())
        assert seen[0][30:42] == pytest.approx(np.zeros(12))
        assert seen[1][30:42] == pytest.approx(np.full(12, 9.0))   # 자르기 전 값

    def test_gyro_goes_in_as_radians(self):
        seen = []
        runner = SQUAT.runner(lambda v: seen.append(v) or np.zeros(12))
        imu = ImuState(gyro_dps=(180.0, 0.0, 0.0), gravity=(0.0, 0.0, -1.0))
        runner.step(at_pose(SQUAT), imu)
        assert seen[0][0] == pytest.approx(math.pi, abs=1e-5)
        assert seen[0][3:6] == pytest.approx([0.0, 0.0, -1.0])


# ===========================================================================
# 명령
# ===========================================================================
class TestCommands:
    def test_h_star_ramps_at_the_max_rate(self):
        cmds = deploy.Commands(SQUAT.commands.values(), {"h_star": 0.80})
        assert cmds.values["h_star"] == pytest.approx(0.8914)
        cmds.step(0.02)
        assert cmds.values["h_star"] == pytest.approx(0.8914 - 0.002)
        for _ in range(100):
            cmds.step(0.02)
        assert cmds.values["h_star"] == pytest.approx(0.80)

    def test_h_star_reaches_the_observation(self):
        seen = []
        cmds = deploy.Commands(SQUAT.commands.values(), {"h_star": 0.80})
        runner = SQUAT.runner(lambda v: seen.append(v) or np.zeros(12), commands=cmds)
        runner.step(at_pose(SQUAT), FakeImu().read())
        assert seen[0][42] == pytest.approx(0.8914 - 0.002)

    def test_h_star_is_clipped_into_the_training_range(self):
        seen = []
        cmds = deploy.Commands(SQUAT.commands.values())
        cmds.values["h_star"] = cmds.targets["h_star"] = 0.5
        runner = SQUAT.runner(lambda v: seen.append(v) or np.zeros(12), commands=cmds)
        runner.step(at_pose(SQUAT), FakeImu().read())
        assert seen[0][42] == pytest.approx(0.65)

    def test_velocity_commands_apply_at_once(self):
        cmds = deploy.Commands(WALK.commands.values(), {"vx": 0.5})
        assert cmds.step(0.02) == {"vx": 0.5, "wz": 0.0}

    def test_unknown_command_is_refused(self):
        with pytest.raises(deploy.ContractError, match="이 정책에 없는 명령"):
            deploy.Commands(WALK.commands.values(), {"h_star": 0.8})
