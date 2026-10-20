"""`config/policies/generalized_walk.pt` 민감도 테스트 — 실제 가중치로 돈다.

    PYTHONPATH=src python3 -m pytest tests/test_generalized_walk_sensitivity.py -q -v

가중치가 실제로 읽히는지, 입력을 바꾸면 출력도 바뀌는지 확인함.

여기서 확인하는 것:

    읽기        obs_dim=44, action_dim=12 로 읽히는가
    결정성      같은 입력 -> 같은 출력
    IMU 민감도   각속도/중력방향을 바꾸면 출력이 바뀌는가
    관절 민감도   관절 각도(이력 포함)를 바꾸면 출력이 바뀌는가
    경로 분리    두 민감도가 서로 다른 출력 칸에 다르게 영향을 주는가(완전히
                같은 방향으로만 움직이면 관찰 칸이 꼬여 들어갔다는 신호임)
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from huphy.control import generalized_walk as gw
from huphy.control import rsl_rl
from huphy.sensors.base import ImuState

REPO = Path(__file__).resolve().parent.parent
WEIGHTS = REPO / "config" / "policies" / "generalized_walk.pt"


@pytest.fixture(scope="module")
def model():
    return rsl_rl.load(WEIGHTS, spec=gw.SPEC, action_dim=len(gw.ORDER))


def _vector(pos=None, gyro=(0.0, 0.0, 0.0), gravity=(0.0, 0.0, -1.0),
            last_action=None, previous_pos=None, command=(0.0, 0.0)) -> np.ndarray:
    pos = pos if pos is not None else dict(gw.DEFAULT_POSE_DEG)
    observation = {f"{j}.pos": float(pos[j]) for j in gw.ORDER}
    imu = ImuState(gyro_dps=tuple(gyro), gravity=tuple(gravity), is_valid=True)
    last_action = last_action if last_action is not None else [0.0] * len(gw.ORDER)
    previous_pos = (
        previous_pos if previous_pos is not None
        else [gw.DEFAULT_POSE_DEG[j] for j in gw.ORDER]
    )
    return gw.observation_vector(observation, imu, last_action, previous_pos, command)


# ===========================================================================
# 읽기
# ===========================================================================
class TestLoad:
    def test_the_file_is_in_the_repo(self):
        assert WEIGHTS.is_file(), f"{WEIGHTS} 가 없음"

    def test_input_is_44(self, model):
        assert model.obs_dim == 44 == gw.OBS_DIM

    def test_output_is_12(self, model):
        assert model.action_dim == 12 == len(gw.ORDER)


# ===========================================================================
# 결정성 / 유한성
# ===========================================================================
class TestBasicBehavior:
    def test_output_is_finite_at_default_pose(self, model):
        out = model(_vector())
        assert out.shape == (12,)
        assert np.all(np.isfinite(out))

    def test_deterministic(self, model):
        v = _vector()
        a = model(v)
        b = model(v)
        assert a == pytest.approx(b)


# ===========================================================================
# IMU 민감도
# ===========================================================================
class TestImuSensitivity:
    """관찰 0~5칸(각속도 3 + 중력방향 3)만 바꿔서 출력이 움직이는지 봄."""

    def test_gyro_changes_output(self, model):
        base = model(_vector())
        perturbed = model(_vector(gyro=(30.0, 0.0, 0.0)))
        delta = np.linalg.norm(perturbed - base)
        assert delta > 1e-3, (
            f"각속도를 30deg/s 줘도 출력이 거의 안 바뀜(delta={delta}) -- "
            f"관찰 0~2칸이 신경망까지 안 이어졌을 수 있음"
        )

    def test_tilt_changes_output(self, model):
        """기울어진 중력방향(넘어지는 중과 같은 신호)에도 반응해야 함."""
        base = model(_vector())
        perturbed = model(_vector(gravity=(0.3, 0.0, -0.95)))
        delta = np.linalg.norm(perturbed - base)
        assert delta > 1e-3, (
            f"중력방향을 기울여도 출력이 거의 안 바뀜(delta={delta}) -- "
            f"관찰 3~5칸이 신경망까지 안 이어졌을 수 있음"
        )

    def test_gyro_and_tilt_move_output_differently(self, model):
        """두 민감도가 같은 벡터면(완전 비례) 두 입력이 안에서 섞여 들어간
        것일 수 있음 -- 방향이 달라야 각 칸이 독립적으로 작동한다는 신호임."""
        base = model(_vector())
        d_gyro = model(_vector(gyro=(30.0, 0.0, 0.0))) - base
        d_tilt = model(_vector(gravity=(0.3, 0.0, -0.95))) - base
        cos_sim = float(
            np.dot(d_gyro, d_tilt) / (np.linalg.norm(d_gyro) * np.linalg.norm(d_tilt) + 1e-9)
        )
        assert abs(cos_sim) < 0.999, (
            f"각속도 교란과 중력방향 교란이 사실상 같은 방향(cos={cos_sim:.4f})으로 "
            f"출력을 움직임 -- 두 관찰 칸이 어딘가에서 같은 가중치로 들어가고 "
            f"있을 가능성(관찰 순서 꼬임)을 의심할 것"
        )


# ===========================================================================
# 명령(vx, wz) 민감도 -- stand 와 달리 이 정책만 가진 칸
# ===========================================================================
class TestCommandSensitivity:
    def test_vx_changes_output(self, model):
        base = model(_vector())
        perturbed = model(_vector(command=(1.0, 0.0)))
        delta = np.linalg.norm(perturbed - base)
        assert delta > 1e-3, (
            f"vx 명령을 줘도 출력이 거의 안 바뀜(delta={delta}) -- "
            f"관찰 42칸이 신경망까지 안 이어졌을 수 있음"
        )

    def test_wz_changes_output(self, model):
        base = model(_vector())
        perturbed = model(_vector(command=(0.0, 0.5)))
        delta = np.linalg.norm(perturbed - base)
        assert delta > 1e-3, (
            f"wz 명령을 줘도 출력이 거의 안 바뀜(delta={delta}) -- "
            f"관찰 43칸이 신경망까지 안 이어졌을 수 있음"
        )


# ===========================================================================
# 관절 민감도
# ===========================================================================
class TestJointSensitivity:
    """관찰 6~29칸(이력 두 프레임)을 바꿔서 출력이 움직이는지 봄."""

    def test_current_position_changes_output(self, model):
        base = model(_vector())
        pos = dict(gw.DEFAULT_POSE_DEG)
        pos["right_leg/knee"] += 10.0
        perturbed = model(_vector(pos=pos))
        delta = np.linalg.norm(perturbed - base)
        assert delta > 1e-3, (
            f"무릎을 10도 틀어도 출력이 거의 안 바뀜(delta={delta}) -- "
            f"q(t) 관찰(18~29칸)이 안 이어졌을 수 있음"
        )

    def test_history_frame_changes_output_independently_of_current(self, model):
        """`previous_pos`(q(t-1), 6~17칸)만 바꿔도 출력이 바뀌어야 함 --
        현재값(q(t))과 같은 칸으로 섞여 있으면 둘을 동시에 반대로 바꿨을 때
        상쇄돼 버림(아래 보완 테스트)."""
        base = model(_vector())
        previous = [gw.DEFAULT_POSE_DEG[j] for j in gw.ORDER]
        previous[3] += 10.0  # right_leg/knee 의 '한 주기 전' 값만 바꿈
        perturbed = model(_vector(previous_pos=previous))
        delta = np.linalg.norm(perturbed - base)
        assert delta > 1e-3, (
            f"이력 프레임만 바꿔도 출력이 거의 안 바뀜(delta={delta}) -- "
            f"q(t-1) 관찰(6~17칸)이 안 이어졌을 수 있음"
        )

    def test_a_joint_outside_order_cannot_be_smuggled_in(self, model):
        """`observation.get(..., 0.0)` 기본값 때문에, ORDER 에 없는 관절 이름으로
        값을 넣어도 조용히 무시됨 -- 오타가 나도 에러가 안 난다는 뜻. 재현해 둠."""
        pos = {f"{j}.pos": float(gw.DEFAULT_POSE_DEG[j]) for j in gw.ORDER}
        pos["right_leg/hip_pitc.pos"] = 999.0  # 오타: hip_pitch 가 아님

        imu = ImuState(gravity=(0, 0, -1), gyro_dps=(0, 0, 0), is_valid=True)
        previous = [gw.DEFAULT_POSE_DEG[j] for j in gw.ORDER]
        vector = gw.observation_vector(pos, imu, [0.0] * 12, previous)
        # 오타 키는 그냥 안 읽히고, 진짜 키가 없으므로 0.0(기본자세 기준 상대각)이
        # 아니라 "관찰에 없음" 취급돼 0.0 - DEFAULT 가 들어감. 에러는 안 남.
        assert vector.shape == (44,)
