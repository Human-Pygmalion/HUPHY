"""체크포인트 읽기 테스트 — torch 없이 실행됨.

    PYTHONPATH=src python3 -m pytest tests -q

`config/policies/` 의 **진짜 가중치 파일**을 읽음. 형식이 바뀌면 여기서 걸림.
"""

from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from huphy.control import deploy, rsl_rl

REPO = Path(__file__).resolve().parent.parent
WALK = deploy.load(REPO / "config" / "policies" / "generalized_walk")
WEIGHTS = WALK.weights_path()
SPEC = WALK.spec


@pytest.fixture(scope="module")
def loaded():
    return rsl_rl.load(WEIGHTS, spec=SPEC)


# ===========================================================================
# 읽기
# ===========================================================================
class TestLoad:
    def test_the_file_is_in_the_repo(self):
        assert WEIGHTS.is_file(), f"{WEIGHTS} 가 없음"

    def test_input_matches_the_spec(self, loaded):
        assert loaded.obs_dim == SPEC.obs_dim

    def test_output_is_twelve_joints(self, loaded):
        assert loaded.action_dim == len(WALK.order)

    def test_a_mismatched_spec_is_refused(self):
        """입력 개수가 다른 규격을 주면 신경망이 엉뚱한 자리의 숫자를 읽음."""
        wrong = replace(SPEC, obs_dim=SPEC.obs_dim + 1, name="wrong")
        with pytest.raises(ValueError, match=f"{SPEC.obs_dim}개인데 wrong"):
            rsl_rl.load(WEIGHTS, spec=wrong)

    def test_a_non_checkpoint_is_refused(self, tmp_path):
        import zipfile

        broken = tmp_path / "broken.pt"
        with zipfile.ZipFile(broken, "w") as z:
            z.writestr("x/other.bin", b"0000")
        with pytest.raises(ValueError, match="data.pkl"):
            rsl_rl.load(broken, spec=SPEC)


# ===========================================================================
# 계산
# ===========================================================================
class TestForward:
    def test_it_runs(self, loaded):
        out = loaded(np.zeros(SPEC.obs_dim, dtype=np.float32))
        assert out.shape == (len(WALK.order),)

    def test_the_output_is_finite(self, loaded):
        for scale in (0.0, 1.0, -1.0, 5.0):
            out = loaded(np.full(SPEC.obs_dim, scale, dtype=np.float32))
            assert np.all(np.isfinite(out))

    def test_it_is_deterministic(self, loaded):
        """학습 때의 탐색용 분포는 안 씀. 같은 입력이면 같은 출력임."""
        vector = np.linspace(-1.0, 1.0, SPEC.obs_dim, dtype=np.float32)
        assert loaded(vector) == pytest.approx(loaded(vector))

    def test_different_inputs_give_different_outputs(self, loaded):
        a = loaded(np.zeros(SPEC.obs_dim, dtype=np.float32))
        b = loaded(np.ones(SPEC.obs_dim, dtype=np.float32))
        assert not np.allclose(a, b)

    def test_normalization_is_applied(self, loaded):
        """정규화가 빠지면 0 벡터와 1 벡터가 같은 답을 낼 리가 없음(평균이 0이
        아닌 항목이 있으므로) -- 이게 그 반대(서로 다름)를 확인함."""
        zeros = loaded(np.zeros(SPEC.obs_dim, dtype=np.float32))
        assert not np.allclose(zeros, loaded(np.ones(SPEC.obs_dim, dtype=np.float32)))

    def test_a_wrong_length_input_is_an_error(self, loaded):
        with pytest.raises(ValueError):
            loaded(np.zeros(SPEC.obs_dim + 1, dtype=np.float32))


class TestElu:
    """학습 설정의 `activation="elu"` 임."""

    def test_positive_passes_through(self):
        got = rsl_rl._elu(np.array([0.5, 2.0], dtype=np.float32))
        assert got == pytest.approx([0.5, 2.0])

    def test_negative_saturates_at_minus_one(self):
        got = rsl_rl._elu(np.array([-100.0], dtype=np.float32))
        assert got[0] == pytest.approx(-1.0)

    def test_zero_is_zero(self):
        assert rsl_rl._elu(np.array([0.0]))[0] == pytest.approx(0.0)

    def test_it_is_continuous_at_zero(self):
        below = rsl_rl._elu(np.array([-1e-6]))[0]
        above = rsl_rl._elu(np.array([1e-6]))[0]
        assert abs(below - above) < 1e-5


# ===========================================================================
# 정책과 이어 붙이기
# ===========================================================================
class TestWithPolicy:
    def test_a_loaded_model_drives_a_motion(self, loaded):
        """읽은 것을 그대로 `DeployPolicy.motion` 에 넣을 수 있어야 함."""
        from huphy.sensors.base import ImuState

        class FakeImu:
            def read(self):
                return ImuState(is_valid=True)

        observation = {
            f"{joint}.pos": WALK.default_pose_deg[joint]
            for joint in WALK.order
        }

        motion = WALK.motion(loaded, FakeImu())
        targets = motion(0.0, observation)
        assert set(targets) == set(WALK.order)
        assert all(np.isfinite(v) for v in targets.values())


class TestActionDim:
    def test_matching_output_passes(self, loaded):
        model = rsl_rl.load(WEIGHTS, spec=SPEC, action_dim=12)
        assert model.action_dim == 12

    def test_a_mismatched_output_is_refused(self):
        """출력 개수와 관절 순서 길이가 다르면 모터를 켜기 전에 멈춤."""
        with pytest.raises(ValueError, match="출력이 12개인데 관절 순서는 6개"):
            rsl_rl.load(WEIGHTS, spec=SPEC, action_dim=6)

    def test_omitting_it_skips_the_check(self):
        """기존 호출부는 그대로 통과함."""
        assert rsl_rl.load(WEIGHTS, spec=SPEC).action_dim == 12
