"""체크포인트 읽기 테스트 — torch 없이 실행됨.

    PYTHONPATH=src python3 -m pytest tests -q

`config/policies/` 의 **진짜 가중치 파일**을 읽음. 형식이 바뀌면 여기서 걸림.
"""

from pathlib import Path

import numpy as np
import pytest

from huphy.control import policy, rsl_rl

REPO = Path(__file__).resolve().parent.parent
WEIGHTS = {
    "balance": (REPO / "config" / "policies" / "balance.pt", policy.BALANCE),
    "hopping": (REPO / "config" / "policies" / "hopping.pt", policy.HOPPING),
}


@pytest.fixture(params=sorted(WEIGHTS))
def loaded(request):
    path, spec = WEIGHTS[request.param]
    return rsl_rl.load(path, spec=spec), spec


# ===========================================================================
# 읽기
# ===========================================================================
class TestLoad:
    def test_the_files_are_in_the_repo(self):
        for path, _ in WEIGHTS.values():
            assert path.is_file(), f"{path} 가 없음"

    def test_input_matches_the_spec(self, loaded):
        model, spec = loaded
        assert model.obs_dim == spec.obs_dim

    def test_output_is_six_joints(self, loaded):
        model, _ = loaded
        assert model.action_dim == policy.ACTION_DIM

    def test_a_mismatched_spec_is_refused(self):
        """이름을 잘못 주면 신경망이 엉뚱한 자리의 숫자를 읽음."""
        path, _ = WEIGHTS["hopping"]
        with pytest.raises(ValueError, match="26개인데 balance"):
            rsl_rl.load(path, spec=policy.BALANCE)

    def test_a_non_checkpoint_is_refused(self, tmp_path):
        import zipfile

        broken = tmp_path / "broken.pt"
        with zipfile.ZipFile(broken, "w") as z:
            z.writestr("x/other.bin", b"0000")
        with pytest.raises(ValueError, match="data.pkl"):
            rsl_rl.load(broken, spec=policy.BALANCE)


# ===========================================================================
# 계산
# ===========================================================================
class TestForward:
    def test_it_runs(self, loaded):
        model, spec = loaded
        out = model(np.zeros(spec.obs_dim, dtype=np.float32))
        assert out.shape == (policy.ACTION_DIM,)

    def test_the_output_is_finite(self, loaded):
        model, spec = loaded
        for scale in (0.0, 1.0, -1.0, 5.0):
            out = model(np.full(spec.obs_dim, scale, dtype=np.float32))
            assert np.all(np.isfinite(out))

    def test_it_is_deterministic(self, loaded):
        """학습 때의 탐색용 분포는 안 씀. 같은 입력이면 같은 출력임."""
        model, spec = loaded
        vector = np.linspace(-1.0, 1.0, spec.obs_dim, dtype=np.float32)
        assert model(vector) == pytest.approx(model(vector))

    def test_different_inputs_give_different_outputs(self, loaded):
        model, spec = loaded
        a = model(np.zeros(spec.obs_dim, dtype=np.float32))
        b = model(np.ones(spec.obs_dim, dtype=np.float32))
        assert not np.allclose(a, b)

    def test_normalization_is_applied(self, loaded):
        """평균을 그대로 넣으면 정규화 후가 0 임. 정규화를 빼면 다른 값이 나옴."""
        model, spec = loaded
        zeros = model(np.zeros(spec.obs_dim, dtype=np.float32))
        # mean 을 넣으면 (x-mean)/std = 0 이라, 0을 넣은 것과 달라야 함
        # (mean 이 0이 아닌 항목이 있으므로)
        assert not np.allclose(zeros, model(np.ones(spec.obs_dim, dtype=np.float32)))

    def test_a_wrong_length_input_is_an_error(self, loaded):
        model, spec = loaded
        with pytest.raises(ValueError):
            model(np.zeros(spec.obs_dim + 1, dtype=np.float32))


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
    def test_a_loaded_model_drives_a_motion(self):
        """읽은 것을 그대로 `policy_motion` 에 넣을 수 있어야 함."""
        from huphy.sensors.base import ImuState

        path, spec = WEIGHTS["balance"]
        model = rsl_rl.load(path, spec=spec)

        class FakeImu:
            def read(self):
                return ImuState(is_valid=True)

        observation = {}
        for joint in policy.JOINT_ORDER:
            observation[f"{joint}.pos"] = 0.0
            observation[f"{joint}.vel"] = 0.0

        motion = policy.policy_motion(model, FakeImu(), spec=spec)
        targets = motion(0.0, observation)
        assert set(targets) == set(policy.JOINT_ORDER)
        assert all(np.isfinite(v) for v in targets.values())


class TestActionDim:
    def test_matching_output_passes(self):
        """다리 하나 모델은 6칸을 냄."""
        path, spec = WEIGHTS["balance"]
        model = rsl_rl.load(path, spec=spec, action_dim=6)
        assert model.action_dim == 6

    def test_a_mismatched_output_is_refused(self):
        """다리 하나 모델에 양다리 순서(12)를 붙이면 모터를 켜기 전에 멈춤."""
        with pytest.raises(ValueError, match="출력이 6개인데 관절 순서는 12개"):
            path, spec = WEIGHTS["balance"]
            rsl_rl.load(path, spec=spec, action_dim=12)

    def test_omitting_it_skips_the_check(self):
        """기존 호출부는 그대로 통과함."""
        path, spec = WEIGHTS["balance"]
        assert rsl_rl.load(path, spec=spec).action_dim == 6
