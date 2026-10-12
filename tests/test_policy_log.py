"""정책 입출력 기록 테스트 — 하드웨어도 가중치 파일도 없이 실행됨.

    PYTHONPATH=src python3 -m pytest tests/test_policy_log.py -q

확인하는 것은 **칸 이름이 관찰 벡터와 같은 길이인지**와 **기록이 제어를 막지
않는지**임.
"""

import csv

import numpy as np

from huphy.control import policy, stand
from huphy.sensors.base import ImuState
from huphy.telemetry.policy_log import PolicyLog, default_path


class FakeImu:
    def read(self) -> ImuState:
        return ImuState(gyro_dps=(0.0, 0.0, 0.0), gravity=(0.0, 0.0, -1.0))


def stand_observation():
    return {f"{j}.pos": float(stand.DEFAULT_POSE_DEG[j]) for j in stand.ORDER}


class TestNames:
    def test_stand_names_match_obs_dim(self):
        assert len(stand.OBS_NAMES) == stand.OBS_DIM
        assert len(set(stand.OBS_NAMES)) == stand.OBS_DIM

    def test_policy_names_match_obs_dim(self):
        for spec in (policy.BALANCE, policy.HOPPING):
            assert len(policy.observation_names(spec)) == spec.obs_dim
        order = policy.BIPED_ORDERS["ab"]
        biped = policy.for_joints(policy.BALANCE, len(order))
        assert len(policy.observation_names(biped, order)) == biped.obs_dim


class TestLog:
    def run_stand(self, log, cycles=3):
        model = log.wrap_model(lambda v: np.full(len(stand.ORDER), 0.1))
        motion = log.wrap_motion(stand.motion(model, FakeImu()))
        for k in range(cycles):
            motion(k * 0.02, stand_observation())

    def test_one_row_per_policy_call(self, tmp_path):
        log = PolicyLog(stand.OBS_NAMES, stand.ORDER)
        self.run_stand(log)
        assert len(log) == 3
        path = log.save(tmp_path / "x" / "log.csv")
        rows = list(csv.reader(path.open()))
        assert rows[0][0] == "t"
        assert len(rows[0]) == 1 + 45 + 12 * 3
        assert all(len(r) == len(rows[0]) for r in rows)
        header = rows[0]
        knee = header.index("tgt/right_leg/knee")
        expected = stand.joint_targets([0.1] * 12)["right_leg/knee"]
        assert abs(float(rows[1][knee]) - expected) < 1e-3

    def test_wrong_size_is_counted_not_raised(self):
        log = PolicyLog(("a",), stand.ORDER)
        self.run_stand(log, cycles=2)
        assert log.errors == 2
        assert len(log) == 0

    def test_default_path(self, tmp_path):
        path = default_path(tmp_path, "stand")
        assert path.parent == tmp_path
        assert path.name.startswith("policy_stand_") and path.suffix == ".csv"
