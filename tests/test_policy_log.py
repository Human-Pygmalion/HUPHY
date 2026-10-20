"""정책 입출력 기록 테스트 — 하드웨어도 가중치 파일도 없이 실행됨.

    PYTHONPATH=src python3 -m pytest tests/test_policy_log.py -q

확인하는 것은 **칸 이름이 관찰 벡터와 같은 길이인지**와 **기록이 제어를 막지
않는지**임.
"""

import csv

import numpy as np

from huphy.control import generalized_walk
from huphy.sensors.base import ImuState
from huphy.telemetry.policy_log import PolicyLog, default_path


class FakeImu:
    def read(self) -> ImuState:
        return ImuState(gyro_dps=(0.0, 0.0, 0.0), gravity=(0.0, 0.0, -1.0))


def gw_observation():
    return {f"{j}.pos": float(generalized_walk.DEFAULT_POSE_DEG[j]) for j in generalized_walk.ORDER}


class TestNames:
    def test_names_match_obs_dim(self):
        assert len(generalized_walk.OBS_NAMES) == generalized_walk.OBS_DIM
        assert len(set(generalized_walk.OBS_NAMES)) == generalized_walk.OBS_DIM


class TestLog:
    def run_motion(self, log, cycles=3):
        model = log.wrap_model(lambda v: np.full(len(generalized_walk.ORDER), 0.1))
        motion = log.wrap_motion(generalized_walk.motion(model, FakeImu()))
        for k in range(cycles):
            motion(k * 0.02, gw_observation())

    def test_one_row_per_policy_call(self, tmp_path):
        log = PolicyLog(generalized_walk.OBS_NAMES, generalized_walk.ORDER)
        self.run_motion(log)
        assert len(log) == 3
        path = log.save(tmp_path / "x" / "log.csv")
        rows = list(csv.reader(path.open()))
        assert rows[0][0] == "t"
        assert len(rows[0]) == 1 + generalized_walk.OBS_DIM + 12 * 3
        assert all(len(r) == len(rows[0]) for r in rows)
        header = rows[0]
        knee = header.index("tgt/right_leg/knee")
        expected = generalized_walk.joint_targets([0.1] * 12)["right_leg/knee"]
        assert abs(float(rows[1][knee]) - expected) < 1e-3

    def test_wrong_size_is_counted_not_raised(self):
        log = PolicyLog(("a",), generalized_walk.ORDER)
        self.run_motion(log, cycles=2)
        assert log.errors == 2
        assert len(log) == 0

    def test_default_path(self, tmp_path):
        path = default_path(tmp_path, "generalized_walk")
        assert path.parent == tmp_path
        assert path.name.startswith("policy_generalized_walk_") and path.suffix == ".csv"
