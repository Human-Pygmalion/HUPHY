"""정책 입출력 기록 테스트 — 하드웨어도 가중치 파일도 없이 실행됨.

    PYTHONPATH=src python3 -m pytest tests/test_policy_log.py -q

확인하는 것은 **칸 이름이 관찰 벡터와 같은 길이인지**와 **기록이 제어를 막지
않는지**임.
"""

import csv
from pathlib import Path

import numpy as np

from huphy.control import deploy

WALK = deploy.load(Path(__file__).resolve().parent.parent / "config" / "policies" / "generalized_walk")
"""걷기 정책 계약. 관절 순서·기본 자세·관찰 구성을 여기서 읽음."""
from huphy.sensors.base import ImuState
from huphy.telemetry.policy_log import PolicyLog, default_path


class FakeImu:
    def read(self) -> ImuState:
        return ImuState(gyro_dps=(0.0, 0.0, 0.0), gravity=(0.0, 0.0, -1.0))


def gw_observation():
    return {f"{j}.pos": float(WALK.default_pose_deg[j]) for j in WALK.order}


class TestNames:
    def test_names_match_obs_dim(self):
        assert len(WALK.obs_names) == WALK.obs_dim
        assert len(set(WALK.obs_names)) == WALK.obs_dim


class TestLog:
    def run_motion(self, log, cycles=3):
        model = log.wrap_model(lambda v: np.full(len(WALK.order), 0.1))
        motion = log.wrap_motion(WALK.motion(model, FakeImu()))
        for k in range(cycles):
            motion(k * 0.02, gw_observation())

    def test_one_row_per_policy_call(self, tmp_path):
        log = PolicyLog(WALK.obs_names, WALK.order)
        self.run_motion(log)
        assert len(log) == 3
        path = log.save(tmp_path / "x" / "log.csv")
        rows = list(csv.reader(path.open()))
        assert rows[0][0] == "t"
        assert len(rows[0]) == 1 + WALK.obs_dim + 12 * 3
        assert all(len(r) == len(rows[0]) for r in rows)
        header = rows[0]
        knee = header.index("tgt/right_leg/knee")
        expected = WALK.joint_targets([0.1] * 12)["right_leg/knee"]
        assert abs(float(rows[1][knee]) - expected) < 1e-3

    def test_wrong_size_is_counted_not_raised(self):
        log = PolicyLog(("a",), WALK.order)
        self.run_motion(log, cycles=2)
        assert log.errors == 2
        assert len(log) == 0

    def test_default_path(self, tmp_path):
        """`<월일>_policy_<이름>[_<인자들>].csv` (`telemetry/run_name.py`)."""
        path = default_path(tmp_path, "generalized_walk")
        assert path.parent == tmp_path
        assert path.name.endswith("_policy_generalized_walk.csv")
        assert path.name[:4].isdigit(), "월일로 시작해야 함"

    def test_default_path_carries_the_arguments(self, tmp_path):
        path = default_path(tmp_path, "generalized_walk", ["g0.3", "vx0.3"])
        assert path.name.endswith("_policy_generalized_walk_g0.3_vx0.3.csv")
