"""정책 입출력 기록 테스트 — 하드웨어도 가중치 파일도 없이 실행됨.

    PYTHONPATH=src python3 -m pytest tests/test_policy_log.py -q

확인하는 것은 **칸 이름이 관찰 벡터와 같은 길이인지**와 **기록이 제어를 막지
않는지**임.
"""

import json
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
        doc = json.loads(log.save(tmp_path / "x" / "log.json").read_text())
        cols = doc["columns"]
        assert cols[0] == "t"
        assert len(cols) == 1 + WALK.obs_dim + 12 * 3
        assert all(len(r) == len(cols) for r in doc["rows"])
        knee = cols.index("tgt/right_leg/knee")
        expected = WALK.joint_targets([0.1] * 12)["right_leg/knee"]
        assert abs(doc["rows"][0][knee] - expected) < 1e-3

    def test_the_run_section_is_written(self, tmp_path):
        """이름에서 뺀 값이 `run` 에 들어감 (`telemetry/run_name.py`)."""
        log = PolicyLog(WALK.obs_names, WALK.order)
        self.run_motion(log)
        doc = json.loads(log.save(tmp_path / "x.json", {"policy": "w", "gain": 0.3}).read_text())
        assert doc["run"] == {"policy": "w", "gain": 0.3}

    def test_wrong_size_is_counted_not_raised(self):
        log = PolicyLog(("a",), WALK.order)
        self.run_motion(log, cycles=2)
        assert log.errors == 2
        assert len(log) == 0

    def test_default_path(self, tmp_path):
        """`<월일>_policy_<이름>[_<인자들>].csv` (`telemetry/run_name.py`)."""
        path = default_path(tmp_path, "generalized_walk")
        assert path.parent == tmp_path / "policy", "종류별 폴더로 나뉨"
        assert path.name.endswith("_policy_generalized_walk.json")
        assert path.name[:4].isdigit(), "월일로 시작해야 함"

    def test_default_path_carries_the_arguments(self, tmp_path):
        path = default_path(tmp_path, "generalized_walk", ["g0.3", "vx0.3"])
        assert path.name.endswith("_policy_generalized_walk_g0.3_vx0.3.json")


class TestOutsideEntry:
    """감싸지 않고 밖에서 넣는 길 (`add_row`).

    ROS `log_node` 가 쓰는 입구임 -- 모델을 안 부르고 `PolicyRow` 토픽으로 완성된
    줄을 받음. **감싸서 모은 것과 같은 파일이 나와야** `huphy-run` 기록과 비교할
    수 있음.
    """

    def test_the_file_matches_what_wrapping_collects(self, tmp_path):
        wrapped = PolicyLog(WALK.obs_names, WALK.order)
        model = wrapped.wrap_model(lambda v: np.full(len(WALK.order), 0.1))
        motion = wrapped.wrap_motion(WALK.motion(model, FakeImu()))
        for k in range(3):
            motion(k * 0.02, gw_observation())
        by_wrap = json.loads(wrapped.save(tmp_path / "wrap.json").read_text())

        outside = PolicyLog(WALK.obs_names, WALK.order)
        n_obs, n_act = WALK.obs_dim, len(WALK.order)
        for row in wrapped.rows:
            outside.add_row(
                row[0],
                row[1:1 + n_obs],
                row[1 + n_obs:1 + n_obs + n_act],
                row[1 + n_obs + n_act:1 + n_obs + 2 * n_act],
                row[1 + n_obs + 2 * n_act:1 + n_obs + 3 * n_act],
            )
        by_topic = json.loads(outside.save(tmp_path / "topic.json").read_text())

        assert by_topic["columns"] == by_wrap["columns"]
        assert by_topic["rows"] == by_wrap["rows"]

    def test_t_is_taken_as_given(self):
        """부르는 쪽이 찍은 시각을 그대로 씀 -- 추론한 기계의 주기 지터가 남아야 함."""
        log = PolicyLog(WALK.obs_names, WALK.order)
        for t in (0.0, 0.02, 0.041):
            log.add_row(t, [0.0] * WALK.obs_dim, [0.0] * 12, [0.0] * 12, [0.0] * 12)
        assert [r[0] for r in log.rows] == [0.0, 0.02, 0.041]

    def test_a_wrong_length_stops_instead_of_shifting_columns(self):
        log = PolicyLog(WALK.obs_names, WALK.order)
        import pytest
        with pytest.raises(ValueError, match="obs"):
            log.add_row(0.0, [0.0] * 3, [0.0] * 12, [0.0] * 12, [0.0] * 12)
        with pytest.raises(ValueError, match="target_deg"):
            log.add_row(0.0, [0.0] * WALK.obs_dim, [0.0] * 12, [0.0] * 5, [0.0] * 12)
        assert len(log) == 0

    def test_limits_can_be_given_without_a_robot(self, tmp_path):
        """`log_node` 는 모터가 다른 기계에 있어 캘리브레이션을 못 읽음."""
        log = PolicyLog(WALK.obs_names, WALK.order,
                        limits={"right_leg/knee": [6, 114]})
        log.add_row(0.0, [0.0] * WALK.obs_dim, [0.0] * 12, [0.0] * 12, [0.0] * 12)
        doc = json.loads(log.save(tmp_path / "x.json").read_text())
        assert doc["limits"] == {"right_leg/knee": [6.0, 114.0]}
