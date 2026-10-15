"""추종 기록 테스트 -- 하드웨어 없이 실행됨."""

import csv

from huphy.telemetry import track_log


def hold(targets):
    return lambda t, observation: dict(targets)


class TestTrackLog:
    def test_one_row_per_cycle(self):
        log = track_log.TrackLog()
        motion = log.wrap_motion(hold({"knee": 30.0, "hip_pitch": 0.0}))
        motion(0.0, {"knee.pos": 10.0, "hip_pitch.pos": 1.0})
        motion(0.01, {"knee.pos": 20.0, "hip_pitch.pos": 0.5})
        assert len(log) == 2
        assert log.columns == ("t", "tgt/knee", "tgt/hip_pitch", "pos/knee", "pos/hip_pitch")
        assert log.rows[1][1:] == [30.0, 0.0, 20.0, 0.5]

    def test_targets_pass_through(self):
        """기록은 관측일 뿐 -- 목표를 바꾸면 안 됨."""
        log = track_log.TrackLog()
        motion = log.wrap_motion(hold({"knee": 30.0}))
        assert motion(0.0, {}) == {"knee": 30.0}

    def test_missing_position_is_nan(self):
        log = track_log.TrackLog()
        log.wrap_motion(hold({"knee": 30.0}))(0.0, {})
        assert log.rows[0][2] != log.rows[0][2]       # NaN

    def test_a_broken_observation_does_not_stop_control(self):
        log = track_log.TrackLog()
        motion = log.wrap_motion(hold({"knee": 30.0}))
        assert motion(0.0, {"knee.pos": "x"}) == {"knee": 30.0}
        assert log.errors == 1

    def test_the_viewer_columns(self, tmp_path):
        """policy_viewer 가 읽는 모양: 첫 칸 t, 관절마다 tgt/ 와 pos/."""
        log = track_log.TrackLog()
        log.wrap_motion(hold({"right_leg/knee": 30.0}))(0.0, {"right_leg/knee.pos": 29.5})
        path = log.save(tmp_path / "track.csv")
        rows = list(csv.reader(path.open()))
        assert rows[0] == ["t", "tgt/right_leg/knee", "pos/right_leg/knee"]
        assert rows[1][1:] == ["30", "29.5"]

    def test_default_path(self, tmp_path):
        path = track_log.default_path(tmp_path, "test_zero")
        assert path.parent == tmp_path
        assert path.name.startswith("track_test_zero_") and path.suffix == ".csv"


class TestSentTargets:
    """목표는 가드를 거쳐 실제로 나간 값 (`robot.last_sent`)."""

    def test_the_clipped_value_is_recorded(self):
        from types import SimpleNamespace

        robot = SimpleNamespace(last_sent={})
        log = track_log.TrackLog(robot=robot)
        motion = log.wrap_motion(hold({"right_leg/knee": 0.0}))
        motion(0.0, {"right_leg/knee.pos": 20.0})
        assert len(log) == 0                      # 아직 아무것도 안 나감
        robot.last_sent = {"right_leg/knee": 9.0}  # 한계 6도 + 여유 3도로 잘림
        motion(0.02, {"right_leg/knee.pos": 8.76})
        assert log.rows[0][1:] == [9.0, 8.76]


class TestImu:
    """IMU 가 붙어 있으면 `imu/<이름>/<값>` 열이 붙음. 고른 값만."""

    def robot(self):
        from types import SimpleNamespace

        state = SimpleNamespace(
            gyro_dps=(1.0, 2.0, 3.0), accel_mps2=(0.0, 0.0, 9.8), gravity=(0.0, 0.0, -1.0),
            extra={"roll": 5.0, "pitch": -2.0, "yaw": 90.0, "qw": 1.0},
            age_ms=lambda now: 4.0,
        )
        imu = SimpleNamespace(name="main", extra_fields=("qw", "roll", "pitch", "yaw"))
        return SimpleNamespace(
            last_sent={"knee": 9.0}, imus=[imu], imu_states=lambda: {"main": state}
        )

    def test_imu_columns_follow_the_joints(self):
        log = track_log.TrackLog(robot=self.robot())
        log.wrap_motion(hold({"knee": 0.0}))(0.0, {"knee.pos": 8.7})
        row = dict(zip(log.columns, log.rows[0]))
        assert log.columns[:3] == ("t", "tgt/knee", "pos/knee")
        assert row["imu/main/roll"] == 5.0
        assert row["imu/main/gz"] == 3.0
        assert row["imu/main/grav_z"] == -1.0
        assert "imu/main/qw" not in row          # IMU_FIELDS 에 없는 것은 안 남김

    def test_no_imu_no_columns(self):
        log = track_log.TrackLog()
        log.wrap_motion(hold({"knee": 0.0}))(0.0, {"knee.pos": 8.7})
        assert log.columns == ("t", "tgt/knee", "pos/knee")


class TestObservationTap:
    """토크를 끈 관찰 모드(`huphy-test watch`). motion 없이 매 주기 관찰만 받음."""

    def test_rows_have_no_targets(self):
        from types import SimpleNamespace

        observation = {"knee.pos": 30.0, "hip_pitch.pos": 5.0}
        robot = SimpleNamespace(last_sent={}, get_observation=lambda: dict(observation))
        log = track_log.TrackLog(("hip_pitch", "knee"), robot=robot)
        tap = track_log.ObservationTap(robot, [log])
        tap.open()
        tap.record(loop_dt_ms=10.0)
        tap.close()
        assert log.columns == ("t", "pos/hip_pitch", "pos/knee")
        assert log.rows[0][1:] == [5.0, 30.0]

    def test_it_passes_through_to_real_telemetry(self):
        from types import SimpleNamespace

        calls = []
        inner = SimpleNamespace(
            open=lambda: calls.append("open"), close=lambda: calls.append("close"),
            record=lambda **kw: calls.append(("record", kw)),
        )
        robot = SimpleNamespace(get_observation=lambda: {})
        tap = track_log.ObservationTap(robot, [], inner=inner)
        tap.open(); tap.record(loop_dt_ms=1.0); tap.close()
        assert calls == ["open", ("record", {"loop_dt_ms": 1.0}), "close"]
