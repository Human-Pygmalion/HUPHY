"""전류계(WCM1600) 테스트 — 하드웨어 없이 실행됨.

    PYTHONPATH=src python3 -m pytest tests/test_wcm1600.py -q

`pyserial` 이 없어도 돌아야 함. `connect()` 안에서 import 하는 것이 그 때문임.

여기서 고정하는 것은 **자르는 자리와 열 이름**임. 자리수는 매뉴얼에서 옴.

    "1.23" A   ->  '0' '1' '2' '3' '0'
    "10.76" A  ->  '1' '0' '7' '6' '0'

앞 네 자리를 100 으로 나눈 값이고 다섯째 자리는 해상도 밖임. 틀려도 숫자가
그럴듯해서(12.30 A 같은 값) 실물에서 안 잡히는 종류의 오류라 여기서 못 박음.
"""

import math
import time

import pytest

from huphy.sensors.current import CurrentMeter, CurrentState
from huphy.sensors.wcm1600 import Wcm1600, protocol
from huphy.telemetry import power
from huphy.telemetry.policy_log import PolicyLog


# ===========================================================================
# 프레임 해석
# ===========================================================================
class TestParse:
    def test_the_manual_examples(self):
        assert protocol.parse(b"~01230") == (True, 1.23)
        assert protocol.parse(b"+01230") == (False, 1.23)
        assert protocol.parse(b"-01230") == (False, -1.23)
        assert protocol.parse(b"~10760") == (True, 10.76)
        assert protocol.parse(b"+10760") == (False, 10.76)
        assert protocol.parse(b"-10760") == (False, -10.76)

    def test_the_line_ending_is_optional(self):
        assert protocol.parse(b"+01230\r\n") == (False, 1.23)
        assert protocol.parse(b"  +01230  ") == (False, 1.23)

    def test_ac_is_a_magnitude_so_it_is_never_negative(self):
        for frame in (b"~00000", b"~99990"):
            is_ac, amps = protocol.parse(frame)
            assert is_ac and amps >= 0.0

    @pytest.mark.parametrize("frame", [
        b"+1230",       # 너무 짧음
        b"+012300",     # 너무 김
        b"*01230",      # 부호 문자가 아님
        b"+0123x",      # 숫자가 아님
        b"",
    ])
    def test_a_broken_frame_is_dropped(self, frame):
        assert protocol.parse(frame) is None

    def test_a_fifth_digit_that_is_not_zero_is_dropped(self):
        """매뉴얼은 이 자리가 항상 `'0'` 임. 아니면 다른 펌웨어이므로 안 읽음."""
        assert protocol.parse(b"+01231") is None
        assert protocol.parse(b"+01230") == (False, 1.23)


# ===========================================================================
# 리더
# ===========================================================================
class TestMeter:
    def test_before_any_frame_it_says_it_does_not_know(self):
        meter = Wcm1600()
        state = meter.read()
        assert not state.is_valid
        assert state.amps == 0.0            # "0 A" 가 아니라 "모름"
        assert state.age_ms() == -1.0       # 무한대는 JSON 으로 못 보냄

    def test_it_keeps_only_the_newest_frame(self):
        meter = Wcm1600()
        assert meter.feed(b"+01230\r\n")
        assert meter.feed(b"-02500\r\n")
        state = meter.read()
        assert state.is_valid and state.amps == -2.5
        assert meter.frames == 2 and meter.dropped == 0

    def test_a_broken_frame_is_counted_and_the_old_value_stays(self):
        meter = Wcm1600()
        meter.feed(b"+01230\r\n")
        assert not meter.feed(b"nope")
        assert meter.read().amps == 1.23    # 전류계 때문에 멈추지 않음
        assert meter.dropped == 1 and meter.frames == 1

    def test_age_grows_from_the_frame_not_from_the_read(self):
        meter = Wcm1600()
        meter.feed(b"+01230")
        now = meter.read().stamp + 0.05
        assert meter.read().age_ms(now) == pytest.approx(50.0)

    def test_it_satisfies_the_meter_protocol(self):
        assert isinstance(Wcm1600(), CurrentMeter)

    def test_it_is_not_an_imu(self):
        """IMU 가 아니므로 설정의 `imus` 표에 없음 (`sensors/registry.py`)."""
        from huphy.sensors import registry
        assert "wcm1600" not in registry.MODELS


# ===========================================================================
# 기록 열
# ===========================================================================
class TestColumns:
    def test_the_measured_column_sits_next_to_the_computed_one(self):
        log = PolicyLog(["obs/a"], ["j"], limits={},
                        extra_motors=["m"], power_motors=["m"], current=True)
        assert log.columns[-3:] == (
            "power/dc_a_lo", "current/total_a", "current/age")

    def test_without_the_meter_there_is_no_column(self):
        log = PolicyLog(["obs/a"], ["j"], limits={})
        assert not [c for c in log.columns if c.startswith("current/")]

    def test_the_column_stays_even_when_no_frame_arrived(self):
        """켜 놓고 값이 안 와도 열은 남음 -- 늦게 오는 값이 버려지지 않게."""
        log = PolicyLog(["obs/a"], ["j"], limits={}, current=True)
        log.add_row(0.0, [1.0], [0.1], [2.0], [2.1], {})
        row = dict(zip(log.columns, log.rows[0]))
        assert math.isnan(row["current/total_a"])

    def test_a_source_function_fills_the_row(self):
        """`huphy-run --current` 길. 줄마다 최근 값을 꺼냄."""
        state = CurrentState(4.25, False, time.monotonic(), True)
        log = PolicyLog(["obs/a"], ["j"], limits={}, current_source=lambda: state)
        model = log.wrap_model(lambda v: [0.5])
        motion = log.wrap_motion(lambda t, obs: {"j": 7.0})
        model([1.0])
        motion(0.0, {"j.pos": 6.5})
        row = dict(zip(log.columns, log.rows[0]))
        assert row["current/total_a"] == 4.25
        assert row["current/age"] >= 0.0

    def test_an_invalid_state_is_null_not_zero(self):
        values = power.current_values(CurrentState())
        assert math.isnan(values["current/total_a"])
        assert values["current/age"] == -1.0

    def test_the_ros_path_makes_the_same_columns(self):
        """`log_node` 길. 두 길이 같은 열 이름을 내야 같은 뷰어로 봄."""
        pytest.importorskip("huphy_ros", reason="ros/huphy_ros 가 경로에 없음")
        from huphy_ros import convert

        class Msg:
            amps, age_ms = 3.25, 18.0

        assert tuple(convert.current_to_columns(Msg())) == power.CURRENT_COLUMNS
