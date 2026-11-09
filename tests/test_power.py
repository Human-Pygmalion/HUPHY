"""전력 열 -- 받은 상전류로 계산해 기록에 붙이는 값.

    PYTHONPATH=src python3 -m pytest tests/test_power.py -q

고정하는 것:

    열 이름과 순서   틀려도 열 수가 같아 코드로는 안 잡힘. 뷰어가 다른 그래프를 그림
    계수 1.5        3상 x 진폭->실효값. 빠지면 값이 3분의 2가 됨
    상저항          사양서는 선저항으로 적음. 2로 안 나누면 손실이 2배가 됨
    NaN 전파        한 모터를 못 받았는데 합이 나오면 일부 합을 전체로 읽게 됨
"""

import math

import pytest

from huphy.motors.robstride import tables
from huphy.telemetry import power
from huphy.telemetry.policy_log import PolicyLog
from huphy.telemetry.track_log import extra_columns


MOTORS = ("right_leg/knee", "right_leg/hip_yaw")
R = {"right_leg/knee": 0.08, "right_leg/hip_yaw": 0.195}
IQ = {"right_leg/knee": 10.0, "right_leg/hip_yaw": 2.0}
VBUS = {"right_leg/knee": 48.0, "right_leg/hip_yaw": 50.0}


class TestConstants:
    def test_phase_resistance_is_half_the_line_resistance(self):
        """사양서값은 선저항(상-상)임. Y 결선이면 코일 둘이 직렬."""
        assert tables.phase_resistance_ohm("RS04") == pytest.approx(0.16 / 2)
        assert tables.phase_resistance_ohm("RS03") == pytest.approx(0.39 / 2)

    def test_unknown_model_is_nan_not_zero(self):
        """0 이면 손실 0 W 로 기록됨 -- 사양을 안 넣은 것과 구별이 안 됨."""
        assert math.isnan(tables.phase_resistance_ohm("RS02"))
        assert math.isnan(tables.phase_resistance_ohm(""))
        assert math.isnan(tables.phase_resistance_ohm("없는모델"))


class TestColumns:
    def test_motor_columns_come_before_the_totals(self):
        assert power.power_columns(MOTORS) == (
            "power/right_leg/knee/copper_w",
            "power/right_leg/hip_yaw/copper_w",
            "power/right_leg/knee/dc_a_lo",
            "power/right_leg/hip_yaw/dc_a_lo",
            "power/copper_w",
            "power/dc_a_lo",
        )

    def test_values_fill_exactly_the_columns(self):
        got = power.power_values(MOTORS, IQ, VBUS, R)
        assert tuple(got) == power.power_columns(MOTORS)


class TestValues:
    def test_copper_loss_has_the_three_phase_factor(self):
        assert power.copper_loss_w(10.0, 0.08) == pytest.approx(1.5 * 100 * 0.08)
        assert power.copper_loss_w(10.0, 0.08) == pytest.approx(12.0)

    def test_sign_of_the_current_does_not_change_the_loss(self):
        """`iq` 는 방향에 따라 음수로 옴. 제곱이라 손실은 같음."""
        assert power.copper_loss_w(-10.0, 0.08) == pytest.approx(12.0)

    def test_total_is_the_sum_of_the_motors(self):
        got = power.power_values(MOTORS, IQ, VBUS, R)
        assert got["power/right_leg/knee/copper_w"] == pytest.approx(12.0)
        assert got["power/right_leg/hip_yaw/copper_w"] == pytest.approx(1.5 * 4 * 0.195)
        assert got["power/copper_w"] == pytest.approx(12.0 + 1.17)

    def test_dc_current_is_the_total_over_the_mean_voltage(self):
        got = power.power_values(MOTORS, IQ, VBUS, R)
        assert got["power/dc_a_lo"] == pytest.approx(13.17 / 49.0)

    def test_each_motor_dc_current_uses_its_own_voltage(self):
        got = power.power_values(MOTORS, IQ, VBUS, R)
        assert got["power/right_leg/knee/dc_a_lo"] == pytest.approx(12.0 / 48.0)
        assert got["power/right_leg/hip_yaw/dc_a_lo"] == pytest.approx(1.17 / 50.0)


class TestMissing:
    def test_a_motor_without_a_current_makes_the_total_nan(self):
        """합에서 그 모터를 빼면 일부만 더한 값이 전체 합으로 보임."""
        got = power.power_values(MOTORS, {"right_leg/knee": 10.0}, VBUS, R)
        assert got["power/right_leg/knee/copper_w"] == pytest.approx(12.0)
        assert math.isnan(got["power/right_leg/hip_yaw/copper_w"])
        assert math.isnan(got["power/copper_w"])
        assert math.isnan(got["power/dc_a_lo"])

    def test_a_motor_without_a_resistance_makes_the_total_nan(self):
        got = power.power_values(MOTORS, IQ, VBUS, {"right_leg/knee": 0.08})
        assert math.isnan(got["power/copper_w"])

    def test_no_voltage_at_all_leaves_the_current_nan(self):
        got = power.power_values(MOTORS, IQ, {}, R)
        assert got["power/copper_w"] == pytest.approx(13.17)
        assert math.isnan(got["power/dc_a_lo"])
        assert math.isnan(got["power/right_leg/knee/dc_a_lo"])

    def test_a_motor_without_a_voltage_is_nan_only_for_itself(self):
        """전압 하나를 못 받아도 그 모터만 NaN. 합은 나머지 전압 평균으로 나옴."""
        got = power.power_values(MOTORS, IQ, {"right_leg/knee": 48.0}, R)
        assert got["power/right_leg/knee/dc_a_lo"] == pytest.approx(12.0 / 48.0)
        assert math.isnan(got["power/right_leg/hip_yaw/dc_a_lo"])
        assert got["power/dc_a_lo"] == pytest.approx(13.17 / 48.0)


class TestRecording:
    """`log_node` 가 열을 미리 박는 길. 전력 열은 전기값 열 뒤임."""

    def test_power_columns_follow_the_extra_columns(self):
        log = PolicyLog(("obs/a",), MOTORS, limits={},
                        extra_motors=MOTORS, power_motors=MOTORS)
        assert log.extra_cols == extra_columns(MOTORS) + power.power_columns(MOTORS)

    def test_rows_keep_their_width_before_the_first_current_arrives(self):
        log = PolicyLog(("obs/a",), MOTORS, limits={},
                        extra_motors=MOTORS, power_motors=MOTORS)
        log.add_row(0.00, [1.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0])
        log.add_row(0.02, [1.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0],
                    power.power_values(MOTORS, IQ, VBUS, R))

        assert len(log.columns) == 1 + 1 + 2 * 3 + 2 * 3 + 4
        assert all(len(r) == len(log.columns) for r in log.rows)
        i = log.columns.index("power/copper_w")
        assert math.isnan(log.rows[0][i])
        assert log.rows[1][i] == pytest.approx(13.17)

    def test_power_motors_alone_adds_no_columns(self):
        """전력은 전기값을 읽을 때만 나옴 -- `extra_motors` 없이는 열을 안 만듦."""
        log = PolicyLog(("obs/a",), MOTORS, limits={}, power_motors=MOTORS)
        assert log.extra_cols is None
