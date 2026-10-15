"""EBIMU 테스트 — 하드웨어 없이 실행됨.

    PYTHONPATH=src python3 -m pytest tests -q

`pyserial` 이 없어도 돌아야 함. `connect()` 안에서 import 하는 것이 그 때문임.

여기서 확인하는 것은 **자르는 자리와 단위**임. 실제 시리얼 타이밍은 실물에서만
확인됨.
"""

import sys
import types

import pytest

from huphy.sensors.base import gravity_from_quat
from huphy.sensors.ebimu import commands as C
from huphy.sensors.ebimu import commissioning as M
from huphy.sensors.ebimu import protocol

FULL = ("quat", "gyro", "accel", "dist", "temp", "time")
LINE = (
    "*-0.0885,0.3304,0.2432,0.9077,"      # quat  z, y, x, w
    "1.20,-0.45,0.33,"                     # gyro  도/초
    "0.643,0.383,0.663,"                   # accel g. 위 자세의 중력과 맞춘 값 (도면 축)
    "0.012,-0.003,0.001,"                  # dist  m
    "31.4,"                                # temp
    "12345"                                # time  ms
)


# ===========================================================================
# 출력 구성
# ===========================================================================
class TestOutput:
    def test_field_count_is_the_sum(self):
        assert C.field_count(FULL) == 15
        assert C.field_count(("quat", "gyro", "accel")) == 10

    def test_unknown_block_is_rejected(self):
        with pytest.raises(ValueError, match="모르는 출력 항목"):
            C.validate(("quat", "gyros"))

    def test_attitude_cannot_be_dropped(self):
        """센서가 끌 수 없는 항목임. 없으면 자르는 자리가 통째로 밀림."""
        with pytest.raises(ValueError, match="자세는 끌 수 없음"):
            C.validate(("gyro", "accel"))

    def test_euler_and_quat_are_exclusive(self):
        """같은 명령(sof)을 나눠 씀."""
        with pytest.raises(ValueError, match="동시에 낼 수 없음"):
            C.validate(("euler", "quat", "gyro"))

    def test_accel_and_vel_are_exclusive(self):
        """같은 명령(soa)을 나눠 씀."""
        with pytest.raises(ValueError, match="동시에 낼 수 없음"):
            C.validate(("quat", "accel", "vel"))

    def test_order_must_match_the_packet(self):
        """개수만 맞고 순서가 틀리면 값이 엉뚱한 자리로 들어감."""
        with pytest.raises(ValueError, match="패킷 순서대로"):
            C.validate(("gyro", "quat"))

    def test_duplicates_are_rejected(self):
        with pytest.raises(ValueError, match="중복"):
            C.validate(("quat", "gyro", "gyro"))


# ===========================================================================
# 명령 만들기
# ===========================================================================
class TestCommands:
    def test_it_turns_off_what_is_not_listed(self):
        """센서에 예전 설정이 남아 있으면 필드가 더 붙어 나와 파싱이 밀림."""
        got = C.output_commands(("quat", "gyro", "accel"), rate_hz=100)
        assert "<som0>" in got and "<sod0>" in got and "<sot0>" in got

    def test_full_output(self):
        assert C.output_commands(FULL, rate_hz=100) == [
            "<sof2>", "<sog1>", "<soa1>", "<som0>", "<sod1>", "<sot1>", "<sots1>", "<sor10>",
        ]

    def test_accel_mode_picks_the_value(self):
        got = C.output_commands(("quat", "accel"), accel_mode="local")
        assert "<soa2>" in got

    def test_rate_must_divide_into_milliseconds(self):
        """센서는 1ms 배수만 됨. 안 떨어지면 실제 주기가 요청과 달라짐."""
        assert C.rate_command(100.0) == "<sor10>"
        assert C.rate_command(50.0) == "<sor20>"
        with pytest.raises(ValueError, match="1ms 단위로 안 떨어짐"):
            C.rate_command(333.0)

    def test_dangerous_commands_are_marked(self):
        assert C.is_dangerous("<lf>")          # 공장초기화
        assert C.is_dangerous("<sb6>")         # 보레이트. 보낸 순간 통신이 끊김
        assert C.is_dangerous("<cg>")          # 캘리브레이션
        assert not C.is_dangerous("<sof2>")


# ===========================================================================
# 패킷 파싱
# ===========================================================================
class TestParse:
    def test_it_cuts_by_the_configured_order(self):
        got = protocol.parse_fields(LINE, FULL)
        assert got["gyro"] == (1.20, -0.45, 0.33)
        assert got["temp"] == (31.4,)

    def test_a_line_without_the_prefix_is_dropped(self):
        assert protocol.parse_fields("0.1,0.2,0.3", FULL) is None

    def test_a_wrong_field_count_drops_the_whole_line(self):
        """앞에서부터 채우면 항목 하나가 빠졌을 때 뒤가 한 칸씩 당겨짐.

        각속도 자리에 가속도가 들어가는데 값이 그럴듯해서 실물에서 안 잡힘.
        """
        assert protocol.parse_fields("*1,2,3", FULL) is None
        assert protocol.decode("*1,2,3", FULL) is None

    def test_a_trailing_checksum_is_ignored(self):
        """끝에 16진수가 붙는 펌웨어가 있음."""
        assert protocol.parse_fields(LINE + ",A3F1", FULL) is not None

    def test_quaternion_order_is_flipped(self):
        """센서는 (z, y, x, w) 로 보냄. 저장소 안에서는 (w, x, y, z) 이고, 센서 축이
        (X, -Y, -Z) 라 y, z 부호를 뒤집어 도면 축으로 옮김."""
        assert protocol.to_quaternion((-0.0885, 0.3304, 0.2432, 0.9077)) == (
            0.9077, 0.2432, -0.3304, 0.0885,
        )

    def test_nose_up_gives_negative_gravity_x(self):
        """기수를 들면(피치 +, 도면 Y 축 기준 오른손) 중력의 x 성분은 음수여야 함.

        센서 축은 Y 가 반대라, 센서 쿼터니언에서는 같은 자세가 y 성분이 음수로 옴.
        부호를 안 뒤집으면 여기서 x 가 양수로 나옴 -- 고치기 전의 버그.
        """
        import math

        half = math.radians(15.0) / 2.0
        # 센서 기준 (w, x, y, z) = (cos, 0, -sin, 0) 을 센서 순서 (z, y, x, w) 로
        packed = (0.0, -math.sin(half), 0.0, math.cos(half))
        g = gravity_from_quat(protocol.to_quaternion(packed))
        assert g[0] == pytest.approx(-math.sin(math.radians(15.0)))
        assert g[2] == pytest.approx(-math.cos(math.radians(15.0)))

    def test_accel_becomes_mps2(self):
        """센서는 g 로 보냄."""
        state = protocol.decode(LINE, FULL)
        assert state.accel_mps2[2] == pytest.approx(0.663 * 9.80665)

    def test_gyro_stays_in_degrees(self):
        """이 센서는 이미 도/초로 줌. 바꿀 것이 없음."""
        assert protocol.decode(LINE, FULL).gyro_dps == (1.20, -0.45, 0.33)

    def test_gravity_comes_from_the_quaternion(self):
        state = protocol.decode(LINE, FULL)
        assert state.gravity == pytest.approx(
            gravity_from_quat((0.9077, 0.2432, -0.3304, 0.0885))
        )

    def test_euler_is_added_for_the_graph(self):
        """쿼터니언을 받아도 사람은 자세를 도로 봄. 제어 경로는 아님."""
        extra = protocol.decode(LINE, FULL).extra
        assert extra["roll"] == pytest.approx(30.0, abs=0.1)
        # 센서 축의 Y 가 도면과 반대라 도면 기준 피치는 부호가 바뀜 (to_quaternion)
        assert extra["pitch"] == pytest.approx(-40.0, abs=0.1)

    def test_blocks_that_are_off_stay_at_zero(self):
        """열이 나타났다 사라지면 CSV 가 밀림. 키는 늘 냄."""
        state = protocol.decode(
            "*-0.0885,0.3304,0.2432,0.9077,1.2,-0.45,0.33,0.643,0.383,0.663",
            ("quat", "gyro", "accel"),
        )
        assert state.extra["dx"] == 0.0
        assert state.extra["temp"] == 0.0
        assert state.extra["sensor_ms"] == -1.0

    def test_extra_fields_are_fixed(self):
        """output 에 무엇을 켜든 열 목록은 같아야 함."""
        assert set(protocol.decode(LINE, FULL).extra) == set(protocol.EXTRA_FIELDS)

    def test_euler_output_is_refused(self):
        """회전 순서 규약을 알아야 해서 쓰지 않음."""
        with pytest.raises(ValueError, match="쿼터니언이 아님"):
            protocol.to_state({"euler": (1.0, 2.0, 3.0)})


# ===========================================================================
# 센서 설정 대조
# ===========================================================================
class TestSettings:
    RESPONSE = "<sof1> <sog1> <soa1> <som0> <sod0> <sot0> <sots0> <sor10> <raa_t10000>"

    def test_it_reads_what_is_on_the_sensor(self):
        got = C.parse_config(self.RESPONSE)
        assert got["sof"] == "1" and got["sor"] == "10"

    def test_it_names_the_blocks_that_are_on(self):
        assert C.output_from_config(C.parse_config(self.RESPONSE)) == [
            "euler", "gyro", "accel",
        ]

    def test_attitude_defaults_to_euler_when_absent(self):
        """끌 수 없는 항목이고 공장 기본값이 오일러임."""
        assert "euler" in C.output_from_config({"sog": "1"})

    def test_it_lists_only_what_differs(self):
        """같은 값을 다시 쓰는 것도 비휘발성 메모리에 쓰는 일임."""
        settings = C.parse_config(self.RESPONSE)
        wanted = C.output_commands(FULL, rate_hz=100)
        got = {m.command for m in M.compare(settings, wanted)}
        assert got == {"<sof2>", "<sod1>", "<sot1>", "<sots1>"}

    def test_nothing_to_do_when_they_match(self):
        settings = C.parse_config(
            "<sof2> <sog1> <soa1> <som0> <sod1> <sot1> <sots1> <sor10>"
        )
        assert M.compare(settings, C.output_commands(FULL, rate_hz=100)) == []

    def test_it_says_what_the_sensor_is_doing_now(self):
        settings = C.parse_config(self.RESPONSE)
        now = {m.command: m.now for m in M.compare(settings, ["<sof2>"])}
        assert now["<sof2>"] == "Euler Angles"

    def test_data_lines_are_stripped_from_a_response(self):
        """설정 명령을 받아도 센서는 출력을 멈추지 않음. <ok> 가 그 사이에 묻힘."""
        assert M.strip_packets("*1,2,3\r\n<ok>\r\n*4,5,6\r\n") == "<ok>"


# ===========================================================================
# 부착 방향
# ===========================================================================
class TestMountCheck:
    def _state(self, **over):
        state = protocol.decode(LINE, FULL)
        for key, value in over.items():
            setattr(state, key, value)
        return state

    def test_a_matching_sensor_passes(self):
        """정지 상태의 가속도계는 중력방향을 직접 잼."""
        result = M.check_mount(self._state())
        assert result.ok and result.error < M.LEVEL_TOLERANCE

    def test_a_flipped_axis_is_caught(self):
        state = self._state()
        x, y, z = state.accel_mps2
        result = M.check_mount(self._state(accel_mps2=(x, -y, z)))
        assert not result.ok

    def test_level_refuses_to_judge(self):
        """수평이면 두 축이 다 0이라 부착이 틀려도 통과함."""
        result = M.check_mount(
            self._state(gravity=(0.0, 0.0, -1.0), accel_mps2=(0.0, 0.0, 9.81))
        )
        assert not result.tilted_enough
        assert not result.ok

    def test_either_accel_sign_convention_works(self):
        """벤더마다 중력과 같은 방향이거나 반대임. 비율로 판정함."""
        state = self._state()
        flipped = self._state(accel_mps2=tuple(-v for v in state.accel_mps2))
        assert M.check_mount(state).ok and M.check_mount(flipped).ok


# ===========================================================================
# EbimuImu
# ===========================================================================
class FakeSerial:
    """`serial.Serial` 자리. 포트를 열지 않음."""

    def __init__(self, lines=(LINE,), **kwargs):
        self.kwargs = kwargs
        self.lines = list(lines)
        self.closed = False
        self.index = 0

    def readline(self):
        if not self.lines:
            return b""
        line = self.lines[self.index % len(self.lines)]
        self.index += 1
        return (line + "\r\n").encode()

    def reset_input_buffer(self):
        pass

    def close(self):
        self.closed = True


@pytest.fixture
def fake_serial(monkeypatch):
    made = []

    def factory(**kwargs):
        port = FakeSerial(**{k: v for k, v in kwargs.items() if k not in ("lines",)})
        port.lines = made_lines[0]
        made.append(port)
        return port

    made_lines = [[LINE]]
    module = types.ModuleType("serial")
    module.Serial = factory
    monkeypatch.setitem(sys.modules, "serial", module)
    return made, made_lines


class TestEbimuImu:
    def _imu(self, output=FULL):
        from huphy.sensors.ebimu import EbimuImu

        return EbimuImu("main", "/dev/ebimu", output=output)

    def test_euler_output_is_refused_at_build_time(self):
        from huphy.sensors.ebimu import EbimuImu

        with pytest.raises(ValueError, match="쿼터니언이어야 함"):
            EbimuImu("main", "/dev/ebimu", output=("euler", "gyro"))

    def test_it_reads_after_connecting(self, fake_serial):
        imu = self._imu()
        imu.connect()
        try:
            for _ in range(200):
                if imu.read().is_valid:
                    break
                __import__("time").sleep(0.005)
            assert imu.read().is_valid
        finally:
            imu.disconnect()

    def test_a_field_count_mismatch_stops_the_connection(self, fake_serial):
        """값이 한 칸씩 밀린 채로 제어를 시작하는 것보다 안 붙는 편이 나음."""
        _, lines = fake_serial
        lines[0] = ["*1,2,3"]
        imu = self._imu()
        with pytest.raises(ConnectionError, match="필드 수가 안 맞음"):
            imu.connect()

    def test_no_packet_at_all_says_check_the_baudrate(self, fake_serial):
        _, lines = fake_serial
        lines[0] = []
        imu = self._imu()
        with pytest.raises(ConnectionError, match="패킷이 없음"):
            imu.connect()

    def test_nothing_received_is_not_valid(self):
        assert self._imu().read().is_valid is False

    def test_disconnect_is_safe_twice(self, fake_serial):
        imu = self._imu()
        imu.connect()
        imu.disconnect()
        imu.disconnect()
        assert not imu.is_connected

    def test_extra_fields_are_declared(self):
        assert self._imu().extra_fields == protocol.EXTRA_FIELDS
