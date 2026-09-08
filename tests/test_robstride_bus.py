"""RobStride 버스 테스트 — 하드웨어 없이 실행됨.

    PYTHONPATH=src python3 -m pytest tests -q

`python-can` 을 가짜 모듈로 갈아끼워 확인함. 다리 구성 그대로 씀 —
RS02 4개(7~10)와 RS00 2개(11, 12)가 한 채널에 물려 있고 토크 범위가 다름.

가짜 모터는 실제 펌웨어가 아님. 확인하는 것은 **프레임 내용·순서·캐시 갱신**임.
"""

import sys
import types
from collections import deque

import pytest

from huphy.motors.base import Gains, Motor
from huphy.motors.canbus import CanBus
from huphy.motors.robstride import tables as T
from huphy.motors.robstride.bus import (
    MitCommand,
    PASSIVE,
    RobStrideBus,
    _command_frame,
)
from huphy.motors.robstride.codec import mit

LEG = {
    7: Motor(id=7, model="RS02", limits_deg=(-117.07, -21.07)),
    8: Motor(id=8, model="RS02", limits_deg=(-5.51, 79.64)),
    9: Motor(id=9, model="RS02", limits_deg=(-41.90, 31.09)),
    10: Motor(id=10, model="RS02", limits_deg=(-20.65, 74.79), gains=Gains(kp=30.0, kd=1.0)),
    11: Motor(id=11, model="RS00", limits_deg=(-79.77, 43.16)),
    12: Motor(id=12, model="RS00", limits_deg=(-12.50, 126.66)),
}


# ===========================================================================
# 가짜 python-can
# ===========================================================================
class FakeMessage:
    def __init__(self, arbitration_id, data, is_extended_id=False):
        self.arbitration_id = arbitration_id
        self.data = data
        self.is_extended_id = is_extended_id


class FakeCanBus:
    """보낸 프레임에 **응답하는** 버스.

    실제 모터는 명령을 받은 뒤에 답함. 미리 큐에 넣어 두면 `flush_rx()` 가 먼저
    지워 버려서 실제 순서를 재현하지 못함.
    """

    instances = []

    def __init__(self, **kwargs):
        self.sent = []
        self.rx = deque()
        self.responses = {}
        """모터 id -> 그 모터에게 프레임을 보냈을 때 돌아올 응답."""
        self.shutdown_called = 0
        FakeCanBus.instances.append(self)

    def send(self, msg):
        self.sent.append(msg)
        reply = self.responses.get(msg.arbitration_id)
        if reply is not None:
            self.rx.append(reply)

    def recv(self, timeout=None):
        return self.rx.popleft() if self.rx else None

    def shutdown(self):
        self.shutdown_called += 1


@pytest.fixture
def fake_can(monkeypatch):
    FakeCanBus.instances = []
    mod = types.ModuleType("can")
    mod.Message = FakeMessage
    mod.interface = types.SimpleNamespace(Bus=FakeCanBus)
    monkeypatch.setitem(sys.modules, "can", mod)
    return mod


@pytest.fixture
def leg(fake_can):
    b = RobStrideBus(CanBus("can1"), LEG)
    b.connect()
    FakeCanBus.instances[-1].sent.clear()
    return b


@pytest.fixture
def raw(leg):
    return FakeCanBus.instances[-1]


def state_frame(motor_id, pos_deg, *, vel_rad_s=0.0, tau_nm=0.0, temp_c=30.0, enc=None,
                mode=T.OperationMode.MOTOR):
    """모터가 돌려주는 상태 프레임을 만듦. 응답은 앞에 모터 id 가 붙음.

    기본이 Motor 모드임 — 실제 로봇은 토크를 켠 채로 돌고, 그때 Byte6[7] 이 서서
    온도 디코딩이 깨졌었음. 기본값이 Reset 이면 그 버그를 다시 놓침.
    """
    enc = enc or T.encoding_for(T.Model.RS02)
    import math

    pos = mit.float_to_uint(math.radians(pos_deg), -enc.pmax_rad, enc.pmax_rad, 16)
    vel = mit.float_to_uint(vel_rad_s, -enc.vmax_rad_s, enc.vmax_rad_s, 12)
    tau = mit.float_to_uint(tau_nm, -enc.tmax_nm, enc.tmax_nm, 12)
    temp = int(temp_c * 10)
    return FakeMessage(
        motor_id,
        bytes([
            motor_id,
            (pos >> 8) & 0xFF, pos & 0xFF,
            (vel >> 4) & 0xFF,
            ((vel & 0x0F) << 4) | ((tau >> 8) & 0x0F),
            tau & 0xFF,
            ((int(mode) & 0x03) << 6) | ((temp >> 8) & 0x0F), temp & 0xFF,
        ]),
    )


# ===========================================================================
# 구성
# ===========================================================================
class TestConstruction:
    def test_keeps_motor_order(self, leg):
        assert leg.motor_ids == (7, 8, 9, 10, 11, 12)

    def test_encoding_differs_by_model(self, leg):
        """다리에 RS02 와 RS00 이 섞여 있음. 같은 바이트가 다른 토크를 뜻함."""
        assert leg.encoding(10).tmax_nm == 17.0      # RS02 무릎
        assert leg.encoding(11).tmax_nm == 14.0      # RS00 발목
        assert leg.encoding(10).pmax_rad == leg.encoding(11).pmax_rad

    def test_unknown_model_fails_at_construction(self, fake_can):
        """모델 문자열을 벤더 enum 으로 옮기는 유일한 지점임.

        여기서 걸러 두면 제어 중에 오타가 드러나는 일이 없음.
        """
        with pytest.raises(ValueError, match="모르는 모델"):
            RobStrideBus(CanBus("can1"), {1: Motor(id=1, model="RS99")})

    def test_error_lists_known_models(self, fake_can):
        with pytest.raises(ValueError, match=r"RS00.*RS02"):
            RobStrideBus(CanBus("can1"), {1: Motor(id=1, model="rs02")})

    def test_states_start_empty(self, leg):
        """한 번도 못 받은 상태와 0도를 구분함."""
        assert leg.state(10).is_valid is False
        assert all(not s.is_valid for s in leg.states().values())


# ===========================================================================
# 제어 명령 프레임
# ===========================================================================
class TestCommandFrames:
    def test_layout(self):
        """data[0:6]=0xFF, data[6]=F_CMD, data[7]=명령 바이트."""
        f = _command_frame(10, T.CMD_ENABLE)
        assert f.data == bytes([0xFF] * 6 + [0xFF, 0xFC])
        assert f.can_id == 10
        assert f.is_extended is False

    def test_same_byte_two_meanings(self):
        """0xFB 가 F_CMD 에 따라 클리어와 조회로 갈림."""
        clear = _command_frame(10, T.CMD_FAULT)
        query = _command_frame(10, T.CMD_FAULT, f_cmd=T.F_CMD_FAULT_QUERY)
        assert clear.data[7] == query.data[7] == 0xFB
        assert clear.data[6] == 0xFF
        assert query.data[6] == 0x00


# ===========================================================================
# 토크
# ===========================================================================
class TestTorque:
    def test_enable_sends_one_frame_per_motor(self, leg, raw):
        leg.enable_torque()
        assert [m.arbitration_id for m in raw.sent] == [7, 8, 9, 10, 11, 12]
        assert all(m.data[7] == T.CMD_ENABLE for m in raw.sent)

    def test_enable_subset(self, leg, raw):
        leg.enable_torque([11, 12])
        assert [m.arbitration_id for m in raw.sent] == [11, 12]

    def test_disable_uses_stop(self, leg, raw):
        leg.disable_torque()
        assert all(m.data[7] == T.CMD_STOP for m in raw.sent)

    def test_tracks_last_command_only(self, leg):
        """모터에 물어본 값이 아니라 마지막으로 보낸 명령 기준임."""
        assert leg.is_torque_on(10) is False
        leg.enable_torque()
        assert leg.is_torque_on(10) is True
        leg.disable_torque([10])
        assert leg.is_torque_on(10) is False
        assert leg.is_torque_on(11) is True

    def test_clear_fault(self, leg, raw):
        leg.clear_fault([10])
        assert raw.sent[0].data[6] == T.F_CMD_DEFAULT
        assert raw.sent[0].data[7] == T.CMD_FAULT

    def test_unknown_motor_rejected(self, leg):
        with pytest.raises(ValueError, match="없는 모터 id"):
            leg.enable_torque([99])


# ===========================================================================
# 수명
# ===========================================================================
class TestLifecycle:
    def test_disconnect_stops_torque_before_closing(self, leg, raw):
        """순서가 반대면 채널이 닫힌 뒤라 정지 명령을 보낼 방법이 없어짐.

        모터는 마지막 명령을 유지하므로 사람이 전원을 뽑을 때까지 힘을 씀.
        """
        leg.enable_torque()
        raw.sent.clear()
        leg.disconnect()
        assert [m.data[7] for m in raw.sent] == [T.CMD_STOP] * 6
        assert raw.shutdown_called == 1

    def test_disconnect_closes_even_if_stop_fails(self, leg, raw):
        """토크 차단이 실패해도 채널은 반드시 닫음.

        여기서 예외를 올리면 정리가 중간에 멈춰 소켓이 열린 채로 남고, 다음 실행
        때 채널을 못 엶.
        """
        raw.send = lambda msg: (_ for _ in ()).throw(OSError("버스 오프"))
        leg.disconnect()
        assert leg.is_connected is False
        assert raw.shutdown_called == 1

    def test_disconnect_twice_is_quiet(self, leg, raw):
        leg.disconnect()
        raw.sent.clear()
        leg.disconnect()
        assert raw.sent == []

    def test_context_manager_stops_torque_on_exception(self, fake_can):
        b = RobStrideBus(CanBus("can1"), LEG)
        with pytest.raises(RuntimeError):
            with b:
                b.enable_torque()
                raise RuntimeError("제어 루프에서 터진 예외")
        raw = FakeCanBus.instances[-1]
        assert raw.sent[-1].data[7] == T.CMD_STOP
        assert b.is_connected is False


# ===========================================================================
# MIT 명령
# ===========================================================================
class TestSendMit:
    def test_one_frame_per_motor_in_order(self, leg, raw):
        leg.send_mit({mid: MitCommand(position_deg=0.0) for mid in (7, 8, 9)})
        assert [m.arbitration_id for m in raw.sent] == [7, 8, 9]
        assert all(len(m.data) == 8 for m in raw.sent)

    def test_position_survives_the_round_trip(self, leg, raw):
        import math

        leg.send_mit({10: MitCommand(position_deg=45.0)})
        q = (raw.sent[0].data[0] << 8) | raw.sent[0].data[1]
        enc = leg.encoding(10)
        back = math.degrees(mit.uint_to_float(q, -enc.pmax_rad, enc.pmax_rad, 16))
        assert back == pytest.approx(45.0, abs=0.03)

    def test_model_decides_torque_scaling(self, leg, raw):
        """같은 토크 값이 모델에 따라 다른 바이트가 됨. RS00 은 범위가 좁아 더 큼."""
        leg.send_mit({10: MitCommand(0.0, torque_nm=7.0), 11: MitCommand(0.0, torque_nm=7.0)})
        rs02 = ((raw.sent[0].data[6] & 0x0F) << 8) | raw.sent[0].data[7]
        rs00 = ((raw.sent[1].data[6] & 0x0F) << 8) | raw.sent[1].data[7]
        assert rs00 > rs02

    def test_passive_is_all_zero_effort(self):
        """게인과 토크가 0이면 토크 식이 통째로 0이 됨. 응답만 받아 옴."""
        assert PASSIVE.kp == 0.0
        assert PASSIVE.kd == 0.0
        assert PASSIVE.torque_nm == 0.0

    def test_unknown_motor_rejected(self, leg):
        """조용히 무시하면 그 관절만 직전 명령을 유지해 자세가 어긋남."""
        with pytest.raises(ValueError, match=r"없는 모터 id: \[99\]"):
            leg.send_mit({99: MitCommand(0.0)})

    def test_returns_sent_count(self, leg):
        assert leg.send_mit({10: MitCommand(0.0), 11: MitCommand(0.0)}) == 2

    def test_command_is_frozen(self):
        """보낸 뒤에 값이 바뀌면 로그와 실제가 어긋남."""
        with pytest.raises(Exception):
            MitCommand(0.0).kp = 30.0


# ===========================================================================
# 상태 수거
# ===========================================================================
class TestCollect:
    def test_updates_cache(self, leg, raw):
        raw.rx.append(state_frame(10, 45.0, temp_c=34.5))
        missing = leg.collect(expect=1)
        st = leg.state(10)
        assert st.position_deg == pytest.approx(45.0, abs=0.05)
        assert st.temp_c == pytest.approx(34.5)
        assert st.is_valid is True
        assert 10 not in missing

    def test_reports_who_did_not_answer(self, leg, raw):
        """한 모터가 한 주기 빠지는 것은 흔한 일임. 예외 대신 목록으로 알림."""
        raw.rx.extend(state_frame(mid, 0.0) for mid in (7, 8, 9, 10))
        missing = leg.collect(timeout_s=0.001)
        assert missing == [11, 12]

    def test_model_decides_decoding(self, leg, raw):
        """모터 id 가 프레임 안(data[0])에 있으므로 그걸로 표를 고름.

        CAN 중재 id 는 모델을 알려주지 않는데, 다리에는 RS02 와 RS00 이 섞여 있음.
        """
        rs00 = T.encoding_for(T.Model.RS00)
        raw.rx.append(state_frame(11, 20.0, tau_nm=7.0, enc=rs00))
        leg.collect(expect=1)
        assert leg.state(11).torque_nm == pytest.approx(7.0, abs=0.02)

    def test_ignores_unknown_motor(self, leg, raw):
        """다른 다리의 프레임이 흘러들어도 캐시를 오염시키지 않음."""
        raw.rx.append(state_frame(4, 90.0))
        leg.collect(timeout_s=0.001)
        assert all(not s.is_valid for s in leg.states().values())

    def test_survives_malformed_frame(self, leg, raw):
        """깨진 프레임 하나가 제어 루프를 죽이지 않음."""
        raw.rx.append(FakeMessage(10, bytes([10, 1, 2])))
        raw.rx.append(state_frame(10, 30.0))
        leg.collect(timeout_s=0.002)
        assert leg.state(10).position_deg == pytest.approx(30.0, abs=0.05)

    def test_states_returns_a_copy(self, leg, raw):
        """밖에서 캐시를 바꿔 쓸 수 없게 함."""
        raw.rx.append(state_frame(10, 45.0))
        leg.collect(expect=1)
        snapshot = leg.states()
        snapshot[10] = None
        assert leg.state(10) is not None

    def test_stamp_advances(self, leg, raw):
        """신선도 판정의 근거임. 오래된 상태로 점프 가드를 돌리면 안 됨."""
        raw.rx.append(state_frame(10, 0.0))
        leg.collect(expect=1)
        first = leg.state(10).stamp
        raw.rx.append(state_frame(10, 1.0))
        leg.collect(expect=1)
        assert leg.state(10).stamp >= first


# ===========================================================================
# 요청 + 수거
# ===========================================================================
class TestRefreshStates:
    def test_sends_passive_then_collects(self, leg, raw):
        """MIT 모드에는 읽기 전용 명령이 없음. 토크 0인 명령의 응답으로 상태를 받음."""
        raw.responses = {mid: state_frame(mid, 10.0) for mid in leg.motor_ids}
        missing = leg.refresh_states()
        assert missing == []
        assert len(raw.sent) == 6
        assert all(len(m.data) == 8 for m in raw.sent)
        assert leg.state(7).position_deg == pytest.approx(10.0, abs=0.05)

    def test_passive_command_carries_no_effort(self, leg, raw):
        """게인 바이트가 0이어야 함. 아니면 상태를 읽는 것만으로 다리가 움직임."""
        leg.refresh_states([10])
        data = raw.sent[0].data
        assert ((data[3] & 0x0F) << 8) | data[4] == 0        # Kp
        assert (data[5] << 4) | (data[6] >> 4) == 0          # Kd

    def test_subset_only_reports_that_subset(self, leg, raw):
        missing = leg.refresh_states([11, 12])
        assert missing == [11, 12]
        assert len(raw.sent) == 2

    def test_clears_queue_first(self, leg, raw):
        """직전 주기의 응답이 남아 있으면 이번 주기의 것으로 오해함."""
        raw.rx.append(state_frame(10, 99.0))     # 묵은 프레임
        leg.refresh_states([10])
        assert leg.state(10).is_valid is False


# ===========================================================================
# 고장
# ===========================================================================
class TestFault:
    def test_decodes_bits(self, leg, raw):
        """작은 자리 먼저 — 과열(bit0)은 Byte1 에 옴 (매뉴얼 6.7절)."""
        raw.responses[10] = FakeMessage(10, bytes([10, 0x01, 0, 0, 0, 0, 0, 0]))
        fault = leg.read_fault(10)
        assert fault.ok is False
        assert fault.active() == ["overtemperature"]

    def test_names_every_bit_it_reports(self, leg, raw):
        """`raw` 는 0 이 아닌데 `active()` 가 비면 디코딩이 틀린 것임."""
        raw.responses[10] = FakeMessage(10, bytes([10, 0x0F, 0x40, 0x01, 0, 0, 0, 0]))
        fault = leg.read_fault(10)
        assert fault.raw != 0
        assert len(fault.active()) == 6

    def test_zero_is_normal(self, leg, raw):
        raw.responses[10] = FakeMessage(10, bytes([10, 0, 0, 0, 0, 0, 0, 0]))
        fault = leg.read_fault(10)
        assert fault.ok is True
        assert fault.active() == []

    def test_no_answer_is_none(self, leg):
        """응답이 없는 것과 고장이 없는 것은 다름."""
        assert leg.read_fault(10, timeout_s=0.001) is None

    def test_uses_query_variant(self, leg, raw):
        """F_CMD 가 0xFF 면 클리어임. 조회하려다 지우면 원인을 잃음."""
        leg.read_fault(10, timeout_s=0.001)
        assert raw.sent[0].data[6] == T.F_CMD_FAULT_QUERY
        assert raw.sent[0].data[7] == T.CMD_FAULT

    def test_ignores_other_motors_answer(self, leg, raw):
        """조회한 모터가 아닌 응답은 버림. 남의 고장값을 자기 것으로 읽으면 안 됨."""
        raw.responses[10] = FakeMessage(11, bytes([11, 0, 0, 0, 1, 0, 0, 0]))
        assert leg.read_fault(10, timeout_s=0.001) is None
