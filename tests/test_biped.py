"""합성 로봇 테스트 — 하드웨어 없이 실행됨.

여기서 고정하는 것은 **묶는 계층의 책임**임.

    이름       관절 이름 앞에 팔다리가 붙는지
    순서       전송이 전부 끝난 뒤에 수거가 시작되는지 (이슈 #10)
    전부 아니면 아무것도    반쪽 연결·반쪽 토크가 남지 않는지

변환 자체는 `Leg` 의 몫이라 여기서 다시 보지 않음 (`test_leg.py`).
"""

import pytest

from huphy.config.schema import SafetyConfig
from huphy.robots.base import Robot
from huphy.robots.biped import Biped, join_name, split_name


# ===========================================================================
# 가짜 팔다리
# ===========================================================================
class FakePart(Robot):
    """호출 순서를 기록하는 팔다리. 버스도 모터도 없음."""

    name = "fake"

    def __init__(self, part_id, log, *, joints=("knee",), motors=None, fail=(),
                 calibrated=True):
        self.id = part_id
        self.log = log
        self.calibrated = calibrated
        self._joints = tuple(joints)
        self.motor_names = tuple(motors if motors is not None else joints)
        self.torque_motors = ()
        self.fail = set(fail)
        self.connected = False
        self.torque = False
        self._sent = {}
        self.missing = ()
        self.config = None

    def _step(self, what):
        self.log.append((self.id, what))
        if what in self.fail:
            raise RuntimeError(f"{self.id} {what} 실패")

    # 구조
    @property
    def joint_names(self):
        return self._joints

    @property
    def observation_features(self):
        return {f"{j}.pos": float for j in self._joints}

    @property
    def action_features(self):
        return {j: float for j in self._joints}

    # 수명
    @property
    def is_connected(self):
        return self.connected

    def connect(self):
        self._step("connect")
        self.connected = True

    def disconnect(self):
        self._step("disconnect")
        self.connected = False

    def enable(self):
        self._step("enable")
        self.torque = True

    def disable(self):
        self._step("disable")
        self.torque = False

    # 캘리브레이션
    @property
    def is_calibrated(self):
        return self.calibrated

    def calibrate(self):
        self._step("calibrate")

    # 관찰
    def get_observation(self):
        return {f"{j}.pos": 1.0 for j in self._joints}

    # 명령
    def build_commands(self, action):
        self._step("build")
        self._sent = dict(action)
        return {i: v for i, v in enumerate(action.values())}

    def send(self, commands):
        self._step("send")
        return len(commands)

    def collect(self):
        self._step("collect")
        return self.missing

    def refresh(self):
        self._step("refresh")
        return self.missing

    @property
    def last_sent(self):
        return dict(self._sent)

    def hold(self):
        self._step("hold")
        return {0: "hold"}


@pytest.fixture
def log():
    return []


@pytest.fixture
def biped(log):
    return Biped(
        [FakePart("right_leg", log), FakePart("left_leg", log)], id="huphy"
    )


# ===========================================================================
# 이름
# ===========================================================================
class TestNames:
    def test_split_and_join(self):
        assert split_name("right_leg/knee") == ("right_leg", "knee")
        assert join_name("right_leg", "knee") == "right_leg/knee"

    def test_bare_name_has_no_part(self):
        """구분자가 없으면 팔다리를 모름. 조용히 어느 한쪽으로 보내지 않음."""
        assert split_name("knee") == (None, "knee")

    def test_joint_names_carry_part(self, biped):
        assert biped.joint_names == ("right_leg/knee", "left_leg/knee")

    def test_observation_features_carry_part(self, biped):
        assert set(biped.observation_features) == {
            "right_leg/knee.pos", "left_leg/knee.pos"
        }

    def test_observation_carries_part(self, biped):
        assert biped.get_observation() == {
            "right_leg/knee.pos": 1.0, "left_leg/knee.pos": 1.0
        }

    def test_duplicate_part_names_rejected(self, log):
        """이름이 같으면 관절 이름이 같아져 한쪽 명령이 사라짐."""
        with pytest.raises(ValueError, match="겹침"):
            Biped([FakePart("leg", log), FakePart("leg", log)])

    def test_empty_part_name_rejected(self, log):
        with pytest.raises(ValueError, match="비어"):
            Biped([FakePart("", log)])

    def test_no_parts_rejected(self):
        with pytest.raises(ValueError, match="하나도 없음"):
            Biped([])


# ===========================================================================
# 명령을 나눔
# ===========================================================================
class TestSplitAction:
    def test_splits_by_part(self, biped):
        assert biped.split_action(
            {"right_leg/knee": 30.0, "left_leg/knee": -10.0}
        ) == {"right_leg": {"knee": 30.0}, "left_leg": {"knee": -10.0}}

    def test_missing_part_gets_empty(self, biped):
        """명령이 없는 팔다리도 자리를 남김. 빼면 그 다리만 조용히 건너뛰어짐."""
        assert biped.split_action({"right_leg/knee": 30.0})["left_leg"] == {}

    def test_unknown_part_is_error(self, biped):
        with pytest.raises(ValueError, match="모르는 관절"):
            biped.split_action({"middle_leg/knee": 0.0})

    def test_bare_joint_is_error(self, biped):
        """접두어 없는 이름은 어느 다리인지 모름. 추측하지 않음."""
        with pytest.raises(ValueError, match="모르는 관절"):
            biped.split_action({"knee": 0.0})

    def test_build_commands_keyed_by_part(self, biped):
        """모터 id 는 버스 안에서만 유일함. 한 사전에 몰면 겹쳐서 덮임."""
        commands = biped.build_commands(
            {"right_leg/knee": 30.0, "left_leg/knee": -10.0}
        )
        assert set(commands) == {"right_leg", "left_leg"}


# ===========================================================================
# 순서
# ===========================================================================
class TestOrdering:
    def test_send_all_precedes_collect(self, biped, log):
        """전송이 전부 끝난 뒤에 수거가 시작됨 (이슈 #10).

        다리마다 전송·수거를 붙여 하면 앞 다리가 응답을 기다리는 동안 뒤 다리는
        보내지도 못해, 두 다리의 명령 시각이 벌어짐.
        """
        action = {"right_leg/knee": 1.0, "left_leg/knee": 2.0}
        biped.send(biped.build_commands(action))
        biped.collect()

        kinds = [what for _, what in log]
        assert kinds == ["build", "build", "send", "send", "collect", "collect"]

    def test_build_touches_no_transport(self, biped, log):
        """계산은 전송·수거를 부르지 않음."""
        biped.build_commands({"right_leg/knee": 1.0, "left_leg/knee": 2.0})
        assert {what for _, what in log} == {"build"}

    def test_send_counts_all_parts(self, biped):
        commands = biped.build_commands(
            {"right_leg/knee": 1.0, "left_leg/knee": 2.0}
        )
        assert biped.send(commands) == 2


# ===========================================================================
# 수거
# ===========================================================================
class TestCollect:
    def test_missing_carries_part(self, biped):
        """모터 id 만으로는 어느 다리인지 갈리지 않음."""
        biped.parts[0].missing = (10,)
        biped.parts[1].missing = (10,)
        assert set(biped.collect()) == {"right_leg/10", "left_leg/10"}

    def test_failed_part_does_not_stop_the_rest(self, log):
        """한쪽 수거가 터져도 나머지를 수거함.

        여기서 멈추면 멀쩡한 다리의 상태가 직전 주기 값에 머물러, 가드가 옛 위치
        기준으로 점프를 판정함.
        """
        bad = FakePart("right_leg", log, fail={"collect"})
        good = FakePart("left_leg", log)
        biped = Biped([bad, good])

        biped.collect()
        assert ("left_leg", "collect") in log

    def test_failed_part_counts_as_missing(self, log):
        """수거 실패는 무응답과 같은 신호로 올라감 — 이어지면 정지 판정에 걸림."""
        bad = FakePart("right_leg", log, fail={"collect"})
        bad.config = type("C", (), {"motor_ids": (7, 8)})()
        biped = Biped([bad, FakePart("left_leg", log)])

        assert set(biped.collect()) == {"right_leg/7", "right_leg/8"}


# ===========================================================================
# 전부 아니면 아무것도
# ===========================================================================
class TestLifecycle:
    def test_connect_rolls_back(self, log):
        """하나라도 못 열면 이미 연 것을 닫음. 반쪽 연결로 진행하지 않음."""
        biped = Biped([FakePart("a", log), FakePart("b", log, fail={"connect"})])

        with pytest.raises(RuntimeError):
            biped.connect()
        assert not biped.parts[0].connected

    def test_enable_rolls_back(self, log):
        """한 다리만 힘이 들어간 로봇은 반드시 넘어짐."""
        biped = Biped([FakePart("a", log), FakePart("b", log, fail={"enable"})])

        with pytest.raises(RuntimeError):
            biped.enable()
        assert not biped.parts[0].torque

    def test_disconnect_continues_after_failure(self, log):
        """여기서 멈추면 다른 다리에 토크가 남음."""
        biped = Biped([FakePart("a", log, fail={"disconnect"}), FakePart("b", log)])

        biped.disconnect()
        assert ("b", "disconnect") in log

    def test_disable_continues_after_failure(self, log):
        biped = Biped([FakePart("a", log, fail={"disable"}), FakePart("b", log)])

        biped.disable()
        assert ("b", "disable") in log

    def test_partial_connection_is_not_connected(self, biped):
        biped.parts[0].connected = True
        assert not biped.is_connected

    def test_hold_covers_every_part(self, biped):
        """정지 절차가 이것을 씀. 양다리가 같이 붙잡아야 함."""
        assert set(biped.hold()) == {"right_leg", "left_leg"}


# ===========================================================================
# 진단
# ===========================================================================
class TestDiagnostics:
    def test_since_clip_takes_the_most_recent(self, log):
        parts = [FakePart("a", log), FakePart("b", log)]
        parts[0].since_clip = lambda now=None: 5.0
        parts[1].since_clip = lambda now=None: 0.5
        assert Biped(parts).since_clip() == 0.5

    def test_since_clip_ignores_never_happened(self, log):
        """-1 은 "없었음" 이라 최근 사건 후보가 아님."""
        parts = [FakePart("a", log), FakePart("b", log)]
        parts[0].since_clip = lambda now=None: -1.0
        parts[1].since_clip = lambda now=None: 3.0
        assert Biped(parts).since_clip() == 3.0

    def test_since_clip_none_is_minus_one(self, biped):
        assert biped.since_clip() == -1.0

    def test_uncalibrated_lists_parts(self, log):
        biped = Biped([FakePart("a", log), FakePart("b", log, calibrated=False)])
        assert biped.uncalibrated() == ("b",)
        assert not biped.is_calibrated

    def test_part_lookup_by_name(self, biped):
        assert biped.part("left_leg") is biped.parts[1]

    def test_unknown_part_lookup_is_error(self, biped):
        with pytest.raises(KeyError, match="가용"):
            biped.part("arm")

    def test_safety_defaults(self, log):
        assert Biped([FakePart("a", log)]).safety == SafetyConfig()


# ===========================================================================
# IMU 는 팔다리와 나란히 있음
# ===========================================================================
class FakeImu:
    """열고 닫는 것만 기록하는 가짜 센서."""

    extra_fields = ()

    def __init__(self, name, *, fail=()):
        self.name = name
        self.fail = set(fail)
        self.opened = False

    def connect(self):
        if "connect" in self.fail:
            raise RuntimeError(f"{self.name} 포트가 없음")
        self.opened = True

    def disconnect(self):
        if "disconnect" in self.fail:
            raise RuntimeError(f"{self.name} 종료 실패")
        self.opened = False

    def read(self):
        return f"{self.name} state"


class TestImus:
    def test_robot_owns_them(self, log):
        """몸통 센서는 어느 다리의 것도 아님."""
        biped = Biped([FakePart("right_leg", log)], imus=[FakeImu("torso")])
        assert biped.imus.names == ("torso",)

    def test_none_by_default(self, biped):
        assert len(biped.imus) == 0

    def test_opened_with_the_robot(self, log):
        imu = FakeImu("torso")
        Biped([FakePart("a", log)], imus=[imu]).connect()
        assert imu.opened

    def test_a_broken_sensor_does_not_stop_the_robot(self, log):
        """IMU 는 관측이지 제어가 아님. 못 열어도 로봇은 써야 함."""
        biped = Biped([FakePart("a", log)], imus=[FakeImu("torso", fail={"connect"})])

        biped.connect()
        assert biped.is_connected

    def test_a_missing_sensor_is_still_connected(self, log):
        """연결 판정은 팔다리만 봄."""
        biped = Biped([FakePart("a", log)], imus=[FakeImu("torso", fail={"connect"})])
        biped.connect()
        assert biped.is_connected

    def test_closed_with_the_robot(self, log):
        imu = FakeImu("torso")
        biped = Biped([FakePart("a", log)], imus=[imu])
        biped.connect()
        biped.disconnect()
        assert not imu.opened

    def test_a_broken_close_does_not_leave_torque_on(self, log):
        """센서 종료가 실패해도 팔다리는 닫아야 함 -- 안 닫으면 토크가 남음."""
        biped = Biped(
            [FakePart("a", log)], imus=[FakeImu("torso", fail={"disconnect"})]
        )
        biped.disconnect()
        assert ("a", "disconnect") in log

    def test_states_merge_leg_sensors(self, log):
        """다리에 달린 센서도 같이 냄. 찾는 쪽은 개체 이름만 앎."""
        part = FakePart("right_leg", log)
        part.imu_states = lambda: {"shin": "shin state"}
        biped = Biped([part], imus=[FakeImu("torso")])

        assert biped.imu_states() == {"torso": "torso state", "shin": "shin state"}

    def test_all_imus_covers_both_places(self, log):
        part = FakePart("right_leg", log)
        part.imus = (FakeImu("shin"),)
        biped = Biped([part], imus=[FakeImu("torso")])

        assert {i.name for i in biped.all_imus} == {"torso", "shin"}


class TestLinkCounts:
    def test_carries_the_part(self, log):
        part = FakePart("right_leg", log)
        part.link_counts = lambda: {"knee": {"asked": 5, "missed": 1}}
        assert Biped([part]).link_counts() == {
            "right_leg/knee": {"asked": 5, "missed": 1}
        }

    def test_skips_parts_without_it(self, biped):
        assert biped.link_counts() == {}
