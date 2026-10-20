"""파이프라인 감사 — 실제 `config/robot.yaml` 과 실제 가중치 파일로 돈다.

    PYTHONPATH=src python3 -m pytest tests/test_pipeline_audit.py -q -v

다른 테스트와 다른 점: `leg_config()` 같은 합성 설정이 아니라 **배포에 쓰는
`config/robot.yaml` 과 `config/calibration/*_v1.0.json` 을 그대로 읽어서** 돈다.
합성 설정으로 통과하는 테스트는 "코드가 맞다"만 보장하고 "지금 로봇에 적용된 값이
맞다"는 보장하지 않는다 -- 이 파일은 후자를 본다.

CAN 전송 계층만 `tests/test_leg.py` 와 같은 방식으로 가짜(`FakeBus`)로 바꾼다.
그 위(`config/`, `robots/leg.py`, `control/`, `motors/base.py`)는 전부 실제
코드다. 하드웨어가 없어도 돈다.

구성:

    TestGainPropagation     robot.yaml 의 kp/kd 가 MitCommand 까지 그대로 오는가
    TestGainOverride        huphy-run 의 Gains() 덮어쓰기가 설계대로 동작하는가
    TestSignAndOffset       cal -> raw 변환이 지금 캘리브레이션(sign=1, offset=0)에서
                            값을 바꾸지 않는가
    TestImuWiring           IMU 가 없거나 못 읽었을 때 정책 입력이 조용히 기본값으로
                            빠지는 것을 재현
"""

from __future__ import annotations

import sys
import types
from collections import deque
from pathlib import Path
from typing import Dict

import pytest

from huphy.config import load_robot
from huphy.motors.base import Gains
from huphy.motors.robstride import tables as T
from huphy.motors.robstride.codec import mit
from huphy.robots.leg import ANKLE_MOTORS, SINGLE_JOINTS
from huphy.scripts.bringup import build_leg
from huphy.sensors.base import ImuState
import math

REPO = Path(__file__).resolve().parent.parent
CONFIG = REPO / "config" / "robot.yaml"


# ===========================================================================
# 가짜 CAN — tests/test_leg.py 의 FakeBus 와 같은 방식.
# ===========================================================================
class _FakeMessage:
    def __init__(self, arbitration_id, data, is_extended_id=False):
        self.arbitration_id = arbitration_id
        self.data = data
        self.is_extended_id = is_extended_id


class _FakeBus:
    """`send`/`recv` 만 구현함. 응답은 보낸 위치를 그대로 돌려줌(추종 1:1)."""

    def __init__(self, **kwargs):
        self.rx: "deque" = deque()
        self.sent = []

    def send(self, msg):
        self.sent.append(msg)
        mid = msg.arbitration_id
        d = msg.data
        if d[0] == 0xFF:
            # 패시브/인에이블류 명령 — 위치 0 으로 응답
            pos_raw = 0.0
        else:
            # 실제 좌표를 모르므로(모델별 인코딩이 다름) 그냥 echo 하지 않고
            # 응답 위치는 0 으로 고정. 이 테스트는 "보낸 프레임의 kp/kd/위치"만
            # 확인하므로 응답 내용은 중요하지 않음.
            pos_raw = 0.0
        q = mit.float_to_uint(math.radians(pos_raw), -12.57, 12.57, 16)
        self.rx.append(_FakeMessage(mid, bytes([
            mid, (q >> 8) & 0xFF, q & 0xFF, 0, 0, 0, 0x01, 0x2C,
        ])))

    def recv(self, timeout=None):
        return self.rx.popleft() if self.rx else None

    def shutdown(self):
        pass


@pytest.fixture
def fake_can(monkeypatch):
    mod = types.ModuleType("can")
    mod.Message = _FakeMessage
    mod.interface = types.SimpleNamespace(Bus=_FakeBus)
    monkeypatch.setitem(sys.modules, "can", mod)
    return mod


def _robot():
    return load_robot(CONFIG)


def _sent_frames(leg) -> list:
    """이 다리가 들고 있는 `_FakeBus` 가 받은 프레임들."""
    return leg.bus.bus._bus.sent  # Leg.bus(RobStrideBus).bus(CanBus)._bus(FakeBus)


# ===========================================================================
# 1. robot.yaml 의 게인이 MitCommand 까지 그대로 오는가
# ===========================================================================
class TestGainPropagation:
    """`build_commands()` 가 내는 `MitCommand.kp/kd` 를 robot.yaml 값과 직접 대조함.

    [config/robot.yaml] 에 적힌 값(RS04 그룹 kp=10.0/kd=1.0 -- 0.5 때 시작값.
    실제로는 `robot.yaml` 심볼릭 링크가 가리키는 파일을 그대로 읽으므로, 이
    파일이 바뀌면 이 테스트가 읽는 값도 같이 바뀜. **상수를 테스트에 박지 않고
    설정 파일에서 직접 읽어 대조하는 이유가 이것임** -- 값이 바뀌어도 테스트가
    따라가고, 전달 과정에서 값이 바뀌는 버그만 잡음.
    """

    def test_every_motor_gain_matches_config_exactly(self, fake_can):
        robot = _robot()
        for limb in robot.limbs_of_kind("leg").values():
            leg = build_leg(robot, limb, allow_uncalibrated=True)
            leg.connect()

            targets = {j: 0.0 for j in SINGLE_JOINTS}
            targets.update({m: 0.0 for m in ANKLE_MOTORS})
            commands = leg.build_commands(targets)

            assert commands, f"{limb.name}: 명령이 하나도 안 나옴"
            for motor_name, motor in limb.motors.items():
                mid = motor.id
                assert mid in commands, (
                    f"{limb.name}/{motor_name}(id={mid}): 명령이 안 나옴 -- "
                    f"가드가 거부했을 가능성"
                )
                cmd = commands[mid]
                assert cmd.kp == pytest.approx(motor.gains.kp), (
                    f"{limb.name}/{motor_name}: kp 가 robot.yaml 값과 다름 "
                    f"(설정 {motor.gains.kp}, 명령 {cmd.kp})"
                )
                assert cmd.kd == pytest.approx(motor.gains.kd), (
                    f"{limb.name}/{motor_name}: kd 가 robot.yaml 값과 다름 "
                    f"(설정 {motor.gains.kd}, 명령 {cmd.kd})"
                )

    def test_gain_groups_match_generalized_walk_requirement(self, fake_can):
        """`generalized_walk.py`가 전제하는 관절 묶음별 게인(학습 쪽 README의
        "Nominal gains" 표)이 실제 `robot.yaml`과 같은지 직접 대조함. 둘이
        어긋나면 로봇이 학습 때와 다른 강성으로 움직임.

        값은 2026-10-20 기준(42f3fa9 "1.0 양다리 게인을 실물에서 정한 값으로
        바꿈") -- 바뀌면 이 표도 같이 갱신할 것.
        """
        robot = _robot()
        limb = robot.limbs_of_kind("leg")["right_leg"]
        expected = {
            "hip_pitch": (135.0, 6.0),
            "hip_roll": (135.0, 6.0),
            "hip_yaw": (75.0, 4.0),
            "knee": (130.0, 6.0),
            "ankle_a": (60.0, 2.7),
            "ankle_b": (60.0, 2.7),
        }
        for name, (kp, kd) in expected.items():
            motor = limb.motors[name]
            assert motor.gains.kp == pytest.approx(kp), (
                f"{name}: robot.yaml kp={motor.gains.kp}, 기대값={kp}"
            )
            assert motor.gains.kd == pytest.approx(kd), (
                f"{name}: robot.yaml kd={motor.gains.kd}, 기대값={kd}"
            )


# ===========================================================================
# 2. huphy-run 의 Gains() 덮어쓰기
# ===========================================================================
class TestGainOverride:
    """`build_leg(..., gains=Gains(...))` 가 설정값을 덮어쓰는지.

    지금 `huphy-run`(`generalized_walk`)은 이 옵션을 안 씀 -- `robot.yaml`이
    이 정책이 학습에서 가정한 게인 그대로라 덮어쓸 이유가 없음(예전 `balance`/
    `hopping`은 달랐음, 지금은 지워짐). `build_leg`의 `gains=` 자체는 여전히
    범용 옵션으로 남아 있고, 쓰일 때 **모터 전부**에 걸리는지만 확인함 -- 모터
    하나만 빠지면 그 관절만 의도한 게인이 아닌 값으로 움직임.
    """

    def test_override_replaces_every_motor(self, fake_can):
        robot = _robot()
        limb = robot.limbs_of_kind("leg")["right_leg"]
        override = Gains(kp=20.0, kd=0.502)
        leg = build_leg(robot, limb, gains=override, allow_uncalibrated=True)
        for motor in leg.config.motors.values():
            assert motor.gains.kp == pytest.approx(20.0)
            assert motor.gains.kd == pytest.approx(0.502)

    def test_without_override_config_survives(self, fake_can):
        """`gains=` 를 안 주면(지금 `huphy-run`의 실제 경로) robot.yaml 값이 그대로 남아야 함."""
        robot = _robot()
        limb = robot.limbs_of_kind("leg")["right_leg"]
        leg = build_leg(robot, limb, allow_uncalibrated=True)
        for name, motor in leg.config.motors.items():
            assert motor.gains.kp == pytest.approx(limb.motors[name].gains.kp)
            assert motor.gains.kd == pytest.approx(limb.motors[name].gains.kd)


# ===========================================================================
# 3. cal -> raw 변환 — 지금 캘리브레이션(sign=1, offset=0)에서 값이 안 바뀌는가
# ===========================================================================
class TestSignAndOffset:
    """`config/calibration/*_v1.0.json` 은 전부 `sign=1.0, offset_deg=0.0` 임
    (아직 아무것도 재지 않은 상태 -- 파일 자체의 주석이 그렇게 말함).

    이 상태에서는 `cal_to_raw`/`raw_to_cal` 이 항등 변환이어야 함. 이 테스트는
    "sign 이 1.0 이면 바뀌지 않는다"는 **코드 로직**을 확인하는 것이지, 실측이
    끝났는지를 확인하는 것이 아님 -- 실측 여부는 아래 '발견된 문제'에 따로 적음.
    """

    def test_cal_to_raw_is_identity_when_sign_is_one_offset_zero(self, fake_can):
        robot = _robot()
        limb = robot.limbs_of_kind("leg")["right_leg"]
        leg = build_leg(robot, limb, allow_uncalibrated=True)
        leg.connect()

        for motor_name in list(SINGLE_JOINTS) + list(ANKLE_MOTORS):
            cal = leg.calibration[motor_name]
            assert cal.sign == pytest.approx(1.0), (
                f"{motor_name}: sign 이 1.0 이 아님 -- 더 이상 플레이스홀더가 "
                f"아니라 실측이 들어간 것일 수 있음. 이 테스트 가정이 깨짐"
            )
            assert cal.offset_deg == pytest.approx(0.0), (
                f"{motor_name}: offset_deg 가 0.0 이 아님 -- 위와 같음"
            )
            raw = leg.cal_to_raw(motor_name, 12.34)
            assert raw == pytest.approx(12.34, abs=1e-6)


# ===========================================================================
# 4. IMU 배선 — 못 읽었을 때 정책이 무엇을 보는가
# ===========================================================================
class TestImuWiring:
    """`control/generalized_walk.py` 의 `observation_vector` 는
    `imu_state.is_valid` 나 `age_ms()` 를 **검사하지 않는다** -- `imu_state.gyro_dps`/
    `.gravity` 를 바로 읽음.

    그래서 센서가 끊기면 `ImuState()` 기본값(수평·정지와 같은 값)이 **조용히**
    정책 입력에 들어감 -- 에러도 경고도 없음. 이 테스트는 그 동작을 고정해
    재현해 둔다. 통과한다고 "문제 없음"이 아니라 "이게 지금 동작"이라는 뜻임.
    """

    def test_invalid_imu_state_feeds_level_defaults_silently(self):
        from huphy.control import generalized_walk as gw

        never_connected = ImuState()  # is_valid=False, 나머지 전부 기본값
        assert never_connected.is_valid is False
        assert never_connected.gravity == (0.0, 0.0, -1.0)
        assert never_connected.gyro_dps == (0.0, 0.0, 0.0)

        observation = {f"{j}.pos": gw.DEFAULT_POSE_DEG[j] for j in gw.ORDER}
        previous = [gw.DEFAULT_POSE_DEG[j] for j in gw.ORDER]

        # is_valid=False 인데도 예외 없이 벡터가 나옴 -- 검사가 없다는 뜻.
        vector = gw.observation_vector(observation, never_connected, [0.0] * 12, previous)
        assert vector.shape == (gw.OBS_DIM,)

    def test_loop_link_check_does_not_cover_imu(self):
        """`ControlLoop._check_link` 은 `robot.link_status` 만 봄
        (control/loop.py:318-342) -- 이건 모터 응답 여부고, IMU 유효성과
        무관함. IMU 가 죽어도 이 경로로는 안 걸림.
        """
        from huphy.control.loop import ControlLoop
        import inspect

        src = inspect.getsource(ControlLoop._check_link)
        assert "imu" not in src.lower(), (
            "ControlLoop._check_link 이 imu 를 언급하기 시작했으면 이 테스트와 "
            "위 주석(문서화된 '검사 없음')을 같이 갱신할 것"
        )
