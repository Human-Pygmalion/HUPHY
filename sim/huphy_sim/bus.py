"""CAN 버스 자리에 들어가는 시뮬 버스. **`Leg` 는 이것이 시뮬인지 모름.**

`robots/leg.py` 가 버스에 쓰는 면은 14개뿐이고 버스는 생성자 인자임. 그래서 이
클래스 하나로 **실물 `Leg`·`Biped` 가 그대로 돎** -- cal<->raw 변환, 발목 IK/FK,
한계·점프 가드, 캘리브레이션 파일, 응답 누락 판정이 전부 실물 코드임.

## 받은 명령으로 토크를 만듦

실물 모터 펌웨어가 MIT 프레임 다섯 칸으로 토크를 만드는 것과 같게 함
(`huphy_sim/actuator.py`). **MJCF 액추에이터의 게인을 쓰지 않음** -- 그 값은 학습
설정의 것이고, 실물은 `robot.yaml` 의 게인으로 돎.

## 공간 변환

    Leg 가 보내는 것    raw 공간 도    (cal_to_raw 를 이미 지난 값)
    MuJoCo 가 아는 것   관절 rad      (= cal 공간)
    Leg 에게 주는 것    raw 공간 도    (raw_to_cal 로 되돌릴 값)

그래서 이 클래스가 `MotorCalibration` 을 한 번 더 들고 역변환함.
**sign/offset 이 틀려도 왕복에서 상쇄되어 안 잡힘** -- 그건 조립을 재야 아는
값이고 시뮬에는 조립이 없음 (`sim/README.md`).

## 언제 물리가 진행되나

`collect()` 가 부를 때 한 번. `Biped` 는 두 다리에 다 보낸 뒤 수거하므로
(`robots/biped.py:376-407`), 양다리 토크가 다 정해진 다음에 한 주기가 굴러감.
두 번째 다리의 `collect()` 는 이미 굴러간 것을 보고 그냥 돌아옴.
"""

from __future__ import annotations

import math
import time
from typing import Dict, Iterable, List, Mapping, Optional, Sequence, Set, Tuple

from huphy.motors.base import Motor, MotorCalibration, MotorState
from huphy.motors.robstride import tables
from huphy.motors.robstride.bus import MitCommand

from . import actuator
from .world import MOTOR_SPEC, SimWorld

TEMP_C = float("nan")
"""모터 온도. **모르는 값임** -- 발열 모델이 없음. 가짜 숫자를 넣으면 로그를 믿게 됨."""


class SimCan:
    """`RobStrideBus.bus` 자리. CAN 채널이 하던 일이 없음."""

    def __init__(self, channel: str = "sim") -> None:
        self.channel = channel

    def flush_rx(self) -> None:
        """받은 것을 버림. 시뮬은 쌓이는 것이 없음."""

    def __str__(self) -> str:
        return f"{self.channel} (SimCan)"


class SimBus:
    """다리 하나의 모터들. 세계는 두 다리가 같이 씀."""

    def __init__(
        self,
        world: SimWorld,
        motors: Mapping[int, Motor],
        joints: Mapping[int, str],
        calibration: Mapping[int, MotorCalibration],
        *,
        channel: str = "sim",
        curves: Optional[Mapping[str, actuator.Curve]] = None,
    ) -> None:
        self.world = world
        self.motors = dict(motors)
        self.joints = dict(joints)
        self.calibration = dict(calibration)
        self.bus = SimCan(channel)
        self.drain_s = 0.0
        self.latency = None
        self.params: Dict[int, Dict[int, Tuple[float, ...]]] = {}
        """`Leg` 의 `--extra` 가 읽는 자리. 시뮬은 상전류·버스전압을 모름."""

        self.curves = dict(curves) if curves else actuator.load_curves(MOTOR_SPEC)
        self._commands: Dict[int, MitCommand] = {}
        self._torque_on: Dict[int, bool] = {mid: False for mid in self.motors}
        self._states: Dict[int, MotorState] = {mid: MotorState() for mid in self.motors}
        self._torque_log: Dict[int, List[actuator.Torque]] = {}
        """이번 주기에 스텝마다 넣은 토크. 주기 시작에 비우고 `torque_report` 가 요약함."""
        self._connected = False
        world.register(self)

    # ---- 수명 -------------------------------------------------------------
    @property
    def is_connected(self) -> bool:
        return self._connected

    def connect(self) -> None:
        self._connected = True
        self._read_states()

    def disconnect(self) -> None:
        """토크를 끊고 끝냄. 여러 번 불려도 안전함."""
        self.disable_torque()
        self._connected = False

    @property
    def motor_ids(self) -> Tuple[int, ...]:
        return tuple(self.motors)

    @property
    def reply_ids(self) -> Dict[int, int]:
        """호스트 id 규약대로 답한 것으로 둠 (`0xF0 + id`)."""
        return {mid: tables.HOST_ID_BASE + mid for mid in self.motors}

    def enable_torque(self, motors: Optional[Iterable[int]] = None) -> None:
        for mid in self._resolve(motors):
            self._torque_on[mid] = True

    def disable_torque(self, motors: Optional[Iterable[int]] = None) -> None:
        """힘을 뺌. 명령도 버림 -- 안 버리면 다시 켤 때 옛 목표로 튐."""
        for mid in self._resolve(motors):
            self._torque_on[mid] = False
            self._commands.pop(mid, None)

    def _resolve(self, motors: Optional[Iterable[int]]) -> Tuple[int, ...]:
        return tuple(self.motors) if motors is None else tuple(motors)

    # ---- 보내기 -----------------------------------------------------------
    def send_mit(
        self,
        commands: Mapping[int, MitCommand],
        reads: Sequence[Tuple[int, int]] = (),
    ) -> int:
        """명령을 받아 둠. **물리는 아직 안 굴림** -- `collect()` 가 굴림."""
        unknown = sorted(set(commands) - set(self.motors))
        if unknown:
            raise KeyError(f"{self.bus.channel}: 이 버스에 없는 모터 {unknown}")
        self._commands.update(commands)
        self.world.pending = True
        return len(commands)

    def refresh_states(
        self,
        motors: Optional[Iterable[int]] = None,
        reads: Sequence[Tuple[int, int]] = (),
    ) -> List[int]:
        """명령 없이 상태만. 실물은 토크 0 명령을 보내므로 여기도 토크를 0 으로 둠."""
        for mid in self._resolve(motors):
            self._commands[mid] = MitCommand(position_deg=0.0)
        self.world.pending = True
        self._advance()
        return []

    # ---- 수거 -------------------------------------------------------------
    def collect(
        self, *, expect: Optional[int] = None, timeout_s: Optional[float] = None
    ) -> List[int]:
        """물리를 한 주기 굴리고 상태를 갱신함. **응답이 빠지는 일이 없음.**

        빈 목록을 돌려주는 것은 "전부 답했다" 는 뜻임. 실물의 응답 누락·지연은
        이 경로로 검증되지 않음.
        """
        self._advance()
        return []

    def collect_until(self, motor_id: int, *, timeout_s: Optional[float] = None) -> Set[int]:
        self._advance()
        return set(self.motors)

    def state(self, motor_id: int) -> MotorState:
        return self._states[motor_id]

    def states(self) -> Dict[int, MotorState]:
        return dict(self._states)

    # ---- 물리 -------------------------------------------------------------
    def _advance(self) -> None:
        """세계가 아직 이 주기를 안 굴렀으면 굴림."""
        if self.world.pending:
            self.world.advance_cycle()
        self._read_states()

    def apply_torque(self) -> None:
        """이번 물리 스텝에 쓸 토크를 넣음. **세계가 스텝마다 부름.**

        한 번 받은 명령으로 20 ms 내내 PD 를 다시 푸는 것임 -- 실물 모터 펌웨어도
        그렇게 함 (명령은 20 ms 마다 오고 펌웨어는 자기 주기로 돎).
        """
        for mid, motor in self.motors.items():
            joint = self.joints[mid]
            command = self._commands.get(mid)
            if command is None or not self._torque_on.get(mid, False):
                self.world.set_torque(joint, 0.0)
                # 힘이 빠진 모터도 줄을 남김 -- 안 남기면 지난 주기 요약이 그대로
                # 남아 꺼진 뒤에도 토크가 나오는 것처럼 보임. 상한은 뜻이 없어 NaN.
                self._torque_log.setdefault(mid, []).append(
                    actuator.Torque(raw=0.0, applied=0.0, limit=float("nan"), source="")
                )
                continue
            cal = self.calibration[mid]
            target_rad = math.radians(cal.raw_to_cal(command.position_deg))
            target_vel = math.radians(command.velocity_deg_s * float(cal.sign))
            detail = actuator.torque_detail(
                kp=command.kp,
                kd=command.kd,
                target_rad=target_rad,
                target_vel_rad_s=target_vel,
                pos_rad=self.world.joint_pos_rad(joint),
                vel_rad_s=self.world.joint_vel_rad_s(joint),
                tau_ff=command.torque_nm,
                curve=self.curves[motor.model],
                effort_limit=tables.encoding_for(motor.model).tmax_nm,
            )
            self._torque_log.setdefault(mid, []).append(detail)
            self.world.set_torque(joint, detail.applied)

    def begin_cycle(self) -> None:
        """새 주기가 시작됨. 모아 둔 토크를 비움. **세계가 부름.**

        주기 끝에 비우지 않는 이유: 양다리가 세계 하나를 같이 쓰므로, 한쪽 버스가
        굴린 주기에 다른 쪽 버스도 토크를 넣음. 자기 상태를 읽을 때 비우면 그 사이에
        쌓인 남의 주기 몫이 남아 두 주기가 한 줄로 섞임.
        """
        self._torque_log.clear()

    def torque_report(self) -> Dict[int, Tuple[float, float, float]]:
        """모터 id -> (토크, 상한, 포화비). **지난 한 주기**를 요약한 값임.

            토크    그 주기에서 크기가 가장 컸던 스텝의 토크 (Nm, 부호 있음)
            상한    그 스텝에 걸려 있던 상한 (Nm, 토크와 같은 부호)
            포화비  잘린 스텝 수 / 전체 스텝 수 (0~1)

        주기 하나에 물리 스텝이 열 번이고(0.02 s / 0.002 s) 스텝마다 PD 를 다시
        푸므로 토크가 주기 안에서 움직임. `MotorState.torque_nm` 은 그중 **마지막
        스텝**만 보여서, 중간에 포화했다가 풀리면 안 보임. 그래서 여기서는 가장
        센 스텝을 골라 그 스텝의 상한을 함께 냄.
        """
        out: Dict[int, Tuple[float, float, float]] = {}
        for mid, items in self._torque_log.items():
            if not items:
                continue
            peak = max(items, key=lambda d: abs(d.applied))
            clipped = sum(1 for d in items if d.clipped)
            sign = 1.0 if peak.applied >= 0.0 else -1.0
            out[mid] = (peak.applied, sign * peak.limit, clipped / len(items))
        return out

    def _read_states(self) -> None:
        """MuJoCo 를 읽어 raw 공간 상태로 둠. `Leg` 가 다시 cal 로 되돌림."""
        now = time.monotonic()
        for mid in self.motors:
            joint = self.joints[mid]
            cal = self.calibration[mid]
            pos_cal = math.degrees(self.world.joint_pos_rad(joint))
            vel_cal = math.degrees(self.world.joint_vel_rad_s(joint))
            self._states[mid] = MotorState(
                position_deg=cal.cal_to_raw(pos_cal),
                velocity_deg_s=vel_cal / float(cal.sign),
                torque_nm=self.world.torque_nm(joint),
                temp_c=TEMP_C,
                stamp=now,
            )
