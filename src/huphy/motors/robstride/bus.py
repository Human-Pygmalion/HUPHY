"""RobStride 버스 — 런타임 조작.

세 조각을 잇는 곳임.

    base.py       MotorsBus 계약, MotorState
    codec/mit.py  숫자 <-> 8바이트
    canbus.py     8바이트 <-> CAN 선

각도는 전부 **raw 공간**임. 관절 이름도 모름 — 모터 id 로만 말함. cal 변환과 관절
이름은 `robots/` 가 함.


## 상태는 명령의 응답으로 옴

MIT 모드에는 "상태 읽기" 명령이 따로 없음. 모터는 **동작 명령을 받으면 응답으로
현재 상태를 돌려줌.**

그래서 움직이지 않고 상태만 보려면 게인과 토크가 0인 명령을 보냄.

    tau = kp*(목표각 - 현재각) + kd*(0 - 현재속도) + 토크_FF
        = 0*(...) + 0*(...) + 0
        = 0

토크가 0이니 아무 일도 일어나지 않고 응답만 옴. `refresh_states()` 가 이걸 씀.


## 전송과 수거를 나눠 둠

    send_mit()   보내기만
    collect()    수거만
    refresh_states()   둘 다 (`MotorsBus` 계약)

버스가 둘일 때 "왼다리 보내기 -> 오른다리 보내기 -> 왼다리 수거 -> 오른다리 수거"
순서를 짜려면 나뉘어 있어야 함 (이슈 #10).


## 여기 없는 것

CAN id 변경, 프로토콜 전환, 기계영점, 플래시 저장은 `commissioning.py` 로 감.
되돌리기 어렵고 한 번만 하는 조작이라, 제어 루프에서 부를 수 있는 자리에 두지 않음.
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from typing import Dict, Iterable, List, Mapping, Optional, Tuple

from ..base import Motor, MotorFault, MotorState, MotorsBus, resolve_motor_list
from ..canbus import CanBus, CanFrame
from . import tables
from .codec import mit

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class MitCommand:
    """MIT 동작 명령 하나. 모터 펌웨어가 이 다섯 값으로 토크를 계산함.

        tau = kp*(position_deg - 현재각) + kd*(velocity_deg_s - 현재속도) + torque_nm

    `position_deg` 는 **raw 공간**임.

    다섯 개를 튜플로 넘기지 않는 이유: 순서를 틀려도 타입이 같아 조용히 통과함.
    kp 자리에 위치가 들어가면 모터가 전력으로 튐.
    """

    position_deg: float
    velocity_deg_s: float = 0.0
    kp: float = 0.0
    kd: float = 0.0
    torque_nm: float = 0.0


PASSIVE = MitCommand(position_deg=0.0)
"""토크가 나가지 않는 명령. 응답만 받아 오는 데 씀."""


def _command_frame(motor_id: int, command: int, f_cmd: int = tables.F_CMD_DEFAULT) -> CanFrame:
    """제어 명령 프레임을 만듦.

        data[0:6] = 0xFF    data[6] = F_CMD    data[7] = 명령 바이트

    같은 명령 바이트가 `F_CMD` 에 따라 두 가지 뜻을 가짐 (`tables.py` 참조).
    """
    return CanFrame(
        can_id=int(motor_id),
        data=bytes([0xFF] * 6 + [int(f_cmd) & 0xFF, int(command) & 0xFF]),
    )


class RobStrideBus(MotorsBus):
    """CAN 채널 하나에 물린 RobStride 모터들.

    모터마다 모델이 다를 수 있음 — 다리는 RS02 4개와 RS00 2개임. 모델이 다르면
    인코딩 범위가 다르므로 모터별로 표를 들고 있음.
    """

    def __init__(
        self,
        bus: CanBus,
        motors: Mapping[int, Motor],
        *,
        protocol: tables.Protocol = tables.Protocol.MIT,
    ) -> None:
        self.bus = bus
        self.motors = dict(motors)
        self.protocol = protocol

        # 모델 문자열을 벤더 enum 으로 옮기는 유일한 지점임. 여기서 걸러 두면
        # 제어 중에 오타가 드러나는 일이 없음.
        self._encoding: Dict[int, tables.EncodingRange] = {}
        for motor_id, motor in self.motors.items():
            try:
                model = tables.Model(motor.model)
            except ValueError as e:
                raise ValueError(
                    f"m{motor_id}: 모르는 모델 {motor.model!r}. "
                    f"가용: {[m.value for m in tables.Model]}"
                ) from e
            self._encoding[motor_id] = tables.encoding_for(model, protocol)

        self._states: Dict[int, MotorState] = {mid: MotorState() for mid in self.motors}
        self._torque_on: Dict[int, bool] = {mid: False for mid in self.motors}

    def __repr__(self) -> str:
        return f"RobStrideBus({self.bus.channel}, 모터 {len(self.motors)}개)"

    # ---- 구성 -------------------------------------------------------------
    @property
    def motor_ids(self) -> Tuple[int, ...]:
        return tuple(self.motors)

    @property
    def is_connected(self) -> bool:
        return self.bus.is_connected

    def encoding(self, motor_id: int) -> tables.EncodingRange:
        """모터의 인코딩 범위. RS02 와 RS00 은 토크 범위가 다름."""
        return self._encoding[motor_id]

    # ---- 수명 -------------------------------------------------------------
    def connect(self) -> None:
        self.bus.connect()

    def disconnect(self) -> None:
        """**토크를 먼저 끊고** 채널을 닫음. 여러 번 불려도 안전함.

        순서가 반대면 채널이 닫힌 뒤라 정지 명령을 보낼 방법이 없어짐. 모터는
        마지막 명령을 계속 유지하므로 사람이 전원을 뽑을 때까지 힘을 씀.
        """
        if self.bus.is_connected:
            try:
                self.disable_torque()
            except Exception as e:
                logger.warning("%s 토크 차단 실패 (계속 진행함): %s", self.bus.channel, e)
        self.bus.disconnect()

    # ---- 토크 -------------------------------------------------------------
    def enable_torque(self, motors: Optional[Iterable[int]] = None) -> None:
        """모터를 활성화함. 이 뒤부터 MIT 명령이 실제로 힘을 씀."""
        ids = resolve_motor_list(motors, self.motor_ids)
        self.bus.send_many([_command_frame(mid, tables.CMD_ENABLE) for mid in ids])
        for mid in ids:
            self._torque_on[mid] = True

    def disable_torque(self, motors: Optional[Iterable[int]] = None) -> None:
        """모터를 정지시킴. 힘이 빠져 관절이 중력에 떨어짐."""
        ids = resolve_motor_list(motors, self.motor_ids)
        self.bus.send_many([_command_frame(mid, tables.CMD_STOP) for mid in ids])
        for mid in ids:
            self._torque_on[mid] = False

    def is_torque_on(self, motor_id: int) -> bool:
        """마지막으로 보낸 명령 기준임. 모터에 물어본 값이 아님."""
        return self._torque_on[motor_id]

    def clear_fault(self, motors: Optional[Iterable[int]] = None) -> None:
        """고장 상태를 지움. 원인이 남아 있으면 다시 뜸."""
        ids = resolve_motor_list(motors, self.motor_ids)
        self.bus.send_many([_command_frame(mid, tables.CMD_FAULT) for mid in ids])

    # ---- 명령 -------------------------------------------------------------
    def send_mit(self, commands: Mapping[int, MitCommand]) -> int:
        """MIT 명령들을 **한 번에** 보냄. 보낸 개수를 반환함.

        수거하지 않음 — 응답은 `collect()` 로 받음. 버스가 둘일 때 전송을 먼저
        몰아야 두 다리의 명령 시각이 붙음 (이슈 #10).

        여기서 한계를 검사하지 않음. 그건 `safety.guards` 가 cal 공간에서 이미 함.
        같은 검사를 두 군데 두면 한쪽만 고쳐졌을 때 어느 쪽이 맞는지 알 수 없음.
        """
        unknown = [mid for mid in commands if mid not in self.motors]
        if unknown:
            raise ValueError(
                f"이 버스에 없는 모터 id: {unknown} (가용: {list(self.motor_ids)})"
            )

        frames = [
            CanFrame(
                can_id=mid,
                data=mit.pack_command(
                    position_deg=c.position_deg,
                    velocity_deg_s=c.velocity_deg_s,
                    kp=c.kp,
                    kd=c.kd,
                    torque_nm=c.torque_nm,
                    enc=self._encoding[mid],
                ),
            )
            for mid, c in commands.items()
        ]
        return self.bus.send_many(frames)

    # ---- 상태 -------------------------------------------------------------
    def collect(self, *, expect: Optional[int] = None, timeout_s: float = 0.002) -> List[int]:
        """도착한 응답을 해석해 캐시에 넣음. **응답이 없었던 모터 id** 를 반환함.

        `expect` 를 주면 그만큼 채웠을 때 즉시 빠져나감. 정상 주기에는 예산을
        쓰지 않음.
        """
        frames = self.bus.drain(expect=expect, timeout_s=timeout_s)

        seen = set()
        now = time.monotonic()
        for f in frames:
            try:
                mid, pos, vel, tau, temp = mit.decode_state(f.data, enc=self._encoding_for_frame(f))
            except (ValueError, KeyError) as e:
                logger.debug("%s 해석 실패 %s: %s", self.bus.channel, f.data.hex(), e)
                continue
            if mid not in self.motors:
                logger.debug("%s 모르는 모터 응답 id=%d", self.bus.channel, mid)
                continue
            self._states[mid] = MotorState(
                position_deg=pos,
                velocity_deg_s=vel,
                torque_nm=tau,
                temp_c=temp,
                stamp=now,
            )
            seen.add(mid)

        return [mid for mid in self.motor_ids if mid not in seen]

    def _encoding_for_frame(self, frame: CanFrame) -> tables.EncodingRange:
        """응답 프레임의 인코딩 범위를 고름.

        모터 id 가 프레임 안(`data[0]`)에 있으므로 그걸로 찾음. CAN 중재 id 는
        모델을 알려주지 않고, 다리에는 RS02 와 RS00 이 섞여 있어 범위가 다름.
        """
        if not frame.data:
            raise ValueError("빈 프레임")
        return self._encoding[frame.data[0]]

    def refresh_states(self, motors: Optional[Iterable[int]] = None) -> List[int]:
        """상태를 요청하고 응답을 수거함. 응답이 없었던 모터 id 를 반환함.

        게인과 토크가 0인 명령을 보냄 — MIT 모드에는 읽기 전용 명령이 없고, 모터는
        동작 명령의 응답으로 상태를 돌려줌. 토크가 0이라 아무 일도 일어나지 않음.

        제어 루프에서는 이걸 쓰지 않음. 이미 보낸 명령의 응답을 `collect()` 로
        받으면 되므로, 프레임을 두 배로 보낼 이유가 없음.

        **요청 없이 받는 길도 있음** — Command 13(`tables.CMD_ACTIVE_REPORT`)으로
        능동 보고를 켜면 모터가 스스로 상태 프레임을 보냄(기본 10ms 주기,
        `EPScan_time` 으로 조정). 미구현이고, 제어 루프에는 쓸 것이 못 됨 —
        모터 타이머 기준이라 상태 시각이 명령과 안 붙음(이슈 #10). 명령 없이 상태만
        보는 자리(브링업·모니터링)에 맞음.
        """
        ids = resolve_motor_list(motors, self.motor_ids)
        self.bus.flush_rx()
        self.send_mit({mid: PASSIVE for mid in ids})
        missing = self.collect(expect=len(ids))
        return [mid for mid in ids if mid in set(missing)]

    def state(self, motor_id: int) -> MotorState:
        """마지막으로 수거된 상태. 새로 읽지 않음."""
        return self._states[motor_id]

    def states(self) -> Dict[int, MotorState]:
        return dict(self._states)

    # ---- 고장 -------------------------------------------------------------
    def read_fault(self, motor_id: int, *, timeout_s: float = 0.01) -> Optional[MotorFault]:
        """고장값을 조회함. 응답이 없으면 `None`.

        조회 전에 큐를 비움 — 고장 응답은 일반 상태 프레임과 CAN ID 가 같아
        구분되지 않으므로, 묵은 프레임이 남아 있으면 그걸 고장값으로 읽음.
        """
        self.bus.flush_rx()
        self.bus.send(
            _command_frame(motor_id, tables.CMD_FAULT, f_cmd=tables.F_CMD_FAULT_QUERY)
        )
        for f in self.bus.drain(expect=1, timeout_s=timeout_s):
            try:
                mid, word = mit.decode_fault(f.data)
            except ValueError:
                continue
            if mid != motor_id:
                continue
            return MotorFault(
                raw=word,
                bits={name: bool(word & (1 << bit)) for name, bit in tables.FAULT_BITS.items()},
            )
        return None
