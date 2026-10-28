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
from typing import Dict, Iterable, List, Mapping, Optional, Sequence, Set, Tuple

from ..base import Motor, MotorFault, MotorState, MotorsBus, resolve_motor_list
from ..canbus import DEFAULT_DRAIN_S, CanBus, CanFrame
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
        drain_s: float = DEFAULT_DRAIN_S,
    ) -> None:
        if drain_s < 0:
            raise ValueError(f"drain_s 는 0 이상이어야 함 (받은 값 {drain_s})")
        self.bus = bus
        self.motors = dict(motors)
        self.protocol = protocol
        self.drain_s = float(drain_s)
        """응답을 기다리는 최대 시간 (초). `collect` 와 `refresh_states` 가 씀.

        기다린 개수가 다 오면 즉시 빠져나가므로 **정상 주기에는 이만큼 쓰지 않음.**
        응답이 늦거나 빠질 때만 끝까지 기다림.

        크게 잡으면 늦게 온 응답을 이번 주기에 받아 무응답이 줄지만, 모터 하나가
        죽으면 매 주기 이만큼을 통째로 씀 -- 주기 예산을 넘겨 밀림이 됨. 작게 잡으면
        그 반대임. 기본 2ms 는 100Hz 예산 10ms 의 20% 임.
        """

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

        self.latency = None
        """지연 기록기. 붙이면 `sent(mid)` 와 `received(mid, 초, late)` 를 부름.

        기본은 없음 -- 없으면 시각을 재지도 않음. `telemetry.latency_log` 가 붙임.
        """
        self._sent_at: Dict[int, float] = {}
        self._prev_sent_at: Dict[int, float] = {}
        self.late_counts: Dict[int, int] = {mid: 0 for mid in self.motors}
        """모터별 누적. 이번 명령보다 **먼저 도착해 있던** (= 직전 주기의) 응답을 받은 횟수.

        그 주기에는 응답이 없던 것이므로 `collect` 는 그 모터를 무응답으로 돌려줌.
        늦게 온 응답이라 상태 캐시에는 반영하지만 "이번 주기에 답함" 으로 세지 않음.
        """

        self.reply_ids: Dict[int, int] = {}
        """모터 id -> 그 모터의 상태 프레임이 실려 온 11비트 CAN id (= 호스트 id).

        `Leg` 가 연결할 때 `tables.HOST_ID_BASE + id` 인지 봄 -- 아니면 burst 가
        안전하지 않음 (docs/motor_setup.md 14절)."""

        # ---- 파라미터 읽기 (Command 14) ----
        self.params: Dict[int, Dict[int, Tuple[float, float]]] = {mid: {} for mid in self.motors}
        """모터 id -> 인덱스 -> (값, 도착 시각). 마지막으로 읽은 것. `Leg.get_observation`
        이 여기서 꺼내 `<모터>.iq` 등으로 냄."""
        self.param_counts: Dict[int, Dict[str, int]] = {
            mid: {"sent": 0, "ok": 0, "late": 0} for mid in self.motors
        }
        """모터별 누적. ok = 그 주기 수거 안에 옴, late = 다음 주기 이후에 옴 (값은
        씀 -- 누구 것인지 CAN id 로 앎). 끝내 안 온 것은 sent - ok - late."""
        self._pending_reads: Set[Tuple[int, int]] = set()
        """이번 주기에 물은 것. 수거 안에 오면 ok 로 셈."""
        self._overdue_reads: Set[Tuple[int, int]] = set()
        """지난 주기에 물었는데 아직 안 온 것. 나중에 오면 late 로 셈."""

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
    def send_mit(
        self,
        commands: Mapping[int, MitCommand],
        reads: Sequence[Tuple[int, int]] = (),
    ) -> int:
        """MIT 명령들을 **한 번에** 보냄. 보낸 개수를 반환함.

        수거하지 않음 — 응답은 `collect()` 로 받음. 버스가 둘일 때 전송을 먼저
        몰아야 두 다리의 명령 시각이 붙음 (이슈 #10).

        `reads` 를 주면 명령 뒤에 파라미터 읽기(Command 14)를 같이 실어 보냄 --
        `(모터 id, 인덱스)` 목록. 응답은 `collect()` 가 같이 받음. **모터마다 호스트
        id 가 달라야 함** -- 읽기 응답에는 모터 id 가 없어서 CAN id 로 가림.

        여기서 한계를 검사하지 않음. 그건 `safety.guards` 가 cal 공간에서 이미 함.
        같은 검사를 두 군데 두면 한쪽만 고쳐졌을 때 어느 쪽이 맞는지 알 수 없음.
        """
        unknown = [mid for mid in commands if mid not in self.motors]
        unknown += [mid for mid, _ in reads if mid not in self.motors]
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
        frames += [
            CanFrame(can_id=mit.param_read_id(mid), data=mit.pack_param_read(index))
            for mid, index in reads
        ]
        self._note_sent(commands)
        self._note_reads(reads)
        return self.bus.send_many(frames)

    def _note_reads(self, reads: Sequence[Tuple[int, int]]) -> None:
        """이번에 물은 것을 기억함. 지난번에 물었는데 안 온 것은 '늦음 후보' 로 넘김."""
        self._overdue_reads |= self._pending_reads
        self._pending_reads = set((int(m), int(i)) for m, i in reads)
        for mid, _ in self._pending_reads:
            self.param_counts[mid]["sent"] += 1

    def _note_sent(self, commands: Mapping[int, MitCommand]) -> None:
        """보낸 시각을 모터별로 남김. **묶음을 보내기 직전 한 시각**을 씀.

        프레임별 시각이 아닌 이유: 제어 루프가 겪는 지연은 "명령을 내기로 한 때부터
        응답이 손에 들어온 때까지" 임. slcan 에서 앞 프레임이 직렬로 나가는 동안 뒤
        프레임이 기다리는 시간도 그 안에 들어가야 함.

        직전 시각도 들고 있음 -- 다음 주기에 늦게 들어온 응답을 그 명령에 맞추려고.
        지연 기록기가 없어도 남김 -- 늦은 응답을 가려내는 데(`_absorb`) 필요함.
        """
        now = time.monotonic()
        for mid in commands:
            if mid in self._sent_at:
                self._prev_sent_at[mid] = self._sent_at[mid]
            self._sent_at[mid] = now
            if self.latency is not None:
                self.latency.sent(mid)

    def _note_received(self, mid: int, stamp: float) -> None:
        """응답 하나의 지연을 남김.

        MIT 응답에는 순번이 없음. 그래서 **도착 시각이 이번 송신보다 앞이면** 직전
        명령의 응답이 늦게 온 것으로 봄 (`late`). 수신 스레드가 도착할 때 시각을
        찍으므로 이 구분이 성립함.
        """
        sent = self._sent_at.get(mid)
        if sent is None or not stamp:
            return
        if stamp >= sent:
            self.latency.received(mid, stamp - sent, False)
            return
        prev = self._prev_sent_at.get(mid)
        if prev is not None and stamp >= prev:
            self.latency.received(mid, stamp - prev, True)

    # ---- 상태 -------------------------------------------------------------
    def collect(
        self, *, expect: Optional[int] = None, timeout_s: Optional[float] = None
    ) -> List[int]:
        """도착한 응답을 해석해 캐시에 넣음. **응답이 없었던 모터 id** 를 반환함.

        `expect` 를 주면 그만큼 채웠을 때 즉시 빠져나감. 정상 주기에는 예산을
        쓰지 않음.

        `timeout_s` 를 안 주면 `self.drain_s` 를 씀. 예전에는 여기에 0.002 가 따로
        박혀 있어서 `canbus.DEFAULT_DRAIN_S` 를 바꿔도 제어 경로에는 안 닿았음.

        `expect` 는 **이번 주기 몫의 응답** 수임 -- 상태 프레임과, 이번에 실어 보낸
        파라미터 읽기의 응답. 지난 주기의 늦은 응답은 세지 않고 계속 받음. 프레임
        개수로만 세면 늦게 온 것 하나가 자리를 차지해, 이번 응답 하나를 남겨 둔 채
        빠져나가고 그 모터가 무응답으로 잡힘.
        """
        wait = self.drain_s if timeout_s is None else float(timeout_s)
        deadline = time.monotonic() + max(0.0, wait)
        seen: Set[int] = set()
        got = 0
        budget = wait                       # 첫 번은 예산 그대로, 그다음은 남은 만큼
        while True:
            need = None if expect is None else max(0, int(expect) - got)
            frames = self.bus.drain(expect=need, timeout_s=max(0.0, budget))
            budget = deadline - time.monotonic()
            new_seen, n_reads = self._absorb_counted(frames)
            got += len(new_seen - seen) + n_reads
            seen |= new_seen
            if expect is None or got >= expect or not frames:
                break
            if time.monotonic() >= deadline:
                break
        return [mid for mid in self.motor_ids if mid not in seen]

    def collect_until(self, motor_id: int, *, timeout_s: Optional[float] = None) -> Set[int]:
        """`motor_id` 의 응답이 올 때까지 받음. **받은 모든 모터 id** 를 반환함.

        하나씩 보내기(`Leg.send_mode == "seq"`)가 씀 -- 한 모터에 보내고 그 응답을
        받은 뒤 다음 모터로 감. 기다리는 동안 다른 모터의 늦은 응답이 들어오면 그것도
        해석해 캐시에 넣음 (버리면 그 모터가 무응답으로 잡힘).

        `timeout_s` 를 안 주면 `self.drain_s` 를 씀 -- 이 모터 하나에 쓰는 시간임.
        """
        wait = self.drain_s if timeout_s is None else float(timeout_s)
        deadline = time.monotonic() + max(0.0, wait)
        seen: Set[int] = set()
        while motor_id not in seen:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            seen |= self._absorb(self.bus.drain(expect=1, timeout_s=remaining))
        return seen

    def _absorb(self, frames: Iterable[CanFrame]) -> Set[int]:
        """응답 프레임을 해석해 캐시에 넣음. **이번 명령에 대한 응답을 준** 모터 id 를 반환함.

        도착 시각이 이번 송신보다 앞선 프레임은 직전 주기 명령의 늦은 응답임. 상태
        캐시에는 넣되(최신 값임) 반환하는 집합에는 넣지 않음 -- 제어 입장에서 그
        주기는 응답이 없었던 것이므로 무응답으로 집계되어야 함. 안 그러면 한 주기
        밀려 온 응답이 다음 주기의 "정상" 으로 둔갑해 유실이 통계에서 사라짐.

        수신 스레드가 없으면 도착 시각이 수거 시각이라 늦은 것을 가릴 수 없음 (전부
        이번 응답으로 봄).
        """
        return self._absorb_counted(frames)[0]

    def _absorb_counted(self, frames: Iterable[CanFrame]) -> Tuple[Set[int], int]:
        """`_absorb` 와 같고, 이번 주기에 물은 파라미터 읽기의 응답 수를 같이 냄.

        **프레임 가르기.** 호스트 id 가 `HOST_ID_BASE + 모터 id` 로 설정돼 있으면
        CAN id 로 모터를 앎 (docs/motor_setup.md 14절):

            data[0] == 그 모터 id                 상태 프레임 (Response Command 1)
            data[2:4] == 00 00, data[1] == 0x70   파라미터 읽기 응답 (Command 14)

        상태 프레임의 data[0] 은 늘 모터 id 라 둘이 섞이지 않음. 호스트 id 가 규칙과
        다르면(출하값 0xFD 등) 예전처럼 data[0] 으로 상태 프레임만 봄.
        """
        seen: Set[int] = set()
        n_reads = 0
        now = time.monotonic()
        for f in frames:
            by_id = f.can_id - tables.HOST_ID_BASE
            if (
                by_id in self.motors and len(f.data) >= 8 and f.data[0] != by_id
                and f.data[1] == 0x70 and f.data[2] == 0 and f.data[3] == 0
            ):
                n_reads += self._absorb_param(by_id, f, now)
                continue
            try:
                mid, pos, vel, tau, temp = mit.decode_state(f.data, enc=self._encoding_for_frame(f))
            except (ValueError, KeyError) as e:
                logger.debug("%s 해석 실패 %s: %s", self.bus.channel, f.data.hex(), e)
                continue
            if mid not in self.motors:
                logger.debug("%s 모르는 모터 응답 id=%d", self.bus.channel, mid)
                continue
            self.reply_ids[mid] = f.can_id
            self._states[mid] = MotorState(
                position_deg=pos,
                velocity_deg_s=vel,
                torque_nm=tau,
                temp_c=temp,
                stamp=now,
            )
            sent = self._sent_at.get(mid)
            if sent is not None and f.stamp and f.stamp < sent:
                self.late_counts[mid] += 1
            else:
                seen.add(mid)
            if self.latency is not None:
                self._note_received(mid, f.stamp)
        return seen, n_reads

    def _absorb_param(self, mid: int, f: CanFrame, now: float) -> int:
        """파라미터 읽기 응답 하나. 이번 주기에 물은 것이면 1 을 냄.

        늦게 온 것도 값은 씀 -- CAN id 로 누구 것인지 알므로 다른 모터 값으로 착각할
        일이 없음. 도착 시각이 그 값의 시각임.
        """
        index = f.data[0] | (f.data[1] << 8)
        value = mit.decode_param_float(f.data)
        stamp = f.stamp or now
        key = (mid, index)
        if key in self._pending_reads:
            self._pending_reads.discard(key)
            self.param_counts[mid]["ok"] += 1
            counted = 1
        elif key in self._overdue_reads:
            self._overdue_reads.discard(key)
            self.param_counts[mid]["late"] += 1
            counted = 0
        else:
            return 0                                # 묻지 않은 것. 다른 프로그램 몫
        self.params[mid][index] = (value, stamp)
        return counted

    def _encoding_for_frame(self, frame: CanFrame) -> tables.EncodingRange:
        """응답 프레임의 인코딩 범위를 고름.

        모터 id 가 프레임 안(`data[0]`)에 있으므로 그걸로 찾음. CAN 중재 id 는
        모델을 알려주지 않고, 다리에는 RS02 와 RS00 이 섞여 있어 범위가 다름.
        """
        if not frame.data:
            raise ValueError("빈 프레임")
        return self._encoding[frame.data[0]]

    def refresh_states(
        self,
        motors: Optional[Iterable[int]] = None,
        reads: Sequence[Tuple[int, int]] = (),
    ) -> List[int]:
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
        self.send_mit({mid: PASSIVE for mid in ids}, reads=reads)
        missing = self.collect(expect=len(ids) + len(reads))
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
