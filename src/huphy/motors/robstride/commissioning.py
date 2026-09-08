"""RobStride 커미셔닝 — 조립할 때 한 번 하는 조작.

제어 루프에서 쓰는 것이 하나도 없음. `bus.py` 와 파일을 나눈 이유임 — 되돌리기
어려운 조작이 제어 경로에서 손 닿는 곳에 있으면 안 됨.

여기 있는 것은 전부 **한 모터씩** 처리함. 여러 모터에 한꺼번에 거는 형태를 두지
않음. CAN id 를 바꾸다 잘못되면 어느 모터가 어떻게 됐는지 알아내기 어렵고, 영점은
자세를 잡아 놓고 하나씩 하는 작업임.


## MIT 표준 프레임으로 되는 것만 있음

이 파일의 명령은 전부 11-bit 표준 프레임이고, 11비트 ID 가 통째로 모터 CAN ID 인
형식만 씀 (매뉴얼 6.3~6.11절).

    11-bit ID = 모터 CAN ID
    data[0:6] = 0xFF    data[6] = F_CMD    data[7] = 명령 코드

**파라미터 읽기·쓰기는 여기 없음.** `PARAM_PROTOCOL_FLAG`(0x201F) 와
`PARAM_ZERO_STA`(0x7029) 를 다루는 명령이 아직 없음. 필요해지면 추가함.

MIT 모드에서 **파라미터를 못 만지는 것이 아님** -- Command 14·15 가 있음
(`tables.SEL_PARAM_READ` / `SEL_PARAM_WRITE`). 다만 그 명령들은 11비트 ID 상위
3비트에 선택자를 싣는 **다른 형식**이라 `_command_frame` 으로는 못 만듦. 붙이려면
프레임 조립부터 손대야 함. 지금은 사람이 `docs/motor_setup.md` 절차대로 private
프로토콜에서 cansend 로 함.


## 응답 확인

이 명령들은 응답이 상태 프레임으로 옴. 하지만 **명령이 반영됐는지를 응답만으로는
알 수 없음** — 상태 프레임에는 CAN id 도 프로토콜도 실리지 않음.

그래서 응답 유무만 확인하고, 실제 반영 여부는 사람이 확인해야 함. `set_can_id` 는
바꾼 뒤 새 id 로 응답이 오는지까지 봄.
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from typing import Callable, Dict, Iterable, List, Optional

from ..base import MotorState, resolve_motor_list, wrap180
from . import tables
from .bus import MitCommand, PASSIVE, RobStrideBus, _command_frame

logger = logging.getLogger(__name__)

SETTLE_S = 0.05
"""명령 사이에 두는 간격. 플래시 쓰기가 있는 조작은 즉시 응답하지 않음."""


class CommissioningError(RuntimeError):
    """커미셔닝 조작이 확인되지 않음."""


def _send_and_confirm(
    bus: RobStrideBus, motor_id: int, command: int, f_cmd: int, *, timeout_s: float = 0.1
) -> bool:
    """명령을 보내고 응답이 오는지 봄.

    큐를 먼저 비움 — 묵은 상태 프레임이 남아 있으면 그걸 응답으로 착각함.
    """
    bus.bus.flush_rx()
    bus.bus.send(_command_frame(motor_id, command, f_cmd=f_cmd))
    frames = bus.bus.drain(expect=1, timeout_s=timeout_s)
    return any(f.data and f.data[0] == motor_id for f in frames)


def set_control_mode(
    bus: RobStrideBus, motor_id: int, mode: tables.ControlMode
) -> None:
    """제어 모드를 바꿈 (Command 6). 즉시 적용되고 전원 재투입이 필요 없음.

    본 프로젝트는 `ControlMode.MIT` 를 씀 — 궤적을 직접 만들고 게인을 매 프레임
    실어 보내야 하기 때문임. 전원 투입 기본값도 MIT 임.

    `ControlMode` 는 무엇을 명령할지를 정하는 것이고, `Protocol` 은 프레임 포맷을
    정하는 것임. 이름이 겹치지만 서로 독립임 (`tables.py` 참조).
    """
    if not _send_and_confirm(bus, motor_id, tables.CMD_SET_MODE, int(mode)):
        raise CommissioningError(f"m{motor_id}: 제어 모드 {mode.name} 설정에 응답이 없음")
    logger.info("m%d 제어 모드 -> %s", motor_id, mode.name)


def set_zero(bus: RobStrideBus, motor_id: int, *, zero_reference: str) -> None:
    """지금 자세를 기계 영점으로 잡음 (Command 4). **전원 재투입 후에도 남음.**

    실물에서 확인함 — 영점을 잡고 전원을 끊었다 켜도 유지됨. 그래서 이 조작이
    시작 절차가 아니라 커미셔닝에 속함.

    `zero_reference` 는 어느 자세에서 잡았는지 사람이 남기는 메모임. 필수 인자로 둔
    이유: 모터는 영점 값을 저장하지만 **"그때 다리가 어떤 자세였는지" 는 어디에도
    남지 않음.** 이 메모가 없으면 나중에 영점을 재현할 수 없고, 재현할 수 없으면
    `offset` 실측이 무의미해짐.

    반환하지 않고 문자열만 받는 이유: 이 값은 `MotorCalibration.zero_reference` 로
    가는 것이고, 파일에 쓰는 것은 `calibration.store` 가 함. 여기는 모터에 명령을
    보내는 것까지만 함.

    **토크가 켜져 있으면 거부함.** 영점을 잡는 순간 모터의 좌표계가 통째로 옮겨가는데
    직전 명령의 목표각은 옛 좌표계 값임. 그대로 유지되면 그 차이만큼 관절이 튐.

    비위치 모드에서만 동작함 (매뉴얼 p.38). MIT 모드는 여기 해당함.
    """
    if not zero_reference.strip():
        raise ValueError(
            f"m{motor_id}: zero_reference 가 비어 있음. "
            f"어느 자세에서 영점을 잡는지 적을 것 (예: '다리 편 상태, 발바닥 평면 접촉')"
        )
    if bus.is_torque_on(motor_id):
        raise CommissioningError(
            f"m{motor_id}: 토크가 켜져 있음. 영점을 잡으면 좌표계가 옮겨가는데 "
            f"직전 목표각은 옛 좌표계 값이라 그 차이만큼 관절이 튐. "
            f"disable_torque() 를 먼저 부를 것"
        )

    if not _send_and_confirm(bus, motor_id, tables.CMD_SET_ZERO, tables.F_CMD_DEFAULT):
        raise CommissioningError(f"m{motor_id}: 영점 설정에 응답이 없음")

    time.sleep(SETTLE_S)
    logger.info("m%d 기계 영점 설정: %s", motor_id, zero_reference)


def set_can_id(bus: RobStrideBus, motor_id: int, new_id: int) -> None:
    """CAN id 를 바꿈 (Command 7). **전원 재투입 후에도 남음.**

    실물에서 확인함 — 매뉴얼은 "effective immediately" 라고만 하고 영속성을 안
    적어 놨지만, 실제로는 저장 명령(Command 12) 없이도 유지됨. `set_zero` 와 같음.

    **이 버스의 구성과 어긋나게 됨.** 바꾼 뒤에는 이 `RobStrideBus` 객체로 그 모터를
    다룰 수 없으므로, 설정 파일을 고치고 새로 만들어야 함.

    한 모터씩 하는 이유: 여러 개를 연속으로 바꾸다 중간에 실패하면 어느 것이 옛 id
    이고 어느 것이 새 id 인지 알 수 없음. 같은 id 가 둘이 되면 응답이 충돌해서
    구분조차 안 됨.
    """
    if not 1 <= int(new_id) <= 0x7F:
        raise ValueError(f"CAN id 는 1~127 이어야 함 (받은 값 {new_id})")
    if int(new_id) == int(motor_id):
        raise ValueError(f"m{motor_id}: 새 id 가 현재 id 와 같음")
    if int(new_id) in bus.motor_ids:
        raise CommissioningError(
            f"m{motor_id}: id {new_id} 는 이 버스의 다른 모터가 쓰고 있음. "
            f"같은 id 가 둘이 되면 응답이 충돌해 구분되지 않음"
        )

    bus.bus.flush_rx()
    bus.bus.send(_command_frame(motor_id, tables.CMD_SET_CAN_ID, f_cmd=int(new_id)))
    time.sleep(SETTLE_S)

    # 새 id 로 응답이 오는지 확인함. 상태 프레임에는 id 가 실리므로 이건 확인 가능함.
    bus.bus.flush_rx()
    bus.bus.send(_command_frame(new_id, tables.CMD_STOP, f_cmd=tables.F_CMD_DEFAULT))
    frames = bus.bus.drain(expect=1, timeout_s=0.1)
    if not any(f.data and f.data[0] == int(new_id) for f in frames):
        raise CommissioningError(
            f"m{motor_id}: id 를 {new_id} 로 바꿨는데 새 id 로 응답이 없음. "
            f"양쪽 id 로 다시 확인할 것 -- 반영됐는데 응답만 놓쳤을 수 있음"
        )
    logger.info("m%d CAN id -> %d. 설정 파일을 고칠 것", motor_id, new_id)


def set_protocol(bus: RobStrideBus, motor_id: int, protocol: tables.Protocol) -> None:
    """통신 프로토콜을 바꿈 (Command 8). **전원 재투입 후 적용됨.**

    프레임 포맷 자체가 바뀌므로, 재투입 전까지는 옛 포맷으로 계속 통신해야 함.
    재투입 후에는 이 코드가 보내는 MIT 표준 프레임이 안 먹을 수 있음.

    공장 기본값은 private(29-bit 확장)이고 이 코드는 MIT(11-bit 표준)를 씀.
    **안 맞으면 명령이 무시되고 에러도 나지 않음** — 연결도 되고 코드도 안 죽는데
    모터만 안 움직임.

    바뀌었는지 이 함수는 확인하지 않음. 재투입 후 `scan` 이 응답하면 MIT 임 --
    표준 프레임에 답한다는 것 자체가 근거임.
    """
    if not _send_and_confirm(bus, motor_id, tables.CMD_SET_PROTOCOL, int(protocol)):
        raise CommissioningError(f"m{motor_id}: 프로토콜 전환에 응답이 없음")
    logger.warning(
        "m%d 프로토콜 -> %s. 전원을 재투입해야 적용됨", motor_id, protocol.name
    )


@dataclass
class NudgeResult:
    """`nudge` 가 관찰한 것. 모터 id ↔ 관절 매핑 확인에 씀 (이슈 #8)."""

    motor_id: int
    start_deg: float
    peak_deg: float
    end_deg: float
    samples: List[MotorState]

    @property
    def moved_deg(self) -> float:
        """실제로 움직인 양. 명령한 양과 다르면 게인이 낮거나 걸린 것임."""
        return self.peak_deg - self.start_deg


def nudge(
    bus: RobStrideBus,
    motor_id: int,
    *,
    delta_deg: float = 5.0,
    kp: float = 5.0,
    kd: float = 0.5,
    steps: int = 20,
    hz: float = 100.0,
) -> NudgeResult:
    """모터 하나를 조금 움직였다 되돌림. **사람이 보고 어느 관절인지 확인하는 용도임.**

    이슈 #8(모터 id ↔ 관절 매핑 실물 미확인)을 해소하는 절차임. 설정에는
    `7=hip_pitch 8=hip_roll 9=hip_yaw 10=knee 11=ankle_a 12=ankle_b` 로 되어 있지만 실물로
    확인된 적이 없음.

    **현재 위치를 기준으로 상대 이동함.** 절대 한계를 쓰지 않는 이유: 이 시점에는
    캘리브레이션이 없어 cal 공간이 존재하지 않고, 따라서 `Motor.limits_deg` 를
    적용할 수 없음. 지금 있는 자리에서 조금 움직이는 것만 안전하게 할 수 있음.

    **기본 게인이 낮음.** 브링업 초반이고 사람이 옆에 있음. 걸리면 못 움직이고 마는
    편이 낫지, 뚫고 나가면 안 됨.

    호출 전에 다리를 받쳐 둘 것. 중력을 이길 만큼의 게인이 아니므로 무릎처럼 하중을
    받는 관절은 지지 없이는 움직이지 않거나 처짐.
    """
    if motor_id not in bus.motor_ids:
        raise ValueError(f"이 버스에 없는 모터 id: {motor_id} (가용: {list(bus.motor_ids)})")
    if abs(float(delta_deg)) > 20.0:
        raise ValueError(
            f"nudge 는 확인용이라 20도까지만 허용함 (받은 값 {delta_deg}). "
            f"본격적인 동작은 control/ 에서 할 것"
        )

    missing = bus.refresh_states([motor_id])
    if missing:
        raise CommissioningError(
            f"m{motor_id}: 응답이 없음. 배선과 CAN id 를 확인할 것"
        )
    start = bus.state(motor_id).position_deg

    dt = 1.0 / float(hz)
    n = max(1, int(steps))
    samples: List[MotorState] = []
    peak = start

    bus.enable_torque([motor_id])
    try:
        # 갔다가 돌아옴. 끝 위치가 시작과 같아야 정상임.
        targets = [start + delta_deg * (i + 1) / n for i in range(n)]
        targets += [start + delta_deg * (n - i - 1) / n for i in range(n)]

        for target in targets:
            bus.send_mit({motor_id: MitCommand(position_deg=target, kp=kp, kd=kd)})
            bus.collect(expect=1, timeout_s=dt)
            st = bus.state(motor_id)
            samples.append(st)
            if abs(st.position_deg - start) > abs(peak - start):
                peak = st.position_deg
            time.sleep(dt)

        # 게인을 뺀 명령으로 마무리함. 토크를 바로 끊으면 관절이 떨어짐.
        bus.send_mit({motor_id: PASSIVE})
        bus.collect(expect=1, timeout_s=dt)
    finally:
        bus.disable_torque([motor_id])

    end = bus.state(motor_id).position_deg
    logger.info(
        "m%d nudge: 시작 %.2f -> 최대 %.2f -> 끝 %.2f (명령 %.1f도)",
        motor_id, start, peak, end, delta_deg,
    )
    return NudgeResult(
        motor_id=motor_id, start_deg=start, peak_deg=peak, end_deg=end, samples=samples
    )


@dataclass
class SweepResult:
    """`sweep` 이 관찰한 것. 관절 하나의 가동 범위임."""

    motor_id: int
    lo_deg: float
    hi_deg: float
    samples: int
    offset_deg: float = 0.0

    @property
    def span_deg(self) -> float:
        return self.hi_deg - self.lo_deg


def measure_offset(
    bus: RobStrideBus, motors: Optional[Iterable[int]] = None
) -> Dict[int, float]:
    """**지금 자세를 관절 0도로 놓는** 오프셋을 냄.

    `cal = sign x raw + offset` 이고 설계대로 조립하면 `sign` 이 1 이므로,
    지금 각도가 0으로 읽히려면 `offset = -raw` 임.

    기계 영점(`set_zero`)과 다른 것임 -- 그쪽은 모터가 각도를 **어디부터 세는지**를
    정하고, 이쪽은 그렇게 센 각도 중 **어디를 관절 0도라 부를지**를 정함. 영점을
    제대로 잡았으면 여기서 0 근처가 나옴. 그것이 곧 확인임.

    토크를 끊고 부름 -- 사람이 자세를 잡고 있는 중임.
    """
    ids = resolve_motor_list(motors, bus.motor_ids)
    bus.disable_torque(ids)

    missing = bus.refresh_states(ids)
    if missing:
        raise CommissioningError(
            f"응답이 없음: {missing}. 배선과 CAN id 를 확인할 것"
        )

    offsets = {mid: -bus.state(mid).position_deg for mid in ids}
    logger.info(
        "관절 0도 기준: %s",
        ", ".join(f"m{mid} offset {value:+.2f}" for mid, value in offsets.items()),
    )
    return offsets


def sweep(
    bus: RobStrideBus,
    motors: Optional[Iterable[int]] = None,
    *,
    should_stop: Callable[[], bool],
    on_update: Optional[Callable[[Dict[int, "SweepResult"], Dict[int, float]], None]] = None,
    hz: float = 20.0,
    offsets: Optional[Dict[int, float]] = None,
) -> Dict[int, SweepResult]:
    """토크를 끄고 **사람이 손으로 미는 동안** 최대·최소를 기록함.

    관절 가동 범위를 재는 유일한 방법임. 설계값이 없거나 실물과 맞는지 확인할 때 씀.

    **토크를 먼저 끊음.** 힘이 들어간 채로 밀면 모터와 싸우게 되고, 손으로 밀 수
    있는 범위가 실제 가동 범위보다 좁게 나옴.

    `should_stop` 이 `True` 를 낼 때까지 돎. 호출부가 키 입력이나 시간으로 정함.

    반환: 모터 id -> `SweepResult`.


    ## 각도는 관절 좌표계임

    `offsets` 를 받아 `cal = raw + offset` 으로 바꿔 기록함. `measure_offset` 이
    관절 0도 자세에서 낸 값을 그대로 넣으면 됨.

    `[-180, 180)` 으로 접음. 모터가 그 범위로만 보고하므로 관절이 경계를 지나면
    보고값이 360 만큼 건너뛰는데, 접지 않으면 그 건너뜀이 그대로 최소·최대에
    들어감 (`motors/base.py` 의 `wrap180`).

    **0도를 먼저 정하고 범위를 재는 순서임.** 그래야 나온 최대·최소가 그대로
    `robot.yaml` 의 `limits_deg` 가 됨 -- 나중에 빼고 더할 일이 없음.

    `offsets` 를 생략하면 0으로 봄. 그때는 raw 공간 값임.


    ## 값이 사람 손에 달려 있음

    끝까지 안 밀면 그만큼 좁게 나옴. **하드스톱에 닿는 느낌을 확인하며** 천천히
    밀 것.

    한 번 더 돌려서 같은 값이 나오는지 보는 것이 확인 방법임.


    ## 발목은 발을 움직임

    두 모터가 로드로 발판에 물려 있어 **한쪽만 손으로 돌릴 수 없음.** 발을 잡고
    움직이면 두 모터가 같이 따라옴 -- 둘 다 토크가 꺼져 있으므로 서로 밀리지 않음.

    그렇게 얻은 두 범위는 각 모터가 **어떤 자세에서든 가질 수 있는 값의 범위**임.
    두 최대값을 동시에 가지는 자세는 없을 수 있으나, 명령은 IK 가 만든 짝에서
    시작하므로 문제되지 않음.
    """
    ids = resolve_motor_list(motors, bus.motor_ids)
    shift = dict(offsets or {})
    bus.disable_torque(ids)

    missing = bus.refresh_states(ids)
    if set(missing) == set(ids):
        raise CommissioningError(
            f"응답이 없음: {missing}. 배선과 CAN id 를 확인할 것"
        )

    results = {}
    for mid in ids:
        start = wrap180(bus.state(mid).position_deg + shift.get(mid, 0.0))
        results[mid] = SweepResult(
            motor_id=mid,
            lo_deg=start,
            hi_deg=start,
            samples=0,
            offset_deg=shift.get(mid, 0.0),
        )

    dt = 1.0 / float(hz)
    while not should_stop():
        bus.refresh_states(ids)
        positions = {}
        for mid in ids:
            state = bus.state(mid)
            if not state.is_valid:
                continue
            entry = results[mid]
            position = wrap180(state.position_deg + entry.offset_deg)
            positions[mid] = position
            results[mid] = SweepResult(
                motor_id=mid,
                lo_deg=min(entry.lo_deg, position),
                hi_deg=max(entry.hi_deg, position),
                samples=entry.samples + 1,
                offset_deg=entry.offset_deg,
            )
        if on_update is not None:
            on_update(results, positions)
        time.sleep(dt)

    logger.info(
        "sweep 끝: %s",
        ", ".join(f"m{r.motor_id} {r.lo_deg:.2f}~{r.hi_deg:.2f}" for r in results.values()),
    )
    return results


def scan(bus: RobStrideBus, *, timeout_s: float = 0.1) -> List[int]:
    """응답하는 모터 id 를 모음. 설정과 실물이 맞는지 보는 첫 확인임.

    응답이 없는 것과 프로토콜이 안 맞는 것은 **여기서 구분되지 않음.** 둘 다 조용히
    빠짐 — 배선, 전원, CAN id, 프로토콜 모드가 전부 후보임.
    """
    found: List[int] = []
    for motor_id in bus.motor_ids:
        bus.bus.flush_rx()
        bus.bus.send_many(
            [_command_frame(motor_id, tables.CMD_STOP, f_cmd=tables.F_CMD_DEFAULT)]
        )
        frames = bus.bus.drain(expect=1, timeout_s=timeout_s)
        if any(f.data and f.data[0] == motor_id for f in frames):
            found.append(motor_id)
    missing = [m for m in bus.motor_ids if m not in found]
    if missing:
        logger.warning("응답 없는 모터: %s", missing)
    return found
