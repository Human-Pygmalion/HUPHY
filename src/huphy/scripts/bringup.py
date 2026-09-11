"""브링업 — 다리를 실제로 움직여 보는 대화형 메뉴.

    python -m huphy.scripts.bringup --limb right_leg

설정·실측값·버스·기구학·안전·텔레메트리·제어 루프를 한데 묶어 씀. **모든 계층이
여기서 만나는 유일한 곳임.**


## 움직이는 것은 전부 제어 루프를 탐

메뉴가 로봇을 직접 부르지 않음. **동작(`Motion`)만 정하고 루프에 넘김.**

    menu -> motion -> ControlLoop -> Leg -> bus

직접 부르면 그 경로에서만 텔레메트리·주기 측정·정지 순서가 빠짐 (이슈 #4).
그러면 그래프가 안 나오는데 텔레메트리가 고장난 줄 알게 되고, 같은 일을 하는 코드가
두 벌이 되어 한쪽만 고쳐짐.

**게인을 튜닝하려면 손으로 움직이면서 그래프를 봐야 함.** 그래서 메뉴가 루프를
타야 함.


## 여기 없는 것

되돌리기 어려운 조작(영점, CAN id, 프로토콜)은 `commission.py` 에 있음. 반복하지
않는 조작이라 주기가 없고, 제어 경로에서 손 닿는 곳에 두지 않으려는 것임.


## 안전

    시작할 때    관찰 모드. 토크가 꺼져 있음
    움직이기 전  freeze 로 지금 자세를 잡고 시작 -- 토크를 넣는 순간 튀지 않게
    끝날 때      hold 후 토크 차단
    Ctrl-C       루프의 finally 를 지나므로 같은 순서를 탐
"""

from __future__ import annotations

import logging
import signal
import sys
from dataclasses import replace
from pathlib import Path
from typing import Callable, Dict, List, Optional, Sequence, Tuple

from .. import calibration as calib
from .. import telemetry as tele
from ..config import ConfigError, LimbConfig, load_robot
from ..config.schema import RobotConfig
from ..control import ControlLoop, Mode, motions
from ..kinematics.ankle import AnkleGeometry, AnkleKinematics
from ..motors.base import Gains
from ..motors.canbus import CanBus
from ..motors.robstride.bus import RobStrideBus
from ..robots.biped import Biped
from ..robots.leg import ANKLE_JOINTS, SINGLE_JOINTS, Leg
from ..sensors import make_imus
from . import table
from .commission import CONFIG_NAME, _find_config, _pick_limb

logger = logging.getLogger(__name__)

DEFAULT_RUN_S = 3.0
NUDGE_DEG = 5.0
"""메뉴에서 한 번에 움직이는 기본 각도. 확인용이라 작게 둠."""


# ===========================================================================
# 조립
# ===========================================================================
def build_leg(
    robot: RobotConfig,
    limb: LimbConfig,
    *,
    gain_scale: float = 1.0,
    allow_uncalibrated: bool = False,
    gains: Optional[Gains] = None,
    ankle_output: str = "position",
    ankle_kp: Tuple[float, float] = (0.0, 0.0),
    ankle_kd: Tuple[float, float] = (0.0, 0.0),
    reader: bool = False,
    imus: Optional[List[object]] = None,
    drain_s: Optional[float] = None,
) -> Leg:
    """설정에서 다리 하나를 만듦.

    `gain_scale` 로 게인을 낮춰 시작할 수 있음. 튜닝값을 찾은 뒤에도 처음엔 낮게
    시작하는 것이 안전함.

    `gains` 를 주면 **설정의 게인 대신 그 값을 모든 모터에 씀.** 정책을 돌릴 때
    학습에 쓴 게인을 그대로 써야 하는데, 그 값은 `robot.yaml` 이 아니라 학습 설정에
    있기 때문임.

    `ankle_output` 이 `"torque"` 면 발목 두 모터에 각도 대신 토크가 나감. 시뮬은
    발목 두 축이 독립 관절이고 각자 서보가 붙어 있는데, 실물은 모터 두 개가 로드로
    둘을 같이 만들어서 관절 게인을 모터에 그대로 걸 수 없음.

    발목 기구학은 **왼쪽이면 거울상**을 씀. 같은 관절 명령에 양다리가 같은 물리
    동작을 하려면 필요함 (이슈 #13 -- 거울상은 실측이 아니라 가정임).

    `reader` 를 켜면 이 다리의 CAN 채널에 수신 스레드가 붙음. 버스가 하나면 얻는
    것이 없어 기본은 꺼져 있고, 양다리를 묶을 때 켬.

    `imus` 를 주면 그것을 씀. 빈 목록을 주면 **센서를 안 붙임** -- 로봇 전체가
    들고 있을 때임 (`build_biped`).

    `drain_s` 는 응답을 기다리는 최대 시간임. 안 주면 버스 기본값(2ms). 양다리는
    다리마다 따로 기다리므로 두 다리 다 응답이 늦으면 최악 두 배가 됨.
    """
    if gains is not None:
        limb = replace(
            limb,
            motors={
                name: replace(motor, gains=gains)
                for name, motor in limb.motors.items()
            },
        )

    if gain_scale != 1.0:
        limb = replace(
            limb,
            motors={
                name: replace(motor, gains=motor.gains.scaled(gain_scale))
                for name, motor in limb.motors.items()
            },
        )

    geometry = AnkleGeometry()
    if limb.side == "left":
        geometry = geometry.mirrored()

    bus_options = {} if drain_s is None else {"drain_s": drain_s}
    bus = RobStrideBus(
        CanBus(limb.channel, interface=limb.interface, reader=reader),
        limb.motors_by_id(),
        **bus_options,
    )
    return Leg(
        limb,
        bus,
        safety=robot.safety,
        kinematics=AnkleKinematics(geometry),
        allow_uncalibrated=allow_uncalibrated,
        imus=_imus_on(robot, limb) if imus is None else imus,
        ankle_output=ankle_output,
        ankle_kp=ankle_kp,
        ankle_kd=ankle_kd,
    )


def build_biped(
    robot: RobotConfig,
    limbs: Optional[Sequence[LimbConfig]] = None,
    **leg_options,
) -> Biped:
    """설정에서 **로봇 전체**를 만듦. 팔다리를 다 세우고 하나로 묶음.

    `limbs` 를 생략하면 `kind: leg` 인 팔다리를 전부 씀. 순서는 설정에 적힌
    순서임 -- 관절 이름과 텔레메트리 열 순서가 그 순서를 따름.

    `build_leg` 과 다른 점 셋.

        수신 스레드    켬. 버스가 둘이면 순차 수거의 총 대기가 두 배가 됨
        IMU           다리가 아니라 로봇이 들고 있음
        안전 설정      로봇 것을 Biped 도 같이 들고 있음

    IMU 를 다리에서 떼는 이유: 몸통 센서는 어느 다리의 것도 아니고, 다리 센서를
    몸통으로 옮길 때 소유자가 바뀌면 안 됨. 값을 찾는 쪽은 개체 이름만 알면 되므로
    (`imu/main/gx`) 어디에 달렸는지는 `mount` 메모로만 남음.

    나머지 인자는 `build_leg` 에 그대로 넘어감 -- 게인 스케일 등이 양다리에 같이
    걸림.
    """
    chosen = list(limbs) if limbs is not None else list(robot.limbs_of_kind("leg").values())
    if not chosen:
        raise ConfigError(
            f"kind: leg 인 팔다리가 없음 (있는 것: {sorted(robot.limbs)}). "
            f"robot.yaml 의 limbs 를 확인할 것"
        )

    leg_options.setdefault("reader", True)
    legs = [build_leg(robot, limb, imus=[], **leg_options) for limb in chosen]
    return Biped(
        legs,
        id=robot.name,
        safety=robot.safety,
        imus=make_imus(robot.imus.values()),
    )


def _imus_on(robot: RobotConfig, limb: LimbConfig) -> List[object]:
    """이 팔다리에 붙은 IMU 만 만듦. 없으면 빈 목록.

    어디에 붙었는지는 설정의 `mount` 가 정함. 몸통으로 옮기면 여기서 안 걸리고,
    다리 코드는 손댈 일이 없음.

    **다리 하나만 돌릴 때 쓰는 길임.** 로봇 전체를 조립할 때는 센서가 다리가 아니라
    로봇에 달림 (`sensors/group.py` 의 `ImuGroup`).
    """
    return list(make_imus(robot.imus_on(limb.name).values()))


# ===========================================================================
# 화면
# ===========================================================================
def _ack_text(ack: float) -> str:
    """직전 주기의 응답 여부. 숫자 세 값을 사람이 읽는 말로.

    `-1`(명령하지 않음)과 `0`(명령했는데 응답 없음)은 다른 상태임 — 앞은 정상이고
    뒤는 씹힌 것임.
    """
    if ack > 0:
        return "받음"
    if ack == 0:
        return "씹힘"
    return "명령안함"


def _age_text(age_ms: float) -> str:
    """마지막 응답 이후 경과. 1초가 넘으면 초로 바꿔 씀.

    100Hz 면 정상값이 한 자리 ms 라 ms 로 봐야 하고, 끊긴 모터는 수십만 ms 가 나와
    ms 로는 몇 분인지 읽히지 않음.
    """
    if age_ms < 0:
        return "없음"
    if age_ms >= 1000.0:
        return f"{age_ms / 1000.0:.1f}초"
    return f"{age_ms:.2f}ms"


def show_state(leg: Leg, loop: ControlLoop) -> None:
    """지금 상태를 표로. **읽기만 함** -- 루프가 이미 수거해 둔 값을 씀."""
    loop.step(None, t=0.0)          # 한 주기 읽어 옴
    observation = leg.get_observation()
    link = leg.link_status()

    print(f"\n  {leg.id}  {leg.config.channel}\n")
    print(
        "  "
        + table.header(
            ("관절", 10, "<"), ("raw", 9), ("cal", 9), ("속도", 9), ("토크", 8),
            ("온도", 6), ("응답", 8), ("마지막 응답", 12),
        )
    )
    for name in leg.motor_names:
        raw = leg.bus.state(leg.config.motors[name].id).position_deg
        status = link.get(name, {})
        print(
            f"  {name:<10} {raw:9.2f} {observation[f'{name}.pos']:9.2f} "
            f"{observation[f'{name}.vel']:9.2f} {observation[f'{name}.torque']:8.2f} "
            f"{observation[f'{name}.temp']:6.1f} "
            f"{table.cell(_ack_text(status.get('ack', -1)), 8)} "
            f"{table.cell(_age_text(status.get('age', -1)), 12)}"
        )

    pose = leg.ankle_pose()
    if pose is not None:
        print(f"\n  발목  pitch {pose[0]:7.2f}   roll {pose[1]:7.2f}")

    show_imus(leg)

    outside = _outside_limits(leg, observation)
    if outside:
        print(
            f"\n  ** 한계 밖에 있는 관절 **\n"
            f"  토크를 넣으면 가드가 한계 안으로 끌어당김 -- 그 방향으로 움직임."
        )
        for name, position, limits in outside:
            print(f"    {name:<10} {position:8.2f}  한계 {limits[0]:.2f} ~ {limits[1]:.2f}")

    unmeasured = calib.unmeasured(leg.calibration)
    if unmeasured:
        print(f"\n  미실측: {list(unmeasured)}  (cal 이 raw 와 같음)")
    if not leg.config.is_configured:
        print(f"  게인·한계 미설정: {list(leg.config.unconfigured())}")


def show_imus(leg: Leg) -> None:
    """붙은 IMU 의 자세와 마지막 패킷 이후 경과. 없으면 아무것도 안 냄.

    **`마지막 값` 이 진짜 신호임.** 센서가 멈춰도 자세는 그럴듯한 숫자로 남아
    있어서, 값만 보면 살아 있는지 알 수 없음.

    자세를 도로 보여줌. 센서가 쿼터니언을 주더라도 벤더 모듈이 `extra` 에 오일러를
    같이 넣으므로 여기서 계산하지 않음 -- 없으면 그 칸만 비움.
    """
    if not leg.imus:
        return

    print("\n  " + table.header(
        ("IMU", 10, "<"), ("roll", 9), ("pitch", 9), ("yaw", 9),
        ("중력", 22), ("마지막 값", 12),
    ))
    for name, state in leg.imu_states().items():
        if not state.is_valid:
            print(f"  {name:<10} {table.cell('응답 없음', 9)}")
            continue
        angles = "".join(
            f"{state.extra[axis]:9.2f}" if axis in state.extra else table.cell("-", 9)
            for axis in ("roll", "pitch", "yaw")
        )
        gravity = "({:6.3f},{:6.3f},{:6.3f})".format(*state.gravity)
        print(
            f"  {name:<10}{angles} {table.cell(gravity, 22)}"
            f"{table.cell(_age_text(state.age_ms()), 12)}"
        )


def _outside_limits(leg: Leg, observation) -> List[Tuple[str, float, Tuple[float, float]]]:
    """한계 밖에 있는 모터들.

    토크를 넣으면 가드가 한계 안으로 끌어당기므로 **그 방향으로 움직임.** 사람이
    알고 있어야 함 -- 다리를 손으로 옮겨 놓거나, 한계값이 실물과 다른 것임.
    """
    out = []
    for name, motor in leg.config.motors.items():
        if motor.limits_deg is None:
            continue
        position = observation.get(f"{name}.pos")
        if position is None:
            continue
        lo, hi = motor.limits_deg
        if not lo <= position <= hi:
            out.append((name, float(position), motor.limits_deg))
    return out


def show_counters(leg: Leg, loop: ControlLoop) -> None:
    print("\n  가드")
    for key, value in leg.counters.as_fields().items():
        print(f"    {key:<20} {value}")
    print("\n  CAN")
    for key, value in leg.bus.bus.counters.as_fields().items():
        print(f"    {key:<20} {value}")
    print(f"\n  마지막 클리핑 이후 {leg.since_clip():.1f}초   "
          f"마지막 거부 이후 {leg.since_reject():.1f}초")
    if loop.stats.cycles:
        print(f"\n  루프  {loop.stats.summary()}")


def _ask(prompt: str, default: float) -> Optional[float]:
    raw = input(f"  {prompt} [{default}]: ").strip()
    if not raw:
        return default
    try:
        return float(raw)
    except ValueError:
        print("  숫자가 아님")
        return None


def _ask_joint(leg: Leg) -> Optional[str]:
    print(f"  관절: {', '.join(leg.joint_names)}")
    name = input("  이름: ").strip()
    if name not in leg.joint_names:
        print(f"  {name!r} 는 없음")
        return None
    return name


# ===========================================================================
# 움직이는 항목 — 전부 루프를 탐
# ===========================================================================
def _run(loop: ControlLoop, motion, seconds: float) -> None:
    """제어 모드로 잠깐 돌림. 끝나면 관찰 모드로 되돌림.

    메뉴 항목이 로봇을 직접 부르지 않고 여기를 지나는 이유: 텔레메트리·주기 측정·
    정지 순서가 한 곳에만 있게 하려는 것임 (이슈 #4).
    """
    previous = loop.mode
    loop.mode = Mode.CONTROL
    try:
        stats = loop.run(motion, duration_s=seconds)
        print(f"\n  {stats.summary()}")
    finally:
        loop.mode = previous


def move_joint(leg: Leg, loop: ControlLoop) -> None:
    """한 관절을 지금 자리에서 조금 옮김."""
    joint = _ask_joint(leg)
    if joint is None:
        return
    delta = _ask("얼마나 (도)", NUDGE_DEG)
    if delta is None:
        return

    observation = leg.get_observation()
    start = _current_joint_value(leg, joint, observation)
    if start is None:
        print("  지금 자세를 알 수 없음. 먼저 상태를 읽을 것")
        return

    print(f"\n  {joint}  {start:.2f} -> {start + delta:.2f} 도")
    _run(
        loop,
        motions.chain(
            (motions.freeze(list(leg.joint_names)), 0.3),
            (motions.ramp(joint, start=start, end=start + delta, seconds=1.0,
                          hold_others=_others(leg, joint, observation)), 2.0),
        ),
        seconds=2.5,
    )


def step_response(leg: Leg, loop: ControlLoop) -> None:
    """계단 응답. **게인 튜닝에서 가장 많은 것을 알려줌.**"""
    joint = _ask_joint(leg)
    if joint is None:
        return
    delta = _ask("계단 크기 (도)", 10.0)
    if delta is None:
        return

    observation = leg.get_observation()
    start = _current_joint_value(leg, joint, observation)
    if start is None:
        print("  지금 자세를 알 수 없음")
        return

    print(
        f"\n  {joint}  {start:.2f} -> {start + delta:.2f} 도\n"
        f"  그래프에서 볼 것:\n"
        f"    못 미침    kp 부족      지나쳤다 돌아옴  kd 부족\n"
        f"    떨림       kp 과함      느리게 도달      kp 올릴 여지"
    )
    _run(
        loop,
        motions.chain(
            (motions.freeze(list(leg.joint_names)), 0.5),
            (motions.step(joint, start=start, end=start + delta, at_s=0.5,
                          hold_others=_others(leg, joint, observation)), 3.0),
        ),
        seconds=3.5,
    )


def sine_sweep(leg: Leg, loop: ControlLoop) -> None:
    """사인파 왕복. 추종 지연과 진폭 감쇠를 봄."""
    joint = _ask_joint(leg)
    if joint is None:
        return
    amplitude = _ask("진폭 (도)", 5.0)
    hz = _ask("주파수 (Hz)", 0.5)
    seconds = _ask("길이 (초)", DEFAULT_RUN_S)
    if None in (amplitude, hz, seconds):
        return

    observation = leg.get_observation()
    center = _current_joint_value(leg, joint, observation)
    if center is None:
        print("  지금 자세를 알 수 없음")
        return

    print(f"\n  {joint}  {center:.2f} 도를 중심으로 ±{amplitude} 도, {hz} Hz")
    _run(
        loop,
        motions.chain(
            (motions.freeze(list(leg.joint_names)), 0.3),
            (motions.sine(joint, center=center, amplitude=amplitude, hz=hz,
                          hold_others=_others(leg, joint, observation)), seconds),
        ),
        seconds=seconds + 0.5,
    )


def hold_pose(leg: Leg, loop: ControlLoop) -> None:
    """지금 자세를 붙잡고 있음. **게인 튜닝의 출발점.**

    여기서 처지면 `kp` 가 부족하고, 떨리면 `kp` 가 과함. 여기서 떨리면 어떤 동작을
    시켜도 떨림.

    **발목은 여기서 FK 로 채움.** 관찰은 모터 단위(`ankle_a`, `ankle_b`)로만
    나오고 액션은 관절 단위(`ankle_pitch`, `ankle_roll`)로 받으므로, 지금 자세를
    목표로 삼으려면 한 번 풀어야 함. FK 는 뉴턴 반복이라 여기서 한 번만 부름 --
    IK 는 닫힌 해라 매 주기 돌아도 됨.
    """
    seconds = _ask("얼마나 (초)", DEFAULT_RUN_S)
    if seconds is None:
        return

    observation = leg.get_observation()
    targets = {
        joint: value
        for joint in SINGLE_JOINTS
        if (value := observation.get(f"{joint}.pos")) is not None
    }
    pose = leg.ankle_pose()
    if pose is not None:
        targets["ankle_pitch"], targets["ankle_roll"] = pose

    missing = [j for j in leg.joint_names if j not in targets]
    if not targets:
        print("\n  지금 자세를 알 수 없음. 먼저 상태를 읽을 것")
        return
    if missing:
        # 토크는 넣되 무엇이 빠졌는지 밝힘. 빠진 관절은 명령을 안 받으므로 모터
        # 내부 목표를 붙잡음 -- 가만히 있는 것이 아니라 그쪽으로 움직임.
        print(f"\n  ** 목표를 못 정한 관절 {missing} -- 이 관절은 붙잡지 못함 **")

    print("\n  지금 자세를 붙잡음. 처지나, 떨리나 볼 것")
    _run(loop, motions.hold(targets), seconds=seconds)


def _current_joint_value(leg: Leg, joint: str, observation) -> Optional[float]:
    """관절의 지금 각도. 발목은 FK 를 거침."""
    if joint in SINGLE_JOINTS:
        value = observation.get(f"{joint}.pos")
        return None if value is None else float(value)
    pose = leg.ankle_pose()
    if pose is None:
        return None
    return pose[0] if joint == "ankle_pitch" else pose[1]


def _others(leg: Leg, joint: str, observation) -> Dict[str, float]:
    """흔들지 않는 관절을 지금 자리에 붙잡아 둠.

    여럿을 같이 흔들면 **어느 관절이 원인인지 섞임.**
    """
    out: Dict[str, float] = {}
    for other in SINGLE_JOINTS:
        if other == joint:
            continue
        value = observation.get(f"{other}.pos")
        if value is not None:
            out[other] = float(value)

    if joint not in ANKLE_JOINTS:
        pose = leg.ankle_pose()
        if pose is not None:
            out["ankle_pitch"], out["ankle_roll"] = pose
    else:
        pose = leg.ankle_pose()
        if pose is not None:
            other = "ankle_roll" if joint == "ankle_pitch" else "ankle_pitch"
            out[other] = pose[1] if other == "ankle_roll" else pose[0]
    return out


# ===========================================================================
# 메뉴
# ===========================================================================
MenuItem = Tuple[str, Callable[[Leg, ControlLoop], None], bool]
"""(설명, 함수, 토크가 필요한가)"""

MENU: List[MenuItem] = [
    ("상태 보기", show_state, False),
    ("카운터 보기", show_counters, False),
    ("자세 유지", hold_pose, True),
    ("한 관절 옮기기", move_joint, True),
    ("계단 응답", step_response, True),
    ("사인파 왕복", sine_sweep, True),
]


def print_menu(leg: Leg, loop: ControlLoop) -> None:
    print(f"\n{'=' * 60}")
    print(f"  {leg.id}  {leg.config.channel}  {loop.hz:.0f}Hz")
    ready = "준비됨" if leg.is_calibrated else "미실측 (allow_uncalibrated 필요)"
    print(f"  캘리브레이션: {ready}")
    print(f"{'=' * 60}")
    for index, (label, _, needs_torque) in enumerate(MENU, start=1):
        mark = " [토크]" if needs_torque else ""
        print(f"  {index}. {label}{mark}")
    print("  q. 나가기")


def run_menu(leg: Leg, loop: ControlLoop) -> None:
    while True:
        print_menu(leg, loop)
        choice = input("\n선택: ").strip().lower()
        if choice in ("q", "quit", "exit", ""):
            return
        if not choice.isdigit() or not 1 <= int(choice) <= len(MENU):
            print("  없는 항목")
            continue

        label, handler, needs_torque = MENU[int(choice) - 1]
        if needs_torque and not leg.is_calibrated and not leg.allow_uncalibrated:
            # 두 가지를 다 냄. 판정이 둘의 곱이라 한쪽만 내면 빈 목록을 보여주며
            # 거부하는 일이 생김 -- 사람이 무엇을 채워야 할지 알 수 없음.
            print(
                f"\n  {label} 은 토크가 필요한데 실측값이 채워지지 않음.\n"
                f"  미실측 관절 (zero_reference 없음) "
                f"{list(calib.unmeasured(leg.calibration))}\n"
                f"  미설정 관절 (limits_deg 나 kp 없음) "
                f"{list(leg.config.unconfigured())}\n"
                f"  확인했으면 --allow-uncalibrated 로 다시 실행할 것"
            )
            continue
        try:
            handler(leg, loop)
        except KeyboardInterrupt:
            # 루프의 finally 를 이미 지나 토크가 끊긴 상태임.
            print("\n  중단됨")
        except Exception as e:
            logger.exception("%s 실패", label)
            print(f"\n  실패: {e}")


# ===========================================================================
# 진입점
# ===========================================================================
def build_parser():
    import argparse

    p = argparse.ArgumentParser(
        prog="python -m huphy.scripts.bringup",
        description="다리를 실제로 움직여 보는 대화형 메뉴.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "움직이는 항목은 전부 제어 루프를 탐 -- 텔레메트리와 주기 측정이 같이 감.\n"
            "되돌리기 어려운 조작(영점, CAN id, 프로토콜)은 commission 에 있음.\n"
        ),
    )
    p.add_argument("--config", type=Path, help=f"기본값: 위로 올라가며 {CONFIG_NAME} 을 찾음")
    p.add_argument("--limb", help="팔다리 이름. 하나뿐이면 생략 가능")
    p.add_argument("--hz", type=float, help="제어 주기. 기본값은 설정의 control_hz")
    p.add_argument(
        "--gain-scale", type=float, default=1.0,
        help="게인을 이 비율로 낮춰 시작함. 브링업 초반에 0.1 등으로 씀",
    )
    p.add_argument(
        "--allow-uncalibrated", action="store_true",
        help="실측 전에도 토크를 넣음. 실측을 하려면 움직여야 하므로 필요함",
    )
    p.add_argument("--no-precise", action="store_true", help="마감 직전 스핀을 끔")
    p.add_argument("-v", "--verbose", action="store_true")
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(levelname)s %(message)s",
    )

    path = args.config or _find_config()
    if path is None:
        raise SystemExit(
            f"{CONFIG_NAME} 을 찾지 못했음. 저장소 안에서 실행하거나 --config 로 지정할 것"
        )
    try:
        robot = load_robot(path)
    except ConfigError as e:
        raise SystemExit(f"{e}") from e

    limb = _pick_limb(robot, args.limb)
    leg = build_leg(
        robot, limb,
        gain_scale=args.gain_scale,
        allow_uncalibrated=args.allow_uncalibrated,
    )

    try:
        leg.connect()
    except ImportError as e:
        raise SystemExit(f"{e}") from e
    except ConnectionError as e:
        raise SystemExit(
            f"{e}\n채널이 올라와 있는지 확인할 것:\n"
            f"  sudo ip link set {limb.channel} up type can bitrate 1000000"
        ) from e

    telemetry = tele.Telemetry.from_config(leg, robot.telemetry)
    loop = ControlLoop(
        leg,
        hz=args.hz or limb.control_hz,
        telemetry=telemetry,
        mode=Mode.OBSERVE,
        precise=not args.no_precise,
    )

    # Ctrl-C 가 루프 안에서 나면 루프의 finally 가 정리함. 메뉴에서 나면 여기가 함.
    signal.signal(signal.SIGINT, lambda *_: loop.stop())

    if args.gain_scale != 1.0:
        print(f"\n  게인을 {args.gain_scale}배로 낮춰 시작함")
    if telemetry.enabled:
        print(f"  텔레메트리: {telemetry}")

    try:
        run_menu(leg, loop)
    finally:
        leg.disconnect()
        telemetry.close()
        print("\n  종료. 토크가 끊겼음")
    return 0


if __name__ == "__main__":
    sys.exit(main())
