"""정책 실행 — 학습한 모델로 다리를 움직임.

    huphy-run --limb right_leg --policy balance
    huphy-run --robot --policy balance --weights runs/biped.pt

브링업과 **같은 다리·같은 루프**를 씀. 다르게 들어가는 것은 매 주기 관절 목표를
내는 것 하나뿐임 -- 브링업은 사인파 같은 것을 넣고, 여기는 모델을 넣음.


## 정책 이름이 규격을 고름

`.pt` 파일에는 신경망만 들어 있고, 모델 출력을 관절 각도로 바꿀 때 곱하는 값
(`action_scale`)이나 뛰는 위상이 관찰에 붙는지는 안 들어 있음. 그건 학습 설정에만
있어서 `control/policy.py` 에 적어 두었고, 이름으로 고름.

    balance   입력 24, x0.25
    hopping   입력 26, x0.5, 위상 2칸

파일의 입력 개수가 그 값과 다르면 **모터를 켜기 전에** 멈춤.


## 발목 공간은 이름이 아니라 인자로 고름

모델이 발목을 발판 자세로 내는지 모터 각도로 내는지는 규격에 안 넣었음 --
`action_scale` 도 `obs_dim` 도 두 공간이 똑같아서, 넣으면 같은 모델이
`balance_rp`/`balance_ab` 로 둘씩 늘어날 뿐임.

    --ankle-space rp    ankle_pitch, ankle_roll    IK 를 거쳐 모터로
    --ankle-space ab    ankle_a, ankle_b           그대로 모터로

**어긋나도 코드로는 안 잡힘.** 관찰 개수도 행동 개수도 같아서 가중치 검사를
그냥 통과함. 사람이 맞게 골라야 하고, 시작 화면에 어느 쪽인지 찍음.

발목 출력은 기본이 위치임. 토크는 관절 공간 PD 를 야코비안으로 내리는 것이라
`rp` 에서만 쓸 수 있음 -- `ab` 와 같이 주면 시작 전에 멈춤.


## 양다리 — `--robot`

모델이 12칸을 냄. 순서는 학습 쪽과 정한 규격임.

    0-5    left_leg/hip_pitch ... left_leg/ankle_roll
    6-11   right_leg/hip_pitch ... right_leg/ankle_roll

**왼다리가 먼저임.** `robot.yaml` 의 `limbs` 순서(지금 오른다리가 먼저)와
무관함 -- 모델 규격은 설정 파일이 아니라 학습이 정하므로 `policy.BIPED_LEGS` 에
고정해 둠. 이름을 붙인 뒤로는 `Biped` 가 이름을 보고 다리별로 나눔.

관찰 길이는 관절 수에서 계산함 (42, 위상이 있으면 44). `action_scale` 과 위상은
`--policy` 로 고른 규격을 그대로 씀 -- 양다리 모델도 같은 값으로 학습했다는
가정이고, 다르면 규격을 하나 더 두면 됨.

가중치 파일의 입력·출력 개수를 **둘 다** 대조함. 다리 하나 모델에 `--robot` 을
주면 모터를 켜기 전에 멈춤.

IMU 는 다리가 아니라 로봇에 붙음 (`build_biped`). 설정의 `mount` 가 어느
다리든 로봇 전체의 센서로 읽힘.


## 게인이 설정 파일 값이 아님

`robot.yaml` 의 `kp`/`kd` 는 사람이 브링업에서 튜닝하는 값임. 정책은 **학습에 쓴
게인**으로 돌아야 함 -- 시뮬에서 그 값으로 움직이는 것을 보고 배웠기 때문임.

    kp = 20.0,  kd = 0.502     mjlab 의 half_huphy.xml

`--ankle-output torque` 면 발목은 모터가 아니라 **관절**에 이 게인을 걺. 모터 두
개가 로드로 두 축을 같이 만들어서 지렛대 비가 자세마다 달라지므로, 관절 토크를
만들어 야코비안으로 내림.


## 아직 없는 것

    상태 기계        지금은 정책 하나만 돎. 넘어져도 안 멈춤
    토크 가드        발목 토크에는 한계 검사가 안 걸림
    torch 대조       numpy 로 다시 짠 계산을 학습 쪽과 맞춰 보지 않았음

`control/POLICY.md` 에 정리해 둠. **사람이 옆에서 지켜보며 돌릴 것.**
"""

from __future__ import annotations

import argparse
import logging
import signal
import sys
import threading
from pathlib import Path
from typing import Dict, Optional

from ..config import ConfigError, load_robot
from ..control import ControlLoop, Mode, policy, rsl_rl
from ..motors.base import Gains
from .bringup import build_biped, build_leg
from .commission import CONFIG_NAME, _find_config, _pick_limb, all_legs
from .selftest import approach

logger = logging.getLogger("huphy.run")

SPECS = {
    "balance": policy.BALANCE,
    "hopping": policy.HOPPING,
}
"""이름 -> 규격. `control/policy.py` 에 적힌 것을 고름."""

WEIGHTS_DIR = Path("config/policies")
"""이름으로 찾을 때 보는 곳. `--weights` 로 덮어쓸 수 있음."""

POLICY_HZ = 50.0
"""제어 주기. 시뮬이 0.005초 x 4 = 50Hz 로 학습했으므로 같아야 함.

더 빠르게 돌리면 모델이 학습 때보다 자주 불려서, 같은 행동이 더 오래 유지되는 것과
같은 효과가 남 -- 시뮬과 다르게 움직임.
"""

POLICY_KP = 20.0
POLICY_KD = 0.502
"""학습에 쓴 게인. mjlab 의 `half_huphy.xml` 액추에이터 값임.

    <position kp="20.0" kv="0.502" .../>
"""


ZERO_POSE = {joint: 0.0 for joint in policy.JOINT_ORDER}
"""정책이 기대하는 시작 자세. 시뮬의 기준 자세가 관절 전부 0 임.

관찰의 관절 각도가 이 자세 기준의 상대값이라, 여기서 시작하지 않으면 정책이 학습 때
본 적 없는 입력을 받음.
"""


def zero_pose(order) -> Dict[str, float]:
    """그 공간의 영자세. 이름만 다르고 값은 전부 0 임.

    두 공간의 영자세가 **같은 물리 자세**임 -- `solve_ik(0, 0) == (0, 0)` 이라
    발판을 0도로 두는 것과 발목 모터를 0도로 두는 것이 같음. 그래서 접근 단계가
    데려다 놓는 자세도 공간과 무관하게 같음.
    """
    return {name: 0.0 for name in order}


DEFAULT_APPROACH_S = 3.0
"""영점 자세까지 옮기는 데 쓰는 시간.

**서서히 가야 함.** 토크를 넣는 순간 목표가 멀리 있으면 관절이 튐 -- 점프 가드가
자르기는 하지만 그 전에 큰 토크가 한 번 나감.
"""


def staged(approach_motion, approach_s: float, policy_motion, hold_pose=None):
    """영점으로 옮김 -> 그 자세로 기다림 -> Enter 누르면 정책.

    셋을 한 덩어리로 묶어 제어 루프에 넘김. 루프는 이 안에 단계가 있는 줄 모름.

    **정책은 자기 구간의 0초부터** 시간을 받음. 뛰는 위상이 시작한 순간부터 세야
    하기 때문임.

    `start()` 를 부르기 전까지는 영점 자세를 계속 보냄 -- 사람이 로봇에서 손을 떼고
    자리를 잡을 시간임. `hold_pose` 를 주면 그것을 보냄. 발목을 모터 공간으로
    돌릴 때는 기다리는 자세도 그 공간의 이름이어야 함.
    """
    started = [False, 0.0]
    waiting = dict(ZERO_POSE if hold_pose is None else hold_pose)

    def motion(t: float, observation):
        if t < approach_s:
            return approach_motion(t, observation)
        if not started[0]:
            return dict(waiting)
        if started[1] == 0.0:
            started[1] = t
        return policy_motion(t - started[1], observation)

    def start() -> None:
        started[0] = True

    motion.start = start          # type: ignore[attr-defined]
    motion.is_started = lambda: started[0]      # type: ignore[attr-defined]
    return motion


class EnterWatcher:
    """Enter 를 기다렸다가 정책을 시작시킴.

    **별도 스레드에서 봄.** 제어 루프는 주기를 지켜야 해서 입력을 기다릴 틈이 없음.

    화면이 아니면(파이프, 서비스) 바로 시작함 -- 누를 사람이 없는데 영원히 기다리면
    안 됨.
    """

    def __init__(self, motion) -> None:
        self.motion = motion
        self.armed = sys.stdin.isatty()
        self._thread: Optional[threading.Thread] = None

    def __enter__(self) -> "EnterWatcher":
        if not self.armed:
            self.motion.start()
            return self
        self._thread = threading.Thread(target=self._wait, daemon=True)
        self._thread.start()
        return self

    def __exit__(self, *exc) -> None:
        return None

    def _wait(self) -> None:
        try:
            input()
        except (EOFError, KeyboardInterrupt):
            return
        print("  시작함")
        self.motion.start()


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="huphy-run",
        description="학습한 정책으로 다리를 움직임.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "예시:\n"
            "  --limb right_leg --policy balance\n"
            "  --limb right_leg --policy hopping --weights runs/model_49999.pt\n"
            "  --limb right_leg --policy balance --ankle-output torque\n"
            "  --limb right_leg --policy balance --ankle-space ab\n"
            "  --robot --policy balance --weights runs/biped.pt   양다리 12칸\n\n"
            "상태 기계와 토크 가드가 아직 없음. 사람이 지켜보며 돌릴 것.\n"
        ),
    )
    p.add_argument("--config", type=Path, help=f"기본값: 위로 올라가며 {CONFIG_NAME} 을 찾음")
    p.add_argument("--limb", help="팔다리 이름. 하나뿐이면 생략 가능")
    p.add_argument(
        "--robot", action="store_true",
        help="양다리 모델을 로봇 전체로 돌림. 12칸, 왼다리 먼저",
    )
    p.add_argument(
        "--policy", required=True, choices=sorted(SPECS),
        help="어느 정책인지. 관찰 개수와 action_scale 이 여기서 정해짐",
    )
    p.add_argument(
        "--weights", type=Path,
        help=f"가중치 파일. 기본값은 {WEIGHTS_DIR}/<정책이름>.pt",
    )
    p.add_argument(
        "--ankle-space", choices=sorted(policy.ORDERS), default="rp",
        help="모델이 발목을 무엇으로 내는지. rp=발판 자세, ab=모터 각도",
    )
    p.add_argument(
        "--ankle-output", choices=("position", "torque"), default="position",
        help="발목 두 모터에 각도를 보낼지 토크를 보낼지. torque 는 rp 에서만",
    )
    p.add_argument(
        "--hz", type=float, default=POLICY_HZ,
        help=f"제어 주기. 기본 {POLICY_HZ:.0f} (학습 주기와 같아야 함)",
    )
    p.add_argument(
        "--approach", type=float, default=DEFAULT_APPROACH_S,
        help=f"영점 자세까지 옮기는 시간. 기본 {DEFAULT_APPROACH_S:.0f}초",
    )
    p.add_argument(
        "--duration", type=float,
        help="이 초만큼만 돌고 나옴. 생략하면 Ctrl-C 까지",
    )
    p.add_argument(
        "--allow-uncalibrated", action="store_true",
        help="실측 전에도 토크를 넣음",
    )
    p.add_argument("-v", "--verbose", action="store_true")
    return p


def _weights_path(args) -> Path:
    """쓸 가중치 파일. 없으면 멈춤."""
    path = args.weights if args.weights else WEIGHTS_DIR / f"{args.policy}.pt"
    if not path.is_file():
        raise SystemExit(
            f"가중치 파일이 없음: {path}\n"
            f"--weights 로 경로를 지정하거나 {WEIGHTS_DIR} 에 넣을 것"
        )
    return path


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

    if args.robot and args.limb:
        raise SystemExit(
            "--robot 과 --limb 을 같이 줄 수 없음. --robot 은 다리 전부이고 "
            "--limb 은 그중 하나를 고르는 것임"
        )
    spec = SPECS[args.policy]

    # 토크 경로는 관절 공간 PD 라 pitch/roll 이 있어야 함. 여기서 막지 않으면
    # Leg 이 첫 주기에 거부하는데, 그때는 이미 토크가 들어간 뒤임.
    if args.ankle_space == "ab" and args.ankle_output == "torque":
        raise SystemExit(
            "--ankle-space ab 는 --ankle-output torque 와 같이 쓸 수 없음. "
            "모터 공간 모델은 발목 각도를 직접 내므로 position 으로 돌릴 것"
        )

    # 양다리면 12칸 순서를 쓰고 관찰 길이를 다시 계산함. 순서는 학습 쪽과 정한
    # 규격(왼다리 먼저)이지 robot.yaml 의 순서가 아님 -- policy.BIPED_LEGS.
    if args.robot:
        limbs = all_legs(robot)
        names = {limb.name for limb in limbs}
        missing = [leg for leg in policy.BIPED_LEGS if leg not in names]
        if missing:
            raise SystemExit(
                f"양다리 모델이 기대하는 다리가 설정에 없음: {missing} "
                f"(있는 것: {sorted(names)}). robot.yaml 의 limbs 이름이 "
                f"{list(policy.BIPED_LEGS)} 여야 함"
            )
        order = policy.BIPED_ORDERS[args.ankle_space]
        spec = policy.for_joints(spec, len(order))
    else:
        limbs = [_pick_limb(robot, args.limb)]
        order = policy.ORDERS[args.ankle_space]

    # 모터를 켜기 전에 읽음. 규격이 어긋나면 여기서 멈추는 것이 안전함. 출력
    # 개수도 대조함 -- 다리 하나 모델에 --robot 을 주면 여기서 걸림.
    weights = _weights_path(args)
    try:
        model = rsl_rl.load(weights, spec=spec, action_dim=len(order))
    except (ValueError, OSError) as e:
        raise SystemExit(f"{e}") from e

    options = dict(
        allow_uncalibrated=args.allow_uncalibrated,
        gains=Gains(kp=POLICY_KP, kd=POLICY_KD),
        ankle_output=args.ankle_output,
        ankle_kp=(POLICY_KP, POLICY_KP),
        ankle_kd=(POLICY_KD, POLICY_KD),
    )
    if args.robot:
        leg = build_biped(robot, limbs, **options)
    else:
        leg = build_leg(robot, limbs[0], **options)

    # 다리 하나면 그 다리에 붙은 IMU, 양다리면 로봇에 붙은 IMU 임
    # (build_biped 가 센서를 다리가 아니라 로봇에 붙임). 둘 다 .imus 로 꺼냄.
    if not leg.imus:
        where = "로봇" if args.robot else limbs[0].name
        hint = (
            "robot.yaml 의 imus 에 적을 것"
            if args.robot
            else f"robot.yaml 의 imus 에 mount: {limbs[0].name} 로 적을 것"
        )
        raise SystemExit(
            f"{where} 에 IMU 가 없음. 정책 입력의 6칸이 IMU 값임 "
            f"(각속도 3, 중력 방향 3).\n{hint}"
        )

    channels = [limb.channel for limb in limbs]
    try:
        leg.connect()
    except ImportError as e:
        raise SystemExit(f"{e}") from e
    except ConnectionError as e:
        raise SystemExit(
            f"{e}\n채널이 올라와 있는지 확인할 것:\n"
            + "\n".join(
                f"  sudo ip link set {c} up type can bitrate 1000000" for c in channels
            )
        ) from e

    loop = ControlLoop(leg, hz=args.hz, mode=Mode.CONTROL)
    signal.signal(signal.SIGINT, lambda *_: loop.stop())

    print(
        f"\n  {leg.id}  {' '.join(channels)}  {args.hz:.0f}Hz\n"
        f"  정책     {args.policy}  (입력 {spec.obs_dim}, x{spec.action_scale})\n"
        f"  가중치   {weights}\n"
        f"  게인     kp={POLICY_KP} kd={POLICY_KD}\n"
        f"  발목     {args.ankle_space} ({order[-2]}, {order[-1]})"
        f"  {args.ankle_output}\n"
        f"  IMU      {', '.join(i.name for i in leg.imus)}\n\n"
        f"  {args.approach:.0f}초에 걸쳐 영점 자세로 옮긴 뒤 그 자세로 기다립니다.\n"
        f"  Enter 를 누르면 정책이 시작됩니다.\n\n"
        f"  ** 상태 기계와 토크 가드가 없음. 넘어져도 멈추지 않음 **\n"
        f"  Ctrl-C 로 멈춤. 멈출 때 자세를 붙잡은 뒤 토크를 끊음.\n"
    )

    target_pose = zero_pose(order)
    start_pose = {
        joint: float(leg.get_observation().get(f"{joint}.pos", 0.0))
        for joint in order
    }
    motion = staged(
        approach(target_pose, start_pose, args.approach),
        args.approach,
        policy.policy_motion(model, leg.imus[0], spec=spec, order=order),
        hold_pose=target_pose,
    )

    try:
        with EnterWatcher(motion):
            stats = loop.run(motion, duration_s=args.duration)
        print(f"\n  {stats.summary()}")
        if not stats.kept_up:
            print("  주기를 못 지킴. 정책이 시뮬과 다르게 움직임")
            return 1
    finally:
        leg.disconnect()
        print("\n  종료. 토크가 끊겼음")
    return 0


if __name__ == "__main__":  # pragma: no cover
    sys.exit(main())
