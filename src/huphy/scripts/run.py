"""정책 실행 — 학습한 걷기 정책으로 양다리를 움직임.

    huphy-run --vx 0.5 --wz 0.0
    huphy-run --vx 1.0 --weights config/policies/other_checkpoint.pt

브링업과 **같은 다리·같은 루프**를 씀. 다르게 들어가는 것은 매 주기 관절 목표를
내는 것 하나뿐임 -- 브링업은 사인파 같은 것을 넣고, 여기는 모델을 넣음.

**정책은 `generalized_walk` 하나뿐임.** 과거에 `balance`/`hopping`/`stand`가
있었는데 더 이상 돌리지 않아서 지웠음 -- `control/generalized_walk.py`가 지금부터의
정식 입출력 규격임(44칸 입력, 12관절 출력, 양다리 전용).


## 명령(vx, wz)을 실행할 때 고정값으로 줌

정책 관찰의 마지막 두 칸이 속도 명령임. `--vx`/`--wz`로 주면 그 값이 **실행
내내 고정**임 -- 매 주기 새로 읽지 않음. 걷는 동안 속도를 바꾸고 싶으면 멈추고
다른 값으로 다시 실행할 것.


## 게인은 `robot.yaml`을 그대로 씀

정책은 **학습에 쓴 게인**으로 돌아야 함. `robot.yaml`(`robot_v1.0.yaml`)이 곧
이 정책이 학습에서 가정한 사양값이라 덮어쓰지 않음 -- 관절 묶음마다 다름(150/150,
75, 130, 60 같은 식이 아니라 지금은 135/75/130/60, `config/robot.yaml` 참조).


## 아직 없는 것

    상태 기계        지금은 정책 하나만 돎. 넘어져도 안 멈춤
    토크 가드        한계 검사가 발목 토크 경로엔 안 걸림(이 정책은 위치 출력만 써서 해당 없음)
    torch 대조       numpy 로 다시 짠 계산을 학습 쪽과 맞춰 보지 않았음

**사람이 옆에서 지켜보며 돌릴 것.**
"""

from __future__ import annotations

import argparse
import logging
import signal
import sys
from pathlib import Path

from ..config import ConfigError, load_robot
from ..control import ControlLoop, Mode, generalized_walk, held, rsl_rl
from ..motors.canbus import DEFAULT_DRAIN_S, bring_up_hint
from ..robots.leg import SEND_MODES, SEND_SEQ
from ..telemetry import latency_log, live, policy_log
from . import failures
from .bringup import build_biped
from .commission import CONFIG_NAME, _find_config, all_legs
from .selftest import QuitWatcher, approach

logger = logging.getLogger("huphy.run")

POLICY_NAME = "generalized_walk"
"""로그 파일 이름·화면 표시에 씀. 정책이 하나뿐이라 선택지가 아니라 상수임."""

SPEC = generalized_walk.SPEC
ORDER = generalized_walk.ORDER

ANKLE_MOTORS = ("ankle_a", "ankle_b")
"""`--gain-scale` 의 두 번째 숫자가 걸리는 모터. `robot.yaml` 의 모터 이름임."""

LOG_SKIP_S = 0.0
"""`--log-skip` 기본값. **정책 시작 부분은 버리지 않음** -- 정책이 켜진 직후가 가장 많이
보는 구간임. 연결 직후의 느린 첫 주기들은 접근 구간에 들어가 이미 빠짐."""

WEIGHTS_DIR = Path("config/policies")
"""`--weights` 를 안 주면 `{WEIGHTS_DIR}/{POLICY_NAME}.pt` 를 찾음."""

POLICY_HZ = generalized_walk.HZ
"""정책이 불리는 주기(50Hz). 시뮬이 0.005초 x 4 = 50Hz 로 학습했으므로 같아야 함.

더 빠르게 부르면 모델이 학습 때보다 자주 불려서, 같은 행동이 더 오래 유지되는 것과
같은 효과가 남 -- 시뮬과 다르게 움직임. **그래서 상수로 고정하고 CLI 인자로 안 뺌.**

모터 전송 주기(`--hz`)는 이것과 다를 수 있음 -- 전송은 `control.held()` 가 이 값
그대로 가둬 두고, 그 사이 주기는 직전 목표를 반복해 보냄."""


DEFAULT_APPROACH_S = 3.0
"""기준 자세까지 옮기는 데 쓰는 시간.

**서서히 가야 함.** 토크를 넣는 순간 목표가 멀리 있으면 관절이 튐 -- 점프 가드가
자르기는 하지만 그 전에 큰 토크가 한 번 나감.
"""


def staged(approach_motion, approach_s: float, policy_motion, hold_pose):
    """기준 자세로 옮김 -> 그 자세로 기다림 -> Enter 누르면 정책.

    셋을 한 덩어리로 묶어 제어 루프에 넘김. 루프는 이 안에 단계가 있는 줄 모름.

    `start()` 를 부르기 전까지는 기준 자세를 계속 보냄 -- 사람이 로봇에서 손을 떼고
    자리를 잡을 시간임.
    """
    started = [False, 0.0]
    waiting = dict(hold_pose)

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


class EnterWatcher(QuitWatcher):
    """Enter 를 누르면 정책을 시작시키고, Ctrl-Q 를 누르면 루프를 멈춤.

    `huphy-test` 와 같은 키로 멈추려고 `QuitWatcher` 를 이어받음. 터미널을 한 글자씩
    읽는 모드로 두므로 `input()` 으로 Enter 를 기다릴 수 없음 -- 둘을 한 스레드가
    같이 봄.

    화면이 아니면(파이프, 서비스) 바로 시작함 -- 누를 사람이 없는데 영원히 기다리면
    안 됨. 그때는 Ctrl-C 로 끊음.
    """

    def __init__(self, motion, loop: ControlLoop) -> None:
        super().__init__(loop)
        self.motion = motion

    def __enter__(self) -> "EnterWatcher":
        if not self.armed:
            self.motion.start()
            return self
        super().__enter__()
        return self

    def on_key(self, key: str) -> None:
        # 한 글자 모드라도 ICRNL 이 살아 있어 Enter 는 보통 "\n" 으로 옴.
        if key in ("\n", "\r") and not self.motion.is_started():
            print("  시작함")
            self.motion.start()


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="huphy-run",
        description="학습한 걷기 정책(generalized_walk)으로 양다리를 움직임.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "예시:\n"
            "  huphy-run --vx 0.5 --wz 0.0\n"
            "  huphy-run --vx 1.0 --drain-ms 4 --hz 50 --log\n\n"
            "상태 기계와 토크 가드가 아직 없음. 사람이 지켜보며 돌릴 것.\n"
        ),
    )
    p.add_argument("--config", type=Path, help=f"기본값: 위로 올라가며 {CONFIG_NAME} 을 찾음")
    p.add_argument(
        "--weights", type=Path,
        help=f"가중치 파일. 기본값은 {WEIGHTS_DIR}/{POLICY_NAME}.pt",
    )
    p.add_argument(
        "--vx", type=float, default=0.0,
        help="앞뒤 속도 명령(m/s). 실행 내내 고정값임 (기본 0.0, 제자리)",
    )
    p.add_argument(
        "--wz", type=float, default=0.0,
        help="회전 속도 명령(rad/s). 실행 내내 고정값임 (기본 0.0)",
    )
    p.add_argument(
        "--hz", type=float, default=POLICY_HZ,
        help=(
            f"모터 전송 주기. 기본 {POLICY_HZ:.0f}. 정책은 항상 {POLICY_HZ:.0f}Hz 로만 "
            f"불리고, 이 값을 그보다 높이면 그 사이 주기는 직전 목표를 반복 전송함. "
            f"{POLICY_HZ:.0f} 의 정수 배여야 함"
        ),
    )
    p.add_argument(
        "--drain-ms", type=float, default=DEFAULT_DRAIN_S * 1000.0,
        help=(
            f"응답을 기다리는 최대 시간(ms). 기본 {DEFAULT_DRAIN_S * 1000.0:g}. "
            f"늘리면 늦은 응답을 받지만 모터가 죽으면 매 주기 그만큼 씀"
        ),
    )
    p.add_argument(
        "--send-mode", choices=SEND_MODES, default=SEND_SEQ,
        help=(
            "명령을 버스에 내는 방식. seq=한 모터씩 보내고 응답을 받은 뒤 다음 (기본), "
            "burst=6개를 연달아 보내고 모음. RobStride 응답은 모든 모터가 같은 CAN id 라 "
            "burst 에서 응답이 겹치면 둘 다 깨짐 -- docs/can_communication.md"
        ),
    )
    p.add_argument(
        "--approach", type=float, default=DEFAULT_APPROACH_S,
        help=f"기준 자세까지 옮기는 시간. 기본 {DEFAULT_APPROACH_S:.0f}초",
    )
    p.add_argument(
        "--duration", type=float,
        help="이 초만큼만 돌고 나옴. 생략하면 Ctrl-Q 까지",
    )
    p.add_argument(
        "--gain-scale", type=float, nargs="+", default=[1.0], metavar="SCALE",
        help=(
            "kp/kd 배율. 숫자 하나면 전체, 둘이면 '나머지 발목' 순서 "
            "(예: --gain-scale 0.5 0.8). 기본 1.0. 정책은 학습 게인을 전제로 하므로 "
            "낮추면 시뮬과 다르게 움직임 -- 첫 통전 확인용"
        ),
    )
    p.add_argument(
        "--live", nargs="?", const=live.DEFAULT_PORT, type=int, metavar="PORT",
        help=(
            f"도는 동안 관절 목표·실측을 브라우저로 봄. 포트를 안 주면 {live.DEFAULT_PORT}. "
            "같은 망의 다른 기계에서도 열림"
        ),
    )
    p.add_argument(
        "--log", nargs="?", const=Path("logs"), type=Path, metavar="DIR",
        help=(
            "정책 입출력과 관절별 응답 지연을 CSV 로 남김. 폴더를 안 주면 logs/. "
            "tools/policy_viewer.html, tools/latency_viewer.html 로 열어 볼 수 있음"
        ),
    )
    p.add_argument(
        "--log-skip", type=float, default=LOG_SKIP_S, metavar="SEC",
        help=(
            f"지연 기록에서 정책 시작 뒤 이 초만큼을 버림. 기본 {LOG_SKIP_S:g} (안 버림). "
            "접근·대기·감쇠 정지 구간은 늘 빠짐"
        ),
    )
    p.add_argument(
        "--allow-uncalibrated", action="store_true",
        help="실측 전에도 토크를 넣음",
    )
    p.add_argument("-v", "--verbose", action="store_true")
    return p


def _gain_line(robot, scale: float = 1.0, ankle_scale: float = 1.0) -> str:
    """시작 화면의 게인 줄. **실제로 모터에 걸리는 값을 읽어서** 찍음.

    이 정책은 게인을 덮어쓰지 않으므로 항상 `robot.yaml` 값임.
    """
    parts = getattr(robot, "parts", None) or (robot,)
    config = getattr(parts[0], "config", None)
    motors = getattr(config, "motors", {}) or {}
    pairs = {(m.gains.kp, m.gains.kd) for m in motors.values()}
    if len(pairs) == 1:
        kp, kd = pairs.pop()
        body = f"kp={kp:g} kd={kd:g}"
    else:
        body = "  ".join(
            f"{name} {m.gains.kp:g}/{m.gains.kd:g}" for name, m in motors.items()
        )
    source = "robot.yaml"
    if ankle_scale != scale:
        source += f", 발목 x{ankle_scale:g} / 나머지 x{scale:g}"
    elif scale != 1.0:
        source += f", x{scale:g} 로 낮춤"
    return f"  게인     {body}  ({source})"


def _save_log(log: "policy_log.PolicyLog", args) -> None:
    """정책 기록을 씀. **실패해도 올리지 않음** -- 토크는 이미 끊긴 뒤임."""
    if not len(log):
        print("  정책 기록: 정책이 한 번도 안 불려서 남길 것이 없음")
        return
    try:
        path = log.save(policy_log.default_path(args.log, POLICY_NAME))
    except OSError as e:
        print(f"  정책 기록 저장 실패: {e}")
        return
    note = f", 기록 실패 {log.errors}회" if log.errors else ""
    print(f"  정책 기록: {path}  ({len(log)}줄{note})")


def _resume_on_first_call(motion, latency: "latency_log.LatencyLog"):
    """정책이 처음 불릴 때 지연 기록을 켬. 그 뒤로는 그대로 넘김."""
    started = [False]

    def wrapped(t, observation):
        if not started[0]:
            started[0] = True
            latency.resume()
        return motion(t, observation)

    return wrapped


def _save_latency(latency: "latency_log.LatencyLog", args) -> None:
    """지연 요약을 찍고 CSV 로 씀. 실패해도 올리지 않음."""
    if not len(latency):
        return
    print("\n" + latency.summary())
    try:
        path = latency.save(latency_log.default_path(args.log, POLICY_NAME))
    except OSError as e:
        print(f"  지연 기록 저장 실패: {e}")
        return
    print(f"  지연 기록: {path}")


def _weights_path(args) -> Path:
    """쓸 가중치 파일. 없으면 멈춤."""
    path = args.weights if args.weights else WEIGHTS_DIR / f"{POLICY_NAME}.pt"
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

    if args.hz < POLICY_HZ or (args.hz / POLICY_HZ) % 1.0 > 1e-6:
        raise SystemExit(
            f"--hz 는 {POLICY_HZ:.0f} 의 정수 배여야 함 (받은 값 {args.hz})"
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

    # 이 정책은 양다리 12관절 전용임 -- 다리 하나로는 입력 개수가 안 맞음.
    limbs = all_legs(robot)
    names = {limb.name for limb in limbs}
    expected = {name.split("/")[0] for name in ORDER}
    missing = sorted(expected - names)
    if missing:
        raise SystemExit(
            f"정책이 기대하는 다리가 설정에 없음: {missing} "
            f"(있는 것: {sorted(names)}). robot.yaml 의 limbs 이름이 "
            f"{sorted(expected)} 여야 함"
        )

    # 모터를 켜기 전에 읽음. 규격이 어긋나면 여기서 멈추는 것이 안전함. 출력
    # 개수도 대조함.
    weights = _weights_path(args)
    try:
        model = rsl_rl.load(weights, spec=SPEC, action_dim=len(ORDER))
    except (ValueError, OSError) as e:
        raise SystemExit(f"{e}") from e

    if args.drain_ms < 0:
        raise SystemExit(f"--drain-ms 는 0 이상이어야 함 (받은 값 {args.drain_ms})")
    if len(args.gain_scale) > 2:
        raise SystemExit(
            f"--gain-scale 은 숫자 하나(전체) 또는 둘(나머지 발목)임 (받은 값 {args.gain_scale})"
        )
    body_scale = args.gain_scale[0]
    ankle_scale = args.gain_scale[-1]
    for value in (body_scale, ankle_scale):
        if not 0.0 < value <= 1.0:
            raise SystemExit(f"--gain-scale 은 0 보다 크고 1 이하여야 함 (받은 값 {value})")

    # 게인은 robot.yaml 을 그대로 씀 -- 이 정책이 학습에서 가정한 사양값임.
    options = dict(
        gain_scale=body_scale,
        joint_gain_scale={name: ankle_scale for name in ANKLE_MOTORS},
        allow_uncalibrated=args.allow_uncalibrated,
        drain_s=args.drain_ms / 1000.0,
        send_mode=args.send_mode,
    )
    leg = build_biped(robot, limbs, **options)

    # IMU 는 다리가 아니라 로봇에 붙음 (build_biped 가 센서를 다리가 아니라
    # 로봇에 붙임). .imus 로 꺼냄.
    if not leg.imus:
        raise SystemExit(
            "로봇에 IMU 가 없음. 정책 입력의 6칸이 IMU 값임 "
            "(각속도 3, 중력 방향 3).\nrobot.yaml 의 imus 에 적을 것"
        )

    channels = [limb.channel for limb in limbs]
    try:
        leg.connect()
    except ImportError as e:
        raise SystemExit(f"{e}") from e
    except ConnectionError as e:
        raise SystemExit(f"{e}\n{bring_up_hint(channels)}") from e

    loop = ControlLoop(leg, hz=args.hz, mode=Mode.CONTROL)
    signal.signal(signal.SIGINT, lambda *_: loop.stop())

    # 기준 자세는 0 이 아님 (엉덩이 10도, 무릎 20도). 0 으로 데려다 놓으면
    # 정책이 켜지는 순간 그 차이를 한 번에 메우려 함.
    target_pose = generalized_walk.default_pose()

    hz_note = f" (정책 {POLICY_HZ:.0f}Hz)" if args.hz != POLICY_HZ else ""
    lines = [
        "",
        f"  {leg.id}  {' '.join(channels)}  전송 {args.hz:.0f}Hz{hz_note}",
        f"  정책     {POLICY_NAME}  (입력 {SPEC.obs_dim}, x{SPEC.action_scale})",
        f"  가중치   {weights}",
        _gain_line(leg, body_scale, ankle_scale),
        f"  명령     vx={args.vx:g} m/s  wz={args.wz:g} rad/s (고정값)",
        f"  IMU      {', '.join(i.name for i in leg.imus)}",
        "",
        f"  기준자세 무릎 {target_pose['right_leg/knee']:+.1f}도"
        f"  엉덩이 {target_pose['right_leg/hip_pitch']:+.1f}도",
        f"  {args.approach:.0f}초에 걸쳐 기준 자세로 옮긴 뒤 그 자세로 기다립니다.",
        "  Enter 를 누르면 정책이 시작됩니다.",
        "",
        "  ** 상태 기계와 토크 가드가 없음. 넘어져도 멈추지 않음 **",
        "  Ctrl-Q 로 멈춤. 멈출 때 kp=0 으로 속도를 죽인 뒤 토크를 끊음.",
        "",
    ]
    print("\n".join(lines))

    start_pose = {
        joint: float(leg.get_observation().get(f"{joint}.pos", 0.0))
        for joint in ORDER
    }
    log = None
    if args.log is not None:
        log = policy_log.PolicyLog(generalized_walk.OBS_NAMES, ORDER, robot=leg)
        model = log.wrap_model(model)
    policy_motion = generalized_walk.motion(
        model, leg.imus[0], command=(args.vx, args.wz)
    )
    if log is not None:
        policy_motion = log.wrap_motion(policy_motion)
    # 정책이 도는 동안만 잼. 꺼 둔 채 시작해 정책이 처음 불릴 때 켜고, 감쇠 정지에
    # 들어가면 끔. 접근·대기·정지 구간은 빠짐. --log-skip 을 주면 앞을 더 자름.
    latency = None
    if args.log is not None:
        latency = latency_log.LatencyLog(
            drain_ms=args.drain_ms, hz=args.hz, skip_s=args.log_skip, active=False,
        ).attach(leg)
        loop.on_settle = latency.on_settle
        policy_motion = _resume_on_first_call(policy_motion, latency)
    motion = staged(
        approach(target_pose, start_pose, args.approach),
        args.approach,
        policy_motion,
        hold_pose=target_pose,
    )
    if args.hz != POLICY_HZ:
        motion = held(motion, inner_hz=POLICY_HZ, outer_hz=args.hz)

    # 실시간 뷰어는 루프에 넘기는 쪽만 감쌈. EnterWatcher 는 원래 motion 의 start()
    # 를 불러야 함.
    looped, server = motion, None
    if args.live is not None:
        feed = live.LiveFeed(
            hz=args.hz, title=f"huphy-run {POLICY_NAME} · {leg.id}",
            name=POLICY_NAME, save_dir=args.log or Path("logs"),
            robot=leg,
        )
        server = live.start(feed, args.live)
        if server is not None:
            looped = feed.wrap_motion(motion)

    try:
        with EnterWatcher(motion, loop):
            stats = loop.run(looped, duration_s=args.duration)
        print("\n" + failures.report(leg, stats))
        if stats.link_loss is not None:
            # 정상 종료와 구분되어야 함 -- 둘 다 조용히 끝남.
            print(f"\n  ** 통신 두절로 멈춤: {stats.link_loss} **")
        if not stats.kept_up:
            print("  주기를 못 지킴. 정책이 시뮬과 다르게 움직임")
            return 1
    finally:
        leg.disconnect()
        print("\n  종료. 토크가 끊겼음")
        if server is not None:
            server.stop()
        if log is not None:
            _save_log(log, args)
        if latency is not None:
            _save_latency(latency, args)
    return 0


if __name__ == "__main__":  # pragma: no cover
    sys.exit(main())
