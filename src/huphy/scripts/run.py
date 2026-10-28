"""정책 실행 — 정책 꾸러미 하나를 골라 양다리를 움직임.

    huphy-run --policy generalized_walk --cmd vx=0.5 --cmd wz=0.1
    huphy-run --policy huphy_squat --cmd h_star=0.80
    huphy-run --policy huphy_squat --checkpoint 4500

브링업과 **같은 다리·같은 루프**를 씀. 다르게 들어가는 것은 매 주기 관절 목표를
내는 것 하나뿐임 -- 브링업은 사인파 같은 것을 넣고, 여기는 모델을 넣음.


## 정책은 꾸러미 폴더 하나임

`--policy 이름` 이면 `config/policies/<이름>/` 을 씀 (폴더 경로를 그대로 줘도 됨).
그 안의 `deploy_contract.json` 이 관절 순서, 기본 자세, 자르기 범위, 관찰 구성, 주기,
명령, 가중치 파일 이름을 정함 -- 읽는 쪽은 `control/deploy.py`. **이 파일에는 정책별
숫자가 없음.**


## 명령: 고정값(`--cmd`) 또는 pygame 조작(`--teleop`)

이름은 계약이 정함: walk 는 `vx`, `wz`, squat 는 `h_star`. 안 주면 계약의 시작값
(walk 0, squat 기본자세 높이 0.8914 m). 범위가 있는 명령은 범위 밖이면 시작 전에
거부함. 속도 제한이 있는 명령(`h_star`, 0.1 m/s)은 시작값에서 목표까지 그 속도로 감.

`--teleop` 이면 pygame 창(별도 프로세스, `control/teleop.py`)에서 실시간으로 조작함.
walk 는 W/S·A/D 를 누르는 동안 가속(적분), 떼면 감속해 0. 상한·가속·감속은
`--max/--accel/--decel 이름=값`. squat 는 ↑/↓ 로 높이.


## 게인은 `robot.yaml`을 그대로 씀

정책은 **학습에 쓴 게인**으로 돌아야 함. `robot.yaml`(`robot_v1.0.yaml`)이 곧
학습에서 가정한 사양값이라 덮어쓰지 않음 (135/75/130/60, `config/robot.yaml` 참조).


## 아직 없는 것

    상태 기계        지금은 정책 하나만 돎. 넘어져도 안 멈춤
    토크 가드        한계 검사가 발목 토크 경로엔 안 걸림(위치 출력만 써서 해당 없음)

**사람이 옆에서 지켜보며 돌릴 것.**
"""

from __future__ import annotations

import argparse
import logging
import signal
import sys
from pathlib import Path

from ..config import ConfigError, load_robot
from ..control import ControlLoop, Mode, deploy, held, teleop
from ..motors.canbus import DEFAULT_DRAIN_S, bring_up_hint
from ..robots.leg import SEND_BURST, SEND_MODES, HostIdError
from ..telemetry import latency_log, live, motor_log, policy_log, run_name
from . import failures
from .bringup import build_biped
from .commission import CONFIG_NAME, _find_config, all_legs
from .selftest import QuitWatcher, approach

logger = logging.getLogger("huphy.run")

ANKLE_MOTORS = ("ankle_a", "ankle_b")
"""`--gain-scale` 의 두 번째 숫자가 걸리는 모터. `robot.yaml` 의 모터 이름임."""

LOG_SKIP_S = 0.0
PROBE_DEFAULT = 2
"""`--probe` 만 줬을 때 다리당 한 주기 읽기 수. 모터 6 x (전류, 전압) = 12칸을 도므로
100Hz 에서 값 하나가 약 17Hz 로 갱신됨."""
"""`--log-skip` 기본값. **정책 시작 부분은 버리지 않음** -- 정책이 켜진 직후가 가장 많이
보는 구간임. 연결 직후의 느린 첫 주기들은 접근 구간에 들어가 이미 빠짐."""

POLICIES_DIR = Path("config/policies")
"""`--policy 이름` 을 찾는 곳. 저장소 꼭대기 기준."""



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
        description="정책 꾸러미(config/policies/<이름>/)로 양다리를 움직임.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "예시:\n"
            "  huphy-run --policy generalized_walk --cmd vx=0.5\n"
            "  huphy-run --policy huphy_squat --cmd h_star=0.80 --log\n\n"
            "상태 기계와 토크 가드가 아직 없음. 사람이 지켜보며 돌릴 것.\n"
        ),
    )
    p.add_argument("--config", type=Path, help=f"기본값: 위로 올라가며 {CONFIG_NAME} 을 찾음")
    p.add_argument(
        "--policy", required=True, metavar="NAME|DIR",
        help=(
            f"정책 꾸러미 (필수). 이름이면 {POLICIES_DIR}/<이름>/, 폴더 경로도 됨. "
            f"예: generalized_walk, huphy_squat"
        ),
    )
    p.add_argument(
        "--checkpoint", metavar="N",
        help="계약의 checkpoints 중 하나. 기본은 계약의 default_checkpoint",
    )
    p.add_argument(
        "--cmd", action="append", default=[], metavar="NAME=VALUE",
        help=(
            "정책 명령. 여러 번 줄 수 있음 (예: --cmd vx=0.5 --cmd wz=0.1, "
            "--cmd h_star=0.80). 이름은 정책 계약이 정함. --teleop 없으면 실행 내내 고정, "
            "있으면 높이 명령의 시작 목표로만 씀"
        ),
    )
    p.add_argument(
        "--teleop", action="store_true",
        help=(
            "pygame 창으로 명령을 실시간 조작 (별도 프로세스). walk: W/S vx, A/D wz -- "
            "누르는 동안 가속, 떼면 감속해 0. squat: 위/아래 화살표로 h_star. "
            "창에서 Enter 시작, Space 속도 0, Esc·Ctrl-C·Ctrl-Q 멈춤. 화면이 필요함 (ssh 면 -X)"
        ),
    )
    for name, what in (("max", "속도 상한"), ("accel", "누르는 동안의 가속도 (높이는 변화 속도 m/s)"),
                       ("decel", "뗐을 때의 감속도")):
        p.add_argument(
            f"--{name}", action="append", default=[], metavar="NAME=VALUE",
            help=f"--teleop 의 {what}. 예: --{name} vx=0.5. 여러 번 줄 수 있음",
        )
    p.add_argument(
        "--hz", type=float,
        help=(
            "모터 전송 주기. 기본은 정책 주기(계약의 control_rate_hz). 정책은 항상 "
            "그 주기로만 불리고, 이 값을 그보다 높이면 그 사이 주기는 직전 목표를 "
            "반복 전송함. 정책 주기의 정수 배여야 함"
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
        "--send-mode", choices=SEND_MODES, default=SEND_BURST,
        help=(
            "명령을 버스에 내는 방식. burst=6개를 연달아 보내고 모음 (기본), "
            "seq=한 모터씩 보내고 응답을 받은 뒤 다음. burst 는 모터마다 호스트 id 가 "
            "0xF0+id 여야 함 -- 연결할 때 확인하고 아니면 멈춤. 못 맞춘 모터가 있으면 seq. "
            "docs/can_communication.md 4절"
        ),
    )
    p.add_argument(
        "--probe", action="store_true",
        help=(
            "모터 전류(iq)·전압(VBUS)을 읽어 기록함. 명령에 실어 보내는 파라미터 읽기로 받음. "
            "burst 에서만. 끝에 요약을 찍고 <--log 폴더 또는 logs>/motor/ 에 JSON 으로 남김"
        ),
    )
    p.add_argument(
        "--probe-per-cycle", type=int, default=PROBE_DEFAULT, metavar="N",
        help=(
            f"--probe 일 때 한 주기에 다리마다 실어 보낼 읽기 수. 기본 {PROBE_DEFAULT} -- "
            f"모터 6 x (전류, 전압) 12칸을 돌아 100Hz 에서 값 하나가 약 17Hz"
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
            "정책 입출력과 관절별 응답 지연을 JSON 으로 남김. 폴더를 안 주면 logs/. 종류별 폴더로 나뉨. "
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
    p.add_argument(
        "--profile", action="store_true",
        help=(
            "한 주기 안 단계(관측/정책/송신/수거)별 소요 시간을 재서 끝에 찍음. "
            "주기를 못 지킬 때 범인을 좁히는 용도"
        ),
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


def _log_naming(args, policy, model, commands) -> tuple:
    """`(이름 조각들, run 칸)`. 두 기록이 같은 것을 씀.

    이름에는 **기본값이 아닌 것만** 들어감 (`telemetry/run_name.py`). 첫 줄에는
    기본값인 것까지 전부 들어감 -- 이름은 찾기용, 첫 줄은 정확한 기록임.

    `teleop` 이면 명령값이 매 주기 바뀌어 이름에 못 넣음. 이름에는 `teleop` 만
    적고 실제 값은 `obs/cmd/<이름>` 열에 주기마다 들어가 있음.
    """
    targets = dict(getattr(commands, "targets", {}) or {})
    starts = {k: v.start for k, v in policy.commands.items()}
    parts = run_name.tags(
        gain_scale=args.gain_scale,
        commands=targets,
        defaults=starts,
        teleop=bool(args.teleop),
        extra={
            "hz": args.hz if args.hz and args.hz != policy.hz else None,
            "drain": args.drain_ms if args.drain_ms != 2.0 else None,
            args.send_mode: True if args.send_mode != SEND_BURST else None,
            "probe": args.probe,
        },
    )
    run_meta = run_name.header(
        policy=policy.name,
        checkpoint=model.checkpoint,
        gain=args.gain_scale,
        hz=args.hz or policy.hz,
        drain_ms=args.drain_ms,
        send_mode=args.send_mode,
        probe=args.probe,
        approach=args.approach,
        teleop=bool(args.teleop),
        **{k: float(v) for k, v in targets.items()},
    )
    return parts, run_meta


def _save_log(log: "policy_log.PolicyLog", args, name: str, parts, run_meta) -> None:
    """정책 기록을 씀. **실패해도 올리지 않음** -- 토크는 이미 끊긴 뒤임."""
    if not len(log):
        print("  정책 기록: 정책이 한 번도 안 불려서 남길 것이 없음")
        return
    try:
        path = log.save(policy_log.default_path(args.log, name, parts), run_meta)
    except OSError as e:
        print(f"  정책 기록 저장 실패: {e}")
        return
    note = f", 기록 실패 {log.errors}회" if log.errors else ""
    print(f"  정책 기록: {path}  ({len(log)}줄{note})")


def _save_motors(motors: "motor_log.MotorLog", directory: Path, name: str,
                 parts=(), run_meta=None) -> None:
    """전류·전압 요약을 찍고 JSON 으로 씀. 실패해도 올리지 않음 -- 토크는 이미 끊긴 뒤임."""
    print("\n" + motors.summary())
    if not len(motors):
        print("  모터 기록: 받은 값이 없음")
        return
    try:
        path = motors.save(motor_log.default_path(directory, name, parts), run_meta or {})
    except OSError as e:
        print(f"  모터 기록 저장 실패: {e}")
        return
    note = f", 기록 실패 {motors.errors}회" if motors.errors else ""
    print(f"  모터 기록: {path}  ({len(motors)}줄{note})")


def _resume_on_first_call(motion, latency: "latency_log.LatencyLog"):
    """정책이 처음 불릴 때 지연 기록을 켬. 그 뒤로는 그대로 넘김."""
    started = [False]

    def wrapped(t, observation):
        if not started[0]:
            started[0] = True
            latency.resume()
        return motion(t, observation)

    return wrapped


def _save_latency(latency: "latency_log.LatencyLog", args, name: str,
                  parts, run_meta) -> None:
    """지연 요약을 찍고 JSON 으로 씀. 실패해도 올리지 않음."""
    if not len(latency):
        return
    print("\n" + latency.summary())
    try:
        path = latency.save(latency_log.default_path(args.log, name, parts), run_meta)
    except OSError as e:
        print(f"  지연 기록 저장 실패: {e}")
        return
    print(f"  지연 기록: {path}")


def _policy_folder(arg: str, config_path: Path) -> Path:
    """`--policy` -> 꾸러미 폴더. 폴더 경로면 그대로, 이름이면 정책 폴더에서 찾음.

    찾는 순서: 주어진 경로 / robot.yaml 옆의 `policies/<이름>` / `config/policies/<이름>`.
    """
    candidates = [
        Path(arg),
        Path(config_path).parent / "policies" / arg,
        POLICIES_DIR / arg,
    ]
    for folder in candidates:
        if (folder / deploy.CONTRACT_NAME).is_file():
            return folder
    raise SystemExit(
        f"정책 꾸러미를 못 찾음: {arg}\n"
        f"{POLICIES_DIR}/<이름>/{deploy.CONTRACT_NAME} 가 있어야 함. 찾아본 곳: "
        + ", ".join(str(c) for c in candidates)
    )


def _parse_commands(items, policy: "deploy.DeployPolicy") -> "deploy.Commands":
    """`--cmd 이름=값` 들 -> 명령. 이름이 틀리거나 범위 밖이면 멈춤 (CAN 을 열기 전)."""
    targets = {}
    for item in items:
        name, sep, value = item.partition("=")
        if not sep:
            raise SystemExit(f"--cmd 는 이름=값 꼴이어야 함 (받은 값 {item!r})")
        try:
            targets[name.strip()] = float(value)
        except ValueError:
            raise SystemExit(f"--cmd {name}: 숫자가 아님 ({value!r})") from None
    for name, value in targets.items():
        spec = policy.commands.get(name)
        if spec is not None and spec.range is not None and not spec.range[0] <= value <= spec.range[1]:
            raise SystemExit(
                f"--cmd {name}={value:g} 는 학습 범위 [{spec.range[0]:g}, {spec.range[1]:g}] 밖임"
            )
    try:
        return deploy.Commands(policy.commands.values(), targets)
    except deploy.ContractError as e:
        raise SystemExit(f"--cmd: {e}") from e


def _name_values(items, flag: str) -> dict:
    """`이름=값` 목록 -> 사전. 꼴이 틀리면 멈춤."""
    out = {}
    for item in items:
        name, sep, value = item.partition("=")
        if not sep:
            raise SystemExit(f"--{flag} 는 이름=값 꼴이어야 함 (받은 값 {item!r})")
        try:
            out[name.strip()] = float(value)
        except ValueError:
            raise SystemExit(f"--{flag} {name}: 숫자가 아님 ({value!r})") from None
    return out


def _make_teleop(args, policy, commands, title: str):
    """`--teleop` -> pygame 프로세스를 띄움. 창이 안 뜨면 멈춤 (CAN 을 열기 전)."""
    overrides = {k: _name_values(getattr(args, k), k) for k in ("max", "accel", "decel")}
    if not args.teleop:
        used = [f"--{k}" for k, v in overrides.items() if v]
        if used:
            raise SystemExit(f"{' '.join(used)} 는 --teleop 과 같이 쓸 것")
        return None
    if not commands.specs:
        raise SystemExit("--teleop: 이 정책에는 조작할 명령이 없음")
    fixed = [n for n in commands.targets if n in teleop.VELOCITY_KEYS and n in
             {i.partition("=")[0].strip() for i in args.cmd}]
    if fixed:
        raise SystemExit(
            f"--teleop 에서는 속도 명령을 --cmd 로 주지 않음 ({fixed}). 키를 떼면 0 이 됨"
        )
    try:
        axes = teleop.make_axes(policy.commands.values(), overrides)
        return teleop.Teleop(axes, start=dict(commands.targets), title=title).start()
    except teleop.TeleopError as e:
        raise SystemExit(f"--teleop: {e}") from e


def _command_line(commands: "deploy.Commands") -> str:
    """시작 화면의 명령 줄."""
    if not commands.specs:
        return "  명령     없음"
    parts = []
    for name, spec in commands.specs.items():
        target = commands.targets[name]
        if spec.max_rate is not None and target != spec.start:
            parts.append(f"{name}={spec.start:g}->{target:g} ({spec.max_rate:g}/s)")
        else:
            parts.append(f"{name}={target:g}")
    return "  명령     " + "  ".join(parts) + "  (고정값)"


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

    # 정책 꾸러미. 모터를 켜기 전에 다 읽고 검사함 -- 계약 모양, sha256, 입출력
    # 개수, 참조 입출력(있으면). 어긋나면 여기서 멈춤.
    try:
        policy = deploy.load(_policy_folder(args.policy, path))
        model = policy.load_model(args.checkpoint)
    except (deploy.ContractError, ValueError, OSError) as e:
        raise SystemExit(f"{e}") from e
    commands = _parse_commands(args.cmd, policy)

    policy_hz = policy.hz
    if args.hz is None:
        args.hz = policy_hz
    if args.hz < policy_hz or (args.hz / policy_hz) % 1.0 > 1e-6:
        raise SystemExit(
            f"--hz 는 {policy_hz:.0f} 의 정수 배여야 함 (받은 값 {args.hz})"
        )

    # 계약의 관절 이름이 가리키는 다리가 설정에 다 있어야 함.
    limbs = all_legs(robot)
    names = {limb.name for limb in limbs}
    expected = {name.split("/")[0] for name in policy.order}
    missing = sorted(expected - names)
    if missing:
        raise SystemExit(
            f"정책이 기대하는 다리가 설정에 없음: {missing} "
            f"(있는 것: {sorted(names)}). robot.yaml 의 limbs 이름이 "
            f"{sorted(expected)} 여야 함"
        )

    if args.drain_ms < 0:
        raise SystemExit(f"--drain-ms 는 0 이상이어야 함 (받은 값 {args.drain_ms})")
    # --probe 는 켜기만 함. 개수는 --probe-per-cycle. 아래는 "켜졌으면 개수, 아니면 None" 하나로 씀.
    # (--probe [N] 으로 두면 뒤에 오는 서브명령 이름을 N 으로 읽어 깨짐)
    args.probe = args.probe_per_cycle if args.probe else None
    if args.probe is not None and args.probe < 1:
        raise SystemExit(f"--probe-per-cycle 은 1 이상이어야 함 (받은 값 {args.probe})")
    if args.probe and args.send_mode != SEND_BURST:
        raise SystemExit("--probe 는 --send-mode burst 에서만 됨 (읽기 응답을 CAN id 로 가름)")
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
        probe=args.probe or 0,
    )
    leg = build_biped(robot, limbs, **options)
    # 연결 전에 붙임 -- 연결할 때의 첫 읽기부터 셈이 맞음.
    motors = (
        motor_log.MotorLog(hz=args.hz, probe=args.probe).attach(leg) if args.probe else None
    )

    # IMU 는 다리가 아니라 로봇에 붙음 (build_biped 가 센서를 다리가 아니라
    # 로봇에 붙임). .imus 로 꺼냄.
    if not leg.imus:
        raise SystemExit(
            "로봇에 IMU 가 없음. 정책 입력의 6칸이 IMU 값임 "
            "(각속도 3, 중력 방향 3).\nrobot.yaml 의 imus 에 적을 것"
        )

    # pygame 창. 못 띄우면 여기서 멈춤 -- 아직 CAN 을 안 엶.
    pad = _make_teleop(args, policy, commands, f"huphy-run {policy.name}")
    if pad is not None:
        commands.source = pad.targets

    channels = [limb.channel for limb in limbs]
    try:
        leg.connect()
    except (ImportError, ConnectionError, HostIdError) as e:
        if pad is not None:
            pad.close()
        hint = f"\n{bring_up_hint(channels)}" if isinstance(e, ConnectionError) else ""
        raise SystemExit(f"{e}{hint}") from e

    loop = ControlLoop(leg, hz=args.hz, mode=Mode.CONTROL, profile=args.profile)
    signal.signal(signal.SIGINT, lambda *_: loop.stop())

    # 기준 자세 = 계약의 행동 오프셋. 0 이 아님. 0 으로 데려다 놓으면 정책이
    # 켜지는 순간 그 차이를 한 번에 메우려 함.
    target_pose = policy.default_pose()

    golden = getattr(model, "golden_max_diff", None)
    golden_note = f", 참조 입출력 통과 {golden:.1e}" if golden is not None else ""
    hz_note = f" (정책 {policy_hz:.0f}Hz)" if args.hz != policy_hz else ""
    lines = [
        "",
        f"  {leg.id}  {' '.join(channels)}  전송 {args.hz:.0f}Hz{hz_note}",
        f"  정책     {policy.name}  (입력 {policy.obs_dim}, 출력 {len(policy.order)})",
        f"  가중치   {model.path}  (체크포인트 {model.checkpoint}{golden_note})",
        _gain_line(leg, body_scale, ankle_scale),
        pad.help_line() if pad is not None else _command_line(commands),
        f"  IMU      {', '.join(i.name for i in leg.imus)}",
        "",
        f"  기준자세 무릎 {target_pose['right_leg/knee']:+.1f}도"
        f"  엉덩이 {target_pose['right_leg/hip_pitch']:+.1f}도",
        f"  {args.approach:.0f}초에 걸쳐 기준 자세로 옮긴 뒤 그 자세로 기다립니다.",
        "  Enter 를 누르면 정책이 시작됩니다."
        + (" (pygame 창에서도 됨)" if pad is not None else ""),
        "",
        "  ** 상태 기계와 토크 가드가 없음. 넘어져도 멈추지 않음 **",
        "  Ctrl-Q 로 멈춤. 멈출 때 kp=0 으로 속도를 죽인 뒤 토크를 끊음.",
        "",
    ]
    print("\n".join(lines))

    start_pose = {
        joint: float(leg.get_observation().get(f"{joint}.pos", 0.0))
        for joint in policy.order
    }
    log = None
    # 이름 조각과 첫 줄은 한 번만 만들어 두 기록이 같이 씀.
    naming = (
        _log_naming(args, policy, model, commands)
        if args.log is not None or motors is not None else ((), "")
    )

    if args.log is not None:
        log = policy_log.PolicyLog(policy.obs_names, policy.order, robot=leg)
        model = log.wrap_model(model)
    policy_motion = policy.motion(model, leg.imus[0], commands=commands)
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
    if args.hz != policy_hz:
        motion = held(motion, inner_hz=policy_hz, outer_hz=args.hz)

    # 실시간 뷰어는 루프에 넘기는 쪽만 감쌈. EnterWatcher 는 원래 motion 의 start()
    # 를 불러야 함.
    looped, server = motion, None
    if args.live is not None:
        feed = live.LiveFeed(
            hz=args.hz, title=f"huphy-run {policy.name} · {leg.id}",
            name=policy.name, save_dir=args.log or Path("logs"),
            robot=leg,
        )
        server = live.start(feed, args.live)
        if server is not None:
            looped = feed.wrap_motion(motion)
    if pad is not None:
        looped = teleop.teleop_motion(looped, pad, loop, motion)

    try:
        with EnterWatcher(motion, loop):
            stats = loop.run(looped, duration_s=args.duration)
        print("\n" + failures.report(leg, stats))
        if loop.stage_times is not None:
            print("\n" + loop.stage_times.summary())
        if stats.link_loss is not None:
            # 정상 종료와 구분되어야 함 -- 둘 다 조용히 끝남.
            print(f"\n  ** 통신 두절로 멈춤: {stats.link_loss} **")
        if not stats.kept_up:
            print("  주기를 못 지킴. 정책이 시뮬과 다르게 움직임")
            return 1
    finally:
        leg.disconnect()
        print("\n  종료. 토크가 끊겼음")
        if pad is not None:
            pad.close()
        if server is not None:
            server.stop()
        if log is not None:
            _save_log(log, args, policy.name, *naming)
        if latency is not None:
            _save_latency(latency, args, policy.name, *naming)
        if motors is not None:
            _save_motors(motors, args.log or Path("logs"), policy.name, *naming)
    return 0


if __name__ == "__main__":  # pragma: no cover
    sys.exit(main())
