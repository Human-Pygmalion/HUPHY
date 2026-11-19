#!/usr/bin/env python3
"""정책 꾸러미를 MuJoCo 안에서 돌림. 실물에 올리기 전에 우리 코드를 보는 것.

    python3 sim/rollout.py --policy walk
    python3 sim/rollout.py --policy motion399 --duration 10 --log logs
    python3 sim/rollout.py --policy walk --view            # MuJoCo 창

`huphy-run` 과 **같은 계산 경로**를 지남 -- 관측 조립(`control/deploy.py`), 추론
(`control/rsl_rl.py`), 발목 기구학(`kinematics/`), 한계·점프 가드(`safety/`),
기록(`telemetry/`) 이 전부 실물용 파일임. CAN 과 시리얼 IMU 자리에만
`huphy_sim/bus.py`·`imu.py` 가 들어감.

꾸러미 고르기·명령 해석·기록 이름은 `scripts/run.py` 의 함수를 그대로 가져다 씀.
거기를 고치면 여기도 따라 바뀌는 것이 **의도한 것임** -- 둘이 갈라지면 "같은
경로로 돌렸다" 가 거짓이 됨.

무엇이 검증되고 무엇이 안 되는지는 `sim/README.md` 에 있음.
"""

from __future__ import annotations

import argparse
import logging
import os
import signal
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))
SRC = HERE.parent / "src"
if SRC.is_dir() and str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))   # 설치하지 않고 돌릴 수 있게. ros/ 도 같은 방식임

from huphy.control import ControlLoop, Mode, deploy, held               # noqa: E402
from huphy.scripts.run import (                                         # noqa: E402
    EnterWatcher,
    _command_line,
    _parse_commands,
    _policy_folder,
    _save_log,
    staged,
)
from huphy.scripts.selftest import approach                             # noqa: E402
from huphy.telemetry import policy_log, run_name                        # noqa: E402

from huphy_sim.build import DEFAULT_CONFIG, build_sim_biped, default_qpos  # noqa: E402

DEFAULT_LOG_DIR = HERE / "logs"
"""기록 폴더. **실물 기록(`logs/`)과 섞이지 않게 `sim/logs/` 에 둠.**

이름 끝에 `_sim` 이 붙고 파일 안 `run` 에 `sim: true` 가 있어서 구분은 되지만,
폴더가 같으면 목록을 볼 때마다 가려내야 함. 뷰어는 파일을 골라 여는 방식이라
폴더가 달라도 나란히 볼 수 있음.
"""

DEFAULT_APPROACH_S = 0.2
"""기준 자세로 옮기는 시간. **짧게 둠.**

시뮬은 기준 자세에 **이미 놓고** 시작하므로 (`world.reset(default_qpos(...))`)
옮길 거리가 없음. 실물에서 3초인 이유는 멀리 있는 목표로 관절이 튀지 않게 하려는
것과, 사람이 로봇에서 손을 떼고 자리를 잡을 시간이기 때문임. 시뮬에는 둘 다 없음.

이 구간과 Enter 대기 구간에는 **물리를 굴리지 않음** (`world.frozen`). 기준 자세는
가만히 못 버티므로(`sim/README.md` 5절) 굴리면 정책이 시작되기 전에 넘어짐.
실물에서 그동안 사람이 붙잡고 있는 자리에 해당하는 것임.
"""
ECHO_S = 1.0
"""정책 출력값을 찍는 간격 (초). `--echo 0` 으로 끔."""

FALL_HEIGHT_M = 0.25
FALL_TILT = 0.7
"""넘어짐 판정. 몸통이 이보다 낮거나, 기울기가 이보다 크면 멈춤.

**높이만으로 보지 않는 이유**: `squat` 은 몸통을 0.46 m 까지 내리는 것이 정상임.
학습 쪽 종료 조건(0.7 m, `env_cfg.py` 의 `root_height_below_minimum`)을 쓰면 정상
스쿼트가 넘어짐으로 잡힘.

기울기는 중력벡터의 수평 성분 크기임 (`ImuState.gravity`). 0.7 은 약 45도.
"""


def naming(args, policy, model, commands, world) -> tuple:
    """기록 이름 조각과 첫 줄. **이름에 `sim` 이 들어감.**

    실물 기록과 같은 폴더(`logs/policy/`)에 같은 열 이름으로 남으므로, 이름에
    표시가 없으면 나중에 둘을 구분할 수 없음. 첫 줄(`run`)에도 MJCF 파일 이름과
    물리 timestep 을 적어 둠 -- 모델이 바뀌면 결과가 달라지는데 그 사실이 파일에
    남아야 함.

    `scripts/run.py` 의 `_log_naming` 을 쓰지 않는 이유: 그쪽은 CAN 인자
    (`drain_ms`, `send_mode`, `gain_scale`)를 읽는데 여기에는 그런 인자가 없음.
    """
    targets = dict(getattr(commands, "targets", {}) or {})
    starts = {k: v.start for k, v in policy.commands.items()}
    parts = run_name.tags(
        commands=targets,
        defaults=starts,
        extra={
            "sim": True,
            "hz": args.hz if args.hz and args.hz != policy.hz else None,
        },
    )
    meta = run_name.header(
        policy=policy.name,
        checkpoint=model.checkpoint,
        sim=True,
        mjcf=world.mjcf.name,
        timestep_ms=world.timestep_s * 1000.0,
        hz=args.hz or policy.hz,
        approach=args.approach,
        **{k: float(v) for k, v in targets.items()},
    )
    return parts, meta


SHORT = {"hip_pitch": "hp", "hip_roll": "hr", "hip_yaw": "hy",
         "knee": "kn", "ankle_a": "aa", "ankle_b": "ab"}


def _echo(elapsed, height, tilt, command, observation, order) -> str:
    """정책이 낸 목표각과 실제 각의 차이를 한 덩어리로 찍음.

    창을 안 띄우고 돌릴 때(ssh, 시험) 정책이 무엇을 냈는지 볼 길이 이것뿐임.
    창을 띄웠으면 같은 값이 반투명 로봇으로도 보임 (`SimWorld.ghost`).
    """
    lines = [f"  정책 {elapsed:5.2f}초  몸통 {height:.3f} m  기울기 {tilt:.2f}"]
    limbs: dict = {}
    for key in order:
        limb, _, motor = key.partition("/")
        limbs.setdefault(limb, []).append((motor, key))
    for limb, motors in limbs.items():
        target = "  ".join(
            f"{SHORT.get(m, m)} {float(command[k]):+7.1f}" for m, k in motors
        )
        worst = max(
            abs(float(command[k]) - float(observation.get(f"{k}.pos", 0.0)))
            for _, k in motors
        )
        lines.append(f"    {limb:9s} 목표도 {target}   (실측과 최대 {worst:.1f}도)")
    return "\n".join(lines)


class _now:
    """`--now` 용. `EnterWatcher` 자리에 들어가 바로 시작시킴."""

    def __init__(self, motion) -> None:
        self.motion = motion

    def __enter__(self):
        self.motion.start()
        return self

    def __exit__(self, *exc) -> bool:
        return False


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument("--policy", required=True, help="config/policies/ 안의 꾸러미 이름")
    p.add_argument("--checkpoint", help="가중치 파일. 생략하면 계약의 기본값")
    p.add_argument("--config", type=Path, help=f"로봇 설정 (기본 {DEFAULT_CONFIG})")
    p.add_argument("--duration", type=float, default=10.0, help="돌릴 시간 (초)")
    p.add_argument("--approach", type=float, default=DEFAULT_APPROACH_S,
                   help=f"기준 자세로 옮기는 시간 (초, 기본 {DEFAULT_APPROACH_S}). "
                        "이 구간과 Enter 대기 구간에는 물리를 굴리지 않음")
    p.add_argument("--now", action="store_true",
                   help="Enter 를 기다리지 않고 바로 시작. 화면이 아니면(파이프) 늘 그럼")
    p.add_argument("--echo", type=float, default=ECHO_S, metavar="초",
                   help=f"정책 출력값을 찍는 간격 (기본 {ECHO_S:g}초, 0 이면 안 찍음)")
    p.add_argument("--no-ghost", dest="ghost", action="store_false",
                   help="--view 창에 명령 자세(반투명 로봇)를 겹치지 않음")
    p.add_argument("--hz", type=float, help="전송 주기. 기본은 정책 주기")
    p.add_argument("--cmd", action="append", default=[], metavar="이름=값",
                   help="정책 명령 (예: vx=0.5). 여러 번 줄 수 있음")
    p.add_argument("--log", nargs="?", type=Path, const=DEFAULT_LOG_DIR,
                   help=f"기록을 남길 폴더 (기본 {DEFAULT_LOG_DIR}). "
                        "열 이름은 실물 로그와 같고 이름 끝에 _sim 이 붙음")
    p.add_argument("--log-skip", type=float, default=0.0, help=argparse.SUPPRESS)
    p.add_argument("--drain-ms", type=float, default=0.0, help=argparse.SUPPRESS)
    p.add_argument("--extra", type=int, default=0, help=argparse.SUPPRESS)
    p.add_argument("--view", action="store_true", help="MuJoCo 창을 띄움")
    p.add_argument("--verbose", action="store_true")
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(levelname)s %(message)s",
    )
    config = args.config or DEFAULT_CONFIG

    try:
        policy = deploy.load(_policy_folder(args.policy, config))
        model = policy.load_model(args.checkpoint)
    except (deploy.ContractError, ValueError, OSError) as e:
        raise SystemExit(f"{e}") from e
    commands = _parse_commands(args.cmd, policy)

    policy_hz = policy.hz
    if args.hz is None:
        args.hz = policy_hz
    if args.hz < policy_hz or (args.hz / policy_hz) % 1.0 > 1e-6:
        raise SystemExit(f"--hz 는 {policy_hz:.0f} 의 정수 배여야 함 (받은 값 {args.hz})")

    robot, world = build_sim_biped(config=config, view=args.view)
    world.cycle_s = 1.0 / float(args.hz)

    target_pose = policy.default_pose()
    world.reset(default_qpos(target_pose))
    robot.connect()

    loop = ControlLoop(robot, hz=args.hz, mode=Mode.CONTROL)
    signal.signal(signal.SIGINT, lambda *_: loop.stop())

    golden = getattr(model, "golden_max_diff", None)
    print("\n".join([
        "",
        f"  시뮬     {world.mjcf.name}  timestep {world.timestep_s*1000:.0f}ms"
        f"  주기 {args.hz:.0f}Hz",
        f"  정책     {policy.name}  (입력 {policy.obs_dim}, 출력 {len(policy.order)})",
        f"  가중치   {model.path}"
        + (f"  (참조 입출력 통과 {golden:.1e})" if golden is not None else ""),
        _command_line(commands),
        f"  기준자세 무릎 {target_pose['right_leg/knee']:+.1f}도"
        f"  엉덩이 {target_pose['right_leg/hip_pitch']:+.1f}도",
        f"  몸통높이 {world.base_height_m:.3f} m 에서 시작",
        f"  {args.approach:g}초에 걸쳐 기준 자세로 옮긴 뒤 그 자세로 기다립니다.",
        "  Enter 를 누르면 정책이 시작됩니다. 그때까지 물리를 굴리지 않습니다.",
        f"  정책이 {args.duration:g}초 돌면 멈춥니다.",
        "",
        "  ** 실물이 아님. 게인·지연·인코더 영점·발열은 이 경로로 검증되지 않음 **",
        "",
    ]))

    start_pose = {
        joint: float(robot.get_observation().get(f"{joint}.pos", 0.0))
        for joint in policy.order
    }

    log = None
    log_naming = (naming(args, policy, model, commands, world)
                  if args.log is not None else ((), ""))
    if args.log is not None:
        log = policy_log.PolicyLog(policy.obs_names, policy.order, robot=robot)
        model = log.wrap_model(model)
    policy_motion = policy.motion(model, robot.imus[0], commands=commands)
    if log is not None:
        policy_motion = log.wrap_motion(policy_motion)

    motion = staged(
        approach(target_pose, start_pose, args.approach),
        args.approach,
        policy_motion,
        hold_pose=target_pose,
    )
    if args.hz != policy_hz:
        motion = held(motion, inner_hz=policy_hz, outer_hz=args.hz)

    fallen = [False]
    lowest = [world.base_height_m]
    last = [world.base_height_m]
    tilted = [0.0]

    imu = robot.imus[0]

    started_at = [None]
    echoed = [0.0]

    def watch(t: float, observation):
        """넘어지면 멈춤. 실물의 상태 기계 자리에 해당하는 것이 시뮬에는 없음.

        정책이 돌기 전(접근·Enter 대기)에는 물리를 멈춰 둠. `--duration` 도 그
        구간은 안 셈 -- 사람이 Enter 를 늦게 누른 만큼 정책 시간이 줄면 안 됨.
        """
        running = motion.is_started() and t >= args.approach
        world.frozen = not running
        if running and started_at[0] is None:
            started_at[0] = t
            echoed[0] = t - args.echo
        command = motion(t, observation)
        if not running:
            return command

        elapsed = t - started_at[0]
        height = world.base_height_m
        lowest[0] = min(lowest[0], height)
        last[0] = height
        gravity = imu.read().gravity
        tilt = (gravity[0] ** 2 + gravity[1] ** 2) ** 0.5
        tilted[0] = max(tilted[0], tilt)

        if args.ghost:
            # 창에 명령 자세를 겹침. 창이 없으면 아무 일도 안 함.
            world.ghost(default_qpos(command))
        if args.echo > 0.0 and t - echoed[0] >= args.echo:
            echoed[0] = t
            print(_echo(elapsed, height, tilt, command, observation, policy.order))

        if (height < FALL_HEIGHT_M or tilt > FALL_TILT) and not fallen[0]:
            fallen[0] = True
            print(f"  ** 넘어짐: 몸통 {height:.3f} m, 기울기 {tilt:.2f}, "
                  f"정책 {elapsed:.2f}초 **")
            loop.stop()
        elif elapsed >= args.duration:
            loop.stop()
        return command

    wall = time.perf_counter()
    world.frozen = True
    try:
        # 실물과 같은 시작 방식임 -- Enter 를 누르면 `staged` 가 정책으로 넘어감.
        # 화면이 아니면(파이프, 시험) `EnterWatcher` 가 바로 시작시킴.
        with EnterWatcher(motion, loop) if not args.now else _now(motion):
            stats = loop.run(watch)
    finally:
        robot.disconnect()
        world.close()
        if log is not None:
            _save_log(log, args, policy.name, *log_naming)

    print("\n".join([
        "",
        f"  시뮬 시간 {world.sim_time_s:.2f}초  물리 스텝 {world.steps}"
        f"  제어 주기 {stats.cycles}"
        f"  벽시계 {time.perf_counter() - wall:.1f}초",
        f"  몸통 높이 끝 {last[0]:.3f} m  최저 {lowest[0]:.3f} m"
        f"   (정지 감쇠 뒤 {world.base_height_m:.3f} m -- 토크를 끊으면 주저앉음)",
        f"  기울기 최대 {tilted[0]:.2f}",
        f"  주기 {stats.mean_hz:.1f}Hz" + ("" if stats.kept_up else "  (주기를 못 지킴)"),
        "  넘어짐" if fallen[0] else "  서 있음",
        "",
    ]))
    if args.view:
        # MuJoCo 창을 띄웠으면 **인터프리터 종료를 건너뛰고 바로 나감.**
        #
        # mujoco 3.17 + glfw 2.11 조합에서 뷰어를 닫은 뒤 프로세스가 끝날 때
        # glfw 가 죽음 (`posix_thread.c:70 _glfwPlatformSetTls` assertion, 코어 덤프).
        # 우리 코드와 무관함 -- 창을 열고 닫기만 하는 10줄짜리 예제도 같이 죽고,
        # `with launch_passive(...)` 로 바꿔도 같음.
        #
        # 롤아웃·기록·요약은 이 줄 전에 다 끝나 있음. 뒤에 남은 것은 파이썬이
        # 객체를 정리하는 일뿐이라 건너뛰어도 잃는 것이 없음.
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(1 if fallen[0] else 0)
    return 1 if fallen[0] else 0


if __name__ == "__main__":
    sys.exit(main())
