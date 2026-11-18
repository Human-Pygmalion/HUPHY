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
옮길 거리가 없음. 그런데 이 구간에는 정책이 안 돌고 기준 자세를 정지 명령으로
붙잡는데, 그 자세는 가만히 못 버팀 -- 1.1초쯤에 발끝을 축으로 앞으로 기욺
(`sim/README.md` 5절). 그래서 실물 기본값(2초)을 그대로 쓰면 **정책이 시작되기도
전에 넘어짐.**

실물에서 2초인 이유는 사람이 로봇에서 손을 떼고 자리를 잡을 시간이기 때문임.
시뮬에는 그럴 사람이 없음.
"""
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
                        "그 뒤 정책이 시작됨. **1초를 넘기면 정책이 시작되기 전에 "
                        "넘어짐** -- 기준 자세는 가만히 못 버팀")
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

    # 실물은 사람이 Enter 를 눌러 정책을 시작함 (`scripts/run.py` 의 EnterWatcher).
    # 시뮬에는 누를 사람이 없으므로 바로 시작시킴 -- 안 부르면 `staged` 가 기준
    # 자세만 계속 보냄. 그 자세를 가만히 붙잡는 것은 **넘어짐** -- 서 있게 하는
    # 것이 정책임 (sim/README.md "기준 자세는 가만히 못 버팀").
    motion.start()

    fallen = [False]
    lowest = [world.base_height_m]
    last = [world.base_height_m]
    tilted = [0.0]

    imu = robot.imus[0]

    def watch(t: float, observation):
        """넘어지면 멈춤. 실물의 상태 기계 자리에 해당하는 것이 시뮬에는 없음."""
        height = world.base_height_m
        lowest[0] = min(lowest[0], height)
        last[0] = height
        gravity = imu.read().gravity
        tilt = (gravity[0] ** 2 + gravity[1] ** 2) ** 0.5
        tilted[0] = max(tilted[0], tilt)
        if (height < FALL_HEIGHT_M or tilt > FALL_TILT) and not fallen[0]:
            fallen[0] = True
            print(f"  ** 넘어짐: 몸통 {height:.3f} m, 기울기 {tilt:.2f}, {t:.2f}초 **")
            if t < args.approach:
                # 정책이 아직 시작도 안 했음. 넘어진 것은 정책이 아니라 정지 자세임.
                print(f"     정책이 시작되기 전임 (--approach {args.approach:g}초 구간). "
                      f"기준 자세는 가만히 못 버팀 -- --approach 를 줄일 것")
            loop.stop()
        return motion(t, observation)

    wall = time.perf_counter()
    try:
        stats = loop.run(watch, duration_s=args.duration)
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
    return 1 if fallen[0] else 0


if __name__ == "__main__":
    sys.exit(main())
