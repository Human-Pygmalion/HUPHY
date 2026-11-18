"""`robot.yaml` 에서 **시뮬 버스를 쓰는 실물 로봇**을 세움.

`scripts/bringup.py` 의 `build_leg`/`build_biped` 와 같은 일을 하되 버스만 바꿈.
그쪽을 고쳐서 분기를 넣지 않는 이유: 실물 띄우는 경로에 시뮬 분기가 섞이면,
하드웨어를 띄울 때 지나는 코드가 늘어남.

여기서 만드는 `Leg`·`Biped`·`AnkleKinematics` 는 **실물과 같은 클래스**임.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Dict, Mapping, Optional, Tuple

from huphy import calibration as calib
from huphy.config import load_robot
from huphy.config.schema import LimbConfig, RobotConfig
from huphy.kinematics import mjcf_loop
from huphy.kinematics.ankle import AnkleGeometry, AnkleKinematics
from huphy.motors.base import Gains
from huphy.robots.biped import Biped
from huphy.robots.leg import Leg

from . import actuator
from .bus import SimBus
from .imu import SimImu
from .world import (
    DEPENDENT_JOINT,
    MOTOR_JOINT,
    MOTOR_SPEC,
    SIDE_PREFIX,
    SimWorld,
    mjcf_joint,
)

DEFAULT_CONFIG = Path(__file__).resolve().parents[2] / "config" / "robot.yaml"


def effort_limits(robot: RobotConfig) -> Dict[str, float]:
    """MJCF 관절 이름 -> 토크 상한 (Nm). 모델의 인코딩 상한을 씀."""
    from huphy.motors.robstride import tables

    out: Dict[str, float] = {}
    for limb in robot.limbs.values():
        for name, motor in limb.motors.items():
            out[mjcf_joint(limb.name, name)] = float(
                tables.encoding_for(motor.model).tmax_nm
            )
    return out


def build_sim_leg(
    robot: RobotConfig,
    limb: LimbConfig,
    world: SimWorld,
    *,
    gains: Optional[Gains] = None,
    curves: Optional[Mapping[str, actuator.Curve]] = None,
) -> Leg:
    """다리 하나. `build_leg` 과 같은 순서로 세우고 버스만 `SimBus` 임."""
    from dataclasses import replace

    if gains is not None:
        limb = replace(
            limb,
            motors={n: replace(m, gains=gains) for n, m in limb.motors.items()},
        )

    geometry = AnkleGeometry()
    if limb.side == "left":
        geometry = geometry.mirrored()

    calibration = calib.load(limb.calibration_path)
    bus = SimBus(
        world,
        limb.motors_by_id(),
        {m.id: mjcf_joint(limb.name, n) for n, m in limb.motors.items()},
        {m.id: calibration[n] for n, m in limb.motors.items()},
        channel=f"sim:{limb.name}",
        curves=curves,
    )
    return Leg(
        limb,
        bus,
        safety=robot.safety,
        kinematics=AnkleKinematics(geometry),
        imus=[],                        # IMU 는 로봇 전체가 들고 있음
        verify_host_ids=False,          # 호스트 id 는 CAN 얘기임
    )


def build_sim_biped(
    *,
    config: Path = DEFAULT_CONFIG,
    gains: Optional[Gains] = None,
    view: bool = False,
    imu_name: Optional[str] = None,
) -> Tuple[Biped, SimWorld]:
    """양다리 로봇과 세계를 함께 세움. 세계는 두 다리가 같이 씀.

    `imu_name` 을 생략하면 `robot.yaml` 의 첫 IMU 이름을 씀 -- 기록 열 이름이
    실물과 같아야 함.
    """
    robot = load_robot(config)
    world = SimWorld(effort_limits=effort_limits(robot), view=view)
    curves = actuator.load_curves(MOTOR_SPEC)

    legs = [
        build_sim_leg(robot, limb, world, gains=gains, curves=curves)
        for limb in robot.limbs.values()
        if limb.kind == "leg"
    ]
    if not legs:
        raise ValueError(f"{config}: kind 가 leg 인 팔다리가 없음")

    name = imu_name or (next(iter(robot.imus)) if robot.imus else "main")
    imu = SimImu(world, name=name)
    biped = Biped(legs, safety=robot.safety, imus=[imu])
    return biped, world


def default_qpos(pose_deg: Mapping[str, float]) -> Dict[str, float]:
    """`{팔다리/모터: cal 도}` -> `{MJCF 관절: rad}`. 종속 관절까지 채움.

    종속 관절(막대 4개, 발목 pitch/roll)은 **그 크랭크각으로 풀어서** 넣음
    (`kinematics/mjcf_loop.solve`). 기준 자세값(`g.default`)을 그냥 쓰면 안 됨 --
    꾸러미마다 크랭크 기준각이 달라서(`squat` 은 +0.1070, 나머지는 +0.1439) 시작
    자세가 `<connect>` 구속을 어긴 상태가 되고, 첫 스텝에 솔버가 로봇을 잡아
    흔들어 1초 안에 넘어짐.

    **cal 공간 도를 MJCF 관절 rad 로 그냥 바꿈.** 지금 캘리브레이션이 sign 1.0,
    offset 0 이라 두 공간이 같음 (`config/calibration/*_v1.0.json`). 영점을 옮기면
    시뮬 초기 자세만 틀어짐 -- 제어 경로는 `MotorCalibration` 을 지나므로 영향 없음.
    """
    out: Dict[str, float] = {}
    for key, value in pose_deg.items():
        limb, _, motor = key.partition("/")
        out[mjcf_joint(limb, motor)] = math.radians(float(value))

    for limb_id, geometry in mjcf_loop.SIDES.items():
        prefix = SIDE_PREFIX[limb_id]
        crank_a = out.get(f"{prefix}_{MOTOR_JOINT['ankle_a']}")
        crank_b = out.get(f"{prefix}_{MOTOR_JOINT['ankle_b']}")
        if crank_a is None or crank_b is None:
            continue
        solved = mjcf_loop.solve(geometry, crank_a, crank_b, dict(geometry.default))
        for our_name, mjcf_name in DEPENDENT_JOINT.items():
            out[f"{prefix}_{mjcf_name}"] = float(solved[our_name])
    return out
