"""MuJoCo 세계 하나. 양다리 로봇과 바닥을 들고 있고, 20 ms 씩 굴림.

MJCF 는 `sim/assets/pygmalion_legonly_ab.xml` 임 -- 학습 레포
(`HuphyChan-dev/mjlab_policy/pygmalion_mjlab/assets/`) 사본이고 **한 글자도
고치지 않음.** 원본이 바뀌면 `diff` 로 바로 알 수 있어야 함.

## MJCF 에 없어서 여기서 붙이는 것

    액추에이터   관절 12개에 토크 액추에이터. 학습 쪽은 mjlab 이 설정으로 붙임
    솔버 설정    iterations 100 / ls 50 / ccd 50. 학습 쪽 SimulationCfg 와 같은 값

timestep 은 MuJoCo 기본값 0.002 를 그대로 씀 -- 학습 쪽 `SimulationCfg.timestep`
과 같음. 정책 주기 20 ms 는 이것의 10 배임.

armature·frictionloss·damping 은 **MJCF 안에 이미 있음** (관절마다 적혀 있음).
여기서 다시 넣지 않음.

## 관절이 25개인데 모터는 12개

    모터 달린 것   hip_pitch, hip_roll, hip_yaw, knee, crank_A, crank_B   (다리당 6)
    종속된 것      rod_A_u1/u2, rod_B_u1/u2, ankle_pitch, ankle_roll      (다리당 6)

종속 관절은 `<equality><connect>` 로 막대 끝이 발의 볼에 묶여 결정됨. **그 값은
읽어 내보내지 않음** -- 실물에는 그 센서가 없음. 꺼내 쓰면 실물이 지나지 않는 길을
검증하게 됨. 정책에 들어가는 24칸은 `kinematics/mjcf_loop.py` 가 크랭크각에서
푼 값임 (`control/deploy.py`).
"""

from __future__ import annotations

from pathlib import Path
from typing import Dict, List, Mapping, Optional, Tuple

ASSETS = Path(__file__).resolve().parent.parent / "assets"
MJCF = ASSETS / "pygmalion_legonly_ab.xml"
MOTOR_SPEC = ASSETS / "motor_spec"

SOLVER_ITERATIONS = 100
SOLVER_LS_ITERATIONS = 50
SOLVER_CCD_ITERATIONS = 50
"""학습 쪽 `mjlab/sim/sim.py` 의 `SimulationCfg` 기본값."""

SIDE_PREFIX = {"right_leg": "R", "left_leg": "L"}
"""팔다리 이름 -> MJCF 이름 앞머리. `robot.yaml` 의 `limbs` 키가 그대로 옴."""

MOTOR_JOINT = {
    "hip_pitch": "hip_pitch_joint",
    "hip_roll": "hip_roll_joint",
    "hip_yaw": "hip_yaw_joint",
    "knee": "knee_joint",
    "ankle_a": "crank_A_joint",
    "ankle_b": "crank_B_joint",
}
"""우리 모터 이름 -> MJCF 관절 이름. **발목만 이름이 다름** (크랭크임)."""

DEPENDENT_JOINT = {
    "rod_a_u1": "rod_A_u1",
    "rod_a_u2": "rod_A_u2",
    "rod_b_u1": "rod_B_u1",
    "rod_b_u2": "rod_B_u2",
    "ankle_pitch": "ankle_pitch_joint",
    "ankle_roll": "ankle_roll_joint",
}
"""`kinematics/mjcf_loop.DEPENDENT` 의 키 -> MJCF 관절 이름. 초기 자세를 놓을 때만 씀."""

IMU_GYRO = "imu_ang_vel"
IMU_ACCEL = "imu_lin_acc"
IMU_UPVECTOR = "imu_upvector"
"""MJCF `<sensor>` 이름. `imu_in_base` site 에 붙어 있음."""

DEFAULT_CYCLE_S = 0.02
"""제어 주기 기본값. 정책 계약의 `control_rate_hz` 가 50 Hz 임."""

GROUND_CLEARANCE_M = 0.001
"""초기 자세에서 가장 낮은 점을 바닥에서 이만큼 띄움.

0 으로 두면 첫 스텝에 접촉이 이미 관통 상태로 시작해 발이 튐.
"""

FLOOR_NAME = "floor"
PLACE_BISECTIONS = 40
"""초기 높이를 접촉이 생기는 지점까지 이분법으로 찾는 횟수. 2 m / 2^40 까지 좁힘."""

FREE_HEIGHT_M = 2.0
"""이분법의 위쪽 끝. 이 높이에서는 바닥에 안 닿는다는 전제임."""

GHOST_RGBA = (0.25, 0.6, 1.0, 0.35)
"""고스트 색. 반투명이라 실제 로봇이 뒤에 비침."""

GHOST_COPY = (
    "type", "dataid", "objtype", "objid", "matid", "texid", "texcoord",
    "texrepeat", "texuniform", "emission", "specular", "shininess",
    "reflectance", "modelrbound", "transparent",
)
"""고스트 geom 에 그대로 옮기는 칸들. 색(`rgba`)·분류(`category`)·`segid` 만 덮어씀."""

SUNK_HEIGHT_M = -1.0
"""이분법의 아래쪽 끝. 이 높이에서는 반드시 바닥을 파고든다는 전제임."""


def mjcf_joint(limb_id: str, motor_name: str) -> str:
    """팔다리 이름과 모터 이름으로 MJCF 관절 이름을 만듦."""
    try:
        prefix = SIDE_PREFIX[limb_id]
    except KeyError:
        raise KeyError(
            f"{limb_id!r} 는 MJCF 에 없는 팔다리임 (가용: {sorted(SIDE_PREFIX)})"
        ) from None
    try:
        return f"{prefix}_{MOTOR_JOINT[motor_name]}"
    except KeyError:
        raise KeyError(
            f"{motor_name!r} 는 MJCF 에 이어 붙일 모터가 아님 (가용: {sorted(MOTOR_JOINT)})"
        ) from None


class SimWorld:
    """MuJoCo 모델·데이터 한 벌. 다리들이 이것을 같이 씀.

    양다리가 **한 MJCF 안에** 있어서 세계가 하나임. 다리마다 세계를 따로 두면 두
    로봇이 각자 바닥에 서 있는 것이 되어 균형이 뜻을 잃음.
    """

    def __init__(
        self,
        *,
        mjcf: Path = MJCF,
        effort_limits: Optional[Mapping[str, float]] = None,
        view: bool = False,
    ) -> None:
        import mujoco                                   # 선택 의존성. 없으면 여기서 터짐

        self._mujoco = mujoco
        self.mjcf = Path(mjcf)
        spec = mujoco.MjSpec.from_file(str(self.mjcf))

        self.joint_names: Tuple[str, ...] = tuple(
            mjcf_joint(limb, motor) for limb in SIDE_PREFIX for motor in MOTOR_JOINT
        )
        limits = dict(effort_limits or {})
        for name in self.joint_names:
            act = spec.add_actuator()
            act.name = f"{name}_torque"
            act.target = name
            act.trntype = mujoco.mjtTrn.mjTRN_JOINT
            act.set_to_motor()
            limit = float(limits.get(name, 0.0))
            if limit > 0.0:
                act.ctrlrange = [-limit, limit]
                act.ctrllimited = 1

        floor = spec.worldbody.add_geom()
        floor.name = FLOOR_NAME
        floor.type = mujoco.mjtGeom.mjGEOM_PLANE
        floor.size = [0.0, 0.0, 0.05]
        floor.condim = 3
        # 마찰은 MuJoCo 기본값(1.0)을 씀. MJCF 의 발 geom 도 기본값이라 쌍의 마찰이
        # 1.0 임. 학습 쪽은 발 마찰을 0.3~1.2 로 흔들지만(DR) 여기서는 안 흔듦.

        self.model = spec.compile()
        self.model.opt.iterations = SOLVER_ITERATIONS
        self.model.opt.ls_iterations = SOLVER_LS_ITERATIONS
        self.model.opt.ccd_iterations = SOLVER_CCD_ITERATIONS
        self.data = mujoco.MjData(self.model)

        self._qpos: Dict[str, int] = {}
        self._dof: Dict[str, int] = {}
        for name in self.joint_names:
            jid = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_JOINT, name
            )
            if jid < 0:
                raise KeyError(f"MJCF 에 관절 {name!r} 이 없음 ({self.mjcf})")
            self._qpos[name] = int(self.model.jnt_qposadr[jid])
            self._dof[name] = int(self.model.jnt_dofadr[jid])

        self._ctrl: Dict[str, int] = {
            name: int(
                mujoco.mj_name2id(
                    self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{name}_torque"
                )
            )
            for name in self.joint_names
        }
        self._floor_id = int(
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, FLOOR_NAME)
        )
        self._sensor = {
            key: self._sensor_slice(key)
            for key in (IMU_GYRO, IMU_ACCEL, IMU_UPVECTOR)
        }

        self.steps = 0
        self.cycle_s = DEFAULT_CYCLE_S
        """제어 한 주기의 길이. `rollout.py` 가 루프 주기로 맞춰 둠.

        여기가 루프 주기와 다르면 시뮬 시간과 제어 시간이 어긋나 속도·중력이
        뒤틀림 -- 20 ms 마다 10 ms 만 굴리면 로봇이 두 배 느린 세계에 있게 됨.
        """
        self.frozen = False
        """참이면 `advance` 가 물리를 굴리지 않음. **창만 다시 그림.**

        실물은 기준 자세로 옮긴 뒤 사람이 Enter 를 누를 때까지 그 자세로 기다림.
        시뮬에서 그 구간에 물리를 굴리면 1.1초쯤에 발끝을 축으로 앞으로 기욺 --
        기준 자세는 가만히 못 버팀(`sim/README.md` 5절). 실물에서는 그동안 사람이
        로봇을 붙잡고 있음. 그 자리에 해당하는 것이 이 멈춤임.
        """
        self.pending = False
        """이번 주기에 명령이 들어왔는지. 버스가 세움, `advance_cycle` 이 내림."""
        self.drivers: List["object"] = []
        self._ghost_data = None
        self._ghost_scene = None
        self._ghost_rgba = None
        self._jnt_adr: Dict[str, int] = dict(self._qpos)
        self.viewer = None
        if view:
            import mujoco.viewer

            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)

    # ---- 구조 -------------------------------------------------------------
    def _sensor_slice(self, name: str) -> slice:
        mujoco = self._mujoco
        sid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, name)
        if sid < 0:
            raise KeyError(f"MJCF 에 센서 {name!r} 이 없음 ({self.mjcf})")
        start = int(self.model.sensor_adr[sid])
        return slice(start, start + int(self.model.sensor_dim[sid]))

    @property
    def timestep_s(self) -> float:
        return float(self.model.opt.timestep)

    @property
    def sim_time_s(self) -> float:
        return float(self.data.time)

    # ---- 자세 -------------------------------------------------------------
    def reset(
        self,
        joint_rad: Mapping[str, float],
        *,
        base_quat: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0),
    ) -> None:
        """자세를 놓고 발이 바닥에 닿도록 몸통 높이를 맞춤. 속도는 0.

        `joint_rad` 는 MJCF 관절 이름 -> 각도(rad). 모터 관절과 종속 관절을 같이
        줘야 함 -- 종속 관절을 빼면 막대가 0 자세에서 시작해 `<connect>` 가
        첫 스텝에 로봇을 잡아 흔듦.
        """
        mujoco = self._mujoco
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[3:7] = base_quat
        for name, value in joint_rad.items():
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid < 0:
                raise KeyError(f"MJCF 에 관절 {name!r} 이 없음")
            self.data.qpos[int(self.model.jnt_qposadr[jid])] = float(value)
        self.data.qvel[:] = 0.0
        self.data.ctrl[:] = 0.0
        mujoco.mj_forward(self.model, self.data)

        self._place_on_ground()
        self.steps = 0

    def _place_on_ground(self) -> None:
        """접촉이 막 생기는 높이를 이분법으로 찾아 그보다 조금 위에 둠.

        경계 상자로 재지 않는 이유: 발 밑면이 메시가 아니라 캡슐·박스라 구 경계로는
        실제 접촉점보다 훨씬 아래로 잡혀 로봇이 공중에 뜸 (처음 1.06 m 로 떠서
        그대로 낙하했음).
        """
        mujoco = self._mujoco
        high, low = FREE_HEIGHT_M, SUNK_HEIGHT_M
        if self._contacts(high):
            raise RuntimeError(f"{high} m 에서 이미 바닥에 닿음 -- 자세를 볼 것")
        if self._contacts(low) == 0:
            raise RuntimeError(
                f"{low} m 로 내려도 바닥에 닿지 않음 -- 바닥 geom 이 없는지 볼 것"
            )
        for _ in range(PLACE_BISECTIONS):
            mid = 0.5 * (high + low)
            if self._contacts(mid):
                low = mid
            else:
                high = mid
        self.data.qpos[2] = high + GROUND_CLEARANCE_M
        mujoco.mj_forward(self.model, self.data)

    def _contacts(self, height: float) -> int:
        """그 높이에서 **바닥과** 닿는 접촉 수.

        `data.ncon` 을 그냥 쓰면 안 됨 -- 로봇 자기들끼리의 접촉이 늘 섞여 있어서
        어느 높이에서나 0 이 아니고, 이분법이 0 m 로 수렴함.
        """
        self.data.qpos[2] = float(height)
        self._mujoco.mj_forward(self.model, self.data)
        floor = self._floor_id
        return sum(
            1
            for i in range(int(self.data.ncon))
            if floor in (self.data.contact[i].geom1, self.data.contact[i].geom2)
        )

    # ---- 읽기 -------------------------------------------------------------
    def joint_pos_rad(self, name: str) -> float:
        return float(self.data.qpos[self._qpos[name]])

    def joint_vel_rad_s(self, name: str) -> float:
        return float(self.data.qvel[self._dof[name]])

    def sensor(self, name: str) -> Tuple[float, float, float]:
        values = self.data.sensordata[self._sensor[name]]
        return (float(values[0]), float(values[1]), float(values[2]))

    @property
    def base_height_m(self) -> float:
        return float(self.data.qpos[2])

    # ---- 쓰기 -------------------------------------------------------------
    def set_torque(self, name: str, tau_nm: float) -> None:
        """다음 스텝에 쓸 토크. `advance` 를 부를 때까지 반영되지 않음."""
        self.data.ctrl[self._ctrl[name]] = float(tau_nm)

    def register(self, driver: "object") -> None:
        """토크를 넣는 쪽(버스)을 등록함. 스텝마다 `apply_torque()` 가 불림."""
        self.drivers.append(driver)

    def advance_cycle(self) -> int:
        """제어 한 주기만큼 굴림. 등록된 버스들이 스텝마다 토크를 다시 넣음."""
        steps = self.advance(self.cycle_s, on_substep=self._apply_all)
        self.pending = False
        return steps

    def _apply_all(self) -> None:
        for driver in self.drivers:
            driver.apply_torque()

    def torque_nm(self, name: str) -> float:
        """그 관절에 마지막으로 넣은 토크 (Nm). 실물 상태 프레임의 토크 자리임."""
        return float(self.data.ctrl[self._ctrl[name]])

    def advance(self, seconds: float, *, on_substep=None) -> int:
        """`seconds` 만큼 굴림. 진행한 스텝 수를 반환함.

        `on_substep` 을 주면 스텝마다 부름 -- 토크를 매 스텝 다시 계산하려는
        것임. 실물 모터 펌웨어도 명령 하나를 받아 자기 주기로 PD 를 다시 푸므로,
        20 ms 동안 토크를 고정하는 것보다 이쪽이 가까움.
        """
        if self.frozen:
            self.sync()
            return 0
        count = int(round(float(seconds) / self.timestep_s))
        for _ in range(count):
            if on_substep is not None:
                on_substep()
            self._mujoco.mj_step(self.model, self.data)
            self.steps += 1
        self.sync()
        return count

    # ---- 창 ---------------------------------------------------------------
    def sync(self) -> None:
        if self.viewer is not None:
            self.viewer.sync()

    def qpos_adr(self, name: str) -> int:
        """관절 이름 -> qpos 자리. 모터·종속 관절 모두 됨."""
        try:
            return self._jnt_adr[name]
        except KeyError:
            pass
        mujoco = self._mujoco
        jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid < 0:
            raise KeyError(f"MJCF 에 관절 {name!r} 이 없음 ({self.mjcf})")
        adr = int(self.model.jnt_qposadr[jid])
        self._jnt_adr[name] = adr
        return adr

    def ghost(self, joint_rad: Mapping[str, float]) -> int:
        """**명령받은 자세**를 반투명 로봇으로 창에 겹쳐 그림. 그린 geom 수를 냄.

        정책이 낸 목표각을 그 자리에 그리는 것임 -- 실제 관절은 그 목표를 PD 로
        따라가므로 둘 사이의 벌어짐이 곧 추종 오차임. 창이 없으면 아무 일도 안 함.

        몸통 위치·자세는 **실측 그대로** 씀. 명령에는 몸통 얘기가 없음.

        물리에 영향이 없음 -- 별도 `MjData` 에 자세를 넣고 순기구학만 풀어
        뷰어의 `user_scn` 에 장식 geom 으로 넣음.

        **장면을 손으로 만들지 않고 `mjv_updateScene` 에 맡김.** `mjv_initGeom` 에
        모델 값을 그대로 넘기면 모양이 깨짐 -- 장면 geom 의 `size` 규약이 모델의
        `geom_size` 와 달라서임 (원기둥은 모델이 `[반지름, 반길이]`, 장면이
        `[반지름, 반지름, 반길이]`). MuJoCo 가 쓰는 변환을 그대로 쓰고 색과 분류만
        덮어씀. 뷰어의 `opt` 를 같이 넘기므로 **뷰어에서 켠 것만 고스트로도 보임.**
        """
        if self.viewer is None:
            return 0
        import numpy as np

        mujoco = self._mujoco
        if self._ghost_data is None:
            self._ghost_data = mujoco.MjData(self.model)
            self._ghost_scene = mujoco.MjvScene(self.model, self.model.ngeom + 16)
            self._ghost_rgba = np.array(GHOST_RGBA, dtype=np.float32)

        gd = self._ghost_data
        gd.qpos[:] = self.data.qpos
        for name, value in joint_rad.items():
            gd.qpos[self.qpos_adr(name)] = float(value)
        mujoco.mj_kinematics(self.model, gd)

        ghost = self._ghost_scene
        mujoco.mjv_updateScene(
            self.model, gd,
            getattr(self.viewer, "opt", None) or mujoco.MjvOption(),
            None,
            getattr(self.viewer, "cam", None) or mujoco.MjvCamera(),
            # 움직이는 몸통의 geom 만. 바닥은 static 이라 안 들어옴.
            mujoco.mjtCatBit.mjCAT_DYNAMIC,
            ghost,
        )

        scene = self.viewer.user_scn
        scene.ngeom = 0
        for k in range(int(ghost.ngeom)):
            if scene.ngeom >= scene.maxgeom:
                break
            src, dst = ghost.geoms[k], scene.geoms[scene.ngeom]
            for field in GHOST_COPY:
                setattr(dst, field, getattr(src, field))
            dst.size[:] = src.size
            dst.pos[:] = src.pos
            dst.mat[:] = src.mat
            dst.rgba[:] = self._ghost_rgba
            dst.category = int(mujoco.mjtCatBit.mjCAT_DECOR)
            dst.segid = scene.ngeom
            dst.label = b""
            scene.ngeom += 1
        return int(scene.ngeom)

    def close(self) -> None:
        if self.viewer is not None:
            self.viewer.close()
            self.viewer = None
