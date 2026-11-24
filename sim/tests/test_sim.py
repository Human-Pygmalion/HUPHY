"""시뮬 계층 시험.

**무엇을 지키려는 것인지**가 시험마다 다름.

    1  MJCF 와 우리 이름이 이어짐        틀리면 엉뚱한 관절을 움직임
    2  IMU 가 규약대로 나옴              부호가 뒤집히면 정책이 반대로 기울었다고 봄
    3  관측 열 이름이 실물과 같음        다르면 deploy.py 가 다른 벡터를 만들어 검증이 무의미해짐
    4  토크-속도 클램프                  구동/제동을 섞으면 모터가 실제보다 세짐
    4b 토크 기록이 상한과 같이 나옴      상한이 없으면 포화인지 수요인지 못 가림
    5  같은 입력이면 같은 결과           결정적이지 않으면 재현이 안 됨
    6  고리 구속이 시작부터 만족됨        어기면 첫 스텝에 로봇이 튐
    7  정책이 서 있음                    경로 전체가 이어졌는지 보는 것
"""

from __future__ import annotations

import math
from pathlib import Path

import pytest

mujoco = pytest.importorskip("mujoco", reason="pip install mujoco (또는 pip install -e .[sim])")

from huphy.control import deploy                                  # noqa: E402
from huphy.motors.robstride.bus import MitCommand                 # noqa: E402
from huphy.robots.leg import REQUIRED_MOTORS                      # noqa: E402

from huphy_sim import actuator                                    # noqa: E402
from huphy_sim.build import build_sim_biped, default_qpos         # noqa: E402
from huphy_sim.imu import SimImu                                  # noqa: E402
from huphy_sim.world import (                                     # noqa: E402
    GHOST_RGBA,
    MOTOR_JOINT,
    MOTOR_SPEC,
    SIDE_PREFIX,
    SimWorld,
    mjcf_joint,
)

REPO = Path(__file__).resolve().parents[2]
WALK = REPO / "config" / "policies" / "walk"


@pytest.fixture(scope="module")
def world():
    return SimWorld()


@pytest.fixture(scope="module")
def default_pose():
    return deploy.load(WALK).default_pose_deg


def standing(pose):
    """기준 자세에 놓고 **토크를 켠** 로봇. 실물의 `ControlLoop._enter` 가 하는 일임.

    켜지 않으면 명령을 보내도 토크가 0 이라 그냥 주저앉음 -- 시험이 조용히
    엉뚱한 것을 재게 됨.
    """
    robot, world = build_sim_biped()
    robot.connect()
    world.reset(default_qpos(pose))
    robot.enable()
    robot.refresh()
    return robot, world


class TestModel:
    def test_every_motor_has_a_joint(self, world):
        """모터 이름 여섯 개가 MJCF 관절로 다 이어짐. 발목만 이름이 다름(크랭크)."""
        assert set(MOTOR_JOINT) == set(REQUIRED_MOTORS)
        for limb in SIDE_PREFIX:
            for motor in REQUIRED_MOTORS:
                assert mjcf_joint(limb, motor) in world.joint_names

    def test_twelve_actuators(self, world):
        assert world.model.nu == 12 == len(world.joint_names)

    def test_unknown_names_are_rejected(self):
        """조용히 넘기면 그 관절만 토크가 안 들어가고 원인을 못 찾음."""
        with pytest.raises(KeyError):
            mjcf_joint("right_arm", "knee")
        with pytest.raises(KeyError):
            mjcf_joint("right_leg", "elbow")

    def test_timestep_matches_training(self, world):
        """학습 쪽 `SimulationCfg.timestep` 과 같아야 접촉이 비슷하게 풀림."""
        assert world.timestep_s == pytest.approx(0.002)


class TestImu:
    def test_level_robot_sees_gravity_down(self, world, default_pose):
        world.reset(default_qpos(default_pose))
        state = SimImu(world).read()
        assert state.gravity[2] == pytest.approx(-1.0, abs=1e-3)
        assert abs(state.gravity[0]) < 1e-2 and abs(state.gravity[1]) < 1e-2
        assert state.is_valid

    def test_still_robot_has_no_rotation(self, world, default_pose):
        world.reset(default_qpos(default_pose))
        assert max(abs(v) for v in SimImu(world).read().gyro_dps) < 1e-6

    def test_gyro_is_degrees(self, world, default_pose):
        """MJCF 는 rad/s 로 주고 `ImuState` 는 도/초임. 57배 틀리는 자리."""
        world.reset(default_qpos(default_pose))
        world.data.qvel[3:6] = [1.0, 0.0, 0.0]            # 몸통 각속도 1 rad/s
        world._mujoco.mj_forward(world.model, world.data)
        gyro = SimImu(world).read().gyro_dps
        assert max(abs(v) for v in gyro) == pytest.approx(math.degrees(1.0), rel=1e-3)


class TestObservation:
    def test_columns_are_the_real_ones(self, default_pose):
        """`Leg` 가 내는 열 이름 그대로여야 함 -- 실물 로그와 겹쳐 볼 수 있어야 함."""
        robot, world = standing(default_pose)
        try:
            observation = robot.get_observation()
        finally:
            robot.disconnect()
        assert set(observation) == set(robot.observation_features)

    def test_angles_come_back_in_cal_space(self, default_pose):
        """보낸 자세를 그대로 관측해야 함. cal<->raw 왕복이 깨지면 여기서 걸림."""
        robot, world = standing(default_pose)
        try:
            observation = robot.get_observation()
        finally:
            robot.disconnect()
        for joint, expected in default_pose.items():
            # 1도 여유: 상태를 받아 오려면 `refresh` 가 한 주기를 굴려야 하고
            # (실물도 토크 0 명령을 보내 응답을 받음), 그 20 ms 동안 중력이
            # 관절을 그만큼 움직임. 부호나 공간이 틀리면 수십 도로 벌어지므로
            # 이 여유로도 잡힘.
            assert observation[f"{joint}.pos"] == pytest.approx(expected, abs=1.0)


class TestActuator:
    @pytest.fixture(scope="class")
    def curves(self):
        return actuator.load_curves(MOTOR_SPEC)

    def test_two_models(self, curves):
        assert set(curves) == {"RS03", "RS04"}

    def test_curve_ends_at_no_load(self, curves):
        for curve in curves.values():
            assert curve[-1][1] == 0.0
            speeds = [w for w, _ in curve]
            assert speeds == sorted(speeds)

    def test_motoring_is_capped_by_the_curve(self, curves):
        curve = curves["RS03"]
        fast = curve[-2][0]
        capped = actuator.clamp_to_curve(1000.0, fast, curve)
        assert capped == pytest.approx(actuator.tn_limit(curve, fast))
        assert capped < curve[0][1]

    def test_braking_gets_the_peak(self, curves):
        """제동은 속도 제약을 안 받음 -- 발전기로 도는 쪽임."""
        curve = curves["RS03"]
        fast = curve[-2][0]
        assert actuator.clamp_to_curve(1000.0, -fast, curve) == pytest.approx(curve[0][1])

    def test_effort_limit_applies(self, curves):
        tau = actuator.torque_nm(
            kp=5000.0, kd=0.0, target_rad=1.0, target_vel_rad_s=0.0,
            pos_rad=0.0, vel_rad_s=0.0, tau_ff=0.0,
            curve=curves["RS04"], effort_limit=10.0,
        )
        assert tau == pytest.approx(10.0)


class TestTorqueDetail:
    """상한과 잘린 이유. **자른 뒤 값만 보면 포화를 알 수 없어서** 같이 냄."""

    @pytest.fixture(scope="class")
    def curves(self):
        return actuator.load_curves(MOTOR_SPEC)

    def detail(self, curves, model="RS04", effort=120.0, **kwargs):
        args = dict(kp=0.0, kd=0.0, target_rad=0.0, target_vel_rad_s=0.0,
                    pos_rad=0.0, vel_rad_s=0.0, tau_ff=0.0)
        args.update(kwargs)
        return actuator.torque_detail(curve=curves[model], effort_limit=effort, **args)

    def test_applied_is_what_torque_nm_gives(self, curves):
        """두 벌로 두면 한쪽만 고치는 일이 생김."""
        args = dict(kp=120.0, kd=2.0, target_rad=0.3, target_vel_rad_s=1.0,
                    pos_rad=0.0, vel_rad_s=-2.0, tau_ff=5.0)
        got = actuator.torque_nm(curve=curves["RS03"], effort_limit=60.0, **args)
        assert got == self.detail(curves, "RS03", 60.0, **args).applied

    def test_not_clipped_leaves_no_mark(self, curves):
        d = self.detail(curves, tau_ff=3.0)
        assert d.applied == pytest.approx(3.0) == pytest.approx(d.raw)
        assert d.source == "" and not d.clipped

    def test_the_fixed_limit_is_named(self, curves):
        d = self.detail(curves, effort=10.0, tau_ff=50.0)
        assert d.applied == pytest.approx(10.0)
        assert d.limit == pytest.approx(10.0)
        assert d.source == "effort"

    def test_the_curve_limit_is_named(self, curves):
        """빨리 도는 쪽은 고정 상한보다 곡선이 먼저 걸림."""
        fast = curves["RS04"][-2][0]
        d = self.detail(curves, vel_rad_s=fast, tau_ff=1000.0)
        assert d.limit == pytest.approx(actuator.tn_limit(curves["RS04"], fast))
        assert d.limit < 120.0
        assert d.applied == pytest.approx(d.limit)
        assert d.source == "curve"

    def test_braking_gets_the_peak(self, curves):
        """제동은 속도 제약을 안 받음. 상한이 피크로 나와야 함."""
        fast = curves["RS03"][-2][0]
        d = self.detail(curves, "RS03", 1000.0, vel_rad_s=-fast, tau_ff=1000.0)
        assert d.limit == pytest.approx(curves["RS03"][0][1])

    def test_the_limit_follows_the_direction(self, curves):
        """음의 토크는 음의 속도 쪽이 제동임. 방향을 안 보면 상한이 뒤바뀜."""
        fast = curves["RS03"][-2][0]
        pushing = self.detail(curves, "RS03", 1000.0, vel_rad_s=-fast, tau_ff=-1000.0)
        braking = self.detail(curves, "RS03", 1000.0, vel_rad_s=+fast, tau_ff=-1000.0)
        assert pushing.limit == pytest.approx(actuator.tn_limit(curves["RS03"], fast))
        assert braking.limit == pytest.approx(curves["RS03"][0][1])
        assert pushing.applied < 0 and braking.applied < 0


class TestTorqueReport:
    """버스가 주기마다 요약하는 토크. `sim/rollout.py --log` 의 `tau/...` 열이 됨."""

    def report(self, pose, commands):
        """명령 한 벌을 보내고 한 주기 굴린 뒤 (요약, 모터 id 표) 를 냄."""
        robot, _ = standing(pose)
        leg = robot.parts[0]
        ids = {name: motor.id for name, motor in leg.config.motors.items()}
        try:
            leg.bus.send_mit({ids[n]: c for n, c in commands.items()})
            leg.bus.collect()
            return leg.bus.torque_report(), ids
        finally:
            robot.disconnect()

    def test_every_motor_shows_up(self, default_pose):
        """명령을 하나만 보냈어도 열두 칸이 다 차야 함 -- 빈 칸은 뷰어에서 끊긴 선임."""
        report, ids = self.report(
            default_pose, {"knee": MitCommand(position_deg=0.0, kp=1.0)}
        )
        assert set(report) == set(ids.values())
        for value in report.values():
            assert len(value) == 3

    def test_a_motor_with_its_torque_off_reads_zero(self, default_pose):
        """힘을 뺀 모터. 지난 주기 값이 남으면 끈 뒤에도 토크가 보임."""
        robot, _ = standing(default_pose)
        leg = robot.parts[0]
        mid = leg.config.motors["hip_yaw"].id
        try:
            leg.bus.disable_torque([mid])
            leg.bus.send_mit({leg.config.motors["knee"].id: MitCommand(position_deg=0.0)})
            leg.bus.collect()
            tau, limit, sat = leg.bus.torque_report()[mid]
        finally:
            robot.disconnect()
        assert tau == 0.0 and sat == 0.0
        assert limit != limit                      # 상한은 뜻이 없어 NaN

    def test_a_saturated_motor_sits_on_its_limit(self, default_pose):
        """상한 바깥을 요구하면 토크가 상한과 같아지고 포화비가 1 이 됨."""
        report, ids = self.report(
            default_pose,
            {"knee": MitCommand(position_deg=0.0, kp=0.0, kd=0.0, torque_nm=1000.0)},
        )
        tau, limit, sat = report[ids["knee"]]
        assert sat == pytest.approx(1.0)
        assert tau == pytest.approx(limit)
        assert tau > 0.0

    def test_one_cycle_only(self, default_pose):
        """양다리가 세계 하나를 같이 씀. 남의 주기 몫이 섞이면 포화비가 반으로 묽어짐."""
        robot, _ = standing(default_pose)
        legs = robot.parts
        mid = legs[0].config.motors["knee"].id
        try:
            for leg in legs:                      # 실제 루프 순서: 둘 다 보낸 뒤 수거
                leg.bus.send_mit({
                    leg.config.motors["knee"].id:
                        MitCommand(position_deg=0.0, kp=0.0, torque_nm=1000.0),
                })
            for leg in legs:
                leg.bus.collect()
            steps = len(legs[0].bus._torque_log[mid])
        finally:
            robot.disconnect()
        assert steps == round(0.02 / 0.002)        # 주기 하나 = 물리 스텝 열 번

    def test_the_peak_step_is_kept(self, default_pose):
        """한 주기에 물리 스텝이 열 번임. 마지막 스텝만 보면 중간 포화가 안 보임."""
        robot, _ = standing(default_pose)
        leg = robot.parts[0]
        mid = leg.config.motors["knee"].id
        try:
            # 한 주기는 세게, 다음 주기는 힘을 빼고 -- 요약이 주기마다 새로 나와야 함
            leg.bus.send_mit({mid: MitCommand(position_deg=0.0, kp=0.0, torque_nm=50.0)})
            leg.bus.collect()
            strong = leg.bus.torque_report()[mid]
            leg.bus.send_mit({mid: MitCommand(position_deg=0.0, kp=0.0, torque_nm=1.0)})
            leg.bus.collect()
            weak = leg.bus.torque_report()[mid]
        finally:
            robot.disconnect()
        assert strong[0] == pytest.approx(50.0)
        assert weak[0] == pytest.approx(1.0)


class TestLoop:
    def test_initial_pose_satisfies_the_linkage(self, default_pose):
        """`<connect>` 를 어긴 자세로 시작하면 첫 스텝에 로봇이 튐."""
        robot, world = standing(default_pose)
        before = world.base_height_m
        try:
            for _ in range(5):
                robot.send_action(default_pose)
        finally:
            robot.disconnect()
        assert abs(world.base_height_m - before) < 0.02

    def test_same_input_gives_the_same_run(self, default_pose):
        """결정적이어야 두 번 돌려 비교할 수 있음."""
        runs = []
        for _ in range(2):
            robot, world = standing(default_pose)
            try:
                for _ in range(10):
                    robot.send_action(default_pose)
                runs.append(robot.get_observation())
            finally:
                robot.disconnect()
        for key, value in runs[0].items():
            assert runs[1][key] == value or (value != value)      # NaN 은 비교 안 됨


class TestFrozen:
    """Enter 를 기다리는 동안 물리를 멈추는 것. `rollout.py` 가 그 구간에 켬."""

    def test_frozen_world_does_not_move(self, default_pose):
        robot, world = standing(default_pose)
        world.frozen = True
        before = world.base_height_m
        time_before = world.sim_time_s
        try:
            for _ in range(100):            # 2초어치 주기
                robot.send_action(default_pose)
        finally:
            robot.disconnect()
        assert world.sim_time_s == time_before
        assert world.base_height_m == before

    def test_unfreezing_resumes(self, default_pose):
        robot, world = standing(default_pose)
        world.frozen = True
        try:
            robot.send_action(default_pose)
            world.frozen = False
            robot.send_action(default_pose)
        finally:
            robot.disconnect()
        assert world.sim_time_s > 0.0


class TestGhost:
    """명령 자세를 창에 겹쳐 그리는 것. 창이 없으면 아무 일도 없어야 함."""

    def test_no_viewer_draws_nothing(self, world, default_pose):
        assert world.viewer is None
        assert world.ghost(default_qpos(default_pose)) == 0

    def test_draws_the_commanded_pose_without_touching_physics(self, default_pose):
        import mujoco

        robot, world = standing(default_pose)

        class Window:
            def __init__(self, model):
                self.user_scn = mujoco.MjvScene(model, 2000)

            def sync(self):
                pass

        try:
            world.viewer = Window(world.model)
            knee = world.qpos_adr(mjcf_joint("right_leg", "knee"))
            before = float(world.data.qpos[knee])
            moved = {k: v + 10.0 for k, v in default_pose.items()}
            count = world.ghost(default_qpos(moved))
            assert count > 0
            assert float(world.data.qpos[knee]) == before
            geom = world.viewer.user_scn.geoms[0]
            assert geom.rgba[3] == pytest.approx(GHOST_RGBA[3])
            assert int(geom.category) == int(mujoco.mjtCatBit.mjCAT_DECOR)
        finally:
            world.viewer = None
            robot.disconnect()


@pytest.mark.slow
class TestPolicy:
    def test_walk_policy_stands(self, default_pose):
        """정책을 실제로 돌려 서 있는지. 경로 어딘가가 끊기면 2초 안에 기움."""
        policy = deploy.load(WALK)
        model = policy.load_model()
        robot, world = standing(default_pose)
        imu = robot.imus[0]
        motion = policy.motion(
            model, imu, commands=deploy.Commands(policy.commands.values(), {})
        )
        start = getattr(motion, "start", None)
        if callable(start):
            start()                    # 안 부르면 정책이 안 돌고 기준 자세만 나감
        try:
            tilt = 0.0
            t = 0.0
            for _ in range(100):                      # 2초
                robot.send_action(motion(t, robot.get_observation()))
                gravity = imu.read().gravity
                tilt = max(tilt, math.hypot(gravity[0], gravity[1]))
                t += 0.02
        finally:
            robot.disconnect()
        assert world.base_height_m > 0.8
        assert tilt < 0.3
