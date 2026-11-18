"""시뮬 계층 시험.

**무엇을 지키려는 것인지**가 시험마다 다름.

    1  MJCF 와 우리 이름이 이어짐        틀리면 엉뚱한 관절을 움직임
    2  IMU 가 규약대로 나옴              부호가 뒤집히면 정책이 반대로 기울었다고 봄
    3  관측 열 이름이 실물과 같음        다르면 deploy.py 가 다른 벡터를 만들어 검증이 무의미해짐
    4  토크-속도 클램프                  구동/제동을 섞으면 모터가 실제보다 세짐
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
from huphy.robots.leg import REQUIRED_MOTORS                      # noqa: E402

from huphy_sim import actuator                                    # noqa: E402
from huphy_sim.build import build_sim_biped, default_qpos         # noqa: E402
from huphy_sim.imu import SimImu                                  # noqa: E402
from huphy_sim.world import (                                     # noqa: E402
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
