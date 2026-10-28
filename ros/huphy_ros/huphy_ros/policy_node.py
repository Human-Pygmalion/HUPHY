"""정책 노드 — 작업 기계에서 돎. 어디로 갈지 정함.

    ros2 run huphy_ros policy_node --ros-args -p policy:=config/policies/generalized_walk
    ros2 run huphy_ros policy_node --ros-args -p policy:=config/policies/huphy_squat

정책은 꾸러미 폴더 하나임 (`huphy-run --policy` 와 같음). 관절 순서, 기본 자세, 관찰
구성, 주기, 명령은 그 안의 `deploy_contract.json` 이 정하고 `control/deploy.py` 가 읽음.

## CAN 을 모름

모터가 몇 개인지, id 가 뭔지, 발목이 크랭크인지 모름. 받는 것은 관절 이름이 붙은
각도뿐이고 내는 것도 같은 모양임.

## 20ms 타이머가 추론을 촉발함

메시지 도착이 아니라 타이머임 (`ros/README.md` 결정 6). 도착으로 하면 **네트워크
지터가 그대로 추론 간격의 지터**가 됨 -- 정책은 20ms 간격을 전제로 학습했고,
관찰의 관절 이력 `q(t-1)` 이 정확히 한 주기 전 값이어야 함 (계약의
`motor_encoder_rel.history_frames`, `control_rate_hz`).

    타이머 20ms
      마지막 joint_states 나이 < 60ms  ->  추론하고 목표각을 냄
      나이 >= 60ms                     ->  아무것도 안 냄

안 내면 모션보드가 100ms 뒤에 감쇠로 들어감.

## 네 단계 -- `huphy-run` 과 같은 순서

    꺼짐      아무것도 안 냄
    접근      지금 자세에서 기준 자세로 approach_s 초 동안 보간한 목표각
    대기      기준 자세를 계속 냄. 사람이 로봇에서 손을 떼고 자리를 잡는 시간
    정책      신경망 출력

    /huphy/approach       꺼짐 -> 접근. approach_s 가 지나면 저절로 대기
    /huphy/start_policy   대기 -> 정책. 대기가 아니면 거절 (= huphy-run 의 Enter)
    /huphy/stop_policy    어디서든 꺼짐

기준 자세로 가는 것과 정책을 켜는 것을 나눈 이유: 예전에는 start_policy 하나가 접근이
끝나자마자 정책으로 넘어가서, 기준 자세에서 로봇이 제대로 섰는지 보고 시작할 틈이 없었음.

접근을 여기서 하는 이유 (`ros/README.md` 결정 3): 기준 자세(계약의 행동 오프셋)를
아는 쪽이 여기뿐임. 모션보드에도 두면 같은 값이 두 군데가 됨.

## 명령 토픽

    /huphy/cmd_vel   Twist     linear.x -> vx, linear.y -> vy, angular.z -> wz
    /huphy/h_star    Float64   -> h_star

정책에 있는 명령 칸만 씀 (walk 는 vx·wz, squat 는 h_star). 없는 칸은 버림. 안 오면
계약의 시작값임. 값은 `deploy.Runner` 의 `commands` 에 넣고, 이력·직전 행동 보관도
`Runner` 가 함 -- `huphy-run` 과 같은 코드임.

**속도 명령은 `cmd_timeout_s`(기본 0.5초) 넘게 안 오면 0 으로 돌림** -- 조작하던 쪽
(teleop 창, `ros2 topic pub -r`)이 죽거나 끊겼는데 마지막 속도로 계속 걸어가면 안 됨.
`huphy-run --teleop` 은 0.2초에 같은 일을 함. 높이(`h_star`)는 마지막 값을 유지함.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist, Vector3Stamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64
from std_srvs.srv import Trigger

from huphy.control import deploy
from huphy.scripts.selftest import approach
from huphy.sensors.base import ImuState

from . import convert, logbridge
from .freshness import POLICY_STATE_STALE_S, Freshness, Latest

STATUS_HZ = 5.0

VELOCITY_COMMANDS = ("vx", "vy", "wz")
"""끊기면 0 으로 돌리는 명령. `cmd_vel` 로 오는 것."""

OFF, APPROACH, READY, RUNNING = "off", "approach", "ready", "running"
"""네 단계. `policy_status` 의 `stage` 에 그대로 나감."""


class PolicyNode(Node):
    def __init__(self) -> None:
        super().__init__("huphy_policy")
        # 라이브러리(huphy) 로그도 이 노드 이름으로 ROS 로그에 나가게 함.
        # 라이브러리를 만들기 전에 붙여야 시작 메시지가 안 빠짐.
        self.log_handler = logbridge.forward(self.get_logger())

        self.declare_parameter("policy", "")
        self.declare_parameter("checkpoint", "")
        self.declare_parameter("hz", 0.0)
        self.declare_parameter("state_stale_ms", POLICY_STATE_STALE_S * 1000.0)
        self.declare_parameter("approach_s", 3.0)
        self.declare_parameter("cmd_timeout_s", 0.5)

        folder = self.get_parameter("policy").value
        if not folder:
            raise SystemExit("policy 파라미터에 정책 꾸러미 폴더를 줄 것 (config/policies/<이름>)")
        checkpoint = self.get_parameter("checkpoint").value or None
        try:
            self.policy = deploy.load(Path(folder))
            model = self.policy.load_model(checkpoint)
        except (deploy.ContractError, ValueError, OSError) as e:
            raise SystemExit(f"{e}") from e
        self.runner = self.policy.runner(model)

        self.hz = float(self.get_parameter("hz").value) or self.policy.hz
        self.approach_s = float(self.get_parameter("approach_s").value)
        self.cmd_timeout_s = float(self.get_parameter("cmd_timeout_s").value)
        self.velocity_names = tuple(n for n in VELOCITY_COMMANDS if n in self.policy.commands)
        self._cmd_vel_at: Optional[float] = None
        """마지막 `cmd_vel` 을 받은 시각. 없으면 아직 한 번도 안 옴 (시작값 그대로)."""
        stale_s = float(self.get_parameter("state_stale_ms").value) / 1000.0
        self.joints = Freshness(stale_s)
        self.imu = Freshness(stale_s)
        self.gravity = Latest()
        """명령 토픽: 안 오면 계약의 시작값(walk 는 제자리, squat 는 기본자세 높이). 한 번
        왔다가 끊기면 속도는 `cmd_timeout_s` 뒤 0 (`_check_cmd_timeout`), 높이는 그대로."""

        self.stage = OFF
        self.started_at: Optional[float] = None
        self.approach_motion = None
        self.hold_pose: dict = {}
        self.emitted = 0

        self.targets_pub = self.create_publisher(
            JointState, "/huphy/joint_targets", qos_profile_sensor_data)
        self.status_pub = self.create_publisher(
            DiagnosticArray, "/huphy/policy_status", 10)
        self.create_subscription(
            JointState, "/huphy/joint_states", self._on_joints, qos_profile_sensor_data)
        self.create_subscription(
            Imu, "/huphy/imu", self._on_imu, qos_profile_sensor_data)
        self.create_subscription(
            Vector3Stamped, "/huphy/gravity", self._on_gravity, qos_profile_sensor_data)
        self.create_subscription(Twist, "/huphy/cmd_vel", self._on_cmd, 10)
        self.create_subscription(Float64, "/huphy/h_star", self._on_h_star, 10)
        self.create_service(Trigger, "/huphy/approach", self._on_approach)
        self.create_service(Trigger, "/huphy/start_policy", self._on_start)
        self.create_service(Trigger, "/huphy/stop_policy", self._on_stop)

        self.create_timer(1.0 / self.hz, self._tick)
        self.create_timer(1.0 / STATUS_HZ, self._publish_status)
        self.get_logger().info(
            f"{self.policy.name}  입력 {self.policy.obs_dim}  출력 {len(self.policy.order)}  "
            f"{self.hz:.0f}Hz  가중치 {model.path}  명령 {sorted(self.policy.commands)}"
            f"  — approach 를 기다림")

    # ---- 구독 — 짧게 끝낼 것 -----------------------------------------------
    def _on_joints(self, msg: JointState) -> None:
        try:
            self.joints.put(convert.arrays_to_observation(msg.name, msg.position))
        except ValueError as e:
            self.get_logger().warning(f"관절 상태를 버림: {e}")

    def _on_imu(self, msg: Imu) -> None:
        self.imu.put((msg.angular_velocity.x, msg.angular_velocity.y,
                      msg.angular_velocity.z))

    def _on_gravity(self, msg: Vector3Stamped) -> None:
        self.gravity.put((msg.vector.x, msg.vector.y, msg.vector.z))

    def _on_cmd(self, msg: Twist) -> None:
        # 정책에 있는 칸만 씀. walk 는 vy 칸이 없어서 linear.y 를 버림.
        self._cmd_vel_at = self._now()
        self._set_commands(
            vx=float(msg.linear.x), vy=float(msg.linear.y), wz=float(msg.angular.z))

    def _check_cmd_timeout(self) -> None:
        """속도 명령이 `cmd_timeout_s` 넘게 안 오면 0 으로. 한 번만 알림."""
        if self._cmd_vel_at is None or not self.velocity_names:
            return
        if self._now() - self._cmd_vel_at <= self.cmd_timeout_s:
            return
        self._cmd_vel_at = None
        moving = [n for n in self.velocity_names if self.runner.commands.values.get(n, 0.0) != 0.0]
        self._set_commands(**{n: 0.0 for n in self.velocity_names})
        if moving:
            self.get_logger().warning(
                f"cmd_vel 이 {self.cmd_timeout_s:g}초 넘게 안 옴 -> 속도 {moving} 를 0 으로")

    def _on_h_star(self, msg: Float64) -> None:
        self._set_commands(h_star=float(msg.data))

    def _set_commands(self, **values) -> None:
        for name, value in values.items():
            if name in self.policy.commands:
                self.runner.commands.set(name, value)

    # ---- 서비스 -----------------------------------------------------------
    def _on_approach(self, request, response):
        """꺼짐 -> 접근. 지금 자세에서 기준 자세로. 끝나면 저절로 대기."""
        del request
        if self.stage == RUNNING:
            response.success = False
            response.message = "정책이 도는 중. stop_policy 로 먼저 끌 것"
            return response
        observation, fresh = self.joints.take()
        if not fresh:
            response.success = False
            response.message = (
                f"관절 상태가 없거나 낡음 (나이 {self.joints.age():.3f}초). "
                f"control_node 가 돌고 있는지 볼 것")
            return response

        start = {n: float(observation.get(f"{n}.pos", 0.0)) for n in self.policy.order}
        self.hold_pose = dict(self.policy.default_pose())
        self.approach_motion = approach(self.hold_pose, start, self.approach_s)
        self.started_at = self._now()
        self.stage = APPROACH
        response.success = True
        response.message = (
            f"{self.approach_s:.0f}초에 걸쳐 기준 자세로. 도착하면 붙잡고 기다림 -> start_policy")
        return response

    def _on_start(self, request, response):
        """대기 -> 정책. `huphy-run` 의 Enter. 대기가 아니면 거절."""
        del request
        if self.stage == RUNNING:
            response.success = True
            response.message = "이미 정책이 도는 중"
            return response
        if self.stage == OFF:
            response.success = False
            response.message = "기준 자세가 아님. 먼저 /huphy/approach"
            return response
        if self.stage == APPROACH:
            left = self.approach_s - (self._now() - (self.started_at or self._now()))
            response.success = False
            response.message = f"아직 기준 자세로 가는 중 ({max(0.0, left):.1f}초 남음)"
            return response
        self.runner.reset()                  # 이력·직전 행동을 지금부터 새로 쌓음
        self.stage = RUNNING
        self.get_logger().info("정책 시작")
        response.success = True
        response.message = "정책 시작"
        return response

    def _on_stop(self, request, response):
        del request
        self.stage = OFF
        self.approach_motion = None
        self.started_at = None
        response.success = True
        response.message = "멈춤. 목표각을 안 냄 -> 모션보드가 감쇠로 들어감"
        return response

    # ---- 20ms 타이머 -------------------------------------------------------
    def _tick(self) -> None:
        self._check_cmd_timeout()
        if self.stage == OFF:
            return

        observation, fresh = self.joints.take()
        if not fresh:
            if self.stage != OFF:
                self.get_logger().warning(
                    f"관절 상태가 {self.joints.stale_s * 1000:.0f}ms 넘게 안 옴. "
                    f"목표각을 안 냄")
            return

        elapsed = self._now() - (self.started_at or self._now())
        if self.stage == APPROACH:
            if elapsed < self.approach_s:
                self._emit(self.approach_motion(elapsed, observation))
                return
            self.stage = READY
            self.get_logger().info("기준 자세 도착. 붙잡고 기다림 -- start_policy 로 정책 시작")
        if self.stage == READY:
            # 기준 자세를 계속 냄. 안 내면 모션보드가 100ms 뒤 감쇠로 떨어짐.
            self._emit(dict(self.hold_pose))
            return

        targets = self._infer(observation)
        if targets is not None:
            self._emit(targets)

    def _infer(self, observation):
        """한 번 추론. 관찰이 모자라면 `None`.

        이력(`q(t-1)`)은 **추론 호출 기준**으로 `Runner` 가 쌓음 -- 상태가 두 번 와도
        한 번만 씀. 첫 주기는 지금 위치로 앞을 메움.
        """
        gyro, fresh = self.imu.take()
        if not fresh:
            self.get_logger().warning("IMU 가 낡음. 목표각을 안 냄")
            return None
        gravity = self.gravity.take()
        if gravity is None:
            return None

        state = ImuState(
            gyro_dps=tuple(math.degrees(float(v)) for v in gyro),
            gravity=tuple(float(v) for v in gravity),
        )
        """`deploy` 가 다시 라디안으로 바꾸므로 도로 되돌려 넣음 --
        `ImuState` 의 계약이 도/초이고 (`sensors/base.py`), 그 계약을 ROS 때문에
        바꾸지 않음."""

        return self.runner.step(observation, state)

    def _emit(self, targets) -> None:
        names, position = convert.targets_to_arrays(targets, self.policy.order)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name, msg.position = names, position
        self.targets_pub.publish(msg)
        self.emitted += 1

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _publish_status(self) -> None:
        pairs = {
            "stage": self.stage,
            "joint_age_s": f"{self.joints.age():.3f}",
            "imu_age_s": f"{self.imu.age():.3f}",
            "joint_stale_count": str(self.joints.stale_count),
            "emitted": str(self.emitted),
        }
        pairs.update({
            f"cmd_{name}": f"{value:.3f}"
            for name, value in self.runner.commands.values.items()
        })
        status = DiagnosticStatus()
        status.name = "huphy/policy"
        status.hardware_id = self.policy.name
        status.level = (
            DiagnosticStatus.OK if self.stage == RUNNING
            else DiagnosticStatus.WARN
        )
        status.message = self.stage
        status.values = [KeyValue(key=k, value=v) for k, v in pairs.items()]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self.status_pub.publish(array)


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = PolicyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        logbridge.stop(node.log_handler)
        # Ctrl-C 면 rclpy 가 이미 shutdown 했음. 두 번 부르면 RCLError.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
