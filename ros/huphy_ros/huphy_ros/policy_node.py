"""정책 노드 — 작업 기계에서 돎. 어디로 갈지 정함.

    ros2 run huphy_ros policy_node --ros-args -p weights:=config/policies/generalized_walk.pt

## CAN 을 모름

모터가 몇 개인지, id 가 뭔지, 발목이 크랭크인지 모름. 받는 것은 관절 이름이 붙은
각도뿐이고 내는 것도 같은 모양임.

## 20ms 타이머가 추론을 촉발함

메시지 도착이 아니라 타이머임 (`ros/README.md` 결정 6). 도착으로 하면 **네트워크
지터가 그대로 추론 간격의 지터**가 됨 -- 정책은 20ms 간격을 전제로 학습했고,
관찰 6-17칸의 `q(t-1)` 이 정확히 한 주기 전 값이어야 함
(`control/generalized_walk.py` 의 `HISTORY_FRAMES`, `HZ`).

    타이머 20ms
      마지막 joint_states 나이 < 60ms  ->  추론하고 목표각을 냄
      나이 >= 60ms                     ->  아무것도 안 냄

안 내면 모션보드가 100ms 뒤에 감쇠로 들어감.

## 세 단계

    꺼짐      아무것도 안 냄
    접근      지금 자세에서 기준 자세로 approach_s 초 동안 보간한 목표각
    정책      신경망 출력

`/huphy/start_policy` 가 꺼짐 -> 접근, `approach_s` 가 지나면 접근 -> 정책.
`/huphy/stop_policy` 가 꺼짐으로.

접근을 여기서 하는 이유 (`ros/README.md` 결정 3): 기준 자세
(`generalized_walk.DEFAULT_POSE_DEG`)를 아는 쪽이 여기뿐임. 모션보드에도 두면 같은
값이 두 군데가 됨.

## motion() 을 안 쓰는 이유

`generalized_walk.motion(model, imu, command=...)` 은 `command` 를 호출 전체에서
고정으로 받음. `huphy-run --vx --wz` 가 시작할 때 한 번 정하는 구조임.

여기는 `/huphy/cmd_vel` 로 매 주기 바뀔 수 있어서, `motion()` 대신 그 안의
`observation_vector` 와 `joint_targets` 를 직접 부르고 `q(t-1)`·`last_action`
보관을 이 노드가 함 (`ros/README.md` 9장).
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import List, Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist, Vector3Stamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState
from std_srvs.srv import Trigger

from huphy.control import generalized_walk as gw
from huphy.control import rsl_rl
from huphy.scripts.selftest import approach
from huphy.sensors.base import ImuState

from . import convert
from .freshness import POLICY_STATE_STALE_S, Freshness, Latest

STATUS_HZ = 5.0

OFF, APPROACH, RUNNING = "off", "approach", "running"
"""세 단계. `policy_status` 에 그대로 나감."""


class PolicyNode(Node):
    def __init__(self) -> None:
        super().__init__("huphy_policy")

        self.declare_parameter("weights", "")
        self.declare_parameter("hz", gw.HZ)
        self.declare_parameter("state_stale_ms", POLICY_STATE_STALE_S * 1000.0)
        self.declare_parameter("approach_s", 3.0)

        weights = self.get_parameter("weights").value
        if not weights:
            raise SystemExit("weights 파라미터에 .pt 경로를 줄 것")
        if not Path(weights).is_file():
            raise SystemExit(f"가중치 파일이 없음: {weights}")
        try:
            self.model = rsl_rl.load(weights, spec=gw.SPEC, action_dim=len(gw.ORDER))
        except (ValueError, OSError) as e:
            raise SystemExit(f"{e}") from e

        self.hz = float(self.get_parameter("hz").value)
        self.approach_s = float(self.get_parameter("approach_s").value)
        stale_s = float(self.get_parameter("state_stale_ms").value) / 1000.0
        self.joints = Freshness(stale_s)
        self.imu = Freshness(stale_s)
        self.gravity = Latest()
        self.command = Latest()
        """`cmd_vel` 은 나이를 안 봄. 안 오면 (0, 0) 이고 그게 제자리임
        (`generalized_walk.COMMAND_ZERO`). 끊겼다고 멈출 이유가 아님."""

        self.stage = OFF
        self.started_at: Optional[float] = None
        self.approach_motion = None
        self.last_action: List[float] = [0.0] * len(gw.ORDER)
        self.previous: List[float] = []
        """`q(t-1)`. **추론 호출 기준**임 -- 상태가 두 번 와도 한 번만 씀."""
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
        self.create_service(Trigger, "/huphy/start_policy", self._on_start)
        self.create_service(Trigger, "/huphy/stop_policy", self._on_stop)

        self.create_timer(1.0 / self.hz, self._tick)
        self.create_timer(1.0 / STATUS_HZ, self._publish_status)
        self.get_logger().info(
            f"generalized_walk  입력 {gw.SPEC.obs_dim}  출력 {len(gw.ORDER)}  "
            f"{self.hz:.0f}Hz  가중치 {weights}  — start_policy 를 기다림")

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
        # linear.y 는 버림 -- 정책 관찰에 vy 칸이 없음 (generalized_walk.COMMAND_ZERO).
        self.command.put((float(msg.linear.x), float(msg.angular.z)))

    # ---- 서비스 -----------------------------------------------------------
    def _on_start(self, request, response):
        del request
        observation, fresh = self.joints.take()
        if not fresh:
            response.success = False
            response.message = (
                f"관절 상태가 없거나 낡음 (나이 {self.joints.age():.3f}초). "
                f"control_node 가 돌고 있는지 볼 것")
            return response

        start = {n: float(observation.get(f"{n}.pos", 0.0)) for n in gw.ORDER}
        self.approach_motion = approach(gw.default_pose(), start, self.approach_s)
        self.started_at = self._now()
        self.last_action = [0.0] * len(gw.ORDER)
        self.previous = []
        self.stage = APPROACH
        response.success = True
        response.message = f"{self.approach_s:.0f}초에 걸쳐 기준 자세로"
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
            self.stage = RUNNING
            self.get_logger().info("기준 자세 도착. 정책 시작")

        targets = self._infer(observation)
        if targets is not None:
            self._emit(targets)

    def _infer(self, observation):
        """한 번 추론. 관찰이 모자라면 `None`.

        `q(t-1)` 은 **직전 추론 때 쓴 관절각**임. 첫 주기는 이력이 없으므로 지금
        위치로 앞을 메움 (`generalized_walk.motion` 이 하는 것과 같음) -- 0 으로
        두면 정책이 한 주기 만에 기본자세만큼 움직였다고 읽음.
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
        """`observation_vector` 가 다시 라디안으로 바꾸므로 도로 되돌려 넣음 --
        `ImuState` 의 계약이 도/초이고 (`sensors/base.py`), 그 계약을 ROS 때문에
        바꾸지 않음."""

        now = [float(observation.get(f"{n}.pos", 0.0)) for n in gw.ORDER]
        if not self.previous:
            self.previous = list(now)

        command = self.command.take() or gw.COMMAND_ZERO
        vector = gw.observation_vector(
            observation, state, self.last_action, self.previous, tuple(command))
        action = self.model(vector)
        self.last_action = [float(v) for v in action]
        self.previous = now
        return gw.joint_targets(self.last_action)

    def _emit(self, targets) -> None:
        names, position = convert.targets_to_arrays(targets, gw.ORDER)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name, msg.position = names, position
        self.targets_pub.publish(msg)
        self.emitted += 1

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _publish_status(self) -> None:
        command = self.command.take() or gw.COMMAND_ZERO
        pairs = {
            "stage": self.stage,
            "joint_age_s": f"{self.joints.age():.3f}",
            "imu_age_s": f"{self.imu.age():.3f}",
            "joint_stale_count": str(self.joints.stale_count),
            "emitted": str(self.emitted),
            "vx": f"{command[0]:.3f}",
            "wz": f"{command[1]:.3f}",
        }
        status = DiagnosticStatus()
        status.name = "huphy/policy"
        status.hardware_id = gw.SPEC.name
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
        rclpy.shutdown()


if __name__ == "__main__":
    main()
