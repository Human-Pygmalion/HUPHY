"""모션 노드 — 작업 기계에서 돎. 기준 궤적 하나를 재생함.

    ros2 run huphy_ros motion_node --ros-args -p policy:=config/policies/union_dr_00545
    ros2 launch huphy_ros motion.launch.py policy:=$HOME/HUPHY/config/policies/union_dr_00545

`policy_node` 와 **같은 자리, 다른 입력**임.

    policy_node   사람이 vx·wz·h* 를 줌. 명령 토픽이 관찰의 명령 칸이 됨
    motion_node   파일에 든 궤적을 재생함. 그 프레임의 관절값이 명령 칸이 됨

그래서 `cmd_vel`·`h_star` 를 안 구독함. 대신 **몇 번째 프레임인지**가 상태이고,
`policy_status` 의 `frame`/`progress` 로 나감.

## 받는 것이 하나 더 있음

`joint_states` 의 **속도**도 씀. `policy_node` 는 위치만 쓰지만 이 정책의 관찰에는
관절 속도 24칸이 있음. 같은 토픽에 이미 실려 오고 있어서 더 받을 것은 없음
(`control_node` 의 `observation_to_arrays`).

## 관절 24개 중 12개는 모터가 없음

다리마다 로드 유니버설 2축 x 2개와 발목 2축임. 인코더가 없고 **크랭크각 2개에서
풀어서** 냄 (`kinematics/mjcf_loop.py`, 한 다리 약 0.5ms). 푸는 것은
`deploy.Runner` 안에서 하므로 이 노드가 따로 할 일은 없음.

## 네 단계 + 끝

    꺼짐      아무것도 안 냄
    접근      지금 자세에서 기준 자세로 approach_s 초 동안 보간
    대기      기준 자세를 계속 냄
    정책      신경망 출력. 한 주기에 기준 프레임 하나씩 나아감
    끝        마지막 프레임까지 재생함. **마지막 목표각을 계속 냄**

    /huphy/approach       꺼짐 -> 접근. approach_s 가 지나면 저절로 대기
    /huphy/start_policy   대기 -> 정책. 프레임 0 (또는 start_frame) 부터
    /huphy/stop_policy    어디서든 꺼짐

**끝에서 꺼지지 않는 이유:** 이 궤적은 몸통이 1.03m 에서 0.55m 로 내려가는
동작이라 끝 자세가 쪼그려 앉은 자세임. 목표각을 안 내면 모션보드가 100ms 뒤
감쇠로 들어가고 5초 뒤 토크를 끊어 그 자세에서 주저앉음. 그래서 마지막 목표각을
유지하고, 다음에 무엇을 할지는 사람이 정함 (`approach` 로 기본자세 복귀,
`stop_policy` 로 감쇠).

**되돌려 반복하지 않음.** 첫 프레임과 끝 프레임의 관절차가 최대 37.6도라 이어
붙이면 한 주기에 그만큼 뛰는 목표각이 나감.

## 기록

`policy_node` 와 같음 -- `log_node` 를 띄우면 기록됨. 단계가 `running` 인 동안만
쌓이므로 끝으로 넘어가는 순간 파일이 쓰임. 관찰 열 이름(`obs_names`)과 관절
순서는 시작할 때 `policy_info` 로 한 번 냄.
"""

from __future__ import annotations

import math
import time
from pathlib import Path
from typing import Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Vector3Stamped
from huphy_ros_interfaces.msg import PolicyInfo, PolicyRow
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState
from std_srvs.srv import Trigger

from huphy.control import deploy
from huphy.scripts.selftest import approach
from huphy.sensors.base import ImuState

from . import convert, logbridge
from .freshness import POLICY_STATE_STALE_S, Freshness, Latest

STATUS_HZ = 5.0

OFF, APPROACH, READY, RUNNING, DONE = "off", "approach", "ready", "running", "done"
"""단계. `policy_status` 의 `stage` 에 그대로 나감.

`log_node` 는 `running` 인 동안만 쌓고 벗어나면 파일을 씀 -- `done` 으로 넘어가는
순간 그 재생의 기록이 떨어짐."""

INFO_QOS = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
"""한 번만 내고 끝나는 값. 나중에 뜬 `log_node` 도 마지막 것을 받음."""


class MotionNode(Node):
    def __init__(self) -> None:
        super().__init__("huphy_motion")
        self.log_handler = logbridge.forward(self.get_logger())

        self.declare_parameter("policy", "")
        self.declare_parameter("checkpoint", "")
        self.declare_parameter("hz", 0.0)
        self.declare_parameter("state_stale_ms", POLICY_STATE_STALE_S * 1000.0)
        self.declare_parameter("approach_s", 3.0)
        self.declare_parameter("start_frame", 0)

        folder = self.get_parameter("policy").value
        if not folder:
            raise SystemExit(
                "policy 파라미터에 정책 꾸러미 폴더를 줄 것 (config/policies/<이름>)")
        checkpoint = self.get_parameter("checkpoint").value or None
        try:
            self.policy = deploy.load(Path(folder))
            model = self.policy.load_model(checkpoint)
        except (deploy.ContractError, ValueError, OSError) as e:
            raise SystemExit(f"{e}") from e
        if self.policy.reference is None:
            raise SystemExit(
                f"{self.policy.name} 에는 기준 모션이 없음 (계약에 motion 칸이 없음). "
                f"속도 명령 정책이면 policy_node 로 띄울 것")
        if self.policy.commands:
            raise SystemExit(
                f"{self.policy.name} 은 명령 칸({sorted(self.policy.commands)})이 있음. "
                f"이 노드는 명령 토픽을 안 받음 -- policy_node 로 띄울 것")

        self.start_frame = int(self.get_parameter("start_frame").value)
        try:
            self.runner = self.policy.runner(model, start_frame=self.start_frame)
        except ValueError as e:
            raise SystemExit(f"{e}") from e

        self.hz = float(self.get_parameter("hz").value) or self.policy.hz
        self.approach_s = float(self.get_parameter("approach_s").value)
        stale_s = float(self.get_parameter("state_stale_ms").value) / 1000.0
        self.joints = Freshness(stale_s)
        self.imu = Freshness(stale_s)
        self.gravity = Latest()

        self.stage = OFF
        self.started_at: Optional[float] = None
        self.approach_motion = None
        self.hold_pose: dict = {}
        """단계가 `대기` 면 기준 자세, `끝` 이면 마지막으로 낸 목표각."""
        self.emitted = 0
        self._row_t0: Optional[float] = None

        self.targets_pub = self.create_publisher(
            JointState, "/huphy/joint_targets", qos_profile_sensor_data)
        self.status_pub = self.create_publisher(
            DiagnosticArray, "/huphy/policy_status", 10)
        self.info_pub = self.create_publisher(
            PolicyInfo, "/huphy/debug/policy_info", INFO_QOS)
        self.row_pub = self.create_publisher(
            PolicyRow, "/huphy/debug/policy_row", qos_profile_sensor_data)
        self.info_pub.publish(PolicyInfo(
            policy=self.policy.name,
            checkpoint=str(checkpoint or self.policy.default_checkpoint),
            hz=self.hz,
            approach_s=self.approach_s,
            obs_names=list(self.policy.obs_names),
            joint_names=list(self.policy.order),
        ))
        self.create_subscription(
            JointState, "/huphy/joint_states", self._on_joints, qos_profile_sensor_data)
        self.create_subscription(
            Imu, "/huphy/imu", self._on_imu, qos_profile_sensor_data)
        self.create_subscription(
            Vector3Stamped, "/huphy/gravity", self._on_gravity, qos_profile_sensor_data)
        self.create_service(Trigger, "/huphy/approach", self._on_approach)
        self.create_service(Trigger, "/huphy/start_policy", self._on_start)
        self.create_service(Trigger, "/huphy/stop_policy", self._on_stop)

        self.create_timer(1.0 / self.hz, self._tick)
        self.create_timer(1.0 / STATUS_HZ, self._publish_status)
        ref = self.policy.reference
        self.get_logger().info(
            f"{self.policy.name}  입력 {self.policy.obs_dim}  출력 {len(self.policy.order)}  "
            f"{self.hz:.0f}Hz  가중치 {model.path}  "
            f"모션 {ref.frames}프레임 {ref.duration_s:.1f}초 (프레임 {self.start_frame} 부터)"
            f"  — approach 를 기다림")

    # ---- 구독 — 짧게 끝낼 것 -----------------------------------------------
    def _on_joints(self, msg: JointState) -> None:
        """위치와 **속도** 둘 다 씀. 관찰에 관절 속도 24칸이 있음."""
        try:
            self.joints.put(convert.arrays_to_observation(
                msg.name, msg.position, msg.velocity))
        except ValueError as e:
            self.get_logger().warning(f"관절 상태를 버림: {e}")

    def _on_imu(self, msg: Imu) -> None:
        self.imu.put((msg.angular_velocity.x, msg.angular_velocity.y,
                      msg.angular_velocity.z))

    def _on_gravity(self, msg: Vector3Stamped) -> None:
        self.gravity.put((msg.vector.x, msg.vector.y, msg.vector.z))

    # ---- 서비스 -------------------------------------------------------------
    def _on_approach(self, request, response):
        """꺼짐·끝 -> 접근. 지금 자세에서 기준 자세로. 끝나면 저절로 대기.

        `끝` 단계에서도 받음 -- 쪼그려 앉은 자세에서 기본자세로 돌아오는 길임.
        """
        del request
        if self.stage == RUNNING:
            response.success, response.message = False, (
                "모션이 도는 중. stop_policy 로 먼저 끌 것")
            return response
        observation, fresh = self.joints.take()
        if not fresh:
            response.success, response.message = False, (
                f"관절 상태가 없거나 낡음 (나이 {self.joints.age():.3f}초). "
                f"control_node 가 떠 있는지 볼 것")
            return response
        start = {n: float(observation.get(f"{n}.pos", 0.0)) for n in self.policy.order}
        self.hold_pose = dict(self.policy.default_pose())
        self.approach_motion = approach(self.hold_pose, start, self.approach_s)
        self.stage = APPROACH
        self.started_at = self._now()
        self._publish_status()
        self.get_logger().info(f"기준 자세로 {self.approach_s:.1f}초 동안 감")
        response.success, response.message = True, "접근 시작"
        return response

    def _on_start(self, request, response):
        if self.stage != READY:
            response.success, response.message = False, (
                f"지금 단계가 {self.stage} 임. approach 로 기준 자세에 간 뒤 부를 것")
            return response
        del request
        self.runner.reset()
        self._row_t0 = None
        self.stage = RUNNING
        self.started_at = self._now()
        self._publish_status()
        ref = self.policy.reference
        self.get_logger().info(
            f"모션 시작 -- 프레임 {self.start_frame}~{ref.frames - 1}, "
            f"{(ref.frames - self.start_frame) / self.hz:.1f}초")
        response.success, response.message = True, "모션 시작"
        return response

    def _stop(self) -> None:
        self.stage = OFF
        self.approach_motion = None
        self.hold_pose = {}
        self._row_t0 = None
        self._publish_status()

    def _on_stop(self, request, response):
        del request
        was = self.stage
        self._stop()
        self.get_logger().info(f"멈춤 ({was} -> {OFF}). 목표각을 안 냄 -> 모션보드가 감쇠")
        response.success, response.message = True, f"{was} -> {OFF}"
        return response

    # ---- 한 주기 ------------------------------------------------------------
    def _tick(self) -> None:
        if self.stage == OFF:
            return

        observation, fresh = self.joints.take()
        if not fresh:
            self.get_logger().warning(
                f"관절 상태가 {self.joints.stale_s * 1000:.0f}ms 넘게 안 옴. 목표각을 안 냄")
            return

        elapsed = self._now() - (self.started_at or self._now())
        if self.stage == APPROACH:
            if elapsed < self.approach_s:
                self._emit(self.approach_motion(elapsed, observation))
                return
            self.stage = READY
            self._publish_status()
            self.get_logger().info(
                "기준 자세 도착. 붙잡고 기다림 -- start_policy 로 모션 시작")
        if self.stage in (READY, DONE):
            # 같은 목표각을 계속 냄. 안 내면 모션보드가 100ms 뒤 감쇠로 떨어짐.
            self._emit(dict(self.hold_pose))
            return

        targets = self._infer(observation)
        if targets is None:
            return
        self._emit(targets)
        if self.row_pub is not None and self.row_pub.get_subscription_count() > 0:
            self._publish_row(targets, observation)
        if self.runner.clock is not None and self.runner.clock.done:
            self.hold_pose = dict(targets)
            self.stage = DONE
            self._publish_status()
            self.get_logger().info(
                f"모션 끝 (프레임 {self.runner.clock.index}). 마지막 목표각을 유지함 -- "
                f"approach 로 기본자세 복귀, stop_policy 로 감쇠")

    def _publish_row(self, targets, observation) -> None:
        """추론 한 번을 `PolicyRow` 로. `log_node` 가 기록의 한 줄로 씀."""
        vector = self.runner.last_vector
        if vector is None:
            return
        now = time.monotonic()
        if self._row_t0 is None:
            self._row_t0 = now
        try:
            msg = PolicyRow()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.t = now - self._row_t0
            msg.obs = [float(v) for v in vector]
            msg.action = [float(v) for v in self.runner.last_action]
            msg.target_deg = [float(targets.get(j, float("nan"))) for j in self.policy.order]
            msg.measured_deg = [
                float(observation.get(f"{j}.pos", float("nan"))) for j in self.policy.order
            ]
            self.row_pub.publish(msg)
        except Exception as e:                # noqa: BLE001
            self.row_pub = None
            self.get_logger().error(f"기록 발행 실패. 기록만 끔 (모션은 계속 돎): {e}")

    def _infer(self, observation):
        """한 번 추론. 관찰이 모자라면 `None`.

        종속 관절 풀이가 실패하면(`LoopUnsolvedError`) 목표각을 안 냄 -- 크랭크
        인코더 둘이 서로 모순되는 값일 때임. 엉뚱한 관찰로 추론하는 것보다 한 주기
        쉬는 쪽이 나음. 이어지면 모션보드가 감쇠로 들어감.
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
        try:
            return self.runner.step(observation, state)
        except Exception as e:                # noqa: BLE001
            self.get_logger().warning(f"추론을 건너뜀: {e}")
            return None

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
        """상태 한 번. 5Hz 타이머와 **단계 전이 직후**.

        `frame`·`progress` 가 재생 위치임. `hardware_id` 의 정책 이름으로
        `log_node` 가 기록 파일 이름을 지음.
        """
        clock = self.runner.clock
        pairs = {
            "stage": self.stage,
            "frame": str(clock.index if clock else -1),
            "frames": str(self.policy.reference.frames),
            "progress": f"{clock.progress:.3f}" if clock else "0.000",
            "joint_age_s": f"{self.joints.age():.3f}",
            "imu_age_s": f"{self.imu.age():.3f}",
            "joint_stale_count": str(self.joints.stale_count),
            "emitted": str(self.emitted),
        }
        status = DiagnosticStatus()
        status.name = "huphy/policy"
        status.hardware_id = self.policy.name
        status.level = (
            DiagnosticStatus.OK if self.stage == RUNNING else DiagnosticStatus.WARN)
        status.message = self.stage
        status.values = [KeyValue(key=k, value=v) for k, v in pairs.items()]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self.status_pub.publish(array)


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = MotionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        logbridge.stop(node.log_handler)
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
