"""기록 노드 — 토픽을 받아 `huphy-run --log` 과 같은 JSON 파일로 씀.

    ros2 launch huphy_ros log.launch.py
    ros2 run huphy_ros log_node --ros-args -p log:=/home/me/HUPHY/ros/logs

제어도 추론도 안 함. 구독만 함.

## 왜 따로 있나

**기계가 둘임** (`ros/README.md` 결정 13). 모터는 모션보드에, 사람과 뷰어는 작업
기계에 있음. 두 노드가 각자 파일을 쓰면 정책 기록과 지연 기록이 **다른 디스크에**
떨어져, 같이 보려면 매번 옮겨야 함. 한 노드가 받아 쓰면 한 군데 모임.

그리고 파일 쓰기가 **제어 주기를 흔들지 않음.** 측정값은 1분 기록이 220~260ms,
5분이면 1.2~1.3초임. `control_node`·`policy_node` 는 단일 스레드 executor 에서
100Hz·50Hz 타이머를 돌리므로(`rclpy.spin`) 그 안에서 쓰면 낡음 기준 60ms/100ms 를
넘김 (`ros/README.md` 결정 12). 이 노드에는 타이머도 발행도 없음.


## 무엇을 받나

    /huphy/debug/policy_info   PolicyInfo    transient_local, 1회
    /huphy/debug/policy_row    PolicyRow     50Hz
    /huphy/debug/control_info  ControlInfo   transient_local, 1회
    /huphy/debug/latency       LatencyBatch  100Hz
    /huphy/debug/motor_extra   MotorExtra    100Hz, control_node 의 extra 가 1 이상일 때만
    /huphy/bus_current         BusCurrent    20Hz, current_node 를 띄웠을 때만
    /huphy/policy_status       DiagnosticArray  5Hz + 단계 전이 직후

두 `*Info` 가 `transient_local` 이라 **이 노드를 나중에 띄워도 받음.** 설정 파일은
하나도 안 읽음 -- 열 이름, 관절 순서, 가동범위, 주기, 수거 대기가 전부 토픽으로 옴.

열 이름을 받아 쓰는 이유: 정책 꾸러미를 다시 읽어 만들면 두 곳에서 만드는 것이
되어 어긋날 수 있음.


## 어느 구간을 기록하나

`policy_status` 의 단계가 `running` 인 동안만 쌓음. 접근·대기·감쇠 구간이 빠짐 --
`huphy-run` 이 `active=False` 와 `loop.on_settle` 로 맞추는 구간과 같음
(`scripts/run.py`).

단계가 `running` 을 벗어나면 **그 자리에서 두 파일을 쓰고** 기록을 새로 시작함.
`stop_policy` 한 번이 한 실행임 -- `huphy-run` 은 프로세스 하나가 한 실행이지만
이 노드는 한 번 띄워 여러 번 받음.


## 파일 이름

    <log>/policy/<월일>_policy_<정책>.json
    <log>/latency/<월일>_latency_<정책>.json

정책 이름은 `policy_status` 의 `hardware_id` 에서 옴. **한 노드가 두 이름을 다
지으므로** 짝이 맞고 `_2`/`_3` 번호도 같이 올라감 (`telemetry/run_name.py`).

`log` 파라미터는 절대경로임 (`log.launch.py` 의 기본값이 `$HOME/HUPHY/ros/logs`).
상대경로를 주면 이 노드를 띄운 터미널의 작업 디렉터리 기준이 되므로, 시작할 때
`resolve()` 한 절대경로를 로그에 찍음.
"""

from __future__ import annotations

import threading
from pathlib import Path
from typing import Dict, Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from huphy_ros_interfaces.msg import (
    BusCurrent, ControlInfo, LatencyBatch, MotorExtra, PolicyInfo, PolicyRow)
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data

from huphy.motors.robstride import tables
from huphy.telemetry import latency_log, policy_log, power, run_name

from . import convert, latency_tap, logbridge

INFO_QOS = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
"""`*Info` 구독에 쓰는 QoS. 내는 쪽과 같아야 함 -- 다르면 아예 안 붙음."""

RUNNING = "running"
"""`policy_status` 의 `message` 가 이 값인 동안만 쌓음. `policy_node` 가 정하는 값임."""

SOURCE = "ros"
"""기록의 `run.source`. `huphy-run` 기록과 같은 폴더에 섞여도 구별되게."""


class LogNode(Node):
    def __init__(self) -> None:
        super().__init__("huphy_log")
        self.log_handler = logbridge.forward(self.get_logger())

        self.declare_parameter("log", "")
        folder = str(self.get_parameter("log").value)
        if not folder:
            raise SystemExit(
                "log 파라미터에 기록 폴더를 줄 것 "
                "(ros2 launch huphy_ros log.launch.py 는 기본값을 줌)")
        self.folder = Path(folder).expanduser().resolve()

        self.policy_info: Optional[PolicyInfo] = None
        self.control_info: Optional[ControlInfo] = None
        self.policy: Optional[policy_log.PolicyLog] = None
        self.latency: Optional[latency_log.LatencyLog] = None
        self.r_phase: Dict[str, float] = {}
        """모터 이름 -> 상저항 (ohm). `ControlInfo.motor_models` 에서 만듦.

        전력 열을 계산하는 데 씀 (`telemetry/power.py`). 모터가 다른 기계에 있어서
        설정 파일을 못 읽으므로 모델명을 토픽으로 받아 여기서 상수로 바꿈. 사양을
        안 넣은 모델은 NaN 이 되고 그 모터의 전력 열이 NaN 이 됨.
        """
        self.extra_latest: Optional[MotorExtra] = None
        """마지막으로 받은 `MotorExtra`. 정책 줄이 올 때 그 줄에 넣음.

        전기값은 `control_node` 가 100Hz 로 내고 정책 줄은 50Hz 로 와서, 줄마다
        가장 최근 값을 씀. 두 기계 시계를 안 맞추므로 시각으로 짝지을 수 없음
        (`freshness.py`). `iq`·`vbus` 는 그보다 느리게 갱신되어 (`extra=2` 면 약
        17Hz) 연속 두세 줄에 같은 값이 기록됨.
        """
        self.current_latest: Optional[BusCurrent] = None
        """마지막으로 받은 `BusCurrent`. 정책 줄이 올 때 그 줄에 넣음.

        **한 번이라도 받으면 열이 생김.** `current_node` 를 띄운 실행에서만 생기고,
        안 띄웠으면 열 자체가 없음 (`telemetry/power.py` 4절). 열이 생긴 뒤 장비가
        멈추면 `current/age` 가 자람.

        전류계는 `MotorExtra` 와 달리 **다른 기계에 꽂혀 있어도 됨** -- CAN 과 무관함.
        """
        self.stage = ""
        self.policy_name = ""
        self.dropped_rows = 0
        """`PolicyInfo` 가 오기 전에 온 줄의 수. 0 이 아니면 늦게 띄운 것임."""
        self._writer: Optional[threading.Thread] = None

        self.create_subscription(
            PolicyInfo, "/huphy/debug/policy_info", self._on_policy_info, INFO_QOS)
        self.create_subscription(
            ControlInfo, "/huphy/debug/control_info", self._on_control_info, INFO_QOS)
        self.create_subscription(
            PolicyRow, "/huphy/debug/policy_row", self._on_row, qos_profile_sensor_data)
        self.create_subscription(
            LatencyBatch, "/huphy/debug/latency", self._on_batch, qos_profile_sensor_data)
        self.create_subscription(
            MotorExtra, "/huphy/debug/motor_extra", self._on_extra, qos_profile_sensor_data)
        self.create_subscription(
            BusCurrent, "/huphy/bus_current", self._on_current, qos_profile_sensor_data)
        self.create_subscription(
            DiagnosticArray, "/huphy/policy_status", self._on_status, 10)

        self.get_logger().info(
            f"{self.folder} 에 씀 — 정책이 시작하기 전에 띄워 둘 것 "
            f"(그 전에 흘러간 줄은 못 받음)")

    # ---- 설명 메시지 -------------------------------------------------------
    def _on_policy_info(self, msg: PolicyInfo) -> None:
        """열 이름과 관절 순서. 이게 와야 정책 기록을 만들 수 있음."""
        self.policy_info = msg
        self._new_policy_log()
        self.get_logger().info(
            f"정책 {msg.policy} 체크포인트 {msg.checkpoint}  입력 {len(msg.obs_names)}  "
            f"관절 {len(msg.joint_names)}  {msg.hz:.0f}Hz")

    def _on_control_info(self, msg: ControlInfo) -> None:
        """관절 이름·가동범위·수거 대기. 가동범위는 모션보드에만 있는 값임."""
        self.control_info = msg
        self.r_phase = {
            name: tables.phase_resistance_ohm(model)
            for name, model in zip(msg.joint_names, msg.motor_models)
        }
        self._new_latency_log()
        # limits 를 채워 다시 만듦. 다만 **이미 쌓인 줄이 있으면 안 바꿈** --
        # control_node 를 정책이 도는 중에 다시 띄우면 이 메시지가 늦게 올 수
        # 있고, 그때 다시 만들면 그 실행의 줄이 날아감.
        if self.policy is None or not len(self.policy):
            self._new_policy_log()
        self.get_logger().info(
            f"로봇 {msg.robot}  관절 {len(msg.joint_names)}  {msg.hz:.0f}Hz  "
            f"수거 {msg.drain_ms:g}ms  {msg.send_mode}  "
            f"전기값 {'끔' if not msg.extra else f'읽기 {msg.extra}개/주기'}")

    def _limits(self) -> dict:
        """`{관절: (하한, 상한)}`. 가동범위가 없는 관절(NaN)은 뺌.

        `joint_limits` 도 한계가 안 정해진 관절을 빼므로(`telemetry/track_log.py`)
        `huphy-run` 기록과 같은 모양이 됨.
        """
        info = self.control_info
        if info is None:
            return {}
        out = {}
        for name, lo, hi in zip(info.joint_names, info.limit_lo_deg, info.limit_hi_deg):
            if lo == lo and hi == hi:        # NaN 이 아님
                out[name] = (lo, hi)
        return out

    def _extra_motors(self) -> Optional[list]:
        """전기값 열을 미리 만들 모터 이름. `extra` 가 0 이거나 설명이 없으면 `None`.

        `None` 이면 `PolicyLog` 가 첫 줄의 전기값으로 열을 정함. `ControlInfo` 가
        먼저 오면 그 전에 열을 박아 둬야 함 -- 첫 정책 줄이 첫 `MotorExtra` 보다
        먼저 오면 빈 열로 굳어 이후 전기값이 다 버려짐.
        """
        info = self.control_info
        if info is None or not info.extra:
            return None
        return list(info.joint_names)

    def _extra_row(self) -> dict:
        """마지막 `MotorExtra` -> 한 줄의 `extra/...` 와 `power/...` 사전.

        아직 못 받았으면 빈 사전임 (그 줄의 칸이 다 `null` 이 됨).

        이름과 순서는 `ControlInfo.joint_names` 임 -- `control_node` 가 그 순서로
        배열을 만듦. `extra/` 는 `telemetry/track_log.py` 의 `extra_values` 와 같은
        모양이어야 `huphy-run --extra` 기록과 열 이름이 같음.

        `power/` 는 받은 상전류로 여기서 계산함 (`telemetry/power.py`) -- 모터가
        DC 전류도 전력도 안 냄. `huphy-run` 쪽에는 이 열이 없음.
        """
        msg, info = self.extra_latest, self.control_info
        out: dict = {}
        if msg is not None and info is not None:
            names = list(info.joint_names)
            out = convert.extra_to_columns(names, msg.iq, msg.vbus, msg.temp)
            if self._power_motors() is not None:
                out.update(power.power_values(
                    names, dict(zip(names, msg.iq)), dict(zip(names, msg.vbus)),
                    self.r_phase))
        if self.current_latest is not None:
            out.update(convert.current_to_columns(self.current_latest))
        return out

    def _power_motors(self) -> Optional[list]:
        """전력 열을 만들 모터 이름. 못 만들면 `None`.

        전기값을 읽을 때(`extra` 가 0 이 아님)만 만듦 -- 전력은 받은 상전류로
        계산한 값임. 상저항을 아는 모터가 하나도 없으면 안 만듦 -- `control_node`
        가 `motor_models` 를 안 보냈거나(예전 판 메시지) 사양을 안 넣은
        모델(RS00·RS02)뿐인 경우임. 열을 만들면 전부 `null` 이 됨.
        """
        info = self.control_info
        if info is None or not info.extra:
            return None
        if not any(v == v for v in self.r_phase.values()):      # 전부 NaN
            return None
        return list(info.joint_names)

    def _new_policy_log(self) -> None:
        info = self.policy_info
        if info is None:
            return
        self.policy = policy_log.PolicyLog(
            info.obs_names, info.joint_names, limits=self._limits(),
            extra_motors=self._extra_motors(), power_motors=self._power_motors(),
            current=self.current_latest is not None)

    def _new_latency_log(self) -> None:
        info = self.control_info
        if info is None:
            return
        self.latency = latency_log.LatencyLog(drain_ms=info.drain_ms, hz=info.hz)
        self.latency.set_joints(info.joint_names)

    # ---- 줄 -----------------------------------------------------------------
    def _on_row(self, msg: PolicyRow) -> None:
        if self.stage != RUNNING:
            return
        if self.policy is None:
            self.dropped_rows += 1
            return
        try:
            self.policy.add_row(msg.t, msg.obs, msg.action, msg.target_deg,
                                msg.measured_deg, self._extra_row())
        except ValueError as e:
            # 길이가 안 맞으면 열이 밀림. 그 줄만 버리고 한 번만 알림.
            self.policy.errors += 1
            if self.policy.errors == 1:
                self.get_logger().error(f"정책 줄을 버림: {e}")

    def _on_batch(self, msg: LatencyBatch) -> None:
        if self.stage != RUNNING or self.latency is None:
            return
        names = self.latency.joints
        for t, idx, ms, kind in zip(msg.t, msg.joint, msg.latency_ms, msg.kind):
            if idx >= len(names):
                continue
            joint = names[idx]
            if kind == latency_tap.SENT:
                self.latency.note_sent(joint)
            else:
                self.latency.add_sample(t, joint, ms, kind == latency_tap.LATE)

    def _on_extra(self, msg: MotorExtra) -> None:
        """전기값·온도. **마지막 것만 둠.** 정책 줄이 올 때 그 줄에 넣음."""
        self.extra_latest = msg

    def _on_current(self, msg: BusCurrent) -> None:
        """전류계 실측. **마지막 것만 둠.** 정책 줄이 올 때 그 줄에 넣음.

        첫 메시지는 열을 만들어야 하므로 기록을 다시 만듦 -- `_on_control_info` 와
        같은 처리임. **이미 쌓인 줄이 있으면 안 바꿈**: 정책이 도는 중에
        `current_node` 를 띄우면 이 메시지가 늦게 오고, 그때 다시 만들면 그 실행의
        줄이 날아감. 그 실행은 전류계 열 없이 끝나고 다음 실행부터 생김.
        """
        first = self.current_latest is None
        self.current_latest = msg
        if first:
            self.get_logger().info(
                f"전류계 값이 옴 ({msg.amps:+.2f} A) -- 기록에 current/ 열을 남김")
            if self.policy is None or not len(self.policy):
                self._new_policy_log()

    # ---- 단계 ---------------------------------------------------------------
    def _on_status(self, msg: DiagnosticArray) -> None:
        if not msg.status:
            return
        status = msg.status[0]
        self.policy_name = status.hardware_id or self.policy_name
        was, self.stage = self.stage, status.message
        if was == RUNNING and self.stage != RUNNING:
            self.save()

    # ---- 저장 ---------------------------------------------------------------
    def save(self) -> None:
        """두 파일을 쓰고 기록을 새로 시작함. **쓰기는 스레드로 넘김.**

        쓰는 데 1분 기록이 220~260ms 걸림 (측정). 이 노드에 타이머는 없지만 구독
        콜백은 계속 와야 하므로 -- 막혀 있는 동안 받은 것은 큐에 쌓이고, 깊이를
        넘기면 버려짐.

        객체를 먼저 새 것으로 바꾼 뒤 옛 것을 스레드에 넘김. 그래야 쓰는 동안
        들어오는 줄이 새 객체로 가고, 쓰는 쪽이 보는 것은 아무도 안 건드림.

        앞 저장이 아직 도는 중이면 기다림 -- `run_name.path` 가 `exists()` 로
        번호를 고르므로(`telemetry/run_name.py`) 둘이 같이 돌면 같은 이름을 고를 수
        있음.
        """
        name = self.policy_name or "unknown"
        jobs = []

        if self.policy is not None and len(self.policy):
            jobs.append((self.policy, policy_log.default_path(self.folder, name)))
            self._new_policy_log()
        if self.latency is not None and len(self.latency):
            jobs.append((self.latency, latency_log.default_path(self.folder, name)))
            self._new_latency_log()
        if self.dropped_rows:
            self.get_logger().warning(
                f"정책 줄 {self.dropped_rows}개를 못 받음 "
                f"(policy_info 가 오기 전에 온 것). 다음에는 먼저 띄울 것")
            self.dropped_rows = 0
        if not jobs:
            return

        if self._writer is not None and self._writer.is_alive():
            self._writer.join()
        run = self._run_section()
        self._writer = threading.Thread(
            target=self._write, args=(jobs, run), name="huphy-log-write")
        self._writer.start()

    def _run_section(self) -> dict:
        """파일의 `run` 칸. 두 파일이 같은 것을 씀.

        `huphy-run` 은 `scripts/run.py` 의 `_log_naming` 이 만들고, 거기 들어가는
        값 중 게인 배율은 ROS 에 파라미터가 없어 빠짐.
        """
        p, c = self.policy_info, self.control_info
        return run_name.header(
            source=SOURCE,
            policy=p.policy if p else None,
            checkpoint=p.checkpoint if p else None,
            hz=p.hz if p else None,
            approach=p.approach_s if p else None,
            robot=c.robot if c else None,
            motor_hz=c.hz if c else None,
            drain_ms=c.drain_ms if c else None,
            send_mode=c.send_mode if c else None,
            # 0 이면 안 적음 -- huphy-run 기록도 --extra 를 안 켰으면 이 칸이 없음
            extra=c.extra if (c and c.extra) else None,
            teleop=True,
        )

    def _write(self, jobs, run: dict) -> None:
        """스레드에서 돎. 실패해도 노드를 안 멈춤."""
        for log, path in jobs:
            try:
                written = log.save(path, run)
                self.get_logger().info(f"{written}  ({len(log)}줄)")
            except OSError as e:
                self.get_logger().error(f"{path} 를 못 씀: {e}")
        for log, _ in jobs:
            summary = getattr(log, "summary", None)
            if summary is not None:
                self.get_logger().info("\n" + summary())

    def shutdown(self) -> None:
        """Ctrl-C 로 끝낼 때. `stop_policy` 없이 끝낸 실행을 남김."""
        if self.stage == RUNNING:
            self.stage = ""
        self.save()
        if self._writer is not None and self._writer.is_alive():
            self._writer.join()


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = LogNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        logbridge.stop(node.log_handler)
        # Ctrl-C 면 rclpy 가 이미 shutdown 했음. 두 번 부르면 RCLError.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
