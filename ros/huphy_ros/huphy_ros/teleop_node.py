"""조작 노드 — 작업 기계에서 돎. `huphy-run --teleop` 의 pygame 창을 ROS 로.

    ros2 launch huphy_ros teleop.launch.py policy:=$HOME/HUPHY/config/policies/huphy_squat

창과 키는 `huphy-run --teleop` 과 같음 (`src/huphy/control/teleop.py` 를 그대로 씀).
정책 꾸러미를 받아 그 정책에 있는 명령만 조작함 -- walk 는 W/S/A/D, squat 는 ↑/↓.

## 키

    W/S/A/D, ↑/↓, Space   명령. 창 값을 20Hz 로 /huphy/cmd_vel, /huphy/h_star 에 냄
    R                     /huphy/approach        기준 자세로 (토크가 켜져 있을 때만)
    Enter                 /huphy/start_policy    정책 시작 (기준 자세 도착 뒤에만 -- 정책 노드가 봄)
    Esc, 창 닫기          /huphy/stop_policy -> /huphy/disable, 그리고 이 노드도 끝남

**토크 켜기(`/huphy/enable`)는 키로 안 함.** 가장 위험한 순간이라 사람이 명령으로 침.

**R 은 토크가 켜져 있을 때만** 보냄. 토크 없이 기준 자세(ready)를 잡아 둔 채로 나중에
enable 하면, 켜지는 순간 모터가 지금 자세에서 기준 자세로 한 번에 튐.

## 상태 줄

창 맨 위에 지금 상태를 띄움 -- `/huphy/loop_status` 의 모드와 `/huphy/policy_status`
의 단계로 정함 (`teleop.STATUS_TEXT`).

## 끊겼을 때

이 노드가 끝나면(Esc, 창 닫기, Ctrl-C) 속도 0 을 한 번 내고 끝남. 갑자기 죽으면 그것도
못 내는데, 그때는 정책 노드가 `cmd_timeout_s`(0.5초) 뒤 속도를 0 으로 돌림. 창 프로세스만
멈추면 `Teleop.targets` 가 0.2초 뒤부터 속도 0 을 냄 (`huphy-run` 과 같음).
"""

from __future__ import annotations

import time
from pathlib import Path
from typing import Dict, Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64
from std_srvs.srv import Trigger

from huphy.control import deploy, teleop

from . import logbridge

PUBLISH_HZ = 20.0
STATUS_STALE_S = 1.0
"""상태 토픽이 이만큼 안 오면 그 노드가 없는 것으로 봄. 둘 다 5Hz 로 냄."""

# 창 상태 줄 코드 (teleop.STATUS_TEXT)
TORQUE_OFF, GO_READY, APPROACHING, READY, RUNNING, NO_CONTROL, NO_POLICY = 1, 2, 3, 4, 5, 6, 7


def _name_values(text: str, what: str) -> Dict[str, float]:
    """`"vx=0.8 wz=1.0"` (쉼표도 됨) -> {"vx": 0.8, "wz": 1.0}. `huphy-run --max` 와 같은 꼴."""
    out: Dict[str, float] = {}
    for item in text.replace(",", " ").split():
        name, sep, value = item.partition("=")
        if not sep:
            raise SystemExit(f"{what} 는 이름=값 꼴이어야 함 (받은 값 {item!r})")
        try:
            out[name.strip()] = float(value)
        except ValueError:
            raise SystemExit(f"{what} {name}: 숫자가 아님 ({value!r})") from None
    return out


class TeleopNode(Node):
    def __init__(self) -> None:
        super().__init__("huphy_teleop")
        self.log_handler = logbridge.forward(self.get_logger())

        self.declare_parameter("policy", "")
        for k in ("max", "accel", "decel"):
            self.declare_parameter(k, "")

        folder = self.get_parameter("policy").value
        if not folder:
            raise SystemExit("policy 파라미터에 정책 꾸러미 폴더를 줄 것 (policy_node 와 같은 것)")
        try:
            self.policy = deploy.load(Path(folder))
        except (deploy.ContractError, ValueError, OSError) as e:
            raise SystemExit(f"{e}") from e
        if not self.policy.commands:
            raise SystemExit(f"{self.policy.name}: 조작할 명령이 없는 정책임")

        overrides = {k: _name_values(str(self.get_parameter(k).value), k)
                     for k in ("max", "accel", "decel")}
        try:
            axes = teleop.make_axes(self.policy.commands.values(), overrides)
            self.pad = teleop.Teleop(
                axes, start={n: s.start for n, s in self.policy.commands.items()},
                title=f"huphy ROS teleop {self.policy.name}",
            ).start()
        except teleop.TeleopError as e:
            raise SystemExit(f"teleop: {e}") from e
        self.velocity_names = [a.name for a in axes if a.kind == "velocity"]
        self.height_names = [a.name for a in axes if a.kind == "height"]

        self.cmd_vel_pub = (self.create_publisher(Twist, "/huphy/cmd_vel", 10)
                            if self.velocity_names else None)
        self.h_star_pub = (self.create_publisher(Float64, "/huphy/h_star", 10)
                           if "h_star" in self.height_names else None)

        self.mode: Optional[str] = None
        self.mode_at = 0.0
        self.stage: Optional[str] = None
        self.stage_at = 0.0
        self.create_subscription(DiagnosticArray, "/huphy/loop_status", self._on_loop, 10)
        self.create_subscription(DiagnosticArray, "/huphy/policy_status", self._on_policy, 10)

        self._triggers = {
            name: self.create_client(Trigger, f"/huphy/{name}")
            for name in ("approach", "start_policy", "stop_policy", "disable")
        }
        self._seen_start = self.pad.start_count()
        self._seen_approach = self.pad.approach_count()
        self.stop_reason: Optional[str] = None
        """Esc / 창 닫기를 받으면 채움. 끄는 절차는 메인 루프가 함 -- 콜백 안에서 서비스
        응답을 기다리면 같은 실행기가 막힘."""
        self.finished = False

        self.create_timer(1.0 / PUBLISH_HZ, self._tick)
        self.get_logger().info(
            f"{self.policy.name}  창: {self.pad.help_line().strip()}  "
            f"-- R 기준 자세, Enter 정책 시작, Esc 정책·토크 끔. 토크는 /huphy/enable 로")

    # ---- 상태 -------------------------------------------------------------
    def _on_loop(self, msg: DiagnosticArray) -> None:
        if msg.status:
            self.mode, self.mode_at = msg.status[0].message, time.monotonic()

    def _on_policy(self, msg: DiagnosticArray) -> None:
        if msg.status:
            self.stage, self.stage_at = msg.status[0].message, time.monotonic()

    def _status(self) -> int:
        now = time.monotonic()
        if now - self.mode_at > STATUS_STALE_S:
            return NO_CONTROL
        if now - self.stage_at > STATUS_STALE_S:
            return NO_POLICY
        if self.mode != "control":
            return TORQUE_OFF
        return {"approach": APPROACHING, "ready": READY, "running": RUNNING}.get(self.stage, GO_READY)

    # ---- 20Hz -------------------------------------------------------------
    def _tick(self) -> None:
        if self.finished or self.stop_reason is not None:
            return
        if self.pad.stop_requested():
            self.stop_reason = "창에서 멈춤 (Esc / 창 닫기)"
            return

        status = self._status()
        self.pad.set_status(status)
        self._publish(self.pad.targets())

        n = self.pad.approach_count()
        if n > self._seen_approach:
            self._seen_approach = n
            if status in (TORQUE_OFF, NO_CONTROL):
                self.get_logger().warning("R: 토크가 꺼져 있음. 먼저 /huphy/enable -- 안 보냄")
            else:
                self._call("approach")

        n = self.pad.start_count()
        if n > self._seen_start:
            self._seen_start = n
            self._call("start_policy")          # ready 가 아니면 정책 노드가 거절하고 이유를 줌

    def _publish(self, values: Dict[str, float]) -> None:
        if self.cmd_vel_pub is not None:
            msg = Twist()
            msg.linear.x = float(values.get("vx", 0.0))
            msg.linear.y = float(values.get("vy", 0.0))
            msg.angular.z = float(values.get("wz", 0.0))
            self.cmd_vel_pub.publish(msg)
        if self.h_star_pub is not None and "h_star" in values:
            self.h_star_pub.publish(Float64(data=float(values["h_star"])))

    def _call(self, name: str):
        """서비스를 부르고 답을 로그로 남김. 기다리지 않음 -- 타이머가 안 막히게."""
        client = self._triggers[name]
        if not client.service_is_ready():
            self.get_logger().warning(f"/huphy/{name} 서비스가 없음 -- 노드가 떠 있는지 볼 것")
            return None
        future = client.call_async(Trigger.Request())

        def done(f, name=name):
            try:
                r = f.result()
            except Exception as e:  # noqa: BLE001
                self.get_logger().warning(f"/huphy/{name} 실패: {e}")
                return
            log = self.get_logger().info if r.success else self.get_logger().warning
            log(f"/huphy/{name}: {r.message}")

        future.add_done_callback(done)
        return future

    def finish(self, why: str) -> None:
        """정책 끔 -> 토크 끔 -> 속도 0 -> 끝냄. **메인 루프에서만** 부름."""
        self.finished = True
        self.get_logger().info(f"{why} -> stop_policy, disable")
        self._publish({n: 0.0 for n in self.velocity_names})
        for name in ("stop_policy", "disable"):
            future = self._call(name)
            if future is not None:
                rclpy.spin_until_future_complete(self, future, timeout_sec=8.0)

    def close(self) -> None:
        try:
            self._publish({n: 0.0 for n in self.velocity_names})
        except Exception:  # noqa: BLE001 - Ctrl-C 뒤면 이미 컨텍스트가 닫혔을 수 있음
            pass
        finally:
            self.pad.close()


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = TeleopNode()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.stop_reason is not None:
                node.finish(node.stop_reason)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        logbridge.stop(node.log_handler)
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
