"""패드 조작 노드 — 작업 기계에서 돎. Xbox 패드로 명령·시작·멈춤.

    ros2 launch huphy_ros gamepad.launch.py policy:=$HOME/HUPHY/config/policies/generalized_walk

`teleop_node` 와 내는 것이 같음 (`/huphy/cmd_vel`, `/huphy/h_star`, 서비스 넷).
다른 것은 입력뿐임 -- pygame 창 대신 `joy` 패키지의 `joy_node` 가 내는
`sensor_msgs/Joy` 를 `/joy` 에서 받음. `gamepad.launch.py` 가 둘을 같이 띄움.


## 조작 (기본값. 전부 파라미터로 바꿈)

    왼쪽 스틱 세로     vx        위로 밀면 +
    왼쪽 스틱 가로     vy        왼쪽으로 밀면 +   (vy 가 있는 정책만)
    오른쪽 스틱 가로   wz        왼쪽으로 밀면 +
    십자키 세로        h_star    (h_star 가 있는 정책만)

    RB  누르고 있는 동안만 속도가 나감 (데드맨). 떼면 0
    A        /huphy/approach        기준 자세로 -- 토크가 켜져 있을 때만 보냄
    RB+Menu  /huphy/start_policy    정책 시작 -- RB 를 누르고 있는 동안에만
    B        /huphy/stop_policy -> /huphy/disable, 그리고 이 노드도 끝남
    View 1초 /huphy/enable          토크 넣기


## 위험한 둘에는 조건을 걸어 둠

**토크 넣기(View)는 `enable_hold_s`(기본 1.0초) 동안 계속 눌려 있어야** 한 번 나감.
토크가 들어가는 순간이 가장 위험해서, 패드를 집다가 한 번 눌리는 것으로는 안 나가게
한 것임. 뗐다 다시 눌러야 또 나감.

**정책 시작(Menu)은 RB 를 누르고 있는 동안에만** 나감. RB 는 속도를 내보내는 데드맨
이기도 해서, 이 조건은 "손이 데드맨에 가 있어야 정책이 돈다" 가 됨. RB 없이 Menu 를
누르면 안 보내고 이유만 로그로 남김. 그 뒤 **Menu 를 누른 채로 RB 를 더해도 안 나감**
-- 서비스는 버튼이 안 눌림에서 눌림으로 바뀐 주기에만 부르기 때문임. Menu 를 뗐다가
RB 를 먼저 쥐고 다시 눌러야 함. RB 를 떼면 정책은 계속 돌지만 속도 명령이 0 이
되어 제자리걸음이 됨 -- 멈추는 것은 B 임.


## 스틱 값 -> 명령 값

    명령 값 = 스틱값 × 그 명령의 max

`max` 는 `teleop.make_axes` 가 정책 계약에서 뽑은 값임 (`--max` 로 덮을 수 있음).
`teleop_node` 는 키가 켜짐/꺼짐뿐이라 `accel`/`decel` 로 값을 올리고 내리지만, 스틱은
-1.0~+1.0 이 바로 오므로 그 램프를 안 씀.

`h_star` 는 범위 안에서 움직여야 하므로 십자키를 누른 동안 `accel`(기본 0.05 m/s)
만큼 더하고 계약의 범위로 자름.

스틱을 놓아도 0 이 정확히 안 나옴. `deadzone`(기본 0.08) 미만은 0 으로 봄.


## 끊겼을 때

`/joy` 가 `joy_timeout_s`(기본 0.5초) 넘게 안 오면 (선이 빠짐, `joy_node` 가 죽음)
스틱과 버튼을 전부 놓은 것으로 보고 속도 0 을 계속 냄. 이 노드까지 죽으면 `/huphy/cmd_vel`
자체가 끊겨서 정책 노드가 `cmd_timeout_s`(0.5초) 뒤 속도를 0 으로 돌림.

**속도만 0 이 됨. 토크는 안 꺼짐** -- 끄는 것은 B 또는 터미널에서 `/huphy/disable`.
"""

from __future__ import annotations

import time
from pathlib import Path
from typing import Dict, List, Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64
from std_srvs.srv import Trigger

from huphy.control import deploy, teleop

from . import logbridge
from .teleop_node import _name_values

PUBLISH_HZ = 20.0
STATUS_STALE_S = 1.0
"""상태 토픽이 이만큼 안 오면 그 노드가 없는 것으로 봄. 둘 다 5Hz 로 냄."""

AXES = {"vx": 1, "vy": 0, "wz": 3, "h_star": 7}
"""명령 이름 -> `Joy.axes` 의 칸 번호. 커널 `xpad` 드라이버 기준임.

    0 왼쪽 스틱 가로   1 왼쪽 스틱 세로   2 LT   3 오른쪽 스틱 가로
    4 오른쪽 스틱 세로 5 RT              6 십자키 가로   7 십자키 세로

블루투스로 붙이거나 `xpadneo` 드라이버가 끼면 번호가 달라짐. `ros2 topic echo /joy`
로 보고 `axis_<이름>` 파라미터로 바꿀 것.
"""

BUTTONS = {"deadman": 5, "approach": 0, "start_policy": 7, "stop": 1, "enable": 6}
"""할 일 -> `Joy.buttons` 의 칸 번호. `xpad` 기준으로 0 A, 1 B, 2 X, 3 Y,
4 LB, 5 RB, 6 View, 7 Menu. `button_<이름>` 파라미터로 바꿈."""


class GamepadNode(Node):
    def __init__(self) -> None:
        super().__init__("huphy_gamepad")
        self.log_handler = logbridge.forward(self.get_logger())

        self.declare_parameter("policy", "")
        for k in ("max", "accel"):
            self.declare_parameter(k, "")
        self.declare_parameter("deadzone", 0.08)
        self.declare_parameter("enable_hold_s", 1.0)
        self.declare_parameter("joy_timeout_s", 0.5)
        for name, index in AXES.items():
            self.declare_parameter(f"axis_{name}", index)
        for name, index in BUTTONS.items():
            self.declare_parameter(f"button_{name}", index)

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
                     for k in ("max", "accel")}
        try:
            axes = teleop.make_axes(self.policy.commands.values(), overrides)
        except teleop.TeleopError as e:
            raise SystemExit(f"gamepad: {e}") from e
        self.axes = {a.name: a for a in axes}

        self.axis_index = {n: int(self.get_parameter(f"axis_{n}").value)
                           for n in AXES if n in self.axes}
        unmapped = sorted(set(self.axes) - set(self.axis_index))
        if unmapped:
            raise SystemExit(
                f"{self.policy.name}: 패드에 배정할 자리가 없는 명령 {unmapped}. "
                f"배정된 것: {sorted(AXES)}")
        self.button_index = {k: int(self.get_parameter(f"button_{k}").value)
                             for k in BUTTONS}
        self.deadzone = float(self.get_parameter("deadzone").value)
        self.enable_hold_s = float(self.get_parameter("enable_hold_s").value)
        self.joy_timeout_s = float(self.get_parameter("joy_timeout_s").value)

        self.velocity_names = [a.name for a in axes if a.kind == "velocity"]
        self.height_names = [a.name for a in axes if a.kind == "height"]
        self.values: Dict[str, float] = {a.name: a.start for a in axes}
        """지금 내보내는 명령 값. 속도는 매 주기 스틱에서 다시 정하고, 높이는 쌓임."""

        self.cmd_vel_pub = (self.create_publisher(Twist, "/huphy/cmd_vel", 10)
                            if self.velocity_names else None)
        self.h_star_pub = (self.create_publisher(Float64, "/huphy/h_star", 10)
                           if "h_star" in self.height_names else None)

        self.joy: Optional[Joy] = None
        self.joy_at = 0.0
        self.pressed: Dict[str, bool] = {k: False for k in BUTTONS}
        """직전 주기에 눌려 있었나. 안 눌림 -> 눌림 으로 바뀐 주기에만 서비스를 부름."""
        self.enable_since: Optional[float] = None
        """View 를 누르기 시작한 시각. 떼면 None."""
        self.enable_sent = False
        """이번에 누르고 있는 동안 이미 보냈나. 떼면 False 로 돌아감."""
        self.warned_joy = False

        self.mode: Optional[str] = None
        self.mode_at = 0.0
        self.stage: Optional[str] = None
        self.stage_at = 0.0
        self.create_subscription(Joy, "/joy", self._on_joy, 10)
        self.create_subscription(DiagnosticArray, "/huphy/loop_status", self._on_loop, 10)
        self.create_subscription(DiagnosticArray, "/huphy/policy_status", self._on_policy, 10)

        self._triggers = {
            name: self.create_client(Trigger, f"/huphy/{name}")
            for name in ("enable", "approach", "start_policy", "stop_policy", "disable")
        }
        self.stop_reason: Optional[str] = None
        """B 를 받으면 채움. 끄는 절차는 메인 루프가 함 -- 콜백 안에서 서비스 응답을
        기다리면 같은 실행기가 막힘."""
        self.finished = False
        self.last_tick = time.monotonic()

        self.create_timer(1.0 / PUBLISH_HZ, self._tick)
        self.get_logger().info(
            f"{self.policy.name}  패드: 스틱 {sorted(self.axes)}  "
            f"RB 누른 동안만 속도  A 기준 자세  RB+Menu 정책 시작  B 끔  "
            f"View {self.enable_hold_s:g}초 토크 넣기")

    # ---- 들어오는 것 -------------------------------------------------------
    def _on_joy(self, msg: Joy) -> None:
        self.joy, self.joy_at = msg, time.monotonic()

    def _on_loop(self, msg: DiagnosticArray) -> None:
        if msg.status:
            self.mode, self.mode_at = msg.status[0].message, time.monotonic()

    def _on_policy(self, msg: DiagnosticArray) -> None:
        if msg.status:
            self.stage, self.stage_at = msg.status[0].message, time.monotonic()

    def _torque_on(self) -> bool:
        """제어 노드가 CONTROL 인가. 상태가 낡았으면 아닌 것으로 봄."""
        return self.mode == "control" and time.monotonic() - self.mode_at <= STATUS_STALE_S

    # ---- 패드 읽기 ---------------------------------------------------------
    def _axis(self, index: int) -> float:
        """`Joy.axes[index]`. 칸이 없으면 0. 가운데 흔들림은 잘라냄."""
        if self.joy is None or index >= len(self.joy.axes):
            return 0.0
        v = float(self.joy.axes[index])
        return 0.0 if abs(v) < self.deadzone else v

    def _button(self, name: str) -> bool:
        if self.joy is None:
            return False
        index = self.button_index[name]
        return index < len(self.joy.buttons) and bool(self.joy.buttons[index])

    def _edges(self) -> List[str]:
        """이번 주기에 **새로** 눌린 버튼 이름들. 누르고 있는 동안은 한 번만 남."""
        fired = []
        for name in BUTTONS:
            now = self._button(name)
            if now and not self.pressed[name]:
                fired.append(name)
            self.pressed[name] = now
        return fired

    # ---- 20Hz -------------------------------------------------------------
    def _tick(self) -> None:
        if self.finished or self.stop_reason is not None:
            return
        now = time.monotonic()
        dt, self.last_tick = now - self.last_tick, now

        if self.joy is not None and now - self.joy_at > self.joy_timeout_s:
            if not self.warned_joy:
                self.warned_joy = True
                self.get_logger().warning(
                    f"/joy 가 {self.joy_timeout_s:g}초 넘게 안 옴 -> 속도 0. "
                    f"패드 선과 joy_node 를 볼 것")
            self.joy = None
            self.pressed = {k: False for k in BUTTONS}
            self.enable_since, self.enable_sent = None, False
        elif self.joy is not None:
            self.warned_joy = False

        held = self._button("deadman")
        for name in self.velocity_names:
            self.values[name] = (self._axis(self.axis_index[name]) * self.axes[name].max
                                 if held else 0.0)
        for name in self.height_names:
            # 높이는 0 으로 돌리면 안 됨 (바닥까지 내려가라는 뜻이 됨). RB 를 떼면
            # 지금 값을 그대로 내보내고 십자키만 안 먹게 함.
            a = self.axes[name]
            step = self._axis(self.axis_index[name]) * a.accel * dt if held else 0.0
            self.values[name] = min(max(self.values[name] + step, a.range[0]), a.range[1])
        self._publish(self.values)

        self._enable_hold(now)
        for name in self._edges():
            if name == "approach":
                if self._torque_on():
                    self._call("approach")
                else:
                    self.get_logger().warning("A: 토크가 꺼져 있음. 먼저 View 1초 -- 안 보냄")
            elif name == "start_policy":
                if held:
                    # ready 가 아니면 정책 노드가 거절하고 이유를 줌
                    self._call("start_policy")
                else:
                    self.get_logger().warning("Menu: RB 를 같이 누를 것 -- 안 보냄")
            elif name == "stop":
                self.stop_reason = "패드 B"

    def _enable_hold(self, now: float) -> None:
        """View 를 `enable_hold_s` 동안 누르고 있으면 `/huphy/enable` 을 한 번 부름."""
        if not self._button("enable"):
            self.enable_since, self.enable_sent = None, False
            return
        if self.enable_sent:
            return                      # 이번에 누르고 있는 동안 이미 보냄. 떼야 다시 셈
        if self.enable_since is None:
            self.enable_since = now     # 눌리기 시작한 주기
            return
        if now - self.enable_since >= self.enable_hold_s:
            self.enable_sent = True
            self._call("enable")

    # ---- 내보내는 것 -------------------------------------------------------
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


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = GamepadNode()
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
