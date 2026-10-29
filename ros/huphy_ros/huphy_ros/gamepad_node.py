"""패드 조작 노드 — 작업 기계에서 돎. Xbox 패드로 명령·시작·멈춤.

    ros2 launch huphy_ros gamepad.launch.py policy:=$HOME/HUPHY/config/policies/generalized_walk

`teleop_node` 와 내는 것이 같음 (`/huphy/cmd_vel`, `/huphy/h_star`, 서비스 넷).
다른 것은 입력뿐임 -- pygame 창 대신 `joy` 패키지의 `joy_node` 가 내는
`sensor_msgs/Joy` 를 `/joy` 에서 받음. `gamepad.launch.py` 가 둘을 같이 띄움.


## 조작 (버튼 번호는 연결 방식마다 다름 -- 아래 "배치")

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


## 배치 -- USB 와 블루투스의 번호가 다름

같은 패드라도 `joy_node` 가 내는 번호가 연결 방식마다 다름. **버튼 수로 가림** (`LAYOUTS`):

    배치        버튼 수  View  Menu  RB  A  B  오른쪽 스틱 가로
    usb         11       6     7     5   0  1  축 3     (커널 xpad 기준)
    bluetooth   16       10    11    7   0  1  축 2     (2026-10-29 Xbox Wireless 실측)

**둘 다 아니면 버튼·스틱을 전부 무시하고 속도 0 만 냄** -- 번호를 모르는 채로 쓰면 엉뚱한
버튼이 토크를 넣음 (블루투스로 붙였는데 usb 번호를 쓰면 LB 가 "View" 가 됐었음).
`layout:=usb|bluetooth` 로 고정하거나, `button_<이름>`/`axis_<이름>` 으로 칸 하나씩 덮음.

`game_controller_node` (SDL 표준 배치)는 안 씀 -- Ubuntu 22.04 의 SDL 2.0.20 이 블루투스
Series X 패드를 잘못 알아서 RB 가 안 잡히고 View 가 8번으로 나왔음.


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

AXES = ("vx", "vy", "wz", "h_star")
"""스틱·십자키로 움직이는 명령."""

BUTTONS = ("deadman", "approach", "start_policy", "stop", "enable")
"""버튼으로 하는 일. RB 데드맨, A 기준 자세, Menu 정책 시작, B 끔, View 토크."""

LAYOUTS = {
    "usb": {
        # 커널 xpad + joy_node(SDL joystick). 0 LX 1 LY 2 LT 3 RX 4 RY 5 RT 6 십자가로 7 십자세로
        "buttons_n": 11,
        "axes": {"vx": 1, "vy": 0, "wz": 3, "h_star": 7},
        # 0 A 1 B 2 X 3 Y 4 LB 5 RB 6 View 7 Menu 8 Xbox 9 LS 10 RS
        "buttons": {"deadman": 5, "approach": 0, "start_policy": 7, "stop": 1, "enable": 6},
    },
    "bluetooth": {
        # 2026-10-29 Xbox Wireless Controller 를 블루투스로 붙여 실측 (/joy 기록과
        # tools/gamepad_viewer.py 로 확인 -- 0~14 는 커널 번호와 같고 joy_node 가 15 를 더 붙임).
        # 축 (joy_node 는 위·왼쪽을 + 로 뒤집어 냄):
        #   0 왼쪽 가로  1 왼쪽 세로  2 오른쪽 가로  3 오른쪽 세로  4 RT  5 LT  6 십자 가로  7 십자 세로
        "buttons_n": 16,
        "axes": {"vx": 1, "vy": 0, "wz": 2, "h_star": 7},
        # 버튼: 0 A(아래)  1 B(오른쪽)  2 C(빈 칸)  3 X(왼쪽)  4 Y(위)  5 Z(빈 칸)  6 LB  7 RB
        #       8 LT 버튼  9 RT 버튼  10 View  11 Menu  12 Xbox  13 왼쪽 스틱 누름  14 오른쪽 스틱 누름
        #   C·Z 는 옛 게임패드 형식의 빈 자리라 USB 배치보다 X 부터 한 칸씩 밀림.
        "buttons": {"deadman": 7, "approach": 0, "start_policy": 11, "stop": 1, "enable": 10},
    },
}
"""연결 방식별 번호. `joy_node` 가 내는 버튼 수로 고름 (`_pick_layout`)."""


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
        self.declare_parameter("layout", "auto")
        for name in AXES:
            self.declare_parameter(f"axis_{name}", -1)      # -1 = 배치 표를 따름
        for name in BUTTONS:
            self.declare_parameter(f"button_{name}", -1)

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

        unmapped = sorted(set(self.axes) - set(AXES))
        if unmapped:
            raise SystemExit(
                f"{self.policy.name}: 패드에 배정할 자리가 없는 명령 {unmapped}. "
                f"배정된 것: {list(AXES)}")
        self.layout_name = str(self.get_parameter("layout").value)
        if self.layout_name not in ("auto", *LAYOUTS):
            raise SystemExit(f"layout 은 auto / {' / '.join(LAYOUTS)} 중 하나 (받은 값 {self.layout_name!r})")
        self.axis_override = {n: int(self.get_parameter(f"axis_{n}").value) for n in AXES}
        self.button_override = {k: int(self.get_parameter(f"button_{k}").value) for k in BUTTONS}
        self.layout: Optional[str] = None
        """지금 쓰는 배치. 첫 `/joy` 를 보고 정함. None 이면 번호를 몰라서 패드를 무시함."""
        self.axis_index: Dict[str, int] = {}
        self.button_index: Dict[str, int] = {}
        self._buttons_n: Optional[int] = None
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
            f"View {self.enable_hold_s:g}초 토크 넣기  -- 배치 {self.layout_name} "
            f"(첫 /joy 를 보고 정함)")

    # ---- 들어오는 것 -------------------------------------------------------
    def _on_joy(self, msg: Joy) -> None:
        if len(msg.buttons) != self._buttons_n:
            self._pick_layout(len(msg.buttons))
        self.joy, self.joy_at = msg, time.monotonic()

    def _pick_layout(self, buttons_n: int) -> None:
        """버튼 수로 배치를 고름. 모르는 수면 `layout` 을 None 으로 -- 패드를 무시함.

        도중에 바뀌면(블루투스를 끊고 USB 로 꽂음) 다시 고르고, 누른 상태를 전부 지움.
        """
        self._buttons_n = buttons_n
        self.pressed = {k: False for k in BUTTONS}
        self.enable_since, self.enable_sent = None, False
        if self.layout_name != "auto":
            name = self.layout_name
        else:
            name = next((n for n, l in LAYOUTS.items() if l["buttons_n"] == buttons_n), None)
        if name is None:
            self.layout = None
            known = ", ".join(f"{n} {l['buttons_n']}개" for n, l in LAYOUTS.items())
            self.get_logger().error(
                f"패드 버튼이 {buttons_n}개 -- 아는 배치가 아님 ({known}). "
                f"번호를 모르면 엉뚱한 버튼이 토크를 넣으므로 **패드를 무시함 (속도 0)**. "
                f"ros2 topic echo /joy 로 보고 layout:=… 이나 button_<이름>:=… 으로 지정할 것")
            return
        table = LAYOUTS[name]
        self.layout = name
        self.axis_index = {n: (self.axis_override[n] if self.axis_override[n] >= 0
                               else table["axes"][n]) for n in AXES}
        self.button_index = {k: (self.button_override[k] if self.button_override[k] >= 0
                                 else table["buttons"][k]) for k in BUTTONS}
        if table["buttons_n"] != buttons_n:
            self.get_logger().warning(
                f"layout:={name} 은 버튼 {table['buttons_n']}개 배치인데 패드는 {buttons_n}개")
        self.get_logger().info(
            f"패드 배치 {name} (버튼 {buttons_n}개): " + ", ".join(
                f"{k} {v}" for k, v in self.button_index.items())
            + "  축 " + ", ".join(f"{k} {v}" for k, v in self.axis_index.items()))

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
        if self.joy is None or self.layout is None or not 0 <= index < len(self.joy.axes):
            return 0.0
        v = float(self.joy.axes[index])
        return 0.0 if abs(v) < self.deadzone else v

    def _button(self, name: str) -> bool:
        if self.joy is None or self.layout is None:
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
            self.values[name] = (self._axis(self.axis_index.get(name, -1)) * self.axes[name].max
                                 if held else 0.0)
        for name in self.height_names:
            # 높이는 0 으로 돌리면 안 됨 (바닥까지 내려가라는 뜻이 됨). RB 를 떼면
            # 지금 값을 그대로 내보내고 십자키만 안 먹게 함.
            a = self.axes[name]
            step = self._axis(self.axis_index.get(name, -1)) * a.accel * dt if held else 0.0
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
