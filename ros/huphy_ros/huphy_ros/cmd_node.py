"""명령 입력 노드 — **디버깅·시험용.** 터미널에 칸을 띄워 명령을 숫자로 줌.

    ros2 run huphy_ros cmd_node --ros-args -p policy:=config/policies/newwalk

`teleop_node`·`gamepad_node` 와 **같은 자리**임 -- 같은 토픽(`/huphy/cmd_vel`,
`/huphy/h_star`)을 같은 주기로 내고 같은 서비스를 부름. 다른 것은 입력뿐으로, pygame
창도 패드도 없이 터미널 안에 칸을 그림 (`curses`). 화면이 없어도 되고 ssh 로도 됨.

`ros2 topic pub` 으로도 값은 줄 수 있지만, 값을 바꾸려면 Ctrl-C 하고 다시 쳐야 하고
멈추려면 또 다른 터미널에서 서비스를 불러야 함. 그 둘을 한 터미널로 모은 것임.

    huphy cmd — newwalk                        20Hz 로 보냄
    명령
    > vx [ 0.300]  < 1.05_        -1.500 .. 1.500
      vy [ 0.000]                 -1.000 .. 1.000
      wz [ 0.200]                 -1.000 .. 1.000
    상태
      control_node  control            policy  running
    키
      Tab 칸 이동   숫자 입력 후 Enter   j/k ∓0.05
      Space 속도 전부 0   e 토크   r 기준 자세   s 정책 시작
      Ctrl-C 끔 + 종료
    /huphy/start_policy: started

`>` 가 지금 칸, `<` 뒤가 치고 있는 글자임.

## 치는 법 -- Enter 가 필요한 것은 숫자 입력 하나뿐

숫자·`.`·`-` 는 칸에 글자로 쌓이고 **Enter 에 적용됨.** 그 전까지는 전에 적용된 값이
계속 나감. Backspace 로 지우거나 Tab 으로 칸을 옮기면 치던 것을 버림.

적용할 때 학습 범위를 보고 밖이면 거절함 -- 보내도 관찰에 들어가기 전에
`deploy._command` 가 자르므로, 치는 사람만 더 큰 값을 믿게 됨.

즉시 듣는 키.

    j / k    그 칸을 ∓`step` (속도 0.05, 높이 0.01)
    Space    속도 명령을 전부 0. **높이는 안 건드림** -- 0 은 바닥까지 내려가라는 뜻
    e        /huphy/enable          토크 넣기
    r        /huphy/approach        기준 자세로
    s        /huphy/start_policy    정책 시작

`0` 을 "전부 0" 으로 쓰지 않는 이유: 가장 많이 치는 값이 `0.3` 처럼 0 으로 시작해서,
첫 글자가 급정지로 먹힘. Space 는 숫자·소수점·음수 기호와 겹치지 않음.

Esc 도 안 씀. `curses` 는 방향키가 ESC 로 시작하는 바이트열이라 ESC 하나를 받으면
방향키인지 보려고 `ESCDELAY` 만큼 기다림 -- 급한 키에 둘 수 없음. 칸 이동도 Tab 만 씀.

## 끄기는 Ctrl-C 하나

속도 0 을 내고 `stop_policy` -> `disable` 을 부른 뒤 노드가 끝남. 그 응답을 기다리는
동안 **Ctrl-C 를 다시 누르면 기다리지 않고 바로 나감** -- 그때는 명령 토픽이 끊긴 것을
`control_node` 가 보고(`stop_on_input_loss`, 0.5초) 감쇠 5초 뒤 토크를 끊음.

그래서 **rclpy 의 Ctrl-C 처리를 안 씀** (`signal_handler_options=NO`). 그것을 쓰면
SIGINT 가 컨텍스트를 먼저 닫아 버려서, 그 뒤에 부르는 `stop_policy`·`disable` 이
선로에 나가지 않음 (실제로 안 나갔음). 대신 `main` 이 직접 SIGINT 를 받아 깃발만
세우고, 창 루프가 정상으로 빠져나온 뒤 `finish` 가 두 서비스를 부름.

정책만 끄고 다시 켜는 키는 없음. 껐다 켜려면 노드를 다시 띄움.

## 데드맨이 없음

패드는 RB 를 쥐고 있는 동안만 속도가 나가는데 타이핑에는 그런 것이 없음. 손을 떼도
로봇이 계속 그 명령으로 감. 그래서 이 노드는 값을 정확히 주고 싶은 디버깅·시험에
쓰고, 사람이 로봇을 조작하는 데는 패드를 씀.

**패드·teleop 창과 같이 띄우면 안 됨.** 같은 토픽에 둘이 번갈아 써서 내 값과 0 이 섞임.

## 왜 런치 파일이 없나

`ros2 launch` 는 띄운 프로세스에 터미널 입력을 넘겨 주지 않음. 이 노드는 그 입력이
일의 전부라 `ros2 run` 으로만 띄움.

## 한 스레드에서 돎

`curses` 는 스레드 안전하지 않아서 그리기·키 읽기를 ROS 처리와 같은 스레드에서 함
(`run_ui`). 키를 기다리지 않고(`nodelay`) 한 바퀴에 `spin_once` 한 번, 키 몇 개,
그리기 한 번임. 발행은 노드의 `rate_hz` 타이머가 하므로 그리기가 늦어도 명령은 나감.
"""

from __future__ import annotations

import signal
import time
from pathlib import Path
from typing import Dict, List, Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Float64
from std_srvs.srv import Trigger

from huphy.control import deploy

from . import logbridge

DEFAULT_HZ = 20.0
"""명령 토픽 주기. `teleop_node`·`gamepad_node` 와 같은 값.

아래로는 끊김 판정(`input_timeout_ms` 500, `cmd_timeout_s` 0.5초)보다 촘촘해야 하고,
위로는 정책이 자기 주기(50Hz)에 가장 최근 값 하나만 읽으므로 더 내도 버려짐."""

DRAW_HZ = 20.0
"""창을 다시 그리는 주기. 발행과 무관함."""

STEP = 0.05
"""`j`/`k` 가 속도 명령을 미는 폭 [m/s, rad/s]. `step` 파라미터로 바꿈."""

HEIGHT_STEP = 0.01
"""`j`/`k` 가 높이 명령을 미는 폭 [m]. `height_step` 파라미터로 바꿈."""

DECIMALS = 3
"""칸에 그리는 소수 자리. 값은 친 그대로 들어감 -- 그리기만 이 자리로 맞춤."""

STATUS_STALE_S = 1.0
"""상태 토픽이 이만큼 안 오면 그 노드가 없는 것으로 봄. 둘 다 5Hz 로 냄."""

HEIGHT_NAMES = ("h_star",)
"""0 으로 돌리면 안 되는 명령. Space 는 이 칸을 건드리지 않음."""

VALUE_CHARS = "0123456789.-"
"""칸에 글자로 쌓이는 키. 나머지는 명령 키임."""


def _key_lines(step: float, has_speed: bool) -> List[str]:
    """키 설명. 보여 주는 `j`/`k` 폭은 **지금 칸의** 폭임 (속도 0.05, 높이 0.01).

    속도 명령이 없는 정책(`squat`)에는 Space 줄을 안 띄움 -- 누를 칸이 없음.
    """
    second = "  e 토크   r 기준 자세   s 정책 시작"
    return [
        f"  Tab 칸 이동   숫자 입력 후 Enter   j/k ∓{step:g}",
        ("  Space 속도 전부 0" + second) if has_speed else second,
        "  Ctrl-C 끔 + 종료",
    ]


class CmdNode(Node):
    def __init__(self) -> None:
        super().__init__("huphy_cmd")
        self.log_handler = logbridge.forward(self.get_logger())

        self.declare_parameter("policy", "")
        self.declare_parameter("rate_hz", DEFAULT_HZ)
        self.declare_parameter("step", STEP)
        self.declare_parameter("height_step", HEIGHT_STEP)

        folder = self.get_parameter("policy").value
        if not folder:
            raise SystemExit("policy 파라미터에 정책 꾸러미 폴더를 줄 것 (policy_node 와 같은 것)")
        try:
            self.policy = deploy.load(Path(folder))
        except (deploy.ContractError, ValueError, OSError) as e:
            raise SystemExit(f"{e}") from e
        if not self.policy.commands:
            raise SystemExit(
                f"{self.policy.name}: 조작할 명령이 없는 정책임 (모션 추종 정책). "
                f"motion_node 를 띄우고 ros2 service call 이나 패드로 시작할 것")

        self.specs = dict(self.policy.commands)
        self.fields: List[str] = list(self.specs)
        """칸 순서. 정책 계약에 적힌 명령 순서 그대로임."""
        self.values: Dict[str, float] = {n: float(s.start) for n, s in self.specs.items()}
        self.velocity_names = [n for n in self.fields if n not in HEIGHT_NAMES]
        self.focus = 0
        self.buffer = ""
        """지금 칸에 치고 있는 글자. Enter 에 적용되고, 그때까지는 값이 안 바뀜."""
        self.message = ("Tab 으로 칸을 옮기고 숫자를 침. "
                        + ("Space 속도 0, " if self.velocity_names else "")
                        + "Ctrl-C 끔")
        """창 맨 아래 줄. 서비스 응답과 거절 이유가 여기 나옴."""

        self.step = float(self.get_parameter("step").value)
        self.height_step = float(self.get_parameter("height_step").value)
        hz = float(self.get_parameter("rate_hz").value)
        if not hz > 0:
            raise SystemExit(f"rate_hz 는 0 보다 커야 함 (받은 값 {hz})")
        self.hz = hz

        self.cmd_vel_pub = (self.create_publisher(Twist, "/huphy/cmd_vel", 10)
                            if self.velocity_names else None)
        self.h_star_pub = (self.create_publisher(Float64, "/huphy/h_star", 10)
                           if "h_star" in self.specs else None)

        self.mode: Optional[str] = None
        self.mode_at = 0.0
        self.stage: Optional[str] = None
        self.stage_at = 0.0
        self.create_subscription(DiagnosticArray, "/huphy/loop_status", self._on_loop, 10)
        self.create_subscription(DiagnosticArray, "/huphy/policy_status", self._on_policy, 10)

        self._triggers = {
            name: self.create_client(Trigger, f"/huphy/{name}")
            for name in ("enable", "approach", "start_policy", "stop_policy", "disable")
        }
        self.finished = False
        self.stop_requested = False
        """Ctrl-C 를 받았나. 창 루프가 보고 빠져나옴 (`run_ui`)."""
        self.create_timer(1.0 / hz, self._tick)

    # ---- 키 하나 ----------------------------------------------------------
    def key(self, key: str) -> None:
        """키 하나를 처리함. `run_ui` 가 `curses` 의 입력을 이 이름들로 바꿔 넘김.

        이름: 한 글자(`"j"`, `"0"`, ...) 또는 `"tab"` · `"enter"` · `"backspace"` ·
        `"space"`. 창을 안 띄우고 이 메서드만 불러 시험함 (`tests/test_ros_cmd.py`).
        """
        if key == "tab":
            self.buffer = ""                        # 치던 것은 버림
            self.focus = (self.focus + 1) % len(self.fields)
        elif key == "enter":
            self._apply()
        elif key == "backspace":
            self.buffer = self.buffer[:-1]
        elif key == "space":
            self._zero_speed()
        elif key in VALUE_CHARS:
            self.buffer += key
        elif key == "j":
            self._nudge(-1.0)
        elif key == "k":
            self._nudge(+1.0)
        elif key == "e":
            self._call("enable")
        elif key == "r":
            self._call("approach")
        elif key == "s":
            self._call("start_policy")
        else:
            self.message = f"{key!r} 는 쓰는 키가 아님"

    @property
    def field(self) -> str:
        return self.fields[self.focus]

    def _step_of(self, name: str) -> float:
        return self.height_step if name in HEIGHT_NAMES else self.step

    def _apply(self) -> None:
        """Enter. 치고 있던 글자를 지금 칸에 넣음."""
        text, self.buffer = self.buffer.strip(), ""
        if not text:
            return
        name = self.field
        try:
            value = float(text)
        except ValueError:
            self.message = f"{name}: 숫자가 아님 ({text!r})"
            return
        self._set(name, value)

    def _nudge(self, sign: float) -> None:
        """`j`/`k`. 치던 글자는 버리고 지금 값에서 한 폭 움직임."""
        self.buffer = ""
        name = self.field
        self._set(name, self.values[name] + sign * self._step_of(name))

    def _set(self, name: str, value: float) -> None:
        """범위를 보고 넣음. 밖이면 안 넣고 이유만 남김.

        범위 밖을 보내도 `deploy._command` 가 관찰에 넣기 전에 자름 -- 치는 사람만 더
        큰 값을 믿게 되므로 여기서 막음.
        """
        spec = self.specs[name]
        if spec.range is not None and not spec.range[0] <= value <= spec.range[1]:
            self.message = (
                f"{name}={value:.{DECIMALS}f} 는 학습 범위 "
                f"[{spec.range[0]:g}, {spec.range[1]:g}] 밖임 -- 안 바꿈")
            return
        self.values[name] = value
        self.message = f"{name} = {value:.{DECIMALS}f}"

    def _zero_speed(self) -> None:
        """Space. 속도만 0 으로. 높이는 그대로 (0 은 바닥까지 내려가라는 뜻이 됨)."""
        self.buffer = ""
        if not self.velocity_names:
            self.message = "속도 명령이 없는 정책임"
            return
        for name in self.velocity_names:
            self.values[name] = 0.0
        self.message = "속도 전부 0"

    # ---- 상태 -------------------------------------------------------------
    def _on_loop(self, msg: DiagnosticArray) -> None:
        if msg.status:
            self.mode, self.mode_at = msg.status[0].message, time.monotonic()

    def _on_policy(self, msg: DiagnosticArray) -> None:
        if msg.status:
            self.stage, self.stage_at = msg.status[0].message, time.monotonic()

    def _fresh(self, at: float, what: Optional[str]) -> str:
        if what is None or time.monotonic() - at > STATUS_STALE_S:
            return "없음"
        return what

    # ---- 그릴 내용 (curses 를 모름) ---------------------------------------
    def lines(self) -> List[str]:
        """창에 그릴 줄들. `curses` 없이 시험할 수 있게 글자만 만듦."""
        out = [f" huphy cmd — {self.policy.name}{'':8}{self.hz:g}Hz 로 보냄", " 명령"]
        for i, name in enumerate(self.fields):
            spec = self.specs[name]
            mark = ">" if i == self.focus else " "
            typing = f"  < {self.buffer}_" if i == self.focus and self.buffer else ""
            span = (f"{spec.range[0]:.{DECIMALS}f} .. {spec.range[1]:.{DECIMALS}f}"
                    if spec.range is not None else "범위 없음")
            out.append(f" {mark} {name:>6} [{self.values[name]:7.{DECIMALS}f}]"
                       f"{typing:<16}  {span}")
        out += [
            " 상태",
            f"   control_node  {self._fresh(self.mode_at, self.mode):<12}"
            f"   policy  {self._fresh(self.stage_at, self.stage)}",
            " 키",
        ]
        out += _key_lines(self._step_of(self.field), bool(self.velocity_names))
        out.append(f" {self.message}")
        return out

    # ---- 주기 -------------------------------------------------------------
    def _tick(self) -> None:
        self._publish(self.values)

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
        """서비스를 부르고 답을 맨 아래 줄에 남김. 기다리지 않음 -- 루프가 안 막히게."""
        client = self._triggers[name]
        if not client.service_is_ready():
            self.message = f"/huphy/{name} 서비스가 없음 -- 노드가 떠 있는지 볼 것"
            return None
        self.message = f"/huphy/{name} ..."
        future = client.call_async(Trigger.Request())

        def done(f, name=name):
            try:
                r = f.result()
            except Exception as e:  # noqa: BLE001
                self.message = f"/huphy/{name} 실패: {e}"
                return
            self.message = (f"/huphy/{name}: {r.message}" if r.success
                            else f"/huphy/{name} 거절: {r.message}")

        future.add_done_callback(done)
        return future

    def finish(self) -> None:
        """속도 0 -> 정책 끔 -> 토크 끔. **Ctrl-C 를 다시 누르면 기다리지 않고 나감.**

        그 경우 명령 토픽이 끊긴 것을 `control_node` 가 보고 감쇠 후 토크를 끊음.
        """
        self.finished = True
        for name in self.velocity_names:
            self.values[name] = 0.0
        self._publish(self.values)
        for name in ("stop_policy", "disable"):
            future = self._call(name)
            if future is None:
                continue
            try:
                rclpy.spin_until_future_complete(self, future, timeout_sec=8.0)
            except KeyboardInterrupt:
                self.get_logger().warning(
                    f"Ctrl-C -- /huphy/{name} 응답을 기다리지 않고 나감. "
                    f"명령이 끊겨 control_node 가 감쇠 후 토크를 끊음")
                return

    def close(self) -> None:
        try:
            for name in self.velocity_names:
                self.values[name] = 0.0
            self._publish(self.values)
        except Exception:  # noqa: BLE001 - 이미 컨텍스트가 닫혔을 수 있음
            pass


# ===========================================================================
# 창 -- 여기만 `curses` 를 앎
# ===========================================================================
def _key_name(ch: int, curses_mod) -> Optional[str]:
    """`curses` 의 키 번호 -> `CmdNode.key` 가 받는 이름. 모르는 키는 None."""
    if ch in (9,):
        return "tab"
    if ch in (10, 13, curses_mod.KEY_ENTER):
        return "enter"
    if ch in (8, 127, curses_mod.KEY_BACKSPACE):
        return "backspace"
    if ch == 32:
        return "space"
    if 32 < ch < 127:
        return chr(ch)
    return None


def run_ui(node: CmdNode) -> None:
    """`curses` 창을 띄우고 돎. 키 읽기·그리기·ROS 처리를 한 스레드에서 함."""
    import curses

    def loop(screen) -> None:
        curses.curs_set(0)
        screen.nodelay(True)
        drawn = 0.0
        while rclpy.ok() and not node.finished and not node.stop_requested:
            rclpy.spin_once(node, timeout_sec=1.0 / (4.0 * DRAW_HZ))
            while True:
                ch = screen.getch()
                if ch == -1:
                    break
                name = _key_name(ch, curses)
                if name is not None:
                    node.key(name)
            now = time.monotonic()
            if now - drawn >= 1.0 / DRAW_HZ:
                drawn = now
                _draw(screen, node)

    curses.wrapper(loop)


def _draw(screen, node: CmdNode) -> None:
    rows, cols = screen.getmaxyx()
    screen.erase()
    for i, line in enumerate(node.lines()):
        if i >= rows:
            break
        screen.addnstr(i, 0, line, max(cols - 1, 0))
    screen.refresh()


def main(argv=None) -> None:
    rclpy.init(args=argv, signal_handler_options=SignalHandlerOptions.NO)
    node = CmdNode()
    first = True

    def on_sigint(signum, frame):
        """첫 Ctrl-C 는 창 루프를 빠져나오게만 함 -- `finish` 가 두 서비스를 부름.
        두 번째는 평소대로 `KeyboardInterrupt` 를 올려 기다리지 않고 나감.
        """
        del signum, frame
        nonlocal first
        if first:
            first = False
            node.stop_requested = True
            return
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, on_sigint)
    try:
        run_ui(node)
        if node.stop_requested:
            node.finish()
    except KeyboardInterrupt:
        node.finish()
    finally:
        node.close()
        time.sleep(0.05)                    # 속도 0 이 선로에 나갈 틈
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
