"""pygame 창으로 정책 명령을 실시간 조작 — **별도 프로세스**로 돎.

    huphy-run --policy generalized_walk --teleop                   # walk: W/S 앞뒤, A/D 회전
    huphy-run --policy generalized_walk --teleop --max vx=0.8 --accel vx=0.4 --decel vx=1.5
    huphy-run --policy huphy_squat --teleop                # squat: ↑/↓ 높이


## 왜 별도 프로세스인가

제어 루프는 파이썬 스레드로 50Hz 이상을 지킴. pygame 을 같은 프로세스에서 돌리면 창을
그리고 이벤트를 처리하는 동안 GIL 을 쥐어 제어 루프가 몇 ms 씩 늦게 깰 수 있음.
프로세스를 나누면 GIL 도 CPU 코어도 따로 씀. 제어 루프가 하는 일은 매 정책 주기
공유 메모리에서 숫자 몇 개를 읽는 것뿐임 (`Teleop.targets`).

    [pygame 프로세스  _process]                    [huphy-run 제어 루프]
      키 누름/뗌 -> TeleopState.update (100Hz)
      -> 공유 메모리: 명령 값, 심장박동 시각  ──>   Commands.step 이 Teleop.targets() 를 읽음
      Enter / Esc -> 시작 / 멈춤 표시          ──>   teleop_motion 이 매 주기 확인


## 속도 명령 (vx, wz, ...) — 누르는 동안 적분, 떼면 0 으로

    누르고 있음     v += 방향 x accel x dt          길게 누를수록 빨라짐
    반대로 누름     v += 방향 x max(accel, decel) x dt   멈추는 쪽은 decel 로 빠르게
    뗌              v 를 decel x dt 씩 0 쪽으로      감쇠, 정해진 시간 안에 정확히 0
    항상            |v| <= max                       상한

키: vx W/S, wz A/D, vy Q/E (계약에 있는 것만). Space 는 속도를 전부 즉시 0.

## 높이 명령 (h_star)

↑/↓ 를 누르는 동안 목표가 `accel`(m/s, 기본 0.05)로 움직이고, 떼면 그 자리에 머묾.
학습 범위 밖으로는 안 감. 실제 값은 `Commands` 가 다시 0.1 m/s 로 따라감.


## 안전

* 창이 포커스를 잃으면 누른 키를 전부 뗀 것으로 봄 (KEYUP 이 안 올 수 있음).
* pygame 프로세스가 죽거나 멈춰서 심장박동이 `STALE_S` 넘게 끊기면, 제어 루프 쪽
  `Teleop.targets` 가 **속도 명령을 0** 으로 냄. 높이는 마지막 값을 유지함.
* 창을 닫거나 Esc·Ctrl-C·Ctrl-Q 를 누르면 `huphy-run` 이 멈춤. 창이 선택돼 있으면
  Ctrl-C/Ctrl-Q 가 터미널로 안 가고 이 창으로 오기 때문에 여기서도 받음.


## ROS 에서도 씀 (`ros/huphy_ros/huphy_ros/teleop_node.py`)

같은 창·키를 ROS 노드가 띄움. 그래서 두 가지가 더 있음 -- `huphy-run` 은 안 씀.

* **R** -- "기준 자세로" 요청 (`approach_count`). ROS 는 기준 자세와 정책 시작이 따로임.
* **상태 줄** -- 부모가 `set_status(코드)` 로 창 맨 위 글을 바꿈 (`STATUS_TEXT`). 0 이면
  예전처럼 Enter 여부만 보여 줌.

Enter·R 은 **누른 횟수**를 셈 (`start_count`, `approach_count`). ROS 에서는 멈췄다가 다시
시작할 수 있어서 한 번 켜지고 끝나는 표시로는 모자람. `start_requested` 는 예전 그대로
"한 번이라도 눌렀나" 임.
"""

from __future__ import annotations

import math
import multiprocessing as mp
import os
import signal
import time
from dataclasses import asdict, dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Set, Tuple

STALE_S = 0.2
"""심장박동이 이만큼 끊기면 속도 명령을 0 으로. 프로세스는 100Hz 로 씀."""

PROCESS_HZ = 100.0
DRAW_HZ = 20.0
READY_TIMEOUT_S = 10.0

VELOCITY_KEYS: Dict[str, Tuple[str, str]] = {
    "vx": ("w", "s"),
    "vy": ("q", "e"),
    "wz": ("a", "d"),
}
"""속도 명령 이름 -> (+ 키, - 키). pygame.key.name() 의 이름."""

HEIGHT_KEYS: Tuple[str, str] = ("up", "down")

VELOCITY_DEFAULTS = {"max": 1.0, "accel": 0.5, "decel": 1.0}
"""속도 명령 기본값. vx 는 m/s·m/s², wz 는 rad/s·rad/s². `--max/--accel/--decel` 로 바꿈."""

HEIGHT_DEFAULTS = {"accel": 0.05}
"""높이 명령: 누르는 동안 목표가 움직이는 속도 [m/s]."""


STATUS_TEXT: Dict[int, Tuple[str, Tuple[int, int, int]]] = {
    1: ("Torque OFF - run /huphy/enable first", (230, 120, 90)),
    2: ("R: go to ready pose", (230, 200, 90)),
    3: ("Moving to ready pose ...", (230, 200, 90)),
    4: ("Ready. Enter: start policy", (230, 200, 90)),
    5: ("POLICY RUNNING", (120, 220, 120)),
    6: ("No control_node (loop_status)", (230, 120, 90)),
    7: ("No policy_node (policy_status)", (230, 120, 90)),
}
"""`set_status` 코드 -> 창 맨 위 글. ROS 노드가 씀. 창 글자는 영어 (기본 글꼴이 한글을 못 그림)."""


class TeleopError(RuntimeError):
    """pygame 창을 못 띄움 등. 모터를 켜기 전에 남."""


@dataclass(frozen=True)
class Axis:
    """조작할 명령 하나."""

    name: str
    kind: str
    """'velocity' 또는 'height'."""
    keys: Tuple[str, str]
    """(+ 키, - 키)."""
    start: float
    max: float = 0.0
    accel: float = 0.0
    decel: float = 0.0
    range: Optional[Tuple[float, float]] = None


def make_axes(
    specs: Iterable,
    overrides: Optional[Dict[str, Dict[str, float]]] = None,
) -> List[Axis]:
    """`deploy.CommandSpec` 들 -> 조작 축. 이름으로 종류를 가름.

    `overrides` 는 {"max": {"vx": 0.8}, "accel": {...}, "decel": {...}} 꼴.
    키가 없는 명령이면 멈춤 -- 조작할 수 없는 칸이 있다는 뜻임.
    """
    overrides = overrides or {}
    axes: List[Axis] = []
    names = set()
    for spec in specs:
        names.add(spec.name)
        if spec.name in VELOCITY_KEYS:
            v = {k: float(overrides.get(k, {}).get(spec.name, d)) for k, d in VELOCITY_DEFAULTS.items()}
            if spec.range is not None:
                v["max"] = min(v["max"], max(abs(spec.range[0]), abs(spec.range[1])))
            for k, value in v.items():
                if not value > 0:
                    raise TeleopError(f"--{k} {spec.name} 는 0 보다 커야 함 (받은 값 {value})")
            axes.append(Axis(spec.name, "velocity", VELOCITY_KEYS[spec.name], 0.0,
                             v["max"], v["accel"], v["decel"], spec.range))
        elif spec.range is not None:
            accel = float(overrides.get("accel", {}).get(spec.name, HEIGHT_DEFAULTS["accel"]))
            if not accel > 0:
                raise TeleopError(f"--accel {spec.name} 는 0 보다 커야 함 (받은 값 {accel})")
            if spec.name in overrides.get("max", {}) or spec.name in overrides.get("decel", {}):
                raise TeleopError(f"{spec.name} 는 높이 명령이라 --accel 만 받음 (범위는 계약이 정함)")
            axes.append(Axis(spec.name, "height", HEIGHT_KEYS, spec.start,
                             accel=accel, range=spec.range))
        else:
            raise TeleopError(f"명령 {spec.name} 를 조작할 키가 없음")
    for kind, table in overrides.items():
        unknown = sorted(set(table) - names)
        if unknown:
            raise TeleopError(f"--{kind}: 이 정책에 없는 명령 {unknown} (있는 것: {sorted(names)})")
    if sum(a.kind == "height" for a in axes) > 1:
        raise TeleopError("높이 명령이 둘 이상이면 ↑/↓ 를 나눠 줄 수 없음")
    return axes


class TeleopState:
    """키 상태 -> 명령 값. pygame 과 무관한 순수 계산 (시험용으로 나눔)."""

    def __init__(self, axes: Sequence[Axis], start: Optional[Dict[str, float]] = None) -> None:
        self.axes = list(axes)
        start = start or {}
        self.values = {a.name: float(start.get(a.name, a.start)) for a in self.axes}

    def update(self, held: Set[str], dt: float) -> Dict[str, float]:
        for a in self.axes:
            plus, minus = a.keys
            direction = (plus in held) - (minus in held)
            v = self.values[a.name]
            if a.kind == "velocity":
                if direction == 0:
                    step = a.decel * dt
                    v = 0.0 if abs(v) <= step else v - math.copysign(step, v)
                else:
                    rate = max(a.accel, a.decel) if v * direction < 0 else a.accel
                    v += direction * rate * dt
                v = min(max(v, -a.max), a.max)
            else:
                v += direction * a.accel * dt
                v = min(max(v, a.range[0]), a.range[1])
            self.values[a.name] = v
        return dict(self.values)

    def stop_velocities(self) -> None:
        """Space: 속도 명령을 즉시 0."""
        for a in self.axes:
            if a.kind == "velocity":
                self.values[a.name] = 0.0


class Keys:
    """키 이벤트 -> 누른 키 목록과 시작·멈춤 요청. pygame 과 무관 (시험용으로 나눔)."""

    def __init__(self, state: TeleopState) -> None:
        self.state = state
        self.held: Set[str] = set()

    def handle(self, kind: str, name: str = "", ctrl: bool = False) -> Optional[str]:
        """`kind`: 'down' / 'up' / 'focus_lost' / 'quit'. 'start' 나 'stop' 을 내기도 함.

        `ctrl` 은 Ctrl 이 같이 눌렸는지. 창이 선택돼 있으면 Ctrl-C/Ctrl-Q 가 터미널이
        아니라 이 창으로 오므로, 여기서도 멈춤으로 처리함 (Esc 와 같음).
        """
        if kind == "quit":
            return "stop"
        if kind == "down" and ctrl and name in ("c", "q"):
            return "stop"
        if kind == "focus_lost":
            self.held.clear()            # 포커스를 잃으면 KEYUP 이 안 올 수 있음
            return None
        if kind == "up":
            self.held.discard(name)
            return None
        if name == "escape":
            return "stop"
        if name in ("return", "enter"):
            return "start"
        if name == "r":
            return "approach"
        if name == "space":
            self.state.stop_velocities()
            self.held.clear()
            return None
        self.held.add(name)
        return None


# ===========================================================================
# 공유 메모리 배치
# ===========================================================================
# [명령 값 n개][심장박동][시작 횟수][멈춤 요청][부모가 끝내라 함][준비됨(1)/실패(-1)]
# [기준 자세 요청 횟수][상태 코드 -- 부모가 씀]
HEARTBEAT, START, STOP, SHUTDOWN, READY, APPROACH, STATUS = range(7)
N_SLOTS = 7


def _slot(n: int, which: int) -> int:
    return n + which


def _process(axes_dicts: List[dict], start: Dict[str, float], shared, parent_pid: int, title: str) -> None:
    """pygame 프로세스 본체. spawn 으로 뜨므로 모듈 맨 위에 있어야 함."""
    axes = [Axis(**{**d, "keys": tuple(d["keys"]),
                    "range": tuple(d["range"]) if d["range"] else None}) for d in axes_dicts]
    n = len(axes)
    # Ctrl-C 는 부모(huphy-run)가 받음 -- 터미널은 같은 프로세스 그룹 전체에 보냄.
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    # SDL 은 기본으로 SIGINT/SIGTERM 을 가로채 QUIT 이벤트로 바꿈. 그러면 부모의
    # terminate() 로 안 죽음.
    os.environ["SDL_NO_SIGNAL_HANDLERS"] = "1"
    try:
        os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
        import pygame

        pygame.display.init()
        pygame.font.init()
        screen = pygame.display.set_mode((460, 60 + 30 * (n + 4)))
        pygame.display.set_caption(title)
        font = pygame.font.SysFont(None, 26)
    except Exception as e:                       # noqa: BLE001 -- 무엇이든 부모에게 알림
        shared[_slot(n, READY)] = -1.0
        print(f"[teleop] pygame 창을 못 띄움: {e}", flush=True)
        return

    state = TeleopState(axes, start)
    keys = Keys(state)
    period = 1.0 / PROCESS_HZ
    last = time.monotonic()
    next_draw = 0.0
    shared[_slot(n, READY)] = 1.0
    focus_lost = getattr(pygame, "WINDOWFOCUSLOST", None)

    try:
        while shared[_slot(n, SHUTDOWN)] == 0.0 and os.getppid() == parent_pid:
            for event in pygame.event.get():
                action = None
                if event.type == pygame.QUIT:
                    action = keys.handle("quit")
                elif event.type == pygame.KEYDOWN:
                    action = keys.handle("down", pygame.key.name(event.key),
                                         ctrl=bool(event.mod & pygame.KMOD_CTRL))
                elif event.type == pygame.KEYUP:
                    action = keys.handle("up", pygame.key.name(event.key))
                elif event.type == focus_lost or (
                    event.type == getattr(pygame, "ACTIVEEVENT", -1) and not getattr(event, "gain", 1)
                ):
                    action = keys.handle("focus_lost")
                if action == "start":
                    shared[_slot(n, START)] += 1.0
                elif action == "approach":
                    shared[_slot(n, APPROACH)] += 1.0
                elif action == "stop":
                    shared[_slot(n, STOP)] = 1.0

            now = time.monotonic()
            values = state.update(keys.held, min(now - last, 0.1))
            last = now
            for i, a in enumerate(axes):
                shared[i] = values[a.name]
            shared[_slot(n, HEARTBEAT)] = now

            if now >= next_draw:
                next_draw = now + 1.0 / DRAW_HZ
                _draw(pygame, screen, font, axes, values, keys.held,
                      shared[_slot(n, START)] != 0.0, int(shared[_slot(n, STATUS)]))
            time.sleep(max(0.0, period - (time.monotonic() - now)))
    finally:
        pygame.quit()


def _draw(pygame, screen, font, axes, values, held, started, status=0) -> None:
    screen.fill((24, 24, 28))
    # 창 글자는 영어 -- pygame 기본 글꼴이 한글을 못 그림.
    if status in STATUS_TEXT:
        lines = [STATUS_TEXT[status]]
    else:
        lines = [("POLICY RUNNING" if started else "Enter: start policy",
                  (120, 220, 120) if started else (230, 200, 90))]
    for a in axes:
        plus, minus = a.keys
        pressed = "+" if plus in held else ("-" if minus in held else " ")
        limit = f"±{a.max:g}" if a.kind == "velocity" else f"{a.range[0]:g}~{a.range[1]:g}"
        keys = f"{plus.upper()}/{minus.upper()}" if a.kind == "velocity" else "UP/DOWN"
        lines.append((f"{a.name:6s} {values[a.name]:+7.3f}  [{pressed}]  {keys}  ({limit})", (230, 230, 230)))
    if status:
        lines.append(("R: ready pose   Enter: start   Space: velocity 0", (160, 160, 170)))
        lines.append(("Esc / close: stop policy + torque off", (160, 160, 170)))
    else:
        lines.append(("Space: velocity 0    Esc / Ctrl-C / Ctrl-Q / close: stop", (160, 160, 170)))
    for i, (text, color) in enumerate(lines):
        screen.blit(font.render(text, True, color), (16, 16 + 30 * i))
    pygame.display.flip()


# ===========================================================================
# 부모(huphy-run) 쪽
# ===========================================================================
class Teleop:
    """pygame 프로세스를 띄우고, 제어 루프가 값을 읽게 함."""

    def __init__(self, axes: Sequence[Axis], start: Optional[Dict[str, float]] = None,
                 title: str = "huphy-run teleop") -> None:
        self.axes = list(axes)
        self.start_values = dict(start or {})
        self.title = title
        self.n = len(self.axes)
        self._ctx = mp.get_context("spawn")       # CAN·스레드가 있는 프로세스를 fork 하지 않음
        self.shared = self._ctx.Array("d", self.n + N_SLOTS, lock=False)
        self.process = None
        self._last_height = {a.name: float(self.start_values.get(a.name, a.start))
                             for a in self.axes if a.kind == "height"}

    def start(self, timeout: float = READY_TIMEOUT_S) -> "Teleop":
        """창이 뜰 때까지 기다림. 못 뜨면 `TeleopError` -- 모터를 켜기 전임."""
        for i, a in enumerate(self.axes):
            self.shared[i] = float(self.start_values.get(a.name, a.start))
        self.process = self._ctx.Process(
            target=_process,
            args=([{**asdict(a)} for a in self.axes], self.start_values, self.shared,
                  os.getpid(), self.title),
            daemon=True,
        )
        self.process.start()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            ready = self.shared[_slot(self.n, READY)]
            if ready == 1.0:
                return self
            if ready == -1.0 or not self.process.is_alive():
                self.close()
                raise TeleopError(
                    "pygame 창을 못 띄움. 화면(DISPLAY)이 있는 곳에서 실행하거나 ssh -X 로 "
                    "접속할 것. pygame 이 없으면 pip install pygame"
                )
            time.sleep(0.02)
        self.close()
        raise TeleopError(f"pygame 창이 {timeout:.0f}초 안에 안 뜸")

    def age(self) -> float:
        return time.monotonic() - self.shared[_slot(self.n, HEARTBEAT)]

    def targets(self) -> Dict[str, float]:
        """명령 목표. **제어 루프에서 매 정책 주기 부름** -- 숫자 몇 개 읽기뿐임.

        심장박동이 `STALE_S` 넘게 끊기면 속도는 0, 높이는 마지막 값.
        """
        if self.age() > STALE_S:
            out = {a.name: 0.0 for a in self.axes if a.kind == "velocity"}
            out.update(self._last_height)
            return out
        out = {a.name: float(self.shared[i]) for i, a in enumerate(self.axes)}
        for name in self._last_height:
            self._last_height[name] = out[name]
        return out

    def start_requested(self) -> bool:
        return self.shared[_slot(self.n, START)] != 0.0

    def start_count(self) -> int:
        """Enter 를 누른 횟수. ROS 노드가 늘어난 만큼 `start_policy` 를 부름."""
        return int(self.shared[_slot(self.n, START)])

    def approach_count(self) -> int:
        """R 을 누른 횟수. ROS 노드가 늘어난 만큼 `approach` 를 부름."""
        return int(self.shared[_slot(self.n, APPROACH)])

    def set_status(self, code: int) -> None:
        """창 맨 위 글을 바꿈 (`STATUS_TEXT`). 0 이면 예전 표시."""
        self.shared[_slot(self.n, STATUS)] = float(code)

    def stop_requested(self) -> bool:
        return self.shared[_slot(self.n, STOP)] != 0.0

    def help_line(self) -> str:
        parts = []
        for a in self.axes:
            if a.kind == "velocity":
                parts.append(f"{a.keys[0].upper()}/{a.keys[1].upper()} {a.name} "
                             f"(최대 {a.max:g}, 가속 {a.accel:g}, 감속 {a.decel:g})")
            else:
                parts.append(f"↑/↓ {a.name} ({a.accel:g}/s)")
        return ("  조작     pygame 창: " + ", ".join(parts)
                + ", Space 속도 0, Enter 시작, Esc·Ctrl-C·Ctrl-Q 멈춤")

    def close(self) -> None:
        if self.process is None:
            return
        self.shared[_slot(self.n, SHUTDOWN)] = 1.0
        self.process.join(timeout=2.0)
        if self.process.is_alive():
            self.process.terminate()
            self.process.join(timeout=1.0)
        if self.process.is_alive():
            self.process.kill()
            self.process.join(timeout=1.0)
        self.process = None


def teleop_motion(motion, teleop: Teleop, loop, staged_motion):
    """제어 루프에 넘기는 `Motion` 을 감쌈. 매 주기 시작·멈춤 요청을 봄 (읽기 두 번)."""

    def wrapped(t, observation):
        if teleop.stop_requested():
            loop.stop()
        elif teleop.start_requested() and not staged_motion.is_started():
            print("  시작함 (pygame Enter)")
            staged_motion.start()
        return motion(t, observation)

    return wrapped
