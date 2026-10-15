"""웹에서 자세를 주고 동작을 엮어 돌리는 세션. `huphy-test studio`.

    FREE    토크 없음. 위치만 읽음 (관찰 모드)        시작 상태
    HOLD    토크 있음. 목표를 붙잡음
    RAMP    HOLD 에서 게인을 바꾸는 중 (RAMP_S 초)
    MOVING  지금 목표에서 새 자세까지 정한 시간에 옮김
    PLAYING 키프레임 목록을 차례로 (반복 가능)

    FREE --토크 켜기--> HOLD --자세--> MOVING --도착--> HOLD
                         HOLD --재생--> PLAYING --끝/정지--> HOLD
                         HOLD --게인--> RAMP --끝--> HOLD
    어디서든 --토크 끄기--> DAMPING (감쇠, 몇 초) --> FREE


## 요청은 큐로만 받음

웹 서버는 따로 스레드라, 거기서 토크를 켜거나 게인을 바꾸면 제어 루프와 **동시에
버스를 씀.** 그래서 웹은 `submit()` 으로 큐에 넣기만 하고, 제어 루프가 매 주기 앞
(`ControlLoop.before_step`)에서 꺼내 처리함. 화면에 보일 상태는 제어 스레드가 만든
사본을 웹이 읽음.


## 튀지 않게

- 토크를 켤 때 목표를 **지금 실측 자세**로 둠. 켜는 순간 당기는 힘이 없음
- 이동과 재생은 늘 **지금 목표**에서 출발함 (실측이 아님) -- 처져 있어도 목표가
  끊기지 않음
- 게인은 HOLD 에서만 바꾸고 RAMP_S 초에 걸쳐 바꿈. 목표와 실측이 벌어져 있을 때
  kp 를 한 번에 올리면 그 순간 힘이 몇 배가 됨


## 게인은 값으로

모터 이름(hip_pitch ... ankle_b)마다 kp/kd 를 **값 그대로** 받고 양다리에 같이 걺.
상한은 모터 통신 형식이 실을 수 있는 최댓값(`max_gains`, MIT 인코딩의 kp_max/kd_max)
뿐임 -- 넘으면 보내는 숫자가 잘려 적은 값과 다른 게인이 나감. `robot.yaml` 값은
처음 값과 화면의 기준값으로만 씀.
- 목표가 한계 밖이면 거부함 (가드가 자르기 전에). 가드는 그대로 걸려 있음


## 화면이 끊기면

이동·재생 중에 웹 요청이 `WATCHDOG_S` 초 동안 없으면 그 자리 목표로 HOLD. 화면을
닫으면 로봇이 혼자 계속 움직이지 않게 함. HOLD·FREE 는 그대로 둠.
"""

from __future__ import annotations

import json
import logging
import re
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple


logger = logging.getLogger(__name__)

RAMP_S = 0.3
"""게인을 바꾸는 데 쓰는 시간."""

WATCHDOG_S = 5.0
"""이동·재생 중 웹 요청이 이만큼 없으면 HOLD."""

MIN_MOVE_S = 0.2
"""이동 시간 하한. 0 이면 한 주기에 목표가 뛰어 점프 가드만 믿게 됨."""

FREE, HOLD, RAMP, MOVING, PLAYING = "FREE", "HOLD", "RAMP", "MOVING", "PLAYING"
DAMPING = "DAMPING"
"""토크를 끄기 전 감쇠 중 (`ControlLoop.DAMPING_S`). 이 동안은 요청을 처리하지 않음."""

_NAME = re.compile(r"^[A-Za-z0-9_\-가-힣]{1,64}$")


@dataclass
class Keyframe:
    pose: Dict[str, float]
    move_s: float
    hold_s: float = 0.0


class Studio:
    """웹 요청을 받아 제어 루프 안에서 실행하는 세션.

    `robot` 은 `set_gains` 를 가진 다리나 양다리. `joints` 는 자세에 쓰는 관절
    이름(발목은 모터각), `limits` 는 이름 -> (최소, 최대) 도.

    게인은 모터 이름(팔다리 없이) -> `(kp, kd)`. `base_gains` 는 기준값(robot.yaml),
    `max_gains` 는 상한, `gain_scale` 은 처음 값을 기준값의 몇 배로 할지.
    """

    def __init__(
        self,
        robot: Any,
        *,
        joints: Sequence[str],
        limits: Mapping[str, Tuple[float, float]],
        base_gains: Mapping[str, Tuple[float, float]],
        max_gains: Mapping[str, Tuple[float, float]],
        gain_scale: float = 1.0,
        sequences_dir: "str | Path" = "config/sequences",
    ) -> None:
        self.robot = robot
        self.joints = tuple(joints)
        self.limits = {j: (float(lo), float(hi)) for j, (lo, hi) in limits.items()}
        self.motors = tuple(base_gains)
        self.base_gains = {m: (float(kp), float(kd)) for m, (kp, kd) in base_gains.items()}
        self.max_gains = {m: (float(kp), float(kd)) for m, (kp, kd) in max_gains.items()}
        self.sequences_dir = Path(sequences_dir)

        self.state = FREE
        s = float(gain_scale)
        self.gains: Dict[str, Tuple[float, float]] = {
            m: (kp * s, kd * s) for m, (kp, kd) in self.base_gains.items()
        }
        self.target: Dict[str, float] = {}
        self.message = "토크 없음. 손으로 움직일 수 있음"

        self._queue: deque = deque()
        self._lock = threading.Lock()
        self._seen = time.monotonic()
        self._t = 0.0
        self._ramp: Optional[Tuple[float, Dict[str, Tuple[float, float]], Dict[str, Tuple[float, float]]]] = None
        self._move: Optional[Tuple[float, Dict[str, float], Dict[str, float], float]] = None
        self._play: Optional[Dict[str, Any]] = None
        self._status: Dict[str, Any] = {}
        self._apply_gains(self.gains)
        self._publish()

    # =======================================================================
    # 웹 쪽 (HTTP 스레드)
    # =======================================================================
    def submit(self, command: Dict[str, Any]) -> None:
        """요청을 큐에 넣음. 실행은 다음 주기에 제어 스레드가 함."""
        self.touch()
        self._queue.append(dict(command))

    def touch(self) -> None:
        """화면이 살아 있다는 표시. 상태를 물을 때마다 부름."""
        self._seen = time.monotonic()

    def status(self) -> Dict[str, Any]:
        self.touch()
        with self._lock:
            return dict(self._status)

    def layout(self) -> Dict[str, Any]:
        """바뀌지 않는 것. 관절 순서와 한계."""
        return {
            "joints": list(self.joints),
            "limits": {j: list(v) for j, v in self.limits.items()},
            "ramp_s": RAMP_S,
            "motors": list(self.motors),
            "base_gains": {m: list(v) for m, v in self.base_gains.items()},
            "max_gains": {m: list(v) for m, v in self.max_gains.items()},
            "watchdog_s": WATCHDOG_S,
            "min_move_s": MIN_MOVE_S,
        }

    # ---- 동작 파일 (CAN 을 안 건드리므로 HTTP 스레드에서 바로) -------------
    def list_sequences(self) -> List[str]:
        if not self.sequences_dir.is_dir():
            return []
        return sorted(p.stem for p in self.sequences_dir.glob("*.json"))

    def load_sequence(self, name: str) -> Dict[str, Any]:
        return json.loads(self._sequence_path(name).read_text(encoding="utf-8"))

    def save_sequence(self, name: str, data: Dict[str, Any]) -> str:
        keyframes = self._keyframes(data.get("keyframes"))   # 저장 전에 검사
        path = self._sequence_path(name)
        path.parent.mkdir(parents=True, exist_ok=True)
        body = {
            "name": name,
            "ankle_space": "ab",
            "repeat": int(data.get("repeat", 1)),
            "keyframes": [
                {"pose": k.pose, "move_s": k.move_s, "hold_s": k.hold_s} for k in keyframes
            ],
        }
        path.write_text(json.dumps(body, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        return str(path)

    def _sequence_path(self, name: str) -> Path:
        if not _NAME.match(name or ""):
            raise ValueError("이름은 영문·숫자·한글·_- 만, 64자까지")
        return self.sequences_dir / f"{name}.json"

    # =======================================================================
    # 제어 루프 쪽 (제어 스레드)
    # =======================================================================
    def before_step(self, loop: Any, t: float) -> None:
        """`ControlLoop.before_step` 에 꽂음. 요청 처리, 게인 램프, 끊김 감시."""
        self._t = t
        while self._queue:
            command = self._queue.popleft()
            try:
                self._handle(loop, command)
            except Exception as e:                     # 잘못된 요청은 거부만
                self.message = f"거부: {e}"
                logger.info("studio 요청 거부 %s: %s", command.get("op"), e)

        if self.state == RAMP:
            self._step_ramp(t)

        if self.state in (MOVING, PLAYING) and time.monotonic() - self._seen > WATCHDOG_S:
            self._enter_hold(f"화면 응답이 {WATCHDOG_S:g}초 없어 그 자리에 멈춤")

        self._publish()

    def motion(self, t: float, observation: Dict[str, Any]) -> Dict[str, float]:
        """제어 모드에서 매 주기 목표. 관찰 모드에서는 안 불림."""
        if self.state == MOVING and self._move is not None:
            self._step_move(t)
        elif self.state == PLAYING and self._play is not None:
            self._step_play(t)
        return dict(self.target)

    def row_targets(self) -> Optional[Dict[str, float]]:
        """기록 줄의 목표. 토크가 있으면 실제로 나간 것, 없으면 None (빈 칸)."""
        if self.state == FREE:
            return None
        sent = getattr(self.robot, "last_sent", None)
        return dict(sent) if sent else dict(self.target)

    # ---- 요청 ---------------------------------------------------------------
    def _handle(self, loop: Any, command: Dict[str, Any]) -> None:
        op = command.get("op")
        if op == "torque_on":
            self._require(FREE, "토크 켜기")
            pose = self._measured()
            self.target = pose
            loop.switch_to_control()
            self._enter_hold("토크 켬. 지금 자세를 붙잡음")
        elif op == "torque_off":
            if self.state == FREE:
                return
            self._move = self._play = self._ramp = None
            # 감쇠(DAMPING_S)하는 동안 제어 스레드가 여기 머묾 -- 먼저 알려 둠.
            self.state = DAMPING
            self.message = "감쇠 중. 끝나면 토크를 끊음"
            self._publish()
            loop.switch_to_observe()
            self.state = FREE
            self.message = "토크 끔 (감쇠 후). 손으로 움직일 수 있음"
        elif op == "gain":
            goal = self._gains(command.get("gains"))
            changed = ", ".join(
                f"{m} {goal[m][0]:g}/{goal[m][1]:g}" for m in self.motors if goal[m] != self.gains[m]
            ) or "바뀐 것 없음"
            if self.state == FREE:
                self._apply_gains(goal)                # 힘이 없으니 바로
                self.message = f"게인 {changed} (토크를 켜면 적용)"
                return
            self._require(HOLD, "게인 바꾸기")
            self._ramp = (self._t, dict(self.gains), goal)
            self.state = RAMP
            self.message = f"게인 {changed} ({RAMP_S:g}초에 걸쳐)"
        elif op == "move":
            self._require(HOLD, "자세 실행")
            goal = self._pose(command.get("pose"))
            seconds = _seconds(command.get("move_s"))
            self._move = (self._t, dict(self.target), {**self.target, **goal}, seconds)
            self.state = MOVING
            self.message = f"자세로 이동 ({seconds:g}초)"
        elif op == "play":
            self._require(HOLD, "재생")
            keyframes = self._keyframes(command.get("keyframes"))
            repeat = int(command.get("repeat", 1))
            if repeat < 0:
                raise ValueError("반복은 0(무한) 이상")
            self._play = {
                "frames": keyframes, "repeat": repeat, "round": 1, "index": 0,
                "phase": "move", "t0": self._t, "start": dict(self.target),
            }
            self.state = PLAYING
            self.message = f"재생 {len(keyframes)}개 · " + ("무한 반복" if repeat == 0 else f"{repeat}번")
        elif op == "stop":
            if self.state in (MOVING, PLAYING):
                self._enter_hold("멈춤. 그 자리를 붙잡음")
        else:
            raise ValueError(f"모르는 요청 {op!r}")

    def _require(self, state: str, what: str) -> None:
        if self.state != state:
            raise ValueError(f"{what} 은(는) {state} 에서만 됨 (지금 {self.state})")

    def _enter_hold(self, message: str) -> None:
        self._move = self._play = self._ramp = None
        self.state = HOLD
        self.message = message

    def _measured(self) -> Dict[str, float]:
        """토크를 켜기 직전의 실측 자세. 하나라도 없으면 켜지 않음."""
        observation = self.robot.get_observation()
        pose, missing = {}, []
        for joint in self.joints:
            value = observation.get(f"{joint}.pos")
            if value is None or value != value:
                missing.append(joint)
            else:
                pose[joint] = float(value)
        if missing:
            raise ValueError(f"위치를 못 읽은 관절이 있어 켜지 않음: {missing}")
        return pose

    def _pose(self, pose: Any) -> Dict[str, float]:
        """요청의 자세를 검사함. 모르는 관절, 한계 밖은 거부."""
        if not isinstance(pose, dict) or not pose:
            raise ValueError("자세가 비어 있음")
        out, outside = {}, []
        for joint, value in pose.items():
            if joint not in self.joints:
                raise ValueError(f"모르는 관절 {joint}")
            v = float(value)
            if v != v:
                raise ValueError(f"{joint} 값이 숫자가 아님")
            lo, hi = self.limits.get(joint, (-float("inf"), float("inf")))
            if not lo <= v <= hi:
                outside.append(f"{joint} {v:g} (한계 {lo:g}~{hi:g})")
            out[joint] = v
        if outside:
            raise ValueError("한계 밖: " + ", ".join(outside))
        return out

    def _gains(self, raw: Any) -> Dict[str, Tuple[float, float]]:
        """요청의 게인. `{모터: {kp, kd}}`, 준 것만 바꾸고 나머지는 지금 값. 상한 밖은 거부."""
        if not isinstance(raw, dict) or not raw:
            raise ValueError("게인이 비어 있음")
        out = dict(self.gains)
        bad = []
        for motor, value in raw.items():
            if motor not in self.gains:
                raise ValueError(f"모르는 모터 {motor}")
            kp, kd = float(value["kp"]), float(value["kd"])
            kp_max, kd_max = self.max_gains.get(motor, (float("inf"), float("inf")))
            if kp != kp or kd != kd or not (0.0 <= kp <= kp_max and 0.0 <= kd <= kd_max):
                bad.append(f"{motor} kp {kp:g} (0~{kp_max:g}) kd {kd:g} (0~{kd_max:g})")
            out[motor] = (kp, kd)
        if bad:
            raise ValueError("범위 밖: " + ", ".join(bad))
        return out

    def _keyframes(self, raw: Any) -> List[Keyframe]:
        if not isinstance(raw, list) or not raw:
            raise ValueError("키프레임이 없음")
        return [
            Keyframe(self._pose(k.get("pose")), _seconds(k.get("move_s")), max(0.0, float(k.get("hold_s", 0.0))))
            for k in raw
        ]

    # ---- 매 주기 -----------------------------------------------------------
    def _step_ramp(self, t: float) -> None:
        t0, start, goal = self._ramp
        f = min(1.0, (t - t0) / RAMP_S)
        self._apply_gains({
            m: (start[m][0] + (goal[m][0] - start[m][0]) * f, start[m][1] + (goal[m][1] - start[m][1]) * f)
            for m in goal
        })
        if f >= 1.0:
            self._enter_hold("게인 바꿈")

    def _step_move(self, t: float) -> None:
        t0, start, goal, seconds = self._move
        f = min(1.0, (t - t0) / seconds)
        self.target = _blend(start, goal, f)
        if f >= 1.0:
            self._enter_hold("도착. 붙잡음")

    def _step_play(self, t: float) -> None:
        p = self._play
        frame: Keyframe = p["frames"][p["index"]]
        goal = {**p["start"], **frame.pose}
        elapsed = t - p["t0"]
        if p["phase"] == "move":
            f = min(1.0, elapsed / frame.move_s)
            self.target = _blend(p["start"], goal, f)
            if f < 1.0:
                return
            p["phase"], p["t0"] = "hold", t
            elapsed = 0.0
        if elapsed < frame.hold_s:
            return
        # 다음 키프레임
        p["start"], p["t0"], p["phase"] = dict(self.target), t, "move"
        p["index"] += 1
        if p["index"] < len(p["frames"]):
            return
        if p["repeat"] and p["round"] >= p["repeat"]:
            self._enter_hold(f"재생 끝 ({p['round']}번)")
            return
        p["index"], p["round"] = 0, p["round"] + 1

    def _apply_gains(self, gains: Mapping[str, Tuple[float, float]]) -> None:
        from ..motors.base import Gains        # 제어 계층이 모터 자료형을 여기서만 앎

        self.gains = {m: (float(kp), float(kd)) for m, (kp, kd) in gains.items()}
        self.robot.set_gains({m: Gains(kp=kp, kd=kd) for m, (kp, kd) in self.gains.items()})

    def _publish(self) -> None:
        play = None
        if self._play is not None:
            p = self._play
            play = {
                "index": p["index"], "count": len(p["frames"]), "round": p["round"],
                "repeat": p["repeat"], "phase": p["phase"],
            }
        status = {
            "state": self.state,
            "torque": self.state != FREE,
            "gains": {m: [round(kp, 3), round(kd, 3)] for m, (kp, kd) in self.gains.items()},
            "target": {j: round(v, 2) for j, v in self.target.items()},
            "message": self.message,
            "play": play,
        }
        with self._lock:
            self._status = status


def _blend(start: Dict[str, float], goal: Dict[str, float], f: float) -> Dict[str, float]:
    return {j: start.get(j, goal[j]) + (goal[j] - start.get(j, goal[j])) * f for j in goal}


def _seconds(value: Any) -> float:
    v = float(value)
    if v < MIN_MOVE_S:
        raise ValueError(f"이동 시간은 {MIN_MOVE_S:g}초 이상 (받은 값 {v:g})")
    return v
