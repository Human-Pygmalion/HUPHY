"""정책 꾸러미(`deploy_contract.json` + `.pt`) 하나를 읽어 그대로 돌리는 공통 실행기.

**정책마다 다른 값은 전부 계약 파일에서 옴.** 이 파일에는 정책 이름도 숫자도 없음.

    config/policies/<이름>/
      deploy_contract.json     입출력 명세 (학습 쪽 export_deploy.py 가 만듦)
      <체크포인트>.pt           가중치. 계약의 checkpoints.<번호>.pt


## 계약에서 읽는 것

    control_rate_hz            정책 주기
    actions[]                  관절 순서 / 배율 / 오프셋(=기본자세) / 자르기 범위
    observations[]             관찰 항목 순서. `func` 이름으로 아래 OBSERVATIONS 표에서 찾음
    checkpoints.<번호>         .pt 파일 이름, sha256, 참조 입출력(golden)
    h_star                     높이 명령의 범위·시작값 (`height_command` 가 있을 때만)


## 코드에 남는 것 (정책과 무관)

    관절 이름 규칙     R_crank_A_joint -> right_leg/ankle_a   (`robot_name`)
    단위              계약은 rad, 제어 루프는 도. 경계에서 한 번 바꿈
    관찰 함수 표       func 이름 -> 계산 (`OBSERVATIONS`)

새 관찰 종류가 계약에 오면 **모터를 켜기 전에 멈춤.** `OBSERVATIONS` 에 함수 하나를
더하면 됨.


## 한 주기

    observation(도) + IMU + 명령
      -> observation_vector()     계약 순서대로 이어 붙임 (rad)
      -> model()                  rsl_rl.load 가 만든 함수 (정규화 포함)
      -> joint_targets()          오프셋 + 배율 x 행동, 자르기, 도로 바꿈
"""

from __future__ import annotations

import hashlib
import json
import logging
import math
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

import numpy as np

from . import rsl_rl
from .policy import PolicySpec

logger = logging.getLogger("huphy.deploy")

CONTRACT_NAME = "deploy_contract.json"

GOLDEN_TOL = 1e-4
"""참조 입출력 허용치. 학습 쪽 `policy_runner.self_test` 와 같은 값."""

H_STAR_MAX_RATE = 0.1
"""h* 를 바꾸는 최대 속도 [m/s]. 계약에 `h_star.max_rate_m_s` 가 없을 때 씀.

squat 꾸러미 README 4절의 권장값 ("실기에서는 초당 0.1 m 이하로", `example_loop.py`
의 `H_RATE`)."""


class ContractError(ValueError):
    """계약 파일이 이 실행기가 다룰 수 있는 모양이 아님."""


# ===========================================================================
# 관절 이름 — 학습 쪽 이름 -> robot.yaml 이름
# ===========================================================================
SIDES = {"R": "right_leg", "L": "left_leg"}
JOINTS = {
    "hip_pitch": "hip_pitch",
    "hip_roll": "hip_roll",
    "hip_yaw": "hip_yaw",
    "knee": "knee",
    "crank_A": "ankle_a",
    "crank_B": "ankle_b",
}
_NAME = re.compile(r"^([RL])_(.+)_joint$")


def robot_name(contract_joint: str) -> str:
    """`R_crank_A_joint` -> `right_leg/ankle_a`. 모르는 이름이면 멈춤."""
    m = _NAME.match(contract_joint)
    if not m or m.group(2) not in JOINTS:
        raise ContractError(
            f"관절 이름 {contract_joint!r} 를 로봇 이름으로 못 바꿈. "
            f"'R_/L_' + {sorted(JOINTS)} + '_joint' 꼴이어야 함"
        )
    return f"{SIDES[m.group(1)]}/{JOINTS[m.group(2)]}"


# ===========================================================================
# 명령
# ===========================================================================
@dataclass(frozen=True)
class CommandSpec:
    """관찰에 들어가는 명령 한 칸. `huphy-run --cmd 이름=값` 의 이름."""

    name: str
    start: float
    """실행을 시작할 때의 값."""
    range: Optional[Tuple[float, float]] = None
    """학습 범위. 밖의 값은 잘라서 넣음. 없으면 자르지 않음."""
    max_rate: Optional[float] = None
    """초당 최대 변화. 있으면 `start` 에서 목표까지 이 속도로 감. 없으면 바로 목표."""


class Commands:
    """명령의 지금 값. 목표는 `set` 으로 바꾸고, `step` 이 한 주기만큼 다가감."""

    def __init__(self, specs: Sequence[CommandSpec], targets: Optional[Dict[str, float]] = None):
        self.specs = {s.name: s for s in specs}
        targets = dict(targets or {})
        unknown = sorted(set(targets) - set(self.specs))
        if unknown:
            raise ContractError(
                f"이 정책에 없는 명령: {unknown} (있는 것: {sorted(self.specs) or '없음'})"
            )
        self.targets = {n: float(targets.get(n, s.start)) for n, s in self.specs.items()}
        self.values = {
            n: (s.start if s.max_rate is not None else self.targets[n])
            for n, s in self.specs.items()
        }
        self.source: Optional[Callable[[], Dict[str, float]]] = None
        """있으면 매 `step` 마다 목표를 여기서 받음 (`teleop.Teleop.targets`)."""

    def set(self, name: str, value: float) -> None:
        if name not in self.specs:
            raise ContractError(f"이 정책에 없는 명령: {name}")
        self.targets[name] = float(value)
        if self.specs[name].max_rate is None:
            self.values[name] = float(value)

    def step(self, dt: float) -> Dict[str, float]:
        if self.source is not None:
            for name, value in self.source().items():
                self.set(name, value)
        for name, spec in self.specs.items():
            if spec.max_rate is None:
                self.values[name] = self.targets[name]
                continue
            gap = self.targets[name] - self.values[name]
            limit = spec.max_rate * dt
            self.values[name] += min(max(gap, -limit), limit)
        return dict(self.values)


# ===========================================================================
# 관찰 항목 — func 이름 -> 계산
# ===========================================================================
@dataclass
class Frame:
    """관찰 함수 하나가 받는 것. 한 주기에 한 번 만들어짐."""

    observation: Dict[str, Any]
    """`Leg/Biped.get_observation()`. `{로봇 관절}.pos` 가 도."""
    imu: Any
    """`ImuState`. `gyro_dps`(도/초), `gravity`(단위벡터)."""
    history: List[Dict[str, float]]
    """관절 위치 이력, **오래된 것이 앞**. 각 칸은 로봇 관절 이름 -> 도."""
    last_action: Sequence[float]
    command: Dict[str, float]


@dataclass(frozen=True)
class Term:
    """계약 `observations[]` 의 한 줄."""

    name: str
    func: str
    width: int
    joints: Tuple[str, ...] = ()
    """`motor_encoder_rel`·`last_action` 의 관절 순서 (로봇 이름)."""
    history: int = 1
    commands: Tuple[str, ...] = ()
    """명령 항목이면 이 칸들에 들어갈 명령 이름."""


def _gyro(term: Term, f: Frame, policy: "DeployPolicy") -> List[float]:
    return [math.radians(v) for v in f.imu.gyro_dps]


def _gravity(term: Term, f: Frame, policy: "DeployPolicy") -> List[float]:
    return [float(v) for v in f.imu.gravity]


def _encoder_rel(term: Term, f: Frame, policy: "DeployPolicy") -> List[float]:
    frames = f.history[-term.history:]
    while len(frames) < term.history:          # 시작 직후: 가장 오래된 것으로 앞을 메움
        frames = [frames[0]] + frames
    return [
        math.radians(frame[j] - policy.default_pose_deg[j])
        for frame in frames for j in term.joints
    ]


def _last_action(term: Term, f: Frame, policy: "DeployPolicy") -> List[float]:
    return [float(v) for v in f.last_action]


def _command(term: Term, f: Frame, policy: "DeployPolicy") -> List[float]:
    out = []
    for name in term.commands:
        value = float(f.command[name])
        spec = policy.commands[name]
        if spec.range is not None:
            value = min(max(value, spec.range[0]), spec.range[1])
        out.append(value)
    return out


OBSERVATIONS: Dict[str, Callable[[Term, Frame, "DeployPolicy"], List[float]]] = {
    "imu_gyro": _gyro,
    "imu_gravity": _gravity,
    "motor_encoder_rel": _encoder_rel,
    "last_action": _last_action,
    "height_command": _command,
    "velocity_command": _command,
}
"""계약의 `func` -> 계산 함수. **새 관찰 종류는 여기에 한 줄.**"""


def _term_names(term: Term) -> List[str]:
    """기록(`PolicyLog`)과 뷰어가 쓰는 칸 이름."""
    if term.func == "imu_gyro":
        return ["obs/gyro/x", "obs/gyro/y", "obs/gyro/z"]
    if term.func == "imu_gravity":
        return ["obs/grav/x", "obs/grav/y", "obs/grav/z"]
    if term.func == "motor_encoder_rel":
        out = []
        for i in range(term.history):
            lag = term.history - 1 - i
            tag = "q" if lag == 0 else ("q_prev" if lag == 1 else f"q_prev{lag}")
            out.extend(f"obs/{tag}/{j}" for j in term.joints)
        return out
    if term.func == "last_action":
        return [f"obs/last_act/{j}" for j in term.joints]
    if term.commands:
        return [f"obs/cmd/{n}" for n in term.commands]
    return [f"obs/{term.name}/{i}" for i in range(term.width)]


# ===========================================================================
# 계약 읽기
# ===========================================================================
@dataclass(frozen=True)
class DeployPolicy:
    """계약 하나를 읽은 결과. 상태 없음 -- 상태는 `Runner` 가 가짐."""

    name: str
    folder: Path
    hz: float
    order: Tuple[str, ...]
    """행동(=모델 출력) 순서. 로봇 관절 이름."""
    scale: Tuple[float, ...]
    default_pose_deg: Dict[str, float]
    """기본 자세 = 행동 오프셋. 시작 자세이자 관찰의 기준점."""
    limits_deg: Dict[str, Tuple[float, float]]
    """목표각 자르기 범위."""
    terms: Tuple[Term, ...]
    obs_dim: int
    commands: Dict[str, CommandSpec]
    checkpoints: Dict[str, Dict[str, Any]]
    default_checkpoint: str
    obs_names: Tuple[str, ...] = field(default=())

    @property
    def spec(self) -> PolicySpec:
        """`rsl_rl.load` 가 입력 개수를 대조할 때 쓰는 규격."""
        return PolicySpec(name=self.name, action_scale=self.scale[0], obs_dim=self.obs_dim)

    def default_pose(self) -> Dict[str, float]:
        """기본 자세 사본 (도)."""
        return dict(self.default_pose_deg)

    def weights_path(self, checkpoint: Optional[str] = None) -> Path:
        key = self._checkpoint_key(checkpoint)
        return self.folder / self.checkpoints[key]["pt"]

    def _checkpoint_key(self, checkpoint: Optional[str]) -> str:
        key = str(checkpoint) if checkpoint is not None else self.default_checkpoint
        if key not in self.checkpoints:
            raise ContractError(
                f"{self.name}: 체크포인트 {key} 가 없음 (있는 것: {sorted(self.checkpoints)})"
            )
        return key

    # ---- 가중치 ----------------------------------------------------------
    def load_model(self, checkpoint: Optional[str] = None):
        """가중치를 읽고 **모터를 켜기 전 검사**를 다 함.

            파일이 있음 / sha256 이 계약과 같음 / 입출력 개수 / 참조 입출력(있으면)
        """
        key = self._checkpoint_key(checkpoint)
        info = self.checkpoints[key]
        path = self.folder / info["pt"]
        if not path.is_file():
            raise ContractError(f"가중치 파일이 없음: {path}")
        want = info.get("pt_sha256")
        if want:
            got = _sha256(path)
            if got != want:
                raise ContractError(
                    f"{path.name} 의 sha256 이 계약과 다름 ({got[:16]}… != {want[:16]}…). "
                    f"계약과 가중치를 같은 꾸러미에서 가져올 것"
                )
        model = rsl_rl.load(path, spec=self.spec, action_dim=len(self.order))
        golden = info.get("golden")
        if golden:
            obs = np.asarray(golden["obs"], dtype=np.float32)
            expected = np.asarray(golden["actions"], dtype=np.float64)
            got = np.stack([np.asarray(model(o), dtype=np.float64) for o in obs])
            worst = float(np.abs(got - expected).max())
            if worst > GOLDEN_TOL:
                raise ContractError(
                    f"{self.name} {key}: 참조 입출력과 최대 {worst:.2e} 어긋남 "
                    f"(허용치 {GOLDEN_TOL:.0e}). 모터를 켜지 말 것"
                )
            model.golden_max_diff = worst        # type: ignore[attr-defined]
        model.checkpoint = key                   # type: ignore[attr-defined]
        model.path = path                        # type: ignore[attr-defined]
        return model

    # ---- 한 주기 계산 ----------------------------------------------------
    def observation_vector(self, frame: Frame) -> np.ndarray:
        out: List[float] = []
        for term in self.terms:
            values = OBSERVATIONS[term.func](term, frame, self)
            if len(values) != term.width:
                raise ValueError(f"{self.name}: {term.name} 이 {len(values)}칸인데 {term.width}칸이어야 함")
            out.extend(values)
        vector = np.asarray(out, dtype=np.float32)
        if not np.all(np.isfinite(vector)):
            raise ValueError(f"{self.name}: 관찰에 NaN/Inf 가 있음")
        return vector

    def joint_targets(self, action: Sequence[float]) -> Dict[str, float]:
        """모델 출력 -> 목표각(도).  목표 = 기본자세 + 배율 x 행동, 그 뒤 자름."""
        if len(action) != len(self.order):
            raise ValueError(f"{self.name}: 행동이 {len(action)}개인데 {len(self.order)}개여야 함")
        out: Dict[str, float] = {}
        for joint, scale, value in zip(self.order, self.scale, action):
            target = self.default_pose_deg[joint] + math.degrees(scale * float(value))
            low, high = self.limits_deg[joint]
            out[joint] = min(max(target, low), high)
        return out

    def history_length(self) -> int:
        return max([t.history for t in self.terms if t.func == "motor_encoder_rel"] or [1])

    def runner(self, model, *, commands: Optional[Commands] = None) -> "Runner":
        return Runner(self, model, commands or Commands(self.commands.values()))

    def motion(self, model, imu, *, commands: Optional[Commands] = None):
        """제어 루프(`ControlLoop`)가 받는 `Motion`. `huphy-run` 이 씀."""
        runner = self.runner(model, commands=commands)

        def step(t: float, observation: Dict[str, Any]) -> Optional[Dict[str, float]]:
            return runner.step(observation, imu.read())

        step.runner = runner          # type: ignore[attr-defined]
        return step


class Runner:
    """한 번 실행하는 동안의 상태: 관절 이력, 직전 행동, 명령."""

    def __init__(self, policy: DeployPolicy, model, commands: Commands) -> None:
        self.policy = policy
        self.model = model
        self.commands = commands
        self.reset()

    def reset(self) -> None:
        self.history: List[Dict[str, float]] = []
        self.last_action = [0.0] * len(self.policy.order)

    def step(self, observation: Dict[str, Any], imu_state: Any) -> Dict[str, float]:
        p = self.policy
        now = {j: float(observation.get(f"{j}.pos", 0.0)) for j in p.order}
        self.history.append(now)
        del self.history[:-p.history_length()]
        command = self.commands.step(1.0 / p.hz)
        frame = Frame(observation, imu_state, list(self.history), self.last_action, command)
        vector = p.observation_vector(frame)
        action = self.model(vector)
        self.last_action = [float(v) for v in action]
        return p.joint_targets(self.last_action)


def _sha256(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def _commands_from(contract: Dict[str, Any], term: Dict[str, Any]) -> List[CommandSpec]:
    """명령 항목 하나 -> 명령 칸들."""
    if term["func"] == "height_command":
        h = contract.get("h_star")
        if not h:
            raise ContractError("height_command 가 있는데 h_star 절이 없음")
        return [CommandSpec(
            name="h_star",
            start=float(h["default_pose_height_m"]),
            range=(float(h["range_m"][0]), float(h["range_m"][1])),
            max_rate=float(h.get("max_rate_m_s", H_STAR_MAX_RATE)),
        )]
    # velocity_command
    ranges = term.get("ranges", {})
    out = []
    for name in term["components"]:
        r = ranges.get(name)
        out.append(CommandSpec(name=name, start=0.0,
                               range=(float(r[0]), float(r[1])) if r else None))
    return out


def load(path: "str | Path") -> DeployPolicy:
    """꾸러미 폴더(또는 계약 파일)를 읽음. 어긋나면 `ContractError` -- CAN 을 열기 전."""
    path = Path(path)
    file = path / CONTRACT_NAME if path.is_dir() else path
    if not file.is_file():
        raise ContractError(f"계약 파일이 없음: {file}")
    c = json.loads(file.read_text(encoding="utf-8"))
    name = str(c.get("package") or file.parent.name)

    actions = sorted(c["actions"], key=lambda a: a["index"])
    order = tuple(robot_name(a["joint"]) for a in actions)
    if len(order) != int(c["action_dim"]) or len(set(order)) != len(order):
        raise ContractError(f"{name}: actions 가 action_dim({c['action_dim']})과 다르거나 겹침")
    scale = tuple(float(a["scale_rad_per_unit"]) for a in actions)
    default = {j: math.degrees(float(a["offset_rad"])) for j, a in zip(order, actions)}
    limits = {
        j: (math.degrees(float(a["clip_rad"][0])), math.degrees(float(a["clip_rad"][1])))
        for j, a in zip(order, actions)
    }

    terms: List[Term] = []
    commands: Dict[str, CommandSpec] = {}
    for t in c["observations"]:
        func = t["func"]
        if func not in OBSERVATIONS:
            raise ContractError(
                f"{name}: 모르는 관찰 항목 {t['term']!r} (func={func!r}). "
                f"control/deploy.py 의 OBSERVATIONS 에 함수를 더할 것"
            )
        width = int(t["width"])
        history = max(1, int(t.get("history_frames") or 1))
        joints: Tuple[str, ...] = ()
        names: Tuple[str, ...] = ()
        if func == "motor_encoder_rel":
            joints = tuple(robot_name(j) for j in t["joint_names"])
            if len(joints) * history != width:
                raise ContractError(f"{name}: {t['term']} 폭 {width} != 관절 {len(joints)} x 프레임 {history}")
            missing = sorted(set(joints) - set(default))
            if missing:
                raise ContractError(f"{name}: 기본 자세를 모르는 관절 {missing}")
        if func == "last_action":
            joints = order
        if func in ("height_command", "velocity_command"):
            specs = _commands_from(c, t)
            names = tuple(s.name for s in specs)
            commands.update({s.name: s for s in specs})
        terms.append(Term(t["term"], func, width, joints, history, names))

    obs_dim = int(c["obs_dim"])
    if sum(t.width for t in terms) != obs_dim:
        raise ContractError(f"{name}: 관찰 항목 폭의 합 {sum(t.width for t in terms)} != obs_dim {obs_dim}")

    checkpoints = {str(k): v for k, v in c["checkpoints"].items()}
    policy = DeployPolicy(
        name=name, folder=file.parent, hz=float(c["control_rate_hz"]),
        order=order, scale=scale, default_pose_deg=default, limits_deg=limits,
        terms=tuple(terms), obs_dim=obs_dim, commands=commands,
        checkpoints=checkpoints,
        default_checkpoint=str(c.get("default_checkpoint", next(iter(checkpoints)))),
    )
    obs_names = tuple(n for t in terms for n in _term_names(t))
    object.__setattr__(policy, "obs_names", obs_names)
    return policy
