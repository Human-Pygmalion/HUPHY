"""기준 모션 — 정책이 따라갈 관절 궤적 하나. 프레임 번호로 읽음.

`union` 같은 **모션 추종** 정책이 씀. 속도 명령 정책과 다른 점:

    속도 명령 정책   사람이 vx·wz 를 줌. 관찰의 명령 칸이 그 숫자임
    모션 추종 정책   파일에 든 궤적을 재생함. 관찰의 명령 칸이 그 프레임의 관절값임

그래서 사람이 줄 값이 없고, 대신 **몇 번째 프레임인지**가 상태임.


## 파일

꾸러미 폴더의 `.npz` 하나임. 다음 배열만 봄.

    joint_pos    (프레임, 관절)  rad.  절대값임 (기본자세 기준 상대가 아님)
    joint_vel    (프레임, 관절)  rad/s
    joint_names  (관절,)         학습 쪽 이름
    fps          스칼라          프레임 속도

`allow_pickle` 없이 읽음 -- 받은 파일을 푸는 것이라 pickle 을 켤 이유가 없음.


## 관찰에 들어가는 값

학습 쪽은 `cat([joint_pos[t], joint_vel[t]])` 를 그대로 명령 칸에 넣음
(릴리스의 `source/.../mdp/commands.py` 의 `MotionCommand.command`). **기본자세를
빼지 않음** -- 로봇 관절 관찰(`joint_pos_rel`)만 상대값임.


## 끝에 닿으면

학습은 에피소드를 다시 시작하지만 실물에는 다시 시작이 없음. `Clock` 은 **마지막
프레임에서 멈추고** `done` 을 세움. 되돌려 반복하지 않음 -- 이 궤적은 첫 프레임과
끝 프레임의 관절차가 최대 37.6도라 이어 붙이면 한 주기에 그만큼 뛰게 됨.

멈춘 뒤 무엇을 할지는 부르는 쪽이 정함 (ROS `motion_node` 는 정책을 멈춤).
"""

from __future__ import annotations

import hashlib
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import numpy as np


class ReferenceError(ValueError):
    """기준 모션 파일이 계약과 다름."""


def _sha256(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for block in iter(lambda: f.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


class ReferenceMotion:
    """기준 모션 하나. 읽기만 함 -- 프레임 번호는 `Clock` 이 가짐."""

    def __init__(self, path: "str | Path", *, sha256: Optional[str] = None,
                 joint_names: Optional[Sequence[str]] = None,
                 fps: Optional[float] = None) -> None:
        path = Path(path)
        if not path.is_file():
            raise ReferenceError(f"기준 모션 파일이 없음: {path}")
        if sha256:
            got = _sha256(path)
            if got != sha256:
                raise ReferenceError(
                    f"{path.name} 의 sha256 이 계약과 다름 ({got[:16]}… != {sha256[:16]}…). "
                    f"계약과 모션을 같은 꾸러미에서 가져올 것")
        data = np.load(path)            # allow_pickle 은 기본값(False) 그대로
        for key in ("joint_pos", "joint_vel"):
            if key not in data:
                raise ReferenceError(f"{path.name}: {key} 가 없음")
        self.path = path
        self.joint_pos = np.asarray(data["joint_pos"], dtype=np.float32)
        self.joint_vel = np.asarray(data["joint_vel"], dtype=np.float32)
        if self.joint_pos.shape != self.joint_vel.shape or self.joint_pos.ndim != 2:
            raise ReferenceError(
                f"{path.name}: joint_pos {self.joint_pos.shape} 와 joint_vel "
                f"{self.joint_vel.shape} 가 같은 2차원이어야 함")
        self.frames = int(self.joint_pos.shape[0])
        self.joints = int(self.joint_pos.shape[1])
        self.names: Tuple[str, ...] = tuple(
            str(n) for n in (data["joint_names"] if "joint_names" in data else ()))
        self.fps = float(np.asarray(data["fps"]).reshape(-1)[0]) if "fps" in data else 0.0

        if joint_names is not None:
            want = tuple(str(n) for n in joint_names)
            if self.names and self.names != want:
                bad = [i for i, (a, b) in enumerate(zip(self.names, want)) if a != b]
                raise ReferenceError(
                    f"{path.name}: 관절 순서가 계약과 다름 (처음 어긋난 칸 {bad[:3]}). "
                    f"순서가 밀리면 정책이 다른 관절의 기준값을 봄")
            if self.joints != len(want):
                raise ReferenceError(
                    f"{path.name}: 관절이 {self.joints}개인데 계약은 {len(want)}개임")
        if fps is not None and self.fps and abs(self.fps - fps) > 1e-6:
            raise ReferenceError(
                f"{path.name}: fps {self.fps} 가 계약의 {fps} 와 다름. "
                f"정책 주기와 프레임 속도가 같아야 1프레임씩 나아감")

    def __repr__(self) -> str:
        return (f"ReferenceMotion({self.path.name}, 프레임 {self.frames}, "
                f"관절 {self.joints}, {self.fps:g}fps)")

    @property
    def duration_s(self) -> float:
        return self.frames / self.fps if self.fps else 0.0

    def command(self, index: int) -> List[float]:
        """`[joint_pos[index], joint_vel[index]]` 을 이어 붙인 것. 관찰의 명령 칸임.

        `index` 는 **프레임 번호**임 (초가 아님). 범위 밖이면 멈춤 -- 끝을
        넘기는 것은 `Clock` 이 막고, 여기까지 오면 부르는 쪽 버그임.
        """
        if not 0 <= index < self.frames:
            raise ReferenceError(
                f"{self.path.name}: 프레임 {index} 가 범위 밖임 (0~{self.frames - 1})")
        return [float(v) for v in self.joint_pos[index]] + \
               [float(v) for v in self.joint_vel[index]]


class Clock:
    """재생 위치. 한 정책 주기에 한 프레임 나아감.

    `step()` 을 **관찰 벡터를 만든 뒤** 부를 것 -- 한 주기는 "지금 프레임을 보고
    행동을 낸다" 이고, 나아가는 것은 그 다음임.
    """

    def __init__(self, motion: ReferenceMotion, *, start: int = 0) -> None:
        self.motion = motion
        self.start = int(start)
        if not 0 <= self.start < motion.frames:
            raise ReferenceError(
                f"시작 프레임 {self.start} 가 범위 밖임 (0~{motion.frames - 1})")
        self.reset()

    def reset(self) -> None:
        self.index = self.start
        self.done = False
        """마지막 프레임까지 재생했음. **멈춘 뒤에도 `index` 는 마지막 프레임임.**"""

    def step(self) -> None:
        if self.index + 1 >= self.motion.frames:
            self.done = True
            return
        self.index += 1

    @property
    def progress(self) -> float:
        """0~1. 로그·상태 토픽에 씀."""
        last = self.motion.frames - 1
        return 1.0 if last <= 0 else min(1.0, self.index / last)
