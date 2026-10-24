"""`PolicySpec` 과 범용 관절 순서 상수 — 어느 정책과도 무관한 공유 조각.

과거엔 0.5의 `balance`/`hopping` 관찰 빌더(`observation_vector`, `BALANCE`,
`HOPPING`, `policy_motion` 등)도 여기 있었음. 그 정책들은 더 이상 돌리지 않아
지웠음 -- 지금은 정책마다 `config/policies/<이름>/deploy_contract.json` 이 규격이고
`deploy.py` 가 읽음.

남은 것 둘은 그 정책들과 무관하게 **지금도 쓰임**:

    PolicySpec          `rsl_rl.load`가 가중치 파일과 대조하는 규격 자료형
    JOINT_ORDER 등       `huphy-test pose`가 발목 공간(`--ankle-space`) 이름과
                         순서를 사람이 입력할 때 쓰는 범용 관절 순서 표기 --
                         특정 RL 정책과 무관한, 이 저장소 전체의 관절 이름 규약임
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Tuple


@dataclass(frozen=True)
class PolicySpec:
    """정책 하나의 규격. 시뮬 설정에서 그대로 옮긴 값임."""

    name: str
    action_scale: float
    """관절 목표 = 기본자세 + `action_scale` x 행동. 시뮬의 `JointPositionActionCfg`."""

    obs_dim: int
    """관찰 벡터 길이. 가중치 파일의 입력 차원과 같아야 함."""


JOINT_ORDER: Tuple[str, ...] = (
    "hip_pitch",
    "hip_roll",
    "hip_yaw",
    "knee",
    "ankle_pitch",
    "ankle_roll",
)
"""다리 하나의 관절 순서. 발목이 발판 자세(pitch/roll)임.

`huphy-test pose`가 `--ankle-space rp`일 때 숫자 나열 순서로 씀
(`scripts/selftest.py:pose_joints`).
"""

MOTOR_ORDER: Tuple[str, ...] = (
    "hip_pitch",
    "hip_roll",
    "hip_yaw",
    "knee",
    "ankle_a",
    "ankle_b",
)
"""다리 하나의 관절 순서. 발목이 모터 각도(ankle_a/ankle_b)임.

`huphy-test pose`가 `--ankle-space ab`(기본값)일 때 씀.
"""

ORDERS: Dict[str, Tuple[str, ...]] = {
    "rp": JOINT_ORDER,
    "ab": MOTOR_ORDER,
}
"""`--ankle-space` 값 -> 순서. **이 매핑은 여기에만 둠.**

진입점이 자기 사전을 들면 두 군데가 되고, 한쪽만 늘어나는 날이 옴.
"""

BIPED_LEGS: Tuple[str, ...] = ("left_leg", "right_leg")
"""`huphy-test --robot pose`가 양다리 12칸 숫자를 나열받을 때 쓰는 다리 순서.
**왼다리가 먼저임.**

이름은 `robot.yaml` 의 `limbs` 키와 같아야 함 (`Biped` 가 그 이름으로 명령을 나눔).
`Biped.joint_names`에서 순서를 끌어오면 안 됨 -- 그쪽은 `robot.yaml`에 적힌
순서를 따르는데 설정은 오른다리가 먼저일 수 있음. 이 상수는 설정과 무관하게
고정된, 사람이 숫자를 입력할 때 쓰는 규약임.
"""


def biped_order(leg_order: Tuple[str, ...]) -> Tuple[str, ...]:
    """다리 하나의 순서를 양다리 12칸으로. 이름 앞에 다리를 붙임.

        (hip_pitch, ..., ankle_roll)
          -> (left_leg/hip_pitch, ..., left_leg/ankle_roll,
              right_leg/hip_pitch, ..., right_leg/ankle_roll)

    구분자 `/` 는 `robots/biped.py` 의 것과 같음. `Biped.split_action` 이 첫 `/`
    앞을 다리 이름으로 봄.
    """
    return tuple(f"{leg}/{joint}" for leg in BIPED_LEGS for joint in leg_order)


BIPED_ORDERS: Dict[str, Tuple[str, ...]] = {
    space: biped_order(order) for space, order in ORDERS.items()
}
"""`--ankle-space` 값 -> 양다리 12칸 순서. `ORDERS` 에서 만들어 둘이 안 갈라짐."""
