"""학습한 정책을 `Motion` 으로 만듦.

모델은 관절 이름을 모름. **정해진 순서의 숫자 벡터**를 받고 같은 순서로 냄. 그
순서와 단위를 정하는 것이 이 파일의 일임.

    관찰 dict + IMU  ->  벡터  ->  모델  ->  벡터  ->  관절 목표 dict

`motions.py` 의 함수들과 같은 모양(`(t, observation) -> action`)이라 제어 루프가
그대로 받음. 텔레메트리·주기 측정·정지 순서가 같이 따라옴.


## 관찰 구성

시뮬(mjlab)이 이 순서로 이어 붙임. 가중치 파일의 입력 차원과 맞음.

    base_ang_vel        3    IMU 각속도 (rad/s)
    projected_gravity   3    중력 방향을 몸체 좌표로. IMU 모듈이 만들어 올림
    joint_pos           6    기본 자세 기준 상대 각도 (rad)
    joint_vel           6    (rad/s)
    actions             6    직전 주기에 낸 행동 그대로
    hop_phase           2    sin/cos. hopping 정책에만 있음

    balance  24    hopping  26


## 단위

    저장소   도, 도/초
    모델     라디안, rad/s

여기서 바꿈. 모델 경계가 단위가 갈리는 유일한 자리임.


## 정규화

학습 중에 쌓인 평균·표준편차가 가중치 파일에 같이 들어 있음.

    정규화된 입력 = (관찰 - mean) / std

**빼먹으면 모델이 전혀 다르게 동작함.** 모델을 부르는 쪽에서 적용해 넘길 것.


## 발목이 두 갈래임

마지막 두 칸이 무엇이냐가 모델마다 다름.

    JOINT_ORDER   ... ankle_pitch  ankle_roll    발판 자세. 실물에서 IK 를 거침
    MOTOR_ORDER   ... ankle_a      ankle_b       모터 각도. 그대로 나감

시뮬이 발목을 두 축 관절로 두느냐 두 모터로 두느냐에 달린 것이고, **가중치
파일에는 안 들어 있음** -- 관찰 개수도 행동 개수도 둘이 같아서(24/26, 6) 파일만
봐서는 구분되지 않음.

그래서 `PolicySpec` 에 넣지 않고 실행할 때 인자로 받음 (`huphy-run
--ankle-space`). 규격에 넣으면 같은 모델이 `balance_rp`/`balance_ab` 로 둘씩
늘어나는데, 정작 규격의 나머지 값(action_scale, obs_dim)은 똑같음.

**이름이 어긋나도 코드로는 안 잡힘.** 사람이 맞게 골라야 하고, `huphy-run` 이
시작 화면에 어느 쪽인지 찍음.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from typing import Any, Callable, Dict, Optional, Sequence, Tuple

import numpy as np

JOINT_ORDER: Tuple[str, ...] = (
    "hip_pitch",
    "hip_roll",
    "hip_yaw",
    "knee",
    "ankle_pitch",
    "ankle_roll",
)
"""모델이 보는 관절 순서. **시뮬과 같아야 함.**

이름은 모델에 안 들어감 -- 순서만 들어감. 여기가 어긋나면 값은 다 정상인데 로봇이
엉뚱하게 움직이고, 프레임도 응답도 정상이라 코드로는 안 잡힘.
"""

MOTOR_ORDER: Tuple[str, ...] = (
    "hip_pitch",
    "hip_roll",
    "hip_yaw",
    "knee",
    "ankle_a",
    "ankle_b",
)
"""발목을 모터로 학습한 모델의 순서. **앞 네 칸은 `JOINT_ORDER` 와 같음.**

발목만 다름 -- 발판 자세(pitch/roll)가 아니라 모터 각도임. 실물에서 기구학을
지나가지 않고 그대로 모터로 감.

길이가 `JOINT_ORDER` 와 같아서 **행동 개수로는 구분되지 않음.** 어느 쪽인지는
실행할 때 사람이 고름.
"""

ORDERS: Dict[str, Tuple[str, ...]] = {
    "rp": JOINT_ORDER,
    "ab": MOTOR_ORDER,
}
"""`--ankle-space` 값 -> 순서. **이 매핑은 여기에만 둠.**

진입점이 자기 사전을 들면 두 군데가 되고, 한쪽만 늘어나는 날이 옴.
"""

ACTION_DIM = len(JOINT_ORDER)
"""행동 개수. 두 순서가 같은 길이라 어느 쪽이든 6임.

검사는 `len(order)` 로 함 -- 길이의 근거는 이 상수가 아니라 쓰는 순서임.
"""

BIPED_LEGS: Tuple[str, ...] = ("left_leg", "right_leg")
"""양다리 모델이 다리를 늘어놓는 순서. **왼다리가 먼저임.**

학습 쪽과 정한 출력 규격임 -- 왼다리 6칸 다음에 오른다리 6칸. 이름은
`robot.yaml` 의 `limbs` 키와 같아야 함 (`Biped` 가 그 이름으로 명령을 나눔).

`Biped.joint_names` 에서 순서를 끌어오면 **안 됨.** 그쪽은 `robot.yaml` 에 적힌
순서를 따르는데 지금 설정은 오른다리가 먼저임. 모델 규격은 설정 파일이 아니라
학습이 정하는 것이라 여기 고정해 둠.
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


def observation_size(joint_count: int, *, uses_hop_phase: bool = False) -> int:
    """관찰 벡터 길이. 관절 수에서 계산함.

        각속도 3 + 중력방향 3 + 관절각 n + 관절속도 n + 직전행동 n (+ 위상 2)

    다리 하나(n=6)면 24/26, 양다리(n=12)면 42/44.

    **학습 쪽이 이 구성을 그대로 늘렸다는 가정임.** 다르게 짰으면 가중치 파일의
    입력 개수와 안 맞아 `rsl_rl.load` 가 모터를 켜기 전에 멈춤 -- 틀린 채로 돌지는
    않음.
    """
    return 3 + 3 + 3 * int(joint_count) + (2 if uses_hop_phase else 0)


@dataclass(frozen=True)
class PolicySpec:
    """정책 하나의 규격. 시뮬 설정에서 그대로 옮긴 값임."""

    name: str
    action_scale: float
    """관절 목표 = 기본자세 + `action_scale` x 행동. 시뮬의 `JointPositionActionCfg`."""

    obs_dim: int
    """관찰 벡터 길이. 가중치 파일의 입력 차원과 같아야 함."""

    uses_hop_phase: bool = False
    """`sin/cos` 위상 두 칸이 관찰 끝에 붙는지."""

    hop_period_s: float = 0.0
    """한 번 뛰는 데 걸리는 시간. `uses_hop_phase` 일 때만 씀."""


BALANCE = PolicySpec(name="balance", action_scale=0.25, obs_dim=24)
HOPPING = PolicySpec(
    name="hopping", action_scale=0.5, obs_dim=26, uses_hop_phase=True, hop_period_s=0.6
)


def for_joints(spec: PolicySpec, joint_count: int) -> PolicySpec:
    """같은 규격을 관절 수에 맞춰 늘림. `obs_dim` 만 다시 계산함.

    양다리 경로가 씀. `action_scale` 과 위상은 그대로 가져감 -- **학습 쪽이 양다리
    모델을 같은 값으로 학습했다는 가정임.** 양다리 규격이 따로 정해지면 그때
    `BALANCE` 같은 상수를 하나 더 두면 됨.
    """
    return replace(
        spec,
        obs_dim=observation_size(joint_count, uses_hop_phase=spec.uses_hop_phase),
    )


def hop_phase(t: float, period_s: float) -> Tuple[float, float]:
    """뛰는 주기 안에서 지금 어디인지. `(sin, cos)`.

    각도가 아니라 사인·코사인으로 주는 이유: 한 바퀴 돌 때 값이 튀지 않음. 위상이
    0.99 에서 0.0 으로 넘어가도 sin/cos 는 이어짐.
    """
    if period_s <= 0:
        return (0.0, 1.0)
    angle = 2.0 * math.pi * ((t / period_s) % 1.0)
    return (math.sin(angle), math.cos(angle))


def observation_vector(
    observation: Dict[str, Any],
    imu_state: Any,
    last_action: Sequence[float],
    *,
    spec: PolicySpec,
    order: Tuple[str, ...] = JOINT_ORDER,
    t: float = 0.0,
) -> np.ndarray:
    """관찰을 모델 입력 벡터로. 길이는 `spec.obs_dim`.

    `last_action` 은 **모델이 직전에 낸 값 그대로**임 -- 관절 목표로 바꾸기 전의
    것. 시뮬의 `last_action` 이 그것임.

    `order` 가 `MOTOR_ORDER` 면 발목 두 칸을 `ankle_a.pos` / `ankle_b.pos` 에서
    읽음. **둘 다 `Leg.get_observation()` 에 이미 있음** -- 모터 값이 기본이고
    발판 자세는 FK 로 덧붙인 것이라, 공간이 바뀌어도 관찰 쪽은 손댈 것이 없음.
    """
    out = []

    # 각속도. IMU 가 도/초로 냄.
    out.extend(math.radians(v) for v in imu_state.gyro_dps)

    # 중력방향은 센서 모듈이 이미 만들어 둔 것을 씀. 센서가 오일러를 주든
    # 쿼터니언을 주든 여기는 모름 -- 형식을 아는 쪽에서 계산해 올림.
    out.extend(imu_state.gravity)

    # 관절 각도·속도. 기본 자세가 전부 0 이라 상대 각도가 곧 각도임.
    out.extend(math.radians(float(observation.get(f"{j}.pos", 0.0))) for j in order)
    out.extend(math.radians(float(observation.get(f"{j}.vel", 0.0))) for j in order)

    out.extend(float(v) for v in last_action)

    if spec.uses_hop_phase:
        out.extend(hop_phase(t, spec.hop_period_s))

    vector = np.asarray(out, dtype=np.float32)
    if vector.size != spec.obs_dim:
        raise ValueError(
            f"{spec.name}: 관찰이 {vector.size}개인데 {spec.obs_dim}개여야 함. "
            f"관절 수나 hop_phase 설정이 시뮬과 다름"
        )
    return vector


def joint_targets(
    action: Sequence[float],
    *,
    spec: PolicySpec,
    order: Tuple[str, ...] = JOINT_ORDER,
) -> Dict[str, float]:
    """모델 출력 -> 목표 각도 (도).

        목표 = 기본자세 + action_scale x 행동

    기본 자세가 전부 0 이라 두 번째 항만 남음.

    **`order` 가 `MOTOR_ORDER` 면 키가 모터 이름으로 나옴** (`ankle_a`/`ankle_b`).
    이름을 붙이는 자리가 여기 하나뿐이라, 공간을 바꾸는 것이 곧 이 인자를 바꾸는
    것임. 받는 쪽(`Leg`)은 들어온 이름을 보고 알아서 갈림.
    """
    if len(action) != len(order):
        raise ValueError(
            f"{spec.name}: 행동이 {len(action)}개인데 {len(order)}개여야 함"
        )
    return {
        joint: math.degrees(spec.action_scale * float(value))
        for joint, value in zip(order, action)
    }


Model = Callable[[np.ndarray], Sequence[float]]
"""정규화까지 끝난 벡터를 받아 행동을 내는 것. 프레임워크를 가리지 않음."""


def policy_motion(
    model: Model,
    imu: Any,
    *,
    spec: PolicySpec = BALANCE,
    order: Tuple[str, ...] = JOINT_ORDER,
) -> Callable[[float, Dict[str, Any]], Optional[Dict[str, float]]]:
    """정책을 `Motion` 으로 만듦. 제어 루프가 그대로 받음.

    직전 행동을 들고 있음 -- 관찰에 들어가기 때문임. 첫 주기에는 0 임.

    **`t` 는 이 동작이 시작한 시점부터임.** 상태 기계가 상태별 경과 시간을 넘기므로
    뛰는 위상이 상태에 들어간 순간부터 셈.

    `order` 가 관찰과 목표 양쪽에 같이 걸림 -- 한쪽만 바꾸면 모델이 읽은 축과
    시킨 축이 어긋남.
    """
    last_action = [0.0] * len(order)

    def motion(t: float, observation: Dict[str, Any]) -> Optional[Dict[str, float]]:
        vector = observation_vector(
            observation, imu.read(), last_action, spec=spec, order=order, t=t
        )
        action = model(vector)
        last_action[:] = [float(v) for v in action]
        return joint_targets(last_action, spec=spec, order=order)

    return motion
