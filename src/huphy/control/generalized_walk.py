"""걷기 정책(`generalized_walk`) 전용 규격과 관찰 빌더.

HUPHY 1.0 양다리 12관절로 **속도 명령을 따라 걷는** 정책임. 이전의 `stand`(제자리
서기 전용, 45칸)를 대체함 -- `stand`/`balance`/`hopping`은 더 이상 돌리지 않으므로
지웠음. 이 정책이 지금부터의 정식 입출력 규격임.

출처: `maido-39/HuphyChan-dev` 저장소 `vibecoded-audit-20261012` 브랜치의
`deploy/generalized_walk/README.md`. 체크포인트 `terrain10_2048_model_37400.pt`
(SHA-256 `7bbd5cbc...fe0fbe00`, 확인함).


## 관찰 44칸

    0-2     각속도            rad/s
    3-5     중력방향          서 있으면 z 가 -1
    6-17    q(t-1) - 기본자세  rad
    18-29   q(t)   - 기본자세  rad
    30-41   직전 행동          모델이 낸 날것. 목표각이 아님
    42-43   속도 명령          (vx, wz). **vy 칸이 없음** -- stand 의 45칸에서 1개 줄어듦

`stand`와 구조는 같고 명령 칸만 다름(3개 -> 2개, vy 없음). 그래서 겹치는 부분(관절
순서, 이력 두 프레임, action_scale)은 숫자가 같아서 **옛 `stand.py`의 값을 그대로
가져옴** -- 복붙이 아니라 "지금 아는 가장 가까운 값"이라는 뜻. 기준 자세
(`DEFAULT_POSE_DEG`)와 한계(`LIMITS_DEG`)는 이 정책 고유의 실측값이 아직 없어서
임시로 옛 값을 씀 -- **실측값이 오면 이 두 표만 바꾸면 됨.**


## 명령은 고정값임 (지금은)

README의 하드웨어 시험 절차는 "0~5초 vx=0.5, 5~10초 vx=1.0" 같은 시간별 자동
전환을 요구했는데, 그건 여기서 구현하지 않음 -- 대신 `huphy-run --vx --wz`로
**사용자가 실행할 때 고정값을 줌.** 값을 바꾸려면 다시 실행하면 됨. 매 주기
새로 읽는 것이 아니라 `motion()`을 만들 때 한 번 받음.


## 발목

`ankle_a`/`ankle_b`를 직접 냄. 발판 자세(pitch/roll)가 아니므로 기구학을 지나가지
않음 -- `Leg`가 들어온 이름을 보고 알아서 갈림.
"""

from __future__ import annotations

import math
from typing import Any, Callable, Dict, Optional, Sequence, Tuple

import numpy as np

from .policy import PolicySpec

ORDER: Tuple[str, ...] = (
    "right_leg/hip_pitch",
    "right_leg/hip_roll",
    "right_leg/hip_yaw",
    "right_leg/knee",
    "right_leg/ankle_a",
    "right_leg/ankle_b",
    "left_leg/hip_pitch",
    "left_leg/hip_roll",
    "left_leg/hip_yaw",
    "left_leg/knee",
    "left_leg/ankle_a",
    "left_leg/ankle_b",
)
"""관찰과 행동에 공통으로 걸리는 순서. 오른다리 6칸 다음 왼다리 6칸.

배포 README가 명시한 순서임 ("right leg first, then left leg"). 이전 `stand.ORDER`와
우연히 같은 값임 -- 같은 로봇 모델(`pygmalion_mjlab/robot.py`)에서 나온 거라서임.
"""

DEFAULT_POSE_DEG: Dict[str, float] = {
    "right_leg/hip_pitch": -10.0268,
    "right_leg/hip_roll": 0.0,
    "right_leg/hip_yaw": 0.0,
    "right_leg/knee": 20.0535,
    "right_leg/ankle_a": 8.2480,
    "right_leg/ankle_b": -8.4261,
    "left_leg/hip_pitch": 10.0268,
    "left_leg/hip_roll": 0.0,
    "left_leg/hip_yaw": 0.0,
    "left_leg/knee": -20.0535,
    "left_leg/ankle_a": -8.2440,
    "left_leg/ankle_b": 8.4150,
}
"""기준 자세. 관절 이름 -> 도 (cal 공간).

**아직 이 정책 고유의 실측값이 아님** -- 옛 `stand.py`의 값을 그대로 가져온
것임(살짝 웅크린 자세, 엉덩이 10도·무릎 20도·발목 크랭크 8.3도). 이 정책의 진짜
기준 자세가 확인되면 이 표만 바꾸면 됨 -- 나머지 코드는 안 건드려도 됨.

**왼쪽과 오른쪽의 부호가 반대임.** 거울 규약이고 뒤집으면 로봇이 뒤로 감.

정책을 켜기 전에 로봇을 여기로 데려다 놔야 함.
"""

LIMITS_DEG: Dict[str, Tuple[float, float]] = {
    "right_leg/hip_pitch": (-112.7503, 17.7500),
    "right_leg/hip_roll": (-19.5000, 79.5000),
    "right_leg/hip_yaw": (-40.5000, 40.5000),
    "right_leg/knee": (6.0000, 114.0003),
    "right_leg/ankle_a": (-61.8794, 61.8794),
    "right_leg/ankle_b": (-61.8794, 61.8794),
    "left_leg/hip_pitch": (-17.7500, 112.7503),
    "left_leg/hip_roll": (-79.5000, 19.5000),
    "left_leg/hip_yaw": (-40.5000, 40.5000),
    "left_leg/knee": (-114.0003, -6.0000),
    "left_leg/ankle_a": (-61.8794, 61.8794),
    "left_leg/ankle_b": (-61.8794, 61.8794),
}
"""목표각을 자르는 범위. 관절 이름 -> (아래, 위) 도.

**이것도 아직 옛 `stand.py` 값임** -- 이 정책 고유의 범위가 확인되면 바꿀 것.
실물 한계는 캘리브레이션의 `limits_deg`이고 `safety/guards.py`가 따로 봄 -- 둘 중
좁은 쪽이 실제로 걸림.
"""

ACTION_SCALE = 0.25
"""행동 1이 기본자세에서 몇 라디안인지. README의 "target = default_pose + 0.25 x
action rad"와 같음. 12관절 전부 같은 값."""

HISTORY_FRAMES = 2
"""관찰에 들어가는 관절 위치 프레임 수. (`q(t-1)`, `q(t)`)."""

COMMAND_ZERO: Tuple[float, float] = (0.0, 0.0)
"""속도 명령 기본값. (vx m/s, wz rad/s). `vy`는 칸 자체가 없음 -- README: "It does
not have a vy observation slot."""

HZ = 50.0
"""정책이 불리는 주기. README: "policy/environment step 0.02 s, 50 Hz, decimation 4"."""

OBS_DIM = 3 + 3 + len(ORDER) * HISTORY_FRAMES + len(ORDER) + 2
"""관찰 길이. 44. stand(45)에서 명령 칸만 3->2로 줄어든 것."""

SPEC = PolicySpec(name="generalized_walk", action_scale=ACTION_SCALE, obs_dim=OBS_DIM)
"""`rsl_rl.load`가 입력 개수를 대조할 때 쓰는 규격."""


OBS_NAMES: Tuple[str, ...] = (
    ("obs/gyro/x", "obs/gyro/y", "obs/gyro/z", "obs/grav/x", "obs/grav/y", "obs/grav/z")
    + tuple(f"obs/q_prev/{j}" for j in ORDER)
    + tuple(f"obs/q/{j}" for j in ORDER)
    + tuple(f"obs/last_act/{j}" for j in ORDER)
    + ("obs/cmd/vx", "obs/cmd/wz")
)
"""관찰 44칸의 이름. `observation_vector`와 같은 순서임. 기록과 뷰어가 씀."""


def default_pose() -> Dict[str, float]:
    """기준 자세 사본. 접근 단계와 대기 자세가 이것을 씀."""
    return dict(DEFAULT_POSE_DEG)


def observation_vector(
    observation: Dict[str, Any],
    imu_state: Any,
    last_action: Sequence[float],
    previous_pos_deg: Sequence[float],
    command: Tuple[float, float] = COMMAND_ZERO,
) -> np.ndarray:
    """관찰을 모델 입력 44칸으로.

    `previous_pos_deg`는 **직전 정책 주기**의 관절 각도임(도, `ORDER` 순서).
    `command`는 (vx, wz) -- 매 주기 바뀌지 않고 실행 시작 때 고정값으로 줌.
    """
    out = []

    out.extend(math.radians(v) for v in imu_state.gyro_dps)
    out.extend(imu_state.gravity)

    now_deg = [float(observation.get(f"{j}.pos", 0.0)) for j in ORDER]
    frames = list(previous_pos_deg) + now_deg
    if len(frames) != len(ORDER) * HISTORY_FRAMES:
        raise ValueError(
            f"generalized_walk: 이력이 {len(frames)}개인데 "
            f"{len(ORDER) * HISTORY_FRAMES}개여야 함"
        )
    out.extend(
        math.radians(value - DEFAULT_POSE_DEG[ORDER[i % len(ORDER)]])
        for i, value in enumerate(frames)
    )

    out.extend(float(v) for v in last_action)
    out.extend(float(v) for v in command)

    vector = np.asarray(out, dtype=np.float32)
    if vector.size != OBS_DIM:
        raise ValueError(f"generalized_walk: 관찰이 {vector.size}개인데 {OBS_DIM}개여야 함")
    return vector


def joint_targets(action: Sequence[float]) -> Dict[str, float]:
    """모델 출력 -> 목표 각도 (도).

        목표 = 기본자세 + 배율 x 행동,  그 뒤 관절마다 자름
    """
    if len(action) != len(ORDER):
        raise ValueError(f"generalized_walk: 행동이 {len(action)}개인데 {len(ORDER)}개여야 함")
    out: Dict[str, float] = {}
    for joint, value in zip(ORDER, action):
        target = DEFAULT_POSE_DEG[joint] + math.degrees(ACTION_SCALE * float(value))
        low, high = LIMITS_DEG[joint]
        out[joint] = min(max(target, low), high)
    return out


def motion(
    model: Callable[[np.ndarray], Sequence[float]],
    imu: Any,
    *,
    command: Tuple[float, float] = COMMAND_ZERO,
) -> Callable[[float, Dict[str, Any]], Optional[Dict[str, float]]]:
    """정책을 `Motion`으로 만듦. 제어 루프가 그대로 받음.

    `command`는 이 호출 전체에서 고정임 -- `huphy-run --vx --wz`가 한 번 정해서 줌.

    첫 주기는 이력이 없으므로 **지금 위치로 앞을 메움**(`q(t-1) = q(t)`).
    """
    last_action = [0.0] * len(ORDER)
    previous: list = []

    def step(t: float, observation: Dict[str, Any]) -> Optional[Dict[str, float]]:
        now = [float(observation.get(f"{j}.pos", 0.0)) for j in ORDER]
        if not previous:
            previous[:] = now
        vector = observation_vector(observation, imu.read(), last_action, previous, command)
        action = model(vector)
        last_action[:] = [float(v) for v in action]
        previous[:] = now
        return joint_targets(last_action)

    return step
