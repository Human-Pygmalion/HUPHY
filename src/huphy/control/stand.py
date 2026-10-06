"""서기 정책 (`legonly_ab_v9_caps`) 전용 규격과 관찰 빌더.

HUPHY 1.0 양다리 12관절을 **가만히 서 있게** 하는 정책임. 0.5 의
`balance`/`hopping` 과 **관찰 구성이 겹치지 않아** `policy.py` 에 분기를 넣지 않고
따로 둠.

                        balance / hopping      stand
    관절 속도           joint_vel n칸          안 씀
    관절 위치           1프레임                2프레임
    기본 자세           전부 0 (안 뺌)         0 이 아님. 빼서 넣음
    속도 명령           없음                   3칸
    관찰 길이           24 / 26                45

`policy.observation_vector` 에 `if` 를 네 개 넣으면 0.5 모델이 지나는 길도 같이
바뀜. 여기 따로 두면 그쪽은 한 줄도 안 바뀜.


## 값의 출처

학습 쪽 배포 꾸러미 `pyg_stand_v9_caps` 의 `deploy_contract.json` 임. 거기 적힌
라디안 값을 **도로 바꿔서** 옮겼음 -- 저장소 전체가 도이고, 라디안은 모델 경계에서만
씀.

관절 이름도 우리 이름으로 바꿔 적었음.

    R_hip_pitch_joint  ->  right_leg/hip_pitch
    L_crank_A_joint    ->  left_leg/ankle_a


## 관찰 45칸

    0-2     각속도            rad/s
    3-5     중력방향          서 있으면 z 가 -1
    6-17    q(t-1) - 기본자세  rad
    18-29   q(t)   - 기본자세  rad
    30-41   직전 행동          모델이 낸 날것. 목표각이 아님
    42-44   속도 명령          (vx, vy, wz). 서기는 전부 0

**이력은 오래된 것이 먼저임.** 관절별로 묶이지 않음 -- `[q(t-1) 전체 12]` 다음에
`[q(t) 전체 12]` 임. 길이가 같아서 뒤집어도 코드로는 안 잡힘.

30-41 은 **모델 출력 그대로**임. 배율과 기본자세를 입히기 전 값이라 관절 목표각을
넣으면 안 됨 -- 역시 길이가 12 로 같아서 안 잡힘.


## 발목

`ankle_a`/`ankle_b` 를 직접 냄. 발판 자세(pitch/roll)가 아니므로 기구학을 지나가지
않음 -- `Leg` 이 들어온 이름을 보고 알아서 갈림.
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
"""관찰과 행동에 **공통**으로 걸리는 순서. 오른다리 6칸 다음 왼다리 6칸.

0.5 의 `policy.BIPED_LEGS` 는 왼다리가 먼저인데 **이 정책은 반대임.** 둘은 서로 다른
학습에서 나온 규격이고, 같은 상수를 쓰면 안 됨.
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

**살짝 웅크린 자세임** -- 엉덩이 10도, 무릎 20도, 발목 크랭크 8.3도. 곧게 선 자세가
아님.

이 값이 두 곳에 걸림.

    관찰   q - 기본자세 를 넣음
    행동   기본자세 + 배율 x 행동 이 목표임

**왼쪽과 오른쪽의 부호가 반대임.** 거울 규약이고 뒤집으면 로봇이 뒤로 감.

정책을 켜기 전에 로봇을 여기로 데려다 놔야 함. 0 에서 시작하면 켜지는 순간 무릎
20도를 한 번에 메우려 함.
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

**학습 모형의 범위**이지 실물의 한계가 아님. 실물 한계는 캘리브레이션의 `limits_deg`
이고 `safety/guards.py` 가 따로 봄 -- 둘 중 좁은 쪽이 실제로 걸림.

발목 크랭크의 ±61.88 도는 학습 쪽에서도 실물 확인이 안 된 값임. 손으로 돌려 본
범위는 ±46 도였다고 함. **크게 움직이기 전에 하드스톱을 직접 볼 것.**
"""

ACTION_SCALE = 0.25
"""행동 1 이 기본자세에서 몇 라디안인지. 12관절 전부 같은 값임.

도로는 14.324 도. `PolicySpec.action_scale` 과 같은 뜻이라 `SPEC` 에도 들어감.
"""

HISTORY_FRAMES = 2
"""관찰에 들어가는 관절 위치 프레임 수. 지금 2 (`q(t-1)`, `q(t)`)."""

COMMAND_ZERO: Tuple[float, float, float] = (0.0, 0.0, 0.0)
"""속도 명령 기본값. (앞뒤 m/s, 좌우 m/s, 돌기 rad/s).

**제자리 서기는 전부 0 임.** 걷게 하려면 여기에 값이 들어가야 하는데, 이 정책은
서기로만 확인됐음.
"""

HZ = 50.0
"""정책이 불리는 주기. 학습이 20ms 를 전제로 돌았음.

**이력이 이 주기 기준임.** `q(t-1)` 은 20ms 전 값이어야 하므로, 전송 주기가 더
빨라도 이력은 정책이 불릴 때만 밀려야 함 (`run.py` 의 `held`).
"""

OBS_DIM = 3 + 3 + len(ORDER) * HISTORY_FRAMES + len(ORDER) + 3
"""관찰 길이. 45.

세어서 두는 이유: `ORDER` 나 `HISTORY_FRAMES` 를 고치면 같이 따라가고, 가중치
파일의 입력 개수와 어긋나면 `rsl_rl.load` 가 멈춤.
"""

SPEC = PolicySpec(name="stand", action_scale=ACTION_SCALE, obs_dim=OBS_DIM)
"""`rsl_rl.load` 가 입력 개수를 대조할 때 쓰는 규격.

`policy.for_joints` 를 **지나면 안 됨** -- 그것은 `3 + 3 + 3n` 으로 다시 계산해서
42 가 나옴. 이 정책은 속도 칸이 없고 이력이 둘이라 식이 다름.
"""


def default_pose() -> Dict[str, float]:
    """기준 자세 사본. 접근 단계와 대기 자세가 이것을 씀."""
    return dict(DEFAULT_POSE_DEG)


def observation_vector(
    observation: Dict[str, Any],
    imu_state: Any,
    last_action: Sequence[float],
    previous_pos_deg: Sequence[float],
) -> np.ndarray:
    """관찰을 모델 입력 45칸으로.

    `previous_pos_deg` 는 **직전 정책 주기**의 관절 각도임 (도, `ORDER` 순서).
    전송 주기가 아님 -- 100Hz 로 보내면서 50Hz 로 정책을 부르면 한 칸이 10ms 전
    값이 되어 학습(20ms)과 달라짐.

    `last_action` 은 모델이 직전에 낸 **날것의 값**임. 목표각이 아님.
    """
    out = []

    # IMU 각속도. 센서는 도/초로 올리고 모델은 rad/s 를 받음.
    out.extend(math.radians(v) for v in imu_state.gyro_dps)

    # 중력방향. 센서 모듈이 이미 만들어 둔 것을 씀 -- 서 있으면 z 가 -1 임.
    out.extend(imu_state.gravity)

    # 관절 위치 이력. **오래된 프레임이 먼저**이고 관절별로 묶이지 않음.
    now_deg = [float(observation.get(f"{j}.pos", 0.0)) for j in ORDER]
    frames = list(previous_pos_deg) + now_deg
    if len(frames) != len(ORDER) * HISTORY_FRAMES:
        raise ValueError(
            f"stand: 이력이 {len(frames)}개인데 "
            f"{len(ORDER) * HISTORY_FRAMES}개여야 함"
        )
    out.extend(
        math.radians(value - DEFAULT_POSE_DEG[ORDER[i % len(ORDER)]])
        for i, value in enumerate(frames)
    )

    out.extend(float(v) for v in last_action)
    out.extend(float(v) for v in COMMAND_ZERO)

    vector = np.asarray(out, dtype=np.float32)
    if vector.size != OBS_DIM:
        raise ValueError(f"stand: 관찰이 {vector.size}개인데 {OBS_DIM}개여야 함")
    return vector


def joint_targets(action: Sequence[float]) -> Dict[str, float]:
    """모델 출력 -> 목표 각도 (도).

        목표 = 기본자세 + 배율 x 행동,  그 뒤 관절마다 자름

    0.5 의 `policy.joint_targets` 와 두 가지가 다름 -- 기본자세 항이 있고, 자름이
    있음.
    """
    if len(action) != len(ORDER):
        raise ValueError(f"stand: 행동이 {len(action)}개인데 {len(ORDER)}개여야 함")
    out: Dict[str, float] = {}
    for joint, value in zip(ORDER, action):
        target = DEFAULT_POSE_DEG[joint] + math.degrees(ACTION_SCALE * float(value))
        low, high = LIMITS_DEG[joint]
        out[joint] = min(max(target, low), high)
    return out


def motion(
    model: Callable[[np.ndarray], Sequence[float]],
    imu: Any,
) -> Callable[[float, Dict[str, Any]], Optional[Dict[str, float]]]:
    """정책을 `Motion` 으로 만듦. 제어 루프가 그대로 받음.

    **두 가지를 들고 있음** -- 직전 행동과 직전 관절 위치. 둘 다 관찰에 들어가므로
    부르는 쪽이 챙기면 거의 반드시 하나를 틀림.

    첫 주기는 이력이 없으므로 **지금 위치로 앞을 메움** (`q(t-1) = q(t)`). 학습 쪽
    순환버퍼가 하는 것과 같음 -- 0 으로 두면 정책이 한 주기 만에 기본자세만큼
    움직였다고 읽음.
    """
    last_action = [0.0] * len(ORDER)
    previous: list = []

    def step(t: float, observation: Dict[str, Any]) -> Optional[Dict[str, float]]:
        now = [float(observation.get(f"{j}.pos", 0.0)) for j in ORDER]
        if not previous:
            previous[:] = now
        vector = observation_vector(observation, imu.read(), last_action, previous)
        action = model(vector)
        last_action[:] = [float(v) for v in action]
        previous[:] = now
        return joint_targets(last_action)

    return step
