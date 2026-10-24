"""단위와 이름을 ROS 쪽 규약으로 바꿈. **`rclpy` 를 import 하지 않음.**

## 왜 한 파일에 모으나

바꾸는 것이 둘인데, 둘 다 틀려도 **길이와 자릿수가 그대로라 코드로는 안 잡힘.**

    단위   huphy 는 도,            ROS 는 라디안 (REP 103)
    이름   huphy 는 right_leg/knee, ROS 는 right_leg__knee

저장소 안에 도<->라디안 변환이 이미 두 곳 있음 -- 프로토콜 경계
(`motors/robstride/codec/mit.py`)와 모델 경계(`control/deploy.py`).
둘은 서로 안 만남. 이 파일이 세 번째이고 **ROS 경계 밖으로는 안 나감.**

## 구분자

`Biped` 는 `right_leg/knee` 처럼 `/` 로 팔다리를 앞에 붙임 (`robots/biped.py` 의
`SEP`). `sensor_msgs/JointState.name` 은 문자열 배열이라 `/` 가 들어가도 동작은
하는데, ROS 에서 `/` 는 네임스페이스 구분자라 사람이 토픽 이름과 헷갈림.

`SEP` 가 바뀌면 여기도 같이 바뀌어야 하므로 상수를 가져다 씀.
"""

from __future__ import annotations

import math
from typing import Dict, Iterable, List, Mapping, Sequence, Tuple

from huphy.robots.biped import SEP

ROS_SEP = "__"
"""ROS 쪽 관절 이름에서 팔다리와 관절 사이에 쓰는 구분자."""


def to_ros_name(name: str) -> str:
    """`right_leg/knee` -> `right_leg__knee`. 팔다리가 없으면 그대로."""
    return name.replace(SEP, ROS_SEP)


def to_huphy_name(name: str) -> str:
    """`right_leg__knee` -> `right_leg/knee`. 팔다리가 없으면 그대로."""
    return name.replace(ROS_SEP, SEP)


def joint_names(observation: Mapping[str, float]) -> Tuple[str, ...]:
    """관찰 사전에서 관절 이름을 뽑음. **huphy 이름**, 나온 순서대로.

    `.pos` 가 있는 것만 관절로 봄 -- `stale_motors` 같은 스칼라 항목을 걸러 냄
    (`robots/leg.py` 의 `get_observation`).
    """
    return tuple(k[: -len(".pos")] for k in observation if k.endswith(".pos"))


def observation_to_arrays(
    observation: Mapping[str, float],
    names: Sequence[str],
) -> Tuple[List[str], List[float], List[float], List[float]]:
    """관찰 사전 -> `JointState` 네 배열 `(이름, 위치, 속도, 토크)`.

    `names` 는 **huphy 이름**으로 주고, 반환되는 이름은 ROS 이름임.

        위치   도   -> rad
        속도   도/초 -> rad/s
        토크   Nm   (변환 없음)

    없는 항목은 `0.0` 으로 채움. `JointState` 는 네 배열의 길이가 같아야 하고 다르면
    받는 쪽이 짝을 못 맞춤 -- 0 이 옳은 값이라는 뜻이 아니라 **길이를 맞추는 것이
    필수**라서임. 값이 유효한지는 `stale_motors` 와 `loop_status` 로 봄.
    """
    return (
        [to_ros_name(n) for n in names],
        [math.radians(float(observation.get(f"{n}.pos", 0.0))) for n in names],
        [math.radians(float(observation.get(f"{n}.vel", 0.0))) for n in names],
        [float(observation.get(f"{n}.torque", 0.0)) for n in names],
    )


def arrays_to_observation(
    names: Iterable[str],
    position_rad: Iterable[float],
    velocity_rad_s: Iterable[float] = (),
) -> Dict[str, float]:
    """`JointState` 배열 -> 관찰 사전 (huphy 이름, 도).

    `policy_node` 가 씀. `deploy.Runner.step` 이 `{관절}.pos` 꼴로
    읽으므로 그 모양으로 되돌림.

    속도는 안 줘도 됨 -- 정책 관찰에 관절 속도 칸이 없음. 받으면 넣어 두되 쓰이지
    않음 (지금 정책들은 위치 두 프레임으로 대신함).
    """
    ros_names = list(names)
    pos = list(position_rad)
    if len(ros_names) != len(pos):
        raise ValueError(
            f"이름 {len(ros_names)}개, 위치 {len(pos)}개로 다름. "
            f"JointState 의 name 과 position 길이가 같아야 함"
        )
    out: Dict[str, float] = {}
    for name, value in zip(ros_names, pos):
        out[f"{to_huphy_name(name)}.pos"] = math.degrees(float(value))
    for name, value in zip(ros_names, velocity_rad_s):
        out[f"{to_huphy_name(name)}.vel"] = math.degrees(float(value))
    return out


def targets_to_arrays(
    targets: Mapping[str, float],
    names: Sequence[str],
) -> Tuple[List[str], List[float]]:
    """목표각 사전 (huphy 이름, 도) -> `(ROS 이름, 라디안)`.

    `names` 로 **순서를 고정**함. 사전 순서에 기대면 보내는 쪽이 바뀌었을 때 조용히
    달라짐 -- 받는 쪽이 `name` 을 보므로 결과는 같지만, 기록을 비교할 때 열이 섞임.
    """
    missing = [n for n in names if n not in targets]
    if missing:
        raise ValueError(f"목표각에 없는 관절: {missing}")
    return (
        [to_ros_name(n) for n in names],
        [math.radians(float(targets[n])) for n in names],
    )


def arrays_to_targets(
    names: Iterable[str],
    position_rad: Iterable[float],
) -> Dict[str, float]:
    """`JointState` 의 이름·위치 -> `Action` (huphy 이름, 도).

    `ControlLoop` 가 받는 `Motion` 의 반환값 모양임 (`control/loop.py` 의 `Motion`).

    **길이가 다르면 멈춤.** `zip` 은 짧은 쪽에 맞춰 조용히 잘라서, 관절 하나가 빠진
    명령이 그대로 통과함.
    """
    ros_names = list(names)
    values = list(position_rad)
    if len(ros_names) != len(values):
        raise ValueError(
            f"이름 {len(ros_names)}개, 위치 {len(values)}개로 다름. "
            f"JointState 의 name 과 position 길이가 같아야 함"
        )
    return {to_huphy_name(n): math.degrees(float(v)) for n, v in zip(ros_names, values)}


def imu_arrays(state) -> Tuple[List[float], List[float], List[float]]:
    """`ImuState` -> `(각속도 rad/s, 가속도 m/s^2, 중력방향 단위벡터)`.

    `sensors/base.py` 의 `ImuState` 는 각속도를 **도/초**로 들고 있음. 중력방향은
    이미 단위벡터라 변환이 없음.

    **중력방향은 `sensor_msgs/Imu` 에 자리가 없음.** 정책이 쓰는 값이 그 3칸이라
    (계약의 `imu_gravity` 칸), 쿼터니언으로 바꿨다 되푸는 왕복을 안 만들고
    `Vector3Stamped` 로 그대로 내보냄 (`ros/README.md` 4-1).
    """
    return (
        [math.radians(float(v)) for v in state.gyro_dps],
        [float(v) for v in state.accel_mps2],
        [float(v) for v in state.gravity],
    )
