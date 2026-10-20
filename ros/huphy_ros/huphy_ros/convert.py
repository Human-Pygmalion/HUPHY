"""단위와 이름을 ROS 쪽 규약으로 바꿈. **`rclpy` 를 import 하지 않음.**

## 왜 한 파일에 모으나

바꾸는 것이 둘인데, 둘 다 틀려도 **길이와 자릿수가 그대로라 코드로는 안 잡힘.**

    단위    huphy 는 도,   ROS 는 라디안 (REP 103)
    이름    huphy 는 right_leg/knee,   ROS 는 구분자를 바꿀 수 있음

여기저기 흩어지면 어디서 한 번 더 변환됐는지 추적이 안 됨. 저장소 안에 도<->라디안
변환이 이미 두 곳 있는데(`motors/robstride/codec/mit.py` 는 프로토콜 경계,
`control/policy.py` 와 `control/stand.py` 는 모델 경계) 서로 안 만남. 이 파일이
세 번째이고, **ROS 경계 밖으로는 안 나감.**


## 관절 이름의 구분자

`Biped` 는 `right_leg/knee` 처럼 `/` 로 팔다리를 앞에 붙임
(`robots/biped.py` 의 `SEP`). `sensor_msgs/JointState.name` 은 그냥 문자열 배열이라
`/` 가 들어가도 동작은 함. 그래도 바꾸는 이유는 **사람이 토픽 이름과 헷갈리기
때문**임 -- ROS 에서 `/` 는 네임스페이스 구분자임.

    huphy   right_leg/knee
    ROS     right_leg__knee

`SEP` 가 바뀌면 여기도 같이 바뀌어야 하므로 상수를 가져다 씀.
"""

from __future__ import annotations

import math
from typing import Dict, Iterable, List, Mapping, Sequence, Tuple

from huphy.robots.biped import SEP

ROS_SEP = "__"
"""ROS 쪽 관절 이름에서 팔다리와 관절 사이에 쓰는 구분자."""

FIELDS: Tuple[str, ...] = ("pos", "vel", "torque")
"""`Leg.get_observation()` 의 접미사 중 `JointState` 로 나가는 것.

    pos      JointState.position    도   -> rad
    vel      JointState.velocity    도/초 -> rad/s
    torque   JointState.effort      Nm   (변환 없음)

`temp` 는 `JointState` 에 자리가 없어 진단 쪽으로 감.
"""


def to_ros_name(name: str) -> str:
    """`right_leg/knee` -> `right_leg__knee`. 팔다리가 없으면 그대로."""
    return name.replace(SEP, ROS_SEP)


def to_huphy_name(name: str) -> str:
    """`right_leg__knee` -> `right_leg/knee`. 팔다리가 없으면 그대로."""
    return name.replace(ROS_SEP, SEP)


def joint_names(observation: Mapping[str, float]) -> Tuple[str, ...]:
    """관찰 사전에서 관절 이름을 뽑음. **huphy 이름 그대로**, 나온 순서대로.

    `.pos` 가 있는 것만 관절로 봄 -- `stale_motors` 같은 스칼라 항목을 걸러 냄.
    """
    out: List[str] = []
    for key in observation:
        if key.endswith(".pos"):
            out.append(key[: -len(".pos")])
    return tuple(out)


def observation_to_arrays(
    observation: Mapping[str, float],
    names: Sequence[str],
) -> Tuple[List[str], List[float], List[float], List[float]]:
    """관찰 사전 -> `JointState` 네 배열.

    `names` 는 **huphy 이름**으로 주고, 반환되는 이름 배열은 ROS 이름임.

    없는 항목은 `0.0` 으로 둠. `JointState` 는 세 배열의 길이가 같아야 하고, 길이가
    다르면 받는 쪽이 짝을 못 맞춤 -- 빈 자리를 비워 두는 것보다 0 이 낫다는 뜻이
    아니라, **길이를 맞추는 것이 필수**라서임. 값이 유효한지는 `stale_motors` 와
    별도 진단 토픽으로 봄.
    """
    ros_names = [to_ros_name(n) for n in names]
    position = [math.radians(float(observation.get(f"{n}.pos", 0.0))) for n in names]
    velocity = [math.radians(float(observation.get(f"{n}.vel", 0.0))) for n in names]
    effort = [float(observation.get(f"{n}.torque", 0.0)) for n in names]
    return ros_names, position, velocity, effort


def command_to_action(
    names: Iterable[str],
    position_rad: Iterable[float],
) -> Dict[str, float]:
    """`JointState` 의 이름·위치 -> `Action` (huphy 이름, 도).

    `ControlLoop` 가 받는 `Motion` 의 반환값 모양임
    (`control/loop.py` 의 `Motion`).

    **길이가 다르면 멈춤.** `zip` 은 짧은 쪽에 맞춰 조용히 잘라서, 관절 하나가
    빠진 명령이 그대로 통과함.
    """
    ros_names = list(names)
    values = list(position_rad)
    if len(ros_names) != len(values):
        raise ValueError(
            f"이름 {len(ros_names)}개, 위치 {len(values)}개로 다름. "
            f"JointState 의 name 과 position 길이가 같아야 함"
        )
    return {
        to_huphy_name(n): math.degrees(float(v)) for n, v in zip(ros_names, values)
    }


def imu_arrays(state) -> Tuple[List[float], List[float], List[float]]:
    """`ImuState` -> (각속도 rad/s, 가속도 m/s^2, 중력방향 단위벡터).

    `sensors/base.py` 의 `ImuState` 는 각속도를 **도/초**로 들고 있음. 중력방향은
    이미 단위벡터라 변환이 없음.

    **중력방향은 `sensor_msgs/Imu` 에 자리가 없음.** `orientation` 쿼터니언으로
    되돌릴 수는 있으나 정책이 실제로 쓰는 값이 중력방향 3칸이므로
    (`control/stand.py` 의 관찰 3-5칸), 그 값을 그대로 별도 토픽으로 내보냄 --
    쿼터니언으로 바꿨다가 다시 푸는 왕복에서 틀릴 자리를 안 만듦.
    """
    gyro = [math.radians(float(v)) for v in state.gyro_dps]
    accel = [float(v) for v in state.accel_mps2]
    gravity = [float(v) for v in state.gravity]
    return gyro, accel, gravity
