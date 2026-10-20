"""ROS 쪽과 제어 스레드 사이의 한 칸. **`rclpy` 를 import 하지 않음.**

## 무엇을 푸는가

제어 루프와 ROS 가 **다른 스레드**에서 돎 (`docs/ros.md` 7장). 둘이 주고받아야
하는 것은 두 개뿐임.

    최신 명령    ROS  -> 제어     구독 콜백이 쓰고, Motion 이 읽음
    최신 관찰    제어 -> ROS      Motion 이 쓰고, 퍼블리시 타이머가 읽음

둘 다 **사전을 통째로 바꿈.** 항목마다 따로 쓰면 그 사이에 읽혀서 일부는 이번
주기, 일부는 지난 주기가 섞인 사전이 나감. 어느 항목이 언제 것인지 알 수 없게 됨.


## 왜 rclpy 를 안 쓰나

여기 있는 판단(오래된 명령을 버리는 것, 사전을 통째로 바꾸는 것)이 틀리면 로봇이
잘못 움직임. ROS 없이 테스트가 돌아야 그 판단을 고정해 둘 수 있음
(`tests/test_ros_bridge.py`).

시각은 `time.monotonic` 을 씀. ROS 시각(`node.get_clock()`)이 아닌 이유는
`huphy` 전체가 `monotonic` 을 쓰기 때문임 -- 벽시계는 NTP 보정으로 뒤로 갈 수 있어
나이 계산이 음수가 됨 (`motors/robstride/bus.py` 의 `MotorState.stamp`).
"""

from __future__ import annotations

import threading
import time
from typing import Any, Callable, Dict, Optional

DEFAULT_STALE_S = 0.2
"""명령이 이 시간보다 오래되면 전송을 멈춤.

**ROS 가 새로 만드는 실패 방식을 막는 것임.** 명령을 보내던 노드가 죽거나 네트워크가
끊기면 토픽이 그냥 안 올 뿐 에러가 안 남 -- 그대로 두면 마지막 목표를 영원히 붙잡고
있게 됨.

0.2초는 50Hz 명령 기준 10주기임. 한두 번 빠지는 것은 넘기고 끊긴 것만 잡으려는
값이고, 실물에서 다시 정해야 함.

`safety/link.py` 의 `LinkWatch` 와 같은 발상이되 보는 대상이 다름 -- 저쪽은 모터가
답을 안 하는 것, 이쪽은 명령이 안 오는 것.
"""


class CommandBridge:
    """최신 명령과 최신 관찰을 한 칸씩 들고 있음. 락 하나로 보호함.

    락을 하나만 쓰는 이유: 두 사전을 각각 다른 락으로 보호하면 `motion()` 이 둘 다
    잡아야 해서 순서를 지켜야 하고, 순서를 틀리면 교착이 생김. 잡고 있는 구간이
    사전 하나 바꾸는 것뿐이라 하나로 충분함.
    """

    def __init__(
        self,
        *,
        stale_s: float = DEFAULT_STALE_S,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.stale_s = float(stale_s)
        self._clock = clock
        """시각을 읽는 함수. 테스트가 가짜 시계를 넣음."""

        self._lock = threading.Lock()
        self._command: Optional[Dict[str, float]] = None
        self._command_stamp: Optional[float] = None
        self._observation: Dict[str, Any] = {}
        self._observation_stamp: Optional[float] = None
        """"아직 없음" 을 `None` 으로 둠. `0.0` 을 쓰면 안 되는 이유는
        `clock()` 이 **정말로 0.0 을 낼 수 있기** 때문임 -- 테스트의 가짜 시계가
        0 에서 시작하고, `time.monotonic()` 도 기준점이 정해져 있지 않음.
        """
        self.stale_drops = 0
        """오래돼서 버린 횟수. 진단 토픽으로 나감."""

    # ---- ROS 쪽에서 부름 ---------------------------------------------------
    def put_command(self, action: Dict[str, float]) -> None:
        """구독 콜백이 부름. **사전을 통째로 바꿈.**

        여기서 계산을 하지 말 것 -- 콜백이 GIL 을 오래 쥐면 그만큼 제어 스레드가
        멈춤 (`docs/ros.md` 7-5). 단위·이름 변환은 `convert.py` 가 콜백 밖에서,
        또는 콜백 안에서라도 짧게 끝내야 함.
        """
        with self._lock:
            self._command = dict(action)
            self._command_stamp = self._clock()

    def take_observation(self) -> Dict[str, Any]:
        """퍼블리시 타이머가 부름. 가장 최근 관찰의 사본.

        제어 스레드가 쓰는 중이어도 **절반만 갱신된 사전이 나오지 않음** -- 쓰는
        쪽이 통째로 바꾸기 때문임.
        """
        with self._lock:
            return dict(self._observation)

    def observation_age(self) -> float:
        """마지막 관찰 이후 흐른 초. 한 번도 없었으면 `-1.0`."""
        with self._lock:
            if self._observation_stamp is None:
                return -1.0
            return self._clock() - self._observation_stamp

    def clear_command(self) -> None:
        """명령을 비움. 정책을 끄거나 모드를 바꿀 때 씀.

        안 비우면 다음에 켤 때 **직전에 받아 둔 목표가 한 주기 나감.**
        """
        with self._lock:
            self._command = None
            self._command_stamp = None

    # ---- 제어 스레드에서 부름 ----------------------------------------------
    def motion(self, t: float, observation: Dict[str, Any]):
        """`ControlLoop` 가 받는 `Motion` (`control/loop.py` 의 `Motion`).

        관찰을 넣어 두고 최신 명령을 냄. 명령이 없거나 오래됐으면 `None` 을 내는데,
        `ControlLoop.step` 이 `None` 을 받으면 **그 주기는 아무것도 안 보냄.**

        `t` 를 안 씀 -- 토픽이 매 주기 오는 구조라 시각이 필요 없음. 뛰는 위상처럼
        시간을 쓰는 동작은 보내는 쪽이 계산해서 목표각으로 보냄.
        """
        now = self._clock()
        with self._lock:
            self._observation = dict(observation)
            self._observation_stamp = now
            if self._command is None or self._command_stamp is None:
                return None
            if now - self._command_stamp > self.stale_s:
                self.stale_drops += 1
                return None
            return dict(self._command)
