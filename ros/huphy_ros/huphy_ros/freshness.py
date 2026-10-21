"""마지막으로 받은 값이 언제 것인지 재고, 낡았으면 알려 줌. **`rclpy` 없음.**

## 무엇을 재나

두 기계의 시계를 안 맞춤 (`docs/ros.md` 결정 9). 그래서 **편도 지연은 못 잼.**
각 노드가 자기 시계로 "마지막 수신 이후 몇 초 지났나" 만 봄.

    policy_node    now - 마지막 joint_states 수신 시각    기준 60ms
    control_node   now - 마지막 joint_targets 수신 시각   기준 100ms

메시지의 `header.stamp` 는 채워서 보내되 **여기서 안 씀.** 보낸 기계의 시계라
이쪽 시계와 비교하면 동기가 안 맞은 만큼 틀림. 기록(`ros2 bag`)과 나중에 시계를
맞출 때를 위해 남기는 값임.

## 왜 두 기준이 다른가

    60ms   정책이 상태를 못 받음   -> 목표각 발행을 멈춤
    100ms  제어가 목표각을 못 받음 -> 감쇠 후 토크 차단

`100 > 60` 이어야 순서가 안 뒤바뀜. 정책이 먼저 멈추고 그 뒤에 제어가 멈춤. 반대로
두면 네트워크가 끊겼을 때 제어가 먼저 토크를 끊고, 정책은 그것도 모른 채 목표각을
계속 냄.

둘 다 실물 측정 전의 제안값임 (`docs/ros.md` 5-2).
"""

from __future__ import annotations

import threading
import time
from typing import Any, Callable, Optional, Tuple

POLICY_STATE_STALE_S = 0.060
"""`policy_node` 기준. 관절 상태가 이보다 낡으면 목표각을 안 냄.

50Hz 상태 발행 기준 3주기임. 한두 번 빠지는 것은 넘기고 끊긴 것만 잡으려는 값.
"""

CONTROL_TARGET_STALE_S = 0.100
"""`control_node` 기준. 목표각이 이보다 안 오면 감쇠로 들어감.

50Hz 발행 기준 5주기임. `POLICY_STATE_STALE_S` 보다 커야 함 -- 위 docstring 참조.
"""


class Freshness:
    """값 하나와 그 수신 시각을 들고 있음. 스레드 둘이 같이 씀.

    ROS 콜백 스레드가 `put`, 제어·추론 스레드가 `take` 를 부름. 락 하나로 보호하고,
    **값은 통째로 바꿈** -- 항목마다 따로 쓰면 그 사이에 읽혀서 일부는 이번 것,
    일부는 지난 것이 섞인 값이 나감.
    """

    def __init__(
        self,
        stale_s: float,
        *,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        if stale_s <= 0:
            raise ValueError(f"stale_s 는 0보다 커야 함 (받은 값 {stale_s})")
        self.stale_s = float(stale_s)
        self._clock = clock
        """시각을 읽는 함수. 테스트가 가짜 시계를 넣음.

        `time.monotonic` 인 이유: 벽시계는 보정으로 뒤로 갈 수 있어 나이가 음수가 됨.
        `huphy` 전체가 같은 이유로 `monotonic` 을 씀
        (`motors/robstride/bus.py` 의 `MotorState.stamp`).
        """

        self._lock = threading.Lock()
        self._value: Any = None
        self._stamp: Optional[float] = None
        """`None` 이 "한 번도 못 받음" 임. `0.0` 을 쓰면 안 되는 이유는 `clock()` 이
        정말로 `0.0` 을 낼 수 있기 때문임 -- 기준점이 정해져 있지 않음."""

        self.stale_count = 0
        """낡아서 거절된 횟수. 상태 토픽으로 나감."""

    def put(self, value: Any) -> None:
        """받은 값을 넣음. **ROS 콜백에서 부름 -- 여기서 계산하지 말 것.**

        콜백이 GIL 을 오래 쥐면 그만큼 제어 스레드가 멈춤 (`docs/ros.md` 9장).
        """
        with self._lock:
            self._value = value
            self._stamp = self._clock()

    def take(self) -> Tuple[Any, bool]:
        """`(값, 쓸 수 있나)`. 한 번도 못 받았거나 낡았으면 둘째가 `False`.

        값은 낡았어도 같이 냄 -- 부르는 쪽이 로그에 찍거나 마지막 값을 보고 싶을 수
        있음. **쓸지 말지는 둘째 값으로 판단할 것.**
        """
        with self._lock:
            if self._stamp is None:
                return None, False
            if self._clock() - self._stamp > self.stale_s:
                self.stale_count += 1
                return self._value, False
            return self._value, True

    def age(self) -> float:
        """마지막 수신 이후 흐른 초. 한 번도 못 받았으면 `-1.0`."""
        with self._lock:
            if self._stamp is None:
                return -1.0
            return self._clock() - self._stamp

    def clear(self) -> None:
        """비움. 정책을 끄거나 토크를 차단할 때 씀.

        안 비우면 다음에 켤 때 **직전 값이 한 주기 나감.**
        """
        with self._lock:
            self._value = None
            self._stamp = None


class Latest:
    """값 하나를 들고 있음. 나이를 안 봄.

    반대 방향에 씀 -- 제어 스레드가 관찰을 써 두고 발행 타이머가 꺼냄. 발행이
    한 번 늦는 것은 문제가 아니므로 낡음 판정이 필요 없음.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._value: Any = None

    def put(self, value: Any) -> None:
        with self._lock:
            self._value = value

    def take(self) -> Any:
        """가장 최근 값. 한 번도 없었으면 `None`."""
        with self._lock:
            return self._value
