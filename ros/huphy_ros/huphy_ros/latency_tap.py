"""버스의 송신·수신 사건을 큐에 담음. **`rclpy` 없음.**

## 무엇을 하나

`RobStrideBus.latency` 에 붙는 자리임. 버스가 두 가지를 부름
(`motors/robstride/bus.py` 의 `_note_sent` / `_note_received`).

    sent(mid)                       모터에 명령을 내보내기 직전
    received(mid, latency_s, late)  그 응답이 도착했을 때

`telemetry.latency_log.LatencyLog` 가 같은 자리에 붙어 **메모리에 쌓고 끝에
파일로** 씀 (`huphy-run --log`). 이쪽은 **쌓지 않고 큐에 넣기만** 함 --
`control_node` 의 발행 타이머가 비워서 `LatencyBatch` 토픽으로 내보내고, 쌓는
것과 파일로 쓰는 것은 다른 기계의 `log_node` 가 함.

나누는 이유는 기계가 둘이기 때문임 (`ros/README.md` 결정 13). 모터는 모션보드에,
사람과 뷰어는 작업 기계에 있음. `control_node` 가 파일을 쓰면 기록이 두 기계로
나뉘어 떨어짐.


## 스레드

세 스레드가 닿음.

    제어 스레드    sent()       -- ControlLoop.run() 안에서 버스가 부름
    수신 스레드    received()   -- CanBus._read_loop 가 응답을 받을 때
    ROS executor   drain()      -- 발행 타이머

`collections.deque` 의 `append` 와 `popleft` 는 GIL 아래에서 원자적이라 락을 두지
않음. 제어 주기 안에서 하는 일이 append 하나여야 해서임.

**큐이지 슬롯이 아님.** `Latest` 처럼 마지막 값만 들고 있으면 덮인 사건이
사라지고, 그만큼 보낸 수가 줄어 유실 집계(`보낸 수 - 받은 수`)가 틀어짐. 제어
루프 주기와 발행 타이머 주기가 둘 다 기본 100Hz 지만 **서로 다른 타이머라 동기가
아님.**


## 길이 상한

`MAX_EVENTS` 를 넘으면 **가장 오래된 것을 버림** (`deque(maxlen=...)`). 발행이
멈춰도(기록을 안 켰거나 구독자가 없어도 발행 타이머는 돎) 메모리가 늘지 않게
하려는 것임. 버린 수는 `dropped` 로 셈 -- `control_node` 가 상태 토픽에 적음.
"""

from __future__ import annotations

import time
from collections import deque
from typing import Deque, Dict, List, Tuple

SENT = 0
ONTIME = 1
LATE = 2
"""`LatencyBatch.msg` 의 `kind` 상수와 같은 값. 메시지 정의가 원본임."""

MAX_EVENTS = 20000
"""큐에 담아 두는 사건 수의 상한.

100Hz x 관절 12개 x (송신 + 수신) = 초당 2400개이므로 약 8초분임. 발행 타이머가
정상이면 한 번에 24개 정도만 쌓임 -- 이 상한에 닿는 것은 발행이 멈춘 경우임.
"""

NAN = float("nan")


class LatencyTap:
    """버스 하나에 붙어 모터 id 를 관절 색인으로 바꿔 큐에 넣음.

        tap = LatencyTap(names, queue, t0)
        bus.latency = tap

    `names` 는 `{모터 id: 관절 색인}` 임. 색인은 `ControlInfo.joint_names` 의
    자리이고, 이름 문자열을 매번 보내지 않으려고 숫자로 보냄.

    여러 버스(양다리면 둘)가 **같은 큐와 같은 `t0`** 를 씀. 한 토픽으로 나가고
    시각이 같은 기준이어야 함.
    """

    __slots__ = ("names", "queue", "t0")

    def __init__(self, names: Dict[int, int], queue: "Deque[Tuple[float, int, float, int]]",
                 t0: float) -> None:
        self.names = names
        self.queue = queue
        self.t0 = t0

    def sent(self, mid: int) -> None:
        idx = self.names.get(mid)
        if idx is not None:
            self.queue.append((time.monotonic() - self.t0, idx, NAN, SENT))

    def received(self, mid: int, latency_s: float, late: bool) -> None:
        idx = self.names.get(mid)
        if idx is not None:
            self.queue.append((
                time.monotonic() - self.t0, idx,
                latency_s * 1000.0, LATE if late else ONTIME,
            ))


class TapSet:
    """로봇 하나의 버스 전부에 `LatencyTap` 을 붙이고 큐를 들고 있음.

    `LatencyLog.attach` 와 같은 자리에서 같은 방식으로 관절 이름을 만듦
    (`telemetry/latency_log.py`) -- 양다리면 `right_leg/knee`, 다리 하나면 그
    팔다리 이름이 앞에 붙음. 기록 파일의 관절 이름이 `huphy-run` 쪽과 같아야
    비교가 됨.
    """

    def __init__(self, robot, *, max_events: int = MAX_EVENTS) -> None:
        self.joints: List[str] = []
        """관절 이름. 이 목록의 자리가 사건의 `joint` 색인임."""
        self.queue: "Deque[Tuple[float, int, float, int]]" = deque(maxlen=max_events)
        self.t0 = time.monotonic()
        self._taps: List[LatencyTap] = []

        for part in getattr(robot, "parts", None) or (robot,):
            bus = part.bus
            names = {}
            for mid in bus.motor_ids:
                name = f"{part.id}/{part.config.joint_of(mid) or mid}"
                if name not in self.joints:
                    self.joints.append(name)
                names[mid] = self.joints.index(name)
            tap = LatencyTap(names, self.queue, self.t0)
            self._taps.append(tap)
            bus.latency = tap

    def detach(self, robot) -> None:
        for part in getattr(robot, "parts", None) or (robot,):
            part.bus.latency = None

    @property
    def pending(self) -> int:
        """아직 안 나간 사건 수. `control_node` 가 상태 토픽에 적음.

        발행 타이머가 정상이면 24 안쪽임 (관절 12개 x 송신+수신, 한 주기분).
        `maxlen` 에 붙어 있으면 **발행이 멈춰 오래된 사건을 버리는 중**임.
        `deque(maxlen=)` 은 버린 수를 세지 않으므로 이 길이로 봄 -- 세려면 제어
        스레드가 매번 길이를 확인해야 하고, 그 자리에서 하는 일은 append 하나여야
        함.
        """
        return len(self.queue)

    @property
    def full(self) -> bool:
        return len(self.queue) >= (self.queue.maxlen or 0)

    def drain(self) -> List[Tuple[float, int, float, int]]:
        """쌓인 사건을 전부 꺼냄. 비어 있으면 빈 목록.

        꺼내는 동안 다른 스레드가 `append` 할 수 있음. `popleft` 가 끝에서
        `IndexError` 를 내는 지점까지만 가져가므로 새로 들어온 것은 다음 번에
        나감 -- 순서는 유지되고 사건이 빠지지 않음.
        """
        out: List[Tuple[float, int, float, int]] = []
        pop = self.queue.popleft
        while True:
            try:
                out.append(pop())
            except IndexError:
                return out
