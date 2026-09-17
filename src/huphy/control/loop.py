"""제어 루프 — 정해진 주기로 한 사이클씩 돌림.

한 사이클이 하는 일.

    1  목표를 정함              동작 제공자(`Motion`)가 냄
    2  계산                     로봇이 관절 -> 모터 -> 프레임으로 풂
    3  전송
    4  수거                     상태 갱신
    5  기록                     텔레메트리
    6  다음 주기까지 기다림

**루프는 무엇을 시킬지 모름.** 그건 `Motion` 이 정하고, 루프는 주기와 안전만 봄.
덕분에 같은 루프로 유지, 사인파 흔들기, 보행 궤적을 다 돌릴 수 있음.


## 기다리는 법

`time.sleep` 은 요청한 만큼 정확히 자지 않음. 운영체제 스케줄러가 깨워 주는
단위가 있어서, 짧은 잠일수록 오차 비중이 커짐.

100Hz 면 한 주기가 10ms 인데 그 오차가 몇 ms 씩 섞이면 주파수가 눈에 띄게 떨어짐.

그래서 **마감 직전 몇 ms 는 자지 않고 돌면서 기다림.**

    남은 시간 > spin_threshold   ->  (남은 시간 - margin) 만큼 잠
    남은 시간 <= spin_threshold  ->  돌면서 마감을 봄

CPU 를 태우는 구간이라 짧게 둠. 100Hz 에서 마지막 1~2ms 정도임.
`precise=False` 로 끄면 `time.sleep` 만 씀.


## 실제 주기를 잼

목표 주기와 실제 주기는 다름. 계산이 길어지거나 수거가 늦어지면 밀림.

**밀렸을 때 조용히 넘어가면 안 됨.** 느려진 것을 아무도 모른 채로 게인을 튜닝하면,
게인이 아니라 주기가 문제인데 게인을 계속 만지게 됨.

`loop_dt` 를 매 주기 텔레메트리로 내보내고, 목표를 크게 넘긴 주기는 세어 둠.


## 두 가지 모드

    OBSERVE   토크를 끄고 상태만 읽음
    CONTROL   명령을 보냄

`OBSERVE` 는 **들어갈 때 토크를 끊음.** 설정 실수로 명령이 나가는 것을 막으려는
것임 — 관찰하려고 켰는데 다리가 움직이면 안 됨.


## 멈출 때

    1  `DAMPING_S` 초 동안 감쇠 (`damp`, kp=0, kd 만 남김)
    2  토크 차단
    3  텔레메트리 flush

**바로 토크를 끊지 않음.** 서 있는 다리에서 힘이 갑자기 빠지면 주저앉음.
`damp` 로 위치를 붙잡는 대신 속도만 죽이면서 몇 초에 걸쳐 늦춘 뒤 끊음 --
목표 위치로 당기는 `kp` 항이 없어 그 자리에 고정되지 않고 중력·관성에 맡겨짐.

예외로 빠져나가도 이 순서를 탐.


## 통신이 끊기면 스스로 멈춤

연속 무응답이 이어지면 그 모터는 **현재 위치를 모른 채 마지막 명령을 유지하고
있음.** 다리 하나면 그 관절만 굳지만, 양다리면 한쪽만 움직여 로봇이 넘어짐.

판정은 `safety/link.py` 가 하고, 여기는 그 결과로 위의 정지 절차를 탐. 몇 주기를
기다릴지는 `SafetyConfig.link_loss_cycles` 임.

**제어 모드에서만 봄.** 관찰 모드는 토크가 없어서 세울 것이 없고, 커미셔닝처럼
응답이 원래 드문 상황에서 멀쩡한 진행이 끊김.
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Callable, Dict, Optional

from ..robots.base import Action, Robot
from ..safety.link import LinkLoss, LinkWatch

logger = logging.getLogger(__name__)

OVERRUN_RATIO = 1.5
"""목표 주기의 이 배를 넘으면 밀린 것으로 셈. 100Hz 에서 15ms."""

DAMPING_S = 5.0
"""멈출 때 `damp` 를 보내는 시간 (초). 힘이 갑자기 빠지지 않게, 위치를 붙잡지
않고 속도만 죽여서 서서히 멈추게 함."""

SPIN_THRESHOLD_S = 0.003
"""마감까지 이만큼 남으면 자지 않고 돌면서 기다림."""

SLEEP_MARGIN_S = 0.003
"""잘 때 마감보다 이만큼 일찍 깨게 함.

**`time.sleep` 의 오차보다 커야 함.** 작으면 한 번의 긴 잠이 마감을 지나쳐 버려
남은 스핀 구간이 없어짐 -- 그러면 정확해지지 않음.

두 값을 재서 고름 (목표별 평균 오차).

    threshold/margin      20ms     10ms      5ms      2ms
    ----------------------------------------------------
    2ms / 1ms           +2.56    +0.85    +0.02    +0.00
    3ms / 3ms           +0.61    +0.00    +0.00    +0.00
    5ms / 5ms           +0.00    +0.00    +0.00    +0.00

3ms 로 100Hz(10ms) 이하에서 오차가 사라짐. 실제 스핀은 한 주기당 약 1ms 임.
5ms 는 더 정확하지만 그만큼 CPU 를 더 태움.

위 수치는 스케줄러 정밀도가 낮은 환경에서 잰 것임. 잠이 정확한 환경에서는 스핀
구간이 더 짧아짐 -- 자동으로 그렇게 됨.
"""


def precise_sleep(
    seconds: float,
    *,
    spin_threshold: float = SPIN_THRESHOLD_S,
    sleep_margin: float = SLEEP_MARGIN_S,
) -> None:
    """`time.sleep` 보다 정확하게 기다림. **CPU 를 조금 더 씀.**

    마감까지 여유가 있으면 자고, 마지막 구간은 돌면서 봄. 자는 시간에서 `margin`
    을 빼 두어 지나쳐 자는 것을 막음.

    `seconds` 가 0 이하면 바로 돌아옴 — 이미 늦은 것이고, 따라잡으려 하지 않음.
    """
    if seconds <= 0:
        return
    deadline = time.perf_counter() + seconds
    while True:
        remaining = deadline - time.perf_counter()
        if remaining <= 0:
            return
        if remaining > spin_threshold:
            time.sleep(max(remaining - sleep_margin, 0.0))
        # 남은 구간은 돌면서 마감을 봄.


class Mode(str, Enum):
    """루프가 무엇을 하는지.

    `OBSERVE` 는 명령을 아예 만들지 않음. 계산을 건너뛰는 것이 아니라 **토크를
    끊고 시작함** — 설정이 잘못돼 있어도 다리가 안 움직이게 하려는 것임.
    """

    OBSERVE = "observe"
    CONTROL = "control"


Motion = Callable[[float, Dict[str, Any]], Optional[Action]]
"""무엇을 시킬지 정하는 함수.

    (경과 시간 초, 지금 관찰) -> 관절 목표  또는  None

`None` 을 내면 그 주기는 명령을 보내지 않음. 궤적이 끝났거나 아직 시작 전일 때 씀.
"""


@dataclass
class LoopStats:
    """돌린 결과. 끝나고 사람이 보는 값임."""

    cycles: int = 0
    overruns: int = 0
    """목표 주기를 크게 넘긴 횟수."""

    worst_dt_ms: float = 0.0
    total_s: float = 0.0
    missing_cycles: int = 0
    """무응답 모터가 하나라도 있었던 주기 수."""

    target_hz: float = 0.0
    """목표 주기. 실제와 대조하는 데 씀."""

    link_loss: Optional["LinkLoss"] = None
    """통신 두절로 멈췄으면 무엇이 끊겼는지. 정상 종료면 `None`.

    **정상 종료와 구분되어야 함.** 둘 다 루프가 조용히 끝나므로, 이 값이 없으면
    사람이 "시간이 다 됐나" 와 "다리가 죽었나" 를 구분할 수 없음.
    """

    @property
    def mean_hz(self) -> float:
        return self.cycles / self.total_s if self.total_s > 0 else 0.0

    @property
    def kept_up(self) -> bool:
        """목표 주기를 대체로 지켰는지. 90% 미만이면 못 지킨 것으로 봄.

        `overruns` 는 튀는 주기를 세지만, **꾸준히 느린 것은 못 잡음** -- 매 주기
        24% 씩 넘으면 한 번도 "밀림" 으로 세지 않으면서 주파수만 떨어짐.
        """
        return self.target_hz <= 0 or self.mean_hz >= self.target_hz * 0.9

    def summary(self) -> str:
        target = f" / 목표 {self.target_hz:.0f}Hz" if self.target_hz else ""
        warn = "" if self.kept_up else "  ** 주기를 못 지킴 **"
        if self.link_loss is not None:
            warn += f"  ** 통신 두절로 멈춤: {self.link_loss} **"
        return (
            f"{self.cycles}주기 {self.total_s:.1f}초 "
            f"(평균 {self.mean_hz:.1f}Hz{target}), "
            f"밀림 {self.overruns}회, 최악 {self.worst_dt_ms:.1f}ms, "
            f"무응답 주기 {self.missing_cycles}회{warn}"
        )


class ControlLoop:
    """로봇 하나를 정해진 주기로 돌림.

    텔레메트리는 선택임 — 없으면 기록만 안 하고 나머지는 같음.
    """

    def __init__(
        self,
        robot: Robot,
        *,
        hz: float = 100.0,
        telemetry: Any = None,
        mode: Mode = Mode.OBSERVE,
        precise: bool = True,
        link_watch: Optional[LinkWatch] = None,
    ) -> None:
        if hz <= 0:
            raise ValueError(f"hz 는 0보다 커야 함 (받은 값 {hz})")
        self.robot = robot
        self.hz = float(hz)
        self.period_s = 1.0 / float(hz)
        self.telemetry = telemetry
        self.mode = mode
        self.precise = precise
        """마감 직전을 돌면서 기다릴지. 끄면 `time.sleep` 만 씀."""
        self.link_watch = (
            link_watch if link_watch is not None else _watch_from(robot)
        )
        """통신 두절 판정. 로봇의 안전 설정에서 만들어짐.

        직접 넘기면 그것을 씀 -- 커미셔닝처럼 응답이 원래 드문 상황에서 끔.
        """

        self.stats = LoopStats(target_hz=self.hz)
        self._stop = False

    def __repr__(self) -> str:
        return f"ControlLoop({self.robot.id}, {self.hz:.0f}Hz, {self.mode.value})"

    def stop(self) -> None:
        """다음 주기에 빠져나오게 함. 다른 스레드나 시그널 처리기에서 부름."""
        self._stop = True

    # ---- 한 주기 ----------------------------------------------------------
    def step(self, motion: Optional[Motion], t: float) -> Dict[str, Any]:
        """한 사이클. 관찰을 돌려줌.

        **기다리지 않음** — 주기 유지는 `run` 이 함. 한 사이클만 떼어 시험하거나
        대화형으로 한 걸음씩 돌릴 때 이것을 씀.
        """
        observation = self.robot.get_observation()

        if self.mode is Mode.OBSERVE:
            # 아무것도 보내지 않으면 아무것도 오지 않음. MIT 모드에는 읽기 전용
            # 명령이 없어서, 힘이 나가지 않는 명령을 보내고 그 응답을 받음.
            missing = self.robot.refresh()
        else:
            action = motion(t, observation) if motion is not None else None
            if action:
                self.robot.send(self.robot.build_commands(action))
            missing = self.robot.collect()
            self._check_link()

        if missing:
            self.stats.missing_cycles += 1
        return observation

    def _check_link(self) -> None:
        """통신이 끊겼으면 루프를 세움. **제어 모드에서만 부름.**

        여기서 토크를 끊지 않음 -- 정지 절차(`_exit`)가 자세를 붙잡은 뒤 끊는
        순서를 가지고 있음. 바로 끊으면 서 있는 다리가 주저앉음.

        **이미 판정된 뒤에는 다시 보지 않음.** 멈추는 중에도 응답이 계속 없을
        것이므로, 다시 보면 같은 경고가 매 주기 쌓임.
        """
        if not self.link_watch.enabled or self.stats.link_loss is not None:
            return
        status = getattr(self.robot, "link_status", None)
        if not callable(status):
            return

        loss = self.link_watch.check(status())
        if loss is None:
            return

        self.stats.link_loss = loss
        logger.error(
            "%s 통신 두절로 멈춤: %s. 자세를 붙잡고 토크를 끊음",
            self.robot.id, loss,
        )
        self.stop()

    # ---- 계속 돌림 ---------------------------------------------------------
    def run(
        self,
        motion: Optional[Motion] = None,
        *,
        duration_s: Optional[float] = None,
        max_cycles: Optional[int] = None,
    ) -> LoopStats:
        """멈출 때까지 돌림.

        `duration_s` 나 `max_cycles` 를 주면 그만큼만 돌고 나옴. 둘 다 없으면
        `stop()` 이 불릴 때까지 돎.

        **끝나는 경로가 하나임.** 정상 종료든 예외든 `finally` 를 지나며 자세를
        붙잡고 토크를 끊음.
        """
        self._stop = False
        # 이번 실행의 통계만 담음. 누적하면 주기 수는 쌓이는데 시간은 이번 것이라
        # 평균 주파수가 목표보다 높게 나옴.
        self.stats = LoopStats(target_hz=self.hz)
        self._enter()

        t0 = time.perf_counter()
        previous = t0

        try:
            while not self._stop:
                cycle_start = time.perf_counter()
                t = cycle_start - t0

                if duration_s is not None and t >= duration_s:
                    break
                if max_cycles is not None and self.stats.cycles >= max_cycles:
                    break

                dt_ms = (cycle_start - previous) * 1000.0
                previous = cycle_start

                self.step(motion, t)
                self._record(dt_ms)
                self._note_timing(dt_ms)

                self._sleep_until(cycle_start + self.period_s)
        finally:
            self.stats.total_s = time.perf_counter() - t0
            self._exit()

        if not self.stats.kept_up:
            logger.warning(
                "%s 주기를 못 지킴: 평균 %.1fHz (목표 %.0fHz). "
                "게인을 만지기 전에 이것부터 볼 것 -- 계산이나 수거가 예산을 넘김",
                self.robot.id, self.stats.mean_hz, self.hz,
            )
        logger.info("%s 종료: %s", self.robot.id, self.stats.summary())
        return self.stats

    # ---- 들어가고 나오기 ----------------------------------------------------
    def _enter(self) -> None:
        if not self.robot.is_connected:
            self.robot.connect()

        if self.mode is Mode.OBSERVE:
            # 관찰 모드에서 토크를 끊는 것은 하드 안전장치임. 설정이 잘못돼 있어도
            # 다리가 안 움직이게 함.
            self.robot.disable()
        else:
            self.robot.enable()

        if self.telemetry is not None:
            self.telemetry.open()

    def _exit(self) -> None:
        """자세를 붙잡은 뒤 힘을 뺌.

        바로 끊으면 서 있는 다리가 주저앉음. 예외로 빠져나가도 이 경로를 탐.
        """
        try:
            if self.mode is Mode.CONTROL and self.robot.is_connected:
                self._settle()
        except Exception as e:
            logger.warning("%s 정지 자세 유지 실패 (계속 진행함): %s", self.robot.id, e)

        try:
            if self.robot.is_connected:
                self.robot.disable()
        except Exception as e:
            logger.warning("%s 토크 차단 실패: %s", self.robot.id, e)

        if self.telemetry is not None:
            self.telemetry.close()

    def _settle(self) -> None:
        """`DAMPING_S` 초 동안 감쇠 명령을 보냄. 위치를 붙잡지 않고 속도만 죽임."""
        damp = getattr(self.robot, "damp", None)
        if not callable(damp):
            return
        cycles = max(1, round(DAMPING_S * self.hz))
        for _ in range(cycles):
            self.robot.send(damp())
            self.robot.collect()
            time.sleep(self.period_s)

    # ---- 주기 -------------------------------------------------------------
    def _sleep_until(self, deadline: float) -> None:
        """다음 주기까지 기다림. 이미 늦었으면 **따라잡지 않음.**

        밀린 만큼 다음 주기를 줄여 따라잡으면 그 주기가 더 짧아져 계산이 또 밀림.
        한 주기를 늦게 시작하고 마는 편이 나음.

        마감을 절대 시각으로 들고 있음. 매 주기 "남은 시간" 을 새로 계산하면 그
        오차가 쌓여 서서히 밀림.
        """
        remaining = deadline - time.perf_counter()
        if remaining <= 0:
            return
        if self.precise:
            precise_sleep(remaining)
        else:
            time.sleep(remaining)

    def _note_timing(self, dt_ms: float) -> None:
        self.stats.cycles += 1
        if dt_ms > self.stats.worst_dt_ms:
            self.stats.worst_dt_ms = dt_ms
        # 첫 주기는 직전이 없어 dt 가 0에 가까움. 세지 않음.
        if self.stats.cycles > 1 and dt_ms > self.period_s * 1000.0 * OVERRUN_RATIO:
            self.stats.overruns += 1
            if self.stats.overruns == 1:
                logger.warning(
                    "%s 주기가 밀림: %.1fms (목표 %.1fms). 이후로는 세기만 함",
                    self.robot.id, dt_ms, self.period_s * 1000.0,
                )

    def _record(self, dt_ms: float) -> None:
        if self.telemetry is None:
            return
        try:
            self.telemetry.record(loop_dt_ms=dt_ms)
        except Exception as e:
            # 관측이 제어를 멈추면 관측할 대상이 없어짐.
            logger.warning("%s 텔레메트리 기록 실패: %s", self.robot.id, e)


def _watch_from(robot: Any) -> LinkWatch:
    """로봇의 안전 설정으로 통신 두절 판정을 만듦.

    설정이 없는 로봇(테스트의 가짜 로봇 등)은 기본값으로 감 — 판정을 켜는 것이
    안전한 쪽임.
    """
    safety = getattr(robot, "safety", None)
    cycles = getattr(safety, "link_loss_cycles", None)
    return LinkWatch() if cycles is None else LinkWatch(cycles)
