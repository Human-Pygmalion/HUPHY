"""명령 가드 — 검사하고 보정한다. 전부 순수 함수.

명령 하나가 모터로 나가기 전에 통과해야 하는 세 관문.

    1. 유한값 검사    NaN / Inf 는 거부      -- 클리핑이 불가능하다
    2. 위치 제한      한계를 넘으면 클리핑
    3. 점프 가드      급격한 변화면 클리핑

2, 3은 **버리지 않고 자른다.** 버리면 그 모터만 직전 명령을 유지해 다리 자세가
어긋난다 -- 발목처럼 2모터가 연동된 곳에서 특히 나쁘다 (docs/issues.md G).


## 1이 반드시 먼저여야 하는 이유

파이썬의 min/max는 NaN을 비교할 수 없어 **그냥 통과시킨다.**

    min(10, nan) -> 10        # nan < 10 이 False라 10을 유지

그래서 인코딩 단계의 클램프가 무력화되고,

    float_to_uint(nan, -12.57, 12.57, 16) -> 65535   # 최대값

**NaN 하나가 720도 목표 명령이 된다.** 조용히, 에러 없이.

NaN이 생기는 경로: 발목 IK 뉴턴 반복 발산, 0으로 나누기(sign=0),
센서 이상값. 클리핑으로 고칠 수 없으므로 거부한다.


## 클리핑은 조용한 변조다

명령한 것과 다른 게 실행되므로 **무슨 일이 있었는지 반드시 함께 돌려준다.**
호출부가 세어서 텔레메트리로 내보낸다.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, Optional, Tuple

from .limits import Limits, clamp


class RejectReason(str, Enum):
    """전송하지 않은 이유. 클리핑으로 고칠 수 없는 것들."""

    NOT_FINITE = "nan"        # NaN 또는 Inf. 자를 대상이 아니다
    NO_STATE = "nostate"      # 유효한 측정 상태가 없다. 점프 폭을 못 잰다


class ClipReason(str, Enum):
    """잘린 이유. 전송은 된다."""

    LIMIT = "limit"           # 위치 제한
    JUMP = "jump"             # 점프 가드 (폭주 상한)
    RATE = "rate"             # 속도 제한 (직전 명령 기준 램프)


@dataclass(frozen=True)
class GuardResult:
    """가드를 통과한 결과.

    value가 None이면 전송하지 않는다. 그 경우 reject에 이유가 담긴다.
    """

    value: Optional[float]
    reject: Optional[RejectReason] = None
    clips: Tuple[ClipReason, ...] = ()

    @property
    def sendable(self) -> bool:
        return self.value is not None


def is_finite(*values: Optional[float]) -> bool:
    """전부 유한한 실수인가. None은 검사 대상이 아니므로 건너뛴다."""
    for v in values:
        if v is None:
            continue
        if not math.isfinite(float(v)):
            return False
    return True


def clamp_jump(
    target_deg: float, current_deg: float, max_delta_deg: float
) -> Tuple[float, bool]:
    """**측정 위치**에서 max_delta 이상 벗어나지 않게 자른다. (값, 잘림여부).

    큰 명령이 한 번에 나가면 모터가 최대 토크로 급가속한다. 이것을 막는 **폭주
    상한**이며, 기본값 50° 가 그 의도다 (`SafetyConfig.max_delta_deg` 참고).

    **이것은 속도 제한이 아니다.** 이전 판 주석은 "클리핑 = 속도 제한이다" 라고
    적었으나, 벤치 실측이 그것을 반증했다 (2026-09-07):

    * 기준점이 **측정 위치**이므로 명령과 현재각의 차이가 max_delta 를 넘지 못한다.
      MIT 식이 ``tau = kp*(명령각 - 현재각) + ...`` 이므로 **토크가
      ``kp * max_delta`` 로 묶인다.** 3° / kp 30 이면 1.57 N·m 다.
    * 그리고 스스로를 조인다. 모터가 뒤처지면 측정이 안 나아가고 → 명령도 안
      나아가고 → 오차가 안 쌓이고 → 토크가 안 커진다. 느릴수록 더 느려진다.
      벤치 실측: 이론 상한 300°/s 인 설정에서 실제 49°/s.

    진짜 속도 제한은 :func:`clamp_rate` 다. 그쪽은 기준점이 **직전 명령**이라
    명령이 모터와 무관하게 보장된 속도로 나아가고, 오차는 자유롭게 커져 따라잡을
    토크를 온전히 쓴다. 이 함수는 그 위에 얹는 넉넉한 폭주 상한으로 남는다.
    """
    cur = float(current_deg)
    tgt = float(target_deg)
    limit = abs(float(max_delta_deg))

    delta = tgt - cur
    if delta > limit:
        return cur + limit, True
    if delta < -limit:
        return cur - limit, True
    return tgt, False


def clamp_rate(
    target_deg: float, prev_cmd_deg: Optional[float], max_step_deg: float
) -> Tuple[float, bool]:
    """**직전 명령**에서 max_step 이상 벗어나지 않게 자른다. (값, 잘림여부).

    이것이 속도 제한이다. ``max_step_deg`` 는 호출부가
    ``max_vel_deg_s / control_hz`` 로 만들어 넘긴다 — 한도는 **속도(°/s)** 로
    적고, 그것을 쓰는 루프가 자기 주기로 환산한다. 상수를 그대로 두면 제어 주기를
    바꿨을 때 실효 속도가 조용히 따라 변한다 (지금 벤치의 3° 가 100 Hz 와 곱해져
    300°/s 가 "우연히" 정해지는 것이 그 예다).

    기준점이 직전 **명령**인 것이 :func:`clamp_jump` 와의 유일하고 결정적인 차이다:

    * 명령은 모터가 뒤처지든 말든 보장된 속도로 계속 나아간다.
    * 오차(명령 − 측정)가 자유롭게 커지므로 따라잡거나 부하를 버틸 토크를 다 쓴다.

    ``prev_cmd_deg`` 가 None 이면(무장 직후, 또는 전송이 끊겼다 재개된 뒤) 자르지
    않고 그대로 통과시킨다. **호출부는 그때 직전 명령을 측정값으로 다시 잡아야
    한다.** 안 그러면 설정점이 관절에서 멀리 떨어진 채 남아 있다가 재개 순간
    그만큼을 명령한다 — 2026-09-07 벤치에서 두 번 난 사고가 이 부류다.
    """
    tgt = float(target_deg)
    if prev_cmd_deg is None:
        return tgt, False
    prev = float(prev_cmd_deg)
    limit = abs(float(max_step_deg))
    delta = tgt - prev
    if delta > limit:
        return prev + limit, True
    if delta < -limit:
        return prev - limit, True
    return tgt, False


def apply(
    target_deg: float,
    current_deg: Optional[float],
    *,
    limits: Optional[Limits],
    command_margin_deg: float,
    max_delta_deg: float,
    enforce_limits: bool = True,
    prev_cmd_deg: Optional[float] = None,
    max_step_deg: Optional[float] = None,
) -> GuardResult:
    """명령 하나에 세 관문을 적용한다.

    순서:
      1. 유한값 -- 산술 전에. NaN은 이후 모든 비교를 무력화한다
      2. 위치 제한 -- 안전한 목표로 만든다
      3. 속도 -- 직전 **명령**에서 한 주기에 갈 수 있는 만큼만 (`max_step_deg`)
      4. 점프 -- 측정 위치에서 너무 멀면 자르는 **폭주 상한** (`max_delta_deg`)

    2가 3·4보다 먼저인 것은 "안전한 목표를 정하고 거기로 가는 속도를 제한한다"는
    순서다. 반대로 하면 나중 관문을 통과한 값이 여전히 한계 밖일 수 있다.

    3이 4보다 먼저인 것은 두 관문이 서로 다른 기준점을 쓰기 때문이다. 속도 제한은
    직전 명령에서 램프를 만들고, 폭주 상한은 그 결과가 측정 위치에서 터무니없이
    멀지 않은지만 본다. 순서를 바꾸면 램프가 폭주 상한이 만든 값 위에서 시작해
    두 관문이 서로를 먹는다.

    ``max_step_deg`` 가 None 이면 속도 제한을 걸지 않는다 — 설정하지 않은 로봇은
    이 함수가 예전과 완전히 동일하게 동작한다.

    **출력이 한계 밖일 수 있다.** 현재 위치가 이미 한계 밖이면(사고 후 복구 중)
    한 번에 돌아오지 않고 max_delta씩 돌아온다. 이게 맞는 동작이다.
    """
    # 1. 유한값 -- 반드시 먼저
    if not is_finite(target_deg, current_deg):
        return GuardResult(value=None, reject=RejectReason.NOT_FINITE)

    if current_deg is None:
        return GuardResult(value=None, reject=RejectReason.NO_STATE)

    clips: list = []
    value = float(target_deg)

    # 2. 위치 제한
    if enforce_limits:
        value, clipped = clamp(value, limits, margin_deg=command_margin_deg)
        if clipped:
            clips.append(ClipReason.LIMIT)

    # 3. 속도 제한 (직전 명령 기준)
    if max_step_deg is not None:
        value, clipped = clamp_rate(value, prev_cmd_deg, max_step_deg)
        if clipped:
            clips.append(ClipReason.RATE)

    # 4. 폭주 상한 (측정 위치 기준)
    value, clipped = clamp_jump(value, current_deg, max_delta_deg)
    if clipped:
        clips.append(ClipReason.JUMP)

    return GuardResult(value=value, clips=tuple(clips))


@dataclass
class GuardCounters:
    """사이클 단위 집계.

    "잘렸다"와 "안 보냈다"는 다른 사건이므로 나눠서 센다.
    합계만 보면 어느 관문이 걸렸는지 알 수 없다.

    원본은 거부 시 print만 했고 그마저 모터당 0.5초 스로틀이라, 100Hz에서
    200번 거부돼도 콘솔에는 1줄만 떴다 -- 산발적인지 지속적인지 알 방법이 없었다.
    """

    clips: Dict[str, int] = field(default_factory=dict)
    rejects: Dict[str, int] = field(default_factory=dict)

    def record(self, result: GuardResult) -> None:
        """GuardResult 하나를 반영한다."""
        if result.reject is not None:
            key = result.reject.value
            self.rejects[key] = self.rejects.get(key, 0) + 1
        for clip in result.clips:
            key = clip.value
            self.clips[key] = self.clips.get(key, 0) + 1

    def reset(self) -> None:
        self.clips = {}
        self.rejects = {}

    @property
    def total_clips(self) -> int:
        return sum(self.clips.values())

    @property
    def total_rejects(self) -> int:
        return sum(self.rejects.values())

    def as_fields(self) -> Dict[str, int]:
        """텔레메트리용 평면 dict.

        모든 키를 항상 내보낸다 -- 0이어도. 필드가 나타났다 사라지면
        PlotJuggler 레이아웃과 CSV 헤더가 깨진다.
        """
        out: Dict[str, int] = {
            "clips": self.total_clips,
            "rejects": self.total_rejects,
        }
        for clip in ClipReason:
            out[f"clips_{clip.value}"] = self.clips.get(clip.value, 0)
        for reject in RejectReason:
            out[f"rejects_{reject.value}"] = self.rejects.get(reject.value, 0)
        return out
