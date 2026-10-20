"""제어 계층.

    loop.py               정해진 주기로 한 사이클씩 돌림
    motions.py            매 주기 무엇을 시킬지 정함
    policy.py             `PolicySpec` -- 가중치 파일과 대조하는 규격 자료형
    generalized_walk.py   지금의 정식 걷기 정책. 관찰 빌더·규격
    rsl_rl.py             체크포인트(.pt) -> 부를 수 있는 함수. numpy 만 씀

둘을 나눈 이유: 루프는 주기와 안전만 보고, 무엇을 시킬지는 동작이 정함. 같은 루프로
유지·계단·사인파·보행 궤적을 다 돌릴 수 있고, 동작은 하드웨어 없이 시험됨.

`motions` 는 순수 함수라 의존이 없음. `loop` 는 로봇을 받으므로 경로를 명시해 가져감.
"""

from . import generalized_walk, motions, policy, rsl_rl
from .loop import ControlLoop, LoopStats, Mode, Motion, held, precise_sleep

__all__ = [
    "motions", "policy", "rsl_rl", "generalized_walk",
    "ControlLoop", "LoopStats", "Mode", "Motion", "held", "precise_sleep",
]
