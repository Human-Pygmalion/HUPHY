"""WCM1600 프레임 한 줄 <-> 값.

장비도 시리얼도 모름. 바이트와 숫자만 다룸. `ebimu/protocol.py` 와 같은 자리임 --
순수 함수라 하드웨어 없이 테스트됨.


## 한 프레임의 모양 (Continuous 모드, 9600 8N1)

    <부호><다섯 자리>(CR)(LF)        8바이트. 전부 ASCII

    ~   AC (크기)
    +   DC 양
    -   DC 음

매뉴얼의 예가 자리수를 못 박아 둠.

    "1.23" A   ->  '0' '1' '2' '3' '0'
    "10.76" A  ->  '1' '0' '7' '6' '0'

**앞 네 자리를 100 으로 나눈 값임.** 다섯째 자리는 두 예에서 모두 `'0'` 이고 값이
변하는 자리가 아님 -- 해상도가 0.01A 라는 뜻임. 그래서 읽지 않고, `'0'` 이 아닌
프레임이 오면 매뉴얼과 다른 펌웨어이므로 그 줄을 버림.

## 깨진 줄은 버림

길이가 다르거나 숫자가 아니면 `None` 을 냄. 부르는 쪽이 세기만 하고 넘어감 --
전류계 때문에 로봇이 멈추면 안 됨. 시작 직후 첫 줄은 중간부터 잘려 들어오므로
`meter.py` 가 한 줄을 버리고 시작함.
"""

from __future__ import annotations

from typing import Optional, Tuple

FRAME_LEN = 6
"""(CR)(LF) 를 떼고 남는 바이트 수. 부호 1 + 숫자 5."""

DIGITS = 4
"""값을 만드는 데 쓰는 앞자리 수. 나머지 한 자리는 해상도 밖임."""

SCALE = 100.0
"""앞 네 자리 -> 암페어. `'0123'` -> 1.23 A."""

PAD = "0"
"""다섯째 자리에 와야 하는 문자. 다르면 매뉴얼과 다른 펌웨어임."""

AC = "~"
DC_PLUS = "+"
DC_MINUS = "-"
SIGNS = (AC, DC_PLUS, DC_MINUS)


def parse(frame: bytes) -> Optional[Tuple[bool, float]]:
    """`b'+01230'` -> `(False, 1.23)`. 형식이 틀리면 `None`.

    내는 것은 `(AC 인지, 암페어)` 임. DC 는 부호가 방향이고 AC 는 크기라 항상
    0 이상임.

    (CR)(LF) 는 떼고 줄 것 -- 양쪽 공백은 여기서도 한 번 더 떼므로 `strip()` 한
    것을 그대로 줘도 됨.
    """
    frame = frame.strip()
    if len(frame) != FRAME_LEN:
        return None
    sign, digits = chr(frame[0]), frame[1:].decode("ascii", errors="ignore")
    if sign not in SIGNS or not digits.isdigit():
        return None
    if digits[DIGITS:] != PAD:
        return None                 # 매뉴얼은 이 자리가 항상 '0' 임
    value = int(digits[:DIGITS]) / SCALE
    if sign == AC:
        return True, value
    return False, value if sign == DC_PLUS else -value
