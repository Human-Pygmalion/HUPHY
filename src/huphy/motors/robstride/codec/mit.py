"""MIT 표준 프레임(11-bit ID) 인코딩/디코딩.

로봇을 전혀 모름. 관절 이름도 모터 배치도 없고 숫자와 바이트만 다룸.
인코딩 범위를 인자로 받으므로 모델·프로토콜에 묶이지 않음.


## 명령 프레임 — Command 3 "MIT Dynamic Parameters" (매뉴얼 p.38)

11-bit 표준 ID = 대상 모터 CAN ID

    Byte0~1               목표각    16bit  <-> (-pmax ~ pmax) rad
    Byte2 + Byte3[7:4]    목표속도  12bit  <-> (-vmax ~ vmax) rad/s
    Byte3[3:0] + Byte4    Kp        12bit  <-> (0 ~ 500)
    Byte5 + Byte6[7:4]    Kd        12bit  <-> (0 ~ 5)
    Byte6[3:0] + Byte7    목표토크  12bit  <-> (-tmax ~ tmax) N.m


## 응답 프레임 — Response Command 1 "Data Feedback" (RS00/RS02/RS04 매뉴얼 6.1절)

    Byte0                 모터 CAN ID
    Byte1~2               현재각    16bit
    Byte3 + Byte4[7:4]    현재속도  12bit
    Byte4[3:0] + Byte5    현재토크  12bit
    Byte6[7:6]            동작 모드  0 Reset / 1 Cali / 2 Motor
    Byte6[5]              고장 있음
    Byte6[4]              경고 있음
    Byte6[3:0] + Byte7    권선 온도  12bit (0.1도 단위)

**명령과 응답의 바이트 배치가 다름** — 응답은 앞에 모터 ID가 붙어 한 칸씩 밀림.

**온도는 12비트임.** Byte6 상위 4비트는 온도가 아니라 모드·고장·경고 플래그임.
16비트로 읽으면 토크를 켠 순간(Motor 모드 -> Byte6[7]=1) 3300도쯤이 나옴.
private 프로토콜(Type 2)에서는 이 플래그들이 29비트 CAN ID 안에 있어 Byte6~7 이
온전한 16비트 온도인데, MIT 은 11비트 ID 라 넣을 자리가 없어 Byte6 로 내려왔음.


## 고장 프레임 — Command 5 응답 (매뉴얼 6.8절)

    Byte0                 모터 CAN ID
    Byte1~4               고장값 32bit, **작은 자리 먼저**. 0이면 정상

6.8절 표에는 바이트 순서가 안 적혀 있지만, 6.7절이 "에러 코드는 응답의 BYTE1
자리로 돌아온다" 고 함 — 즉 Byte1 이 최하위 바이트임. 매뉴얼 전체를 봐도 큰 자리
먼저는 비트를 손으로 욱여넣은 제어·피드백 프레임뿐이고, 날 것의 32비트 값
(파라미터 읽기·쓰기, 고장값)은 전부 작은 자리 먼저임. 고장값은 파라미터로도
읽히는 같은 레지스터(`faultSta`, uint32)임.

일반 상태 프레임과 CAN ID가 같아(둘 다 11비트 ID = 호스트 ID) 겉으로 구분되지
않음. 조회 명령을 보낸 직후의 첫 응답으로 간주해야 함.


## 단위

내부는 rad, 외부는 deg. 변환은 이 파일에서만 일어남 — 나머지 코드가 라디안을
신경 쓰지 않아도 되도록 경계에 가둠.
"""

from __future__ import annotations

import math
from typing import NamedTuple, Tuple

from ..tables import EncodingRange

FRAME_LEN = 8


class StatusFlags(NamedTuple):
    """상태 프레임 Byte6 상위 4비트. 매 응답마다 공짜로 딸려 옴.

    `fault` 는 고장이 **있다** 는 것만 알려 줌. 무슨 고장인지는 `decode_fault` 로
    따로 물어야 함. 대신 왕복이 필요 없어 제어 주기 안에서 볼 수 있음.

    `mode` 는 모터가 스스로 보고하는 값임 — 보낸 명령이 아니라 실제 상태이므로
    토크가 정말 켜졌는지 확인하는 데 씀 (2 = Motor 모드).
    """

    mode: int
    fault: bool
    warning: bool


def float_to_uint(x: float, x_min: float, x_max: float, bits: int) -> int:
    """실수를 [x_min, x_max] 범위 안에서 bits 비트 정수로 양자화함.

    범위를 벗어나면 **클램프됨** (감싸지 않음). 따라서 전송 전에 값이 범위 안인지
    확인해야 함 — 넘으면 조용히 최대/최소값이 나감.

    NaN은 min/max 비교가 전부 False라 이 클램프를 통과함. 상위(safety.guards)에서
    걸러야 함.
    """
    x = max(x_min, min(x_max, float(x)))
    span = x_max - x_min
    norm = (x - x_min) / span if span > 0 else 0.0
    return int(norm * ((1 << bits) - 1))


def uint_to_float(x: int, x_min: float, x_max: float, bits: int) -> float:
    """float_to_uint의 역변환."""
    span = x_max - x_min
    norm = float(x) / ((1 << bits) - 1)
    return norm * span + x_min


def pack_command(
    *,
    position_deg: float,
    velocity_deg_s: float,
    kp: float,
    kd: float,
    torque_nm: float,
    enc: EncodingRange,
) -> bytes:
    """MIT 동작 제어 명령 8바이트를 만듦.

    모터 펌웨어가 이 다섯 값으로 PD를 계산함:
        tau = kp*(목표각 - 현재각) + kd*(목표속도 - 현재속도) + 토크_FF
    """
    q = float_to_uint(math.radians(position_deg), -enc.pmax_rad, enc.pmax_rad, enc.pos_bits)
    dq = float_to_uint(
        math.radians(velocity_deg_s), -enc.vmax_rad_s, enc.vmax_rad_s, enc.vel_bits
    )
    kp_u = float_to_uint(kp, 0.0, enc.kp_max, enc.gain_bits)
    kd_u = float_to_uint(kd, 0.0, enc.kd_max, enc.gain_bits)
    tau = float_to_uint(torque_nm, -enc.tmax_nm, enc.tmax_nm, enc.tau_bits)

    return bytes(
        [
            (q >> 8) & 0xFF,                            # Byte0  목표각 상위
            q & 0xFF,                                   # Byte1  목표각 하위
            (dq >> 4) & 0xFF,                           # Byte2  목표속도 상위 8
            ((dq & 0x0F) << 4) | ((kp_u >> 8) & 0x0F),  # Byte3  속도 하위4 | Kp 상위4
            kp_u & 0xFF,                                # Byte4  Kp 하위 8
            (kd_u >> 4) & 0xFF,                         # Byte5  Kd 상위 8
            ((kd_u & 0x0F) << 4) | ((tau >> 8) & 0x0F), # Byte6  Kd 하위4 | 토크 상위4
            tau & 0xFF,                                 # Byte7  토크 하위 8
        ]
    )


def decode_state(data: bytes, *, enc: EncodingRange) -> Tuple[int, float, float, float, float]:
    """상태 프레임을 해석함.

    반환: (motor_id, position_deg, velocity_deg_s, torque_nm, temp_c)
    """
    if len(data) < FRAME_LEN:
        raise ValueError(f"상태 프레임은 {FRAME_LEN}바이트여야 함 (받은 길이 {len(data)})")

    motor_id = int(data[0])
    q_u = (data[1] << 8) | data[2]
    dq_u = (data[3] << 4) | (data[4] >> 4)
    tau_u = ((data[4] & 0x0F) << 8) | data[5]
    temp_u = ((data[6] & 0x0F) << 8) | data[7]   # 12bit. 상위 4비트는 플래그임

    pos_rad = uint_to_float(q_u, -enc.pmax_rad, enc.pmax_rad, enc.pos_bits)
    vel_rad = uint_to_float(dq_u, -enc.vmax_rad_s, enc.vmax_rad_s, enc.vel_bits)
    tau_nm = uint_to_float(tau_u, -enc.tmax_nm, enc.tmax_nm, enc.tau_bits)

    return (
        motor_id,
        math.degrees(pos_rad),
        math.degrees(vel_rad),
        tau_nm,
        float(temp_u) / 10.0,
    )


def decode_flags(data: bytes) -> StatusFlags:
    """상태 프레임 Byte6 의 모드·고장·경고 비트를 뽑음.

    `decode_state` 와 나눠 둔 이유 — 하나는 인코딩 범위가 필요한 물리량이고 이건
    범위와 무관한 플래그임. 범위표가 틀려도 이 값은 맞음.
    """
    if len(data) < FRAME_LEN:
        raise ValueError(f"상태 프레임은 {FRAME_LEN}바이트여야 함 (받은 길이 {len(data)})")
    b6 = data[6]
    return StatusFlags(
        mode=(b6 >> 6) & 0x03,
        fault=bool(b6 & 0x20),
        warning=bool(b6 & 0x10),
    )


def decode_fault(data: bytes) -> Tuple[int, int]:
    """고장 응답 프레임을 해석함. 반환: (motor_id, fault_word). 0이면 정상.

    고장값은 **작은 자리 먼저** 임 — `data[1]` 이 최하위 바이트. 상태 프레임이
    큰 자리 먼저인 것과 반대임 (모듈 첫머리 설명 참조).

    일반 상태 프레임과 CAN ID가 같아 겉으로 구분되지 않으므로, 조회 명령을 보낸
    직후의 첫 응답으로 간주해야 함.
    """
    if len(data) < 5:
        raise ValueError(f"고장 프레임은 최소 5바이트 필요 (받은 길이 {len(data)})")
    motor_id = int(data[0])
    word = data[1] | (data[2] << 8) | (data[3] << 16) | (data[4] << 24)
    return motor_id, word
