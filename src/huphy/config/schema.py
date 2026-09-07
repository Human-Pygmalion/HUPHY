"""설정 자료형 — 사람이 적는 것의 구조.

`config/robot.yaml` 을 읽어 들이면 이 모양이 됨. 파일 읽기는 `loader.py` 가 함.

전부 `frozen` 임. 설정은 시작할 때 한 번 읽고 나면 바뀌지 않음 — 제어 중에 누가
한계를 넓히거나 게인을 올리는 일이 없어야 함.


## 종류와 개체를 나눔

    limbs:
      right_leg:          <- 개체 이름. 이 로봇에서 이 팔다리를 부르는 이름
        kind: leg         <- 종류. 어떤 기구학을 쓰는지
        side: right       <- 기하. 거울상인지

셋을 한 필드가 겸하면 확장할 때 막힘 (이슈 #5). 팔이 붙으면 `left_arm`, `right_arm`
이 생기고, 허리처럼 좌우가 없는 것도 생김.

개체 이름이 키인 이유: 같은 이름이 둘일 수 없게 하려는 것임.


## 각도는 전부 cal 공간임

`Motor.limits_deg` 는 관절 각도임. 모터가 보고하는 raw 값이 아님.
자세한 것은 `motors/README.md` 의 "raw 와 cal" 참조.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Optional, Tuple

from ..motors.base import Motor

DEFAULT_CONTROL_HZ = 100.0
DEFAULT_INTERFACE = "socketcan"


@dataclass(frozen=True)
class SafetyConfig:
    """명령이 모터로 나가기 전에 통과할 관문의 설정.

    `safety.guards.apply` 가 그대로 받아 씀.
    """

    command_margin_deg: float = 3.0
    """한계에서 이만큼 안쪽까지만 명령함.

    오버슛·관성·측정오차를 흡수하는 여유임. 게인을 튜닝하면 필요한 값이 달라짐 —
    지금 3° 는 임의로 잡은 출발점임.
    """

    max_delta_deg: float = 50.0
    """**측정 위치**에서 한 주기에 벗어날 수 있는 최대 각도 — 폭주 상한.

    100Hz 기준이라 50° 는 초당 5000° 임. 실제로 그렇게 도는 것이 아니라, 계산이
    튀었을 때 그 이상은 안 나가게 막는 상한임.

    **속도 제한으로 쓰지 말 것** (2026-09-07 벤치). 이 값을 낮추면 명령과 현재각의
    차이가 그만큼으로 묶이는데, MIT 식이 ``tau = kp*(명령각 - 현재각) + ...`` 이라
    **토크가 ``kp * 이 값`` 으로 함께 묶인다.** 벤치에서 3° 로 낮춘 결과 kp 30 에서
    최대 1.57 N·m 밖에 못 냈고, 기준점이 측정값이라 모터가 뒤처지면 명령도 안
    나아가 느릴수록 더 느려졌다 (이론 300°/s 설정에서 실측 49°/s).

    속도를 제한하려면 :attr:`max_vel_deg_s` 를 쓸 것.
    """

    max_vel_deg_s: Optional[float] = None
    """진짜 속도 제한. 명령 설정점이 초당 몇 도까지 나아갈 수 있는가.

    None 이면 속도 제한 없음 — 설정하지 않은 로봇은 예전과 동일하게 동작함.

    **각도/초로 적는다.** 이것을 쓰는 제어 루프가 자기 주기로
    ``max_vel_deg_s / control_hz`` 를 계산해 한 주기 몫으로 환산함. 상수를 "한
    주기에 몇 도" 로 적으면 제어 주기를 바꿨을 때 실효 속도가 조용히 따라 변한다 —
    벤치의 3° 가 100 Hz 와 곱해져 300°/s 가 "우연히" 정해졌던 것이 그 예임.

    기준점이 **직전 명령**이라(``safety.guards.clamp_rate``) 명령은 모터가 뒤처지든
    말든 보장된 속도로 나아가고, 오차는 자유롭게 커져 따라잡을 토크를 다 씀.

    데이터시트 값(출력단, RS03/RS04 매뉴얼):

    ==========  =========  =========  =========
    조건        rpm        rad/s      °/s
    ==========  =========  =========  =========
    무부하      200 ±10%   20.94      1200
    RS03 정격   180        18.85      1080
    RS04 정격   167        17.49      1002
    ==========  =========  =========  =========

    기준은 **정격**이어야 함. 무부하 200 rpm 은 토크 0 일 때만 나오는 값이라 어떤
    부하에서도 유지되지 않음. 벤치에서는 그 일부만 씀.
    """

    max_vel_deg_s_by_motor: Dict[str, float] = field(default_factory=dict)
    """모터별 속도 제한. :attr:`max_vel_deg_s` 를 덮어씀.

    관절마다 감속비도 부하도 다르므로(무릎 RS04 대 발목 RS00) 하나의 값으로 묶는
    것이 맞지 않음. 여기 없는 모터는 :attr:`max_vel_deg_s` 를 씀.
    """

    def vel_limit_deg_s(self, motor_name: str) -> Optional[float]:
        """이 모터에 적용할 속도 제한. 없으면 None (제한 없음)."""
        v = self.max_vel_deg_s_by_motor.get(motor_name)
        return self.max_vel_deg_s if v is None else float(v)

    def max_step_deg(self, motor_name: str, control_hz: float) -> Optional[float]:
        """한 주기 몫으로 환산한 값. 루프가 자기 ``control_hz`` 를 넘겨 부름."""
        v = self.vel_limit_deg_s(motor_name)
        if v is None or control_hz <= 0:
            return None
        return float(v) / float(control_hz)

    enforce_limits: bool = True
    """`False` 로 두면 한계 클리핑을 건너뜀. **커미셔닝 전용임.**

    캘리브레이션 전에는 한계가 어디인지 모르므로 검사할 것이 없음. 제어에서는
    켜 둘 것.
    """

    link_loss_cycles: int = 5
    """이 주기 수만큼 연속으로 응답이 없으면 **로봇을 세움.** 100Hz 에서 50ms.

    한 주기 빠지는 것은 흔함 — 다음 주기에 옴. 이어지면 그 모터는 현재 위치를
    모른 채 마지막 명령을 유지하고 있음. 양다리에서는 한쪽만 움직이게 되어
    로봇이 넘어짐.

    `0` 이면 판정하지 않음. 커미셔닝처럼 응답이 원래 드문 상황용임.

    제어 모드에서만 봄 — 관찰 모드는 토크가 없어서 세울 것이 없음.
    """

    def __post_init__(self) -> None:
        if self.command_margin_deg < 0:
            raise ValueError(f"command_margin_deg 는 0 이상이어야 함 (받은 값 {self.command_margin_deg})")
        if self.max_delta_deg <= 0:
            raise ValueError(f"max_delta_deg 는 0보다 커야 함 (받은 값 {self.max_delta_deg})")
        if self.link_loss_cycles < 0:
            raise ValueError(
                f"link_loss_cycles 는 0 이상이어야 함 (받은 값 {self.link_loss_cycles}). "
                f"0 은 판정하지 않는다는 뜻임"
            )


@dataclass(frozen=True)
class TelemetryConfig:
    """관측 설정. 제어에 영향을 주지 않음."""

    host: Optional[str] = None
    """UDP 수신 주소. `None` 이면 송신하지 않음."""

    port: int = 9870
    csv_path: Optional[str] = None
    csv_flush_every: int = 50
    """N 주기마다 디스크에 씀. 매번 쓰면 제어 주기가 튐."""

    @property
    def udp_enabled(self) -> bool:
        return bool(self.host)


@dataclass(frozen=True)
class ImuConfig:
    """IMU 하나. **로봇 아래에 있고, 어디 붙었는지는 `mount` 가 말함.**

    팔다리 안에 두지 않는 이유: 같은 센서가 다리에 붙었다가 몸통으로 옮겨감. 팔다리
    안에 두면 옮길 때 설정 구조가 바뀌고 텔레메트리 필드 이름도 따라 바뀜.
    """

    name: str
    """개체 이름. `imus` 의 키와 같음. **텔레메트리 필드 앞에 붙음.**"""

    model: str
    """어느 센서인지. `sensors.registry.MODELS` 가 구현체로 품.

    `Motor.model` 과 같은 자리임 -- 이 계층은 벤더를 모르고, 유효한지는 센서 모듈이
    판단함.
    """

    port: str
    """시리얼 장치 경로.

    **udev 로 고정한 심볼릭 링크를 쓸 것.** USB 는 꽂는 순서대로 `ttyUSB0`,
    `ttyUSB1` 이 붙어 재부팅마다 달라짐.
    """

    baudrate: Optional[int] = None
    """센서에 저장된 값과 반드시 같아야 함. 다르면 조용히 아무 패킷도 안 들어옴.

    `None` 이면 **벤더 모듈의 기본값**을 씀. 여기 숫자를 하나 박아 두지 않는 이유:
    출하 기본값이 센서마다 다름 (EBIMU 115200, Xsens MTi 921600). 한쪽 값을 여기
    기본값으로 두면 다른 쪽 설정에서 이 줄을 생략했을 때 조용히 안 붙음.
    """

    mount: Optional[str] = None
    """어디에 붙었는지. 팔다리 이름이거나 `torso` 처럼 팔다리가 아닌 자리.

    `None` 이면 아직 안 정한 것임. 다리에서 몸통으로 옮길 때 **여기 한 줄만** 바뀜.
    """

    output: Tuple[str, ...] = ("quat", "gyro", "accel")
    """센서가 보내는 항목. **순서가 곧 패킷 안의 순서임.**

    센서에 저장되는 값이지만 여기가 기준임 -- `huphy-imu apply` 가 이 목록대로
    센서를 맞춤. 반대로 하면(센서에 물어보고 코드가 따라감) 센서를 갈아 끼울 때마다
    동작이 조용히 달라짐.

    적어 두어야 하는 이유: 패킷에 **무엇이 켜져 있는지가 안 적혀 있음.** 숫자만
    오고, 필드 수가 같은 조합이 여럿이라 개수로도 알 수 없음.
    """

    accel_mode: str = "gravity"
    """가속도 종류. `gravity` / `local` / `global`.

    `gravity` 는 중력 포함이라 정지 상태에서 중력방향이 그대로 읽힘. **부착 방향
    검사(`huphy-imu check`)가 이것에 기대므로 바꾸지 말 것.**

    따로 적는 이유: 셋 다 필드 수가 3으로 같아 패킷으로는 구분이 안 됨.
    """

    dist_mode: str = "local"
    """거리 기준. `local` 은 센서 축, `global` 은 동서남북.

    다리에 붙은 센서면 `local` 임. 이 값도 필드 수가 같아 패킷으로 구분이 안 됨.
    """

    rate_hz: float = 100.0
    """센서 출력 주기. **제어 주기와 맞출 것.**

    센서가 느리면 제어 루프가 같은 값을 두 번 보고, 빠르면 받은 패킷을 버림.
    센서는 1ms 단위로만 설정되므로 나누어떨어지는 값이어야 함.
    """

    def __post_init__(self) -> None:
        if not self.name:
            raise ValueError("IMU 이름이 비어 있음")
        if not self.model:
            raise ValueError(f"{self.name}: model 이 비어 있음")
        if not self.port:
            raise ValueError(f"{self.name}: port 가 비어 있음")
        if self.baudrate is not None and self.baudrate <= 0:
            raise ValueError(f"{self.name}: baudrate 는 0보다 커야 함 (받은 값 {self.baudrate})")
        if not self.output:
            raise ValueError(f"{self.name}: output 이 비어 있음")
        if self.rate_hz <= 0:
            raise ValueError(f"{self.name}: rate_hz 는 0보다 커야 함 (받은 값 {self.rate_hz})")
        object.__setattr__(self, "output", tuple(self.output))


@dataclass(frozen=True)
class LimbConfig:
    """팔다리 하나. **CAN 채널 하나에 대응함.**

    다리 하나가 버스 하나를 쓰는 이유: 양다리를 한 버스에 묶으면 12개 모터의
    프레임이 같은 선을 나눠 쓰게 되어 주기 예산이 두 배가 됨. 두 버스는 물리적으로
    독립이라 진짜로 겹쳐 보낼 수 있음 (이슈 #10).
    """

    name: str
    """개체 이름. `limbs` 의 키와 같음."""

    kind: str
    """종류. `leg`, `arm` 등. 어떤 기구학을 쓸지 정함."""

    side: Optional[str] = None
    """`left` / `right`. 좌우가 없는 부위는 `None`.

    거울상 여부를 정함. 실제 부호 뒤집기는 캘리브레이션의 `sign` 이 함 — 여기는
    "거울상이다" 라는 사실만 적음.
    """

    channel: str = "can0"
    interface: str = DEFAULT_INTERFACE
    control_hz: float = DEFAULT_CONTROL_HZ

    motors: Dict[str, Motor] = field(default_factory=dict)
    """관절 이름 -> 모터. 이름은 이 팔다리 안에서만 유일하면 됨.

    `hip_pitch`, `knee` 처럼 짧게 씀. 어느 다리인지는 `LimbConfig.name` 이 이미 말함.
    """

    calibration_path: Optional[Path] = None
    """실측값 파일. `None` 이면 미캘리브레이션 상태임."""

    def __post_init__(self) -> None:
        if not self.name:
            raise ValueError("팔다리 이름이 비어 있음")
        if not self.channel:
            raise ValueError(f"{self.name}: channel 이 비어 있음")
        if self.control_hz <= 0:
            raise ValueError(f"{self.name}: control_hz 는 0보다 커야 함 (받은 값 {self.control_hz})")
        if self.side not in (None, "left", "right"):
            raise ValueError(f"{self.name}: side 는 left/right 여야 함 (받은 값 {self.side!r})")

        ids = [m.id for m in self.motors.values()]
        dup = {i for i in ids if ids.count(i) > 1}
        if dup:
            raise ValueError(
                f"{self.name}: 같은 CAN id 를 여러 관절이 씀 {sorted(dup)}. "
                f"한 버스에서 id 가 겹치면 응답이 충돌해 구분되지 않음"
            )

    @property
    def period_s(self) -> float:
        return 1.0 / self.control_hz

    @property
    def motor_ids(self) -> Tuple[int, ...]:
        return tuple(m.id for m in self.motors.values())

    def motors_by_id(self) -> Dict[int, Motor]:
        """`RobStrideBus` 가 받는 형태. 관절 이름을 버린 것임."""
        return {m.id: m for m in self.motors.values()}

    def joint_of(self, motor_id: int) -> Optional[str]:
        """모터 id 로 관절 이름을 찾음. 진단 메시지에 씀."""
        for joint, motor in self.motors.items():
            if motor.id == motor_id:
                return joint
        return None

    @property
    def is_configured(self) -> bool:
        """모든 모터에 한계와 게인이 채워졌는지."""
        return bool(self.motors) and all(m.is_configured for m in self.motors.values())

    def unconfigured(self) -> Tuple[str, ...]:
        """아직 채워지지 않은 관절 이름들."""
        return tuple(name for name, m in self.motors.items() if not m.is_configured)


@dataclass(frozen=True)
class RobotConfig:
    """로봇 전체.

    지금은 다리 하나지만 팔·상체가 붙어도 이 구조가 유지됨 — `limbs` 에 항목이
    늘어날 뿐임.
    """

    name: str
    limbs: Dict[str, LimbConfig] = field(default_factory=dict)
    imus: Dict[str, ImuConfig] = field(default_factory=dict)
    """개체 이름 -> IMU. 팔다리와 나란히 있음. 없어도 로봇은 돎."""

    safety: SafetyConfig = field(default_factory=SafetyConfig)
    telemetry: TelemetryConfig = field(default_factory=TelemetryConfig)
    source_path: Optional[Path] = None
    """읽어 온 파일. 오류 메시지에 씀."""

    def __post_init__(self) -> None:
        if not self.name:
            raise ValueError("로봇 이름이 비어 있음")

        # 한 포트를 두 IMU 가 쓰면 둘 다 제대로 못 읽음. 같은 바이트열을 나눠 갖게 됨.
        ports: Dict[str, str] = {}
        for imu in self.imus.values():
            if imu.port in ports:
                raise ValueError(
                    f"{imu.port} 를 {ports[imu.port]} 와 {imu.name} 가 같이 씀. "
                    f"한 포트를 둘이 읽으면 바이트열이 나뉘어 둘 다 깨짐"
                )
            ports[imu.port] = imu.name

        # 채널이 겹치면 두 팔다리가 같은 선을 쓰는 것이므로, id 도 겹치면 안 됨.
        by_channel: Dict[str, list] = {}
        for limb in self.limbs.values():
            by_channel.setdefault(limb.channel, []).extend(
                (limb.name, joint, m.id) for joint, m in limb.motors.items()
            )
        for channel, entries in by_channel.items():
            seen: Dict[int, str] = {}
            for limb_name, joint, motor_id in entries:
                if motor_id in seen:
                    raise ValueError(
                        f"{channel}: CAN id {motor_id} 를 "
                        f"{seen[motor_id]} 와 {limb_name}.{joint} 가 같이 씀"
                    )
                seen[motor_id] = f"{limb_name}.{joint}"

    def limb(self, name: str) -> LimbConfig:
        try:
            return self.limbs[name]
        except KeyError:
            raise KeyError(
                f"{name!r} 이라는 팔다리가 없음 (가용: {sorted(self.limbs)})"
            ) from None

    def limbs_of_kind(self, kind: str) -> Dict[str, LimbConfig]:
        """종류로 골라냄. 두 다리에 같은 처리를 걸 때 씀."""
        return {n: l for n, l in self.limbs.items() if l.kind == kind}

    def imus_on(self, mount: str) -> Dict[str, ImuConfig]:
        """그 자리에 붙은 IMU 들. 없으면 빈 사전.

        조립할 때 `imus_on(limb.name)` 으로 그 팔다리 몫만 가져감. 몸통으로 옮기면
        여기서 안 걸리고, 다리 코드는 손댈 일이 없음.
        """
        return {n: i for n, i in self.imus.items() if i.mount == mount}

    @property
    def channels(self) -> Tuple[str, ...]:
        """쓰이는 CAN 채널들. 중복 없이, 나온 순서대로."""
        out = []
        for limb in self.limbs.values():
            if limb.channel not in out:
                out.append(limb.channel)
        return tuple(out)

    @property
    def motor_count(self) -> int:
        return sum(len(l.motors) for l in self.limbs.values())
