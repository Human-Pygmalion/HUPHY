"""`robot.yaml` 을 읽어 `RobotConfig` 로 만듦.

읽기만 함. 이 파일은 모터도 CAN 도 모름.


## 모르는 키를 에러로 냄

YAML 은 오타를 조용히 삼킴. `contorl_hz: 200` 이라고 쓰면 그 줄이 무시되고 기본값
100Hz 로 돕니다 — 설정을 고쳤는데 아무것도 안 바뀌는 상황이 됨.

찾기 어려운 종류의 문제라 **모르는 키가 있으면 멈춤.** 오타는 대부분 여기서 걸림.


## 기본값을 여기 두지 않음

기본값은 전부 `schema.py` 의 dataclass 에 있음. 두 군데 있으면 어느 쪽이 실제로
쓰이는지 알 수 없음.

이 파일이 하는 것은 **YAML 에 있는 키만 골라 넘기는 것**임. 없는 키는 넘기지 않고,
그러면 dataclass 의 기본값이 쓰임.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, Iterable, Mapping, Optional

from ..motors.base import Gains, Motor
from .schema import ImuConfig, LimbConfig, RobotConfig, SafetyConfig, TelemetryConfig

ROBOT_KEYS = {"name", "limbs", "imus", "safety", "telemetry"}
LIMB_KEYS = {
    "kind", "side", "channel", "interface", "control_hz", "calibration", "motors",
}
MOTOR_KEYS = {"id", "model", "kp", "kd"}
IMU_KEYS = {
    "model", "port", "baudrate", "mount",
    "output", "accel_mode", "dist_mode", "rate_hz",
}
SAFETY_KEYS = {
    "command_margin_deg", "max_delta_deg", "max_vel_deg_s", "max_vel_deg_s_by_motor",
    "enforce_limits", "link_loss_cycles",
}
TELEMETRY_KEYS = {"host", "port", "csv_path", "csv_flush_every"}

NON_LIMB_MOUNTS = {"torso", "pelvis", "head"}
"""팔다리가 아닌 부착 자리. `limbs` 에 없어도 되는 이름들임.

몸통에 붙는 IMU 가 `limbs` 항목을 요구하면, 모터가 하나도 없는 가짜 팔다리를 적어야
함.
"""


class ConfigError(ValueError):
    """설정 파일이 읽히지 않거나 앞뒤가 안 맞음."""


def _check_keys(where: str, data: Mapping[str, Any], allowed: Iterable[str]) -> None:
    unknown = sorted(set(data) - set(allowed))
    if unknown:
        raise ConfigError(
            f"{where}: 모르는 키 {unknown} (가용: {sorted(allowed)}). "
            f"오타이거나 지원하지 않는 설정임"
        )


def _pick(data: Mapping[str, Any], keys: Iterable[str]) -> Dict[str, Any]:
    """있는 키만 골라냄. 없는 것은 넘기지 않아 dataclass 기본값이 쓰이게 함."""
    return {k: data[k] for k in keys if k in data}


def _motor(where: str, joint: str, data: Mapping[str, Any]) -> Motor:
    if not isinstance(data, Mapping):
        raise ConfigError(f"{where}.{joint}: 항목이 사전이어야 함 (받은 값 {data!r})")
    if "limits_deg" in data:
        # 한계는 재는 값이라 캘리브레이션 파일에 있음. 여기에 두면 같은 값이 두
        # 군데 생기고, 어긋났을 때 어느 쪽이 진짜인지 알 수 없음 (이슈 #2).
        raise ConfigError(
            f"{where}.{joint}: limits_deg 는 여기 두지 않음. "
            f"캘리브레이션 파일에 있음 -- commission sweep 이 재서 적음"
        )
    _check_keys(f"{where}.{joint}", data, MOTOR_KEYS)

    for required in ("id", "model"):
        if required not in data:
            raise ConfigError(f"{where}.{joint}: {required} 항목이 없음")

    try:
        return Motor(
            id=int(data["id"]),
            model=str(data["model"]),
            gains=Gains(kp=float(data.get("kp", 0.0)), kd=float(data.get("kd", 0.0))),
        )
    except (TypeError, ValueError) as e:
        raise ConfigError(f"{where}.{joint}: {e}") from e


def _imu(name: str, data: Mapping[str, Any]) -> ImuConfig:
    where = f"imus.{name}"
    if not isinstance(data, Mapping):
        raise ConfigError(f"{where}: 항목이 사전이어야 함 (받은 값 {data!r})")
    _check_keys(where, data, IMU_KEYS)

    for required in ("model", "port"):
        if required not in data:
            raise ConfigError(f"{where}: {required} 항목이 없음")

    extra = dict(_pick(data, {"baudrate", "mount", "accel_mode", "dist_mode", "rate_hz"}))
    if "output" in data:
        # 문자열 하나로 적는 것도 받음 -- `output: quat,gyro,accel` 이 손으로 적기
        # 편하고, YAML 목록과 뜻이 같음.
        raw = data["output"]
        items = raw.split(",") if isinstance(raw, str) else raw
        try:
            extra["output"] = tuple(str(item).strip() for item in items)
        except TypeError as e:
            raise ConfigError(f"{where}.output: 목록이어야 함 (받은 값 {raw!r})") from e

    try:
        return ImuConfig(
            name=name,
            model=str(data["model"]),
            port=str(data["port"]),
            **extra,
        )
    except (TypeError, ValueError) as e:
        raise ConfigError(f"{where}: {e}") from e


def _limb(name: str, data: Mapping[str, Any], *, base_dir: Path) -> LimbConfig:
    where = f"limbs.{name}"
    if not isinstance(data, Mapping):
        raise ConfigError(f"{where}: 항목이 사전이어야 함")
    _check_keys(where, data, LIMB_KEYS)

    if "kind" not in data:
        raise ConfigError(f"{where}: kind 가 없음 (leg, arm 등)")

    motors_raw = data.get("motors") or {}
    if isinstance(motors_raw, list):
        raise ConfigError(
            f"{where}.motors: 목록이 아니라 사전이어야 함. "
            f"관절 이름을 키로 씀 -- knee: {{id: 10, model: RS02}}"
        )
    if not isinstance(motors_raw, Mapping):
        raise ConfigError(f"{where}.motors: 사전이어야 함 (받은 값 {type(motors_raw).__name__})")
    if not motors_raw:
        raise ConfigError(f"{where}: 모터가 하나도 없음")

    calibration = data.get("calibration")
    fields = _pick(data, ("kind", "side", "channel", "interface", "control_hz"))

    motors = {j: _motor(f"{where}.motors", j, m) for j, m in motors_raw.items()}
    try:
        return LimbConfig(
            name=name,
            motors=motors,
            calibration_path=(base_dir / calibration) if calibration else None,
            **fields,
        )
    except ConfigError:
        raise
    except (TypeError, ValueError) as e:
        # LimbConfig 의 메시지는 이미 팔다리 이름으로 시작하므로 그대로 씀.
        raise ConfigError(str(e)) from e


def load_robot(path: "str | Path") -> RobotConfig:
    """`robot.yaml` 을 읽음.

    상대 경로(`calibration:`)는 **이 파일이 있는 폴더 기준**으로 풀림. 실행 위치가
    달라져도 같은 파일을 가리키게 하려는 것임.
    """
    try:
        import yaml  # noqa: PLC0415
    except ImportError as e:
        raise ImportError("설정을 읽으려면 PyYAML 이 필요함. `pip install PyYAML`") from e

    p = Path(path)
    if not p.is_file():
        raise ConfigError(f"설정 파일이 없음: {p}")

    try:
        data = yaml.safe_load(p.read_text(encoding="utf-8"))
    except yaml.YAMLError as e:
        raise ConfigError(f"{p}: YAML 을 읽을 수 없음\n{e}") from e

    if not isinstance(data, Mapping):
        raise ConfigError(f"{p}: 최상위가 사전이어야 함 (받은 값 {type(data).__name__})")
    _check_keys(str(p), data, ROBOT_KEYS)

    limbs_raw = data.get("limbs") or {}
    if not isinstance(limbs_raw, Mapping):
        raise ConfigError(f"{p}: limbs 는 사전이어야 함")
    if not limbs_raw:
        raise ConfigError(f"{p}: limbs 가 비어 있음")

    imus_raw = data.get("imus") or {}
    if not isinstance(imus_raw, Mapping):
        raise ConfigError(f"{p}: imus 는 사전이어야 함")

    safety_raw = data.get("safety") or {}
    telemetry_raw = data.get("telemetry") or {}
    _check_keys(f"{p}: safety", safety_raw, SAFETY_KEYS)
    _check_keys(f"{p}: telemetry", telemetry_raw, TELEMETRY_KEYS)

    limbs = {n: _limb(n, d, base_dir=p.parent) for n, d in limbs_raw.items()}
    imus = {n: _imu(n, d) for n, d in imus_raw.items()}

    # 붙은 자리가 팔다리 이름이면 그런 팔다리가 있어야 함. `torso` 처럼 팔다리가
    # 아닌 자리는 그대로 둠 -- 몸통은 limbs 에 없음.
    for imu in imus.values():
        if imu.mount and imu.mount not in limbs and imu.mount not in NON_LIMB_MOUNTS:
            raise ConfigError(
                f"{p}: imus.{imu.name}.mount 가 {imu.mount!r} 인데 그런 팔다리가 없음 "
                f"(팔다리: {sorted(limbs)}, 팔다리가 아닌 자리: {sorted(NON_LIMB_MOUNTS)})"
            )

    try:
        return RobotConfig(
            name=str(data.get("name") or p.stem),
            limbs=limbs,
            imus=imus,
            safety=SafetyConfig(**_pick(safety_raw, SAFETY_KEYS)),
            telemetry=TelemetryConfig(**_pick(telemetry_raw, TELEMETRY_KEYS)),
            source_path=p,
        )
    except ConfigError:
        raise
    except (TypeError, ValueError) as e:
        raise ConfigError(f"{p}: {e}") from e
