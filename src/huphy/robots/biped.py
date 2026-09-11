"""여러 팔다리를 로봇 하나로 묶음.

`Robot` 계약을 채우므로 제어 루프는 이것이 다리 하나인지 양다리인지 모름.
`Leg` 를 고치지 않고 위에 얹힘 — 다리는 자기가 묶였다는 것을 알 필요가 없음.


## 왜 따로 있나

다리 하나일 때는 `Leg` 가 곧 로봇이었음. 팔다리가 둘이 되면 **어느 계층도 맡지
않는 일**이 생김.

    전송 순서       두 다리 명령이 같은 시각에 나가야 함
    이름 구분       무릎이 둘이 됨
    정지            한쪽만 멈추면 그쪽으로 주저앉음

셋 다 다리 하나의 책임이 아님. 다리는 자기 버스만 알고, 다른 다리가 있는지도 모름.


## 관절 이름에 팔다리 이름이 붙음

    right_leg/knee      knee 가 둘이므로 구분이 필요함

구분자가 `/` 인 이유: 관찰 필드가 이미 `knee.pos` 처럼 `.` 을 씀. 같은 문자를
쓰면 `right_leg.knee.pos` 에서 어디까지가 팔다리 이름인지 갈리지 않음.

**팔다리 이름은 `Leg.id` 임** — `robot.yaml` 의 `limbs` 키가 그대로 옴. 설정과
로그와 명령이 같은 말을 쓰게 됨.


## 전송을 몰아서 함

    왼다리 계산 -> 오른다리 계산 -> 왼다리 전송 -> 오른다리 전송 -> 수거

두 CAN 채널은 물리적으로 독립이라 실제로 겹쳐 보낼 수 있음. 다리마다
`send_action()` 을 부르면 앞 다리가 응답을 기다리는 동안 뒤 다리는 아직 보내지도
못해서, 두 다리의 명령 시각이 응답 대기 시간만큼 벌어짐 (이슈 #10).

그래서 `send_action()` 을 쓰지 않고 계산·전송·수거를 직접 엮음.


## IMU 는 팔다리와 나란히 있음

    Biped
     ├ Leg("right_leg")
     ├ Leg("left_leg")
     └ ImuGroup

몸통에 붙은 센서는 어느 다리의 것도 아님. 다리가 들고 있으면 센서를 몸통으로
옮길 때 소유자가 바뀌어야 하는데, `mount` 한 줄만 고쳐도 되게 하려는 것이
설정을 그렇게 나눈 이유였음 (`config/schema.py` 의 `ImuConfig`).

다리에 붙은 센서는 여전히 그 다리가 들고 있어도 됨 — 조립하는 쪽이 정함.


## 명령 꾸러미가 팔다리별로 나뉨

    {"right_leg": {10: MitCommand(...)}, "left_leg": {4: MitCommand(...)}}

모터 id 는 **버스 안에서만** 유일함. 채널이 다르면 같은 id 가 양쪽에 있을 수
있으므로 한 사전에 담으면 한쪽이 조용히 덮임.
"""

from __future__ import annotations

import logging
from typing import Any, Dict, Iterable, Optional, Sequence, Tuple

from ..config.schema import SafetyConfig
from ..sensors.group import ImuGroup
from .base import Action, Commands, Observation, Robot

logger = logging.getLogger(__name__)

SEP = "/"
"""팔다리 이름과 관절 이름 사이의 구분자. `right_leg/knee`."""


def split_name(name: str) -> Tuple[Optional[str], str]:
    """`right_leg/knee` -> `("right_leg", "knee")`. 구분자가 없으면 `(None, name)`."""
    part, sep, rest = name.partition(SEP)
    return (part, rest) if sep else (None, name)


def join_name(part: str, name: str) -> str:
    return f"{part}{SEP}{name}"


class Biped(Robot):
    """팔다리 여럿을 한 로봇으로. 지금은 다리 둘이지만 팔이 붙어도 같음.

    **부품을 고치지 않음.** 각 팔다리는 자기 버스·캘리브레이션·기구학을 그대로
    들고 있고, 이 클래스는 순서와 이름만 맡음.
    """

    name = "biped"

    def __init__(
        self,
        parts: Sequence[Robot],
        *,
        id: str = "biped",
        safety: Optional[SafetyConfig] = None,
        imus: Iterable[Any] = (),
    ) -> None:
        if not parts:
            raise ValueError("팔다리가 하나도 없음")

        ids = [p.id for p in parts]
        if not all(ids):
            raise ValueError(
                f"이름이 빈 팔다리가 있음 {ids}. 관절 이름 앞에 붙는 값이라 "
                f"비어 있으면 구분이 안 됨"
            )
        duplicate = {i for i in ids if ids.count(i) > 1}
        if duplicate:
            raise ValueError(
                f"팔다리 이름이 겹침 {sorted(duplicate)}. 관절 이름이 같아져 "
                f"명령이 한쪽으로만 감"
            )

        self.id = id
        self.parts: Tuple[Robot, ...] = tuple(parts)
        self._by_id: Dict[str, Robot] = {p.id: p for p in self.parts}
        self.safety = safety if safety is not None else SafetyConfig()
        """로봇 전체의 안전 설정. **판정은 각 팔다리가 함.**

        여기 두는 이유는 값의 출처를 한 곳으로 두려는 것임 — 팔다리마다 다른 여유를
        주면 어느 쪽이 먼저 걸리는지 로그만 보고 알 수 없음.
        """

        self.imus = ImuGroup(imus, owner=id)
        """로봇 전체에 붙은 IMU. **팔다리와 나란히 있음.**

        몸통 센서는 어느 다리의 것도 아니고, 다리 센서를 몸통으로 옮길 때 소유자가
        바뀌지 않아야 함. 다리가 자기 센서를 따로 들고 있어도 됨 -- 조립하는 쪽이
        어디에 넣을지 정함.
        """

    def __repr__(self) -> str:
        return f"Biped({self.id}, {[p.id for p in self.parts]})"

    def part(self, name: str) -> Robot:
        try:
            return self._by_id[name]
        except KeyError:
            raise KeyError(
                f"{name!r} 이라는 팔다리가 없음 (가용: {sorted(self._by_id)})"
            ) from None

    # ---- 구조 -------------------------------------------------------------
    @property
    def joint_names(self) -> Tuple[str, ...]:
        return tuple(
            join_name(p.id, j) for p in self.parts for j in p.joint_names
        )

    @property
    def motor_names(self) -> Tuple[str, ...]:
        return tuple(
            join_name(p.id, m) for p in self.parts for m in _motor_names(p)
        )

    @property
    def torque_motors(self) -> Tuple[str, ...]:
        """토크를 실어 보낼 수 있는 모터. 팔다리가 내는 목록을 이어 붙임."""
        return tuple(
            join_name(p.id, m)
            for p in self.parts
            for m in getattr(p, "torque_motors", ())
        )

    @property
    def observation_features(self) -> Dict[str, type]:
        return self._merged("observation_features")

    @property
    def action_features(self) -> Dict[str, type]:
        return self._merged("action_features")

    def _merged(self, attribute: str) -> Dict[str, type]:
        out: Dict[str, type] = {}
        for part in self.parts:
            for name, kind in getattr(part, attribute).items():
                out[join_name(part.id, name)] = kind
        return out

    # ---- 수명 -------------------------------------------------------------
    @property
    def is_connected(self) -> bool:
        """**팔다리가 전부** 붙어 있어야 참임.

        하나라도 빠지면 그 다리는 명령을 못 받는데, 나머지만 움직이면 로봇이
        넘어짐. 부분 연결을 정상으로 보지 않음.

        **IMU 는 안 봄.** 관측이라 없어도 로봇은 돎.
        """
        return all(p.is_connected for p in self.parts)

    def connect(self) -> None:
        """전부 엶. **하나라도 실패하면 이미 연 것을 닫고 올림.**

        절반만 연결된 채로 진행하면 그 상태가 위의 판단을 흐림 — 한 다리만 토크가
        들어간 로봇이 만들어짐.
        """
        opened = []
        try:
            for part in self.parts:
                part.connect()
                opened.append(part)
        except Exception:
            for part in opened:
                try:
                    part.disconnect()
                except Exception as e:
                    logger.warning("%s 정리 실패: %s", part.id, e)
            raise

        # 팔다리가 다 붙은 뒤에 엶. **실패해도 올리지 않음** -- IMU 는 관측이지
        # 제어가 아니라, 센서 때문에 로봇을 못 움직이면 안전한 자세로 되돌리는
        # 것조차 막힘. 값이 필요한 쪽이 시작 전에 확인함.
        self.imus.connect()

    def disconnect(self) -> None:
        """전부 닫음. **하나가 실패해도 나머지를 닫음.**

        여기서 멈추면 다른 다리에 토크가 남음.
        """
        self.imus.disconnect()
        for part in self.parts:
            try:
                part.disconnect()
            except Exception as e:
                logger.warning("%s 종료 실패 (계속 진행함): %s", part.id, e)

    def enable(self) -> None:
        """토크를 넣음. **하나라도 실패하면 전부 끊고 올림.**

        한 다리만 힘이 들어간 로봇은 반드시 넘어짐.
        """
        enabled = []
        try:
            for part in self.parts:
                part.enable()
                enabled.append(part)
        except Exception:
            for part in enabled:
                try:
                    part.disable()
                except Exception as e:
                    logger.warning("%s 토크 차단 실패: %s", part.id, e)
            raise

    def disable(self) -> None:
        for part in self.parts:
            try:
                part.disable()
            except Exception as e:
                logger.warning("%s 토크 차단 실패 (계속 진행함): %s", part.id, e)

    # ---- 캘리브레이션 -----------------------------------------------------
    @property
    def is_calibrated(self) -> bool:
        return all(p.is_calibrated for p in self.parts)

    def uncalibrated(self) -> Tuple[str, ...]:
        """실측이 덜 된 팔다리 이름. 사람에게 무엇을 재야 하는지 알려 줌."""
        return tuple(p.id for p in self.parts if not p.is_calibrated)

    def calibrate(self) -> None:
        for part in self.parts:
            part.calibrate()

    # ---- 관찰 -------------------------------------------------------------
    def get_observation(self) -> Observation:
        """팔다리별 관찰을 이름 앞에 팔다리를 붙여 합침. **새로 읽지 않음.**"""
        out: Observation = {}
        for part in self.parts:
            for name, value in part.get_observation().items():
                out[join_name(part.id, name)] = value
        return out

    def imu_states(self) -> Dict[str, Any]:
        """붙은 IMU 들의 최신 값. 개체 이름 -> `ImuState`.

        **팔다리에 달린 것도 같이 냄.** 텔레메트리와 정책은 센서가 어디 달렸는지가
        아니라 개체 이름으로 값을 찾음 — 다리에서 몸통으로 옮겨도 이름이 그대로임.
        """
        out = dict(self.imus.states())
        for part in self.parts:
            states = getattr(part, "imu_states", None)
            if callable(states):
                out.update(states())
        return out

    @property
    def all_imus(self) -> Tuple[Any, ...]:
        """로봇에 달린 것과 팔다리에 달린 것을 합친 목록."""
        return tuple(self.imus) + tuple(
            imu for part in self.parts for imu in getattr(part, "imus", ())
        )

    def link_status(self, now: Optional[float] = None) -> Dict[str, Dict[str, float]]:
        """모터별 링크 상태. `팔다리/모터` -> `{age, ack, miss}`.

        판정은 팔다리가 함 — 여기는 이름만 붙임.
        """
        out: Dict[str, Dict[str, float]] = {}
        for part in self.parts:
            status = getattr(part, "link_status", None)
            if not callable(status):
                continue
            for motor, value in status(now).items():
                out[join_name(part.id, motor)] = value
        return out

    def link_counts(self) -> Dict[str, Dict[str, int]]:
        """모터별 누적 무응답. `팔다리/모터` -> `{asked, missed}`. 세는 것은 팔다리가 함."""
        out: Dict[str, Dict[str, int]] = {}
        for part in self.parts:
            counts = getattr(part, "link_counts", None)
            if not callable(counts):
                continue
            for motor, value in counts().items():
                out[join_name(part.id, motor)] = value
        return out

    def since_clip(self, now: Optional[float] = None) -> float:
        """마지막 클리핑 이후 경과 (초). 없었으면 -1.

        **가장 최근 것**을 냄. 어느 다리에서 났는지는 팔다리별 값이 말함.
        """
        return _most_recent(p.since_clip(now) for p in self.parts if hasattr(p, "since_clip"))

    def since_reject(self, now: Optional[float] = None) -> float:
        return _most_recent(
            p.since_reject(now) for p in self.parts if hasattr(p, "since_reject")
        )

    # ---- 명령 -------------------------------------------------------------
    def build_commands(self, action: Action) -> Commands:
        """팔다리별로 나눠 계산함. **CAN 을 쓰지 않음.**

        반환: `{팔다리 이름: {모터 id: 명령}}`. 모터 id 는 버스 안에서만 유일해서
        한 사전에 몰 수 없음.

        명령이 없는 팔다리도 **빈 사전으로 남겨 둠** — `send` 가 팔다리 목록을
        이 꾸러미에서 읽으므로, 빼 버리면 그 다리만 조용히 건너뛰어짐.
        """
        split = self.split_action(action)
        return {part.id: part.build_commands(split[part.id]) for part in self.parts}

    def split_action(self, action: Action) -> Dict[str, Dict[str, float]]:
        """`{"right_leg/knee": 30.0}` -> `{"right_leg": {"knee": 30.0}}`.

        **모르는 이름은 에러임.** 조용히 버리면 그 관절만 직전 명령을 유지해
        자세가 어긋나는데, 명령은 정상으로 보임.
        """
        out: Dict[str, Dict[str, float]] = {p.id: {} for p in self.parts}
        unknown = []
        for name, value in action.items():
            part, joint = split_name(name)
            if part is None or part not in out:
                unknown.append(name)
                continue
            out[part][joint] = float(value)
        if unknown:
            raise ValueError(
                f"{self.id}: 모르는 관절 {sorted(unknown)}. "
                f"이름 앞에 팔다리를 붙일 것 (예: {self.joint_names[0]!r})"
            )
        return out

    def send(self, commands: Commands) -> int:
        """**전부 보낸 뒤에 수거함.** 여기서는 보내기만 함.

        팔다리 순서대로 보냄. 순차 전송이라 프레임 사이의 지연은 남지만, 응답
        대기가 사이에 끼지 않는 것이 핵심임 — 대기는 수거로 미뤄져 있음.
        """
        sent = 0
        for part in self.parts:
            sent += part.send(commands.get(part.id, {}))
        return sent

    def collect(self) -> Tuple[Any, ...]:
        """전부 수거함. 응답이 없었던 `팔다리/모터id` 를 냄.

        **하나가 실패해도 나머지를 수거함.** 여기서 멈추면 멀쩡한 다리의 상태가
        직전 주기 값에 머물러, 가드가 옛 위치로 점프를 판정함.
        """
        return self._gather("collect")

    def refresh(self) -> Tuple[Any, ...]:
        return self._gather("refresh")

    def _gather(self, method: str) -> Tuple[Any, ...]:
        """팔다리를 돌며 수거함. **실패한 팔다리는 전부 무응답으로 셈.**

        예외를 올리지 않는 이유: 여기서 멈추면 뒤 팔다리를 수거하지 못해, 멀쩡한
        다리의 상태가 직전 주기 값에 머묾. 그 상태로 다음 명령을 만들면 가드가
        옛 위치 기준으로 점프를 판정함.

        수거 실패는 무응답과 같은 신호로 위에 올라감 — 이어지면 정지 판정에 걸림.
        """
        missing = []
        for part in self.parts:
            try:
                missing.extend(
                    join_name(part.id, str(m)) for m in getattr(part, method)()
                )
            except Exception as e:  # noqa: BLE001 - 나머지 팔다리를 마저 수거해야 함
                logger.warning("%s %s 실패 (무응답으로 셈): %s", part.id, method, e)
                missing.extend(join_name(part.id, str(m)) for m in _motor_ids(part))
        return tuple(missing)

    @property
    def last_sent(self) -> Dict[str, float]:
        out: Dict[str, float] = {}
        for part in self.parts:
            for name, value in part.last_sent.items():
                out[join_name(part.id, name)] = value
        return out

    def hold(self) -> Commands:
        """지금 자세를 유지하는 명령. 팔다리별로 나뉜 꾸러미임.

        정지 절차가 이것을 씀. **양다리가 같이 붙잡아야** 한쪽으로 주저앉지 않음.
        """
        out: Dict[str, Any] = {}
        for part in self.parts:
            hold = getattr(part, "hold", None)
            out[part.id] = hold() if callable(hold) else {}
        return out


def _motor_names(part: Robot) -> Tuple[str, ...]:
    return tuple(getattr(part, "motor_names", ()))


def _motor_ids(part: Robot) -> Tuple[int, ...]:
    config = getattr(part, "config", None)
    return tuple(getattr(config, "motor_ids", ()))


def _most_recent(values: Iterable[float]) -> float:
    """여럿 중 가장 최근 사건. 전부 없었으면 -1.

    작은 값이 최근임 -- 사건 이후 경과 시간이기 때문. `-1`(없음)은 후보에서 뺌.
    """
    found = [v for v in values if v >= 0.0]
    return min(found) if found else -1.0
