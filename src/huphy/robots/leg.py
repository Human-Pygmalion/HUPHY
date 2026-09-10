"""다리 하나 — 관절 이름과 모터 id 를 잇는 곳.

`Robot` 계약을 채움. 설정·실측값·버스·기구학·안전을 한데 모아 쓰는 유일한 계층임.


## 좌표계와 관절 이름

**오른손 좌표계, X 가 앞(발이 나가는 방향), Z 가 위, Y 가 왼쪽.**

    roll    X축 회전    다리를 옆으로 기울임
    pitch   Y축 회전    다리를 앞뒤로 듦
    yaw     Z축 회전    발끝 방향을 돌림

관절 이름이 곧 어느 축인지를 말함. 이름만 보고 알 수 있어야 함 -- 예전 이름
(`hipz`/`hipx`/`hipy`)은 어느 좌표계 기준인지 적혀 있지 않아 매번 실물로 확인해야
했음.


## 관절과 모터가 1:1 이 아님

    hip_pitch  hip_roll  hip_yaw  knee    모터 하나가 관절 하나
    ankle_pitch  ankle_roll             모터 **두 개**가 관절 두 개를 만듦

그래서 명령은 6개 관절로 받고, 발목만 기구학을 거쳐 모터 두 개로 풀림.

    명령    hip_pitch  hip_roll  hip_yaw  knee  ankle_pitch  ankle_roll
    모터    m7         m8        m9       m10   m11          m12

기구 순서도 이 순서임 -- 몸통에서 발로 내려가며 pitch, roll, yaw, knee, 발목.


## 한 주기에 일어나는 일

    1  관절 목표 (cal)
    2  발목 pitch/roll -> a1/a2          기구학. 닫힌 해
    3  모터별 cal 목표
    4  한계·점프 검사                     safety.guards. cal 공간
    5  cal -> raw                        캘리브레이션
    6  MIT 프레임                         코덱
    7  전송
    8  수거 -> 상태 갱신

4번이 cal 공간인 이유: 한계가 cal 공간에 있음 (이슈 #2). raw 로 내린 뒤에 검사하면
`sign` 이 -1 인 관절에서 부호가 뒤집혀 한계가 반대로 걸림.


## 발목의 두 가지 실패는 다르게 다룸

**IK 가 안 풀리면 통째로 버림.** 두 모터 다 직전 명령을 유지함. 한쪽만 새 명령을
받으면 두 로드가 서로 다른 자세를 요구해 관절이 비틀림.

**한계에 잘리는 것은 개별로 처리해도 됨.** 로드가 둘이고 자유도가 둘이라 어떤
`(a1, a2)` 조합이든 대응하는 발 자세가 하나 있음 — 잘린 각도 쌍도 비틀린 자세가
아니라 그냥 다른 자세임. 실제로 실행된 자세는 FK 로 되짚어 `last_sent` 에 냄.

나머지 관절은 서로 독립이라 한 관절이 잘려도 다른 관절은 정상임.
"""

from __future__ import annotations

import logging
import time
from dataclasses import replace
from typing import Any, Dict, Mapping, Optional, Sequence, Tuple

import numpy as np

from .. import calibration as calib
from ..config.schema import LimbConfig, SafetyConfig
from ..kinematics.ankle import AnkleKinematics, AnkleUnreachableError
from ..motors.base import MotorCalibration
from ..motors.robstride.bus import MitCommand, RobStrideBus
from ..safety import guards
from ..sensors.group import ImuGroup
from .base import Action, Observation, Robot

logger = logging.getLogger(__name__)

SINGLE_JOINTS = ("hip_pitch", "hip_roll", "hip_yaw", "knee")
ANKLE_JOINTS = ("ankle_pitch", "ankle_roll")
ANKLE_MOTORS = ("ankle_a", "ankle_b")

JOINT_NAMES = SINGLE_JOINTS + ANKLE_JOINTS
REQUIRED_MOTORS = SINGLE_JOINTS + ANKLE_MOTORS
"""설정에 반드시 있어야 하는 모터 이름. 발목만 관절 이름과 다름."""

COMMANDABLE = JOINT_NAMES + ANKLE_MOTORS
"""명령으로 받을 수 있는 이름 전부. **여덟 개가 동시에 오는 것이 아님.**

발목은 둘 중 한 쌍만 옴 -- `ankle_pitch`+`ankle_roll` 이거나 `ankle_a`+`ankle_b`.
어느 쪽인지는 `_ankle_space` 가 이름을 보고 판정함.
"""

ANKLE_SPACE_JOINT = "rp"
ANKLE_SPACE_MOTOR = "ab"
"""발목 명령이 어느 공간으로 왔는지.

    rp   발판 자세 (ankle_pitch, ankle_roll)   IK 를 거쳐 모터각으로
    ab   모터 각도 (ankle_a, ankle_b)          그대로 나감

**플래그로 받지 않고 이름으로 판정함.** 플래그를 따로 들면 명령과 그 해석이
어긋날 수 있는데, 명령 자체가 어느 공간인지 말해 주면 어긋날 수가 없음.
"""

ANKLE_POSITION = "position"
ANKLE_TORQUE = "torque"
"""발목 두 모터에 무엇을 실어 보낼지.

MIT 프레임은 하나고 칸이 다섯임 (q, dq, kp, kd, tau_ff). 두 가지 명령이 있는 것이
아니라 **같은 프레임을 다르게 채우는 것**임.

    position   q=목표각  kp=게인  kd=게인  tau_ff=0    모터 펌웨어가 PD 를 함
    torque     q=0       kp=0     kd=0     tau_ff=tau  모터는 시킨 토크만 냄

모터 각도에 게인을 걸면 로드 지렛대 비가 자세마다 달라 발목이 내는 강성이 자세마다
달라짐. 관절 토크를 만들어 야코비안으로 내리면 그 비가 보정됨.
"""


class Leg(Robot):
    """다리 하나. CAN 채널 하나를 씀.

    설정에는 모터가 `ankle_a`/`ankle_b` 로 있지만, 명령은 `ankle_pitch`/
    `ankle_roll` 로 받음 — 사람과 보행 궤적이 다루는 것은 관절이지 링키지가 아님.
    """

    name = "leg"

    torque_motors = ANKLE_MOTORS
    """토크를 실어 보낼 수 있는 모터. **모드와 무관하게 고정임.**

    텔레메트리 열이 모드에 따라 나타났다 사라지면 CSV 가 밀림. 위치 모드에서는
    0으로 나감.
    """

    def __init__(
        self,
        config: LimbConfig,
        bus: RobStrideBus,
        *,
        safety: Optional[SafetyConfig] = None,
        calibration: Optional[Mapping[str, MotorCalibration]] = None,
        kinematics: Optional[AnkleKinematics] = None,
        allow_uncalibrated: bool = False,
        imus: Optional[Sequence[Any]] = None,
        ankle_output: str = ANKLE_POSITION,
        ankle_kp: Tuple[float, float] = (0.0, 0.0),
        ankle_kd: Tuple[float, float] = (0.0, 0.0),
    ) -> None:
        self.config = config
        self.id = config.name
        self.bus = bus
        self.safety = safety if safety is not None else SafetyConfig()
        self.allow_uncalibrated = allow_uncalibrated

        if ankle_output not in (ANKLE_POSITION, ANKLE_TORQUE):
            raise ValueError(
                f"{config.name}: ankle_output 은 {ANKLE_POSITION!r} 또는 "
                f"{ANKLE_TORQUE!r} 여야 함 (받은 값 {ankle_output!r})"
            )
        self.ankle_output = ankle_output
        """발목 두 모터에 무엇을 실어 보낼지. 나머지 4관절은 늘 위치임.

        한 실행 안에서 바뀌지 않음 -- 브링업은 위치로, 정책 실행은 토크로 만듦.
        섞어 쓸 일이 생기면 그때 주기마다 바꾸는 길을 내면 됨.
        """

        self.ankle_kp = (float(ankle_kp[0]), float(ankle_kp[1]))
        self.ankle_kd = (float(ankle_kd[0]), float(ankle_kd[1]))
        """발목 관절 PD 게인. `(pitch, roll)` 순서, Nm/rad 와 Nm/(rad/s).

        `ankle_output` 이 토크일 때만 씀. 모터 각도에 거는 `Motor.gains` 와 다른
        값임 -- 이쪽은 **관절 각도**에 거는 것이고, 로드 지렛대 비는 야코비안이 봄.
        """

        self.imus = ImuGroup(imus or (), owner=config.name)
        """이 다리에 붙은 IMU 들. **없어도 다리는 그대로 돎.**

        어느 다리에 붙었는지는 설정(`ImuConfig.mount`)이 정하고, 조립하는 쪽이
        골라서 넣어 줌. 다리는 어느 회사 센서인지 모름 -- `sensors.base.Imu` 만 봄.

        **다리에 붙은 센서만 여기 옴.** 몸통 IMU 는 로봇 전체의 것이라
        `Biped` 가 들고 있음 (`sensors/group.py`).
        """

        missing = [m for m in REQUIRED_MOTORS if m not in config.motors]
        if missing:
            raise ValueError(
                f"{config.name}: 다리에 필요한 모터가 없음 {missing}. "
                f"설정에 있는 것: {sorted(config.motors)}"
            )

        self.kinematics = kinematics if kinematics is not None else AnkleKinematics()

        if calibration is None:
            calibration = (
                calib.load(config.calibration_path)
                if config.calibration_path and config.calibration_path.is_file()
                else calib.identity(config.motors)
            )
        self._set_calibration(calibration)

        self._last_sent: Dict[str, float] = {}
        self._ankle_guess: Tuple[float, float] = (0.0, 0.0)
        self._ankle_pose: Optional[Tuple[float, float]] = None
        """이번 주기의 발목각. 못 풀었으면 `None`. `_update_ankle_pose` 가 채움."""

        self._last_torque: Dict[str, float] = {m: 0.0 for m in ANKLE_MOTORS}
        """이번 주기에 `tau_ff` 로 실어 보낸 토크. 안 보냈으면 0.

        모터가 보고하는 `MotorState.torque_nm` 은 **모터가 냈다는 값**이고 이쪽은
        **우리가 시킨 값**임. 둘을 같이 봐야 시킨 만큼 못 낸 것인지, 애초에 적게
        시킨 것인지 구분됨.
        """
        self.counters = guards.GuardCounters()

        # 링크 상태. 명령이 씹혔는지 판단하는 근거임 -- MIT 모드는 명령을 받으면
        # 반드시 상태 프레임으로 답하므로, 안 오면 그 모터가 처리하지 않은 것임.
        self._missing: set = set(self.config.motor_ids)
        self._miss_streak: Dict[int, int] = {i: 0 for i in self.config.motor_ids}
        self._awaiting: Tuple[int, ...] = tuple(self.config.motor_ids)
        """직전에 명령을 보낸 모터들. 응답을 기다리는 대상임."""

        # 마지막 사건 시각. 누적 카운터만으로는 언제 일어났는지 그래프에서 계단을
        # 찾아야 함. 이 값은 사건 직후 0으로 떨어져 눈에 바로 띔.
        self._last_clip_t: Optional[float] = None
        self._last_reject_t: Optional[float] = None

    # ---- 구성 -------------------------------------------------------------
    def _set_calibration(self, calibration: Mapping[str, MotorCalibration]) -> None:
        """실측값을 붙임. **한계각도 여기서 들어옴.**

        `robot.yaml` 에는 한계가 없음 -- 재는 값이라 캘리브레이션 파일에 있음
        (이슈 #2). 가드는 `Motor.limits_deg` 를 보므로, 읽어 온 값을 설정 사본에
        옮겨 넣어 한 군데서만 보게 함.
        """
        self.calibration = dict(calibration)
        self._by_id = calib.attach(self.calibration, self.config.motors)
        self.config = replace(
            self.config,
            motors={
                name: replace(motor, limits_deg=self.calibration[name].limits_deg)
                for name, motor in self.config.motors.items()
            },
        )

    def _motor_id(self, motor_name: str) -> int:
        return self.config.motors[motor_name].id

    @property
    def last_torque(self) -> Dict[str, float]:
        """직전 주기에 `tau_ff` 로 보낸 토크. 모터 이름 -> Nm."""
        return dict(self._last_torque)

    def __repr__(self) -> str:
        return f"Leg({self.id}, {self.config.channel}, 관절 {len(JOINT_NAMES)}개)"

    @property
    def joint_names(self) -> Tuple[str, ...]:
        return JOINT_NAMES

    @property
    def motor_names(self) -> Tuple[str, ...]:
        """설정에 적힌 모터 이름. 발목이 둘로 나뉘어 있음."""
        return tuple(self.config.motors)

    @property
    def observation_features(self) -> Dict[str, type]:
        """관찰 필드. 모터 단위 값과 발목각.

        모터 단위가 기본임 — 실제로 측정되는 것이 그것임. 발목 `pitch`/`roll` 만
        FK 로 푼 값이고, 모델이 관절각을 입력으로 받으므로 여기 같이 냄.
        """
        out: Dict[str, type] = {}
        for motor in self.motor_names:
            out[f"{motor}.pos"] = float
            out[f"{motor}.vel"] = float
            out[f"{motor}.torque"] = float
            out[f"{motor}.temp"] = float
        out["ankle_pitch.pos"] = float
        out["ankle_roll.pos"] = float
        out["ankle_pitch.vel"] = float
        out["ankle_roll.vel"] = float
        out["stale_motors"] = int
        return out

    @property
    def action_features(self) -> Dict[str, type]:
        """명령으로 받을 수 있는 필드. **발목이 두 갈래임.**

        여덟 개가 동시에 오는 것이 아니라 발목은 둘 중 한 쌍만 옴 --
        `ankle_pitch`+`ankle_roll` 이거나 `ankle_a`+`ankle_b`. 나머지 네 관절은
        모터와 1:1 이라 갈릴 것이 없음.
        """
        return {j: float for j in COMMANDABLE}

    # ---- 수명 -------------------------------------------------------------
    @property
    def is_connected(self) -> bool:
        return self.bus.is_connected

    def connect(self) -> None:
        self.bus.connect()
        self.refresh()
        self._connect_imus()

    def disconnect(self) -> None:
        self.imus.disconnect()
        self.bus.disconnect()

    def _connect_imus(self) -> None:
        """IMU 를 엶. **실패해도 다리는 계속 씀.**

        IMU 는 관측이지 제어가 아님. 센서가 안 붙어 있다고 다리를 못 움직이면,
        고장 났을 때 로봇을 안전한 자세로 되돌리는 것조차 못 하게 됨.
        """
        self.imus.connect()

    def imu_states(self) -> Dict[str, Any]:
        """붙은 IMU 들의 최신 값. 개체 이름 -> `ImuState`.

        **새로 통신하지 않음.** 벤더 구현이 백그라운드로 받아 둔 것을 꺼내기만 함.
        """
        return self.imus.states()

    def enable(self) -> None:
        """토크를 넣음. **캘리브레이션과 설정이 채워져 있어야 함.**

        `allow_uncalibrated=True` 로 만들었으면 경고만 하고 진행함 — 커미셔닝
        단계에서 필요함.
        """
        if not self.is_calibrated:
            message = (
                f"{self.id}: 실측값이 채워지지 않음. "
                f"미설정 관절 {self.config.unconfigured()}, "
                f"미실측 관절 {calib.unmeasured(self.calibration)}"
            )
            if not self.allow_uncalibrated:
                raise RuntimeError(
                    f"{message}. 확인했으면 allow_uncalibrated=True 로 만들 것"
                )
            logger.warning("%s (allow_uncalibrated=True 라 진행함)", message)
        self.bus.enable_torque()

    def disable(self) -> None:
        self.bus.disable_torque()

    # ---- 캘리브레이션 -----------------------------------------------------
    @property
    def is_calibrated(self) -> bool:
        """한계·게인이 채워지고 영점 메모가 남아 있는지.

        둘 다 봐야 함 — 게인만 있으면 어디까지 가도 되는지 모르고, 한계만 있으면
        토크가 안 나감.
        """
        return self.config.is_configured and not calib.unmeasured(self.calibration)

    def calibrate(self) -> None:
        """실측값 파일을 다시 읽어 들임.

        재는 절차 자체는 여기 없음 — 사람이 `scripts/commission.py` 로 함.
        """
        if not self.config.calibration_path:
            raise RuntimeError(f"{self.id}: 캘리브레이션 파일이 설정되어 있지 않음")
        self._set_calibration(calib.load(self.config.calibration_path))
        logger.info("%s 캘리브레이션 다시 읽음", self.id)

    # ---- 좌표 변환 ---------------------------------------------------------
    def raw_to_cal(self, motor_name: str, raw_deg: float) -> float:
        return self._by_id[self._motor_id(motor_name)].raw_to_cal(raw_deg)

    def cal_to_raw(self, motor_name: str, cal_deg: float) -> float:
        return self._by_id[self._motor_id(motor_name)].cal_to_raw(cal_deg)

    # ---- 관찰 -------------------------------------------------------------
    def get_observation(self) -> Observation:
        """마지막으로 수거된 상태. **새로 읽지 않음.**

        각도는 cal 공간임. 제어 루프가 `collect()` 로 갱신함.
        """
        out: Observation = {}
        stale = 0
        for motor_name in self.motor_names:
            state = self.bus.state(self._motor_id(motor_name))
            if not state.is_valid:
                stale += 1
            out[f"{motor_name}.pos"] = self.raw_to_cal(motor_name, state.position_deg)
            out[f"{motor_name}.vel"] = state.velocity_deg_s
            out[f"{motor_name}.torque"] = state.torque_nm
            out[f"{motor_name}.temp"] = state.temp_c

        # 발목각. 모터가 보고하는 값이 아니라 FK 로 푼 값임. 못 풀었으면 마지막으로
        # 알려진 자세를 냄 -- 모터각이 응답 없을 때 직전 상태를 그대로 두는 것과 같음.
        pitch, roll = self._ankle_pose if self._ankle_pose is not None else self._ankle_guess
        out["ankle_pitch.pos"] = pitch
        out["ankle_roll.pos"] = roll
        out["ankle_pitch.vel"], out["ankle_roll.vel"] = self.ankle_velocity()

        out["stale_motors"] = stale
        return out

    def ankle_pose(self) -> Optional[Tuple[float, float]]:
        """지금 발목 (pitch, roll). 아직 못 풀었으면 `None`.

        **여기서 계산하지 않음.** 상태가 갱신되는 자리(`collect`, `refresh`)에서
        주기당 한 번 풀어 둔 것을 꺼냄 -- `get_observation()` 이 한 주기에 여러 번
        불려서, 부를 때마다 풀면 같은 계산을 세 번 하게 됨.
        """
        return self._ankle_pose

    def _update_ankle_pose(self) -> None:
        """모터각에서 발목각을 풂. **주기당 한 번.**

        직전 결과를 추정으로 씀. 같은 모터각 조합이 서로 다른 자세 둘에 대응하므로
        추정이 가까워야 함.

        못 풀면 `None` 로 둠. 추정(`_ankle_guess`)은 그대로 두어 다음 주기가 마지막
        성공 지점에서 다시 시작하게 함.
        """
        a1 = self.raw_to_cal("ankle_a", self.bus.state(self._motor_id("ankle_a")).position_deg)
        a2 = self.raw_to_cal("ankle_b", self.bus.state(self._motor_id("ankle_b")).position_deg)
        try:
            pose = self.kinematics.solve_fk(
                a1, a2,
                guess_pitch_deg=self._ankle_guess[0],
                guess_roll_deg=self._ankle_guess[1],
            )
        except AnkleUnreachableError as e:
            logger.debug("%s 발목 FK 실패: %s", self.id, e)
            self._ankle_pose = None
            return
        self._ankle_guess = pose
        self._ankle_pose = pose

    # ---- 명령 -------------------------------------------------------------
    def _ankle_space(self, action: Action) -> Optional[str]:
        """발목 명령이 어느 공간으로 왔는지. 발목 명령이 없으면 `None`.

        **한 쌍을 통째로 줘야 함.** 모터 두 개가 두 자유도를 같이 만들어서, 한쪽만
        받으면 두 로드가 서로 다른 자세를 요구해 관절이 비틀림.

        **두 공간을 섞어 주는 것도 거부함.** 어느 쪽을 따를지 정할 근거가 없고,
        조용히 하나를 고르면 나머지가 무시된 줄 모른 채로 돎.
        """
        joints = set(action) & set(ANKLE_JOINTS)
        motors = set(action) & set(ANKLE_MOTORS)

        if joints and motors:
            raise ValueError(
                f"{self.id}: 발목 명령에 두 공간이 섞임 "
                f"(관절 {sorted(joints)}, 모터 {sorted(motors)}). "
                f"둘 중 한 쌍만 줄 것"
            )
        if not joints and not motors:
            return None

        pair, space = (
            (ANKLE_JOINTS, ANKLE_SPACE_JOINT) if joints else (ANKLE_MOTORS, ANKLE_SPACE_MOTOR)
        )
        if not all(name in action for name in pair):
            raise ValueError(
                f"{self.id}: 발목은 {pair[0]} 와 {pair[1]} 를 함께 줘야 함 "
                f"(받은 것: {sorted(joints or motors)}). "
                f"모터 두 개가 두 자유도를 같이 만들기 때문임"
            )
        return space

    def _motor_targets(self, action: Action, space: Optional[str]) -> Dict[str, float]:
        """명령 -> 모터별 cal 목표. **발목만 공간에 따라 갈림.**

        모르는 이름은 에러임. 오타를 조용히 무시하면 그 관절만 직전 명령을 유지해
        자세가 어긋남.

        `ab` 는 받은 값이 이미 모터 각도라 기구학을 지나가지 않음. 아래(가드,
        cal -> raw)는 원래부터 모터 이름으로 돌아가므로 손댈 것이 없음.
        """
        unknown = sorted(set(action) - set(COMMANDABLE))
        if unknown:
            raise ValueError(
                f"{self.id}: 모르는 관절 {unknown} (가용: {list(COMMANDABLE)})"
            )

        targets = {j: float(action[j]) for j in SINGLE_JOINTS if j in action}

        if space == ANKLE_SPACE_MOTOR:
            if self.ankle_output == ANKLE_TORQUE:
                # 토크 경로는 관절 공간 PD 라 pitch/roll 이 있어야 함. 그냥 두면
                # `_ankle_torque` 가 빈 사전을 내고 발목만 조용히 명령을 못 받음.
                raise ValueError(
                    f"{self.id}: 모터 공간 발목 명령({', '.join(ANKLE_MOTORS)})은 "
                    f"ankle_output={ANKLE_TORQUE!r} 와 같이 쓸 수 없음. "
                    f"{ANKLE_POSITION!r} 로 만들 것"
                )
            targets.update({m: float(action[m]) for m in ANKLE_MOTORS})

        elif space == ANKLE_SPACE_JOINT:
            if self.ankle_output == ANKLE_TORQUE:
                # 토크로 보낼 때는 모터 각도가 필요 없음. `_ankle_torque` 가 처리함.
                return targets
            try:
                a1, a2 = self.kinematics.solve_ik(
                    action["ankle_pitch"], action["ankle_roll"]
                )
            except AnkleUnreachableError as e:
                # 통째로 버림. 한쪽만 보내면 두 로드가 다른 자세를 요구해 비틀림.
                logger.warning("%s 발목 목표를 버림: %s", self.id, e)
            else:
                targets["ankle_a"] = a1
                targets["ankle_b"] = a2

        return targets

    def build_commands(self, action: Action) -> Dict[int, MitCommand]:
        """명령을 계산함. **CAN 을 쓰지 않음.**

        한계와 점프를 cal 공간에서 검사한 뒤 raw 로 내림. 무엇이 잘렸는지는
        `counters` 에 쌓임.
        """
        now = time.monotonic()
        space = self._ankle_space(action)
        commands: Dict[int, MitCommand] = {}
        sent: Dict[str, float] = {}
        # 이번 주기에 실제로 실어 보낸 것만 남김. 안 보냈으면 0임.
        self._last_torque = {m: 0.0 for m in ANKLE_MOTORS}

        for motor_name, target_cal in self._motor_targets(action, space).items():
            motor = self.config.motors[motor_name]
            motor_id = motor.id
            state = self.bus.state(motor_id)
            current_cal = (
                self.raw_to_cal(motor_name, state.position_deg)
                if state.is_valid
                else None
            )

            result = guards.apply(
                target_cal,
                current_cal,
                limits=motor.limits_deg,
                command_margin_deg=self.safety.command_margin_deg,
                max_delta_deg=self.safety.max_delta_deg,
                enforce_limits=self.safety.enforce_limits,
            )
            self.counters.record(result)
            if result.clips:
                self._last_clip_t = now
            if result.reject is not None:
                self._last_reject_t = now
            if not result.sendable:
                continue

            sent[motor_name] = result.value
            commands[motor_id] = MitCommand(
                position_deg=self.cal_to_raw(motor_name, result.value),
                kp=motor.gains.kp,
                kd=motor.gains.kd,
            )

        if self.ankle_output == ANKLE_TORQUE:
            commands.update(self._ankle_torque(action, sent))

        self._last_sent = self._as_joint_space(
            sent, keep_motors=space == ANKLE_SPACE_MOTOR
        )
        return commands

    def _ankle_torque(
        self, action: Action, sent: Dict[str, float]
    ) -> Dict[int, MitCommand]:
        """발목 두 모터에 실을 토크. 못 만들면 빈 사전.

        `kp=kd=0` 으로 보내므로 **모터는 위치를 보지 않음.** 자세를 잡는 일이 전부
        여기 계산에 달려 있고, 토크가 0이면 발목은 중력에 따라 떨어짐.

        **한계 가드가 걸리지 않음.** `guards.apply` 는 목표 각도를 받아 자르는데
        토크 명령에는 넘길 각도가 없음. 발목이 한계를 넘어가도 여기서 막지 못함
        -- 토크 쪽 가드는 아직 없음.

        지금 자세를 못 풀었으면 아무것도 안 보냄. 야코비안이 지금 자세에서 나오므로
        자세를 모르면 토크를 만들 수 없음.
        """
        if not all(j in action for j in ANKLE_JOINTS):
            return {}

        pose = self.ankle_pose()
        if pose is None:
            self.counters.rejects["ankle_nopose"] = (
                self.counters.rejects.get("ankle_nopose", 0) + 1
            )
            self._last_reject_t = time.monotonic()
            return {}

        try:
            tau1, tau2 = self.kinematics.mit_torque(
                (action["ankle_pitch"], action["ankle_roll"]),
                pose,
                current_velocity=self.ankle_velocity(),
                kp=self.ankle_kp,
                kd=self.ankle_kd,
            )
        except AnkleUnreachableError as e:
            # 통째로 버림. 한쪽만 보내면 두 로드가 다른 자세를 요구해 비틀림.
            logger.warning("%s 발목 토크를 버림: %s", self.id, e)
            self.counters.rejects["ankle_singular"] = (
                self.counters.rejects.get("ankle_singular", 0) + 1
            )
            self._last_reject_t = time.monotonic()
            return {}

        sent["ankle_pitch"] = float(action["ankle_pitch"])
        sent["ankle_roll"] = float(action["ankle_roll"])
        self._last_torque = {"ankle_a": tau1, "ankle_b": tau2}
        return {
            self._motor_id("ankle_a"): MitCommand(
                position_deg=0.0, kp=0.0, kd=0.0, torque_nm=tau1
            ),
            self._motor_id("ankle_b"): MitCommand(
                position_deg=0.0, kp=0.0, kd=0.0, torque_nm=tau2
            ),
        }

    def ankle_velocity(self) -> Tuple[float, float]:
        """지금 발목 각속도 (pitch, roll). 도/초.

        모터 속도에서 풂. `da = J dpr` 이므로 `dpr = J^-1 da` 임.

        자세를 모르거나 특이점이면 `(0, 0)` 을 냄. 감쇠가 0이 되는 것이라 부족하게
        잡히지, 반대로 밀지는 않음.
        """
        pose = self.ankle_pose()
        if pose is None:
            return (0.0, 0.0)
        try:
            jac = self.kinematics.jacobian(pose[0], pose[1])
            motor = np.array([
                self.bus.state(self._motor_id("ankle_a")).velocity_deg_s,
                self.bus.state(self._motor_id("ankle_b")).velocity_deg_s,
            ])
            joint = np.linalg.solve(jac, motor)
        except (AnkleUnreachableError, np.linalg.LinAlgError):
            return (0.0, 0.0)
        return (float(joint[0]), float(joint[1]))

    def _as_joint_space(
        self, motor_cal: Mapping[str, float], *, keep_motors: bool = False
    ) -> Dict[str, float]:
        """실제로 나간 모터 목표를 관절 이름으로 되돌림.

        발목은 FK 를 거쳐야 하는데 비싸므로, 잘리지 않았으면 명령한 값을 그대로 씀.
        잘렸으면 FK 로 되짚어 **실제로 실행된 자세**를 냄 — 로그를 믿으려면
        무엇을 보냈는지가 아니라 무엇이 실행됐는지가 필요함.

        `keep_motors` 면 발목 **모터** 목표도 남김. 모터 공간으로 명령했을 때 켬 --
        그때는 명령한 축이 곧 그 모터인데, 목표가 없으면 텔레메트리가 실측을 목표로
        대신 써서(`snapshot.py` 의 `sent.get(motor, pos)`) `ankle_a/err` 이 늘 0이
        됨. 정작 봐야 할 추종 오차가 사라짐.

        관절 공간으로 명령했을 때는 안 켬. 그때 명령한 축은 pitch/roll 이고 모터
        열은 부수적이라, 켜면 예전 로그와 같은 열이 다른 뜻이 됨.
        """
        out = {j: v for j, v in motor_cal.items() if j in SINGLE_JOINTS}
        if keep_motors:
            out.update({m: v for m, v in motor_cal.items() if m in ANKLE_MOTORS})
        # 토크로 보냈으면 발목이 이미 관절 이름으로 들어 있음. 되짚을 모터각이 없음.
        out.update({j: v for j, v in motor_cal.items() if j in ANKLE_JOINTS})
        if not all(m in motor_cal for m in ANKLE_MOTORS):
            return out
        try:
            pitch, roll = self.kinematics.solve_fk(
                motor_cal["ankle_a"], motor_cal["ankle_b"],
                guess_pitch_deg=self._ankle_guess[0],
                guess_roll_deg=self._ankle_guess[1],
            )
        except AnkleUnreachableError:
            return out
        out["ankle_pitch"] = pitch
        out["ankle_roll"] = roll
        return out

    def send(self, commands: Mapping[int, MitCommand]) -> int:
        """계산된 명령을 보냄. **수거하지 않음.**

        보낸 대상을 기억해 둠 — 응답은 **명령을 받은 모터만** 보내므로, 기다릴
        개수가 명령한 개수임. 전체를 기다리면 명령하지 않은 모터가 무응답으로 잡힘.
        """
        self._awaiting = tuple(commands)
        return self.bus.send_mit(dict(commands))

    def refresh(self) -> Tuple[int, ...]:
        """명령하지 않고 상태만 읽음. 응답이 없었던 모터 id 를 반환함.

        게인과 토크가 0인 명령을 보내고 그 응답을 받음 — MIT 모드에는 읽기 전용
        명령이 없음. 토크가 0이라 아무 일도 일어나지 않음.

        관찰 모드와 시작 직전에 씀.
        """
        self._awaiting = tuple(self.config.motor_ids)
        missing = self.bus.refresh_states()
        self._note_link(missing)
        self._update_ankle_pose()
        return tuple(missing)

    def collect(self) -> Tuple[int, ...]:
        """응답을 수거함. **직전에 명령한 모터만** 기다림."""
        missing = self.bus.collect(expect=len(self._awaiting))
        self._note_link(missing)
        self._update_ankle_pose()
        return tuple(m for m in missing if m in self._awaiting)

    def _note_link(self, missing) -> None:
        """이번 주기에 누가 답했는지 기록함.

        MIT 모드는 명령을 받으면 반드시 상태 프레임으로 답함. **안 오면 그 모터가
        명령을 처리하지 않은 것임** -- 이것이 애플리케이션 레벨 ack 임.

        CAN 하드웨어 ACK 와 다름. 하드웨어 ACK 는 버스에 붙은 아무 노드나 찍어주므로
        "누군가 들었다" 일 뿐이고, 그건 `can/tx_errors` 가 봄. 둘을 같이 보면
        원인이 좁혀짐 -- tx_errors 가 0인데 응답이 없으면 모터가 명령을 무시한
        것이고, 프로토콜이나 제어 모드가 어긋난 것임 (이슈 #11).
        """
        # 명령하지 않은 모터는 판정 대상이 아님. 직전 상태를 그대로 둠.
        self._missing = {m for m in missing if m in self._awaiting}
        for motor_id in self._awaiting:
            if motor_id in self._missing:
                self._miss_streak[motor_id] += 1
            else:
                self._miss_streak[motor_id] = 0

    def link_status(self, now: Optional[float] = None) -> Dict[str, Dict[str, float]]:
        """모터별 링크 상태. 관절 이름 -> `{age, ack, miss}`.

            age    마지막 응답 이후 경과 (ms). 한 번도 못 받았으면 -1
            ack     1  직전 주기에 명령하고 응답을 받음
                    0  직전 주기에 명령했는데 응답이 없음  <- 씹힘
                   -1  직전 주기에 명령하지 않음
            miss   연속 무응답 주기 수

        **`ack` 가 세 값인 이유**: 명령하지 않은 모터를 `1` 로 내면 거짓말이 되고
        `0` 으로 내면 없는 고장이 보임.

        이 구분이 실제로 필요함. 응답이 한 번도 없던 모터는 현재 위치를 모르므로
        가드가 명령을 거부함(`reject_nostate`) — 그러면 명령이 안 나가고, 명령이
        안 나갔으니 응답 대상에서도 빠짐. **그 모터가 진단에서 통째로 사라짐.**

        그래서 진짜 신호는 `age` 임. `-1` 이면 한 번도 응답한 적이 없다는 뜻이고,
        이 값은 명령 여부와 무관하게 참임.

        `-1` 을 쓰는 이유: 무한대는 JSON 으로 못 보내고 CSV 에서도 읽기 어려움.
        음수는 정상 범위에 없으므로 "없음" 을 뜻함.
        """
        stamp = time.monotonic() if now is None else now
        out: Dict[str, Dict[str, float]] = {}
        for motor_name, motor in self.config.motors.items():
            state = self.bus.state(motor.id)
            age = state.age(stamp)
            if motor.id not in self._awaiting:
                ack = -1.0
            elif motor.id in self._missing:
                ack = 0.0
            else:
                ack = 1.0
            out[motor_name] = {
                "age": -1.0 if age == float("inf") else age * 1000.0,
                "ack": ack,
                "miss": float(self._miss_streak[motor.id]),
            }
        return out

    def since_clip(self, now: Optional[float] = None) -> float:
        """마지막 클리핑 이후 경과 (초). 없었으면 -1."""
        return self._since(self._last_clip_t, now)

    def since_reject(self, now: Optional[float] = None) -> float:
        """마지막 거부 이후 경과 (초). 없었으면 -1."""
        return self._since(self._last_reject_t, now)

    @staticmethod
    def _since(stamp: Optional[float], now: Optional[float]) -> float:
        if stamp is None:
            return -1.0
        return max(0.0, (time.monotonic() if now is None else now) - stamp)

    @property
    def last_sent(self) -> Dict[str, float]:
        return dict(self._last_sent)

    def hold(self) -> Dict[int, MitCommand]:
        """지금 자세를 그대로 유지하는 명령. 정지 직전이나 대기에 씀.

        목표를 현재 위치로 두므로 오차가 0이 되어 자세를 붙잡기만 함.
        """
        commands: Dict[int, MitCommand] = {}
        for motor_name in self.motor_names:
            motor = self.config.motors[motor_name]
            state = self.bus.state(motor.id)
            if not state.is_valid:
                continue
            commands[motor.id] = MitCommand(
                position_deg=state.position_deg,
                kp=motor.gains.kp,
                kd=motor.gains.kd,
            )
        return commands
