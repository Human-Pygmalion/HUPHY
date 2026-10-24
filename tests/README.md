# `tests/`

```
tests/
├── test_safety.py          safety/                     42개
├── test_codec.py           robstride 사양·코덱          31개
├── test_base.py            motors/base.py              44개
├── test_canbus.py          motors/canbus.py            33개
├── test_robstride_bus.py   robstride/bus.py            40개
├── test_commissioning.py   robstride/commissioning.py  46개
├── test_config.py          config/                     60개
├── test_calibration.py     calibration/                38개
├── test_commission_cli.py  scripts/commission.py       71개
├── test_ankle.py           kinematics/ankle.py        165개
├── test_leg.py             robots/leg.py               68개
├── test_sensors.py         sensors/                    31개
├── test_ebimu.py           sensors/ebimu/              42개
├── test_telemetry.py       telemetry/                  69개
├── test_control.py         control/                    45개
├── test_policy.py          control/policy.py           27개
├── test_rsl_rl.py          control/rsl_rl.py           24개
├── test_deploy.py          control/deploy.py           34개
├── test_teleop.py          control/teleop.py           37개
├── test_run.py             scripts/run.py              17개
├── test_bringup.py         scripts/bringup.py          36개
└── test_selftest.py        scripts/selftest.py         27개
```

## 실행

```bash
PYTHONPATH=src python3 -m pytest tests -q
```

```
........................................................................ [ 88%]
........................................................................ [100%]
956 passed in 18.2s
```

**하드웨어도 `python-can` 도 필요 없음.**

순수 계산 계층은 애초에 `python-can` 을 쓰지 않고, 전송 계층은 `import can` 이
함수 안에 있어 가짜 모듈로 갈아끼울 수 있음.

설정과 캘리브레이션 테스트는 **저장소의 실제 파일을 읽되 쓰지 않음.** 쓰기가 필요한
것은 임시 폴더에 사본을 만들어 씀.

일부만 실행:

```bash
PYTHONPATH=src python3 -m pytest tests/test_canbus.py -q
PYTHONPATH=src python3 -m pytest tests -q -k TestApply
PYTHONPATH=src python3 -m pytest tests -v          # 이름까지 출력
```

---

## 실물이 필요한 값을 쓰는 곳

한계값은 실제 오른다리 값을 씀. **비대칭이라** 좌우 구분이 필요한 검사에 적합함.

```
무릎 m10   -20.65 ~ 74.79
```

버스 테스트는 0.5 다리 구성 그대로 씀 — RS02 4개(7~10)와 RS00 2개(11, 12)가 한
채널에 물려 있고 **토크 범위가 다름**(17 vs 14 N·m).

EBIMU 테스트는 실제 패킷 한 줄을 씀. 15필드이고 자세가 약 `roll 30도, pitch 40도`
임 — 두 축이 다 기울어져 있어야 부착 검사가 판정할 수 있음.

---

## 구성

### `test_safety.py` — 42개

| 클래스 | 개수 | 대상 |
|---|---|---|
| `TestSafeWindow` | 3 | `limits.safe_window` |
| `TestClamp` | 5 | `limits.clamp` |
| `TestMarginToLimit` | 5 | `limits.margin_to_limit` |
| `TestClosestToLimit` | 3 | `limits.closest_to_limit` |
| `TestIsFinite` | 4 | `guards.is_finite` |
| `TestNanIsDangerous` | 3 | 유한값 검사의 근거 |
| `TestClampJump` | 5 | `guards.clamp_jump` |
| `TestApply` | 10 | `guards.apply` — 세 관문 전체 |
| `TestGuardCounters` | 4 | `guards.GuardCounters` |

### `test_codec.py` — 31개

| 클래스 | 개수 | 대상 |
|---|---|---|
| `TestEncodingTables` | 6 | `tables.encoding_for`, 프로토콜 축 |
| `TestCommandBytes` | 3 | 겹치는 명령 바이트 |
| `TestQuantization` | 5 | `float_to_uint` / `uint_to_float` |
| `TestPackCommand` | 9 | 명령 8바이트 배치 |
| `TestDecodeState` | 4 | 상태 프레임 해석 |
| `TestDecodeFault` | 4 | 고장 워드 해석 |

### `test_base.py` — 44개

| 클래스 | 개수 | 대상 |
|---|---|---|
| `TestGains` | 4 | `Gains`, `scaled` |
| `TestMotor` | 9 | 한계 순서 검증, `is_configured` |
| `TestMotorCalibration` | 14 | raw ↔ cal 변환, ±180 경계 접기 |
| `TestMotorState` | 5 | 신선도 판정 |
| `TestMotorFault` | 3 | 비트 필드 |
| `TestResolveMotorList` | 5 | 대상 모터 확정 |
| `TestMotorsBusContract` | 4 | ABC 강제, 컨텍스트 매니저 |

### `test_canbus.py` — 33개

| 클래스 | 개수 | 대상 |
|---|---|---|
| `TestCanFrame` | 3 | 표준 프레임 기본값, frozen |
| `TestInterface` | 2 | socketcan 기본값 |
| `TestLifecycle` | 8 | 중복 연결·해제, 미연결 조작 거부 |
| `TestSend` | 6 | 순서 유지, 실패 후 계속 |
| `TestDrain` | 8 | 조기 반환, 응답 누락, 상한, `flush_rx` |
| `TestDrainAll` | 3 | 채널별 묶음, 전송이 수거보다 먼저 |
| `TestCounters` | 3 | 0이어도 전 키 출력 |

### `test_robstride_bus.py` — 40개

| 클래스 | 개수 | 대상 |
|---|---|---|
| `TestConstruction` | 5 | 모델별 인코딩, 모델 오타 거부 |
| `TestCommandFrames` | 2 | 프레임 배치, `0xFB` 의 두 뜻 |
| `TestTorque` | 6 | 활성·정지·고장 클리어 |
| `TestLifecycle` | 4 | 정지 후 종료 |
| `TestSendMit` | 7 | 순서, 위치 왕복, 모델별 토크 스케일 |
| `TestCollect` | 7 | 캐시 갱신, 무응답 보고, 깨진 프레임 |
| `TestRefreshStates` | 4 | `PASSIVE` 전송, 게인 0, 큐 비우기 |
| `TestFault` | 5 | 비트 해석, 무응답과 정상의 구분 |

### `test_commissioning.py` — 46개

| 클래스 | 개수 | 대상 |
|---|---|---|
| `TestSetControlMode` | 3 | `set_control_mode` |
| `TestSetZero` | 6 | 메모 필수, 토크 켜진 상태 거부 |
| `TestSetCanId` | 6 | 범위·중복·동일 id 거부 |
| `TestSetProtocol` | 4 | 전원 재투입 경고 |
| `TestNudge` | 10 | 시작 복귀, 토크 정리, 진폭 상한 |
| `TestScan` | 4 | 응답자 수집 |
| `TestSweep` | 9 | 최대·최소 기록, 토크 차단, 오프셋 반영, ±180 경계 |
| `TestMeasureOffset` | 4 | 지금 자세가 0도, 토크 차단, 무응답 거부 |

### `test_config.py` — 58개

| 클래스 | 개수 | 대상 |
|---|---|---|
| `TestRealFile` | 10 | 실제 `config/robot.yaml`, 한계가 어느 파일에 있나 |
| `TestUnknownKeys` | 5 | 오타 거부 |
| `TestStructure` | 8 | 필수 항목 누락 |
| `TestValues` | 5 | `limits_deg` 거부, 0 이하 값 |
| `TestIdCollision` | 4 | 채널 안·채널 넘어 |
| `TestDefaults` | 6 | 빠진 절이 기본값을 쓰는지 |
| `TestSchema` | 8 | frozen, 조회 도우미 |

### `test_calibration.py` — 38개

| 클래스 | 개수 | 대상 |
|---|---|---|
| `TestRealFiles` | 6 | 실제 두 파일, `robot.yaml` 과 대조 |
| `TestLoadRejects` | 14 | 형식 번호, 한계 순서·개수, 게인 키 잔존, `sign=0` |
| `TestSave` | 7 | 왕복, 크래시 후 원본 생존 |
| `TestAttach` | 6 | 모터 id 재키잉, 관절 이름 불일치 |
| `TestUnmeasured` | 5 | 메모 기준 판정 |

### `test_commission_cli.py` — 71개

| 클래스 | 개수 | 대상 |
|---|---|---|
| `TestScan` | 4 | 무응답 표시, 원인 후보 |
| `TestState` | 3 | raw/cal 병기 |
| `TestFault` | 3 | 비트 이름, 무응답과 정상의 구분 |
| `TestNudge` | 5 | 왕복, 안 움직임 경고, 진폭 상한 |
| `TestDangerous` | 6 | `--yes` 요구, 승인 전 무전송 |
| `TestZero` | 4 | 메모 저장, 토크 차단 순서 |
| `TestTargeting` | 7 | `--limb` 요구, 관절 생략, 설정 오류가 버스보다 먼저 |
| `TestOptions` | 10 | 기본값 출처, 쉼표 한 줄, 필수 값 거부 |
| `TestSweepSteps` | 5 | 발목 묶기, 순서, 빠짐 없음 |
| `TestSweepNeedsAScreen` | 1 | 파이프에서 거부 |

### `test_ankle.py` — 165개

| 클래스 | 개수 | 대상 |
|---|---|---|
| `TestGeometry` | 6 | 로드 길이, frozen, 거울상 부호 |
| `TestSolveIk` | 13 | 중립 0, pitch/roll 방향, 규약 |
| `TestSolveFk` | 111 | 왕복, 다중해, 범위 격자 |
| `TestReachability` | 7 | 시험 범위, 로드 해 없음 |
| `TestMirror` | 10 | 반대칭 |
| `TestWrap` | 2 | `[-180, 180)` 접기 |
| `TestJacobian` | 5 | 수치 미분과 대조, 단위 무관 |
| `TestJointTorqueToMotor` | 5 | 가상일 보존, 선형성, 조건수 |
| `TestMitTorque` | 6 | 게인 단위, 감쇠, 실측 자세에서 선형화 |

`TestSolveFk` 가 큰 것은 시험 범위 격자 187개를 도는 매개변수 테스트 때문임 —
**추정 `(0, 0)` 으로도 항상 원래 자세를 찾는지**를 범위 전체에서 확인함.

### `test_leg.py` — 68개

| 클래스 | 개수 | 대상 |
|---|---|---|
| `TestConstruction` | 6 | 관절·모터 이름, 필드 목록 |
| `TestCoordinates` | 4 | cal ↔ raw, offset, sign |
| `TestAnkle` | 8 | IK 연동, 통째 거부, 실행된 자세 |
| `TestSafety` | 9 | cal 공간 한계, NaN, 점프, 카운터 |
| `TestPipeline` | 7 | 계산·전송·수거 분리 |
| `TestCalibration` | 5 | 미실측 거부 |
| `TestLifecycle` | 2 | 예외 중 토크 차단 |
| `TestMirroredLeg` | 1 | 좌우 대칭 |
| `TestLinkStatus` | 7 | ack, age, miss, since_* |
| `TestAnkleOutputMode` | 13 | 위치/토크 전환, IK 건너뜀, 시킨 토크 기록 |
| `TestAnkleVelocity` | 2 | 모터 속도 -> 관절 속도 |

### `test_sensors.py` — 31개

| 클래스 | 개수 | 대상 |
|---|---|---|
| `TestImuState` | 6 | 모르는 것과 수평의 구분, age, extra 가 상태마다 따로 |
| `TestGravity` | 6 | 오일러 경로와 쿼터니언 경로가 같은 값을 냄 |
| `TestRegistry` | 4 | model 풀기, 만들 때 포트 안 엶 |
| `TestXsensLifecycle` | 5 | 두 번 열고 닫기, 보드레이트 전달 |
| `TestXsensRead` | 9 | 단위 변환, 오일러가 extra 로, 빠진 필드, 멈춘 센서 |

리더를 가짜로 갈아 끼움 — `pyserial` 없이 돌아야 함.

### `test_ebimu.py` — 42개

| 클래스 | 개수 | 대상 |
|---|---|---|
| `TestOutput` | 7 | 자세는 못 끔, 배타 쌍, 패킷 순서, 중복 |
| `TestCommands` | 5 | 안 쓰는 항목을 끔, 모드 값, 1ms 배수, 위험 명령 |
| `TestParse` | 12 | 자르는 자리, 개수 불일치는 통째로 버림, 쿼터니언 순서, g -> m/s² |
| `TestSettings` | 7 | `<cfg>` 읽기, 다른 것만 보냄, 응답에서 데이터 줄 제거 |
| `TestMountCheck` | 4 | 축 뒤집힘 검출, 수평이면 판정 거부, 가속도계 부호 규약 |
| `TestEbimuImu` | 7 | 필드 수 불일치면 연결 중단, 오일러 출력 거부 |

**개수가 안 맞는 줄은 통째로 버림.** 앞에서부터 채우면 항목 하나가 빠졌을 때 뒤가
한 칸씩 당겨져 각속도 자리에 가속도가 들어가는데, 값이 그럴듯해서 실물에서 안 잡힘.

### `test_telemetry.py` — 69개

| 클래스 | 개수 | 대상 |
|---|---|---|
| `TestFieldNames` | 8 | 실행 전 목록, 빠른 것/진단 분리 |
| `TestBuild` | 8 | 오차, 실제로 나간 목표, 없는 카운터 |
| `TestUdp` | 10 | 진짜 소켓 왕복, 반올림, MTU, 실패 |
| `TestCsv` | 9 | 헤더, 열 순서, flush, 실패 |
| `TestTelemetry` | 8 | 둘이 같은 스냅샷, 진단 감축 |
| `TestCommandedTorque` | 5 | 시킨 토크와 보고된 토크 |
| `TestAnkleJointFields` | 8 | 관절 축, 오차, 각속도, MTU |
| `TestImuFields` | 7 | IMU 개체 이름 접두, 없을 때 열 |
| `TestImuPacket` | 2 | 다리와 별도 패킷 |

UDP 는 **진짜 소켓**으로 자기 자신에게 보내 받아 봄 — 직렬화와 반올림이 실제로
왕복하는지 확인함.

### `test_control.py` — 45개

| 클래스 | 개수 | 대상 |
|---|---|---|
| `TestMotions` | 13 | 순수 함수. 하드웨어 없이 그냥 부름 |
| `TestMode` | 6 | 관찰이 토크를 끊고 시작, 관찰도 읽음 |
| `TestShutdown` | 6 | hold 후 토크 차단, 예외 중에도 |
| `TestTiming` | 10 | 주기 측정, 꾸준한 느림, 정밀 대기 |
| `TestTelemetryHookup` | 5 | 기록 실패가 루프를 안 멈춤 |
| `TestStep` | 2 | 한 걸음씩 |

### `test_policy.py` — 30개

| 클래스 | 개수 | 대상 |
|---|---|---|
| `TestObservationVector` | 7 | 24/26 길이, 순서, 도->rad |
| `TestProjectedGravity` | 5 | 수평은 (0,0,-1), 단위벡터 |
| `TestHopPhase` | 4 | 한 바퀴에서 안 튐 |
| `TestJointTargets` | 5 | action_scale, 관절 순서 |
| `TestPolicyMotion` | 5 | Motion 모양, 직전 행동 |
| `TestSpecs` | 4 | 가중치 파일과 맞는 값 |

모델 자리에 함수를 넣어 돌림 — torch 없이 실행됨.

### `test_rsl_rl.py` — 24개

| 클래스 | 개수 | 대상 |
|---|---|---|
| `TestLoad` | 5 | 규격 대조, 잘못된 파일 |
| `TestForward` | 6 | 계산, 정규화, 같은 입력이면 같은 출력 |
| `TestElu` | 4 | 학습 설정의 활성함수 |
| `TestWithPolicy` | 1 | 읽은 것을 Motion 에 그대로 |

`config/policies/` 의 **진짜 가중치 파일**을 읽음. torch 없이 돌아감.

### `test_run.py` — 17개

| 클래스 | 개수 | 대상 |
|---|---|---|
| `TestRefusals` | 4 | 모르는 정책, 없는 파일, 규격 불일치, IMU 없음 |
| `TestRun` | 6 | 실행, 발목이 토크로, 학습 게인 |
| `TestStaged` | 5 | **바로 정책으로 뛰지 않음** |
| `TestEnterWatcher` | 2 | 화면이면 기다리고 아니면 바로 |

가짜 모터·IMU 에 명령줄을 그대로 돌리고, `config/policies/` 의 진짜 가중치를 읽음.

### `test_bringup.py` — 36개

| 클래스 | 개수 | 대상 |
|---|---|---|
| `TestBuildLeg` | 3 | 게인 축소, 왼다리 거울상 기구학 |
| `TestGoesThroughTheLoop` | 3 | **움직이는 항목이 루프를 탐** (이슈 #4) |
| `TestShowState` | 7 | raw/cal, 링크 상태, 한계 밖 경고 |
| `TestTorqueGate` | 3 | 미실측이면 토크 거부 |
| `TestMotionItems` | 5 | 계단·사인파·이동, 잘못된 입력 |
| `TestMenu` | 5 | 항목 표시, 종료 시 토크 차단 |
| `TestImuOnTheLeg` | 6 | 다리와 같이 열림, 상태 표에 나옴 |

메뉴를 실제로 돌림 — `builtins.input` 을 미리 넣은 값으로 갈아끼움. 루프가 실제로
도는 항목이 있어 **이 파일이 가장 오래 걸림**(약 7초).

### `test_selftest.py` — 27개

| 클래스 | 개수 | 대상 |
|---|---|---|
| `TestJointLimits` | 4 | 한계 출처 둘, 미실측 관절 제외 |
| `TestInset` | 3 | 한계 여유, 좁은 관절 |
| `TestApproach` | 6 | 지금 자리에서 출발, 도착 후 유지 |
| `TestCycle` | 5 | 가운데 시작, 양 끝 도달, 한계 안 |
| `TestThen` | 3 | 인계 시각, 뒤 구간의 0초 |
| `TestMidpoints` | 1 | 중앙값 |
| `TestParser` | 5 | 공통 옵션이 앞뒤 어디서나, 덮어쓰지 않음 |

동작이 **순수 함수**라 하드웨어도 루프도 없이 확인함 — 시간과 관찰을 넣고 관절
목표를 봄. 가장 빠른 파일임(0.2초).

---

## 하드웨어 없이 어떻게 확인하나

### 순수 계층 — 그냥 부름

`safety/`, `tables.py`, `codec/mit.py`, `base.py` 는 `python-can` 을 import하지
않음. 인자를 넣고 반환값을 보면 됨.

### 전송 계층 — 가짜 모듈로 갈아끼움

`canbus.py` 가 `import can` 을 **함수 안에서** 하므로 `sys.modules` 에 가짜를 넣을
수 있음.

```python
mod = types.ModuleType("can")
mod.Message = FakeMessage
mod.interface = types.SimpleNamespace(Bus=FakeCanBus)
monkeypatch.setitem(sys.modules, "can", mod)
```

### 가짜 버스는 명령에 **응답함**

```python
def send(self, msg):
    self.sent.append(msg)
    reply = self.responses.get(msg.arbitration_id)
    if reply is not None:
        self.rx.append(reply)
```

미리 큐에 넣어 두는 방식은 `refresh_states` 와 `read_fault` 에서 재현이 깨짐 —
둘 다 `flush_rx()` 를 먼저 부르므로 그 프레임이 지워짐. 실제 모터가 명령을 받은
뒤에 답하는 것과 같은 순서로 맞춤.

### 확인되지 않는 것

가짜 버스는 타이밍을 재현하지 않음. **순서·개수·집계·프레임 내용**까지가 한계임.

| | 확인 방법 |
|---|---|
| 전송 지연, CAN 중재 | 실물 |
| 모터가 실제로 응답하는지 | 실물 |
| 게인 값이 적절한지 | 실물 |
| `zero_sta` 가 켜져 있는지 | 실물 |
| 실제 제어 주기가 유지되는지 | 실물 (스케줄러 정밀도와 부하에 달림) |

---

## 고정한 것

### `TestNanIsDangerous` — 유한값 검사의 근거

파이썬 `min`/`max` 가 NaN을 통과시키는 것과, `limits.clamp` 도 NaN을 못 잡는 것을
**직접 실행해 확인함.**

```python
def test_python_minmax_passes_nan_through(self):
    assert min(10, NAN) == 10

def test_clamp_does_not_catch_nan(self):
    value, clipped = limits.clamp(NAN, KNEE, margin_deg=3)
    assert math.isnan(value)
    assert clipped is False        # 잘렸다고 보고하지도 않음

def test_guards_catches_it(self):
    r = guards.apply(NAN, 0.0, ...)
    assert r.reject is guards.RejectReason.NOT_FINITE
```

**"그래서 `guards` 가 따로 검사해야 한다"는 근거가 코드로 남음.** 나중에 이 검사를
지우려는 사람이 왜 있는지 알게 됨.

### `test_limit_applied_before_jump` — 클리핑 순서

목표 200, 현재 60, `max_delta` 50 에서 **순서에 따라 결과가 달라짐.**

```
한계 먼저 (현재 구현)
  clamp(200) -> 71.79
  clamp_jump(71.79, 60, 50) -> 차이 11.79 < 50 이므로 그대로
  결과 71.79, clips = (LIMIT,)          ← 점프는 걸리지 않음

점프 먼저 (잘못된 순서)
  clamp_jump(200, 60, 50) -> 110        ← 한계(74.79)를 넘은 값
  clamp(110) -> 71.79
  결과 71.79, clips = (JUMP, LIMIT)     ← 불필요한 점프 클리핑이 기록됨
```

값은 같지만 경로가 다름. 역순은 중간에 한계 밖 값을 만들고, 실제로는 걸릴 필요가
없는 점프 클리핑을 카운터에 남김. `clips` 튜플로 순서를 고정함.

### `test_output_may_be_outside_limits_while_recovering`

현재가 이미 한계 밖(200)이면 **한 번에 복귀하지 않고** `max_delta` 씩 돌아옴.

```python
r = guards.apply(0, 200, ...)
assert r.value == pytest.approx(150.0)
assert r.value > KNEE[1]          # 아직 한계 밖
```

의도된 동작임 — 한 번에 뛰면 위험함. 나중에 "왜 한계 밖 값이 나가지?"라며 고치려는
것을 막음.

### `test_reaches_far_target_over_cycles`

버리지 않고 자르므로 먼 목표에도 결국 도달함.

```python
cur = 0.0
for _ in range(3):
    cur, _ = guards.clamp_jump(100, cur, 50)
assert cur == pytest.approx(100.0)
```

버리는 방식이면 도달 불가. **클리핑 = 속도 제한**임을 고정함.

### `test_nan_passes_the_clamp` — 코덱은 막지 못함

```python
assert mit.float_to_uint(NAN, -12.57, 12.57, 16) == 65535
```

65535는 최대값, 즉 **720° 목표 명령**임. "코덱이 알아서 막겠지"라는 오해를 막음.

### `test_protocol_axis_is_real` — 프로토콜별 속도 범위

```python
assert mit_rs02.vmax_rad_s == 33.0    # 매뉴얼 p.37~38
assert pri_rs02.vmax_rad_s == 44.0    # 매뉴얼 p.20~21
```

이 축을 없애고 "모델별 사양" 하나로 뭉치면 private 값을 MIT에 가져다 쓰는 실수가 남.
틀리면 속도 읽기가 44/33 = 1.33배 어긋남.

### `test_hard_stop_moves_in_raw_but_not_in_cal` — 한계값을 cal 에 둔 근거

영점을 3° 다른 자세에서 다시 잡으면 raw 는 달라지고 cal 은 그대로임.

```python
before = MotorCalibration(motor_id=10, offset_deg=12.0)
after  = MotorCalibration(motor_id=10, offset_deg=15.0)

assert before.cal_to_raw(74.79) == pytest.approx(62.79)
assert after.cal_to_raw(74.79)  == pytest.approx(59.79)
```

하드스톱은 쇳덩어리라 움직이지 않는데 raw 숫자만 바뀜 (이슈 #2).

### `test_mirrored_legs_share_one_cal_number`

```python
right = MotorCalibration(motor_id=10, sign=1.0)
left  = MotorCalibration(motor_id=4,  sign=-1.0)

assert right.raw_to_cal(45.0)  == pytest.approx(45.0)
assert left.raw_to_cal(-45.0)  == pytest.approx(45.0)
```

보행 궤적이 "무릎 45°" 라고 하면 양다리가 같은 동작을 함. **cal 공간이 존재하는
이유임.**

### `test_identity_when_unmeasured` — 지금 두 공간이 같은 이유

`sign=1, offset=0` 이라 `cal == raw` 임. 어느 쪽으로 해석해도 동작이 같아서 이슈 #2
가 드러나지 않음. 실측값을 넣는 순간 갈라짐.

### `test_disconnects_even_on_exception`

제어 중 예외가 나도 토크가 끊겨야 함 (이슈 #6). 없으면 모터가 마지막 명령을 계속
유지함 — 사람이 전원을 뽑을 때까지 다리가 힘을 주고 있음.

### `test_send_all_precedes_drain_all` — 이슈 #10의 순서

모든 가짜 버스가 공유하는 이벤트 순서로 확인함.

```python
kinds = [kind for kind, _ in FakeCanBus.events]
assert kinds == ["send"] * 6              # 수거가 전송 사이에 끼지 않음
```

버스별 `recv` 호출 수로는 확인되지 않음 — `drain` 은 항상 최소 한 번 `recv` 를 부름.

### `test_passive_command_carries_no_effort`

```python
leg.refresh_states([10])
assert ((data[3] & 0x0F) << 8) | data[4] == 0        # Kp
assert (data[5] << 4) | (data[6] >> 4) == 0          # Kd
```

게인이 0이 아니면 **상태를 읽는 것만으로 다리가 움직임.**

### `test_model_decides_torque_scaling`

같은 토크 값이 모델에 따라 다른 바이트가 됨. RS00은 범위가 좁아(14 N·m) RS02(17
N·m)보다 큰 수가 나감.

### `test_uses_query_variant` — 고장 조회

```python
assert raw.sent[0].data[6] == T.F_CMD_FAULT_QUERY
```

`F_CMD` 가 `0xFF` 면 **클리어**임. 조회하려다 지우면 원인을 잃음.

### `test_nothing_is_sent_without_yes` — 승인이 버스보다 먼저

```python
with pytest.raises(SystemExit):
    run("--limb", "right_leg", "zero", "knee", "--note", "편 상태")
assert FakeBus.instances == []
```

되돌리기 어려운 조작은 **모터에 아무것도 보내지 않은 상태에서** 멈춰야 함.
설정 오류도 마찬가지임 — 오타가 있는 설정으로 부르면 채널을 열지 않음.

### `test_disables_torque_first` — 영점 잡기 순서

```python
stop = next(i for i, m in enumerate(sent) if m.data[7] == T.CMD_STOP)
zero = next(i for i, m in enumerate(sent) if m.data[7] == T.CMD_SET_ZERO)
assert stop < zero
```

영점을 잡으면 좌표계가 통째로 옮겨가는데 직전 목표각은 **옛 좌표계 값**임. 그대로
유지되면 그 차이만큼 관절이 튐.

### `test_original_survives_a_crash` — 실측값 보호

`os.replace` 를 실패시켜 원본이 남는지 확인함. 실측값을 잃으면 다시 재는 수밖에
없는데, 그건 로봇을 분해해야 하는 작업일 수 있음. 임시 파일도 남지 않음.

### `test_missing_joint_is_an_error` — 관절 이름 불일치

관절 하나가 조용히 항등변환으로 도는 것이 가장 나쁨. `sign` 이 반대인 관절이
그렇게 되면 목표에서 **멀어지는 방향**으로 토크가 걸림.

### `test_limb_typo` — 설정 오타

```
ConfigError: limbs.right_leg: 모르는 키 ['contorl_hz']
```

YAML 은 모르는 키를 조용히 넘김. 고쳤는데 아무것도 안 바뀌고, 증상이 "느리다" 로
나타나므로 원인을 설정에서 찾을 이유가 없어 오래 걸림.

### `test_fk_is_not_unique` — 링키지의 성질

```python
a1, a2 = ankle.solve_ik(90, 90, enforce_envelope=False)
other = ankle.solve_fk(a1, a2)                    # (2.41, -40.48)
assert ankle.solve_ik(*other, enforce_envelope=False) == (a1, a2)
```

**같은 모터각 조합이 서로 다른 자세 둘에 대응함.** 버그가 아니라 링키지의 성질임.
나중에 "FK 가 이상한 값을 낸다" 며 고치려는 것을 막음.

바로 다음 테스트가 **시험 범위 안에서는 문제가 안 된다**는 것을 격자 187개로
확인함.

### `test_antisymmetry` — 거울상

```
거울상.solve_ik(pitch, -roll) == -원본.solve_ik(pitch, roll)
```

이게 성립해야 보행 궤적 한 벌로 양다리를 움직일 수 있음. 좌표만 뒤집고 부호를
그대로 두면 근사로만 맞음.

### `test_limit_is_applied_in_cal_space` — 경계의 순서

```python
cal["knee"] = MotorCalibration(motor_id=-1, sign=-1.0)
commands = leg.build_commands({"knee": 200.0})

assert leg.last_sent["knee"] == pytest.approx(74.79 - 3.0)         # cal 로 잘림
assert commands[10].position_deg == pytest.approx(-(74.79 - 3.0))  # 그 뒤에 raw
```

한계가 cal 공간에 있으므로 **검사가 변환보다 먼저여야 함.** raw 로 내린 뒤에
검사하면 `sign` 이 -1 인 관절에서 부호가 뒤집혀 한계가 반대로 걸림 (이슈 #2).

### `test_unreachable_drops_both` — 발목은 통째로

IK 가 안 풀리면 두 모터 다 직전 명령을 유지함. 한쪽만 새 명령을 받으면 두 로드가
서로 다른 자세를 요구해 관절이 비틀림.

한계에 잘리는 것은 다름 — 잘린 각도 쌍도 대응하는 발 자세가 있으므로 개별로
처리해도 됨.

### `test_build_does_not_touch_can` — 이슈 #10

계산이 프레임을 하나도 안 보냄. 버스가 둘일 때 계산을 먼저 몰 수 있는 근거임.

### `test_matches_what_build_produces` — 스키마가 한 곳

```python
assert set(snapshot.build(robot, t=0.0)) == set(snapshot.field_names(robot))
```

두 군데에서 필드를 만들면 반드시 어긋남 — CSV 헤더에는 있는데 UDP 에는 없는 값이
생기고, 어느 쪽이 맞는지 알 수 없어짐.

### `test_each_packet_fits` / `test_merged_would_not_fit` — MTU

빠른 패킷 약 850 B, 진단 약 950 B 로 각각은 들어감. **합치면 넘쳐서 조각나고,
조각 하나만 잃어도 패킷 전체가 버려짐.** 나눈 근거를 양쪽으로 고정함.

### `test_silent_motor_is_counted` — 명령이 씹혔는지

MIT 모드는 명령을 받으면 반드시 답함. 안 오면 그 모터가 처리하지 않은 것임.

`tx_errors` 가 0인데 `ack` 가 0이면 **모터가 명령을 무시하는 것** — 배선이 아니라
프로토콜이나 제어 모드가 어긋난 것임 (이슈 #11).

### `test_collect_waits_only_for_commanded_motors`

응답은 명령을 받은 모터만 보냄. 전체를 기다리면 명령하지 않은 모터가 무응답으로
잡혀 **가짜 고장**이 보임.

### `test_send_failure_does_not_raise` — 관측이 제어를 멈추면 안 됨

소켓을 망가뜨리고 보내 봄. 예외 대신 세기만 함. CSV 도 같음 — 디스크가 가득 차도
제어 루프는 계속 돌아야 함.

### `test_close_flushes`

버퍼에 남은 몇 줄이 사라지면 하필 사고 직전 부분을 잃음.

### `test_observe_still_reads` — 관찰도 통신함

MIT 프로토콜에는 읽기 전용 명령이 없음. 아무것도 보내지 않으면 아무것도 오지 않음.
관찰 모드는 힘이 나가지 않는 명령을 보내고 그 응답을 받음.

### `test_exception_still_cuts_torque`

제어 중 예외가 나면 모터가 마지막 명령을 계속 유지함. 자세 유지가 실패해도 토크는
반드시 끊김.

### `test_kept_up_flags_a_sustained_shortfall`

`overruns` 는 튀는 주기를 세지만 **꾸준히 느린 것은 못 잡음** — 매 주기 24%씩
넘으면 한 번도 밀림으로 세지 않으면서 주파수만 떨어짐.

### `test_moving_items_use_the_loop` — 이슈 #4

`ControlLoop.run` 을 감시해 **움직이는 항목이 전부 루프를 지나는지** 봄.

메뉴가 로봇을 직접 부르면 그 경로에서만 텔레메트리·주기 측정·정지 순서가 빠짐.
그러면 그래프가 안 나오는데 텔레메트리가 고장난 줄 알게 됨.

### `test_warns_when_outside_limits`

관절이 한계 밖에 있으면 **토크를 넣는 순간 가드가 한계 안으로 끌어당김.** 그
방향으로 움직이므로 사람이 알고 있어야 함.

### `test_all_keys_always_present`

카운터가 0이어도 모든 키를 출력하는지. 필드가 나타났다 사라지면 PlotJuggler
레이아웃과 CSV 헤더가 깨짐. `safety` 와 `can` 양쪽에 있음.

---

## 미커버

| 대상 | 왜 없나 |
|---|---|
| `robots/humanoid.py` | 미구현. 양다리를 묶을 때 만듦 |
| `robstride/codec/private.py` | 미구현. 29-bit 확장 프레임이 필요해지면 |

계층을 만든 순서와 각 단계의 중점은 [docs/build_log.md](../docs/build_log.md) 참조.
