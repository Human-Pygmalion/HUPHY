# `docs/`

설계 판단과 기록. 사용법은 [루트 README](../README.md).

```
architecture.md    계층과 의존 방향. 왜 그렇게 나눴나
flow_diagrams.md   누가 누구를 부르나. 그림
cycle.md           한 주기. 값이 어떤 모양으로 어디를 지나나
full_robot.md      다리 하나와 양다리. 무엇이 남았나
policy_runner.md   최종 정책 실행기 설계. 설정 파일 하나로 도는 것
build_log.md       무엇을 어떤 순서로 만들었나. 각 단계의 중점
issues.md          구현하면서 마주친 판단·미확인 사항
motor_setup.md     출하 상태 모터 세팅. 보낼 프레임까지
monitoring.md      무엇을 왜 보는가
```

---

## 어디부터 읽나

| 알고 싶은 것 | 볼 곳 |
|---|---|
| 로봇을 어떻게 움직이나 | [루트 README](../README.md) |
| 무슨 값을 고쳐야 하나 | [루트 README](../README.md) · [`config/README.md`](../config/README.md) |
| 계층이 왜 이렇게 나뉘었나 | [architecture.md](architecture.md) |
| 이 함수가 어디서 불리나 | [flow_diagrams.md](flow_diagrams.md) |
| 값이 어떤 모양으로 흘러가나 | [cycle.md](cycle.md) |
| 양다리는 어떻게 묶이나 | [full_robot.md](full_robot.md) |
| 학습 결과를 설정 파일로 돌리려면 | [policy_runner.md](policy_runner.md) |
| 왜 이 순서로 만들었나 | [build_log.md](build_log.md) |
| 이 값이 왜 이렇게 정해졌나 | [issues.md](issues.md) |
| 그래프에서 무엇을 보나 | [monitoring.md](monitoring.md) |
| 갓 뜯은 모터를 어떻게 세팅하나 | [motor_setup.md](motor_setup.md) |

---

## 코드 옆 문서

폴더별 상세는 소스 옆에 있음. 각 문서는 **자기 계층과 이웃 계층만** 다룸.

| 위치 | 내용 |
|---|---|
| [`config/README.md`](../config/README.md) | 설정 값. 두 파일을 나눈 이유 |
| [`src/huphy/config/`](../src/huphy/config/README.md) | 설정 읽기 |
| [`src/huphy/calibration/`](../src/huphy/calibration/README.md) | 실측값 읽기·쓰기 |
| [`src/huphy/safety/`](../src/huphy/safety/README.md) | 명령의 최종 관문 |
| [`src/huphy/motors/`](../src/huphy/motors/README.md) | 벤더 중립 자료형, CAN 전송, 하드웨어 전제 |
| [`src/huphy/motors/robstride/`](../src/huphy/motors/robstride/README.md) | 벤더 사양, 코덱, 버스, 커미셔닝 |
| [`src/huphy/kinematics/`](../src/huphy/kinematics/README.md) | 발목 링키지 |
| [`src/huphy/robots/`](../src/huphy/robots/README.md) | 관절 ↔ 모터 경계 |
| [`src/huphy/telemetry/`](../src/huphy/telemetry/README.md) | 관찰 |
| [`src/huphy/control/`](../src/huphy/control/README.md) | 제어 루프, 게인 튜닝 |
| [`src/huphy/scripts/`](../src/huphy/scripts/README.md) | 터미널 진입점 |
| [`tests/README.md`](../tests/README.md) | 테스트가 고정하는 것 |

---

## 지금 남은 것

전부 **실물이 있어야** 채워짐.

| | 무엇이 막히나 |
|---|---|
| [#8](issues.md) 모터 매핑 미확인 | 명령한 관절이 아닌 것이 움직일 수 있음 |
| [#9](issues.md) 게인 미튜닝 | 시작값 `kp = 20` 에서 튜닝해야 함 |
| [#13](issues.md) 발목 기하 출처 | 어느 다리 것인지 모름 |
| [#11](issues.md) 전제를 코드로 못 읽음 | 외부 도구로 대체 중 |
