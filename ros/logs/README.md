# ros/logs

ROS 로 실물을 돌린 기록. `log_node`(`log.launch.py`)가 남김.

    policy/    정책 기록. 관찰·행동·목표각 (`1102_policy_<정책>.json`)
    latency/   모터 응답 지연 기록 (`1102_latency_<정책>.json`)

같은 이름의 policy·latency 파일은 같은 실행임.


## 1104 (2026-11-04)

정책 50Hz · 모터 100Hz. 계약대로 돈 실행임 (기록의 행 간격 20ms 로 확인).

| 파일 | 정책 | 가중치 | 길이 | 무엇 |
|---|---|---|---|---|
| `squat_baseline` | huphy_squat | 5000 | 91초 | **기준 움직임.** 다른 squat 기록은 이것과 비교함 |
| `squat_with_cam` | huphy_squat | 5000 | 94초 | 카메라를 달고 같은 동작. 카메라 무게 영향 확인용 |
| `walk_baseline` | generalized_walk | 49999 | 172초 | **기준 움직임.** 다른 walk 기록은 이것과 비교함 |


## 1102 (2026-11-02)

**전부 정책이 100Hz 로 돌았던 실행임.** 계약(`deploy_contract.json`)의 정책 주기는
50Hz 인데, `local.launch.py` 에서 모터 쪽 `hz`(100) 인자가 정책 노드에 새어 들어감.
정책 성능을 볼 때 이 기록을 50Hz 기준으로 보지 말 것.

| 파일 | 정책 | 가중치 |
|---|---|---|
| `1102_*_huphy_squat` | huphy_squat | 5000 |
| `1102_*_huphy_squat_2` | huphy_squat | 5000 |
| `1102_*_generalized_walk` | generalized_walk | 49999 |
| `1102_*_stairs_slopes12` | stairs_slopes12 | 99998 |
| `1102_*_stairs_slopes12_2` | stairs_slopes12 | 99998 |
