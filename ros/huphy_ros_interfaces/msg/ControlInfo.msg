# control_node 가 시작할 때 한 번 냄. QoS 는 transient_local.
#
# 가동범위(limit_*)는 캘리브레이션 파일에서 오고 그 파일은 모션보드에 있음.
# 다른 기계의 log_node 가 그 값을 얻는 유일한 경로임.

string    robot           # 로봇 이름 ("biped", "right_leg")
float64   hz              # 제어 주기
float64   drain_ms        # 응답 수거 대기. 기록의 meta.drain_ms
string    send_mode       # "seq" 또는 "burst"
int32     extra           # 한 주기에 다리마다 보낼 파라미터 읽기 요청 수.
                          # 0 이면 전류·전압을 안 읽고 MotorExtra 도 안 냄.
                          # log_node 가 이 값으로 전기값 열을 미리 만듦

# LatencyBatch.joint 의 색인이 이 배열의 자리임. bus.motor_ids 순서.
string[]  joint_names     # 관절 수. huphy 이름
string[]  motor_models    # 관절 수. 벤더 모델명 ("RS04"). 전력 열의 상저항을 가름.
                          # 모델을 못 찾으면 빈 문자열 -- 그 모터 전력이 NaN 이 됨
float64[] limit_lo_deg    # 관절 수. 가동범위 하한 (도)
float64[] limit_hi_deg    # 관절 수. 가동범위 상한 (도)
