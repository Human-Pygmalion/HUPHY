# control_node 가 시작할 때 한 번 냄. QoS 는 transient_local.
#
# 가동범위(limit_*)는 캘리브레이션 파일에서 오고 그 파일은 모션보드에 있음.
# 다른 기계의 log_node 가 그 값을 얻는 유일한 경로임.

string    robot           # 로봇 이름 ("biped", "right_leg")
float64   hz              # 제어 주기
float64   drain_ms        # 응답 수거 대기. 기록의 meta.drain_ms
string    send_mode       # "seq" 또는 "burst"

# LatencyBatch.joint 의 색인이 이 배열의 자리임. bus.motor_ids 순서.
string[]  joint_names     # 관절 수. huphy 이름
float64[] limit_lo_deg    # 관절 수. 가동범위 하한 (도)
float64[] limit_hi_deg    # 관절 수. 가동범위 상한 (도)
