# 추론 한 번. telemetry/policy_log.py 기록의 한 줄과 같은 내용임.
#
# header 는 ROS 쪽 시각 (ros2 bag, rqt_plot 이 봄).
# t 는 기록의 첫 칸이고 time.monotonic() 기준 상대 시각임 -- 추론한 기계에서
# 찍어야 그 기계의 주기 지터가 남음. 받는 쪽이 도착 시각으로 찍으면 DDS 지연이 섞임.

std_msgs/Header header

float64   t               # 초. 첫 추론부터
float64[] obs             # obs_dim 개. 모델에 넣은 벡터 그대로 (정규화 전)
float64[] action          # 관절 수. 모델이 낸 날 것의 값 (배율·기본자세 전)
float64[] target_deg      # 관절 수. 관절 목표각 (도)
float64[] measured_deg    # 관절 수. 그 주기에 쓴 실측 관절각 (도)
