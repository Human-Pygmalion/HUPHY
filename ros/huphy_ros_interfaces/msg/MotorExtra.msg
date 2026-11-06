# 모터 전기값·온도. control_node 가 state_hz 로 냄 (기본 100Hz).
# log_node 가 구독할 때만 냄 -- 구독자가 없으면 만들지도 않음.
#
# 배열 자리는 ControlInfo.joint_names 와 같음. 이름을 매 주기 보내지 않으려고
# 순서로만 맞춤.
#
# iq 와 vbus 는 상태 응답 프레임에 없어서 파라미터 읽기 요청으로 받음 --
# control_node 의 extra 파라미터가 0 이면 요청을 안 보내고 이 토픽도 안 냄.
# 한 주기에 다리마다 extra 개씩 읽으므로 값 하나의 갱신 주기는 state_hz 보다
# 느림 (모터 6개 x 값 2개 = 12칸, extra=2 면 6주기에 한 바퀴).
#
# temp 는 상태 응답 프레임에 있어서 매 주기 갱신됨.

std_msgs/Header header

float64[] iq              # 관절 수. 토크를 만드는 전류 (A). 못 받았으면 NaN
float64[] vbus            # 관절 수. 모터가 보는 버스 전압 (V). 못 받았으면 NaN
float64[] temp            # 관절 수. 모터 온도 (C)
