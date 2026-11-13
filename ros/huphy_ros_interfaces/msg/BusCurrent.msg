# 전류계(WCM1600)가 잰 로봇 전체 DC 전류. current_node 가 rate_hz 로 냄 (기본 20Hz).
#
# 장비는 UART 로 8바이트 프레임을 계속 보냄. 노드는 그것을 스레드로 받아 두고
# 타이머로 가장 최근 값만 냄 -- 프레임마다 내면 발행 주기가 장비 출력 주기에
# 끌려가고, 받는 쪽(log_node)은 어차피 줄마다 최근 값 하나만 씀.
#
# 모터에서 계산하는 전류(MotorExtra -> power/dc_a_lo)와 **다른 값임.** 계산값은
# 구리 손실만 보므로 작고, 이 값은 모터 말고 연산보드·팬·통신까지 다 포함하므로 큼.
# 둘을 바로 빼서 손실로 읽으면 안 됨 (huphy/telemetry/power.py 4절).

std_msgs/Header header

float64 amps              # 전체 DC 전류 (A). 부호가 방향. 못 받았으면 NaN
float64 age_ms            # 이 값이 장비에서 온 뒤 흐른 시간 (ms). 못 받았으면 -1
bool is_ac                # AC 프레임이었는지. 참이면 장비 설정이 틀린 것임
