# 송신·수신 사건 묶음. telemetry/latency_log.py 가 모으는 것과 같은 내용임.
#
# 주기로 묶지 않음 -- 버스가 주기 경계를 알려주지 않고, 모터마다 sent 와
# 응답마다 received 를 부를 뿐임 (motors/robstride/bus.py).
#
# SENT 를 같이 보내는 이유: 유실은 사건이 아니라 **보낸 수 - 받은 수**임.
# 받은 것만 보내면 복원할 수 없음.
#
# 네 배열의 길이가 같음. 같은 자리가 한 사건임.

uint8 SENT=0      # 보냄. latency_ms 는 NaN
uint8 ONTIME=1    # 그 주기 수거에서 받음
uint8 LATE=2      # 수거 대기를 넘겨 다음 주기에 들어옴. 지연은 원래 명령 기준

std_msgs/Header header

float64[] t             # 초. control_node 의 첫 송신부터
uint8[]   joint         # ControlInfo.joint_names 의 색인
float64[] latency_ms    # SENT 면 NaN
uint8[]   kind          # 위 상수
