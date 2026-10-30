# policy_node 가 시작할 때 한 번 냄. QoS 는 transient_local --
# 나중에 뜬 log_node 도 마지막 값을 받음.
#
# 이 값이 토픽으로 건너가므로 log_node 는 정책 꾸러미 폴더를 읽지 않음.
# 두 곳에서 열 이름을 만들면 어긋날 수 있어서임.

string    policy        # 꾸러미 이름. 기록 파일 이름에 쓰임
string    checkpoint    # 쓴 가중치. 기록의 run.checkpoint
float64   hz            # 추론 주기
float64   approach_s    # 기준 자세로 가는 시간

# 기록의 열 이름. control/deploy.py 의 _term_names 가 만든 것 그대로.
string[]  obs_names     # obs_dim 개.  "obs/gyro/x" 꼴
string[]  joint_names   # 관절 수.    policy.order (huphy 이름, "right_leg/knee")
