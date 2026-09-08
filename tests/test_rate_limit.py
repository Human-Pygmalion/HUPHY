"""속도 제한 — 직전 **명령** 기준 램프 (2026-09-07 벤치).

## 왜 새로 만들었나

`max_delta_deg` 를 낮춰 속도를 제한하고 있었는데, 그것은 속도 제한이 아니었다.
기준점이 **측정 위치**이므로 명령과 현재각의 차이가 그 값으로 묶이고, MIT 식이
``tau = kp*(명령각 - 현재각) + ...`` 이라 **토크가 함께 묶인다.** 벤치에서 3° / kp 30
이면 최대 1.57 N·m 였고, 게다가 스스로를 조인다: 모터가 뒤처지면 측정이 안 나아가고
→ 명령도 안 나아가고 → 오차가 안 쌓이고 → 토크가 안 커진다. 이론 상한 300°/s 인
설정에서 실측 49°/s 가 나온 이유다.

`max_delta_deg` 자체는 원래 그 용도가 아니었다. 스키마의 기본값 50° 와 그 설명
("계산이 튀었을 때 그 이상은 안 나가게 막는 상한")대로 **폭주 상한**이며, 그 역할로
남는다. 속도는 `max_vel_deg_s` 가 맡고, 기준점은 직전 명령이다.

## 이 파일이 지키는 성질

1. 명령은 모터가 뒤처져도 보장된 속도로 계속 나아간다 (안 그러면 예전 병으로 회귀)
2. 오차는 자유롭게 커진다 — 따라잡을 토크를 다 쓸 수 있어야 한다
3. 한도는 **속도**로 적고 루프가 자기 주기로 환산한다 — 주기를 바꿔도 속도는 그대로
4. 명령이 끊겼다 재개되면 설정점을 측정값으로 다시 잡는다 (사고 재발 방지)
5. 설정하지 않은 로봇은 예전과 완전히 동일하게 동작한다
"""

import math

import pytest

from huphy.config.schema import SafetyConfig
from huphy.safety import guards


# --------------------------------------------------------------- 1. 기준점이 직전 명령
def test_command_advances_even_while_the_motor_lags():
    """예전 방식과의 결정적 차이. 모터가 전혀 못 따라와도(측정이 고정) 명령은
    한 주기에 max_step 씩 계속 나아가야 한다."""
    cmd = 0.0
    measured = 0.0            # 모터가 완전히 멈춰 있다고 가정
    for _ in range(10):
        cmd, _ = guards.clamp_rate(100.0, cmd, max_step_deg=1.0)
    assert cmd == pytest.approx(10.0), "10 주기면 10도. 측정이 안 움직여도 명령은 나아간다"
    assert cmd - measured == pytest.approx(10.0), "오차가 자유롭게 커진다 = 토크를 다 쓴다"


def test_the_old_anchor_would_have_stalled():
    """대조군. 같은 상황에서 측정 위치를 기준으로 삼으면(예전 방식) 명령이
    영원히 measured+1 도에 머문다 - 오차가 1도를 못 넘으니 토크도 못 넘는다."""
    measured = 0.0
    for _ in range(10):
        cmd, _ = guards.clamp_jump(100.0, measured, max_delta_deg=1.0)
    assert cmd == pytest.approx(1.0), "측정이 안 움직이면 명령도 제자리"


def test_it_stops_at_the_target_and_does_not_overshoot():
    cmd = 0.0
    for _ in range(20):
        cmd, clipped = guards.clamp_rate(3.5, cmd, max_step_deg=1.0)
    assert cmd == pytest.approx(3.5)
    assert clipped is False, "도착한 뒤에는 자르지 않는다"


def test_both_directions():
    down, clipped = guards.clamp_rate(-100.0, 0.0, max_step_deg=2.0)
    assert down == pytest.approx(-2.0) and clipped is True


def test_no_previous_command_passes_through():
    """무장 직후에는 자를 기준이 없다. 호출부가 그때 측정값으로 다시 잡는다."""
    v, clipped = guards.clamp_rate(50.0, None, max_step_deg=1.0)
    assert v == pytest.approx(50.0) and clipped is False


# ------------------------------------------------------- 3. 한도는 속도로, 주기는 루프가
def test_the_limit_is_a_speed_not_a_step():
    """같은 속도 한도라면 제어 주기가 달라져도 초당 이동량이 같아야 한다. 지금 벤치의
    3° 상수는 100 Hz 와 곱해져 300°/s 가 '우연히' 정해진다 - 그것을 없애는 것이 목적."""
    cfg = SafetyConfig(max_vel_deg_s=60.0)
    for hz in (50.0, 100.0, 200.0):
        step = cfg.max_step_deg("knee", hz)
        cmd = 0.0
        for _ in range(int(hz)):          # 정확히 1초치
            cmd, _ = guards.clamp_rate(1e6, cmd, step)
        assert cmd == pytest.approx(60.0, rel=1e-9), f"{hz} Hz 에서 초당 {cmd}도"


def test_per_motor_overrides_the_shared_limit():
    cfg = SafetyConfig(max_vel_deg_s=60.0, max_vel_deg_s_by_motor={"knee": 20.0})
    assert cfg.vel_limit_deg_s("knee") == 20.0
    assert cfg.vel_limit_deg_s("hip_yaw") == 60.0


def test_unset_means_no_rate_limit_at_all():
    """설정하지 않은 로봇은 예전과 완전히 동일하게 동작해야 한다."""
    cfg = SafetyConfig()
    assert cfg.vel_limit_deg_s("knee") is None
    assert cfg.max_step_deg("knee", 100.0) is None
    r = guards.apply(100.0, 0.0, limits=None, command_margin_deg=0.0,
                     max_delta_deg=50.0, enforce_limits=False,
                     prev_cmd_deg=0.0, max_step_deg=None)
    assert r.value == pytest.approx(50.0), "폭주 상한만 걸린다 - 예전 동작 그대로"
    assert guards.ClipReason.RATE not in r.clips


# ---------------------------------------------------- 두 관문이 함께 있을 때의 순서
def test_speed_limit_runs_before_the_blowup_ceiling():
    """속도 제한은 직전 명령에서 램프를 만들고, 폭주 상한은 그 결과가 측정에서
    터무니없이 멀지 않은지만 본다. 순서가 바뀌면 램프가 폭주 상한이 만든 값 위에서
    시작해 두 관문이 서로를 먹는다."""
    r = guards.apply(1000.0, 0.0, limits=None, command_margin_deg=0.0,
                     max_delta_deg=50.0, enforce_limits=False,
                     prev_cmd_deg=0.0, max_step_deg=0.6)
    assert r.value == pytest.approx(0.6), "속도 제한이 이긴다 (더 빡빡하므로)"
    assert guards.ClipReason.RATE in r.clips
    assert guards.ClipReason.JUMP not in r.clips, "폭주 상한은 걸릴 일이 없다"


def test_the_blowup_ceiling_still_catches_a_runaway_setpoint():
    """속도 제한만으로는 설정점이 관절에서 얼마든지 멀어질 수 있다 - 그게 목적이다.
    폭주 상한은 그 위에 남아, 넉넉하지만 무한하지는 않게 붙잡는다."""
    r = guards.apply(1000.0, 0.0, limits=None, command_margin_deg=0.0,
                     max_delta_deg=50.0, enforce_limits=False,
                     prev_cmd_deg=900.0, max_step_deg=100.0)
    assert r.value == pytest.approx(50.0), "측정 0도에서 50도까지만"
    assert guards.ClipReason.JUMP in r.clips


def test_the_two_reasons_are_counted_separately():
    """어느 쪽이 자르고 있는지가 곧 '왜 느린가' 의 답이므로 한 칸에 합치면 안 된다."""
    c = guards.GuardCounters()
    c.record(guards.apply(1000.0, 0.0, limits=None, command_margin_deg=0.0,
                          max_delta_deg=50.0, enforce_limits=False,
                          prev_cmd_deg=0.0, max_step_deg=0.6))
    c.record(guards.apply(1000.0, 0.0, limits=None, command_margin_deg=0.0,
                          max_delta_deg=50.0, enforce_limits=False,
                          prev_cmd_deg=900.0, max_step_deg=100.0))
    f = c.as_fields()
    assert f["clips_rate"] == 1 and f["clips_jump"] == 1


# ------------------------------------------------ 데이터시트 값이 실제로 그 속도를 낸다
@pytest.mark.parametrize("rpm,deg_s", [(200, 1200.0), (180, 1080.0), (167, 1002.0)])
def test_datasheet_rpm_converts_to_the_documented_degrees_per_second(rpm, deg_s):
    """RS03/RS04 매뉴얼(출력단): 무부하 200 rpm, 정격 RS03 180 / RS04 167 rpm.
    rpm x 6 = °/s, rpm x 2pi/60 = rad/s. 단위를 틀리면 60배가 조용히 어긋난다."""
    assert rpm * 6.0 == pytest.approx(deg_s)
    assert rpm * 2.0 * math.pi / 60.0 == pytest.approx(math.radians(deg_s))


def test_a_datasheet_derived_limit_reaches_that_speed_in_one_second():
    """RS04 정격의 6% 를 벤치 한도로 잡았을 때, 1초 뒤 실제로 그만큼 가 있어야 한다."""
    bench = 1002.0 * 0.06                       # 60.1 °/s
    cfg = SafetyConfig(max_vel_deg_s=bench)
    step = cfg.max_step_deg("knee", 100.0)
    cmd = 0.0
    for _ in range(100):
        cmd, _ = guards.clamp_rate(1e6, cmd, step)
    assert cmd == pytest.approx(bench, rel=1e-9)
