"""L1 tests for Motor Control limits and mode gating (domain 4)."""

import pytest
from wheelchair_control.limits import (
    AccelLimiter, BRINGUP, clamp_command, DEFAULT_PROFILE, Limits, NORMAL, PROFILES,
    source_allowed, SourceArbiter)


# --- 4.1.1 / 4.1.2 / 4.1.4 arbitration ---------------------------------------------------

@pytest.mark.req('4.1.1')
def test_highest_priority_fresh_source_wins():
    a = SourceArbiter()
    a.set_command('auto', 0.1, 0.0, now=0.0)
    a.set_command('test', 0.2, 0.0, now=0.0)
    a.set_command('teleop', 0.3, 0.0, now=0.0)
    assert a.select('auto', now=0.0) == ('teleop', 0.3, 0.0)


@pytest.mark.req('4.1.1', '2.4.2')
def test_teleop_outranks_other_sources():
    # teleop and test coexist in Test mode; teleop and auto coexist in Auto mode.
    # (test and auto never share a mode, so their relative priority is unobservable.)
    a = SourceArbiter()
    a.set_command('teleop', 0.3, 0.0, now=0.0)
    a.set_command('test', 0.2, 0.0, now=0.0)
    assert a.select('test', now=0.0)[0] == 'teleop'
    b = SourceArbiter()
    b.set_command('teleop', 0.3, 0.0, now=0.0)
    b.set_command('auto', 0.1, 0.0, now=0.0)
    assert b.select('auto', now=0.0)[0] == 'teleop'


@pytest.mark.req('4.1.2')
def test_stale_source_drops_out():
    a = SourceArbiter(timeout=0.25)
    a.set_command('teleop', 0.3, 0.0, now=0.0)
    a.set_command('test', 0.2, 0.0, now=0.10)
    assert a.select('test', now=0.25)[0] == 'teleop'    # teleop fresh at the boundary
    assert a.select('test', now=0.26) == ('test', 0.2, 0.0)   # teleop stale -> test wins
    assert a.select('test', now=0.36) == (None, 0.0, 0.0)     # test now stale too


@pytest.mark.req('4.1.2', '4.1.4')
def test_all_stale_gives_zero():
    a = SourceArbiter(timeout=0.25)
    a.set_command('teleop', 0.3, 0.0, now=0.0)
    assert a.select('manual', now=0.3) == (None, 0.0, 0.0)


@pytest.mark.req('4.1.4')
def test_no_sources_gives_zero():
    assert SourceArbiter().select('manual', now=0.0) == (None, 0.0, 0.0)


@pytest.mark.req('4.1.3', '3.3.1')
def test_mode_blocks_disallowed_source():
    # In Auto mode, teleop and auto are allowed but test is not, even though test
    # outranks auto. A blocked source cannot win.
    a = SourceArbiter()
    a.set_command('test', 0.2, 0.0, now=0.0)
    a.set_command('auto', 0.1, 0.0, now=0.0)
    assert a.select('auto', now=0.0)[0] == 'auto'


@pytest.mark.req('4.1.3')
def test_idle_and_fault_block_every_source():
    a = SourceArbiter()
    for source in ('teleop', 'test', 'auto'):
        a.set_command(source, 0.3, 0.0, now=0.0)
    for mode in ('idle', 'fault', 'estop', 'unknown'):
        assert a.select(mode, now=0.0) == (None, 0.0, 0.0)


@pytest.mark.req('4.1.1')
def test_custom_priorities():
    a = SourceArbiter(priorities={'teleop': 1, 'test': 99, 'auto': 0})
    a.set_command('teleop', 0.3, 0.0, now=0.0)
    a.set_command('test', 0.2, 0.0, now=0.0)
    assert a.select('test', now=0.0)[0] == 'test'


# --- 4.1.3 / 3.3.1 mode gating -----------------------------------------------------------

@pytest.mark.req('4.1.3', '3.3.1', '3.1.1')
@pytest.mark.parametrize('mode, allowed', [
    ('idle', set()),
    ('manual', {'teleop'}),
    ('test', {'test', 'teleop'}),
    ('auto', {'auto', 'teleop'}),
    ('fault', set()),
    ('estop', set()),
])
def test_only_allowed_sources_pass(mode, allowed):
    for source in ('teleop', 'test', 'auto'):
        assert source_allowed(mode, source) == (source in allowed)


@pytest.mark.req('4.1.3', '1.4.1')
def test_unknown_mode_blocks_everything():
    for source in ('teleop', 'test', 'auto'):
        assert not source_allowed('bogus', source)


# --- 4.2 speed limits --------------------------------------------------------------------

@pytest.mark.req('4.2.1')
@pytest.mark.parametrize('cmd, expected', [
    ((0.3, 0.5), (0.3, 0.5)),
    ((0.5, 1.0), (0.5, 1.0)),
    ((2.0, 3.0), (0.5, 1.0)),
    ((0.0, -3.0), (0.0, -1.0)),
])
def test_speed_clamped(cmd, expected):
    assert clamp_command(*cmd) == pytest.approx(expected)


@pytest.mark.req('4.2.2')
@pytest.mark.parametrize('linear, expected', [(-0.1, -0.1), (-0.25, -0.25), (-0.5, -0.25)])
def test_reverse_limited_to_half_forward(linear, expected):
    assert clamp_command(linear, 0.0)[0] == pytest.approx(expected)


@pytest.mark.req('4.2.2')
def test_reverse_ratio_is_configurable():
    limits = Limits(reverse_ratio=0.2)
    assert clamp_command(-1.0, 0.0, limits)[0] == pytest.approx(-0.1)


@pytest.mark.req('4.2.3')
def test_bringup_profile_is_20_percent_and_default():
    assert BRINGUP.max_linear == pytest.approx(0.2 * NORMAL.max_linear)
    assert BRINGUP.max_angular == pytest.approx(0.2 * NORMAL.max_angular)
    assert PROFILES[DEFAULT_PROFILE] is BRINGUP
    assert clamp_command(1.0, 1.0, BRINGUP) == pytest.approx((0.1, 0.2))
    assert clamp_command(-1.0, 0.0, BRINGUP)[0] == pytest.approx(-0.05)


# --- 4.3 acceleration limits -------------------------------------------------------------

def run(limiter, target, start, end, dt=0.05):
    out = None
    t = start
    while t <= end + 1e-9:
        out = limiter.step(*target, now=t)
        t += dt
    return out


@pytest.mark.req('4.3.1')
def test_linear_acceleration_limited():
    lim = AccelLimiter()
    lim.step(0.5, 0.0, now=0.0)
    assert lim.step(0.5, 0.0, now=0.1)[0] == pytest.approx(0.05)
    assert lim.step(0.5, 0.0, now=0.5)[0] == pytest.approx(0.25)
    assert lim.step(0.5, 0.0, now=2.0)[0] == pytest.approx(0.5)


@pytest.mark.req('4.3.2')
def test_angular_acceleration_limited():
    lim = AccelLimiter()
    lim.step(0.0, 1.0, now=0.0)
    assert lim.step(0.0, 1.0, now=0.2)[1] == pytest.approx(0.3)
    assert lim.step(0.0, -1.0, now=0.4)[1] == pytest.approx(0.0)


@pytest.mark.req('4.3.3')
def test_deceleration_toward_zero_uses_higher_limit():
    lim = AccelLimiter()
    run(lim, (0.5, 0.0), 0.0, 1.0)            # reach 0.5 m/s
    lim.step(0.0, 0.0, now=1.0)
    assert lim.step(0.0, 0.0, now=1.1)[0] == pytest.approx(0.4)   # 1.0 m/s^2
    assert lim.step(0.0, 0.0, now=1.6)[0] == pytest.approx(0.0)


@pytest.mark.req('4.3.3', '4.3.1')
def test_reversing_decelerates_then_accelerates():
    lim = AccelLimiter()
    run(lim, (0.1, 0.0), 0.0, 1.0)
    lim.step(-0.25, 0.0, now=1.0)
    # 0.1 s to stop at 1.0 m/s^2, then 0.1 s at 0.5 m/s^2 backwards.
    assert lim.step(-0.25, 0.0, now=1.2)[0] == pytest.approx(-0.05)


@pytest.mark.req('4.3.1')
def test_first_step_does_not_jump():
    assert AccelLimiter().step(0.5, 1.0, now=10.0) == (0.0, 0.0)


@pytest.mark.req('4.3.1')
def test_time_going_backwards_does_not_move_output():
    lim = AccelLimiter()
    lim.step(0.5, 0.0, now=1.0)
    assert lim.step(0.5, 0.0, now=0.5) == (0.0, 0.0)
