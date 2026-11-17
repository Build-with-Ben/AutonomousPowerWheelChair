"""L1 tests for test sequence profiles and runner (domain 8)."""

import pytest
from wheelchair_test_sequences.sequence import (
    ABORTED, FAILED, forward_backward, PASSED, Profile, SequenceRunner, spin_in_place, Step)


RATE = 20.0


def started(steps=None):
    runner = SequenceRunner(steps or forward_backward())
    ok, _ = runner.start(now=0.0, mode='test', safety_ok=True)
    assert ok
    return runner


def run_to_end(runner, t0=0.0, limit=60.0):
    out = []
    t = t0
    while runner.running and t < limit:
        out.append((t, runner.tick(t)))
        t = round(t + 1 / RATE, 6)
    return out


# --- 8.2.1 forward / backward profile ----------------------------------------------------

@pytest.mark.req('8.2.1')
def test_forward_backward_step_order():
    names = [s.name for s in forward_backward()]
    assert names == ['ramp_forward', 'hold_forward', 'ramp_to_zero', 'pause',
                     'ramp_backward', 'hold_backward', 'ramp_to_stop']


@pytest.mark.req('8.2.1.1', '8.2.1.2', '8.2.1.3', '8.2.1.4', '8.2.1.5', '8.2.1.6')
@pytest.mark.parametrize('t, expected', [
    (0.0, 0.0),     # ramp up starts at zero
    (0.5, 0.05),    # half way up the 1 s ramp
    (1.0, 0.1),     # hold forward
    (3.9, 0.1),
    (4.5, 0.05),    # ramping down
    (5.0, 0.0),     # pause at zero
    (5.9, 0.0),
    (6.5, -0.05),   # ramp backward
    (7.0, -0.1),    # hold backward
    (9.9, -0.1),
    (10.5, -0.05),  # ramp to stop
])
def test_forward_backward_profile_values(t, expected):
    _, linear, angular = Profile(forward_backward()).at(t)
    assert linear == pytest.approx(expected)
    assert angular == 0.0


@pytest.mark.req('8.2.1', '8.3.2.3')
def test_profile_never_turns():
    profile = Profile(forward_backward())
    t = 0.0
    while profile.at(t) is not None:
        assert profile.at(t)[2] == 0.0
        t += 0.01


@pytest.mark.req('8.2.1.6')
def test_profile_ends_after_total_duration():
    profile = Profile(forward_backward())
    assert profile.duration == pytest.approx(11.0)
    assert profile.at(11.0) is None


@pytest.mark.req('8.1.5')
def test_every_speed_and_time_is_a_parameter():
    profile = Profile(forward_backward(speed=0.2, ramp=0.5, hold=1.0, pause=0.25))
    assert profile.duration == pytest.approx(0.5 * 4 + 1.0 * 2 + 0.25)
    assert profile.at(0.5)[1] == pytest.approx(0.2)
    assert profile.at(2.0)[1] == pytest.approx(0.0)


@pytest.mark.req('8.4.1', '1.3.5.3')
def test_spin_turns_left_first_without_driving():
    profile = Profile(spin_in_place(rate=0.3))
    _, linear, angular = profile.at(1.5)
    assert linear == 0.0 and angular == pytest.approx(0.3)
    assert profile.at(6.5)[2] == pytest.approx(-0.3)


# --- 8.1 execution -----------------------------------------------------------------------

@pytest.mark.req('8.1.1', '8.1.1.1', '8.1.1.2')
@pytest.mark.parametrize('mode, safety_ok, expected', [
    ('test', True, True),
    ('idle', True, False),
    ('manual', True, False),
    ('test', False, False),
])
def test_start_requires_test_mode_and_safety_ok(mode, safety_ok, expected):
    runner = SequenceRunner(forward_backward())
    ok, message = runner.start(now=0.0, mode=mode, safety_ok=safety_ok)
    assert ok == expected and runner.running == expected
    assert message


@pytest.mark.req('8.1.1')
def test_nothing_published_before_start():
    assert SequenceRunner(forward_backward()).tick(0.0) is None


@pytest.mark.req('8.1.1')
def test_second_start_while_running_is_refused():
    runner = started()
    assert not runner.start(now=1.0, mode='test', safety_ok=True)[0]


@pytest.mark.req('8.1.4')
def test_publishes_every_tick_while_running():
    out = run_to_end(started())
    assert all(cmd is not None for _, cmd in out)
    assert len(out) == pytest.approx(11.0 * RATE, abs=2)


@pytest.mark.req('8.1.2')
def test_stop_ends_immediately_with_zero():
    runner = started()
    runner.tick(2.0)
    assert runner.stop(now=2.0) == (0.0, 0.0)
    assert not runner.running
    assert runner.tick(2.05) is None
    assert runner.summary.result == ABORTED


@pytest.mark.req('8.1.3', '8.2.3.2')
def test_safety_fault_aborts():
    runner = started()
    runner.tick(2.0)
    runner.on_safety_state(False, ['estop'])
    assert not runner.running
    assert runner.tick(2.05) is None
    assert runner.summary.result == ABORTED
    assert 'estop' in runner.summary.abort_reason


@pytest.mark.req('8.1.3')
@pytest.mark.parametrize('mode', ['idle', 'manual', 'fault', 'estop'])
def test_leaving_test_mode_aborts(mode):
    runner = started()
    runner.tick(1.0)
    runner.on_mode(mode)
    assert not runner.running
    assert mode in runner.summary.abort_reason


@pytest.mark.req('8.1.3')
def test_safety_ok_and_test_mode_do_not_abort():
    runner = started()
    runner.on_safety_state(True)
    runner.on_mode('test')
    assert runner.running


# --- 8.2.2 / 8.2.3 logging and summary ---------------------------------------------------

@pytest.mark.req('8.2.2')
def test_each_step_logged_once_when_it_starts():
    runner = started()
    run_to_end(runner)
    names = [name for _, name in runner.step_log]
    assert names == [s.name for s in forward_backward()]
    times = [t for t, _ in runner.step_log]
    assert times == pytest.approx([0.0, 1.0, 4.0, 5.0, 6.0, 7.0, 10.0])


@pytest.mark.req('8.2.3.1')
def test_completed_run_passes_and_ends_with_zero():
    runner = started()
    out = run_to_end(runner)
    assert out[-1][1] == (0.0, 0.0)
    assert runner.summary.result == PASSED
    assert runner.summary.abort_reason is None


@pytest.mark.req('8.2.3.1')
def test_fail_reports_failed():
    runner = started()
    runner.fail('current above limit')
    assert runner.summary.result == FAILED


@pytest.mark.req('8.2.3.3')
def test_peak_current_tracked_per_step():
    runner = started([Step('a', 1.0, linear=0.1), Step('b', 1.0)])
    runner.tick(0.0)
    for amps in (3.0, 9.0, 4.0):
        runner.record_current(amps)
    runner.tick(1.5)
    runner.record_current(2.0)
    assert runner.summary.peak_current == {'a': 9.0, 'b': 2.0}


@pytest.mark.req('8.2.3.4')
def test_joystick_frames_counted_only_while_running():
    runner = SequenceRunner(forward_backward())
    runner.record_joystick_frame()          # before start: ignored
    runner.start(now=0.0, mode='test', safety_ok=True)
    runner.record_joystick_frame(count=100)
    runner.stop(now=1.0)
    runner.record_joystick_frame()          # after stop: ignored
    assert runner.summary.joystick_frames == 100
