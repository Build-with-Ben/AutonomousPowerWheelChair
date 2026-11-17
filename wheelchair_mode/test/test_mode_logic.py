"""L1 tests for the mode state machine (domain 3)."""

import pytest
from wheelchair_mode.mode_logic import (
    AUTO, ESTOP, FAULT, IDLE, MANUAL, ModeManager, MODES, STARTUP_FAULT, TEST)


def ready(**kwargs):
    """Return a manager in Idle with safety ok (Startup fault already reset)."""
    m = ModeManager(**kwargs)
    m.on_safety_state([], now=0.0)
    return m


@pytest.mark.req('3.1.1')
def test_all_modes_exist():
    assert set(MODES) == {IDLE, MANUAL, TEST, AUTO, FAULT, ESTOP}


@pytest.mark.req('3.1.3', '1.4.2')
def test_starts_in_idle():
    assert ModeManager().mode == IDLE


# --- 3.2.1 explicit requests -------------------------------------------------------------

@pytest.mark.req('3.2.1')
@pytest.mark.parametrize('target', [MANUAL, TEST])
def test_request_from_idle_enters_driving_mode(target):
    m = ready()
    assert m.request(target, now=1.0)[0]
    assert m.mode == target


@pytest.mark.req('3.2.1')
def test_safety_state_alone_never_enters_driving_mode():
    m = ready()
    for _ in range(3):
        m.on_safety_state([], now=1.0)
    assert m.mode == IDLE


@pytest.mark.req('3.2.1')
@pytest.mark.parametrize('start, target', [(MANUAL, TEST), (TEST, MANUAL)])
def test_driving_mode_to_driving_mode_goes_through_idle(start, target):
    m = ready()
    m.request(start, now=1.0)
    ok, reason = m.request(target, now=2.0)
    assert not ok and 'idle' in reason
    assert m.mode == start


@pytest.mark.req('3.2.1')
def test_request_idle_from_driving_mode():
    m = ready()
    m.request(MANUAL, now=1.0)
    assert m.request(IDLE, now=2.0)[0]
    assert m.mode == IDLE


@pytest.mark.req('3.2.1')
@pytest.mark.parametrize('target', [FAULT, ESTOP, 'bogus'])
def test_fault_estop_and_unknown_cannot_be_requested(target):
    m = ready()
    assert not m.request(target, now=1.0)[0]
    assert m.mode == IDLE


# --- 3.2.2 refuse while safety not ok ----------------------------------------------------

@pytest.mark.req('3.2.2', '5.7.2')
def test_startup_fault_blocks_leaving_idle_but_stays_idle():
    m = ModeManager()
    m.on_safety_state([STARTUP_FAULT], now=0.0)
    assert m.mode == IDLE
    ok, reason = m.request(MANUAL, now=1.0)
    assert not ok and STARTUP_FAULT in reason
    assert m.mode == IDLE


@pytest.mark.req('3.2.2', '1.4.2')
def test_cannot_leave_idle_before_first_safety_state():
    m = ModeManager()
    assert not m.request(MANUAL, now=0.0)[0]
    assert m.mode == IDLE


# --- 3.2.3 faults and e-stop force a mode ------------------------------------------------

@pytest.mark.req('3.2.3')
@pytest.mark.parametrize('start', [IDLE, MANUAL, TEST])
def test_fault_forces_fault_mode(start):
    m = ready()
    if start != IDLE:
        m.request(start, now=1.0)
    m.on_safety_state(['pm_heartbeat_lost'], now=2.0)
    assert m.mode == FAULT


@pytest.mark.req('3.2.3')
@pytest.mark.parametrize('faults', [['estop'], ['estop', 'pm_heartbeat_lost']])
def test_estop_forces_estop_mode_over_fault(faults):
    m = ready()
    m.request(MANUAL, now=1.0)
    m.on_safety_state(faults, now=2.0)
    assert m.mode == ESTOP


@pytest.mark.req('3.2.3')
def test_estop_while_in_fault_moves_to_estop():
    m = ready()
    m.on_safety_state(['over_current'], now=1.0)
    m.on_safety_state(['over_current', 'estop'], now=2.0)
    assert m.mode == ESTOP


# --- 3.2.4 leaving fault / e-stop --------------------------------------------------------

@pytest.mark.req('3.2.4', '3.2.4.1', '3.2.4.2')
@pytest.mark.parametrize('fault, latched_mode', [('pm_fault', FAULT), ('estop', ESTOP)])
def test_cleared_and_reset_returns_to_idle_not_driving(fault, latched_mode):
    m = ready()
    m.request(MANUAL, now=1.0)
    m.on_safety_state([fault], now=2.0)
    assert m.mode == latched_mode
    m.on_safety_state([], now=3.0)          # safety cleared the latch after a reset
    assert m.mode == IDLE


@pytest.mark.req('3.2.4')
@pytest.mark.parametrize('target', [IDLE, MANUAL, TEST])
def test_requests_refused_while_faulted(target):
    m = ready()
    m.on_safety_state(['pm_fault'], now=1.0)
    assert not m.request(target, now=2.0)[0]
    assert m.mode == FAULT


# --- 3.2.5 / 3.2.6 -----------------------------------------------------------------------

@pytest.mark.req('3.2.5')
def test_sequence_end_returns_to_idle():
    m = ready()
    m.request(TEST, now=1.0)
    m.on_sequence_ended(now=5.0, reason='aborted')
    assert m.mode == IDLE
    assert m.history[-1].reason == 'aborted'


@pytest.mark.req('3.2.5')
def test_sequence_end_outside_test_mode_is_ignored():
    m = ready()
    m.request(MANUAL, now=1.0)
    assert m.on_sequence_ended(now=2.0) is None
    assert m.mode == MANUAL


@pytest.mark.req('3.2.6', '1.1.3')
def test_auto_rejected_by_default():
    m = ready()
    ok, reason = m.request(AUTO, now=1.0)
    assert not ok and 'auto' in reason
    assert m.mode == IDLE


@pytest.mark.req('3.2.6')
def test_auto_available_once_enabled():
    m = ready(auto_enabled=True)
    assert m.request(AUTO, now=1.0)[0]


# --- 3.3.2 logging -----------------------------------------------------------------------

@pytest.mark.req('3.3.2')
def test_every_change_logged_with_time_old_new_reason():
    m = ready()
    m.request(MANUAL, now=1.0)
    m.on_safety_state(['estop'], now=2.0)
    m.on_safety_state([], now=3.0)
    assert [(c.time, c.old, c.new) for c in m.history] == [
        (1.0, IDLE, MANUAL), (2.0, MANUAL, ESTOP), (3.0, ESTOP, IDLE)]
    assert all(c.reason for c in m.history)


@pytest.mark.req('3.3.2')
def test_no_log_entry_when_mode_unchanged():
    m = ready()
    m.request(IDLE, now=1.0)
    m.on_safety_state([], now=2.0)
    assert m.history == []
