"""L1 tests for the Safety gate (domain 5)."""

import pytest
from wheelchair_safety.safety_logic import (
    BRIDGE_STATUS_LOST, CAN_TX_FAILURE, COMMAND_STALE, ESTOP, LOW_BATTERY, OVER_CURRENT,
    PM_FAULT, PM_HEARTBEAT_LOST, SafetyConfig, SafetyMonitor, STARTUP)

CMD = (0.3, 0.5)


class Healthy:
    """A safety monitor with healthy inputs that the test can step through time."""

    def __init__(self, **config):
        self.now = 0.0
        self.status = {}
        self.m = SafetyMonitor(SafetyConfig(**config), now=0.0)
        self.feed()
        assert self.m.reset(self.now)[0], 'startup reset should succeed'

    def feed(self, **status):
        """Send a fresh command and a fresh status with a heartbeat."""
        self.status.update(status)
        self.m.on_cmd(*CMD, now=self.now)
        self.m.on_status(self.now, pm_heartbeat_time=self.now, **self.status)

    def advance(self, seconds, step=0.02, **status):
        """Feed healthy inputs every ``step`` for ``seconds``."""
        end = self.now + seconds
        while self.now + step <= end + 1e-9:
            self.now += step
            self.feed(**status)


# --- 5.1 pass-through --------------------------------------------------------------------

@pytest.mark.req('5.1.1')
def test_passes_command_through_when_healthy():
    h = Healthy()
    assert h.m.output(h.now) == CMD
    assert h.m.state(h.now).ok


@pytest.mark.req('5.1.1', '5.1.2')
def test_passes_latest_command():
    h = Healthy()
    h.m.on_cmd(0.1, -0.2, now=h.now)
    assert h.m.output(h.now) == (0.1, -0.2)


# --- 5.2 command watchdog ----------------------------------------------------------------

@pytest.mark.req('5.2.1', '1.4.1')
def test_stale_command_outputs_zero():
    h = Healthy(cmd_timeout=0.2)
    t0 = h.now
    h.m.on_status(t0 + 0.2, pm_heartbeat_time=t0 + 0.2)
    assert h.m.output(t0 + 0.2) == CMD
    assert h.m.output(t0 + 0.201) == (0.0, 0.0)


@pytest.mark.req('5.2.2')
def test_stale_command_does_not_latch_and_resumes():
    h = Healthy(cmd_timeout=0.2)
    later = h.now + 0.5
    h.m.on_status(later, pm_heartbeat_time=later)
    assert h.m.output(later) == (0.0, 0.0)
    state = h.m.state(later)
    assert state.ok and not state.latched
    assert COMMAND_STALE in state.warnings
    h.m.on_cmd(*CMD, now=later)
    assert h.m.output(later) == CMD


@pytest.mark.req('1.4.2', '5.2.1')
def test_no_command_yet_outputs_zero():
    m = SafetyMonitor(now=0.0)
    m.on_status(0.0, pm_heartbeat_time=0.0)
    m.reset(0.0)
    assert m.output(0.0) == (0.0, 0.0)


# --- 5.3 power module monitoring ---------------------------------------------------------

@pytest.mark.req('5.3.1')
def test_heartbeat_lost_latches_after_timeout():
    h = Healthy(heartbeat_timeout=2.5)
    last_hb = h.now
    for i in range(1, 125):                      # status keeps coming, heartbeat doesn't
        t = last_hb + i * 0.02
        h.m.on_cmd(*CMD, now=t)
        h.m.on_status(t, pm_heartbeat_time=last_hb)
        assert h.m.output(t) == CMD, f'stopped early at {t - last_hb:.2f} s'
    t = last_hb + 2.5
    h.m.on_cmd(*CMD, now=t)
    h.m.on_status(t, pm_heartbeat_time=last_hb)
    assert h.m.output(t) == (0.0, 0.0)
    assert PM_HEARTBEAT_LOST in h.m.state(t).faults


@pytest.mark.req('5.3.1')
def test_heartbeat_every_second_keeps_driving():
    h = Healthy()
    hb = h.now
    for i in range(1, 500):
        t = h.now + i * 0.02
        if i % 50 == 0:
            hb = t
        h.m.on_cmd(*CMD, now=t)
        h.m.on_status(t, pm_heartbeat_time=hb)
    assert h.m.state(t).ok


@pytest.mark.req('5.3.2')
def test_over_current_latches_after_sustained_time():
    h = Healthy(current_limit=40.0, current_time=2.0)
    h.advance(1.9, motor_current_a=45.0)       # above the limit since t = 0.02
    assert h.m.output(h.now) == CMD
    h.advance(0.2, motor_current_a=45.0)
    assert h.m.output(h.now) == (0.0, 0.0)
    assert OVER_CURRENT in h.m.state(h.now).faults


@pytest.mark.req('5.3.2')
def test_short_current_spikes_do_not_latch():
    h = Healthy(current_limit=40.0, current_time=2.0)
    for _ in range(5):
        h.advance(1.5, motor_current_a=45.0)
        h.advance(0.1, motor_current_a=10.0)
    assert h.m.state(h.now).ok


@pytest.mark.req('5.3.2')
def test_over_current_disabled_while_limit_tbd():
    h = Healthy()
    h.advance(5.0, motor_current_a=1000.0)
    assert h.m.state(h.now).ok


@pytest.mark.req('5.3.3')
def test_pm_fault_latches_immediately():
    h = Healthy()
    h.feed(pm_fault=True)
    assert h.m.output(h.now) == (0.0, 0.0)
    assert PM_FAULT in h.m.state(h.now).faults


@pytest.mark.req('5.3.4')
def test_bridge_status_lost_latches():
    h = Healthy(status_timeout=1.0)
    t0 = h.now
    h.m.on_cmd(*CMD, now=t0 + 0.99)
    assert h.m.output(t0 + 0.99) == CMD
    h.m.on_cmd(*CMD, now=t0 + 1.0)
    assert h.m.output(t0 + 1.0) == (0.0, 0.0)
    assert BRIDGE_STATUS_LOST in h.m.state(t0 + 1.0).faults


@pytest.mark.req('5.3.5')
def test_can_tx_failure_latches():
    h = Healthy()
    h.feed(can_tx_failure=True)
    assert h.m.output(h.now) == (0.0, 0.0)
    assert CAN_TX_FAILURE in h.m.state(h.now).faults


# --- 5.4 e-stop --------------------------------------------------------------------------

@pytest.mark.req('5.4.1', '2.3.4')
def test_estop_zeroes_output_on_next_cycle():
    h = Healthy()
    h.m.on_estop(True, now=h.now)
    assert h.m.output(h.now + 0.02) == (0.0, 0.0)
    state = h.m.state(h.now)
    assert ESTOP in state.faults and not state.ok and state.latched


@pytest.mark.req('5.4.2')
def test_hardware_estop_open_acts_like_software_estop():
    h = Healthy()
    h.m.on_hw_estop(True, now=h.now)
    assert h.m.output(h.now) == (0.0, 0.0)
    assert ESTOP in h.m.state(h.now).faults


@pytest.mark.req('5.4.3')
def test_estop_stays_latched_after_release_until_reset():
    h = Healthy()
    h.m.on_estop(True, now=h.now)
    h.advance(0.1)
    h.m.on_estop(False, now=h.now)
    h.advance(0.5)
    assert h.m.output(h.now) == (0.0, 0.0)
    assert h.m.reset(h.now)[0]
    assert h.m.output(h.now) == CMD


@pytest.mark.req('5.4.3', '5.6.2')
def test_reset_refused_while_estop_still_active():
    h = Healthy()
    h.m.on_estop(True, now=h.now)
    ok, message = h.m.reset(h.now)
    assert not ok and ESTOP in message
    assert h.m.output(h.now) == (0.0, 0.0)


@pytest.mark.req('5.4.3', '5.4.2')
def test_reset_refused_while_hardware_estop_open():
    h = Healthy()
    h.m.on_hw_estop(True, now=h.now)
    h.m.on_estop(False, now=h.now)
    assert not h.m.reset(h.now)[0]


# --- 5.5 low battery ---------------------------------------------------------------------

@pytest.mark.req('5.5.1', '7.2.1')
def test_low_battery_warns_without_stopping():
    h = Healthy()
    h.feed(battery_pct=19)
    state = h.m.state(h.now)
    assert state.ok and LOW_BATTERY in state.warnings
    assert h.m.output(h.now) == CMD


@pytest.mark.req('5.5.1')
def test_battery_at_threshold_does_not_warn():
    h = Healthy()
    h.feed(battery_pct=20)
    assert LOW_BATTERY not in h.m.state(h.now).warnings


@pytest.mark.req('5.5.2')
def test_very_low_battery_limits_speed_to_25_percent():
    h = Healthy()
    h.feed(battery_pct=9)
    h.m.on_cmd(0.5, -1.0, now=h.now)
    assert h.m.output(h.now) == pytest.approx((0.125, -0.25))
    h.m.on_cmd(0.1, 0.1, now=h.now)
    assert h.m.output(h.now) == pytest.approx((0.1, 0.1))


# --- 5.6 state and reset -----------------------------------------------------------------

@pytest.mark.req('5.6.1.1', '5.6.1.2', '5.6.1.3')
def test_state_lists_every_active_fault():
    h = Healthy()
    h.feed(pm_fault=True, can_tx_failure=True)
    h.m.on_estop(True, now=h.now)
    state = h.m.state(h.now)
    assert not state.ok
    assert state.latched
    assert set(state.faults) == {PM_FAULT, CAN_TX_FAILURE, ESTOP}


@pytest.mark.req('5.6.1.1', '5.6.1.3')
def test_state_ok_when_nothing_latched():
    h = Healthy()
    state = h.m.state(h.now)
    assert state.ok and not state.latched and state.faults == ()


@pytest.mark.req('5.6.2')
def test_reset_clears_faults_whose_cause_is_gone():
    h = Healthy()
    h.feed(pm_fault=True)
    h.feed(pm_fault=False)
    assert PM_FAULT in h.m.state(h.now).faults      # still latched
    ok, _ = h.m.reset(h.now)
    assert ok
    assert h.m.state(h.now).ok


@pytest.mark.req('5.6.2')
def test_reset_refuses_and_names_cause_still_present():
    h = Healthy()
    h.feed(pm_fault=True, can_tx_failure=True)
    h.feed(pm_fault=False)                          # PM fault gone, TX failure remains
    ok, message = h.m.reset(h.now)
    assert not ok and CAN_TX_FAILURE in message
    assert h.m.state(h.now).faults == (CAN_TX_FAILURE,)


@pytest.mark.req('5.6.2', '5.3.2')
def test_reset_refused_while_current_still_high():
    h = Healthy(current_limit=40.0, current_time=2.0)
    h.advance(2.1, motor_current_a=45.0)
    assert not h.m.reset(h.now)[0]
    h.feed(motor_current_a=10.0)
    assert h.m.reset(h.now)[0]


@pytest.mark.req('5.6.3')
def test_fault_raise_and_clear_are_logged():
    h = Healthy()
    h.feed(pm_fault=True)
    h.feed(pm_fault=False)
    h.m.reset(h.now)
    pm_events = [(e.action, e.fault) for e in h.m.events if e.fault == PM_FAULT]
    assert pm_events == [('raised', PM_FAULT), ('cleared', PM_FAULT)]
    assert all(e.detail for e in h.m.events)


@pytest.mark.req('5.6.3')
def test_fault_logged_once_while_it_persists():
    h = Healthy()
    for _ in range(5):
        h.feed(pm_fault=True)
    assert sum(e.fault == PM_FAULT for e in h.m.events) == 1


# --- 5.7 startup -------------------------------------------------------------------------

@pytest.mark.req('5.7.2', '1.4.2')
def test_starts_with_startup_fault_latched():
    m = SafetyMonitor(now=0.0)
    m.on_cmd(*CMD, now=0.0)
    m.on_status(0.0, pm_heartbeat_time=0.0)
    state = m.state(0.0)
    assert state.faults == (STARTUP,) and not state.ok
    assert m.output(0.0) == (0.0, 0.0)


@pytest.mark.req('5.7.2')
def test_reset_clears_startup_fault():
    m = SafetyMonitor(now=0.0)
    m.on_cmd(*CMD, now=0.0)
    m.on_status(0.0, pm_heartbeat_time=0.0)
    assert m.reset(0.0)[0]
    assert m.output(0.0) == CMD
