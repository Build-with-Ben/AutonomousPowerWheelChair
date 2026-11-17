"""L1 tests for bridge TX, RX and bus-health logic (domain 6)."""

import pytest
from rnet_bridge.bridge_logic import JitterMeter, JoystickTx, StatusTracker, TxHealth


JSM_NIBBLE = 0x2
PM_NIBBLE = 0x0


def make_tx(**kwargs):
    kwargs.setdefault('tx_enabled', True)
    return JoystickTx(JSM_NIBBLE, **kwargs)


# --- 6.3 command transmission --------------------------------------------------------------

@pytest.mark.req('6.3.1')
def test_frame_uses_joystick_id_and_dlc_2():
    tx = make_tx()
    frame_id, data = tx.next_frame(now=0.0)
    assert frame_id == 0x02000200
    assert len(data) == 2


@pytest.mark.req('6.3.5', '9.1.5')
def test_tx_disabled_by_default():
    tx = JoystickTx(JSM_NIBBLE)
    tx.set_command(0.5, 0.0, now=0.0)
    assert tx.tx_enabled is False
    assert tx.next_frame(now=0.0) is None


@pytest.mark.req('6.3.5')
def test_tx_disabled_sends_nothing_even_with_fresh_commands():
    tx = make_tx(tx_enabled=False)
    for i in range(10):
        tx.set_command(0.5, 0.5, now=i * 0.01)
        assert tx.next_frame(now=i * 0.01) is None


@pytest.mark.req('1.4.2', '6.3.4')
def test_sends_zero_before_any_command():
    assert make_tx().next_frame(now=0.0)[1] == bytes([0, 0])


@pytest.mark.req('6.3.3')
def test_sends_most_recent_command():
    tx = make_tx()
    tx.set_command(0.5, 0.0, now=0.000)
    tx.set_command(0.25, 0.0, now=0.005)
    assert tx.joystick_pct(now=0.010) == (0, 10)


@pytest.mark.req('6.3.3')
def test_same_command_repeated_every_tick_without_new_input():
    tx = make_tx()
    tx.set_command(0.5, 0.0, now=0.0)
    sent = [tx.next_frame(now=t / 1000)[1] for t in range(0, 100, 10)]
    assert sent == [bytes([0, 20])] * 10


@pytest.mark.req('6.3.4', '1.4.1', '5.7.1')
def test_stale_command_sends_zero():
    tx = make_tx(stale_timeout=0.1)
    tx.set_command(0.5, 1.0, now=0.0)
    assert tx.joystick_pct(now=0.100) == (-20, 20)   # left turn -> negative X
    assert tx.joystick_pct(now=0.101) == (0, 0)


@pytest.mark.req('6.3.4')
def test_fresh_command_after_stale_resumes():
    tx = make_tx(stale_timeout=0.1)
    tx.set_command(0.5, 0.0, now=0.0)
    assert tx.joystick_pct(now=1.0) == (0, 0)
    tx.set_command(0.5, 0.0, now=1.0)
    assert tx.joystick_pct(now=1.01) == (0, 20)


@pytest.mark.req('6.3.6')
def test_max_pct_defaults_to_20():
    tx = make_tx()
    assert tx.max_pct == 20
    tx.set_command(10.0, -10.0, now=0.0)
    assert tx.joystick_pct(now=0.0) == (20, 20)    # right turn -> positive X


# --- 6.3.7 jitter ------------------------------------------------------------------------

@pytest.mark.req('6.3.7', '7.2.3.4')
def test_jitter_zero_for_perfect_period():
    meter = JitterMeter(period=0.010)
    for i in range(10):
        meter.record(i * 0.010)
    assert meter.max_jitter == pytest.approx(0.0, abs=1e-9)


@pytest.mark.req('6.3.7', '7.2.3.4')
def test_jitter_reports_worst_deviation():
    meter = JitterMeter(period=0.010)
    for t in (0.000, 0.010, 0.023, 0.031, 0.041):
        meter.record(t)
    assert meter.max_jitter == pytest.approx(0.003)
    meter.reset()
    assert meter.max_jitter == 0.0


# --- 6.4 status reception ------------------------------------------------------------------

@pytest.mark.req('6.4.1', '6.4.2')
def test_status_fields_carry_last_update_time():
    tracker = StatusTracker(PM_NIBBLE)
    tracker.on_frame(0x0C140000, b'', now=1.0)
    tracker.on_frame(0x1C0C0000, bytes([80]), now=1.5)
    tracker.on_frame(0x1C0C0000, bytes([79]), now=2.5)
    assert tracker.fields == {'pm_alive': True, 'battery_pct': 79}
    assert tracker.stamps == {'pm_alive': 1.0, 'battery_pct': 2.5}


@pytest.mark.req('6.4.3', '7.2.3.3')
def test_unknown_frames_counted_by_id_without_error():
    tracker = StatusTracker(PM_NIBBLE)
    for _ in range(3):
        tracker.on_frame(0x123, bytes(8), now=0.0)
    tracker.on_frame(0x456, b'', now=0.0)
    assert tracker.unknown == {0x123: 3, 0x456: 1}
    assert tracker.fields == {}


@pytest.mark.req('6.5.3')
def test_rx_silent_after_timeout():
    tracker = StatusTracker(PM_NIBBLE, rx_silent_timeout=1.0, now=0.0)
    assert not tracker.rx_silent(now=0.99)
    assert tracker.rx_silent(now=1.0)


@pytest.mark.req('6.5.3')
def test_any_frame_clears_rx_silent():
    tracker = StatusTracker(PM_NIBBLE, rx_silent_timeout=1.0, now=0.0)
    tracker.on_frame(0x123, b'', now=1.5)  # unknown frames still count as traffic
    assert not tracker.rx_silent(now=2.0)
    assert tracker.rx_count == 1


# --- 6.5 bus health ----------------------------------------------------------------------

@pytest.mark.req('6.5.1', '7.2.3.2')
def test_send_failures_counted():
    health = TxHealth()
    for ok in (True, False, False, True, False):
        health.record(ok, now=0.0)
    assert health.failures == 3
    assert health.sent == 2


@pytest.mark.req('6.5.2')
def test_tx_failure_reported_after_100_ms_of_failures():
    health = TxHealth(failure_timeout=0.1)
    for i in range(10):
        health.record(False, now=i * 0.010)
        assert not health.tx_failure(now=i * 0.010)
    health.record(False, now=0.100)
    assert health.tx_failure(now=0.100)


@pytest.mark.req('6.5.2')
def test_intermittent_failures_do_not_report_tx_failure():
    health = TxHealth(failure_timeout=0.1)
    for i in range(30):
        health.record(i % 2 == 0, now=i * 0.010)
    assert not health.tx_failure(now=0.300)


@pytest.mark.req('6.5.2')
def test_successful_send_clears_tx_failure():
    health = TxHealth(failure_timeout=0.1)
    health.record(False, now=0.0)
    assert health.tx_failure(now=0.2)
    health.record(True, now=0.2)
    assert not health.tx_failure(now=0.2)
