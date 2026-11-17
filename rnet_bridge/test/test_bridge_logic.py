"""L1 tests for bridge TX, RX and bus-health logic (domain 6)."""

import pytest
from rnet_bridge.bridge_logic import JoystickFollower, ReplyMeter, StatusTracker, TxHealth


JSM_NIBBLE = 0x2
PM_NIBBLE = 0x0
CENTRED = bytes([0x00, 0x00])
FORWARD = bytes([0x00, 0x60])        # chair joystick pushed forward (Y = +96)
LEFT = bytes([0xA0, 0x00])           # chair joystick pushed left (X = -96)
PERIOD = 0.010                       # JSM joystick frame period, measured on the chair


def make_follower(**kwargs):
    kwargs.setdefault('tx_enabled', True)
    return JoystickFollower(JSM_NIBBLE, **kwargs)


def driving(**kwargs):
    """Return a follower with a fresh forward command at t = 0."""
    f = make_follower(**kwargs)
    f.set_command(0.5, 0.0, now=0.0)
    return f


# --- 6.3.1 reply to each JSM frame ---------------------------------------------------------

@pytest.mark.req('6.3.1')
def test_reply_uses_joystick_id_and_dlc_2():
    frame_id, data = driving().on_jsm_frame(CENTRED, now=0.0)
    assert frame_id == 0x02000200
    assert len(data) == 2


@pytest.mark.req('6.3.1')
def test_recognises_jsm_frame_id():
    f = make_follower()
    assert f.is_jsm_frame(0x02000200)
    assert not f.is_jsm_frame(0x02000100)
    assert not f.is_jsm_frame(0x0C140000)


@pytest.mark.req('6.3.1')
def test_one_reply_per_jsm_frame():
    f = driving()
    replies = [f.on_jsm_frame(CENTRED, now=i * PERIOD) for i in range(10)]
    assert replies == [(0x02000200, bytes([0, 20]))] * 10


@pytest.mark.req('6.3.1')
def test_has_no_timer_api():
    # Sending is only ever triggered by a JSM frame.
    assert not hasattr(make_follower(), 'next_frame')


# --- 6.3.3 / 6.3.4 / 6.3.5 / 6.3.6 ---------------------------------------------------------

@pytest.mark.req('6.3.3')
def test_reply_carries_most_recent_command():
    f = make_follower()
    f.set_command(0.5, 0.0, now=0.000)
    f.set_command(0.25, 0.0, now=0.005)
    assert f.on_jsm_frame(CENTRED, now=0.010)[1] == bytes([0, 10])


@pytest.mark.req('6.3.5', '9.1.5')
def test_tx_disabled_by_default():
    f = JoystickFollower(JSM_NIBBLE)
    f.set_command(0.5, 0.0, now=0.0)
    assert f.tx_enabled is False
    assert f.on_jsm_frame(CENTRED, now=0.0) is None


@pytest.mark.req('6.3.5')
def test_tx_disabled_never_replies():
    f = make_follower(tx_enabled=False)
    for i in range(10):
        f.set_command(0.5, 0.5, now=i * PERIOD)
        assert f.on_jsm_frame(CENTRED, now=i * PERIOD) is None


@pytest.mark.req('1.4.2', '6.3.4')
def test_no_reply_before_any_command():
    assert make_follower().on_jsm_frame(CENTRED, now=0.0) is None


@pytest.mark.req('6.3.4', '1.4.1', '5.7.1')
def test_stale_command_stops_replies():
    f = make_follower(stale_timeout=0.1)
    f.set_command(0.5, 1.0, now=0.0)
    assert f.on_jsm_frame(CENTRED, now=0.100)[1] == bytes([0xEC, 20])   # left -> X = -20
    assert f.on_jsm_frame(CENTRED, now=0.101) is None


@pytest.mark.req('6.3.4')
def test_fresh_command_after_stale_resumes():
    f = make_follower(stale_timeout=0.1)
    f.set_command(0.5, 0.0, now=0.0)
    assert f.on_jsm_frame(CENTRED, now=1.0) is None
    f.set_command(0.5, 0.0, now=1.0)
    assert f.on_jsm_frame(CENTRED, now=1.01)[1] == bytes([0, 20])


@pytest.mark.req('6.3.6')
def test_max_pct_defaults_to_20():
    f = make_follower()
    assert f.max_pct == 20
    f.set_command(10.0, -10.0, now=0.0)
    assert f.joystick_pct(now=0.0) == (20, 20)    # right turn -> positive X


# --- 6.3.8 operator override ---------------------------------------------------------------

@pytest.mark.req('6.3.8.1')
@pytest.mark.parametrize('deflected', [FORWARD, LEFT, bytes([0x01, 0x00]), bytes([0x00, 0xFF])])
def test_no_reply_while_chair_joystick_deflected(deflected):
    f = driving()
    assert f.on_jsm_frame(deflected, now=0.0) is None
    assert f.operator_override(now=0.0)


@pytest.mark.req('6.3.8.2')
def test_rearms_only_after_centred_for_rearm_time():
    f = make_follower(rearm_time=0.5)
    f.on_jsm_frame(FORWARD, now=0.0)
    t = PERIOD
    while t < 0.5 - 1e-9:
        f.set_command(0.5, 0.0, now=t)
        assert f.on_jsm_frame(CENTRED, now=t) is None, f'replied at {t:.2f} s'
        t += PERIOD
    f.set_command(0.5, 0.0, now=0.5)
    assert f.on_jsm_frame(CENTRED, now=0.5) is not None


@pytest.mark.req('6.3.8.2')
def test_stick_passing_through_centre_does_not_rearm():
    f = make_follower(rearm_time=0.5)
    # Operator rocks the stick forward and back, passing through centre.
    for i, data in enumerate([FORWARD, CENTRED, CENTRED, LEFT, CENTRED, FORWARD, CENTRED]):
        now = i * PERIOD
        f.set_command(0.5, 0.0, now=now)
        assert f.on_jsm_frame(data, now=now) is None


@pytest.mark.req('6.3.8.1', '6.3.5')
def test_deflection_tracked_while_tx_disabled():
    f = make_follower(tx_enabled=False, rearm_time=0.5)
    f.set_command(0.5, 0.0, now=0.0)
    f.on_jsm_frame(FORWARD, now=0.0)
    f.tx_enabled = True                           # enabled mid-use
    f.set_command(0.5, 0.0, now=0.1)
    assert f.on_jsm_frame(CENTRED, now=0.1) is None


@pytest.mark.req('6.3.8.1')
@pytest.mark.parametrize('data', [b'', bytes([0x00])])
def test_malformed_jsm_frame_treated_as_operator_input(data):
    f = driving()
    assert f.on_jsm_frame(data, now=0.0) is None
    assert f.operator_override(now=0.0)


# --- 6.3.7 reply delay ---------------------------------------------------------------------

@pytest.mark.req('6.3.7', '7.2.3.4')
def test_reply_meter_counts_late_replies():
    meter = ReplyMeter(window=0.001)
    for jsm, sent in [(0.000, 0.0003), (0.010, 0.0108), (0.020, 0.0215), (0.030, 0.0320)]:
        meter.record(jsm, sent)
    assert meter.replies == 4
    assert meter.late == 2
    assert meter.max_delay == pytest.approx(0.0020)


@pytest.mark.req('6.3.7')
def test_reply_meter_reset():
    meter = ReplyMeter()
    meter.record(0.0, 0.005)
    meter.reset()
    assert (meter.replies, meter.late, meter.max_delay) == (0, 0, 0.0)


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
