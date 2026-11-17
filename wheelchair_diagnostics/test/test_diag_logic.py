"""L1 tests for diagnostics levels and counters (domain 7)."""

import pytest
from wheelchair_diagnostics.diag_logic import (
    battery_level, CurrentStats, ERROR, heartbeat_level, late_reply_level, OK, RateCounter,
    STALE, WARN)


@pytest.mark.req('7.1.2')
def test_levels_match_diagnostic_msgs():
    assert (OK, WARN, ERROR, STALE) == (0, 1, 2, 3)


@pytest.mark.req('7.2.1', '5.5.1', '5.5.2')
@pytest.mark.parametrize('pct, level', [
    (100, OK), (20, OK), (19.9, WARN), (10, WARN), (9.9, ERROR), (0, ERROR), (None, STALE),
])
def test_battery_levels(pct, level):
    assert battery_level(pct) == level


@pytest.mark.req('7.2.2')
def test_current_present_and_peak():
    stats = CurrentStats()
    assert stats.present is None and stats.peak is None
    for amps in (5.0, 22.5, 12.0):
        stats.update(amps)
    assert stats.present == 12.0
    assert stats.peak == 22.5


@pytest.mark.req('7.2.3.1')
def test_rate_counter_frames_per_second():
    counter = RateCounter(window=1.0)
    for i in range(300):                 # 100 Hz for 3 s
        counter.add(i * 0.01)
    assert counter.rate(now=2.995) == pytest.approx(100, abs=1)
    assert counter.total == 300


@pytest.mark.req('7.2.3.1', '7.2.5')
def test_rate_drops_to_zero_when_traffic_stops():
    counter = RateCounter(window=1.0)
    for i in range(20):
        counter.add(i * 0.05)
    assert counter.rate(now=0.99) == pytest.approx(20)
    assert counter.rate(now=2.0) == 0


@pytest.mark.req('7.2.3.4', '6.3.7')
@pytest.mark.parametrize('late, replies, level', [
    (0, 0, OK), (0, 1000, OK), (10, 1000, OK), (11, 1000, WARN), (50, 1000, WARN),
    (51, 1000, ERROR), (1000, 1000, ERROR),
])
def test_late_reply_levels(late, replies, level):
    assert late_reply_level(late, replies) == level


@pytest.mark.req('7.2.4', '5.3.1')
def test_heartbeat_age_and_level():
    assert heartbeat_level(None, now=5.0) == (STALE, None)
    level, age = heartbeat_level(4.0, now=5.0)
    assert level == OK and age == pytest.approx(1.0)
    assert heartbeat_level(2.5, now=5.0)[0] == ERROR
