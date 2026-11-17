"""
Health levels and counters reported on ``/diagnostics`` (domain 7).

Pure logic, no rclpy. Levels use the ``diagnostic_msgs/DiagnosticStatus`` values (7.1.2).
"""

from collections import deque

OK = 0
WARN = 1
ERROR = 2
STALE = 3


def battery_level(pct, warn_pct=20.0, error_pct=10.0):
    """Return the level for a battery percentage, matching Safety 5.5 (7.2.1)."""
    if pct is None:
        return STALE
    if pct < error_pct:
        return ERROR
    if pct < warn_pct:
        return WARN
    return OK


def heartbeat_level(last_seen, now, timeout=2.5):
    """Return ``(level, age)`` for the power-module heartbeat (7.2.4)."""
    if last_seen is None:
        return STALE, None
    age = now - last_seen
    return (ERROR if age >= timeout else OK), age


def late_reply_level(late, replies, warn=0.01, error=0.05):
    """
    Return the level for the share of joystick replies later than 1 ms (6.3.7, 7.2.3.4).

    A late reply means the PM used the JSM's centred frame for that cycle, so the chair
    stutters. No replies at all (TX off, or operator override) is OK, not an error.
    """
    if replies == 0:
        return OK
    ratio = late / replies
    if ratio > error:
        return ERROR
    if ratio > warn:
        return WARN
    return OK


class CurrentStats:
    """Present motor current and peak since startup (7.2.2)."""

    def __init__(self):
        self.present = None
        self.peak = None

    def update(self, amps):
        """Record one current reading."""
        self.present = amps
        self.peak = amps if self.peak is None else max(self.peak, amps)


class RateCounter:
    """Counts events in a sliding window to give a per-second rate (7.2.3.1, 7.2.5)."""

    def __init__(self, window=1.0):
        self.window = window
        self.total = 0
        self._times = deque()

    def add(self, now, count=1):
        """Record ``count`` events at ``now``."""
        self.total += count
        self._times.extend([now] * count)

    def rate(self, now):
        """Return events per second over the last ``window`` seconds."""
        while self._times and self._times[0] <= now - self.window:
            self._times.popleft()
        return len(self._times) / self.window
