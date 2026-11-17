"""
Bridge transmit, receive and bus-health logic (requirements 6.3, 6.4, 6.5).

Pure logic, no rclpy. Every method that depends on time takes ``now`` in seconds,
so tests can step time instead of sleeping.
"""

from collections import Counter

from rnet_bridge import frames


class JoystickTx:
    """Holds the latest ``/cmd_vel/safe`` command and builds each joystick frame."""

    def __init__(self, nibble, linear_limit=0.5, angular_limit=1.0, max_pct=20,
                 stale_timeout=0.1, tx_enabled=False):
        self.frame_id = frames.joystick_id(nibble)
        self.linear_limit = linear_limit
        self.angular_limit = angular_limit
        self.max_pct = max_pct
        self.stale_timeout = stale_timeout
        self.tx_enabled = tx_enabled
        self._cmd = (0.0, 0.0)
        self._cmd_time = None

    def set_command(self, linear_x, angular_z, now):
        """Store the most recent command (6.3.3)."""
        self._cmd = (linear_x, angular_z)
        self._cmd_time = now

    def joystick_pct(self, now):
        """Return ``(x, y)`` to send now; ``(0, 0)`` if the command is stale (6.3.4)."""
        if self._cmd_time is None or now - self._cmd_time > self.stale_timeout:
            return 0, 0
        linear_x, angular_z = self._cmd
        return frames.twist_to_joystick(linear_x, angular_z, self.linear_limit,
                                        self.angular_limit, self.max_pct)

    def next_frame(self, now):
        """Return ``(frame_id, data)`` for this 10 ms tick, or ``None`` if TX is off (6.3.5)."""
        if not self.tx_enabled:
            return None
        return self.frame_id, frames.encode_joystick(*self.joystick_pct(now))


class StatusTracker:
    """Decodes received frames and keeps the latest RnetStatus fields (6.4)."""

    def __init__(self, nibble, current_scale=1.0, rx_silent_timeout=1.0, now=0.0):
        self.nibble = nibble
        self.current_scale = current_scale
        self.rx_silent_timeout = rx_silent_timeout
        self.fields = {}
        self.stamps = {}
        self.unknown = Counter()
        self.rx_count = 0
        self._last_rx = now

    def on_frame(self, frame_id, data, now):
        """Handle one frame from ``/from_can_bus``. Never raises on unknown frames."""
        self.rx_count += 1
        self._last_rx = now
        decoded = frames.decode_status(frame_id, data, self.nibble, self.current_scale)
        if decoded is None:
            self.unknown[frame_id] += 1
            return
        for key, value in decoded.items():
            self.fields[key] = value
            self.stamps[key] = now

    def rx_silent(self, now):
        """Return True if no frame at all arrived for ``rx_silent_timeout`` (6.5.3)."""
        return now - self._last_rx >= self.rx_silent_timeout


class TxHealth:
    """Counts send failures and detects a sustained CAN TX failure (6.5.1, 6.5.2)."""

    def __init__(self, failure_timeout=0.1):
        self.failure_timeout = failure_timeout
        self.failures = 0
        self.sent = 0
        self._failing_since = None

    def record(self, ok, now):
        """Record the result of one send attempt."""
        if ok:
            self.sent += 1
            self._failing_since = None
        else:
            self.failures += 1
            if self._failing_since is None:
                self._failing_since = now

    def tx_failure(self, now):
        """Return True once sends have failed continuously for ``failure_timeout``."""
        return (self._failing_since is not None
                and now - self._failing_since >= self.failure_timeout)


class JitterMeter:
    """Measures how far send intervals stray from the nominal period (6.3.7)."""

    def __init__(self, period=0.010):
        self.period = period
        self.max_jitter = 0.0
        self._last = None

    def record(self, now):
        """Record one send time."""
        if self._last is not None:
            self.max_jitter = max(self.max_jitter, abs(now - self._last - self.period))
        self._last = now

    def reset(self):
        """Start a new measurement window, keeping the last send time."""
        self.max_jitter = 0.0
