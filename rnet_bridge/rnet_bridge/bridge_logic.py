"""
Bridge transmit, receive and bus-health logic (requirements 6.3, 6.4, 6.5).

Pure logic, no rclpy. Every method that depends on time takes ``now`` in seconds,
so tests can step time instead of sleeping.
"""

from collections import Counter

from rnet_bridge import frames


class JoystickFollower:
    """
    Replies to the chair joystick module's frames with our own (FollowJSM).

    The bridge never sends joystick frames on its own timer (6.3.1). For each joystick
    frame received from the JSM it calls ``on_jsm_frame`` and, if that returns a frame,
    sends it straight away. See docs/analysis/jsm_handling.md.
    """

    def __init__(self, joystick_nibble, linear_limit=0.5, angular_limit=1.0, max_pct=20,
                 stale_timeout=0.1, rearm_time=0.5, tx_enabled=False):
        self.frame_id = frames.joystick_id(joystick_nibble)
        self.linear_limit = linear_limit
        self.angular_limit = angular_limit
        self.max_pct = max_pct
        self.stale_timeout = stale_timeout
        self.rearm_time = rearm_time
        self.tx_enabled = tx_enabled
        self._cmd = (0.0, 0.0)
        self._cmd_time = None
        self._deflected_at = None

    def set_command(self, linear_x, angular_z, now):
        """Store the most recent ``/cmd_vel/safe`` command (6.3.3)."""
        self._cmd = (linear_x, angular_z)
        self._cmd_time = now

    def joystick_pct(self, now):
        """Return ``(x, y)`` for the latest command, or ``None`` if it is stale (6.3.4)."""
        if self._cmd_time is None or now - self._cmd_time > self.stale_timeout:
            return None
        linear_x, angular_z = self._cmd
        return frames.twist_to_joystick(linear_x, angular_z, self.linear_limit,
                                        self.angular_limit, self.max_pct)

    def is_jsm_frame(self, frame_id):
        """Return True if ``frame_id`` is the JSM's joystick frame."""
        return frame_id == self.frame_id

    def operator_override(self, now):
        """Return True while the chair's own joystick is deflected or re-arming (6.3.8)."""
        return (self._deflected_at is not None
                and now - self._deflected_at < self.rearm_time)

    def on_jsm_frame(self, data, now):
        """
        Handle one JSM joystick frame and return ``(frame_id, data)`` to send, or ``None``.

        Deflection is tracked even while TX is disabled, so enabling TX while the chair's
        joystick is in use doesn't take over from the operator.
        """
        try:
            centred = frames.decode_joystick(data) == (0, 0)
        except ValueError:
            centred = False             # malformed: treat as the operator's input
        if not centred:
            self._deflected_at = now    # 6.3.8.1
            return None
        if self.operator_override(now):
            return None                 # 6.3.8.2
        if not self.tx_enabled:
            return None                 # 6.3.5
        pct = self.joystick_pct(now)
        if pct is None:
            return None                 # 6.3.4: stale, let the JSM's centred frame stand
        return self.frame_id, frames.encode_joystick(*pct)


class StatusTracker:
    """Decodes received frames and keeps the latest RnetStatus fields (6.4)."""

    def __init__(self, pm_nibble, current_scale=1.0, rx_silent_timeout=1.0, now=0.0):
        self.pm_nibble = pm_nibble
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
        decoded = frames.decode_status(frame_id, data, self.pm_nibble, self.current_scale)
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


class ReplyMeter:
    """Measures the delay from each JSM frame to our reply (6.3.7)."""

    def __init__(self, window=0.001):
        self.window = window
        self.replies = 0
        self.late = 0
        self.max_delay = 0.0

    def record(self, jsm_time, sent_time):
        """Record one reply, sent at ``sent_time`` for a JSM frame received at ``jsm_time``."""
        delay = sent_time - jsm_time
        self.replies += 1
        if delay > self.window:
            self.late += 1
        self.max_delay = max(self.max_delay, delay)

    def reset(self):
        """Start a new measurement window."""
        self.replies = 0
        self.late = 0
        self.max_delay = 0.0
