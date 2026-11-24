"""
Operator-input-to-velocity logic for teleop (requirements 2.2-2.3).

Pure logic, no rclpy. The node calls ``on_input`` for each ``/operator/input`` message
and ``tick`` from its 20 Hz timer, then publishes whatever ``tick`` returns.

Everything device specific — which stick, which button, which way round — lives in
``wheelchair_input``'s device profiles (2.1.2), so this module only ever sees named
functions on a normalised -1..1 scale. Any controller that can produce
``wheelchair_msgs/OperatorInput`` drives the chair through this one code path.
"""

from dataclasses import dataclass, field


@dataclass
class TeleopConfig:
    """Behaviour and limits. Every field is a ROS parameter (2.1.2, 2.2.4)."""

    deadzone: float = 0.1
    max_linear: float = 0.5
    max_angular: float = 1.0
    input_timeout: float = 0.5
    speed_scales: tuple = (0.25, 0.5, 1.0)
    #: Device ``source`` names to accept; empty accepts whichever device is publishing.
    sources: tuple = ()


@dataclass
class InputEvents:
    """One-shot requests raised by an input message (presses, not holds)."""

    estop: bool = False
    manual_mode: bool = False


def apply_deadzone(value, deadzone):
    """Return 0 inside ``±deadzone``, else rescale so full deflection stays 1.0 (2.2.3)."""
    if abs(value) <= deadzone:
        return 0.0
    sign = 1.0 if value > 0 else -1.0
    return sign * min(1.0, (abs(value) - deadzone) / (1.0 - deadzone))


@dataclass
class Teleop:
    """Deadman, disconnect and speed-scale state for the operator's control device."""

    config: TeleopConfig = field(default_factory=TeleopConfig)
    speed_step: int = 0
    source: str = ''            # device whose input is in effect
    _state: object = None       # last accepted OperatorInput-shaped input
    _last_input: float = None
    _was_driving: bool = False

    def accepts(self, source):
        """Return whether input from ``source`` is used (the enable/disable seam)."""
        return not self.config.sources or source in self.config.sources

    @staticmethod
    def _held(state, name):
        return bool(state is not None and getattr(state, name, False))

    def _rising(self, state, name):
        return self._held(state, name) and not self._held(self._state, name)

    def on_input(self, state, now):
        """Store one input message and return the one-shot requests it makes.

        ``state`` is anything with the fields of ``wheelchair_msgs/OperatorInput``.
        Input from a device this teleop does not accept is ignored entirely.
        """
        if not self.accepts(getattr(state, 'source', '')):
            return InputEvents()
        events = InputEvents(
            estop=self._rising(state, 'estop'),
            manual_mode=self._rising(state, 'mode_manual'),
        )
        if self._rising(state, 'speed_up'):
            self.speed_step = min(self.speed_step + 1, len(self.config.speed_scales) - 1)
        if self._rising(state, 'speed_down'):
            self.speed_step = max(self.speed_step - 1, 0)
        self._state = state
        self.source = getattr(state, 'source', '')
        self._last_input = now
        return events

    def connected(self, now):
        """Return False if no input arrived within ``input_timeout`` (2.1.3)."""
        return (self._last_input is not None
                and now - self._last_input <= self.config.input_timeout)

    def tick(self, now):
        """Return ``(linear_x, angular_z)`` to publish this cycle, or ``None`` to stay silent."""
        cfg = self.config
        if not self.connected(now):
            self._was_driving = False   # stop publishing; the watchdog stops the motors (2.3.3)
            return None
        if not self._held(self._state, 'deadman'):
            if self._was_driving:
                self._was_driving = False
                return 0.0, 0.0         # one zero on release (2.3.2)
            return None
        self._was_driving = True
        scale = cfg.speed_scales[self.speed_step]
        return (apply_deadzone(self._state.linear, cfg.deadzone) * cfg.max_linear * scale,
                apply_deadzone(self._state.angular, cfg.deadzone) * cfg.max_angular * scale)
