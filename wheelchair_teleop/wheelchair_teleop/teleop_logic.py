"""
Gamepad-to-velocity logic for teleop (requirements 2.1-2.3).

Pure logic, no rclpy. The node calls ``on_joy`` for each ``/joy`` message and ``tick``
from its 20 Hz timer, then publishes whatever ``tick`` returns.
"""

from dataclasses import dataclass, field


@dataclass
class TeleopConfig:
    """Button/axis mapping and limits. Every field is a ROS parameter (2.1.2)."""

    linear_axis: int = 1
    angular_axis: int = 0
    deadman_button: int = 4
    estop_button: int = 1
    speed_down_button: int = 6
    speed_up_button: int = 7
    manual_mode_buttons: tuple = (8, 9)
    deadzone: float = 0.1
    max_linear: float = 0.5
    max_angular: float = 1.0
    joy_timeout: float = 0.5
    speed_scales: tuple = (0.25, 0.5, 1.0)


@dataclass
class JoyEvents:
    """One-shot requests raised by a ``/joy`` message (button presses, not holds)."""

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
    """Deadman, disconnect and speed-scale state for one gamepad."""

    config: TeleopConfig = field(default_factory=TeleopConfig)
    speed_step: int = 0
    _axes: tuple = ()
    _buttons: tuple = ()
    _last_joy: float = None
    _was_driving: bool = False

    def _pressed(self, buttons, index):
        return index < len(buttons) and bool(buttons[index])

    def _rising(self, buttons, index):
        return self._pressed(buttons, index) and not self._pressed(self._buttons, index)

    def on_joy(self, axes, buttons, now):
        """Store a ``/joy`` message and return any one-shot requests it makes."""
        cfg = self.config
        combo = cfg.manual_mode_buttons
        events = JoyEvents(
            estop=self._rising(buttons, cfg.estop_button),
            manual_mode=(all(self._pressed(buttons, b) for b in combo)
                         and not all(self._pressed(self._buttons, b) for b in combo)),
        )
        if self._rising(buttons, cfg.speed_up_button):
            self.speed_step = min(self.speed_step + 1, len(cfg.speed_scales) - 1)
        if self._rising(buttons, cfg.speed_down_button):
            self.speed_step = max(self.speed_step - 1, 0)
        self._axes = tuple(axes)
        self._buttons = tuple(buttons)
        self._last_joy = now
        return events

    def connected(self, now):
        """Return False if no ``/joy`` arrived within ``joy_timeout`` (2.1.3)."""
        return self._last_joy is not None and now - self._last_joy <= self.config.joy_timeout

    def _axis(self, index):
        value = self._axes[index] if index < len(self._axes) else 0.0
        return apply_deadzone(value, self.config.deadzone)

    def tick(self, now):
        """Return ``(linear_x, angular_z)`` to publish this cycle, or ``None`` to stay silent."""
        cfg = self.config
        if not self.connected(now):
            self._was_driving = False   # stop publishing; the watchdog stops the motors (2.3.3)
            return None
        if not self._pressed(self._buttons, cfg.deadman_button):
            if self._was_driving:
                self._was_driving = False
                return 0.0, 0.0         # one zero on release (2.3.2)
            return None
        self._was_driving = True
        scale = cfg.speed_scales[self.speed_step]
        return (self._axis(cfg.linear_axis) * cfg.max_linear * scale,
                self._axis(cfg.angular_axis) * cfg.max_angular * scale)
