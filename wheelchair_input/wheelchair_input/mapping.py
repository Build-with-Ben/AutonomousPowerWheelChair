"""
Raw device message to device-independent operator input (requirement 2.1.2).

Pure logic, no rclpy and no ROS messages: the node hands ``apply`` the ``axes`` and
``buttons`` arrays of one ``/joy`` message and publishes the returned state.

The mapper is stateless. Everything that needs memory — rising edges, speed steps,
deadzones, timeouts — lives downstream in ``wheelchair_teleop``, so a device profile can
be swapped at any time without carrying stale state across the change.
"""

from dataclasses import dataclass, field

from wheelchair_input.profile import DRIVE_AXES, FUNCTIONS, Profile


@dataclass
class OperatorState:
    """One frame of operator input, mirroring ``wheelchair_msgs/OperatorInput``."""

    source: str = ''
    linear: float = 0.0        # -1.0 .. +1.0, positive forward
    angular: float = 0.0       # -1.0 .. +1.0, positive is a left turn
    deadman: bool = False
    estop: bool = False
    speed_up: bool = False
    speed_down: bool = False
    mode_manual: bool = False
    rearm: bool = False

    @property
    def idle(self):
        """Return True when nothing is deflected and no function is held."""
        return (self.linear == 0.0 and self.angular == 0.0
                and not any(getattr(self, f) for f in FUNCTIONS))


@dataclass
class InputMapping:
    """Applies one device profile to raw device messages."""

    profile: Profile = field(default_factory=lambda: Profile(name='unmapped'))

    def apply(self, axes, buttons):
        """Return the :class:`OperatorState` for one raw device message."""
        axes = list(axes)
        buttons = list(buttons)
        state = OperatorState(source=self.profile.name)
        for name in DRIVE_AXES:
            setattr(state, name, float(self.profile.axis(name).value(axes, buttons)))
        for name in FUNCTIONS:
            setattr(state, name, bool(self.profile.function(name).held(axes, buttons)))
        return state
