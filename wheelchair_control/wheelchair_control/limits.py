"""
Speed limits, acceleration limits and mode gating for Motor Control (domain 4).

Pure logic, no rclpy. Command arbitration itself (4.1.1, 4.1.2) is done by ``twist_mux``
and is tested at L2. This module shapes the selected command and blocks sources the
current mode doesn't allow.
"""

from dataclasses import dataclass, replace


BRINGUP_SCALE = 0.2

# Sources allowed to drive in each mode (3.1.1). Mode names match /mode/state.
MODE_SOURCES = {
    'idle': frozenset(),
    'manual': frozenset({'teleop'}),
    'test': frozenset({'test', 'teleop'}),
    'auto': frozenset({'auto', 'teleop'}),
    'fault': frozenset(),
    'estop': frozenset(),
}


@dataclass(frozen=True)
class Limits:
    """Speed and acceleration limits. Every field is a ROS parameter."""

    max_linear: float = 0.5        # m/s (4.2.1)
    max_angular: float = 1.0       # rad/s (4.2.1)
    reverse_ratio: float = 0.5     # of max_linear (4.2.2)
    accel_linear: float = 0.5      # m/s^2 (4.3.1)
    accel_angular: float = 1.5     # rad/s^2 (4.3.2)
    decel_linear: float = 1.0      # m/s^2, toward zero (4.3.3)


NORMAL = Limits()
BRINGUP = replace(NORMAL, max_linear=NORMAL.max_linear * BRINGUP_SCALE,
                  max_angular=NORMAL.max_angular * BRINGUP_SCALE)
PROFILES = {'normal': NORMAL, 'bringup': BRINGUP}
DEFAULT_PROFILE = 'bringup'   # until the 8.3 wheels-on-ground tests pass (4.2.3)


def source_allowed(mode, source):
    """Return True if ``source`` may drive in ``mode`` (3.3.1, 4.1.3). Unknown mode: no."""
    return source in MODE_SOURCES.get(mode, frozenset())


def clamp_command(linear_x, angular_z, limits=NORMAL):
    """Clamp a command to the speed limits, with a lower reverse limit (4.2.1, 4.2.2)."""
    reverse = limits.max_linear * limits.reverse_ratio
    linear_x = max(-reverse, min(limits.max_linear, linear_x))
    angular_z = max(-limits.max_angular, min(limits.max_angular, angular_z))
    return linear_x, angular_z


def _slew(current, target, accel, decel, dt):
    """Move ``current`` toward ``target`` by at most the allowed change in ``dt``."""
    if target == current:
        return target
    if current != 0 and (target * current <= 0 or abs(target) < abs(current)):
        # Slowing down: use the decel limit until zero is reached.
        stop = target if target * current > 0 else 0.0
        step = decel * dt
        if abs(current - stop) > step:
            return current - step if current > 0 else current + step
        dt -= abs(current - stop) / decel
        current = stop
        if current == target or dt <= 0:
            return current
    step = accel * dt
    if abs(target - current) <= step:
        return target
    return current + step if target > current else current - step


class AccelLimiter:
    """Limits how fast the output command may change (4.3)."""

    def __init__(self, limits=NORMAL):
        self.limits = limits
        self.output = (0.0, 0.0)
        self._last = None

    def step(self, linear_x, angular_z, now):
        """Return the limited command for a new target at time ``now``."""
        dt = 0.0 if self._last is None else max(0.0, now - self._last)
        self._last = now
        lim = self.limits
        lin, ang = self.output
        self.output = (
            _slew(lin, linear_x, lim.accel_linear, lim.decel_linear, dt),
            _slew(ang, angular_z, lim.accel_angular, lim.accel_angular, dt),
        )
        return self.output
