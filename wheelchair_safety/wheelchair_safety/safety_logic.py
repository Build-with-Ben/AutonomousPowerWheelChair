"""
Safety gate logic: watchdogs, fault latching, e-stop and reset (domain 5).

Pure logic, no rclpy. The node feeds inputs in as they arrive and calls ``output`` from
its 50 Hz timer and ``state`` from its 10 Hz timer. Every method that depends on time
takes ``now`` in seconds.
"""

from dataclasses import dataclass

STARTUP = 'startup'
PM_HEARTBEAT_LOST = 'pm_heartbeat_lost'
OVER_CURRENT = 'over_current'
PM_FAULT = 'pm_fault'
BRIDGE_STATUS_LOST = 'bridge_status_lost'
CAN_TX_FAILURE = 'can_tx_failure'
ESTOP = 'estop'

LOW_BATTERY = 'low_battery'
COMMAND_STALE = 'command_stale'


@dataclass(frozen=True)
class SafetyConfig:
    """Thresholds and timeouts. Every field is a ROS parameter."""

    cmd_timeout: float = 0.2            # s (5.2.1)
    heartbeat_timeout: float = 2.5      # s (5.3.1)
    current_limit: float = None         # A, TBD (5.3.2); None disables the check
    current_time: float = 2.0           # s (5.3.2)
    status_timeout: float = 1.0         # s (5.3.4)
    battery_warn_pct: float = 20.0      # (5.5.1)
    battery_limit_pct: float = 10.0     # (5.5.2)
    battery_speed_scale: float = 0.25   # (5.5.2)
    max_linear: float = 0.5             # m/s, normal limit that 5.5.2 scales
    max_angular: float = 1.0            # rad/s


@dataclass(frozen=True)
class SafetyStatus:
    """Content of ``wheelchair_msgs/SafetyState`` (5.6.1)."""

    ok: bool
    faults: tuple
    latched: bool
    warnings: tuple


@dataclass(frozen=True)
class FaultEvent:
    """One logged fault being raised or cleared (5.6.3)."""

    time: float
    action: str
    fault: str
    detail: str


class SafetyMonitor:
    """Tracks every stop condition and decides what goes to ``/cmd_vel/safe``."""

    def __init__(self, config=None, now=0.0):
        self.config = config or SafetyConfig()
        self.events = []
        self._latched = {}
        self._cmd = (0.0, 0.0)
        self._cmd_time = None
        self._heartbeat_time = now
        self._status_time = now
        self._current = None
        self._over_current_since = None
        self._battery = None
        self._pm_fault = False
        self._can_tx_failure = False
        self._estop = False
        self._hw_estop_open = False
        self._latch(STARTUP, now, 'safety node started')     # 5.7.2

    # --- inputs ---------------------------------------------------------------------------

    def on_cmd(self, linear_x, angular_z, now):
        """Handle ``/cmd_vel/mux``."""
        self._cmd = (linear_x, angular_z)
        self._cmd_time = now

    def on_status(self, now, pm_heartbeat_time=None, motor_current_a=None,
                  battery_pct=None, pm_fault=False, can_tx_failure=False):
        """Handle ``/rnet/status``. ``pm_heartbeat_time`` is when the PM was last seen."""
        self._status_time = now
        if pm_heartbeat_time is not None:
            self._heartbeat_time = max(self._heartbeat_time, pm_heartbeat_time)
        if motor_current_a is not None:
            self._current = motor_current_a
        if battery_pct is not None:
            self._battery = battery_pct
        self._pm_fault = pm_fault
        self._can_tx_failure = can_tx_failure
        self._evaluate(now)

    def on_estop(self, active, now):
        """Handle ``/safety/estop`` (5.4.1)."""
        self._estop = bool(active)
        self._evaluate(now)

    def on_hw_estop(self, circuit_open, now):
        """Handle the hardware e-stop circuit state (5.4.2)."""
        self._hw_estop_open = bool(circuit_open)
        self._evaluate(now)

    # --- evaluation -----------------------------------------------------------------------

    def _latch(self, fault, now, detail):
        if fault not in self._latched:
            self._latched[fault] = detail
            self.events.append(FaultEvent(now, 'raised', fault, detail))

    def _causes(self, now):
        """Return ``{fault: detail}`` for every cause present right now."""
        cfg = self.config
        causes = {}
        age = now - self._heartbeat_time
        if age >= cfg.heartbeat_timeout:
            causes[PM_HEARTBEAT_LOST] = f'no PM heartbeat for {age:.2f} s'
        if (cfg.current_limit is not None and self._current is not None
                and self._current > cfg.current_limit):
            if self._over_current_since is None:
                self._over_current_since = now
            if now - self._over_current_since >= cfg.current_time:
                causes[OVER_CURRENT] = f'{self._current:.1f} A > {cfg.current_limit:.1f} A'
        else:
            self._over_current_since = None
        if self._pm_fault:
            causes[PM_FAULT] = 'power module reports error or trip'
        age = now - self._status_time
        if age >= cfg.status_timeout:
            causes[BRIDGE_STATUS_LOST] = f'no /rnet/status for {age:.2f} s'
        if self._can_tx_failure:
            causes[CAN_TX_FAILURE] = 'bridge reports CAN TX failure'
        if self._estop or self._hw_estop_open:
            causes[ESTOP] = 'software e-stop' if self._estop else 'hardware e-stop open'
        return causes

    def _cause_present(self, fault, causes):
        if fault == OVER_CURRENT:
            limit = self.config.current_limit
            return self._current is not None and limit is not None and self._current > limit
        return fault in causes

    def _evaluate(self, now):
        causes = self._causes(now)
        for fault, detail in causes.items():
            self._latch(fault, now, detail)
        return causes

    def _cmd_stale(self, now):
        return self._cmd_time is None or now - self._cmd_time > self.config.cmd_timeout

    def warnings(self, now):
        """Return active warnings. These never stop the chair on their own."""
        w = []
        if self._battery is not None and self._battery < self.config.battery_warn_pct:
            w.append(LOW_BATTERY)
        if self._cmd_stale(now):
            w.append(COMMAND_STALE)
        return tuple(w)

    # --- outputs --------------------------------------------------------------------------

    def output(self, now):
        """Return the ``(linear_x, angular_z)`` to publish on ``/cmd_vel/safe``."""
        self._evaluate(now)
        if self._latched or self._cmd_stale(now):
            return 0.0, 0.0
        linear_x, angular_z = self._cmd
        cfg = self.config
        if self._battery is not None and self._battery < cfg.battery_limit_pct:
            lin_max = cfg.max_linear * cfg.battery_speed_scale
            ang_max = cfg.max_angular * cfg.battery_speed_scale
            linear_x = max(-lin_max, min(lin_max, linear_x))
            angular_z = max(-ang_max, min(ang_max, angular_z))
        return linear_x, angular_z

    def state(self, now):
        """Return the ``SafetyState`` content (5.6.1)."""
        self._evaluate(now)
        faults = tuple(sorted(self._latched))
        return SafetyStatus(ok=not faults, faults=faults, latched=bool(faults),
                            warnings=self.warnings(now))

    def reset(self, now):
        """Handle ``/safety/reset``. Return ``(success, message)`` (5.6.2, 5.4.3)."""
        causes = self._evaluate(now)
        still = sorted(f for f in self._latched if self._cause_present(f, causes))
        for fault in [f for f in self._latched if f not in still]:
            del self._latched[fault]
            self.events.append(FaultEvent(now, 'cleared', fault, 'reset'))
        if still:
            reasons = '; '.join(f'{f}: {self._latched[f]}' for f in still)
            return False, f'cause still present: {reasons}'
        return True, 'all faults cleared'
