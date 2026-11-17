"""
System mode state machine (domain 3).

Pure logic, no rclpy. The node feeds it ``/safety/state`` and ``/mode/request`` calls,
and publishes ``mode`` on ``/mode/state``.

The **Startup** fault (5.7.2) blocks leaving Idle but doesn't force Fault mode, so a fresh
boot ends in Idle with the Startup fault latched (9.1.4.3).
"""

from dataclasses import dataclass

IDLE = 'idle'
MANUAL = 'manual'
TEST = 'test'
AUTO = 'auto'
FAULT = 'fault'
ESTOP = 'estop'

MODES = (IDLE, MANUAL, TEST, AUTO, FAULT, ESTOP)
DRIVING_MODES = (MANUAL, TEST, AUTO)

ESTOP_FAULT = 'estop'
STARTUP_FAULT = 'startup'
NO_SAFETY_STATE = 'no safety state received'


@dataclass(frozen=True)
class ModeChange:
    """One logged mode change (3.3.2)."""

    time: float
    old: str
    new: str
    reason: str


class ModeManager:
    """Owns the current mode and decides which transitions are allowed."""

    def __init__(self, auto_enabled=False):
        self.mode = IDLE                     # 3.1.3
        self.auto_enabled = auto_enabled     # 3.2.6: stays False until 1.1.3 is lifted
        self.history = []
        self._faults = (NO_SAFETY_STATE,)    # not ok until Safety reports (1.4.2)

    def _change(self, new, reason, now):
        if new == self.mode:
            return None
        change = ModeChange(now, self.mode, new, reason)
        self.mode = new
        self.history.append(change)
        return change

    def request(self, mode, now):
        """Handle a ``/mode/request`` call. Return ``(accepted, reason)``."""
        if mode not in MODES:
            return False, f'unknown mode {mode!r}'
        if mode == self.mode:
            return True, f'already {mode}'
        if self.mode in (FAULT, ESTOP):
            return False, f'in {self.mode}; clear the cause and call /safety/reset'
        if mode in DRIVING_MODES:
            if self.mode != IDLE:
                return False, f'{mode} can only be entered from idle'
            if self._faults:
                return False, 'safety not ok: ' + ', '.join(self._faults)
            if mode == AUTO and not self.auto_enabled:
                return False, 'auto mode is not available yet'
            self._change(mode, 'requested', now)
            return True, f'entered {mode}'
        if mode == IDLE:
            self._change(IDLE, 'requested', now)
            return True, 'entered idle'
        return False, f'{mode} cannot be requested'

    def on_safety_state(self, faults, now):
        """Handle a ``/safety/state`` message with its list of active faults (3.2.3, 3.2.4)."""
        self._faults = tuple(faults)
        forcing = [f for f in self._faults if f != STARTUP_FAULT]
        if ESTOP_FAULT in forcing:
            return self._change(ESTOP, 'e-stop', now)
        if forcing:
            return self._change(FAULT, 'fault: ' + ', '.join(forcing), now)
        if self.mode in (FAULT, ESTOP):
            # Safety only clears latched faults after a reset, so an empty list here
            # means the cause is gone and the operator reset (3.2.4).
            return self._change(IDLE, 'safety reset', now)
        if self._faults and self.mode in DRIVING_MODES:
            return self._change(IDLE, 'safety not ok', now)
        return None

    def on_sequence_ended(self, now, reason='sequence ended'):
        """Return to Idle when a test sequence finishes or aborts (3.2.5)."""
        if self.mode == TEST:
            return self._change(IDLE, reason, now)
        return None
