"""
Test sequence profiles and runner (domain 8).

Pure logic, no rclpy. The node calls ``tick`` from its 20 Hz timer and publishes the
result on ``/cmd_vel/test``. Every speed and time is a parameter (8.1.5).
"""

from dataclasses import dataclass, field

PASSED = 'passed'
ABORTED = 'aborted'
FAILED = 'failed'


@dataclass(frozen=True)
class Step:
    """One step: move linearly from the previous target to this one over ``duration``."""

    name: str
    duration: float
    linear: float = 0.0
    angular: float = 0.0


def forward_backward(speed=0.1, ramp=1.0, hold=3.0, pause=1.0):
    """Return the forward/backward test steps (8.2.1)."""
    return [
        Step('ramp_forward', ramp, linear=speed),           # 8.2.1.1
        Step('hold_forward', hold, linear=speed),           # 8.2.1.2
        Step('ramp_to_zero', ramp),                         # 8.2.1.3
        Step('pause', pause),
        Step('ramp_backward', ramp, linear=-speed),         # 8.2.1.4
        Step('hold_backward', hold, linear=-speed),         # 8.2.1.5
        Step('ramp_to_stop', ramp),                         # 8.2.1.6
    ]


def spin_in_place(rate=0.3, ramp=1.0, hold=2.0, pause=1.0):
    """Return spin-left then spin-right steps for checking turn direction (8.4.1)."""
    return [
        Step('ramp_left', ramp, angular=rate),
        Step('hold_left', hold, angular=rate),
        Step('ramp_to_zero', ramp),
        Step('pause', pause),
        Step('ramp_right', ramp, angular=-rate),
        Step('hold_right', hold, angular=-rate),
        Step('ramp_to_stop', ramp),
    ]


class Profile:
    """Turns a list of steps into a velocity at any time since the start."""

    def __init__(self, steps):
        self.steps = list(steps)
        self.duration = sum(s.duration for s in self.steps)

    def at(self, t):
        """Return ``(step_index, linear, angular)`` at time ``t``, or ``None`` once done."""
        start = 0.0
        prev = (0.0, 0.0)
        for i, step in enumerate(self.steps):
            if t < start + step.duration:
                frac = (t - start) / step.duration
                return (i,
                        prev[0] + (step.linear - prev[0]) * frac,
                        prev[1] + (step.angular - prev[1]) * frac)
            start += step.duration
            prev = (step.linear, step.angular)
        return None


@dataclass
class Summary:
    """End-of-sequence report (8.2.3)."""

    result: str = None
    abort_reason: str = None
    peak_current: dict = field(default_factory=dict)
    joystick_frames: int = 0


class SequenceRunner:
    """Runs one profile, with start checks, stop and abort (8.1)."""

    def __init__(self, steps):
        self.profile = Profile(steps)
        self.running = False
        self.summary = None
        self.step_log = []
        self._start = None
        self._step = None

    def start(self, now, mode, safety_ok):
        """Handle ``/test/start``. Return ``(success, message)`` (8.1.1)."""
        if self.running:
            return False, 'a sequence is already running'
        if mode != 'test':
            return False, f'mode is {mode}, not test'
        if not safety_ok:
            return False, 'safety state is not ok'
        self.running = True
        self.summary = Summary()
        self.step_log = []
        self._start = now
        self._step = None
        return True, 'started'

    def _finish(self, result, reason=None):
        self.running = False
        self.summary.result = result
        self.summary.abort_reason = reason

    def stop(self, now):
        """Handle ``/test/stop``: end now and return a zero command (8.1.2)."""
        if self.running:
            self._finish(ABORTED, 'stopped by /test/stop')
        return 0.0, 0.0

    def on_safety_state(self, ok, faults=()):
        """Abort if Safety reports a fault (8.1.3)."""
        if self.running and not ok:
            self._finish(ABORTED, 'safety fault: ' + ', '.join(faults))

    def on_mode(self, mode):
        """Abort if the mode leaves Test (8.1.3)."""
        if self.running and mode != 'test':
            self._finish(ABORTED, f'mode changed to {mode}')

    def fail(self, reason):
        """Finish as failed, e.g. when a measured check doesn't meet its limit."""
        if self.running:
            self._finish(FAILED, reason)

    def record_current(self, amps):
        """Track peak motor current for the current step (8.2.3.3)."""
        if self.running and self._step is not None:
            name = self.profile.steps[self._step].name
            self.summary.peak_current[name] = max(amps, self.summary.peak_current.get(name, amps))

    def record_joystick_frame(self, count=1):
        """Count joystick frames sent while the sequence runs (8.2.3.4)."""
        if self.running:
            self.summary.joystick_frames += count

    def tick(self, now):
        """Return ``(linear, angular)`` to publish, or ``None`` when not running."""
        if not self.running:
            return None
        sample = self.profile.at(now - self._start)
        if sample is None:
            self._finish(PASSED)
            return 0.0, 0.0
        index, linear, angular = sample
        if index != self._step:
            self._step = index
            self.step_log.append((now, self.profile.steps[index].name))   # 8.2.2
        return linear, angular
