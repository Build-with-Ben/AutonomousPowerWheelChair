#!/usr/bin/env python3
"""
Discover a controller's axis and button layout by watching ``/joy``.

Run this once for any new input device, follow the prompts, and paste the reported
indices into a device profile under ``wheelchair_input/config/``. Indices are read from
``/joy`` itself, so they are exactly what the mapper will see.

    ros2 run joy joy_node &
    python3 tools/joy_probe.py

``--segment`` splits the recording into one action per pause, so a scripted run
("move a control, pause, move the next") reports each action separately.
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

AXIS_MOVED = 0.35      # a deliberate axis move, well clear of stick noise and drift
BUTTON_DOWN = 0.5


class Probe(Node):
    """Records which axes and buttons change, grouped into actions by pauses."""

    def __init__(self, gap):
        super().__init__('joy_probe')
        self._gap = gap
        self._rest = None          # axis values at rest, from the first message
        self._last_event = None
        self.actions = []          # list of dicts: {'axes': {i: value}, 'buttons': {i}}
        self._current = None
        self.create_subscription(Joy, 'joy', self._on_joy, 10)

    def _on_joy(self, msg):
        if self._rest is None:
            self._rest = list(msg.axes)
            return
        axes = {}
        for i, value in enumerate(msg.axes):
            rest = self._rest[i] if i < len(self._rest) else 0.0
            if abs(value - rest) >= AXIS_MOVED:
                axes[i] = value - rest
        buttons = {i for i, v in enumerate(msg.buttons) if v > BUTTON_DOWN}
        if not axes and not buttons:
            return

        now = time.time()
        if self._current is None or (self._last_event is not None
                                     and now - self._last_event > self._gap):
            self._current = {'axes': {}, 'buttons': set()}
            self.actions.append(self._current)
        self._last_event = now
        for i, delta in axes.items():
            # Keep the largest deflection seen, with its sign.
            if abs(delta) > abs(self._current['axes'].get(i, 0.0)):
                self._current['axes'][i] = delta
        self._current['buttons'] |= buttons


def describe(action):
    """Render one recorded action as a line of text."""
    parts = []
    for i, delta in sorted(action['axes'].items(), key=lambda kv: -abs(kv[1])):
        parts.append(f'axis {i} {"+" if delta > 0 else "-"} (peak {delta:+.2f})')
    for i in sorted(action['buttons']):
        parts.append(f'button {i}')
    return ', '.join(parts) if parts else '(nothing)'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=60.0,
                        help='how long to record (default 60)')
    parser.add_argument('--gap', type=float, default=0.8,
                        help='pause in seconds that separates two actions (default 0.8)')
    parser.add_argument('--steps', nargs='*', default=None,
                        help='names for the actions, in the order they are performed')
    args = parser.parse_args()

    rclpy.init()
    probe = Probe(args.gap)
    print(f'Recording /joy for {args.seconds:.0f}s. '
          f'Leave every stick centred before you start.', file=sys.stderr)
    end = time.time() + args.seconds
    try:
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(probe, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass

    if probe._rest is None:
        print('No /joy messages arrived. Is joy_node running?', file=sys.stderr)
        return 1

    print(f'\nResting axis values: '
          f'{[round(v, 2) for v in probe._rest]}')
    print(f'{len(probe.actions)} action(s) recorded:\n')
    for n, action in enumerate(probe.actions):
        name = args.steps[n] if args.steps and n < len(args.steps) else f'action {n + 1}'
        print(f'  {name:<22} {describe(action)}')
    probe.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
