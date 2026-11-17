"""
Motor Control node (domain 4).

Arbitrates the command sources (teleop > test > auto), blocks any source the current
mode doesn't allow, clamps the chosen command to the speed limits and limits its rate of
change, then publishes ``/cmd_vel/mux`` at a fixed 20 Hz (4.1-4.3).

The current mode comes from ``/mode/state`` (wheelchair_msgs/ModeState). Until a fresh
mode is seen, and whenever the mode goes stale, the node falls back to ``default_mode``
(``idle`` by default), which blocks every source. Runs on the default single-threaded
executor, so the callbacks never overlap.
"""

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from wheelchair_control.limits import (
    AccelLimiter, clamp_command, DEFAULT_PRIORITIES, PROFILES, SourceArbiter)
from wheelchair_msgs.msg import ModeState

SOURCES = ('teleop', 'test', 'auto')


class ControlNode(Node):
    """Command arbitration and shaping for the drive motors."""

    def __init__(self):
        super().__init__('wheelchair_control')
        self.declare_parameter('source_timeout', 0.25)
        self.declare_parameter('output_rate_hz', 20.0)
        self.declare_parameter('limits_profile', 'bringup')
        self.declare_parameter('default_mode', 'idle')
        self.declare_parameter('mode_timeout', 2.0)
        for source in SOURCES:
            self.declare_parameter(f'priority.{source}', DEFAULT_PRIORITIES[source])

        priorities = {s: self.get_parameter(f'priority.{s}').value for s in SOURCES}
        self._arbiter = SourceArbiter(priorities,
                                      timeout=self.get_parameter('source_timeout').value)
        profile = self.get_parameter('limits_profile').value
        if profile not in PROFILES:
            raise ValueError(f'unknown limits_profile {profile!r}')
        self._limits = PROFILES[profile]
        self._accel = AccelLimiter(self._limits)
        self._default_mode = self.get_parameter('default_mode').value
        self._mode_timeout = self.get_parameter('mode_timeout').value
        self._mode = self._default_mode
        self._mode_time = None
        self._active = None

        self._mux_pub = self.create_publisher(Twist, 'cmd_vel/mux', 10)
        for source in SOURCES:
            self.create_subscription(
                Twist, f'cmd_vel/{source}',
                lambda msg, s=source: self._on_cmd(s, msg), 10)
        self.create_subscription(ModeState, 'mode/state', self._on_mode, 10)

        rate = self.get_parameter('output_rate_hz').value
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f"wheelchair_control up: '{profile}' limits, arbitrating at {rate:.0f} Hz.")

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_cmd(self, source, msg):
        self._arbiter.set_command(source, msg.linear.x, msg.angular.z, self._now())

    def _on_mode(self, msg):
        self._mode = msg.mode
        self._mode_time = self._now()

    def _current_mode(self, now):
        """Return the mode to enforce, falling back to default_mode when stale (4.1.3)."""
        if self._mode_time is None or now - self._mode_time > self._mode_timeout:
            return self._default_mode
        return self._mode

    def _tick(self):
        now = self._now()
        mode = self._current_mode(now)
        source, linear_x, angular_z = self._arbiter.select(mode, now)
        if source != self._active:
            if source is None:
                self.get_logger().info(f'no active source in mode {mode!r}; commanding zero')
            else:
                self.get_logger().info(f'active source: {source} (mode {mode!r})')
            self._active = source

        clamped = clamp_command(linear_x, angular_z, self._limits)
        out_linear, out_angular = self._accel.step(clamped[0], clamped[1], now)
        msg = Twist()
        msg.linear.x = out_linear
        msg.angular.z = out_angular
        self._mux_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
