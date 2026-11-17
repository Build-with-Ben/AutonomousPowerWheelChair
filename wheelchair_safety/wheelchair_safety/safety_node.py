"""
Safety gate node (domain 5).

Wraps the tested SafetyMonitor. Copies ``/cmd_vel/mux`` to ``/cmd_vel/safe`` while every
check passes, zeroes it the moment any check fails (5.1), publishes ``/safety/state`` and
serves ``/safety/reset``. It starts with a latched Startup fault, so an operator reset is
needed before the first drive (5.7.2).

Runs on the default single-threaded executor, so the subscriptions, timers and service
callback never overlap and the monitor needs no locking.
"""

from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from wheelchair_msgs.msg import RnetStatus, SafetyState

from wheelchair_safety.safety_logic import SafetyConfig, SafetyMonitor


def _stamp_to_sec(stamp):
    """Convert a builtin_interfaces/Time to float seconds, or None if it is zero."""
    sec = stamp.sec + stamp.nanosec * 1e-9
    return sec if sec > 0.0 else None


class SafetyNode(Node):
    """The last software gate before the motors."""

    def __init__(self):
        super().__init__('wheelchair_safety')
        p = self._declare_params()
        current_limit = p['current_limit']
        config = SafetyConfig(
            cmd_timeout=p['cmd_timeout'],
            heartbeat_timeout=p['heartbeat_timeout'],
            current_limit=current_limit if current_limit >= 0.0 else None,
            current_time=p['current_time'],
            status_timeout=p['status_timeout'],
            battery_warn_pct=p['battery_warn_pct'],
            battery_limit_pct=p['battery_limit_pct'],
            battery_speed_scale=p['battery_speed_scale'],
            max_linear=p['max_linear'],
            max_angular=p['max_angular'],
        )
        self._monitor = SafetyMonitor(config, now=self._now())
        self._logged_events = 0
        self._last_key = None

        self._safe_pub = self.create_publisher(Twist, 'cmd_vel/safe', 10)
        self._state_pub = self.create_publisher(SafetyState, 'safety/state', 10)
        self.create_subscription(Twist, 'cmd_vel/mux', self._on_cmd, 10)
        self.create_subscription(RnetStatus, 'rnet/status', self._on_status, 10)
        self.create_subscription(Bool, 'safety/estop', self._on_estop, 10)
        self.create_service(Trigger, 'safety/reset', self._on_reset)

        self.create_timer(1.0 / p['output_rate_hz'], self._publish_safe)
        self.create_timer(1.0 / p['state_rate_hz'], self._publish_state)
        self.get_logger().warning(
            'wheelchair_safety up with a latched Startup fault; call /safety/reset '
            'before driving.')

    # --- setup helpers --------------------------------------------------------------------

    def _declare_params(self):
        defaults = {
            'cmd_timeout': 0.2, 'heartbeat_timeout': 2.5, 'current_limit': -1.0,
            'current_time': 2.0, 'status_timeout': 1.0, 'battery_warn_pct': 20.0,
            'battery_limit_pct': 10.0, 'battery_speed_scale': 0.25, 'max_linear': 0.5,
            'max_angular': 1.0, 'output_rate_hz': 50.0, 'state_rate_hz': 10.0,
        }
        return {name: self.declare_parameter(name, value).value
                for name, value in defaults.items()}

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # --- inputs ---------------------------------------------------------------------------

    def _on_cmd(self, msg):
        self._monitor.on_cmd(msg.linear.x, msg.angular.z, self._now())

    def _on_status(self, msg):
        self._monitor.on_status(
            self._now(),
            pm_heartbeat_time=_stamp_to_sec(msg.pm_last_seen) if msg.pm_alive else None,
            motor_current_a=msg.motor_current_a,
            battery_pct=float(msg.battery_pct),
        )
        self._publish_state_on_change()

    def _on_estop(self, msg):
        self._monitor.on_estop(msg.data, self._now())
        self._publish_state_on_change()

    def _on_reset(self, request, response):
        ok, message = self._monitor.reset(self._now())
        response.success = ok
        response.message = message
        self.get_logger().info(f'/safety/reset: {message}')
        self._publish_state_on_change()
        return response

    # --- outputs --------------------------------------------------------------------------

    def _publish_safe(self):
        linear_x, angular_z = self._monitor.output(self._now())
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self._safe_pub.publish(msg)

    def _build_state(self, status):
        msg = SafetyState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.ok = status.ok
        msg.faults = list(status.faults)
        msg.latched = status.latched
        msg.warnings = list(status.warnings)
        return msg

    def _publish_state(self):
        status = self._monitor.state(self._now())
        self._state_pub.publish(self._build_state(status))
        self._last_key = (status.ok, status.faults, status.latched)
        self._log_new_events()

    def _publish_state_on_change(self):
        """Publish immediately when ok/faults/latched change (5.6.1)."""
        status = self._monitor.state(self._now())
        key = (status.ok, status.faults, status.latched)
        if key != self._last_key:
            self._state_pub.publish(self._build_state(status))
            self._last_key = key
        self._log_new_events()

    def _log_new_events(self):
        """Log faults raised or cleared since last time (5.6.3)."""
        events = self._monitor.events
        for event in events[self._logged_events:]:
            text = f'fault {event.action}: {event.fault} ({event.detail})'
            if event.action == 'raised':
                self.get_logger().warning(text)
            else:
                self.get_logger().info(text)
        self._logged_events = len(events)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
