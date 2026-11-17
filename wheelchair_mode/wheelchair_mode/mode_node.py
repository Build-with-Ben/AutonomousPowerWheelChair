"""
Mode Manager node (domain 3).

Wraps the tested ModeManager. Owns the system mode, serves ``/mode/request`` (3.2.1),
forces Fault or E-stop from ``/safety/state`` (3.2.3), and publishes ``/mode/state`` at
1 Hz and immediately on every change (3.1.2). Starts in Idle and refuses to leave it until
Safety reports no fault (3.1.3, 3.2.2). Runs on the default single-threaded executor.
"""

import rclpy
from rclpy.node import Node
from wheelchair_mode.mode_logic import ModeManager
from wheelchair_msgs.msg import ModeState, SafetyState
from wheelchair_msgs.srv import RequestMode


class ModeNode(Node):
    """Owns the overall system state and which command sources may drive."""

    def __init__(self):
        super().__init__('wheelchair_mode')
        self.declare_parameter('auto_enabled', False)   # 3.2.6: stays off until 1.1.3 lifts
        self.declare_parameter('state_rate_hz', 1.0)

        self._manager = ModeManager(
            auto_enabled=self.get_parameter('auto_enabled').value)
        self._logged_changes = 0
        self._last_reason = 'startup'

        self._state_pub = self.create_publisher(ModeState, 'mode/state', 10)
        self.create_subscription(SafetyState, 'safety/state', self._on_safety_state, 10)
        self.create_service(RequestMode, 'mode/request', self._on_request)

        rate = self.get_parameter('state_rate_hz').value
        self.create_timer(1.0 / rate, self._publish_state)
        self.get_logger().info(
            f"wheelchair_mode up in '{self._manager.mode}'; "
            f'publishing /mode/state at {rate:.0f} Hz.')
        self._publish_state()

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_safety_state(self, msg):
        change = self._manager.on_safety_state(list(msg.faults), self._now())
        self._after_possible_change(change)

    def _on_request(self, request, response):
        accepted, reason = self._manager.request(request.mode, self._now())
        response.success = accepted
        response.message = reason
        response.current_mode = self._manager.mode
        self.get_logger().info(
            f"/mode/request '{request.mode}': "
            f"{'accepted' if accepted else 'refused'} ({reason})")
        if accepted and self._manager.history \
                and self._manager.history[-1].reason == 'requested':
            self._after_possible_change(self._manager.history[-1])
        return response

    def _after_possible_change(self, change):
        """Publish immediately and log when the mode actually changed (3.1.2, 3.3.2)."""
        self._log_new_changes()
        if change is not None:
            self._last_reason = change.reason
            self._publish_state()

    def _publish_state(self):
        msg = ModeState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.mode = self._manager.mode
        msg.last_change_reason = self._last_reason
        self._state_pub.publish(msg)

    def _log_new_changes(self):
        """Log each mode change with old, new and reason (3.3.2)."""
        history = self._manager.history
        for change in history[self._logged_changes:]:
            self.get_logger().info(
                f'mode {change.old} -> {change.new} ({change.reason})')
        self._logged_changes = len(history)


def main(args=None):
    rclpy.init(args=args)
    node = ModeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
