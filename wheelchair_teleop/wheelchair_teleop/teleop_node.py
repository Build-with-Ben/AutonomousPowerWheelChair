"""
Teleop node (domain 2).

Wraps the tested :class:`Teleop`. Consumes device-independent ``/operator/input`` from
``wheelchair_input`` and publishes ``/cmd_vel/teleop`` at 20 Hz while the deadman is held
(2.2.2), one zero command when it is released (2.3.2), and nothing at all once input goes
stale (2.1.3, 2.3.3). Raises the software e-stop on ``/safety/estop`` (2.3.4) and requests
Manual mode through ``/mode/request`` (2.3.5).

No axis or button index appears here or in ``teleop_logic``: the device profile in
``wheelchair_input`` owns all of that (2.1.2). Runs on the default single-threaded
executor.
"""

from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool
from wheelchair_msgs.msg import OperatorInput
from wheelchair_msgs.srv import RequestMode
from wheelchair_teleop.teleop_logic import Teleop, TeleopConfig

MANUAL = 'manual'


class TeleopNode(Node):
    """Turns operator input into the highest-priority command source (2.4.1)."""

    def __init__(self):
        super().__init__('wheelchair_teleop')
        self.declare_parameter('deadzone', 0.1)
        self.declare_parameter('max_linear', 0.5)
        self.declare_parameter('max_angular', 1.0)
        self.declare_parameter('input_timeout', 0.5)
        self.declare_parameter('speed_scales', [0.25, 0.5, 1.0])
        self.declare_parameter('output_rate_hz', 20.0)
        # Device 'source' names to accept; empty accepts whichever device publishes.
        self.declare_parameter('sources', [''])
        self.declare_parameter('request_manual_mode', True)

        sources = tuple(s for s in self.get_parameter('sources').value if s)
        self._teleop = Teleop(TeleopConfig(
            deadzone=self.get_parameter('deadzone').value,
            max_linear=self.get_parameter('max_linear').value,
            max_angular=self.get_parameter('max_angular').value,
            input_timeout=self.get_parameter('input_timeout').value,
            speed_scales=tuple(self.get_parameter('speed_scales').value),
            sources=sources,
        ))
        self._request_mode = self.get_parameter('request_manual_mode').value
        self._driving = False
        self._speed_step = self._teleop.speed_step
        self._last_source = ''

        self._cmd_pub = self.create_publisher(Twist, 'cmd_vel/teleop', 10)
        self._estop_pub = self.create_publisher(Bool, 'safety/estop', 10)
        self.create_subscription(OperatorInput, 'operator/input', self._on_input, 10)
        self._mode_client = self.create_client(RequestMode, 'mode/request')

        rate = self.get_parameter('output_rate_hz').value
        self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info(
            f'wheelchair_teleop up; /operator/input -> /cmd_vel/teleop at {rate:.0f} Hz, '
            f'accepting {", ".join(sources) if sources else "any device"}. '
            f'Hold the deadman to drive.')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_input(self, msg):
        if not self._teleop.accepts(msg.source):
            return
        if msg.source != self._last_source:
            self.get_logger().info(f"operator input from '{msg.source}'.")
            self._last_source = msg.source
        events = self._teleop.on_input(msg, self._now())
        if self._teleop.speed_step != self._speed_step:
            self._speed_step = self._teleop.speed_step
            scale = self._teleop.config.speed_scales[self._speed_step]
            self.get_logger().info(
                f'speed step {self._speed_step} ({scale:.2f} of maximum).')
        if events.estop:
            self.get_logger().warn('operator e-stop pressed; publishing /safety/estop.')
            self._estop_pub.publish(Bool(data=True))
        if events.manual_mode and self._request_mode:
            self._request_manual()

    def _request_manual(self):
        """Ask Mode Management for Manual mode (2.3.5). Fire and forget."""
        if not self._mode_client.service_is_ready():
            self.get_logger().warn('/mode/request is not available; cannot request Manual.')
            return
        future = self._mode_client.call_async(RequestMode.Request(mode=MANUAL))
        future.add_done_callback(self._on_mode_response)

    def _on_mode_response(self, future):
        try:
            response = future.result()
        except Exception as exc:                                  # noqa: BLE001
            self.get_logger().warn(f'/mode/request failed: {exc}')
            return
        level = self.get_logger().info if response.success else self.get_logger().warn
        level(f"Manual mode {'accepted' if response.success else 'refused'}: "
              f'{response.message} (now {response.current_mode})')

    def _publish(self):
        """Publish one command, or nothing, according to the tested logic (2.2.2)."""
        command = self._teleop.tick(self._now())
        if command is None:
            if self._driving:
                self._driving = False
                self.get_logger().info('teleop silent: deadman released or input stale.')
            return
        if not self._driving:
            self._driving = True
            self.get_logger().info('teleop driving: deadman held.')
        linear_x, angular_z = command
        msg = Twist()               # every other field stays zero (2.2.1.3)
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self._cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
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
