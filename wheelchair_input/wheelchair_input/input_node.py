"""
Operator input node (domain 2.1).

Turns one raw controller's ``/joy`` messages into device-independent
``/operator/input`` using a device profile (requirement 2.1.2), so that
``wheelchair_teleop`` never sees an axis or button index.

The node is a pass-through: one ``/joy`` message in, one ``/operator/input`` message out.
It deliberately does not republish on a timer, because silence is how a disconnected
controller reaches teleop's timeout (2.1.3, 2.3.3). Two consequences:

* ``joy_node`` must run with ``autorepeat_rate`` at or above teleop's publish rate,
  otherwise a held-still stick produces no messages and teleop times out while the
  operator is still driving. The launch file sets this.
* Nothing here latches or smooths. Deadzone, speed scaling, rising edges and the
  deadman all stay in teleop, which is where they are specified and tested.

Running several controllers at once is supported by starting one node per device, each
with its own ``profile`` and node name, all publishing to ``/operator/input``. The
``source`` field says which device a message came from, and ``enabled`` can be set at
runtime to mute a device without stopping its node:

    ros2 param set /operator_input_xbox enabled false
"""

import os

from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Joy
from wheelchair_input import profile as profiles
from wheelchair_input.mapping import InputMapping
from wheelchair_msgs.msg import OperatorInput


def find_profile(name, path=''):
    """Load a profile by explicit ``path``, else by ``name`` from this package's config."""
    if path:
        return profiles.load(path)
    share = os.path.join(get_package_share_directory('wheelchair_input'), 'config')
    return profiles.load(os.path.join(share, f'{name}.yaml'))


class InputNode(Node):
    """Maps one controller to operator input through a swappable device profile."""

    def __init__(self):
        super().__init__('wheelchair_input')
        self.declare_parameter('profile', 'xbox_generic')
        self.declare_parameter('profile_path', '')   # overrides 'profile' when set
        self.declare_parameter('enabled', True)      # mute this device without stopping it
        self.declare_parameter('joy_timeout', 0.5)   # warn-only; teleop owns the stop (2.1.3)

        self._mapping = InputMapping(
            find_profile(self.get_parameter('profile').value,
                         self.get_parameter('profile_path').value))
        self._enabled = self.get_parameter('enabled').value
        self._timeout = self.get_parameter('joy_timeout').value
        self._last_joy = None
        self._warned_silent = False

        self._pub = self.create_publisher(OperatorInput, 'operator/input', 10)
        self.create_subscription(Joy, 'joy', self._on_joy, 10)
        self.create_timer(self._timeout, self._check_silence)
        self.add_on_set_parameters_callback(self._on_set_parameters)

        profile = self._mapping.profile
        self.get_logger().info(
            f"wheelchair_input up with profile '{profile.name}'"
            f"{' (disabled)' if not self._enabled else ''}: "
            f'/joy -> /operator/input.')
        if profile.description:
            self.get_logger().info(f'profile: {profile.description.strip()}')
        if profile.unmapped:
            self.get_logger().warn(
                f"profile '{profile.name}' leaves "
                f"{', '.join(profile.unmapped)} unmapped; "
                f'those controls are unavailable.')
        if 'deadman' in profile.unmapped:
            self.get_logger().error(
                f"profile '{profile.name}' has no deadman control, so teleop will never "
                f'command motion (2.3.1). Map one before driving.')

    def _on_set_parameters(self, params):
        """Accept a runtime change to ``enabled``, so a device can be muted live."""
        for param in params:
            if param.name == 'enabled' and param.type_ == Parameter.Type.BOOL:
                self._enabled = param.value
                self.get_logger().info(
                    f"profile '{self._mapping.profile.name}' "
                    f"{'enabled' if param.value else 'disabled'}.")
        return SetParametersResult(successful=True)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_joy(self, msg):
        self._last_joy = self._now()
        if self._warned_silent:
            self.get_logger().info(
                f"'{self._mapping.profile.name}' is publishing again.")
            self._warned_silent = False
        if not self._enabled:
            return
        state = self._mapping.apply(msg.axes, msg.buttons)
        out = OperatorInput()
        out.header.stamp = msg.header.stamp    # keep the device's own timestamp
        out.source = state.source
        out.linear = state.linear
        out.angular = state.angular
        out.deadman = state.deadman
        out.estop = state.estop
        out.speed_up = state.speed_up
        out.speed_down = state.speed_down
        out.mode_manual = state.mode_manual
        self._pub.publish(out)

    def _check_silence(self):
        """Log once when ``/joy`` goes quiet. Teleop's timeout is what stops the chair."""
        if self._warned_silent or self._last_joy is None:
            return
        if self._now() - self._last_joy > self._timeout:
            self._warned_silent = True
            self.get_logger().warn(
                f"no /joy for {self._timeout:.1f}s; '{self._mapping.profile.name}' may be "
                f'disconnected. Teleop will stop publishing (2.3.3).')


def main(args=None):
    rclpy.init(args=args)
    node = InputNode()
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
