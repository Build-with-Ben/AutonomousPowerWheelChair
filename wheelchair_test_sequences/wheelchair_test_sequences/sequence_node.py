"""
Test sequence node (domain 8).

Wraps the tested SequenceRunner. Runs a scripted motion (forward/back by default) and
publishes ``/cmd_vel/test`` at 20 Hz while running (8.1.4). ``/test/start`` begins a run,
but only while the system is in Test mode and ``/safety/state.ok`` is true (8.1.1);
``/test/stop`` ends it with a zero command (8.1.2). A safety fault or the mode leaving
Test aborts the run (8.1.3). When a run ends for any reason the node commands zero, logs a
summary (8.2.3), and requests Idle from the Mode Manager (3.2.5).

Peak motor current per step (8.2.3.3) comes from ``/rnet/status``; the joystick frame
count (8.2.3.4) is taken from ``/to_can_bus``. Runs on the default single-threaded
executor, so the callbacks never overlap.
"""

from can_msgs.msg import Frame
from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_srvs.srv import Trigger
from wheelchair_msgs.msg import ModeState, RnetStatus, SafetyState
from wheelchair_msgs.srv import RequestMode

from wheelchair_test_sequences.sequence import (
    forward_backward, SequenceRunner, spin_in_place)

JOYSTICK_BASE = 0x02000000
TEST_MODE = 'test'


class SequenceNode(Node):
    """Drives a scripted test sequence through the normal command path."""

    def __init__(self):
        super().__init__('wheelchair_test_sequences')
        self.declare_parameter('output_rate_hz', 20.0)
        self.declare_parameter('sequence', 'forward_back')
        self.declare_parameter('speed', 0.1)
        self.declare_parameter('ramp', 1.0)
        self.declare_parameter('hold', 3.0)
        self.declare_parameter('pause', 1.0)
        self.declare_parameter('spin_rate', 0.3)
        self.declare_parameter('joystick_nibble', 2)
        self.declare_parameter('return_to_idle', True)

        self._runner = SequenceRunner(self._build_steps())
        self._joystick_id = JOYSTICK_BASE | (self.get_parameter('joystick_nibble').value << 8)
        self._return_to_idle = self.get_parameter('return_to_idle').value
        self._mode = 'idle'
        self._safety_ok = False
        self._was_running = False
        self._logged_steps = 0

        self._test_pub = self.create_publisher(Twist, 'cmd_vel/test', 10)
        self.create_subscription(ModeState, 'mode/state', self._on_mode, 10)
        self.create_subscription(SafetyState, 'safety/state', self._on_safety, 10)
        self.create_subscription(RnetStatus, 'rnet/status', self._on_status, 10)
        self.create_subscription(Frame, 'to_can_bus', self._on_tx_frame, 50)
        self.create_service(Trigger, 'test/start', self._on_start)
        self.create_service(Trigger, 'test/stop', self._on_stop)
        self._mode_client = self.create_client(RequestMode, 'mode/request')

        rate = self.get_parameter('output_rate_hz').value
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f"wheelchair_test_sequences ready: '{self.get_parameter('sequence').value}' "
            f'sequence, publishing /cmd_vel/test at {rate:.0f} Hz when running.')

    def _build_steps(self):
        p = self.get_parameter
        if p('sequence').value == 'spin':
            return spin_in_place(rate=p('spin_rate').value, ramp=p('ramp').value,
                                 hold=p('hold').value, pause=p('pause').value)
        return forward_backward(speed=p('speed').value, ramp=p('ramp').value,
                                hold=p('hold').value, pause=p('pause').value)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # --- inputs ---------------------------------------------------------------------------

    def _on_mode(self, msg):
        self._mode = msg.mode
        self._runner.on_mode(msg.mode)      # aborts if the mode left Test (8.1.3)
        self._finish_if_ended()

    def _on_safety(self, msg):
        self._safety_ok = msg.ok
        self._runner.on_safety_state(msg.ok, tuple(msg.faults))   # aborts on fault (8.1.3)
        self._finish_if_ended()

    def _on_status(self, msg):
        self._runner.record_current(msg.motor_current_a)          # 8.2.3.3

    def _on_tx_frame(self, msg):
        if msg.id == self._joystick_id:
            self._runner.record_joystick_frame()                  # 8.2.3.4

    def _on_start(self, request, response):
        ok, message = self._runner.start(self._now(), self._mode, self._safety_ok)
        if ok:
            self._was_running = True
            self._logged_steps = 0
            self.get_logger().info('test sequence started')
        else:
            self.get_logger().warning(f'/test/start refused: {message}')
        response.success = ok
        response.message = message
        return response

    def _on_stop(self, request, response):
        self._runner.stop(self._now())
        self._publish(0.0, 0.0)
        self._finish_if_ended()
        response.success = True
        response.message = 'sequence stopped'
        return response

    # --- outputs --------------------------------------------------------------------------

    def _tick(self):
        cmd = self._runner.tick(self._now())
        if cmd is not None:
            self._publish(cmd[0], cmd[1])
        self._log_new_steps()
        self._finish_if_ended()

    def _publish(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self._test_pub.publish(msg)

    def _log_new_steps(self):
        """Log each step as it starts (8.2.2)."""
        log = self._runner.step_log
        for _, name in log[self._logged_steps:]:
            self.get_logger().info(f'step: {name}')
        self._logged_steps = len(log)

    def _finish_if_ended(self):
        """When a run ends, command zero, log the summary and return to Idle (3.2.5)."""
        if not self._was_running or self._runner.running:
            return
        self._was_running = False
        self._publish(0.0, 0.0)
        self._log_new_steps()
        summary = self._runner.summary
        self.get_logger().info(
            f'sequence {summary.result}'
            + (f' (abort: {summary.abort_reason})' if summary.abort_reason else '')
            + f'; peak current per step: {summary.peak_current}'
            + f'; joystick frames sent: {summary.joystick_frames}')
        if self._return_to_idle:
            self._request_idle()

    def _request_idle(self):
        if not self._mode_client.service_is_ready():
            self.get_logger().warning('/mode/request not available; cannot return to Idle')
            return
        request = RequestMode.Request()
        request.mode = 'idle'
        future = self._mode_client.call_async(request)
        future.add_done_callback(self._idle_done)

    def _idle_done(self, future):
        try:
            result = future.result()
        except Exception as err:  # noqa: BLE001 - log any service failure
            self.get_logger().warning(f'return-to-Idle request failed: {err}')
            return
        if not result.success:
            self.get_logger().warning(f'return-to-Idle refused: {result.message}')


def main(args=None):
    rclpy.init(args=args)
    node = SequenceNode()
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
