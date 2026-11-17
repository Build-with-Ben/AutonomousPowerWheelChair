"""
L2 node tests for the Safety gate (domain 5).

Starts the real safety_node and drives it over topics and services, with no hardware.
Timeouts are shortened through parameters (test plan 3.4) so the suite stays fast.

The node is launched once for the whole class, so faults latched by one test would leak
into the next. Tests are numbered to fix their order, and every test after the first
starts from a known state via ``make_healthy_and_reset``.
"""

import os
import time
import unittest

from geometry_msgs.msg import Twist
import launch
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from rclpy.node import Node as RclpyNode
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from wheelchair_msgs.msg import RnetStatus, SafetyState

CMD_TIMEOUT = 0.2
HEARTBEAT_TIMEOUT = 0.4
STATUS_TIMEOUT = 0.3
OUTPUT_RATE = 50.0
STATE_RATE = 10.0

# Each package's launch tests run in parallel under colcon, and they share topic
# names, so give every suite its own ROS domain.
ROS_DOMAIN_ID = '71'
os.environ['ROS_DOMAIN_ID'] = ROS_DOMAIN_ID


@pytest.mark.launch_test
def generate_test_description():
    """Launch the safety node with short timeouts."""
    safety = Node(
        package='wheelchair_safety',
        executable='safety_node',
        name='wheelchair_safety',
        parameters=[{
            'cmd_timeout': CMD_TIMEOUT,
            'heartbeat_timeout': HEARTBEAT_TIMEOUT,
            'status_timeout': STATUS_TIMEOUT,
            'output_rate_hz': OUTPUT_RATE,
            'state_rate_hz': STATE_RATE,
        }],
        additional_env={'ROS_DOMAIN_ID': ROS_DOMAIN_ID},
    )
    return launch.LaunchDescription([safety, launch_testing.actions.ReadyToTest()]), {}


class Harness(RclpyNode):
    """Publishes the safety node's inputs and collects its outputs."""

    def __init__(self):
        super().__init__('safety_test_harness')
        self.safe = []
        self.states = []
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel/mux', 10)
        self._status_pub = self.create_publisher(RnetStatus, '/rnet/status', 10)
        self._estop_pub = self.create_publisher(Bool, '/safety/estop', 10)
        self.create_subscription(Twist, '/cmd_vel/safe', self.safe.append, 10)
        self.create_subscription(SafetyState, '/safety/state', self.states.append, 10)
        self._reset = self.create_client(Trigger, '/safety/reset')

    def send_status(self):
        msg = RnetStatus()
        stamp = self.get_clock().now().to_msg()
        msg.header.stamp = stamp
        msg.pm_alive = True
        msg.pm_last_seen = stamp
        msg.battery_pct = 50
        msg.motor_current_a = 1.0
        self._status_pub.publish(msg)

    def send_cmd(self, linear_x, angular_z=0.0):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self._cmd_pub.publish(msg)

    def send_estop(self, active):
        msg = Bool()
        msg.data = active
        self._estop_pub.publish(msg)

    def pump(self, duration, cmd=None, status=True, estop=None):
        """Spin for ``duration``, meanwhile feeding the chosen inputs."""
        end = time.time() + duration
        next_status = 0.0
        while time.time() < end:
            now = time.time()
            if status and now >= next_status:
                self.send_status()
                next_status = now + 0.05
            if cmd is not None:
                self.send_cmd(*cmd)
            if estop is not None:
                self.send_estop(estop)
            rclpy.spin_once(self, timeout_sec=0.01)

    def clear(self):
        self.safe.clear()
        self.states.clear()

    def call_reset(self, timeout=5.0):
        assert self._reset.wait_for_service(timeout_sec=timeout), 'no /safety/reset'
        future = self._reset.call_async(Trigger.Request())
        end = time.time() + timeout
        while not future.done() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        assert future.done(), '/safety/reset did not answer'
        return future.result()

    def make_healthy_and_reset(self):
        """Feed good inputs and clear any latched fault, so a test starts from ok."""
        self.pump(0.4, cmd=(0.0, 0.0))
        result = self.call_reset()
        assert result.success, f'reset should succeed when healthy: {result.message}'
        self.pump(0.1, cmd=(0.0, 0.0))

    def wait_ready(self, timeout=20.0):
        """Spin until the node is really publishing, so tests never race its startup."""
        end = time.time() + timeout
        while time.time() < end and not self.safe:
            self.pump(0.1, cmd=(0.0, 0.0))
        assert self.safe, f'safety_node published no /cmd_vel/safe within {timeout:.0f} s'

    def last_state(self):
        assert self.states, 'no /safety/state received'
        return self.states[-1]


class TestSafetyNode(unittest.TestCase):
    """Drives the running safety node over its real interfaces."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.harness = Harness()
        cls.harness.wait_ready()

    @classmethod
    def tearDownClass(cls):
        cls.harness.destroy_node()
        rclpy.shutdown()

    @pytest.mark.req('5.7.2', '5.1.2', '1.4.2')
    def test_01_startup_fault_blocks_output(self):
        h = self.harness
        h.clear()
        h.pump(0.4, cmd=(0.3, 0.0))     # a healthy, fresh command
        self.assertTrue(h.safe, 'safety published nothing on /cmd_vel/safe')
        self.assertTrue(all(m.linear.x == 0.0 for m in h.safe),
                        'output must stay zero while the Startup fault is latched')
        state = h.last_state()
        self.assertFalse(state.ok)
        self.assertTrue(state.latched)
        self.assertIn('startup', state.faults)

    @pytest.mark.req('5.6.2', '5.1.1')
    def test_02_reset_enables_passthrough(self):
        h = self.harness
        h.make_healthy_and_reset()
        h.clear()
        h.pump(0.3, cmd=(0.3, 0.5))
        self.assertTrue(h.last_state().ok, 'safety should be ok after reset')
        self.assertAlmostEqual(h.safe[-1].linear.x, 0.3, places=3)
        self.assertAlmostEqual(h.safe[-1].angular.z, 0.5, places=3)

    @pytest.mark.req('5.1.3')
    def test_03_output_published_at_fixed_rate(self):
        h = self.harness
        h.make_healthy_and_reset()
        h.clear()
        h.pump(1.0, cmd=(0.1, 0.0))
        self.assertGreater(len(h.safe), OUTPUT_RATE * 0.5,
                           f'/cmd_vel/safe too slow: {len(h.safe)} msgs in 1 s')
        self.assertLess(len(h.safe), OUTPUT_RATE * 1.4,
                        f'/cmd_vel/safe too fast: {len(h.safe)} msgs in 1 s')

    @pytest.mark.req('5.6.1')
    def test_04_state_published_at_fixed_rate(self):
        h = self.harness
        h.make_healthy_and_reset()
        h.clear()
        h.pump(1.0, cmd=(0.0, 0.0))
        self.assertGreater(len(h.states), STATE_RATE * 0.5,
                           f'/safety/state too slow: {len(h.states)} msgs in 1 s')

    @pytest.mark.req('5.2.1', '5.2.2')
    def test_05_command_watchdog_zeroes_without_latching(self):
        h = self.harness
        h.make_healthy_and_reset()
        h.pump(0.3, cmd=(0.3, 0.0))
        h.clear()
        h.pump(CMD_TIMEOUT + 0.25, cmd=None)    # status stays fresh, commands stop
        self.assertEqual(h.safe[-1].linear.x, 0.0, 'stale command must zero the output')
        state = h.last_state()
        self.assertTrue(state.ok, 'a stale command alone must not latch a fault (5.2.2)')
        self.assertFalse(state.latched)
        h.clear()
        h.pump(0.3, cmd=(0.2, 0.0))             # fresh commands return
        self.assertAlmostEqual(h.safe[-1].linear.x, 0.2, places=3,
                               msg='pass-through must resume when commands return')

    @pytest.mark.req('5.3.1', '5.3.4', '5.6.2')
    def test_06_status_loss_latches_and_reset_refused(self):
        h = self.harness
        h.make_healthy_and_reset()
        h.clear()
        h.pump(HEARTBEAT_TIMEOUT + 0.3, cmd=(0.3, 0.0), status=False)
        self.assertEqual(h.safe[-1].linear.x, 0.0, 'lost status must zero the output')
        faults = set(h.last_state().faults)
        self.assertIn('bridge_status_lost', faults)
        self.assertIn('pm_heartbeat_lost', faults)
        # The causes are still present, so a reset must be refused and say why.
        result = h.call_reset()
        self.assertFalse(result.success)
        self.assertIn('bridge_status_lost', result.message)

    @pytest.mark.req('5.4.1', '5.4.3')
    def test_07_estop_latches_until_cleared_and_reset(self):
        h = self.harness
        h.make_healthy_and_reset()
        h.clear()
        h.pump(0.3, cmd=(0.3, 0.0), estop=True)
        self.assertEqual(h.safe[-1].linear.x, 0.0, 'e-stop must zero the output')
        self.assertIn('estop', h.last_state().faults)
        # Releasing the e-stop alone must not clear it.
        h.pump(0.3, cmd=(0.3, 0.0), estop=False)
        self.assertIn('estop', h.last_state().faults, 'e-stop must stay latched (5.4.3)')
        self.assertEqual(h.safe[-1].linear.x, 0.0)
        # Cleared cause plus a reset clears it.
        result = h.call_reset()
        self.assertTrue(result.success, result.message)
        h.clear()
        h.pump(0.3, cmd=(0.3, 0.0), estop=False)
        self.assertAlmostEqual(h.safe[-1].linear.x, 0.3, places=3)


@launch_testing.post_shutdown_test()
class TestSafetyNodeShutdown(unittest.TestCase):
    """Checks the node exits cleanly, so a crash on shutdown can't hide."""

    def test_clean_exit(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
