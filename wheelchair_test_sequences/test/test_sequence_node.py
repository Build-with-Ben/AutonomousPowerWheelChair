"""
L2 node tests for the test sequence node (domain 8).

Starts the real sequence_node with a short profile and drives it over /mode/state,
/safety/state and the /test/{start,stop} services. The harness also serves
/mode/request, so the return-to-Idle wiring (3.2.5) can be checked without the Mode
Manager running.
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
from std_srvs.srv import Trigger
from wheelchair_msgs.msg import ModeState, SafetyState
from wheelchair_msgs.srv import RequestMode

RAMP = 0.2
HOLD = 0.3
PAUSE = 0.2
SPEED = 0.1
TOTAL = 4 * RAMP + 2 * HOLD + PAUSE     # the forward/back profile's total duration
OUTPUT_RATE = 20.0

# Each package's launch tests run in parallel under colcon, and they share topic
# names, so give every suite its own ROS domain.
ROS_DOMAIN_ID = '74'
os.environ['ROS_DOMAIN_ID'] = ROS_DOMAIN_ID


@pytest.mark.launch_test
def generate_test_description():
    """Launch the sequence node with a short forward/back profile."""
    sequence = Node(
        package='wheelchair_test_sequences',
        executable='sequence_node',
        name='wheelchair_test_sequences',
        parameters=[{
            'sequence': 'forward_back',
            'speed': SPEED,
            'ramp': RAMP,
            'hold': HOLD,
            'pause': PAUSE,
            'output_rate_hz': OUTPUT_RATE,
            'return_to_idle': True,
        }],
        additional_env={'ROS_DOMAIN_ID': ROS_DOMAIN_ID},
    )
    return launch.LaunchDescription([sequence, launch_testing.actions.ReadyToTest()]), {}


class Harness(RclpyNode):
    """Feeds mode and safety state, calls /test/*, and stands in for /mode/request."""

    def __init__(self):
        super().__init__('sequence_test_harness')
        self.test_cmds = []
        self.mode_requests = []
        self._mode_pub = self.create_publisher(ModeState, '/mode/state', 10)
        self._safety_pub = self.create_publisher(SafetyState, '/safety/state', 10)
        self.create_subscription(Twist, '/cmd_vel/test', self.test_cmds.append, 10)
        self._start = self.create_client(Trigger, '/test/start')
        self._stop = self.create_client(Trigger, '/test/stop')
        self.create_service(RequestMode, '/mode/request', self._on_mode_request)

    def _on_mode_request(self, request, response):
        self.mode_requests.append(request.mode)
        response.success = True
        response.message = 'accepted by test harness'
        response.current_mode = request.mode
        return response

    def send_mode(self, mode):
        msg = ModeState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.mode = mode
        self._mode_pub.publish(msg)

    def send_safety(self, faults=()):
        msg = SafetyState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.faults = list(faults)
        msg.ok = not faults
        msg.latched = bool(faults)
        self._safety_pub.publish(msg)

    def pump(self, duration, mode=None, faults=()):
        """Spin for ``duration``, publishing mode and safety state periodically."""
        end = time.time() + duration
        nxt = 0.0
        while time.time() < end:
            now = time.time()
            if now >= nxt:
                if mode is not None:
                    self.send_mode(mode)
                self.send_safety(faults)
                nxt = now + 0.05
            rclpy.spin_once(self, timeout_sec=0.01)

    def _call(self, client, name, timeout=5.0):
        assert client.wait_for_service(timeout_sec=timeout), f'no {name}'
        future = client.call_async(Trigger.Request())
        end = time.time() + timeout
        while not future.done() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        assert future.done(), f'{name} did not answer'
        return future.result()

    def start(self):
        return self._call(self._start, '/test/start')

    def stop(self):
        return self._call(self._stop, '/test/stop')

    def clear(self):
        self.test_cmds.clear()
        self.mode_requests.clear()


class TestSequenceNode(unittest.TestCase):
    """Drives the running sequence node over its real interfaces."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.harness = Harness()
        cls.harness.pump(0.5, mode='idle')

    @classmethod
    def tearDownClass(cls):
        cls.harness.destroy_node()
        rclpy.shutdown()

    @pytest.mark.req('8.1.1.1')
    def test_01_start_refused_outside_test_mode(self):
        h = self.harness
        h.pump(0.3, mode='idle')
        result = h.start()
        self.assertFalse(result.success)
        self.assertIn('test', result.message.lower())

    @pytest.mark.req('8.1.1.2')
    def test_02_start_refused_when_safety_not_ok(self):
        h = self.harness
        h.pump(0.3, mode='test', faults=['pm_heartbeat_lost'])
        result = h.start()
        self.assertFalse(result.success)
        self.assertIn('safety', result.message.lower())

    @pytest.mark.req('8.1.1')
    def test_03_nothing_published_before_start(self):
        h = self.harness
        h.pump(0.3, mode='test')
        h.clear()
        h.pump(0.4, mode='test')
        self.assertEqual(h.test_cmds, [], 'must not publish before /test/start')

    @pytest.mark.req('8.1.1', '8.1.4', '8.2.1', '8.2.3.1', '3.2.5')
    def test_04_full_run_profile_rate_and_return_to_idle(self):
        h = self.harness
        h.pump(0.3, mode='test')
        h.clear()
        self.assertTrue(h.start().success)
        h.pump(TOTAL + 0.4, mode='test')

        values = [m.linear.x for m in h.test_cmds]
        self.assertGreater(len(values), TOTAL * OUTPUT_RATE * 0.6,
                           f'/cmd_vel/test too slow: {len(values)} msgs in {TOTAL:.1f} s')
        self.assertGreater(max(values), SPEED * 0.8, 'the run should go forward')
        self.assertLess(min(values), -SPEED * 0.8, 'the run should also go backward')
        self.assertAlmostEqual(values[-1], 0.0, places=3, msg='it must finish at zero')
        self.assertTrue(all(m.angular.z == 0.0 for m in h.test_cmds),
                        'the forward/back profile must not turn (8.3.2.3)')
        self.assertIn('idle', h.mode_requests,
                      'the node must request Idle when a run ends (3.2.5)')

    @pytest.mark.req('8.1.2')
    def test_05_stop_ends_the_run_with_zero(self):
        h = self.harness
        h.pump(0.3, mode='test')
        h.clear()
        self.assertTrue(h.start().success)
        h.pump(RAMP + 0.1, mode='test')
        self.assertTrue(h.stop().success)
        h.pump(0.2, mode='test')
        self.assertAlmostEqual(h.test_cmds[-1].linear.x, 0.0, places=3,
                               msg='/test/stop must command zero')
        count = len(h.test_cmds)
        h.pump(0.4, mode='test')
        self.assertEqual(len(h.test_cmds), count, 'publishing must stop after /test/stop')

    @pytest.mark.req('8.1.3')
    def test_06_safety_fault_aborts_the_run(self):
        h = self.harness
        h.pump(0.3, mode='test')
        h.clear()
        self.assertTrue(h.start().success)
        h.pump(RAMP, mode='test')
        h.send_safety(['estop'])
        h.pump(0.2, mode='test', faults=['estop'])
        self.assertAlmostEqual(h.test_cmds[-1].linear.x, 0.0, places=3,
                               msg='a fault must abort the run and command zero')
        count = len(h.test_cmds)
        h.pump(0.4, mode='test', faults=['estop'])
        self.assertEqual(len(h.test_cmds), count, 'an aborted run must stop publishing')

    @pytest.mark.req('8.1.3')
    def test_07_leaving_test_mode_aborts_the_run(self):
        h = self.harness
        h.pump(0.3, mode='test')
        h.clear()
        self.assertTrue(h.start().success)
        h.pump(RAMP, mode='test')
        h.pump(0.3, mode='manual')
        count = len(h.test_cmds)
        h.pump(0.4, mode='manual')
        self.assertEqual(len(h.test_cmds), count,
                         'leaving Test mode must abort the run')


@launch_testing.post_shutdown_test()
class TestSequenceNodeShutdown(unittest.TestCase):
    """Checks the node exits cleanly."""

    def test_clean_exit(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
