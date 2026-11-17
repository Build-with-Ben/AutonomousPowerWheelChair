"""
L2 node tests for the Mode Manager (domain 3).

Starts the real mode_node and drives it over /safety/state and /mode/request. The node is
launched once for the class, so tests are numbered to fix their order.
"""

import os
import time
import unittest

import launch
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from rclpy.node import Node as RclpyNode
from wheelchair_msgs.msg import ModeState, SafetyState
from wheelchair_msgs.srv import RequestMode

STATE_RATE = 1.0

# Each package's launch tests run in parallel under colcon, and they share topic
# names, so give every suite its own ROS domain.
ROS_DOMAIN_ID = '72'
os.environ['ROS_DOMAIN_ID'] = ROS_DOMAIN_ID


@pytest.mark.launch_test
def generate_test_description():
    """Launch the mode node with Auto disabled (the default)."""
    mode = Node(
        package='wheelchair_mode',
        executable='mode_node',
        name='wheelchair_mode',
        parameters=[{'auto_enabled': False, 'state_rate_hz': STATE_RATE}],
        additional_env={'ROS_DOMAIN_ID': ROS_DOMAIN_ID},
    )
    return launch.LaunchDescription([mode, launch_testing.actions.ReadyToTest()]), {}


class Harness(RclpyNode):
    """Publishes /safety/state and calls /mode/request."""

    def __init__(self):
        super().__init__('mode_test_harness')
        self.modes = []
        self._safety_pub = self.create_publisher(SafetyState, '/safety/state', 10)
        self.create_subscription(ModeState, '/mode/state', self.modes.append, 10)
        self._request = self.create_client(RequestMode, '/mode/request')

    def send_safety(self, faults=()):
        msg = SafetyState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.faults = list(faults)
        msg.ok = not faults
        msg.latched = bool(faults)
        self._safety_pub.publish(msg)

    def pump(self, duration, faults=None):
        """Spin for ``duration``, publishing /safety/state when ``faults`` is given."""
        end = time.time() + duration
        next_safety = 0.0
        while time.time() < end:
            now = time.time()
            if faults is not None and now >= next_safety:
                self.send_safety(faults)
                next_safety = now + 0.05
            rclpy.spin_once(self, timeout_sec=0.01)

    def request(self, mode, timeout=5.0):
        assert self._request.wait_for_service(timeout_sec=timeout), 'no /mode/request'
        req = RequestMode.Request()
        req.mode = mode
        future = self._request.call_async(req)
        end = time.time() + timeout
        while not future.done() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        assert future.done(), '/mode/request did not answer'
        return future.result()

    def wait_for_mode(self, expected, timeout=1.0):
        """Spin until /mode/state reports ``expected``; return how long that took."""
        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.01)
            if self.modes and self.modes[-1].mode == expected:
                return time.time() - start
        return None

    def clear(self):
        self.modes.clear()

    def last_mode(self):
        assert self.modes, 'no /mode/state received'
        return self.modes[-1].mode


class TestModeNode(unittest.TestCase):
    """Drives the running mode node over its real interfaces."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.harness = Harness()
        cls.harness.pump(0.5)

    @classmethod
    def tearDownClass(cls):
        cls.harness.destroy_node()
        rclpy.shutdown()

    @pytest.mark.req('3.1.3', '3.1.2')
    def test_01_starts_in_idle(self):
        h = self.harness
        h.pump(1.5)
        self.assertEqual(h.last_mode(), 'idle')

    @pytest.mark.req('3.2.2', '1.4.2')
    def test_02_request_refused_before_safety_reports(self):
        # No /safety/state has been published yet, so driving must be refused.
        result = self.harness.request('manual')
        self.assertFalse(result.success)
        self.assertEqual(result.current_mode, 'idle')

    @pytest.mark.req('3.2.1')
    def test_03_request_accepted_when_safety_ok(self):
        h = self.harness
        h.pump(0.4, faults=())
        result = h.request('manual')
        self.assertTrue(result.success, result.message)
        self.assertEqual(result.current_mode, 'manual')
        self.assertIsNotNone(h.wait_for_mode('manual'), 'mode/state did not report manual')

    @pytest.mark.req('3.2.3')
    def test_04_fault_forces_fault_mode_immediately(self):
        h = self.harness
        h.pump(0.3, faults=())
        h.request('manual')
        h.clear()
        h.send_safety(['pm_heartbeat_lost'])
        elapsed = h.wait_for_mode('fault', timeout=1.0)
        self.assertIsNotNone(elapsed, 'a fault must force Fault mode')
        # Much sooner than the 1 Hz timer, i.e. published on change, not on the tick.
        self.assertLess(elapsed, 1.0 / STATE_RATE,
                        'Fault must be reported immediately, not on the next tick')

    @pytest.mark.req('3.2.4')
    def test_05_requests_refused_while_faulted(self):
        h = self.harness
        h.pump(0.2, faults=['pm_heartbeat_lost'])
        for mode in ('manual', 'test', 'idle'):
            result = h.request(mode)
            self.assertFalse(result.success, f'{mode} must be refused while faulted')
            self.assertEqual(result.current_mode, 'fault')

    @pytest.mark.req('3.2.4', '3.2.4.1', '3.2.4.2')
    def test_06_cleared_fault_returns_to_idle_not_driving(self):
        h = self.harness
        h.clear()
        h.pump(0.4, faults=())      # cause gone and Safety cleared its latch
        self.assertEqual(h.last_mode(), 'idle',
                         'leaving Fault must land in Idle, never a driving mode')

    @pytest.mark.req('3.2.3')
    def test_07_estop_forces_estop_mode(self):
        h = self.harness
        h.pump(0.3, faults=())
        h.request('manual')
        h.clear()
        h.send_safety(['estop'])
        self.assertIsNotNone(h.wait_for_mode('estop'), 'e-stop must force E-stop mode')
        h.pump(0.4, faults=())
        self.assertEqual(h.last_mode(), 'idle')

    @pytest.mark.req('3.2.6', '1.1.3')
    def test_08_auto_is_rejected(self):
        h = self.harness
        h.pump(0.3, faults=())
        result = h.request('auto')
        self.assertFalse(result.success)
        self.assertNotEqual(result.current_mode, 'auto')

    @pytest.mark.req('3.1.2')
    def test_09_state_published_at_fixed_rate(self):
        h = self.harness
        h.clear()
        h.pump(2.5, faults=())
        self.assertGreaterEqual(len(h.modes), 2,
                                f'/mode/state should tick ~{STATE_RATE} Hz, '
                                f'got {len(h.modes)} in 2.5 s')


@launch_testing.post_shutdown_test()
class TestModeNodeShutdown(unittest.TestCase):
    """Checks the node exits cleanly."""

    def test_clean_exit(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
