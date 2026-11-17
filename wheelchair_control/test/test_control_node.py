"""
L2 node tests for Motor Control (domain 4).

Starts the real control_node and drives it over /cmd_vel/{teleop,test} and /mode/state.
The bringup limits profile is used, so the 0.5 m/s commands below clamp to 0.1 m/s.
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
from wheelchair_msgs.msg import ModeState

OUTPUT_RATE = 20.0
SOURCE_TIMEOUT = 0.25
BRINGUP_MAX_LINEAR = 0.1      # 20% of the 0.5 m/s normal limit (4.2.3)

# Each package's launch tests run in parallel under colcon, and they share topic
# names, so give every suite its own ROS domain.
ROS_DOMAIN_ID = '73'
os.environ['ROS_DOMAIN_ID'] = ROS_DOMAIN_ID


@pytest.mark.launch_test
def generate_test_description():
    """Launch the control node on the bringup limits profile."""
    control = Node(
        package='wheelchair_control',
        executable='control_node',
        name='wheelchair_control',
        parameters=[{
            'limits_profile': 'bringup',
            'source_timeout': SOURCE_TIMEOUT,
            'output_rate_hz': OUTPUT_RATE,
            'default_mode': 'idle',
            'mode_timeout': 2.0,
        }],
        additional_env={'ROS_DOMAIN_ID': ROS_DOMAIN_ID},
    )
    return launch.LaunchDescription([control, launch_testing.actions.ReadyToTest()]), {}


class Harness(RclpyNode):
    """Publishes command sources and the mode, and collects /cmd_vel/mux."""

    def __init__(self):
        super().__init__('control_test_harness')
        self.mux = []
        self._pubs = {
            source: self.create_publisher(Twist, f'/cmd_vel/{source}', 10)
            for source in ('teleop', 'test', 'auto')
        }
        self._mode_pub = self.create_publisher(ModeState, '/mode/state', 10)
        self.create_subscription(Twist, '/cmd_vel/mux', self.mux.append, 10)

    def send_mode(self, mode):
        msg = ModeState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.mode = mode
        self._mode_pub.publish(msg)

    def send_cmd(self, source, linear_x, angular_z=0.0):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self._pubs[source].publish(msg)

    def pump(self, duration, mode=None, sources=()):
        """Spin for ``duration``, publishing the mode and the given source commands."""
        end = time.time() + duration
        next_mode = 0.0
        while time.time() < end:
            now = time.time()
            if mode is not None and now >= next_mode:
                self.send_mode(mode)
                next_mode = now + 0.1
            for source, value in sources:
                self.send_cmd(source, value)
            rclpy.spin_once(self, timeout_sec=0.01)

    def clear(self):
        self.mux.clear()

    def last(self):
        assert self.mux, 'no /cmd_vel/mux received'
        return self.mux[-1]


class TestControlNode(unittest.TestCase):
    """Drives the running control node over its real interfaces."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.harness = Harness()
        cls.harness.pump(0.5)

    @classmethod
    def tearDownClass(cls):
        cls.harness.destroy_node()
        rclpy.shutdown()

    @pytest.mark.req('4.1.4')
    def test_01_zero_at_fixed_rate_with_no_source(self):
        h = self.harness
        h.clear()
        h.pump(1.0)
        self.assertGreater(len(h.mux), OUTPUT_RATE * 0.6,
                           f'/cmd_vel/mux too slow: {len(h.mux)} msgs in 1 s')
        self.assertTrue(all(m.linear.x == 0.0 and m.angular.z == 0.0 for m in h.mux),
                        'with no active source the output must be zero')

    @pytest.mark.req('4.1.3', '3.3.1')
    def test_02_idle_blocks_every_source(self):
        h = self.harness
        h.clear()
        h.pump(0.8, mode='idle', sources=[('teleop', 0.5), ('test', 0.5), ('auto', 0.5)])
        self.assertTrue(all(m.linear.x == 0.0 for m in h.mux),
                        'Idle must block every command source')

    @pytest.mark.req('4.1.1', '4.2.1', '4.2.3')
    def test_03_manual_passes_teleop_clamped_to_profile(self):
        h = self.harness
        h.pump(0.8, mode='manual', sources=[('teleop', 0.5)])
        self.assertAlmostEqual(h.last().linear.x, BRINGUP_MAX_LINEAR, places=3,
                               msg='teleop should drive, clamped to the bringup limit')

    @pytest.mark.req('4.1.1', '2.4.2')
    def test_04_teleop_outranks_test(self):
        h = self.harness
        # Both allowed in Test mode; teleop has the higher priority, so its value wins.
        h.pump(1.0, mode='test', sources=[('teleop', 0.02), ('test', 0.5)])
        self.assertAlmostEqual(h.last().linear.x, 0.02, places=3,
                               msg='teleop must win arbitration over the test source')

    @pytest.mark.req('4.1.2')
    def test_05_stale_source_drops_out(self):
        h = self.harness
        # teleop goes quiet; after source_timeout the test source takes over.
        h.pump(SOURCE_TIMEOUT + 0.9, mode='test', sources=[('test', 0.5)])
        self.assertAlmostEqual(h.last().linear.x, BRINGUP_MAX_LINEAR, places=3,
                               msg='test source should take over once teleop goes stale')

    @pytest.mark.req('4.1.3')
    def test_06_mode_blocks_a_source_it_disallows(self):
        h = self.harness
        # Back to Manual: the test source is no longer allowed, so output returns to zero.
        h.pump(1.2, mode='manual', sources=[('test', 0.5)])
        self.assertAlmostEqual(h.last().linear.x, 0.0, places=3,
                               msg='the test source must be blocked in Manual mode')

    @pytest.mark.req('4.1.3', '1.4.1')
    def test_07_mode_going_stale_falls_back_to_blocking(self):
        h = self.harness
        h.pump(0.8, mode='manual', sources=[('teleop', 0.5)])
        self.assertGreater(h.last().linear.x, 0.0, 'precondition: teleop is driving')
        # Stop publishing the mode; after mode_timeout the node falls back to idle.
        h.pump(2.6, mode=None, sources=[('teleop', 0.5)])
        self.assertAlmostEqual(h.last().linear.x, 0.0, places=3,
                               msg='a stale mode must block all sources')


@launch_testing.post_shutdown_test()
class TestControlNodeShutdown(unittest.TestCase):
    """Checks the node exits cleanly."""

    def test_clean_exit(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
