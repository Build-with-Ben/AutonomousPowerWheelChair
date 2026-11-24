"""
L2 node tests for operator input mapping (domain 2.1).

Starts the real input_node with the shipped gamepad_generic profile and drives it over
``/joy``, checking that what comes out of ``/operator/input`` is device independent and
that a muted device publishes nothing. The node is launched once for the class, so tests
are numbered to fix their order.
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
from rcl_interfaces.srv import SetParameters
import rclpy
from rclpy.node import Node as RclpyNode
from rclpy.parameter import Parameter
from sensor_msgs.msg import Joy
from wheelchair_msgs.msg import OperatorInput

PROFILE = 'gamepad_generic'
# Indices from config/gamepad_generic.yaml. The test asserts the node honours the profile,
# so these are the profile's values, not a second copy of a device layout.
LINEAR_AXIS = 1
ANGULAR_AXIS = 0
DEADMAN_BUTTON = 4
ESTOP_BUTTON = 1
MANUAL_COMBO = (8, 9)
N_AXES = 6
N_BUTTONS = 13

# Each package's launch tests run in parallel under colcon and share topic names, so give
# every suite its own ROS domain.
ROS_DOMAIN_ID = '75'
os.environ['ROS_DOMAIN_ID'] = ROS_DOMAIN_ID


@pytest.mark.launch_test
def generate_test_description():
    """Launch the input node with the generic gamepad profile."""
    node = Node(
        package='wheelchair_input',
        executable='input_node',
        name='wheelchair_input',
        parameters=[{'profile': PROFILE, 'joy_timeout': 0.5}],
        additional_env={'ROS_DOMAIN_ID': ROS_DOMAIN_ID},
    )
    return launch.LaunchDescription([node, launch_testing.actions.ReadyToTest()]), {}


class Harness(RclpyNode):
    """Publishes /joy and collects /operator/input."""

    def __init__(self):
        super().__init__('input_test_harness')
        self.inputs = []
        self._joy_pub = self.create_publisher(Joy, '/joy', 10)
        # A deep queue: the node answers every /joy message, so a shallow one drops some.
        self.create_subscription(OperatorInput, '/operator/input', self.inputs.append, 50)
        self._set_params = self.create_client(
            SetParameters, '/wheelchair_input/set_parameters')

    def joy(self, axes=None, buttons=()):
        """Publish one /joy message with the named buttons held."""
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.axes = [0.0] * N_AXES
        for index, value in (axes or {}).items():
            msg.axes[index] = value
        msg.buttons = [1 if i in buttons else 0 for i in range(N_BUTTONS)]
        self._joy_pub.publish(msg)

    def pump(self, duration, axes=None, buttons=(), send=True):
        """Spin for ``duration``, publishing /joy at ~50 Hz unless ``send`` is False."""
        end = time.time() + duration
        next_joy = 0.0
        while time.time() < end:
            now = time.time()
            if send and now >= next_joy:
                self.joy(axes, buttons)
                next_joy = now + 0.02
            rclpy.spin_once(self, timeout_sec=0.01)

    def send_and_wait(self, axes=None, buttons=(), timeout=5.0):
        """
        Hold one /joy state, then return a mapped message produced from that state.

        The node answers every /joy message, so output from the previous state can still
        be in flight. Publishing the new state for a settling window and only then taking
        a message keeps each assertion about the state under test.
        """
        self.pump(0.3, axes, buttons)       # flush anything still in flight
        self.clear()
        end = time.time() + timeout
        while time.time() < end:
            self.joy(axes, buttons)
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.inputs:
                return self.inputs[-1]
        raise AssertionError('no /operator/input within the timeout')

    def set_enabled(self, value, timeout=10.0):
        """Flip the node's ``enabled`` parameter at runtime."""
        assert self._set_params.wait_for_service(timeout_sec=timeout), 'no set_parameters'
        request = SetParameters.Request()
        request.parameters = [Parameter('enabled', Parameter.Type.BOOL,
                                        value).to_parameter_msg()]
        future = self._set_params.call_async(request)
        end = time.time() + timeout
        while not future.done() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        assert future.done(), 'set_parameters did not answer'
        assert future.result().results[0].successful

    def wait_ready(self, timeout=20.0):
        """Spin until the node really answers, so tests never race its startup."""
        self.send_and_wait(buttons=(), timeout=timeout)

    def clear(self):
        self.inputs.clear()


class TestInputNode(unittest.TestCase):
    """Drives the running input node over its real interfaces."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.harness = Harness()
        cls.harness.wait_ready()

    @classmethod
    def tearDownClass(cls):
        cls.harness.destroy_node()
        rclpy.shutdown()

    @pytest.mark.req('2.1.1', '2.1.2')
    def test_01_joy_is_mapped_to_operator_input(self):
        msg = self.harness.send_and_wait(axes={LINEAR_AXIS: 0.8})
        self.assertEqual(msg.source, PROFILE)
        self.assertAlmostEqual(msg.linear, 0.8, places=3)

    @pytest.mark.req('2.1.2', '2.2.1.1', '2.2.1.2')
    def test_02_each_drive_axis_reaches_its_own_field(self):
        h = self.harness
        msg = h.send_and_wait(axes={LINEAR_AXIS: 1.0})
        self.assertAlmostEqual(msg.linear, 1.0, places=3)
        self.assertAlmostEqual(msg.angular, 0.0, places=3)
        msg = h.send_and_wait(axes={ANGULAR_AXIS: 1.0})
        self.assertAlmostEqual(msg.linear, 0.0, places=3)
        self.assertAlmostEqual(msg.angular, 1.0, places=3)

    @pytest.mark.req('2.1.2', '2.3.1')
    def test_03_deadman_button_sets_the_deadman_field(self):
        h = self.harness
        self.assertTrue(h.send_and_wait(buttons=(DEADMAN_BUTTON,)).deadman)
        self.assertFalse(h.send_and_wait(buttons=()).deadman)

    @pytest.mark.req('2.1.2', '2.3.4')
    def test_04_estop_button_sets_the_estop_field(self):
        self.assertTrue(self.harness.send_and_wait(buttons=(ESTOP_BUTTON,)).estop)

    @pytest.mark.req('2.1.2', '2.3.5')
    def test_05_manual_mode_needs_the_whole_combo(self):
        h = self.harness
        self.assertFalse(h.send_and_wait(buttons=MANUAL_COMBO[:1]).mode_manual)
        self.assertTrue(h.send_and_wait(buttons=MANUAL_COMBO).mode_manual)

    @pytest.mark.req('2.1.2')
    def test_06_short_joy_message_does_not_kill_the_node(self):
        """A pad with fewer axes than the profile expects must not crash the mapper."""
        h = self.harness
        h.clear()
        msg = Joy()
        msg.header.stamp = h.get_clock().now().to_msg()
        msg.axes = [0.0]
        msg.buttons = [0]
        for _ in range(10):
            h._joy_pub.publish(msg)
            rclpy.spin_once(h, timeout_sec=0.05)
        # Still alive and still mapping.
        self.assertAlmostEqual(
            h.send_and_wait(axes={LINEAR_AXIS: 0.5}).linear, 0.5, places=3)

    @pytest.mark.req('2.1.2')
    def test_07_one_output_per_joy_message(self):
        """A pass-through, so a disconnect reaches teleop as silence (2.3.3)."""
        h = self.harness
        h.clear()
        for _ in range(10):
            h.joy(buttons=())
            rclpy.spin_once(h, timeout_sec=0.02)
        h.pump(0.3, send=False)
        self.assertGreaterEqual(len(h.inputs), 8,
                                f'expected ~10 mapped messages, got {len(h.inputs)}')
        self.assertLessEqual(len(h.inputs), 14,
                             'the node must not republish on a timer')

    @pytest.mark.req('2.1.3', '2.3.3')
    def test_08_silence_on_joy_produces_silence_downstream(self):
        """Nothing may be republished when /joy stops, or teleop could never time out."""
        h = self.harness
        h.pump(0.2, buttons=())
        h.clear()
        h.pump(1.0, send=False)        # no /joy published at all
        self.assertEqual(len(h.inputs), 0,
                         'input_node must stay silent while /joy is silent')

    @pytest.mark.req('2.1.2')
    def test_09_disabled_device_publishes_nothing(self):
        """The enable/disable seam for running more than one controller."""
        h = self.harness
        h.set_enabled(False)
        h.clear()
        h.pump(0.6, axes={LINEAR_AXIS: 1.0}, buttons=(DEADMAN_BUTTON,))
        self.assertEqual(len(h.inputs), 0, 'a disabled device must publish nothing')

    @pytest.mark.req('2.1.2')
    def test_10_re_enabled_device_publishes_again(self):
        h = self.harness
        h.set_enabled(True)
        self.assertTrue(h.send_and_wait(buttons=(DEADMAN_BUTTON,)).deadman)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    """The node must exit cleanly, not crash."""

    def test_exits_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
