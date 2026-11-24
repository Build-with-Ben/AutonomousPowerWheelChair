"""
L2 node tests for teleop (domain 2).

Starts the real teleop_node and drives it over ``/operator/input``, the
device-independent topic, checking the published rate, the deadman behaviour, the
software e-stop and the input timeout. No ``/joy`` message and no axis or button index
appears here: that is ``wheelchair_input``'s job (2.1.2).

The node is launched once for the class, so tests are numbered to fix their order.
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
from wheelchair_msgs.msg import OperatorInput

OUTPUT_RATE = 20.0
INPUT_TIMEOUT = 0.5
MAX_LINEAR = 0.5
MAX_ANGULAR = 1.0
SPEED_SCALES = [0.25, 0.5, 1.0]
SOURCE = 'bench_pad'

# Each package's launch tests run in parallel under colcon and share topic names, so give
# every suite its own ROS domain.
ROS_DOMAIN_ID = '76'
os.environ['ROS_DOMAIN_ID'] = ROS_DOMAIN_ID


@pytest.mark.launch_test
def generate_test_description():
    """Launch teleop with the default limits and no device restriction."""
    node = Node(
        package='wheelchair_teleop',
        executable='teleop_node',
        name='wheelchair_teleop',
        parameters=[{
            'output_rate_hz': OUTPUT_RATE,
            'input_timeout': INPUT_TIMEOUT,
            'max_linear': MAX_LINEAR,
            'max_angular': MAX_ANGULAR,
            'speed_scales': SPEED_SCALES,
            'deadzone': 0.1,
            'request_manual_mode': False,   # no mode manager runs in this suite
            'rearm_delay': 0.2,
        }],
        additional_env={'ROS_DOMAIN_ID': ROS_DOMAIN_ID},
    )
    return launch.LaunchDescription([node, launch_testing.actions.ReadyToTest()]), {}


class Harness(RclpyNode):
    """Publishes /operator/input and collects /cmd_vel/teleop and /safety/estop."""

    def __init__(self):
        super().__init__('teleop_test_harness')
        self.commands = []
        self.estops = []
        self.resets = []
        self._input_pub = self.create_publisher(OperatorInput, '/operator/input', 10)
        # Stand in for wheelchair_safety, which does not run in this suite, so the
        # node's real service call can be observed.
        self.create_service(Trigger, '/safety/reset', self._on_reset)
        # Deep queues: teleop publishes at 20 Hz and a shallow one drops commands.
        self.create_subscription(Twist, '/cmd_vel/teleop', self.commands.append, 50)
        self.create_subscription(Bool, '/safety/estop', self.estops.append, 10)

    def send(self, **fields):
        """Publish one operator input message; unnamed fields stay at their defaults."""
        msg = OperatorInput()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.source = fields.pop('source', SOURCE)
        for name, value in fields.items():
            setattr(msg, name, value)
        self._input_pub.publish(msg)

    def pump(self, duration, send=None):
        """Spin for ``duration``, publishing ``send`` at ~50 Hz when given."""
        end = time.time() + duration
        next_send = 0.0
        while time.time() < end:
            now = time.time()
            if send is not None and now >= next_send:
                self.send(**send)
                next_send = now + 0.02
            rclpy.spin_once(self, timeout_sec=0.01)

    def wait_for_command(self, timeout=5.0, send=None, settle=0.3):
        """
        Hold one input state, then return a command computed from that state.

        Teleop publishes on its own timer from the last input it saw, so a command
        issued just before the new state arrived can still be in flight. Holding the
        state for a settling window first keeps each assertion about the state
        under test.
        """
        if send is not None:
            self.pump(settle, send=send)
        self.commands.clear()
        end = time.time() + timeout
        while time.time() < end:
            if send is not None:
                self.send(**send)
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.commands:
                return self.commands[-1]
        return None

    def _on_reset(self, request, response):
        self.resets.append(request)
        response.success = True
        response.message = 'all faults cleared'
        return response

    @staticmethod
    def zeros(commands):
        """Return the commands in ``commands`` that are a full stop."""
        return [c for c in commands if c.linear.x == 0.0 and c.angular.z == 0.0]

    def wait_ready(self, timeout=20.0):
        """Spin until teleop really answers, so tests never race its startup."""
        assert self.wait_for_command(timeout=timeout, send={'deadman': True}) is not None, \
            'teleop_node published no /cmd_vel/teleop'

    def step_speed_to_top(self):
        """Press speed_up enough times to reach the highest step."""
        for _ in range(len(SPEED_SCALES)):
            self.pump(0.12, send={'speed_up': True})
            self.pump(0.12, send={})

    def clear(self):
        self.commands.clear()
        self.estops.clear()
        self.resets.clear()


class TestTeleopNode(unittest.TestCase):
    """Drives the running teleop node over its real interfaces."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.harness = Harness()
        cls.harness.wait_ready()

    @classmethod
    def tearDownClass(cls):
        cls.harness.destroy_node()
        rclpy.shutdown()

    @pytest.mark.req('2.2.1', '2.3.1')
    def test_01_deadman_held_publishes_twist(self):
        msg = self.harness.wait_for_command(send={'deadman': True})
        self.assertIsNotNone(msg, 'holding the deadman must produce a command')
        self.assertIsInstance(msg, Twist)

    @pytest.mark.req('2.2.1.3')
    def test_02_only_linear_x_and_angular_z_are_set(self):
        h = self.harness
        msg = h.wait_for_command(send={'deadman': True, 'linear': 1.0, 'angular': 1.0})
        self.assertIsNotNone(msg)
        self.assertEqual(
            (msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y),
            (0.0, 0.0, 0.0, 0.0),
            'every Twist field other than linear.x and angular.z must stay zero')

    @pytest.mark.req('2.2.4', '2.2.5')
    def test_03_full_deflection_at_top_step_reaches_the_maximum_speeds(self):
        h = self.harness
        h.step_speed_to_top()
        msg = h.wait_for_command(send={'deadman': True, 'linear': 1.0, 'angular': -1.0})
        self.assertIsNotNone(msg)
        self.assertAlmostEqual(msg.linear.x, MAX_LINEAR, places=3)
        self.assertAlmostEqual(msg.angular.z, -MAX_ANGULAR, places=3)

    @pytest.mark.req('2.2.3')
    def test_04_input_inside_the_deadzone_commands_zero(self):
        h = self.harness
        msg = h.wait_for_command(send={'deadman': True, 'linear': 0.05, 'angular': 0.05})
        self.assertIsNotNone(msg)
        self.assertAlmostEqual(msg.linear.x, 0.0, places=4)
        self.assertAlmostEqual(msg.angular.z, 0.0, places=4)

    @pytest.mark.req('2.2.2')
    def test_05_publishes_at_the_configured_rate_while_centred(self):
        h = self.harness
        h.pump(0.3, send={'deadman': True})     # settle, then measure a clean second
        h.clear()
        h.pump(1.0, send={'deadman': True})
        # Allow generous slack for CI scheduling, but it must be a steady stream.
        self.assertGreaterEqual(len(h.commands), int(OUTPUT_RATE * 0.5),
                                f'expected ~{OUTPUT_RATE:.0f} commands in 1 s, '
                                f'got {len(h.commands)}')
        self.assertLessEqual(len(h.commands), int(OUTPUT_RATE * 2.0),
                             f'published faster than {OUTPUT_RATE:.0f} Hz: '
                             f'{len(h.commands)} in 1 s')
        for msg in h.commands:
            self.assertAlmostEqual(msg.linear.x, 0.0, places=4)

    @pytest.mark.req('2.3.1')
    def test_06_no_command_without_the_deadman(self):
        h = self.harness
        h.pump(0.4, send={'deadman': True})     # make sure it is driving first
        h.pump(0.4, send={})                    # release, which emits one zero
        h.clear()
        h.pump(0.6, send={'linear': 1.0})       # full stick, deadman released
        self.assertEqual(len(h.commands), 0,
                         'teleop must publish nothing while the deadman is released')

    @pytest.mark.req('2.3.2')
    def test_07_release_publishes_one_zero_then_stops(self):
        h = self.harness
        h.pump(0.4, send={'deadman': True, 'linear': 1.0})
        h.clear()
        h.pump(0.6, send={'linear': 1.0})       # deadman released, input still arriving
        released = list(h.commands)
        h.clear()
        h.pump(0.6, send={'linear': 1.0})       # still released, input still arriving
        self.assertEqual(len(h.zeros(released)), 1,
                         f'release must publish exactly one stop command, got '
                         f'{len(h.zeros(released))} of {len(released)} commands')
        self.assertEqual(len(h.commands), 0,
                         f'teleop must stop publishing after the release zero, '
                         f'got {len(h.commands)} more commands')

    @pytest.mark.req('2.1.3', '2.3.3')
    def test_08_stale_input_stops_publishing(self):
        h = self.harness
        h.pump(0.4, send={'deadman': True, 'linear': 1.0})
        h.clear()
        h.pump(INPUT_TIMEOUT + 0.4)     # no input at all, as if the pad unplugged
        before = len(h.commands)
        h.clear()
        h.pump(0.5)                     # still nothing
        self.assertEqual(len(h.commands), 0,
                         f'teleop must fall silent once input is stale '
                         f'(saw {before} during the timeout, {len(h.commands)} after)')

    @pytest.mark.req('2.3.4')
    def test_09_estop_input_publishes_true_on_safety_estop(self):
        h = self.harness
        h.clear()
        end = time.time() + 5.0
        while time.time() < end and not h.estops:
            h.send(estop=True)
            rclpy.spin_once(h, timeout_sec=0.05)
        self.assertTrue(h.estops, 'an e-stop press must publish on /safety/estop')
        self.assertTrue(h.estops[0].data, '/safety/estop must carry true')

    @pytest.mark.req('2.2.5')
    def test_10_speed_step_scales_the_output(self):
        h = self.harness
        # Walk back down to the lowest step, then compare against the top step.
        for _ in range(len(SPEED_SCALES) + 1):
            h.pump(0.12, send={'speed_down': True})
            h.pump(0.12, send={})
        low = h.wait_for_command(send={'deadman': True, 'linear': 1.0})
        self.assertIsNotNone(low)
        self.assertAlmostEqual(low.linear.x, MAX_LINEAR * SPEED_SCALES[0], places=3)
        h.step_speed_to_top()
        high = h.wait_for_command(send={'deadman': True, 'linear': 1.0})
        self.assertIsNotNone(high)
        self.assertAlmostEqual(high.linear.x, MAX_LINEAR * SPEED_SCALES[-1], places=3)

    @pytest.mark.req('2.1.2')
    def test_11_any_device_source_drives_by_default(self):
        """With no `sources` restriction, whichever device publishes is accepted."""
        h = self.harness
        for source in ('gamepad_generic', 'atari_remote', ''):
            msg = h.wait_for_command(send={'deadman': True, 'source': source})
            self.assertIsNotNone(msg, f'input from {source!r} should drive')

    @pytest.mark.req('2.3.4')
    def test_12_rearm_releases_the_estop_and_resets_safety(self):
        """A latched stop is cleared by releasing the cause, then resetting (2.3.4)."""
        h = self.harness
        h.pump(0.3, send={})          # make sure nothing is held
        h.clear()
        end = time.time() + 8.0
        while time.time() < end and not h.resets:
            h.send(rearm=True)
            rclpy.spin_once(h, timeout_sec=0.05)
        released = [m for m in h.estops if not m.data]
        self.assertTrue(released,
                        'a re-arm must publish false on /safety/estop')
        self.assertTrue(h.resets, 'a re-arm must call /safety/reset')

    @pytest.mark.req('2.3.4', '2.3.1')
    def test_13_rearm_ignored_while_the_deadman_is_held(self):
        """Clearing a latched stop must never happen while asking the chair to move."""
        h = self.harness
        h.pump(0.3, send={})
        h.clear()
        h.pump(1.5, send={'rearm': True, 'deadman': True})
        self.assertEqual(len(h.resets), 0,
                         'a re-arm with the deadman held must not reset safety')
        self.assertEqual([m for m in h.estops if not m.data], [],
                         'a re-arm with the deadman held must not release the e-stop')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    """The node must exit cleanly, not crash."""

    def test_exits_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
