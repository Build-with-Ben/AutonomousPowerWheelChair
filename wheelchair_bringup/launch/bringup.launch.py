"""
Bring up the full wheelchair stack (requirement 9.1).

Starts the nodes that exist today: the bridge, safety, control, mode manager, the
operator input chain and the test sequence source. Diagnostics is added when that node
exists (9.1.1).

Arguments (9.1.2):
    interface       CAN interface: can0 (default) or vcan0 for testing
    tx_enabled      bridge transmits (follows the JSM) when true; default false (9.1.5)
    limits_profile  bringup (default, 20% limits) or normal (4.2.3)
    nodes           all (default), pi (bridge/safety/control/mode) or operator (sources)
    joystick        start the gamepad chain when true; default true (9.1.1)
    profile         device profile for wheelchair_input; default xbox_one (2.1.2)

Nodes start in stages so a command source never comes up before safety (9.1.3): bridge and
safety first, then control and the mode manager, then the input sources. Shared parameters
come from config/bringup.yaml (9.2.1); interface, tx_enabled and limits_profile override it
from the arguments above.

    ros2 launch wheelchair_bringup bringup.launch.py
    ros2 launch wheelchair_bringup bringup.launch.py interface:=can0 tx_enabled:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    interface = LaunchConfiguration('interface').perform(context)
    tx_enabled = LaunchConfiguration('tx_enabled').perform(context).lower() in ('true', '1')
    joystick = LaunchConfiguration('joystick').perform(context).lower() in ('true', '1')
    profile = LaunchConfiguration('profile').perform(context)
    limits_profile = LaunchConfiguration('limits_profile').perform(context)
    which = LaunchConfiguration('nodes').perform(context)
    config = os.path.join(
        get_package_share_directory('wheelchair_bringup'), 'config', 'bringup.yaml')

    pi_side = which in ('all', 'pi')
    operator_side = which in ('all', 'operator')

    def node(package, executable, name, extra=None):
        params = [config] + ([extra] if extra else [])
        return Node(package=package, executable=executable, name=name,
                    output='screen', parameters=params)

    # Stage 1: the bus owner and the safety gate.
    stage1 = []
    if pi_side:
        stage1.append(node('rnet_bridge', 'bridge_node', 'rnet_bridge',
                           {'interface': interface, 'tx_enabled': tx_enabled}))
        stage1.append(node('wheelchair_safety', 'safety_node', 'wheelchair_safety'))

    # Stage 2: control and the mode manager.
    stage2 = []
    if pi_side:
        stage2.append(node('wheelchair_control', 'control_node', 'wheelchair_control',
                           {'limits_profile': limits_profile}))
        stage2.append(node('wheelchair_mode', 'mode_node', 'wheelchair_mode'))

    # Stage 3: command sources (never before safety is up).
    #
    # The operator chain is joy_node -> wheelchair_input -> wheelchair_teleop, so the
    # device layout lives in a profile and teleop stays device independent (2.1.2).
    # joy_node needs autorepeat_rate above teleop's publish rate: without it a stick
    # held still produces no /joy messages and teleop's input timeout stops the chair
    # mid-drive (2.1.3).
    stage3 = []
    if operator_side:
        if joystick:
            stage3.append(Node(package='joy', executable='joy_node', name='joy_node',
                               output='screen',
                               parameters=[config, {'autorepeat_rate': 25.0,
                                                    'deadzone': 0.0}]))
            stage3.append(node('wheelchair_input', 'input_node', 'wheelchair_input',
                               {'profile': profile}))
            stage3.append(node('wheelchair_teleop', 'teleop_node', 'wheelchair_teleop'))
        stage3.append(node('wheelchair_test_sequences', 'sequence_node',
                           'wheelchair_test_sequences'))

    return stage1 + [
        TimerAction(period=1.0, actions=stage2),
        TimerAction(period=2.0, actions=stage3),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('interface', default_value='can0',
                              description='CAN interface: can0 or vcan0'),
        DeclareLaunchArgument('tx_enabled', default_value='false',
                              description='bridge transmits when true (default false)'),
        DeclareLaunchArgument('limits_profile', default_value='bringup',
                              description='bringup (20% limits) or normal'),
        DeclareLaunchArgument('nodes', default_value='all',
                              description='all, pi, or operator'),
        DeclareLaunchArgument('joystick', default_value='true',
                              description='start joy_node, wheelchair_input and teleop'),
        DeclareLaunchArgument('profile', default_value='xbox_one',
                              description='device profile from wheelchair_input/config'),
        OpaqueFunction(function=_setup),
    ])
