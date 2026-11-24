"""
Bring up the operator input chain: controller -> mapping -> teleop.

    joy_node -> /joy -> input_node -> /operator/input -> teleop_node -> /cmd_vel/teleop

Use this on its own to try a controller on a development machine without the chair, or
let ``wheelchair_bringup`` start the same three nodes as part of the full stack.

Arguments (2.1.2):
    profile     device profile name from wheelchair_input/config (default xbox_generic)
    device_id   which joystick to open when several are plugged in (default 0)
    joy_rate    joy_node autorepeat rate in Hz; see the note below (default 25)
    teleop      start teleop_node as well (default true)

``joy_node`` only publishes when something changes, so ``autorepeat_rate`` must stay above
teleop's publish rate: without it, holding the stick perfectly still produces no ``/joy``
messages and teleop's input timeout stops the chair mid-drive (2.1.3). The deadzone is left
at 0 here because teleop owns it as a parameter (2.2.3).

    ros2 launch wheelchair_input operator.launch.py
    ros2 launch wheelchair_input operator.launch.py profile:=atari_remote
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Start joy_node, the input mapper and, unless disabled, teleop."""
    profile = LaunchConfiguration('profile')
    device_id = LaunchConfiguration('device_id')
    joy_rate = LaunchConfiguration('joy_rate')

    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value='xbox_generic',
                              description='device profile from wheelchair_input/config'),
        DeclareLaunchArgument('device_id', default_value='0',
                              description='which joystick to open'),
        DeclareLaunchArgument('joy_rate', default_value='25.0',
                              description='joy_node autorepeat rate in Hz'),
        DeclareLaunchArgument('teleop', default_value='true',
                              description='also start teleop_node'),

        Node(package='joy', executable='joy_node', name='joy_node', output='screen',
             parameters=[{'device_id': device_id,
                          'autorepeat_rate': joy_rate,
                          'deadzone': 0.0}]),      # teleop owns the deadzone (2.2.3)
        Node(package='wheelchair_input', executable='input_node',
             name='wheelchair_input', output='screen',
             parameters=[{'profile': profile}]),
        Node(package='wheelchair_teleop', executable='teleop_node',
             name='wheelchair_teleop', output='screen',
             condition=IfCondition(LaunchConfiguration('teleop'))),
    ])
