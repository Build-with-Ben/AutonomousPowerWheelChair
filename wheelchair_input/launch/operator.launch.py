"""
Bring up the operator input chain: controller -> mapping -> teleop.

    joy_node -> /joy -> input_node -> /operator/input -> teleop_node -> /cmd_vel/teleop

Use this on its own to try a controller on a development machine without the chair, or
let ``wheelchair_bringup`` start the same three nodes as part of the full stack.

Arguments (2.1.2):
    profile     device profile name from wheelchair_input/config (default xbox_one)
    device      joystick name to open (default 'Xbox One Controller'); '' falls back to
                device_id, which is positional and so changes between machines and boots
    joy_rate    joy_node autorepeat rate in Hz; see the note below (default 25)
    teleop      start teleop_node as well (default true)

``joy_node`` only publishes when something changes, so ``autorepeat_rate`` must stay above
teleop's publish rate: without it, holding the stick perfectly still produces no ``/joy``
messages and teleop's input timeout stops the chair mid-drive (2.1.3). The deadzone is left
at 0 here because teleop owns it as a parameter (2.2.3); ``joy_node``'s own default is 0.05,
which would otherwise be applied on top.

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
    device = LaunchConfiguration('device')
    joy_rate = LaunchConfiguration('joy_rate')

    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value='xbox_one',
                              description='device profile from wheelchair_input/config'),
        DeclareLaunchArgument('device', default_value='Xbox One Controller',
                              description="joystick name to open; '' uses index 0"),
        DeclareLaunchArgument('joy_rate', default_value='25.0',
                              description='joy_node autorepeat rate in Hz'),
        DeclareLaunchArgument('teleop', default_value='true',
                              description='also start teleop_node'),

        # Named rather than indexed: an index shifts with how many input devices are
        # attached, a name does not. joy_node retries until the named device appears.
        Node(package='joy', executable='joy_node', name='joy_node', output='screen',
             parameters=[{'device_name': device,
                          'autorepeat_rate': joy_rate,
                          'deadzone': 0.0}]),      # teleop owns the deadzone (2.2.3)
        Node(package='wheelchair_input', executable='input_node',
             name='wheelchair_input', output='screen',
             parameters=[{'profile': profile}]),
        Node(package='wheelchair_teleop', executable='teleop_node',
             name='wheelchair_teleop', output='screen',
             # Matches bringup.yaml; see the note there on the measured stick offset.
             parameters=[{'deadzone': 0.15}],
             condition=IfCondition(LaunchConfiguration('teleop'))),
    ])
