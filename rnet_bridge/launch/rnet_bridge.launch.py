"""
Launch the R-Net bridge.

The bridge opens the CAN interface itself, so ros2_socketcan is not started here. It
transmits only when tx_enabled is true; the default is false (receive-only), so this
launch file does not drive the chair unless tx_enabled:=true is passed.

    ros2 launch rnet_bridge rnet_bridge.launch.py interface:=can0
    ros2 launch rnet_bridge rnet_bridge.launch.py interface:=can0 tx_enabled:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    interface = LaunchConfiguration('interface')
    pm_nibble = LaunchConfiguration('pm_nibble')
    joystick_nibble = LaunchConfiguration('joystick_nibble')
    tx_enabled = LaunchConfiguration('tx_enabled')
    return LaunchDescription([
        DeclareLaunchArgument('interface', default_value='can0',
                              description='SocketCAN interface (can0, or vcan0 for testing)'),
        DeclareLaunchArgument('pm_nibble', default_value='0',
                              description='power module device nibble (measured: 0)'),
        DeclareLaunchArgument('joystick_nibble', default_value='2',
                              description='joystick module device nibble (measured: 2)'),
        DeclareLaunchArgument('tx_enabled', default_value='false',
                              description='follow the JSM and transmit; false = receive-only'),
        Node(
            package='rnet_bridge',
            executable='bridge_node',
            name='rnet_bridge',
            output='screen',
            parameters=[{
                'interface': interface,
                'pm_nibble': pm_nibble,
                'joystick_nibble': joystick_nibble,
                'tx_enabled': tx_enabled,
            }],
        ),
    ])
