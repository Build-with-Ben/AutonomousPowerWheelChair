"""
Launch the receive-only R-Net bridge.

The bridge opens the CAN interface itself, so ros2_socketcan is not started here. This
launch file does not transmit on the bus.

    ros2 launch rnet_bridge rnet_bridge.launch.py interface:=can0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    interface = LaunchConfiguration('interface')
    pm_nibble = LaunchConfiguration('pm_nibble')
    return LaunchDescription([
        DeclareLaunchArgument('interface', default_value='can0',
                              description='SocketCAN interface (can0, or vcan0 for testing)'),
        DeclareLaunchArgument('pm_nibble', default_value='0',
                              description='power module device nibble (measured: 0)'),
        Node(
            package='rnet_bridge',
            executable='bridge_node',
            name='rnet_bridge',
            output='screen',
            parameters=[{
                'interface': interface,
                'pm_nibble': pm_nibble,
            }],
        ),
    ])
