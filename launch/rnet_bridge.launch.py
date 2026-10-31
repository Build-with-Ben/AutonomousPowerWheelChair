from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_socketcan',
            executable='socket_can_sender_node_exe',
            name='socket_can_sender_node',
            output='screen',
            parameters=[{
                'interface': 'can0',
                'auto_configure_and_activate': True,
            }],
        ),
        Node(
            package='rnet_bridge',
            executable='simple_tx_node',
            name='simple_tx_node',
            output='screen',
        ),
    ])