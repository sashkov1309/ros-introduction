from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='demo_package',
            executable='demo_server',
            name='demo_server_1'
        ),
        Node(
            package='demo_package',
            executable='demo_client',
            name='demo_client_1'
        ),
    ])
