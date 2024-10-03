from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='demo_package',
            executable='demo_node',
            name='demo_node_single'
        )
    ])
