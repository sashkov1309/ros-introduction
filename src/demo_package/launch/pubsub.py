from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='demo_package',
            executable='demo_publisher',
            name='demo_publisher_1'
        ),
        Node(
            package='demo_package',
            executable='demo_subscriber',
            name='demo_subscriber_1'
        ),
    ])
