from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='temperature_monitoring',
            executable='temp_pub',
            name='temperature_publisher_node',
            output='screen',
        ),
        Node(
            package='temperature_monitoring',
            executable='temp_sub',
            name='temperature_subscriber_node',
            output='screen',
        ),
    ])