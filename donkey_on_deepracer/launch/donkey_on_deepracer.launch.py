from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_pkg',
            namespace='camera_pkg',
            executable='camera_node',
            name='camera_node'
        ),

        Node(
            package='servo_pkg',
            namespace='servo_pkg',
            executable='servo_node',
            name='servo_node'
        ), 

        Node(
            package='donkey_on_deepracer',
            executable="donkey_interface_node",
        ),

    ])