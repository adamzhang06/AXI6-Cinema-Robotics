from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='axi6_camera_system',
            executable='vision_node',
            name='vision_node',
            output='screen'
        ),
        Node(
            package='axi6_camera_system',
            executable='controller_node',
            name='controller_node',
            output='screen'
        ),
        Node(
            package='axi6_camera_system',
            executable='hardware_node',
            name='hardware_node',
            output='screen'
        ),
    ])
