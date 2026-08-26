from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package='duburi_sim_web',
                executable='lab_server',
                name='duburi_sim_lab_server',
                output='screen',
            ),
        ]
    )
