from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package='mongla_sim_web',
                executable='lab_server',
                name='mongla_sim_lab_server',
                output='screen',
            ),
        ]
    )
