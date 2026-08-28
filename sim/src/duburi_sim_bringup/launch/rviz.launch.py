"""RViz2 for the Duburi simulator: robot model, sensors, and truth vs belief.

Why RViz when Gazebo already shows the pool: Gazebo renders what is TRUE, RViz
renders what the robot BELIEVES and what it can actually see. That distinction
is the whole subject of the sim audit -- depth read from `AHRS2.altitude` sits
~0.33 m off truth at the surface, and the DVL integrator drifts -- so this
config deliberately draws both poses at once.

Brings up three things:
  robot_state_publisher  base_link -> camera/DVL frames, from the generated URDF
  duburi_sim_bridge/tf_broadcaster  odom -> base_link from ground truth
  rviz2                  with config/duburi.rviz

Run standalone against a live sim:
    ros2 launch duburi_sim_bringup rviz.launch.py
or as part of the sim:
    ros2 run duburi_sim_bringup duburi_sim sim rviz:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    description_share = get_package_share_directory('duburi_sim_description')
    bringup_share = get_package_share_directory('duburi_sim_bringup')

    urdf_path = os.path.join(description_share, 'urdf', 'duburi_heavy.urdf')
    rviz_config = os.path.join(bringup_share, 'config', 'duburi.rviz')

    # Read the URDF here rather than passing a path: robot_state_publisher wants
    # the description as a string parameter, and failing loudly at launch is
    # better than RViz showing an empty scene with no explanation.
    with open(urdf_path) as fh:
        robot_description = fh.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Start rviz2 itself (false = TF and robot model only).'),
        DeclareLaunchArgument(
            'gui', default_value='false',
            description='Start joint_state_publisher_gui. Off: the model has no '
                        'movable joints worth publishing.'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='duburi_sim_robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description,
                         'use_sim_time': False}],
        ),
        Node(
            package='duburi_sim_bridge',
            executable='tf_broadcaster',
            name='duburi_sim_tf',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='duburi_rviz',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
