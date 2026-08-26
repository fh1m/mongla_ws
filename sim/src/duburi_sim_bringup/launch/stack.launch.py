"""Bring up the unmodified duburi_ws stack against a running simulator.

Prerequisites: the simulator is already up

    ros2 launch duburi_sim_bringup sim.launch.py course:=sauvc26_qualification gui:=false

Then, in a second terminal (both workspaces sourced):

    ros2 launch duburi_sim_bringup stack.launch.py

For a control-only smoke test with no YOLO weights:

    ros2 launch duburi_sim_bringup stack.launch.py vision:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            'vision', default_value='true',
            description='Start the vision pipeline on the sim front camera.',
        ),
        DeclareLaunchArgument(
            'model', default_value='gate_rescue_repair',
            description='Detector weights stem under duburi_vision/models.',
        ),
        DeclareLaunchArgument('models', default_value='gate=gate_rescue_repair'),
        DeclareLaunchArgument('active_model', default_value='gate'),
        DeclareLaunchArgument('classes', default_value='gate'),
        DeclareLaunchArgument('viewer', default_value='false'),
        DeclareLaunchArgument(
            'image_topic',
            default_value='/duburi/sim/front_camera/image_raw',
            description='Gazebo front-camera topic from the sim bridge.',
        ),
    ]

    # Reuse duburi_manager's own bringup so we stay aligned with its defaults,
    # then override only what SITL needs. Vision is launched separately below
    # because bringup.launch.py has no `topic:=` argument, and camera:=sim_front
    # would name the detector /duburi_detector_sim_front while missions look for
    # /duburi_detector_forward.
    manager = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('duburi_manager'),
                'launch', 'bringup.launch.py',
            )
        ),
        launch_arguments={
            'mode': 'sim',
            'flight_controller': 'pixhawk',
            'yaw_source': 'mavlink_ahrs',
            'vision': 'false',
            'dvl_auto_connect': 'false',
            'viewer': 'false',
        }.items(),
    )

    vision = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('duburi_vision'),
                'launch', 'vision.launch.py',
            )
        ),
        launch_arguments={
            'camera': 'forward',
            'topic': LaunchConfiguration('image_topic'),
            'model': LaunchConfiguration('model'),
            'models': LaunchConfiguration('models'),
            'active_model': LaunchConfiguration('active_model'),
            'classes': LaunchConfiguration('classes'),
            'viewer': LaunchConfiguration('viewer'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('vision')),
    )

    return LaunchDescription(args + [manager, vision])
