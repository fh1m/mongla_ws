"""A guided camera calibration in Batighor: the camera, the calibration node, Batighor.

    ros2 launch mongla_batighor calibrate.launch.py \
        device_path:=/dev/mongla_cam_downward applies_to:=pi_downward

Stop the vision launch first: it holds the cameras. The camera runs under the
name `calibration` at the size calibrations are taken at (1280x720), so every
topic is under /mongla/vision/calibration/ whichever camera it is, and the
`calibrate` layout serves them all. Then open http://<host>:<port>/?layout=calibrate.

`applies_to` names the camera profile the result is for, and `medium` whether
the board is seen through air or water; both can be changed from the panel
without a restart. Every process restarts after it dies (`launch_respawn`).
"""
import os
from functools import partial

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from mongla_localization import launch_defaults
from mongla_localization.launch_respawn import respawning


def generate_launch_description():
    args = [
        DeclareLaunchArgument('device_path', default_value='/dev/mongla_cam_downward',
                              description='the camera to calibrate'),
        DeclareLaunchArgument('applies_to', default_value='pi_downward',
                              description='the camera profile the result is for'),
        DeclareLaunchArgument('medium', default_value='air',
                              description='air | water: what the board is seen through'),
        DeclareLaunchArgument('grid', default_value='8x6',
                              description="the board's INNER corners"),
        DeclareLaunchArgument('square', default_value='0.025',
                              description="the board's square, metres"),
        DeclareLaunchArgument('width', default_value='1280'),
        DeclareLaunchArgument('height', default_value='720'),
        DeclareLaunchArgument('batighor', default_value='true',
                              description='start Batighor too'),
    ]
    args, settings = launch_defaults.for_launch_file(args, __file__)
    camera = partial(
        Node, package='mongla_vision', executable='camera_node',
        name='mongla_camera_calibration', output='screen',
        parameters=[{'source': 'v4l2', 'name': 'calibration',
                     'device_path': LaunchConfiguration('device_path'),
                     'width': LaunchConfiguration('width'),
                     'height': LaunchConfiguration('height'),
                     'stream_rate_hz': 15.0}])
    calibrate = partial(
        Node, package='mongla_vision', executable='calibrate',
        name='mongla_calibrate', output='screen',
        parameters=[{'device_path': LaunchConfiguration('device_path'),
                     'applies_to': LaunchConfiguration('applies_to'),
                     'medium': LaunchConfiguration('medium'),
                     'grid': LaunchConfiguration('grid'),
                     'square': LaunchConfiguration('square')}])
    batighor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('mongla_batighor'), 'launch',
            'batighor.launch.py')),
        condition=IfCondition(LaunchConfiguration('batighor')))
    return LaunchDescription(args + settings + [
        *respawning(camera, 'the calibration camera'),
        *respawning(calibrate, 'the calibration node'),
        batighor,
    ])
