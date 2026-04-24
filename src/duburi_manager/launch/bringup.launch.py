"""bringup -- start the AUV control stack (manager + optional vision).

One-command pool-day bringup:

    # Control only (default -- DVL yaw source, no vision):
    ros2 launch duburi_manager bringup.launch.py

    # With vision + gate model (forward camera):
    ros2 launch duburi_manager bringup.launch.py vision:=true

    # Explicit yaw source (mavlink_ahrs for bench / sim):
    ros2 launch duburi_manager bringup.launch.py yaw_source:=mavlink_ahrs

    # Full pool day (DVL + gate vision, no rqt):
    ros2 launch duburi_manager bringup.launch.py vision:=true rqt:=false

After launch, connect the DVL:
    ros2 run duburi_planner duburi dvl_connect

Run the gate mission:
    ros2 run duburi_planner mission gate_prequal
"""

from launch                     import LaunchDescription
from launch.actions             import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions          import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions       import LaunchConfiguration
from launch_ros.actions         import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    args = [
        DeclareLaunchArgument('mode',       default_value='auto',
                              description='Connection mode: auto|sim|pool|desk|laptop'),
        DeclareLaunchArgument('yaw_source', default_value='dvl',
                              description='Yaw source: dvl|mavlink_ahrs|bno085'),
        DeclareLaunchArgument('dvl_host',   default_value='192.168.2.201'),
        DeclareLaunchArgument('dvl_port',   default_value='9000'),
        DeclareLaunchArgument('vision',     default_value='false',
                              description='Start camera + detector alongside manager'),
        DeclareLaunchArgument('camera',     default_value='forward',
                              description='Camera profile name (forward|downward|laptop)'),
        DeclareLaunchArgument('model',      default_value='gate_nano_100ep',
                              description='Detector model (gate_nano_100ep|gate_medium_100ep|yolo26_nano_pretrained)'),
        DeclareLaunchArgument('classes',    default_value='gate',
                              description='CSV class names for detector'),
        DeclareLaunchArgument('conf',       default_value='0.30'),
        DeclareLaunchArgument('rqt',        default_value='true'),
    ]

    manager_node = Node(
        package='duburi_manager',
        executable='start',
        name='duburi_manager',
        output='screen',
        parameters=[{
            'mode':                 LaunchConfiguration('mode'),
            'yaw_source':           LaunchConfiguration('yaw_source'),
            'nucleus_dvl_host':     LaunchConfiguration('dvl_host'),
            'nucleus_dvl_port':     LaunchConfiguration('dvl_port'),
            'nucleus_dvl_password': 'nortek',
        }],
    )

    vision_launch_path = os.path.join(
        get_package_share_directory('duburi_vision'),
        'launch', 'cameras_.launch.py')

    vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(vision_launch_path),
        launch_arguments={
            'camera':  LaunchConfiguration('camera'),
            'model':   LaunchConfiguration('model'),
            'classes': LaunchConfiguration('classes'),
            'conf':    LaunchConfiguration('conf'),
            'rqt':     LaunchConfiguration('rqt'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('vision')),
    )

    return LaunchDescription(args + [manager_node, vision_launch])
