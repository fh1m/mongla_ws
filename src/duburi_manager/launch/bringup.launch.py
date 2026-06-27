"""bringup -- start the AUV control stack (manager + optional vision).

One-command pool-day bringup (defaults: pool mode, DVL auto-connect):

    # Control only (no vision):
    ros2 launch duburi_manager bringup.launch.py

    # With vision + gate+flare model (forward camera):
    ros2 launch duburi_manager bringup.launch.py vision:=true

    # Full pool day (BNO085 heading + DVL distance, gate+flare model, no viewer):
    ros2 launch duburi_manager bringup.launch.py vision:=true \\
        yaw_source:=bno085_dvl viewer:=false

    # Bench / sim -- no DVL, use mavlink AHRS:
    ros2 launch duburi_manager bringup.launch.py mode:=sim yaw_source:=mavlink_ahrs

Multi-model registry (switch models mid-mission without restart):
    ros2 launch duburi_manager bringup.launch.py vision:=true \\
        models:="gate=gate_nano_100ep,flare=flare_medium_100ep,combined=gate_flare_medium_100ep" \\
        active_model:=gate classes:=gate conf:=0.45

    # In the mission DSL:
    #   duburi.use('gate')               # phase: gate
    #   duburi.use('flare', 'flare')     # phase: flare
    #   duburi.use('combined', 'gate')   # phase: combined model, gate filter

DVL connects automatically on startup (dvl_auto_connect:=true by default).
Manual override: ros2 run duburi_planner duburi dvl_connect

Vision commands to test gate detection:
    ros2 run duburi_planner duburi vision_align --camera forward --target_class gate --axes yaw,lat --duration 10
    ros2 run duburi_planner duburi vision_move  --camera forward --target_class gate --fwd_fill 80 --mode area --duration 15

Run the gate+flare mission:
    ros2 run duburi_planner mission gate_flare_prequal
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
        DeclareLaunchArgument('mode',       default_value='pool',
                              description='Connection mode: pool|sim|auto|desk|laptop'),
        DeclareLaunchArgument('yaw_source', default_value='dvl',
                              description='Yaw source: dvl|mavlink_ahrs|bno085'),
        DeclareLaunchArgument('dvl_host',        default_value='192.168.2.201'),
        DeclareLaunchArgument('dvl_port',        default_value='9000'),
        DeclareLaunchArgument('dvl_auto_connect', default_value='true',
                              description='Auto-connect DVL at startup (true|false)'),
        DeclareLaunchArgument('vision',     default_value='false',
                              description='Start camera + detector alongside manager'),
        DeclareLaunchArgument('camera',     default_value='forward',
                              description='Camera profile name (forward|downward|laptop)'),
        DeclareLaunchArgument('model',      default_value='gate_flare_medium_100ep',
                              description='Single-model: gate_flare_medium_100ep|gate_nano_100ep|gate_medium_100ep|flare_medium_100ep'
                                          '|yolov11n (ROBOSUB-tested pretrained, sim/bench)|yolo26_nano_pretrained'),
        DeclareLaunchArgument('models',     default_value='',
                              description='Multi-model registry (CSV name=stem): '
                                          '"gate=gate_nano_100ep,combined=gate_flare_medium_100ep"'),
        DeclareLaunchArgument('active_model', default_value='',
                              description='Registry key to start with (requires models:="..." to be set)'),
        DeclareLaunchArgument('classes',    default_value='gate',
                              description='CSV class names for detector'),
        DeclareLaunchArgument('conf',       default_value='0.30'),
        DeclareLaunchArgument('viewer',     default_value='true',
                              description='Open vision_display (OpenCV viewer) alongside vision pipeline'),
        DeclareLaunchArgument('quiet',      default_value='true',
                              description='Mission-quiet logging: show only mission progress + the '
                                          'vision operator line (demote [STATE]/[ARDUB]/[RC ] telemetry '
                                          'to debug). Set false for full telemetry.'),
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
            'dvl_auto_connect':     LaunchConfiguration('dvl_auto_connect'),
            'mission_quiet':        LaunchConfiguration('quiet'),
        }],
    )

    vision_launch_path = os.path.join(
        get_package_share_directory('duburi_vision'),
        'launch', 'vision.launch.py')

    vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(vision_launch_path),
        launch_arguments={
            'camera':        LaunchConfiguration('camera'),
            'model':         LaunchConfiguration('model'),
            'models':        LaunchConfiguration('models'),
            'active_model':  LaunchConfiguration('active_model'),
            'classes':       LaunchConfiguration('classes'),
            'conf':          LaunchConfiguration('conf'),
            'viewer':        LaunchConfiguration('viewer'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('vision')),
    )

    return LaunchDescription(args + [manager_node, vision_launch])
