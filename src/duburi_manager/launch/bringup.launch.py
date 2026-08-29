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
        # 'auto' scans USB VID/PID for the CH340 payload board, which is the
        # right default on the vehicle. In simulation there is no CH340: point
        # this at the PTY that `payload_sim` creates
        # (payload_port:=/tmp/duburi-$USER/payload) and fire() works there too.
        DeclareLaunchArgument('payload_port', default_value='auto',
                              description='Payload board device, or "auto" to '
                                          'scan USB VID/PID.'),
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
        DeclareLaunchArgument('imgsz',      default_value='640',
                              description='Inference square size. NOTE: a TensorRT .engine bakes '
                                          'imgsz at export -- this only re-scales the .pt fallback. '
                                          'For TRT, re-export to match (export_engine --all '
                                          '--imgsz <N>), then imgsz:=<N>.'),
        DeclareLaunchArgument('max_det',    default_value='100',
                              description='Post-NMS detection cap (runtime; lower toward ~10 if NMS '
                                          'is the FPS bottleneck on busy frames).'),
        DeclareLaunchArgument('viewer',     default_value='true',
                              description='Open vision_display (OpenCV viewer) alongside vision pipeline'),
        DeclareLaunchArgument('foxglove',   default_value='false',
                              description='Start foxglove_bridge (WebSocket telemetry on foxglove_port). '
                                          'Off the mission path -- pure viz. Needs '
                                          'ros-humble-foxglove-bridge installed. Connect the Foxglove '
                                          'desktop app to ws://<jetson-ip>:<foxglove_port>.'),
        DeclareLaunchArgument('foxglove_port', default_value='8765',
                              description='foxglove_bridge WebSocket port'),
    ]

    manager_node = Node(
        package='duburi_manager',
        executable='start',
        name='duburi_manager',
        output='screen',
        # Process default warn silences rcl/rmw framework gibberish; the manager's
        # own logger is pinned to info so all Mongla telemetry ([STATE]/[ARDUB]/
        # [RC ]/[ACT]) and the mission progress lines always show.
        ros_arguments=['--log-level', 'warn', '--log-level', 'duburi_manager:=info'],
        parameters=[{
            'mode':                 LaunchConfiguration('mode'),
            'yaw_source':           LaunchConfiguration('yaw_source'),
            'nucleus_dvl_host':     LaunchConfiguration('dvl_host'),
            'nucleus_dvl_port':     LaunchConfiguration('dvl_port'),
            'nucleus_dvl_password': 'nortek',
            'dvl_auto_connect':     LaunchConfiguration('dvl_auto_connect'),
            'payload_port':         LaunchConfiguration('payload_port'),
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
            'imgsz':         LaunchConfiguration('imgsz'),
            'max_det':       LaunchConfiguration('max_det'),
            'viewer':        LaunchConfiguration('viewer'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('vision')),
    )

    # Foxglove telemetry bridge -- opt-in, off the mission path (pure viz). Auto-
    # exposes every topic over a WebSocket the Foxglove desktop app renders (our
    # detections/images/state are standard vision_msgs/sensor_msgs). use_compression
    # cuts tether bandwidth; still confirm detection FPS is unperturbed with it up
    # and images being viewed (Mongla is FPS-coupled). See foxglove-and-bags.md.
    foxglove_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        ros_arguments=['--log-level', 'warn', '--log-level', 'foxglove_bridge:=info'],
        parameters=[{
            'port':            LaunchConfiguration('foxglove_port'),
            'address':         '0.0.0.0',
            'use_compression': True,
            # /duburi/move/_action/feedback + /status live under the _action
            # namespace => ROS 2 HIDDEN topics. Without this the live err_x_px/
            # err_y_px convergence plot is silently empty (default is false).
            'include_hidden':  True,
        }],
        condition=IfCondition(LaunchConfiguration('foxglove')),
    )

    return LaunchDescription(args + [manager_node, vision_launch, foxglove_node])
