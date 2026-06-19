"""dual_cameras -- forward + downward cameras, each with its own model + tracker, shared viewer.

Usage:
    ros2 launch duburi_vision dual_cameras.launch.py
    ros2 launch duburi_vision dual_cameras.launch.py fwd_device:=0 dwn_device:=4

    # Per-camera models (most common pool use-case):
    ros2 launch duburi_vision dual_cameras.launch.py \\
        fwd_model:=gate_flare_medium_100ep fwd_classes:=gate,flare \\
        dwn_model:=bin_model dwn_classes:=bin_fire,bin_blood

    ros2 launch duburi_vision dual_cameras.launch.py viewer:=false   # headless / mission mode

Viewer keys: f=forward  d=downward  b=side-by-side  D=depth-map
"""

from launch                  import LaunchDescription
from launch.actions          import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions       import IfCondition
from launch.event_handlers   import OnProcessExit
from launch.events           import Shutdown
from launch.substitutions    import LaunchConfiguration
from launch_ros.actions      import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('fwd_device',   default_value='0',
                              description='/dev/videoN for forward camera'),
        DeclareLaunchArgument('dwn_device',   default_value='4',
                              description='/dev/videoN for downward camera'),
        # Forward camera — gate/slalom tasks
        DeclareLaunchArgument('fwd_model',    default_value='gate_flare_medium_100ep',
                              description='YOLO model stem for forward detector'),
        DeclareLaunchArgument('fwd_classes',  default_value='gate,flare',
                              description='Comma-sep class filter for forward detector'),
        # Downward camera — bin task (use yolo11n until bin model is trained)
        DeclareLaunchArgument('dwn_model',    default_value='yolo11n',
                              description='YOLO model stem for downward detector'),
        DeclareLaunchArgument('dwn_classes',  default_value='',
                              description='Comma-sep class filter for downward detector (empty=all)'),
        DeclareLaunchArgument('conf',         default_value='0.35'),
        DeclareLaunchArgument('cls_device',   default_value='cuda:0'),
        DeclareLaunchArgument('viewer',       default_value='true'),
        DeclareLaunchArgument('tracking',     default_value='true'),
    ]

    def camera(profile: str, device_arg: str, node_name: str) -> Node:
        return Node(
            package='duburi_vision', executable='camera_node', name=node_name,
            output='screen',
            parameters=[{
                'profile': profile,
                'name':    profile,
                'device':  LaunchConfiguration(device_arg),
            }],
        )

    def detector(cam: str, model_arg: str, classes_arg: str, node_name: str) -> Node:
        return Node(
            package='duburi_vision', executable='detector_node', name=node_name,
            output='screen',
            parameters=[{
                'camera':     cam,
                'model_path': LaunchConfiguration(model_arg),
                'device':     LaunchConfiguration('cls_device'),
                'classes':    LaunchConfiguration(classes_arg),
                'conf':       LaunchConfiguration('conf'),
                'half':       True,
                'publish_debug_image': True,
                'debug_image_hz': 10.0,
            }],
        )

    def tracker(cam: str, node_name: str) -> Node:
        return Node(
            package='duburi_vision', executable='tracker_node', name=node_name,
            output='screen',
            parameters=[{'camera': cam}],
            condition=IfCondition(LaunchConfiguration('tracking')),
        )

    viewer = Node(
        package='duburi_vision', executable='vision_display',
        name='duburi_display', output='screen',
        parameters=[{'camera': 'forward'}],
        condition=IfCondition(LaunchConfiguration('viewer')),
    )

    shutdown_on_exit = RegisterEventHandler(
        OnProcessExit(target_action=viewer, on_exit=[EmitEvent(event=Shutdown())])
    )

    return LaunchDescription(args + [
        camera('forward',  'fwd_device', 'duburi_camera_fwd'),
        camera('downward', 'dwn_device', 'duburi_camera_dwn'),
        detector('forward',  'fwd_model', 'fwd_classes', 'duburi_detector_fwd'),
        detector('downward', 'dwn_model', 'dwn_classes', 'duburi_detector_dwn'),
        tracker('forward',   'duburi_tracker_fwd'),
        tracker('downward',  'duburi_tracker_dwn'),
        viewer,
        shutdown_on_exit,
    ])
