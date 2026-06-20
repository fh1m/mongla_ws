"""full_mission -- competition dual-camera launch for RoboSub 2026.

Both detectors start PAUSED. Mission chunks call
    duburi.resume_detector('forward' | 'downward')
before each task and
    duburi.pause_detector(...)
after, saving GPU on the Jetson Orin Nano.

Usage:
    ros2 launch duburi_vision full_mission.launch.py
    ros2 launch duburi_vision full_mission.launch.py viewer:=false   # headless pool

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
        # Forward camera — gate/slalom/torpedo tasks
        DeclareLaunchArgument('fwd_model',    default_value='gate_rescue_repair',
                              description='YOLO model for forward detector (gate/rescue/repair)'),
        DeclareLaunchArgument('fwd_classes',  default_value='gate,rescue,repair',
                              description='Comma-sep class filter for forward detector'),
        # Downward camera — bin task (placeholder until bin_fire_blood trained)
        DeclareLaunchArgument('dwn_model',    default_value='yolo11n',
                              description='YOLO model for downward detector (bin_fire_blood when trained)'),
        DeclareLaunchArgument('dwn_classes',  default_value='fire,blood',
                              description='Comma-sep class filter for downward detector'),
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
                'paused':     True,   # resume_detector() per task, saves Jetson GPU
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
