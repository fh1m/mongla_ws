"""vision_dual -- forward + downward cameras, the RoboSub 2026 competition vision bringup.

Brings up both cameras, each with its own detector (+ tracker) and a
shared HUD viewer. Detector nodes are named ``duburi_detector_forward``
and ``duburi_detector_downward`` -- exactly the names the mission DSL
targets via ``duburi.set_model(... camera='forward')`` /
``resume_detector('downward')`` -- so per-task model/class hot-switching
just works.

Both detectors start PAUSED by default (saves Jetson GPU); each task
chunk calls ``duburi.resume_detector(<cam>)`` then ``pause_detector``.

Start it AFTER the control stack (``duburi start`` / bringup).

Usage:
    # Competition defaults (gate_rescue_repair fwd, bin_fire_blood dwn, paused):
    ros2 launch duburi_vision vision_dual.launch.py

    # Headless pool day:
    ros2 launch duburi_vision vision_dual.launch.py viewer:=false

    # Override per-camera models / devices:
    ros2 launch duburi_vision vision_dual.launch.py \\
        fwd_model:=gate_flare_medium_100ep fwd_classes:=gate,flare \\
        dwn_model:=bin_fire_blood dwn_classes:=fire,blood \\
        fwd_device:=0 dwn_device:=4

    # Detectors live from the start (no per-task resume):
    ros2 launch duburi_vision vision_dual.launch.py paused:=false

Viewer keys: f=forward  d=downward  b=side-by-side  D=depth-map
Camera/detector/tracker log at WARN; the viewer logs at INFO.
"""

from launch                  import LaunchDescription
from launch.actions          import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions       import IfCondition
from launch.event_handlers   import OnProcessExit
from launch.events           import Shutdown
from launch.substitutions    import LaunchConfiguration
from launch_ros.actions      import Node

_QUIET = ['--log-level', 'warn']


def generate_launch_description():
    args = [
        DeclareLaunchArgument('fwd_device',   default_value='0',
                              description='/dev/videoN for the forward camera'),
        DeclareLaunchArgument('dwn_device',   default_value='4',
                              description='/dev/videoN for the downward camera'),
        # Forward camera -- gate / slalom / torpedo tasks
        DeclareLaunchArgument('fwd_model',    default_value='gate_rescue_repair',
                              description='YOLO model stem for the forward detector'),
        DeclareLaunchArgument('fwd_classes',  default_value='gate,rescue,repair',
                              description='Class filter for the forward detector'),
        # Downward camera -- bin / drop tasks
        DeclareLaunchArgument('dwn_model',    default_value='bin_fire_blood',
                              description='YOLO model stem for the downward detector'),
        DeclareLaunchArgument('dwn_classes',  default_value='fire,blood',
                              description='Class filter for the downward detector'),
        DeclareLaunchArgument('fwd_conf',     default_value='0.35'),
        DeclareLaunchArgument('dwn_conf',     default_value='0.35'),
        DeclareLaunchArgument('device_cls',   default_value='cuda:0',
                              description='Inference device for YOLO (cuda:0 | cpu)'),
        DeclareLaunchArgument('imgsz',        default_value='640',
                              description='Inference square size (both detectors). NOTE: a TensorRT '
                                          '.engine bakes imgsz at export -- this only re-scales the '
                                          '.pt fallback. For TRT, re-export to match: '
                                          'export_engine --all --imgsz <N>, then imgsz:=<N> here.'),
        DeclareLaunchArgument('max_det',      default_value='100',
                              description='Post-NMS detection cap, both detectors (runtime; lower '
                                          'toward ~10 if NMS is the FPS bottleneck on busy frames).'),
        DeclareLaunchArgument('paused',       default_value='true',
                              description='Start both detectors paused (resume_detector per task)'),
        DeclareLaunchArgument('viewer',       default_value='true'),
        DeclareLaunchArgument('tracking',     default_value='true'),
    ]

    def camera(profile: str, device_arg: str) -> Node:
        return Node(
            package='duburi_vision', executable='camera_node',
            name=f'duburi_camera_{profile}', output='screen', ros_arguments=_QUIET,
            parameters=[{
                'profile': profile,
                'name':    profile,
                'device':  LaunchConfiguration(device_arg),
            }],
        )

    def detector(profile: str, model_arg: str, classes_arg: str, conf_arg: str) -> Node:
        return Node(
            package='duburi_vision', executable='detector_node',
            name=f'duburi_detector_{profile}', output='screen', ros_arguments=_QUIET,
            parameters=[{
                'camera':              profile,
                'model_path':          LaunchConfiguration(model_arg),
                'device':              LaunchConfiguration('device_cls'),
                'classes':             LaunchConfiguration(classes_arg),
                'conf':                LaunchConfiguration(conf_arg),
                'imgsz':               LaunchConfiguration('imgsz'),
                'max_det':             LaunchConfiguration('max_det'),
                'half':                True,
                'paused':              LaunchConfiguration('paused'),
                'publish_debug_image': True,
                'debug_image_hz':      10.0,
            }],
        )

    def tracker(profile: str) -> Node:
        return Node(
            package='duburi_vision', executable='tracker_node',
            name=f'duburi_tracker_{profile}', output='screen', ros_arguments=_QUIET,
            parameters=[{'camera': profile}],
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
        camera('forward',  'fwd_device'),
        camera('downward', 'dwn_device'),
        detector('forward',  'fwd_model', 'fwd_classes', 'fwd_conf'),
        detector('downward', 'dwn_model', 'dwn_classes', 'dwn_conf'),
        tracker('forward'),
        tracker('downward'),
        viewer,
        shutdown_on_exit,
    ])
