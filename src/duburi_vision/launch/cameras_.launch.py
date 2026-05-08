"""cameras_ -- camera + YOLO26 detector + tracker + OpenCV viewer.

Named camera profiles (config.py / cameras.yaml):
    logitech → Logitech USB RGB webcam, /dev/video4 (default)
    laptop   → same as logitech
    forward  → Blue Robotics fwd cam, /dev/video0  (Jetson, pool)
    downward → Blue Robotics down cam, /dev/video2 (Jetson, pool)

Usage:
    ros2 launch duburi_vision cameras_.launch.py                        # laptop profile
    ros2 launch duburi_vision cameras_.launch.py camera:=forward        # vehicle fwd cam
    ros2 launch duburi_vision cameras_.launch.py camera:=laptop device:=4  # explicit device
    ros2 launch duburi_vision cameras_.launch.py viewer:=false          # headless
    ros2 launch duburi_vision cameras_.launch.py model:=gate_flare_medium_100ep classes:=gate,flare

    # Run on a pre-recorded video instead of a live webcam:
    ros2 launch duburi_vision cameras_.launch.py video_file:=/path/to/pool_run.mp4
    ros2 launch duburi_vision cameras_.launch.py video_file:=/tmp/gate.mp4 classes:=gate loop:=false

    # Live-tune tracker without restart:
    ros2 param set /duburi_tracker min_hits 1
    ros2 param set /duburi_tracker track_buffer 30
"""

from launch                       import LaunchDescription
from launch.actions               import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions            import IfCondition
from launch.event_handlers        import OnProcessExit
from launch.events                import Shutdown
from launch.substitutions         import LaunchConfiguration, PythonExpression
from launch_ros.actions           import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('camera',        default_value='logitech'),
        DeclareLaunchArgument('device',        default_value='0',
                              description='Device index override; -1 = use profile default'),
        DeclareLaunchArgument('width',         default_value='640'),
        DeclareLaunchArgument('height',        default_value='480'),
        DeclareLaunchArgument('fps',           default_value='30'),
        DeclareLaunchArgument('video_file',    default_value='',
                              description='Path to a video file; when set, overrides webcam source'),
        DeclareLaunchArgument('loop',          default_value='true',
                              description='Loop the video file when it reaches EOF (video_file only)'),
        DeclareLaunchArgument('model',         default_value='yolov11n',
                              description='Model name (from models/) or path to .pt file (single-model mode). '
                                          'Pretrained: yolov11n (ROBOSUB tested, person class). '
                                          'Pool: gate_flare_medium_100ep'),
        DeclareLaunchArgument('models',        default_value='',
                              description='CSV name=stem pairs to build a named registry: '
                                          '"gate=gate_nano_100ep,combined=gate_flare_medium_100ep"'),
        DeclareLaunchArgument('active_model',  default_value='',
                              description='Registry key to start with (requires models:="..." to be set)'),
        DeclareLaunchArgument('cls_device',    default_value='cuda:0'),
        DeclareLaunchArgument('classes',       default_value='person',
                              description='CSV class names or indices to detect; empty = all'),
        DeclareLaunchArgument('conf',          default_value='0.35'),
        DeclareLaunchArgument('iou',           default_value='0.5'),
        DeclareLaunchArgument('viewer',        default_value='true',
                              description='Open vision_display (OpenCV viewer) on image_raw at full FPS'),
        DeclareLaunchArgument('with_tracking', default_value='true',
                              description='Start tracker_node (ByteTrack + Kalman) alongside detector'),
        DeclareLaunchArgument('track_buffer',  default_value='30',
                              description='tracker_node: frames to hold lost track before expiry'),
        DeclareLaunchArgument('min_hits',      default_value='1',
                              description='tracker_node: consecutive detections before track is confirmed (1=instant)'),
        DeclareLaunchArgument('max_predict',   default_value='10',
                              description='tracker_node: Kalman frames to predict during detection gap'),
    ]

    cam_name   = LaunchConfiguration('camera')
    video_file = LaunchConfiguration('video_file')

    # Use named profile when not playing a video file; let profile own source/device/frame_id.
    # When video_file is set, pass source='video_file' and leave profile empty.
    profile_expr = PythonExpression(["'", cam_name, "' if not '", video_file, "' else ''"])
    source_expr  = PythonExpression(["'video_file' if '", video_file, "' else ''"])

    camera_node = Node(
        package='duburi_vision', executable='camera_node', name='duburi_camera',
        output='screen',
        parameters=[{
            'profile':         profile_expr,
            'source':          source_expr,
            'name':            cam_name,
            'device':          LaunchConfiguration('device'),   # -1 = use profile default
            'path':            video_file,
            'loop':            LaunchConfiguration('loop'),
            'width':           LaunchConfiguration('width'),
            'height':          LaunchConfiguration('height'),
            'fps':             LaunchConfiguration('fps'),
            'publish_rate_hz': LaunchConfiguration('fps'),
        }],
    )

    detector_node = Node(
        package='duburi_vision', executable='detector_node', name='duburi_detector',
        output='screen',
        parameters=[{
            'camera':              cam_name,
            'model_path':          LaunchConfiguration('model'),
            'models':              LaunchConfiguration('models'),
            'active_model':        LaunchConfiguration('active_model'),
            'device':              LaunchConfiguration('cls_device'),
            'classes':             LaunchConfiguration('classes'),
            'conf':                LaunchConfiguration('conf'),
            'iou':                 LaunchConfiguration('iou'),
            'half':                True,
            'publish_debug_image': True,
            'debug_image_hz':      10.0,
        }],
    )

    tracker_node = Node(
        package='duburi_vision', executable='tracker_node', name='duburi_tracker',
        output='screen',
        parameters=[{
            'camera':             cam_name,
            'track_buffer':       LaunchConfiguration('track_buffer'),
            'min_hits':           LaunchConfiguration('min_hits'),
            'max_predict_frames': LaunchConfiguration('max_predict'),
        }],
        condition=IfCondition(LaunchConfiguration('with_tracking')),
    )

    # True when a video file is being played (not a live camera).
    video_file_mode = PythonExpression(["True if '", video_file, "' else False"])

    image_viewer = Node(
        package='duburi_vision', executable='vision_display',
        name='duburi_image_view', output='screen',
        parameters=[{
            'camera':          cam_name,
            'video_file_mode': video_file_mode,
        }],
        condition=IfCondition(LaunchConfiguration('viewer')),
    )

    # When the viewer exits (Q key), shut down the entire launch group so
    # camera / detector / tracker nodes don't linger as orphans.
    shutdown_on_viewer_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=image_viewer,
            on_exit=[EmitEvent(event=Shutdown())],
        )
    )

    return LaunchDescription(
        args + [camera_node, detector_node, tracker_node, image_viewer,
                shutdown_on_viewer_exit]
    )
