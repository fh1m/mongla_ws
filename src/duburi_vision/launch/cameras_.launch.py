"""cameras_ -- camera + YOLO26 detector + optional tracker + debug viewer.

The general-purpose launch file for the vision pipeline.  Detection is on
by default; tracking (ByteTrack + Kalman) is opt-in via with_tracking:=true.

Usage:
    ros2 launch duburi_vision cameras_.launch.py
    ros2 launch duburi_vision cameras_.launch.py device:=2 conf:=0.5
    ros2 launch duburi_vision cameras_.launch.py rqt:=false           # headless
    ros2 launch duburi_vision cameras_.launch.py device_param_type:=string device:=/dev/video2
    ros2 launch duburi_vision cameras_.launch.py with_tracking:=true  # enable ByteTrack + Kalman
    ros2 launch duburi_vision cameras_.launch.py model:=gate_flare_v1 classes:=gate,flare

    # Run on a pre-recorded video instead of a live webcam:
    ros2 launch duburi_vision cameras_.launch.py video_file:=/path/to/pool_run.mp4
    ros2 launch duburi_vision cameras_.launch.py video_file:=/tmp/gate.mp4 classes:=gate loop:=false
"""

from launch                       import LaunchDescription
from launch.actions               import DeclareLaunchArgument
from launch.conditions            import IfCondition
from launch.substitutions         import LaunchConfiguration, PythonExpression
from launch_ros.actions           import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('camera',        default_value='laptop'),
        DeclareLaunchArgument('device',        default_value='0'),
        DeclareLaunchArgument('width',         default_value='640'),
        DeclareLaunchArgument('height',        default_value='480'),
        DeclareLaunchArgument('fps',           default_value='30'),
        DeclareLaunchArgument('video_file',    default_value='',
                              description='Path to a video file; when set, overrides webcam source'),
        DeclareLaunchArgument('loop',          default_value='true',
                              description='Loop the video file when it reaches EOF (video_file only)'),
        DeclareLaunchArgument('model',         default_value='yolo26_nano_pretrained',
                              description='Model name (from models/) or path to .pt file'),
        DeclareLaunchArgument('cls_device',    default_value='cuda:0'),
        DeclareLaunchArgument('classes',       default_value='person',
                              description='CSV class names or indices to detect; empty = all'),
        DeclareLaunchArgument('conf',          default_value='0.35'),
        DeclareLaunchArgument('iou',           default_value='0.5'),
        DeclareLaunchArgument('rqt',           default_value='true',
                              description='Open rqt_image_view on image_debug'),
        DeclareLaunchArgument('with_tracking', default_value='false',
                              description='Start tracker_node (ByteTrack + Kalman) alongside detector'),
        DeclareLaunchArgument('track_buffer',  default_value='30',
                              description='tracker_node: frames to hold lost track before expiry'),
    ]

    cam_name   = LaunchConfiguration('camera')
    video_file = LaunchConfiguration('video_file')

    # source is 'video_file' when video_file arg is non-empty, else 'webcam'
    source_expr = PythonExpression(
        ["'video_file' if '", video_file, "' else 'webcam'"])

    camera_node = Node(
        package='duburi_vision', executable='camera_node', name='duburi_camera',
        output='screen',
        parameters=[{
            'source':          source_expr,
            'name':            cam_name,
            'frame_id':        'laptop_cam',
            'device':          LaunchConfiguration('device'),
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
            'device':              LaunchConfiguration('cls_device'),
            'classes':             LaunchConfiguration('classes'),
            'conf':                LaunchConfiguration('conf'),
            'iou':                 LaunchConfiguration('iou'),
            'publish_debug_image': True,
            'debug_image_hz':      10.0,
        }],
    )

    tracker_node = Node(
        package='duburi_vision', executable='tracker_node', name='duburi_tracker',
        output='screen',
        parameters=[{
            'camera':       cam_name,
            'track_buffer': LaunchConfiguration('track_buffer'),
        }],
        condition=IfCondition(LaunchConfiguration('with_tracking')),
    )

    image_topic = PythonExpression(["'/duburi/vision/' + '", cam_name, "' + '/image_debug'"])
    image_viewer = Node(
        package='rqt_image_view', executable='rqt_image_view',
        name='duburi_image_view', output='screen',
        arguments=[image_topic],
        condition=IfCondition(LaunchConfiguration('rqt')),
    )

    return LaunchDescription(args + [camera_node, detector_node, tracker_node, image_viewer])
