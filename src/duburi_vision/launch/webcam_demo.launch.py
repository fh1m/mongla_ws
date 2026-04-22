"""webcam_demo.launch.py -- DEPRECATED. Use cameras_.launch.py instead.

This stub delegates to cameras_.launch.py so older scripts keep working.
"""

from launch                              import LaunchDescription
from launch.actions                      import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.launch_description_sources  import PythonLaunchDescriptionSource
from launch.substitutions               import LaunchConfiguration, ThisLaunchFileDir
from ament_index_python.packages        import get_package_share_directory
import os


def generate_launch_description():
    share = get_package_share_directory('duburi_vision')
    cameras_launch = os.path.join(share, 'launch', 'cameras_.launch.py')

    args = [
        DeclareLaunchArgument('camera',        default_value='laptop'),
        DeclareLaunchArgument('device',        default_value='0'),
        DeclareLaunchArgument('width',         default_value='640'),
        DeclareLaunchArgument('height',        default_value='480'),
        DeclareLaunchArgument('fps',           default_value='30'),
        DeclareLaunchArgument('model',         default_value='yolo26_nano_pretrained'),
        DeclareLaunchArgument('cls_device',    default_value='cuda:0'),
        DeclareLaunchArgument('classes',       default_value='person'),
        DeclareLaunchArgument('conf',          default_value='0.35'),
        DeclareLaunchArgument('iou',           default_value='0.5'),
        DeclareLaunchArgument('rqt',           default_value='true'),
        DeclareLaunchArgument('with_tracking', default_value='false'),
        DeclareLaunchArgument('track_buffer',  default_value='30'),
    ]

    warn = LogInfo(msg='[DEPRECATED] webcam_demo.launch.py -- use cameras_.launch.py instead')

    delegate = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(cameras_launch),
        launch_arguments={
            'camera':        LaunchConfiguration('camera'),
            'device':        LaunchConfiguration('device'),
            'width':         LaunchConfiguration('width'),
            'height':        LaunchConfiguration('height'),
            'fps':           LaunchConfiguration('fps'),
            'model':         LaunchConfiguration('model'),
            'cls_device':    LaunchConfiguration('cls_device'),
            'classes':       LaunchConfiguration('classes'),
            'conf':          LaunchConfiguration('conf'),
            'iou':           LaunchConfiguration('iou'),
            'rqt':           LaunchConfiguration('rqt'),
            'with_tracking': LaunchConfiguration('with_tracking'),
            'track_buffer':  LaunchConfiguration('track_buffer'),
        }.items(),
    )

    return LaunchDescription(args + [warn, delegate])
