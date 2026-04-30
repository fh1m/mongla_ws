"""debug_view -- standalone OpenCV viewer on /duburi/vision/<cam>/image_debug.

Handy when the detector is already running in another terminal/launch and
you just want a viewer. Uses vision_display (no Qt/rqt needed).

Usage:
    ros2 launch duburi_vision debug_view.launch.py
    ros2 launch duburi_vision debug_view.launch.py camera:=forward
    ros2 launch duburi_vision debug_view.launch.py camera:=sim_front

Equivalent one-liner:
    ros2 run duburi_vision vision_display --ros-args -p camera:=forward
"""

from launch               import LaunchDescription
from launch.actions       import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions   import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('camera', default_value='forward'),
    ]

    viewer = Node(
        package='duburi_vision', executable='vision_display',
        name='duburi_image_view', output='screen',
        parameters=[{'camera': LaunchConfiguration('camera')}],
    )

    return LaunchDescription(args + [viewer])
