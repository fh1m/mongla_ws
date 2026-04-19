"""debug_view -- standalone rqt_image_view on /duburi/vision/<cam>/image_debug.

Handy when the detector is already running in another terminal/launch and
you just want a viewer:

    ros2 launch duburi_vision debug_view.launch.py
    ros2 launch duburi_vision debug_view.launch.py camera:=sim_front
"""

from launch                 import LaunchDescription
from launch.actions         import DeclareLaunchArgument
from launch.substitutions   import LaunchConfiguration, PythonExpression
from launch_ros.actions     import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('camera', default_value='laptop'),
    ]
    cam_name = LaunchConfiguration('camera')
    image_topic = PythonExpression(["'/duburi/vision/' + '", cam_name, "' + '/image_debug'"])

    viewer = Node(
        package='rqt_image_view', executable='rqt_image_view',
        name='duburi_image_view', output='screen',
        arguments=[image_topic],
    )

    return LaunchDescription(args + [viewer])
