"""sim_demo -- subscribe to a Gazebo image topic and run YOLO26 on it.

Note on Gazebo: the bluerov2_gz model used today does NOT ship a camera
plugin. Either (a) add a <sensor type="camera"/> + ros_gz image_bridge
to the world before launching this, or (b) point the topic argument at
any Image publisher you already have (a re-published webcam works fine
for end-to-end smoke testing of the perception pipeline).

Usage:
    ros2 launch duburi_vision sim_demo.launch.py
    ros2 launch duburi_vision sim_demo.launch.py topic:=/my_other_image_topic
    ros2 launch duburi_vision sim_demo.launch.py camera:=sim_bottom \\
                                                  topic:=/duburi/sim/bottom_camera/image_raw
"""

from launch                 import LaunchDescription
from launch.actions         import DeclareLaunchArgument
from launch.conditions      import IfCondition
from launch.substitutions   import LaunchConfiguration, PythonExpression
from launch_ros.actions     import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('camera',  default_value='sim_front'),
        DeclareLaunchArgument('topic',   default_value='/duburi/sim/front_camera/image_raw'),
        DeclareLaunchArgument('model',   default_value='yolo26n.pt'),
        DeclareLaunchArgument('cls_device', default_value='cuda:0'),
        DeclareLaunchArgument('classes', default_value='person'),
        DeclareLaunchArgument('conf',    default_value='0.35'),
        DeclareLaunchArgument('iou',     default_value='0.5'),
        DeclareLaunchArgument('rqt',          default_value='true'),
        DeclareLaunchArgument('with_tracking', default_value='false',
                              description='Start tracker_node alongside detector'),
        DeclareLaunchArgument('track_buffer',  default_value='30'),
    ]

    cam_name = LaunchConfiguration('camera')

    # We DON'T need a camera_node here: the detector subscribes directly to
    # the Gazebo topic. If you want a /duburi/vision/<cam>/image_raw passthrough
    # too, run camera_node with source=ros_topic alongside.
    detector = Node(
        package='duburi_vision', executable='detector_node', name='duburi_detector',
        output='screen',
        parameters=[{
            'camera':              cam_name,
            'image_topic':         LaunchConfiguration('topic'),
            'model_path':          LaunchConfiguration('model'),
            'device':              LaunchConfiguration('cls_device'),
            'classes':             LaunchConfiguration('classes'),
            'conf':                LaunchConfiguration('conf'),
            'iou':                 LaunchConfiguration('iou'),
            'publish_debug_image': True,
            'debug_image_hz':      10.0,
        }],
    )

    tracker = Node(
        package='duburi_vision', executable='tracker_node', name='duburi_tracker',
        output='screen',
        parameters=[{
            'camera':       cam_name,
            'track_buffer': LaunchConfiguration('track_buffer'),
        }],
        condition=IfCondition(LaunchConfiguration('with_tracking')),
    )

    image_topic = PythonExpression(["'/duburi/vision/' + '", cam_name, "' + '/image_debug'"])
    viewer = Node(
        package='rqt_image_view', executable='rqt_image_view',
        name='duburi_image_view', output='screen',
        arguments=[image_topic],
        condition=IfCondition(LaunchConfiguration('rqt')),
    )

    return LaunchDescription(args + [detector, tracker, viewer])
