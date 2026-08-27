"""Bridge Gazebo topics into the /duburi/sim namespace that duburi_ws expects.

Deliberately small. All vehicle control reaches ArduSub over MAVLink on UDP
14550, not across this bridge, so the only things that have to cross are camera
images, their calibration, ground truth and the clock.

Also launches underwater_fx, which publishes image_fx alongside the raw
contract topics (raw topics stay untouched).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

FRONT_IMAGE = '/duburi/sim/front_camera/image_raw'
BOTTOM_IMAGE = '/duburi/sim/bottom_camera/image_raw'
FRONT_INFO = '/duburi/sim/front_camera/camera_info'
BOTTOM_INFO = '/duburi/sim/bottom_camera/camera_info'
GROUND_TRUTH = '/duburi/sim/ground_truth'


def generate_launch_description():
    vehicle_name = LaunchConfiguration('vehicle_name')
    fx = LaunchConfiguration('fx')

    odom_gz_topic = ['/model/', vehicle_name, '/odometry']
    odom_spec = odom_gz_topic + ['@nav_msgs/msg/Odometry[gz.msgs.Odometry']

    fx_params = os.path.join(
        get_package_share_directory('duburi_sim_bridge'),
        'config',
        'underwater_fx.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'vehicle_name',
            default_value='duburi',
            description='Vehicle instance name, as set by the course YAML.',
        ),
        DeclareLaunchArgument(
            'fx',
            default_value='true',
            description='Launch underwater_fx (image_fx topics).',
        ),

        Node(
            package='ros_gz_image',
            executable='image_bridge',
            name='duburi_sim_image_bridge',
            arguments=['/front_camera', '/bottom_camera'],
            remappings=[
                ('/front_camera', FRONT_IMAGE),
                ('/bottom_camera', BOTTOM_IMAGE),
            ],
            output='screen',
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='duburi_sim_bridge',
            arguments=[
                '/front_camera/camera_info@sensor_msgs/msg/CameraInfo['
                'gz.msgs.CameraInfo',
                '/bottom_camera/camera_info@sensor_msgs/msg/CameraInfo['
                'gz.msgs.CameraInfo',
                odom_spec,
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            ],
            remappings=[
                ('/front_camera/camera_info', FRONT_INFO),
                ('/bottom_camera/camera_info', BOTTOM_INFO),
                (odom_gz_topic, GROUND_TRUTH),
            ],
            output='screen',
        ),

        Node(
            package='duburi_sim_bridge',
            executable='underwater_fx',
            name='underwater_fx',
            parameters=[fx_params],
            output='screen',
            condition=IfCondition(fx),
        ),
        # DVL. NOT a parameter_bridge entry: ros_gz_bridge has no
        # gz.msgs.DVLVelocityTracking conversion and ros_gz_interfaces has no
        # matching .msg, so this speaks gz-transport directly.
        #
        # Observability only. The control path (duburi_sensors' SimDvlSource)
        # subscribes to the gz topic itself, so the *_dist verbs keep working
        # even if this node is down -- deliberately, so a debugging tool can
        # never be load-bearing for vehicle control.
        Node(
            package='duburi_sim_bridge',
            executable='dvl_bridge',
            name='duburi_sim_dvl_bridge',
            output='screen',
        ),
    ])
