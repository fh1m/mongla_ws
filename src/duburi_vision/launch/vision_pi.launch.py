"""Two cameras + one Hailo detector process, on the Pi 5 + AI HAT+.

This is `vision_dual` for the Pi, and it differs in exactly three ways -- each
of which is a thing that was silently wrong before:

1. ONE DETECTOR PROCESS, not two. The chip allows one VDevice per process, so
   `vision_dual`'s second `detector_node` dies with
   HAILO_OUT_OF_PHYSICAL_DEVICES (74) and the two-camera path has never
   started here. `detector_dual_node` holds both, keeps both node names, and
   hands the chip's single activation back and forth (4.15 ms a swap; zero
   swaps under the mission model, where one camera is live and the other is
   paused).

2. conf DEFAULTS TO 0.15. Every other launch path ships 0.35-0.45, which is the
   CUDA number. INT8 costs ~0.08 of score without moving the box centre, so on
   this backend that is ~3x too tight -- the most likely single reason a
   detection is missed. Our HEFs are baked at 0.05 precisely so this point is
   reachable.

3. THE CALIBRATION IS ACTUALLY PASSED. No launch file in this repo passes it,
   so `CameraInfo.k` publishes all-zero and every pixel->bearing conversion
   silently falls back to a GUESSED field of view -- on a stack whose whole
   uplink contract is bearings. The measured intrinsics
   (63.8 deg air / 46.7 deg water, +-0.7) exist as a file and were reaching
   nothing.

    ros2 launch duburi_vision vision_pi.launch.py \\
        fwd_model:=gate_rescue_repair fwd_classes:=gate,rescue,repair \\
        dwn_model:=bin_fire_blood     dwn_classes:=fire,blood
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

# Framework chatter off, our own nodes at info. Same convention as the other
# launch files: a process default of `warn` with per-logger `info` pins.
_QUIET = ['--log-level', 'warn']


def _calib(name: str) -> str:
    """Path to a calibration JSON in the installed share, or '' if absent.

    Empty rather than a missing path: `camera_node` treats '' as "no
    calibration" and says so, while a path that does not exist reads as a
    configuration error for a camera nobody has calibrated yet.
    """
    try:
        p = os.path.join(get_package_share_directory('duburi_vision'),
                         'config', 'calibration', name)
        return p if os.path.exists(p) else ''
    except Exception:
        return ''


def generate_launch_description():
    args = [
        DeclareLaunchArgument('fwd_profile', default_value='pi_forward'),
        DeclareLaunchArgument('dwn_profile', default_value='pi_downward'),
        DeclareLaunchArgument('fwd_model',   default_value='gate_rescue_repair'),
        DeclareLaunchArgument('dwn_model',   default_value='bin_fire_blood'),
        DeclareLaunchArgument('fwd_models',  default_value=''),
        DeclareLaunchArgument('dwn_models',  default_value=''),
        DeclareLaunchArgument('fwd_classes', default_value=''),
        DeclareLaunchArgument('dwn_classes', default_value=''),
        DeclareLaunchArgument(
            'conf', default_value='0.15',
            description='INT8 operating point. NOT the CUDA path\'s 0.35-0.45 '
                        '-- see the module docstring.'),
        DeclareLaunchArgument('max_det',   default_value='100'),
        DeclareLaunchArgument('imgsz',     default_value='640'),
        DeclareLaunchArgument(
            'paused', default_value='false',
            description='Start both detectors paused. The mission resumes the '
                        'one it needs; leaving BOTH live makes them compete '
                        'for the chip (~35 Hz each instead of ~98).'),
        DeclareLaunchArgument('viewer',    default_value='false'),
        DeclareLaunchArgument('tracking',  default_value='true'),
        DeclareLaunchArgument('tracker_type', default_value='ocsort'),
        DeclareLaunchArgument(
            'fwd_calibration',
            default_value=_calib('pi_forward_1280x720.json')),
        DeclareLaunchArgument('dwn_calibration', default_value=''),
        DeclareLaunchArgument('fwd_device_path', default_value=''),
        DeclareLaunchArgument('dwn_device_path', default_value=''),
    ]

    def camera(camera_name: str, profile_arg: str, calib_arg: str,
               device_path_arg: str) -> Node:
        return Node(
            package='duburi_vision', executable='camera_node',
            name=f'duburi_camera_{camera_name}', output='screen',
            ros_arguments=_QUIET,
            parameters=[{
                'profile':     LaunchConfiguration(profile_arg),
                'name':        camera_name,
                'device_path': LaunchConfiguration(device_path_arg),
                'calibration': LaunchConfiguration(calib_arg),
            }],
        )

    detectors = Node(
        package='duburi_vision', executable='detector_dual_node',
        name='duburi_detector_dual', output='screen', ros_arguments=_QUIET,
        parameters=[{
            'fwd_model_path': LaunchConfiguration('fwd_model'),
            'dwn_model_path': LaunchConfiguration('dwn_model'),
            'fwd_models':     LaunchConfiguration('fwd_models'),
            'dwn_models':     LaunchConfiguration('dwn_models'),
            'fwd_classes':    LaunchConfiguration('fwd_classes'),
            'dwn_classes':    LaunchConfiguration('dwn_classes'),
            'fwd_conf':       LaunchConfiguration('conf'),
            'dwn_conf':       LaunchConfiguration('conf'),
            'imgsz':          LaunchConfiguration('imgsz'),
            'max_det':        LaunchConfiguration('max_det'),
            'paused':         LaunchConfiguration('paused'),
            # The Hailo backend ignores it (`**_ignored`), but the value has to
            # be SOMETHING that is not 'cuda:0': `gpu.select_device` raises on a
            # Pi, and the factory builds the backend from the model extension,
            # not from this.
            'device':         'cpu',
            'half':           False,
        }],
    )

    def tracker(camera_name: str) -> Node:
        return Node(
            package='duburi_vision', executable='tracker_node',
            name=f'duburi_tracker_{camera_name}', output='screen',
            ros_arguments=_QUIET,
            parameters=[{'camera': camera_name,
                         'tracker_type': LaunchConfiguration('tracker_type')}],
            condition=IfCondition(LaunchConfiguration('tracking')),
        )

    return LaunchDescription(args + [
        camera('forward',  'fwd_profile', 'fwd_calibration', 'fwd_device_path'),
        camera('downward', 'dwn_profile', 'dwn_calibration', 'dwn_device_path'),
        detectors,
        tracker('forward'),
        tracker('downward'),
        Node(package='duburi_vision', executable='vision_display',
             name='duburi_display', output='screen',
             parameters=[{'camera': 'forward'}],
             condition=IfCondition(LaunchConfiguration('viewer'))),
    ])
