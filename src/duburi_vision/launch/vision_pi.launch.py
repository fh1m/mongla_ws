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
        # PER CAMERA, because they do not run at the same rate: measured on the
        # Pi, forward 55 Hz (Hailo-bound) and downward 15 Hz (the Fantech unit
        # caps there). Every coast window is sized from this, so one shared
        # value guarantees one of the two trackers has a wrong coast -- and a
        # tracker with a truncated coast still publishes and looks healthy.
        # The node measures the real rate and warns if these are off.
        DeclareLaunchArgument('fwd_frame_rate', default_value='55.0'),
        DeclareLaunchArgument('dwn_frame_rate', default_value='15.0'),
        DeclareLaunchArgument(
            'fwd_calibration',
            default_value=_calib('pi_forward_1280x720.json')),
        DeclareLaunchArgument('dwn_calibration', default_value=''),
        # 60, AND CAPPING IS WHAT MAKES IT FASTER -- which is the opposite of
        # what the number looks like. The profile asks for 210, the camera
        # delivers ~68, and the detector consumes ~30: every surplus frame is
        # an MJPEG decode and a ROS publish spent on an image that `_on_image`
        # immediately drops from its single-slot queue. Measured on the Pi,
        # both cameras live, two runs each:
        #
        #     uncapped   forward 20.2 / 21.3 det/s      downward 16.1 / 15.7
        #     capped 60  forward 32.3 / 31.2 det/s      downward 15.4 / 14.8
        #
        # +53 % on the camera we steer on, and the other camera unchanged
        # within run-to-run spread. 30 was also measured (26.9 fwd / 16.7 dwn):
        # it buys the downward camera a little and costs the forward one more.
        #
        # 0 = the profile decides, which is the right value on any machine
        # whose camera is not outrunning its detector.
        DeclareLaunchArgument('fwd_fps', default_value='60'),
        DeclareLaunchArgument('dwn_fps', default_value='0'),
        DeclareLaunchArgument('fwd_device_path', default_value=''),
        DeclareLaunchArgument('dwn_device_path', default_value=''),
    ]

    def camera(camera_name: str, profile_arg: str, calib_arg: str,
               device_path_arg: str, fps_arg: str) -> Node:
        return Node(
            package='duburi_vision', executable='camera_node',
            name=f'duburi_camera_{camera_name}', output='screen',
            ros_arguments=_QUIET,
            parameters=[{
                'profile':     LaunchConfiguration(profile_arg),
                'name':        camera_name,
                'device_path': LaunchConfiguration(device_path_arg),
                'calibration': LaunchConfiguration(calib_arg),
                'fps':         LaunchConfiguration(fps_arg),
            }],
        )

    # NO `name=` HERE, DELIBERATELY. Launch implements it as `-r __node:=`,
    # which is PROCESS-wide: setting it renamed all THREE nodes in this process
    # to `duburi_detector_dual` and ros2 warned about duplicate names. The
    # launcher's own default name is already `duburi_detector_dual`, and the two
    # detectors name themselves.
    detectors = Node(
        package='duburi_vision', executable='detector_dual_node',
        output='screen', ros_arguments=_QUIET,
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

    def tracker(camera_name: str, rate_arg: str) -> Node:
        return Node(
            package='duburi_vision', executable='tracker_node',
            name=f'duburi_tracker_{camera_name}', output='screen',
            ros_arguments=_QUIET,
            parameters=[{'camera': camera_name,
                         'tracker_type': LaunchConfiguration('tracker_type'),
                         'frame_rate': LaunchConfiguration(rate_arg)}],
            condition=IfCondition(LaunchConfiguration('tracking')),
        )

    return LaunchDescription(args + [
        camera('forward',  'fwd_profile', 'fwd_calibration', 'fwd_device_path',
               'fwd_fps'),
        camera('downward', 'dwn_profile', 'dwn_calibration', 'dwn_device_path',
               'dwn_fps'),
        detectors,
        tracker('forward',  'fwd_frame_rate'),
        tracker('downward', 'dwn_frame_rate'),
        Node(package='duburi_vision', executable='vision_display',
             name='duburi_display', output='screen',
             parameters=[{'camera': 'forward'}],
             condition=IfCondition(LaunchConfiguration('viewer'))),
    ])
