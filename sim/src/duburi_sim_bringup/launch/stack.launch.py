"""Bring up the unmodified duburi_ws stack against a running simulator.

Prerequisites: the simulator is already up

    ros2 launch duburi_sim_bringup sim.launch.py course:=sauvc26_qualification gui:=false

Then, in a second terminal (both workspaces sourced):

    ros2 launch duburi_sim_bringup stack.launch.py

For a control-only smoke test with no YOLO weights:

    ros2 launch duburi_sim_bringup stack.launch.py vision:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # sim_dvl = MAVLink AHRS heading + the Gazebo DVL's bottom-track position.
    # Position is what the *_dist verbs need; before the sim had a DVL they fell
    # back to a timed guess and reported success anyway (1.0 m commanded ->
    # 2.361 m travelled). Set mavlink_ahrs to reproduce the old heading-only
    # setup; the *_dist verbs then refuse rather than guessing.
    # The virtual payload board `payload_sim` creates. Passed explicitly rather
    # than left at 'auto': auto scans USB for a CH340, finds nothing in sim, and
    # every fire() then fails quietly -- which is the exact gap payload_sim was
    # written to close.
    payload_arg = DeclareLaunchArgument(
        'payload_port',
        default_value=os.path.join(
            f'/tmp/duburi-{os.environ.get("USER", "user")}', 'payload'),
        description='Payload device. Defaults to the sim virtual board; pass '
                    '"auto" to scan USB instead.')

    # The virtual BNO085 that `bno085_sim` creates. Passed explicitly for the
    # same reason as payload_port: 'auto' scans USB for the ESP32-C3's
    # VID/PID (303a:1001), finds nothing in sim, and yaw_source:=bno085 dies
    # at startup -- which is what made the vehicle's own heading source
    # untestable here.
    bno_arg = DeclareLaunchArgument(
        'bno085_port',
        default_value=os.path.join(
            f'/tmp/duburi-{os.environ.get("USER", "user")}', 'bno085'),
        description='BNO085 device. Defaults to the sim virtual board; pass '
                    '"auto" to scan USB instead.')

    yaw_source_arg = DeclareLaunchArgument(
        'yaw_source', default_value='sim_dvl',
        description='sim_dvl (AHRS heading + Gazebo DVL position) | mavlink_ahrs')

    args = [
        DeclareLaunchArgument(
            'vision', default_value='true',
            description='Start the vision pipeline on BOTH sim cameras.',
        ),
        DeclareLaunchArgument(
            'model', default_value='gate_rescue_repair',
            description='Detector weights stem under duburi_vision/models.',
        ),
        DeclareLaunchArgument('models', default_value='gate=gate_rescue_repair'),
        DeclareLaunchArgument('active_model', default_value='gate'),
        DeclareLaunchArgument('classes', default_value='gate'),
        DeclareLaunchArgument('viewer', default_value='false'),
        DeclareLaunchArgument(
            'bottom_image_topic',
            default_value='/duburi/sim/bottom_camera/image_raw',
            description='Sim bottom camera -> the `downward` detector.'),
        DeclareLaunchArgument(
            'dwn_model', default_value='bin_fire_blood',
            description='Downward detector weights stem.'),
        DeclareLaunchArgument('dwn_models', default_value=''),
        DeclareLaunchArgument(
            'dwn_classes', default_value='fire,blood',
            description='NOTE: bin_fire_blood has NO "bin" class -- its embedded '
                        'names are {0: blood, 1: fire}. Asking for bin detects '
                        'nothing, silently.'),
        DeclareLaunchArgument(
            'device_cls', default_value='cuda:0',
            description='YOLO inference device. Set cpu on a host without CUDA: '
                        'in registry mode the detector RAISES and the node dies '
                        'rather than degrading.'),
        DeclareLaunchArgument(
            'image_topic',
            default_value='/duburi/sim/front_camera/image_raw',
            description='Gazebo front-camera topic from the sim bridge.',
        ),
    ]

    # Reuse duburi_manager's own bringup so we stay aligned with its defaults,
    # then override only what SITL needs. Vision is launched separately below
    # because bringup.launch.py has no `topic:=` argument, and camera:=sim_front
    # would name the detector /duburi_detector_sim_front while missions look for
    # /duburi_detector_forward.
    # SCOPED. IncludeLaunchDescription does NOT scope its launch_arguments, so
    # the `vision: 'false'` below (which correctly tells the MANAGER not to start
    # its own vision) leaked into this file's scope and overwrote our own
    # `vision` argument. The vision include further down then evaluated
    # IfCondition(LaunchConfiguration('vision')) as false and skipped itself --
    # silently, with no error, for any value of vision:=.
    #
    # That is why `vision:=true` never produced a single node here, and why every
    # doc in sim/.context tells operators to run --no-vision. The GroupAction is
    # what confines the leak.
    manager = GroupAction([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('duburi_manager'),
                'launch', 'bringup.launch.py',
            )
        ),
        launch_arguments={
            'mode': 'sim',
            'flight_controller': 'pixhawk',
            'yaw_source': LaunchConfiguration('yaw_source'),
            'vision': 'false',
            'dvl_auto_connect': 'false',
            'payload_port': LaunchConfiguration('payload_port'),
            'bno085_port': LaunchConfiguration('bno085_port'),
            # SITL's barometer cannot be calibrated: it ACKs
            # PREFLIGHT_CALIBRATION and then stops tracking depth (measured:
            # readback froze near 0 while the hull sat 1.2 m down, so surface()
            # CONFIRMED while submerged -- a false pass, worse than the hang it
            # replaced). mission_reset auto-runs that calibration, so it has to
            # be off here. The pool default is unchanged.
            'baro_calibration': 'false',
            'viewer': 'false',
        }.items(),
    )], scoped=True)

    # DUAL camera, not single. The sim publishes a front AND a bottom camera and
    # only the front one was ever wired, so half the competition -- the bin drop
    # and anything flown looking down -- could not be practised at all. Camera
    # names stay forward/downward because missions and the vision verbs resolve
    # /duburi/vision/<name>/* by those names (AUDIT.md:74).
    vision = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('duburi_vision'),
                'launch', 'vision_dual.launch.py',
            )
        ),
        launch_arguments={
            'fwd_topic': LaunchConfiguration('image_topic'),
            'dwn_topic': LaunchConfiguration('bottom_image_topic'),
            'fwd_model': LaunchConfiguration('model'),
            'fwd_models': LaunchConfiguration('models'),
            'fwd_classes': LaunchConfiguration('classes'),
            'dwn_model': LaunchConfiguration('dwn_model'),
            'dwn_models': LaunchConfiguration('dwn_models'),
            'dwn_classes': LaunchConfiguration('dwn_classes'),
            'device_cls': LaunchConfiguration('device_cls'),
            'viewer': LaunchConfiguration('viewer'),
            # Both detectors live from the start. vision_dual defaults `paused`
            # to true for the real vehicle (two USB cameras on one bus), but in
            # sim the frames are free and a paused detector just looks broken.
            'paused': 'false',
        }.items(),
        condition=IfCondition(LaunchConfiguration('vision')),
    )

    return LaunchDescription([yaw_source_arg, payload_arg, bno_arg] + args + [manager, vision])
