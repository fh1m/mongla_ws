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
from launch_ros.parameter_descriptions import ParameterValue

# Framework chatter off, our own nodes at info. Same convention as the other
# launch files: a process default of `warn` with per-logger `info` pins.
# Process default `warn` to silence rcl/rmw framework chatter, then the nodes
# we actually want pinned back to `info` BY NAME.
#
# The pin was missing here and the process default alone silenced two things
# the operator depends on:
#
#   * the always-on `[ offset lat=.. depth=..px ]` alignment readout, which is
#     the pool-day bearing telemetry and is emitted at INFO by
#     `detector_node._log_alignment`
#   * the 0.1 Hz chip-efficiency line, which reports whether the chip is only
#     ever seeing the freshest frame -- visible ONLY when it was broken,
#     because the healthy case logs at info and the failure at warn
#
# `vision.launch.py` has always pinned its detector; this launch -- the one
# the vehicle actually runs -- did not. Every node in the composed process
# needs naming individually, because a log level is per-LOGGER and the
# composed process holds five of them.
_QUIET = ['--log-level', 'warn']
for _n in ('duburi_detector_dual',
           'duburi_detector_forward', 'duburi_detector_downward',
           'duburi_camera_forward', 'duburi_camera_downward'):
    _QUIET += ['--log-level', f'{_n}:=info']


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
        # Which of `*_models` is live. A COMMA-SEPARATED value runs several on
        # every frame -- `fwd_active:=gate_rescue_repair,gate_seg` gives
        # detection and segmentation merged into one detection array and one
        # contour message. The first name stays "the" model: it owns the class
        # filter and the published vision_info. Empty means the registry's
        # first entry, which is what shipped before this existed.
        #
        # ⛔ REQUIRES A REGISTRY. `*_active` selects FROM `*_models`, so both
        # must be given and every name must appear in the registry:
        #
        #     fwd_models:=gate_rescue_repair,yolov8n_seg \
        #     fwd_active:=gate_rescue_repair,yolov8n_seg
        #
        # A single-model launch (`fwd_model:=`) has no registry to select from
        # and the node REFUSES any `active_model` that is not the loaded stem.
        # Verified on the vehicle: the registry form brings up both models on
        # the forward camera beside the downward detector -- three network
        # groups resident -- and logs the cost warning.
        #
        # ⚠ Not free. The accelerator runs one graph at a time and each
        # handover costs ~4 ms, so two models is the SUM plus the swaps --
        # measured 37.5 Hz for a pair against 95 Hz for one.
        DeclareLaunchArgument('fwd_active',  default_value=''),
        DeclareLaunchArgument('dwn_active',  default_value=''),
        # The outline topic. On by default and cheap -- measured 0.075 ms for
        # one box and 0.46 ms for three masks against a ~10.5 ms frame -- but
        # switchable, because an evidence topic should be a parameter rather
        # than a branch.
        DeclareLaunchArgument('contours',    default_value='true'),
        DeclareLaunchArgument('fwd_classes', default_value=''),
        DeclareLaunchArgument('dwn_classes', default_value=''),
        DeclareLaunchArgument(
            'conf', default_value='0.15',
            description='INT8 operating point. NOT the CUDA path\'s 0.35-0.45 '
                        '-- see the module docstring.'),
        DeclareLaunchArgument('max_det',   default_value='100'),
        # UNDERWATER CONTRAST PREPROCESSING -- 'clahe' or 'off'.
        # Measured on real RoboSub 2025 footage: target presence 10.7 % ->
        # 56.2 % on the blurry gate approach, no cost on footage that already
        # works. 3.78 ms/frame on the Pi (77 Hz -> ~46), which is why it is
        # opt-in rather than default.
        # NOT 'off' -- launch coerces that literal to boolean False and the
        # composed process dies at declare_parameter with
        # InvalidParameterTypeException. 'none' means the same thing to
        # `make_preprocessor` and survives the round trip as a string.
        # ONE WORD for the water, instead of four interacting knobs. Every
        # value in a profile is measured; `detection/profiles.py` names the
        # measurement beside each. An explicit knob still overrides it.
        #
        #   vision:=murky   green, low visibility
        #   vision:=clear   good visibility
        #   vision:=close   alignment / docking / firing
        #   vision:=fast    maximum pipeline rate (default)
        DeclareLaunchArgument('vision', default_value='fast'),
        # 'auto' = not set -> the profile decides. A concrete value here
        # would override every profile, which is how `vision:=murky`
        # once logged that it had applied while changing nothing.
        DeclareLaunchArgument('preprocess', default_value='auto'),
        # -1 = not set -> the profile decides. An int, not a bool, because a
        # bool has no third state and a profile could then never turn it on.
        DeclareLaunchArgument('range_crop', default_value='-1'),
        DeclareLaunchArgument('preprocess_clip', default_value='0.0'),
        DeclareLaunchArgument('imgsz',     default_value='640'),
        DeclareLaunchArgument(
            'paused', default_value='true',
            description='Start both detectors paused. The mission resumes the '
                        'one it needs; leaving BOTH live makes them compete '
                        'for the chip (~35 Hz each instead of ~98). '
                        'paused:=false to watch both streams with no mission.'),
        DeclareLaunchArgument('viewer',    default_value='false'),
        DeclareLaunchArgument('tracking',  default_value='true'),
        DeclareLaunchArgument('tracker_type', default_value='ocsort'),
        # Exposed so the A/B is RUNNABLE. `kalman_adaptive_noise` is read at
        # node construction, so `ros2 param set` cannot flip it afterwards --
        # and an arg the launch file does not declare is SILENTLY DROPPED, so
        # `:=false` read back as True and both arms of the A/B were identical.
        # Third appearance of that trap; the fix is to declare the argument.
        DeclareLaunchArgument('kalman_adaptive_noise', default_value='true'),
        # PER CAMERA, because they do not run at the same rate. Every coast
        # window is sized from this, so one shared value guarantees one of the
        # two trackers has a wrong coast -- and a tracker with a truncated
        # coast still publishes and looks healthy.
        #
        # Full configuration, both cameras live with tracking on, taken from
        # the TRACKER'S OWN in-process count -- which is the only trustworthy
        # source on this machine. Two earlier values were wrong for two
        # different reasons and the check caught both:
        #
        #   55  the standalone figure with the other camera paused, which is
        #       not how the stack runs
        #   30  what `ros2 topic hz` reported -- and that tool's own subscriber
        #       load depresses the stream it measures. It read 27.7 det/s on
        #       the same run the tracker measured 49.
        #
        # The 1.8x gap is worth carrying: `ros2 topic hz` UNDER-REPORTS on a
        # loaded Pi, so every rate quoted from it is a floor, not a figure.
        # ⛔ THESE SIZE THE COAST WINDOW, AND BOTH WERE STALE.
        # `tracker_node` converts its `max_predict_s` (1.5 s) into FRAMES as
        # ceil(max_predict_s * frame_rate), and that frame count is the
        # outermost rung of the coast ladder
        # (_freshness 0.4 < vision.coast_s 0.8 < lost_grace_s 1.0 < THIS).
        # Under-estimating frame_rate shortens the outer rung until it falls
        # BELOW an inner one, and the coast then truncates early -- a target
        # dropped mid-lock, while the tracker still publishes and looks
        # healthy. The node's own comment records this happening once already,
        # "made live by perception getting 5x faster".
        #
        # It went stale again on 2026-09-09 when pi_forward's fps went 15->60.
        # Measured live on the vehicle with the full vision_pi stack up, off
        # the node's own rate check:
        #
        #     forward  detections 30 Hz  (arg said 49 -- over, harmless)
        #     downward detections 32 Hz  (arg said 15 -- UNDER, and 1.5*15/32
        #                                 = 0.70 s of coast against a 0.8 s
        #                                 vision.coast_s: truncating)
        #
        # Set to the measured rates, so max_predict_s means 1.5 s of real
        # time. `frame_rate_warn_ratio` (1.5) still catches the next drift at
        # runtime -- that warning is what surfaced this.
        DeclareLaunchArgument('fwd_frame_rate', default_value='30.0'),
        DeclareLaunchArgument('dwn_frame_rate', default_value='32.0'),
        # ⛔ THESE WERE THE WRONG WAY ROUND, and both halves were live.
        # The only calibration we hold declares itself, in its own metadata,
        # as `pi_test_global_shutter (Microdia USB)` -- USB vendor 0c45,
        # which is the SONIX unit, which is the DOWNWARD camera. It was
        # committed on 2026-09-03, FOUR DAYS BEFORE the udev rules were found
        # to have the two cameras swapped, so it was named for the camera the
        # system then believed it was looking at.
        #
        # The consequence ran both ways. The FORWARD Fantech published
        # CameraInfo with a 63.82 deg HFOV that belongs to a different lens,
        # so every pixel->bearing on the srot vision uplink was computed with
        # the wrong focal length. And the DOWNWARD camera -- the DVL, the one
        # whose intrinsics round 38 measured a 3.08 % axis asymmetry to fix --
        # got NO calibration at all, so `flow_node` fell back to one focal
        # length and the frame centre. The fix was verified in a tool that
        # passed the path by hand and never reached the launch.
        #
        # The Fantech is UNCALIBRATED and that is said, not implied by an
        # empty default that looks like an oversight -- but it is now WIRED
        # BY NAME rather than hardcoded to ''. `_calib` returns '' when the
        # file is absent, so this is the uncalibrated state today AND becomes
        # live the moment `fov_solve.py --install` writes the file. Nobody has
        # to remember to come back and change a launch default, which is the
        # step that would otherwise be forgotten between calibrating the
        # camera and the calibration actually reaching it -- the exact class
        # of gap that produced the wrong-camera bug above.
        DeclareLaunchArgument(
            'dwn_calibration',
            default_value=_calib('pi_downward_1280x720.json')),
        DeclareLaunchArgument(
            'fwd_calibration',
            default_value=_calib('pi_forward_1280x720.json')),
        # THE DVL. flow_node existed only as a setup.py entry point -- in no
        # launch file at all -- so the bottom-camera velocity sensor had to be
        # started by hand, which on a pool deck means it does not get started.
        # OFF by default because it needs `pool_depth_m`, which it REFUSES to
        # guess: height is a clean multiplier on every velocity it emits.
        DeclareLaunchArgument('flow', default_value='false'),
        # THE LOCK LADDER. Built, measured, and in no launch file until now --
        # so the follower and XFeat anchor had never run in a mission. It
        # publishes `<ns>/lock`, NEVER `/detections`: a followed or anchored
        # box on the detector's topic is indistinguishable from something the
        # detector saw, and every consumer including the HUD would report a
        # detection that never happened.
        #
        # ⛔ STARTING IT DOES NOT STEER THE VEHICLE. The control loop consults
        # the ladder only when `vision.lock_s > 0`, and that stays 0. This
        # publishes the evidence so it can be watched on the deck before
        # anything acts on it -- the same staging the firmware vision uplink
        # uses (echo the numbers, confirm they match, THEN actuate).
        # DEFAULT OFF, DELIBERATELY. Measured A/B on the vehicle: the ladder
        # costs 23 % of the forward detection rate (30.23 -> 23.14 Hz) and
        # 30 % of the downward (42.71 -> 29.82). `vision.lock_s` is 0, so the
        # control loop does not read `/lock` -- paying a quarter of perception
        # for evidence nothing consumes is the wrong trade. Turn BOTH on
        # together: `lock:=true` plus `vision.lock_s > 0`, once the deck has
        # watched /lock and the rungs agree with what the operator sees.
        DeclareLaunchArgument(
            'lock', default_value='false',
            description='Run the lock ladder (follower + XFeat anchor) on the '
                        'forward camera, publishing <ns>/lock. Control ignores '
                        'it until vision.lock_s > 0.'),
        DeclareLaunchArgument(
            'lock_class', default_value='',
            description='Class the ladder locks onto. Empty = whatever the '
                        'detector is publishing.'),
        DeclareLaunchArgument(
            'dwn_lock_class', default_value='',
            description='Class the DOWNWARD ladder follows. Empty = any '
                        'class, and no 6-DoF pose (the geometry table is '
                        'keyed by class).'),
        DeclareLaunchArgument(
            'pool_depth_m', default_value='nan',
            description='Water depth in metres. REQUIRED with flow:=true -- '
                        'the node refuses to publish velocity without it.'),
        DeclareLaunchArgument(
            'medium', default_value='water',
            description="The medium the VEHICLE is in. 'water' engages the "
                        "flat-port rectification in flow_node, lock_node AND "
                        "pnp_node; 'air' for a dry bench run. Was "
                        "'flow_medium' when only the flow node read it -- one "
                        "value, so one argument, or the three drift apart and "
                        "each reports a plausible number."),
        # 0 = the profile's own rate (210), AND THAT REVERSES THE 60 THIS
        # SHIPPED WITH LAST ROUND. The cap was measured correctly and is now
        # wrong, because the mailbox changed what a captured frame costs.
        #
        # The cap's evidence: uncapped 20.2/21.3 det/s, capped-60 32.3/31.2 --
        # +53 %. True at the time. `cap.read()` DECODED every frame back then,
        # so a camera outrunning its consumer burned a core on JPEGs nobody
        # used, and throttling the camera throttled the waste.
        #
        # The mailbox decoupled them. The pump COPIES (13 us) and does not
        # decode; `read()` decodes only the frames a consumer actually takes.
        # So capture rate is now a LATENCY knob rather than a throughput one --
        # frame age at dequeue is about one capture period. Measured with the
        # consumer pinned at 30 Hz and decode on demand:
        #
        #     capture  30    age 46.12 ms   p95 59.52    CPU 8.0 %
        #     capture  60    age 23.54 ms   p95 33.32    CPU 7.8 %
        #     capture 120    age 13.45 ms   p95 18.09    CPU 8.0 %
        #     capture 210    age  8.94 ms   p95 14.28    CPU 7.8 %
        #
        # 2.6x fresher for the SAME CPU. The 60 was costing 14.6 ms of age and
        # buying nothing once the decode stopped being per-captured-frame.
        #
        # MJPEG stays the format, and that is also measured rather than
        # inherited: YUYV needs no decode (0.55 ms vs 1.94) but 640x360 YUYV is
        # 450 kB a frame and the bus caps it at 35.4 Hz -- age 20 ms. Paying
        # 1.4 ms of CPU to halve the age is the right side of that trade.
        DeclareLaunchArgument('fwd_fps', default_value='0'),
        DeclareLaunchArgument('dwn_fps', default_value='0'),
        # CAPTURE FAST, PUBLISH AT THE CONSUMER'S RATE. These are different
        # knobs and conflating them cost a measured regression: uncapping
        # `fps` alone took the image age down (26.0 -> 23.6 ms) and pushed the
        # DETECTION age UP (47.5 -> 62.6), because every captured frame was
        # also PUBLISHED -- 94 messages a second of 691 kB each, which the
        # executor and DDS pay for whether or not anyone wanted them.
        #
        # The mailbox decouples capture from DECODE. This decouples capture
        # from TRANSPORT: the pump keeps the newest frame available at 210 Hz
        # for ~0 CPU, and the publish hands over whichever one is newest when
        # the consumer's period comes round. 0 = publish every captured frame.
        #
        # 40 because it is the measured knee, capture pinned at 210:
        #
        #     publish every frame (~107 Hz)   detections 55.3 ms  p95 79.0
        #     publish  60 Hz                  detections 36.6 ms  p95 51.3
        #     publish  40 Hz                  detections 31.7 ms  p95 39.6
        #
        # Slower publishing gives FRESHER detections, which is only paradoxical
        # if you think of the topic as a stream. It is a mailbox: the consumer
        # reads about 36 Hz whatever we publish, so anything above that is
        # 691 kB messages the executor serialises and the consumer discards --
        # work that delays the frame it does want. 40 is matched to the
        # consumer; lower would starve it.
        DeclareLaunchArgument('fwd_publish_hz', default_value='40'),
        DeclareLaunchArgument('dwn_publish_hz', default_value='0'),
        DeclareLaunchArgument('fwd_device_path', default_value=''),
        DeclareLaunchArgument('dwn_device_path', default_value=''),
    ]

    # NO SEPARATE CAMERA PROCESSES. The cameras live inside the same process
    # as the detectors now, so their frames reach inference by reference
    # instead of by topic -- see `detector_dual_node`. Their parameters arrive
    # here `fwd_cam_`/`dwn_cam_`-prefixed for the same reason the detectors'
    # do: launch's `name=`/`parameters=` are process-wide.

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
            'fwd_active_model': LaunchConfiguration('fwd_active'),
            'dwn_active_model': LaunchConfiguration('dwn_active'),
            'publish_contours': LaunchConfiguration('contours'),
            'fwd_classes':    LaunchConfiguration('fwd_classes'),
            'dwn_classes':    LaunchConfiguration('dwn_classes'),
            'fwd_conf':       LaunchConfiguration('conf'),
            'dwn_conf':       LaunchConfiguration('conf'),
            'imgsz':          LaunchConfiguration('imgsz'),
            'max_det':        LaunchConfiguration('max_det'),
            'vision_profile':  LaunchConfiguration('vision'),
            'preprocess':      LaunchConfiguration('preprocess'),
            'range_crop':      LaunchConfiguration('range_crop'),
            'preprocess_clip': LaunchConfiguration('preprocess_clip'),
            'paused':         LaunchConfiguration('paused'),
            # The Hailo backend ignores it (`**_ignored`), but the value has to
            # be SOMETHING that is not 'cuda:0': `gpu.select_device` raises on a
            # Pi, and the factory builds the backend from the model extension,
            # not from this.
            'device':         'cpu',
            'half':           False,

            'fwd_cam_profile':         LaunchConfiguration('fwd_profile'),
            'dwn_cam_profile':         LaunchConfiguration('dwn_profile'),
            'fwd_cam_device_path':     LaunchConfiguration('fwd_device_path'),
            'dwn_cam_device_path':     LaunchConfiguration('dwn_device_path'),
            'fwd_cam_calibration':     LaunchConfiguration('fwd_calibration'),
            'dwn_cam_calibration':     LaunchConfiguration('dwn_calibration'),
            'fwd_cam_fps':             LaunchConfiguration('fwd_fps'),
            'dwn_cam_fps':             LaunchConfiguration('dwn_fps'),
            'fwd_cam_publish_rate_hz': LaunchConfiguration('fwd_publish_hz'),
            'dwn_cam_publish_rate_hz': LaunchConfiguration('dwn_publish_hz'),
        }],
    )

    def tracker(camera_name: str, rate_arg: str) -> Node:
        return Node(
            package='duburi_vision', executable='tracker_node',
            name=f'duburi_tracker_{camera_name}', output='screen',
            ros_arguments=_QUIET,
            parameters=[{'camera': camera_name,
                         'tracker_type': LaunchConfiguration('tracker_type'),
                         'kalman_adaptive_noise':
                             LaunchConfiguration('kalman_adaptive_noise'),
                         'frame_rate': LaunchConfiguration(rate_arg),
                         # Clamp the tracker's gates to the detector's
                         # floor: above it NO track is created and
                         # /tracks stays empty while looking healthy.
                         'detector_conf': LaunchConfiguration('conf')}],
            condition=IfCondition(LaunchConfiguration('tracking')),
        )

    def ladder(camera_name: str, class_arg: str) -> Node:
        """The continuity ladder, PER CAMERA.

        It used to exist only for `forward`, hard-coded. Every downward task --
        the bin drop, the dropper alignment -- therefore steered with no
        gap-bridging at all, while the forward camera had three rungs. Nothing
        reported that; the capability was simply absent on one half of the
        vehicle, which is the same shape as the ladder itself being in no
        launch file.
        """
        return Node(
            package='duburi_vision', executable='lock_node',
            name=f'duburi_lock_{camera_name}', output='screen',
            parameters=[{
                'camera':       camera_name,
                'target_class': LaunchConfiguration(class_arg),
                'follow':       True,
                # The anchor rung is asked for unconditionally and DEGRADES
                # BY ITSELF: lock_node logs `anchor DISABLED ... the follower
                # rung still runs` when no xfeat_*.onnx resolves. A second
                # launch flag for it would only be a way to disable a rung
                # that already disables itself.
                'anchor':       True,
                # lock_node still rectifies its OBJECT points (reference
                # pixels scaled to metres); pnp_node rectifies the image side.
                # Two nodes, one value, one launch argument.
                'medium':       LaunchConfiguration('medium'),
            }],
            condition=IfCondition(LaunchConfiguration('lock')),
        )

    def solver(camera_name: str) -> Node:
        """The PnP solver, PER CAMERA -- started with the ladder that feeds it.

        Gated on the SAME `lock` condition on purpose. `lock_node` publishes
        correspondences and no longer solves, so a ladder without its solver
        would leave `target_pose` silent while every node looked healthy --
        the capability-present-but-unreachable failure this stack keeps
        hitting.
        """
        return Node(
            package='duburi_vision', executable='pnp_node',
            name=f'duburi_pnp_{camera_name}', output='screen',
            parameters=[{
                'camera': camera_name,
                'medium': LaunchConfiguration('medium'),
            }],
            condition=IfCondition(LaunchConfiguration('lock')),
        )

    def pose_fuse(camera_name):
        """Fuse the solver's per-frame poses into one answer with its support.

        Same `lock` gate as the solver that feeds it, for the same reason: a
        fuser with no pose stream publishes nothing while looking healthy.
        Publishes BESIDE `/target_pose`, never over it -- comparing an
        estimator against its own fused output is how a regression in either
        becomes visible.
        """
        return Node(
            package='duburi_vision', executable='pose_fuse_node',
            name=f'duburi_pose_fuse_{camera_name}', output='screen',
            parameters=[{'camera': camera_name}],
            condition=IfCondition(LaunchConfiguration('lock')),
        )

    return LaunchDescription(args + [
        detectors,
        tracker('forward',  'fwd_frame_rate'),
        tracker('downward', 'dwn_frame_rate'),
        Node(package='duburi_vision', executable='flow_node',
             name='duburi_flow_velocity', output='screen',
             parameters=[{
                 'camera':       'downward',
                 'medium':       LaunchConfiguration('medium'),
                 'pool_depth_m': ParameterValue(
                     LaunchConfiguration('pool_depth_m'), value_type=float),
                 # The SAME calibration the downward camera_node gets. Passing
                 # them separately is how they came to disagree.
                 'calibration':  LaunchConfiguration('dwn_calibration'),
             }],
             condition=IfCondition(LaunchConfiguration('flow'))),
        ladder('forward',  'lock_class'),
        ladder('downward', 'dwn_lock_class'),
        solver('forward'),
        solver('downward'),
        pose_fuse('forward'),
        pose_fuse('downward'),
        Node(package='duburi_vision', executable='vision_display',
             name='duburi_display', output='screen',
             parameters=[{'camera': 'forward'}],
             condition=IfCondition(LaunchConfiguration('viewer'))),
    ])
