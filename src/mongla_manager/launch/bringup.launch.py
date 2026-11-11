"""bringup -- start the vehicle: the manager, localization, and optionally vision.

The one command for the vehicle (SROT board + Pi 5 + Hailo-8):

    # Control only. Talks to the board over USB-C; no cameras.
    ros2 launch mongla_manager bringup.launch.py

    # The vehicle with both cameras, the Hailo models and the lock ladder.
    ros2 launch mongla_manager bringup.launch.py vision:=true

    # ...plus downward-camera velocity. pool_depth_m is REQUIRED: without it
    # flow_node refuses and publishes quality 0 rather than guess the scale.
    ros2 launch mongla_manager bringup.launch.py vision:=true flow:=true \\
        pool_depth_m:=1.2

    # On the bench, in air: the optics change by ~1.33, so say so.
    ros2 launch mongla_manager bringup.launch.py vision:=true medium:=air

    # Batighor, the operator center: open http://mongla.local:8000/ in any browser.
    ros2 launch mongla_manager bringup.launch.py vision:=true batighor:=true

    # Single camera, .pt model on a dev box (footage replay; not the vehicle path).
    ros2 launch mongla_manager bringup.launch.py vision:=true \\
        vision_stack:=generic model:=gate_sharks conf:=0.35

Settings can also live in a file instead of on the command line -- see
`mongla_localization/launch_defaults.py`. The command line always wins:

    echo 'vision: true' > ~/.mongla/bringup.yaml
    ros2 launch mongla_manager bringup.launch.py         # vision is on

Node parameters (the vision gains and every other knob that is not an
argument) go in the same file, under `nodes:`. One file for the whole day.

Then, from another shell:

    ros2 run mongla_manager bringup_check                 # can it arm?
    ros2 run mongla_planner mongla arm
    ros2 run mongla_planner mission --list

Every argument, its default and its meaning:
`.claude/context/reference/commands.md` section 3 (generated from this file).
"""

from functools                  import partial

from launch                     import LaunchDescription
from launch.actions             import (DeclareLaunchArgument, GroupAction,
                                        IncludeLaunchDescription, LogInfo,
                                        OpaqueFunction, SetEnvironmentVariable)
from launch.conditions          import IfCondition
from launch.substitutions       import PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions       import LaunchConfiguration
from launch_ros.actions         import Node
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_directory

from mongla_localization import launch_defaults
from mongla_localization.launch_respawn import respawning


# Grace between SIGINT and SIGTERM for the manager. Above the manager's
# EMERGENCY_DISARM_S plus the brake, so a slow disarm confirmation is logged
# rather than killed mid-poll (a test compares the two).

# What a browser may do through foxglove_bridge. The bridge cannot tell one
# browser from another, and its defaults let every one of them call any
# service, set any node's parameters and publish on any topic -- which with
# include_hidden reaches /mongla/move/_action/send_goal and
# shield_max_depth_m, past the helm. So: drive only through the helm
# (mongla_batighor/helm.py), tune only the detector's live settings, publish
# nothing. test_the_bridge_only_lets_a_browser_drive_through_the_helm.
BRIDGE_LIMITS = {
    'capabilities': ['parameters', 'parametersSubscribe', 'services',
                     'connectionGraph', 'assets'],
    'service_whitelist': ['^/mongla/batighor/helm$'],
    'param_whitelist': [
        r'^/mongla_detector_(forward|downward)\.(conf|nms_floor|nms_iou|active_model|classes)$'],
    'client_topic_whitelist': ['^$'],
}

MANAGER_SIGTERM_TIMEOUT_S = 12

# The vehicle vision stack's own arguments, the ones this file does not
# declare for itself. Bringup re-declares each FROM `vision_pi.launch.py`
# (`arguments()` there, the one place their defaults are written) and passes it
# on explicitly, so every vision setting is an ordinary bringup argument: on
# the command line, in ~/.mongla/bringup.yaml, in the generated example, and
# checked by command_lint. Before this they were reachable only by the include
# leak described below, and bringup.yaml refused them as unknown.
# `test_bringup_settings_file` fails if this list and the two files disagree.
VISION_PI_PASSTHROUGH = (
    'fwd_profile', 'dwn_profile', 'fwd_models', 'dwn_models', 'fwd_active',
    'dwn_active', 'contours', 'assoc_conf', 'preprocess', 'range_crop', 'preprocess_clip',
    'tracking', 'tracker_type', 'kalman_adaptive_noise', 'fwd_frame_rate',
    'dwn_frame_rate', 'dwn_calibration', 'fwd_calibration', 'act_conf',
    'anchor_xfeat_hef', 'anchor_semi_dense', 'target_width_m_forward',
    'target_width_m_downward', 'dwn_lock_class', 'bank_forward', 'bank_downward',
    'fwd_fps', 'dwn_fps', 'fwd_publish_hz', 'dwn_publish_hz', 'replay',
    'fwd_device_path', 'dwn_device_path',
)


# Switches that START something inside the vision stack, so they do nothing
# while `vision` is false. Typed alone they used to be accepted without a word:
# `flow:=true` gave a vehicle with no velocity and no message.
VISION_ONLY = {'flow': 'true', 'viewer': 'true', 'replay': 'true'}


def vision_only_ignored(cfg: dict) -> list:
    """The vision-only switches that are on while `vision` is off."""
    if str(cfg.get('vision', 'false')).lower() in ('true', '1'):
        return []
    on = [f'{k}:={v}' for k, v in VISION_ONLY.items() if str(cfg.get(k, '')).lower() == v]
    if str(cfg.get('lock_class', '')).strip():
        on.append(f"lock_class:={cfg['lock_class']}")
    return on


def _warn_vision_only(context):
    ignored = vision_only_ignored(context.launch_configurations)
    if not ignored:
        return []
    return [LogInfo(msg=f"WARNING: {' '.join(ignored)} has no effect: vision is off. "
                        f"Add vision:=true (or `vision: true` in bringup.yaml).")]


def _vision_pi_arguments(path: str) -> list:
    """`VISION_PI_PASSTHROUGH`, declared exactly as vision_pi.launch.py does."""
    import importlib.util
    spec = importlib.util.spec_from_file_location('vision_pi_launch', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    by_name = {a.name: a for a in mod.arguments()}
    missing = [n for n in VISION_PI_PASSTHROUGH if n not in by_name]
    if missing:
        raise RuntimeError(f'bringup passes {missing} to vision_pi.launch.py, which '
                           f'no longer declares them')
    return [by_name[n] for n in VISION_PI_PASSTHROUGH]


def generate_launch_description():
    args = [
        # ---- the board ------------------------------------------------------ #
        DeclareLaunchArgument('mav_device', default_value='',
                              description="SROT serial device, '' = autodetect. A path "
                                          '(/dev/serial/by-id/...) or any pymavlink '
                                          'connection string'),
        DeclareLaunchArgument('payload_channels', default_value='',
                              description='OPTIONAL labels for the log, '
                                          '"<board_channel>:<name>", e.g. '
                                          '"9:torpedo_1, 11:dropper_1". Labels ONLY '
                                          '-- fire(N) always addresses board channel '
                                          'N; nothing here routes a shot'),
        # Deliberately verbose: below firmware behaviour rev 2 MOVE_STOP coasts and
        # this host carries no brake, so `stop` would not decelerate the hull.
        DeclareLaunchArgument('allow_fw_behaviour_mismatch', default_value='false',
                              description='Arm against firmware older than '
                                          'FW_BEHAVIOUR_REV_REQUIRED. Accepts an '
                                          'un-braked stop -- leave false'),
        DeclareLaunchArgument('vision',     default_value='false',
                              description='Start camera + detector alongside manager'),
        # WHICH vision stack. `pi` is the VEHICLE path: vision_pi.launch.py runs
        # BOTH cameras through detector_dual_node, wires the measured calibration
        # JSONs by name, and can start flow_node. `generic` is the single-camera
        # dev-box path (vision.launch.py) this file used to include
        # unconditionally -- which meant the documented pool-day command brought
        # up one camera, no calibration, and a model that is not on the vehicle.
        DeclareLaunchArgument('vision_stack', default_value='pi',
                              choices=['pi', 'generic'],
                              description='pi = both cameras + calibration '
                                          '(vision_pi.launch.py, the vehicle); '
                                          'generic = single camera, no calibration '
                                          '(vision.launch.py, dev box).'),
        # vision_pi's own `vision` argument is its PROFILE ('fast'|...), while
        # `vision` HERE is the boolean that decides whether vision starts at
        # all. Launch forwards configurations into an include, so `vision:=true`
        # arrived as the profile and killed detector_dual_node with
        #   InvalidParameterTypeException ... 'True' of type 'BOOL',
        #   expecting type 'STRING': vision_profile
        # Exposing it under a distinct name and passing it explicitly is what
        # actually fixes that -- the passed value wins over the inherited one.
        DeclareLaunchArgument('vision_profile', default_value='fast',
                              description="vision_stack:=pi -- vision_pi's own "
                                          "profile argument (its `vision:=`). "
                                          "Renamed here because `vision` is this "
                                          "file's boolean on/off switch."),
        DeclareLaunchArgument('fwd_model',   default_value='gate_sharks',
                              description='vision_stack:=pi -- forward-camera model stem.'),
        DeclareLaunchArgument('dwn_model',   default_value='bin_fire_blood',
                              description='vision_stack:=pi -- downward-camera model stem.'),
        DeclareLaunchArgument('fwd_classes', default_value='',
                              description='vision_stack:=pi -- forward class allowlist '
                                          '(empty = the model sidecar\'s full set).'),
        DeclareLaunchArgument('dwn_classes', default_value='',
                              description='vision_stack:=pi -- downward class allowlist.'),
        DeclareLaunchArgument('localization', default_value='true',
                              description='Run the invariant filter (mongla_'
                                          'localization). Read-only: it '
                                          'publishes /mongla/odom and commands '
                                          'nothing, so it is on by default. '
                                          'Set false to take it off the graph.'),
        DeclareLaunchArgument('flow',        default_value='false',
                              description='vision_stack:=pi -- start flow_node '
                                          '(downward-camera velocity). Pairs with the '
                                          'manager\'s position_source:=flow.'),
        # flow_node REFUSES to publish velocity until it is told the height to
        # the floor -- "a default here would turn an unknown SCALE into a
        # confident wrong speed". So `flow:=true` alone yields quality 0 and
        # nothing else; these two are what make the path usable, and they are
        # exposed here because bringup is where `flow:=true` is offered.
        DeclareLaunchArgument('pool_depth_min_m', default_value='nan',
                              description='flow:=true -- the venue\'s '
                                          'shallowest floor, metres (#35).'),
        DeclareLaunchArgument('pool_depth_max_m', default_value='nan',
                              description='flow:=true -- the venue\'s deepest '
                                          'floor, metres (#35).'),
        DeclareLaunchArgument('pool_depth_m', default_value='nan',
                              description='flow:=true -- metres from the DOWNWARD '
                                          'camera to the floor. Required: without '
                                          'it flow_node publishes quality 0 and '
                                          'refuses, by design.'),
        # --- feature switches: every one defaults to what the node ships, so a
        # plain launch is unchanged; flip one at a time to test it alone. The
        # full list and what each one gates: .claude/context/ROADMAP.md §9.
        DeclareLaunchArgument('velocity_uplink', default_value='false',
                              description='manager: RIEKF body velocity -> board '
                                          '(VISION_SPEED_ESTIMATE). Needs fw PR #23 to act.'),
        DeclareLaunchArgument('position_uplink', default_value='false',
                              description='manager: RIEKF pose -> board '
                                          '(VISION_POSITION_ESTIMATE).'),
        DeclareLaunchArgument('mixer_aware', default_value='true',
                              description='manager: srot vision frames fitted to the '
                                          'mixer yaw-first + saturation-aware anti-windup.'),
        DeclareLaunchArgument('zupt', default_value='true',
                              description='localization: zero-velocity updates when still.'),
        DeclareLaunchArgument('demand_aid', default_value='true',
                              description='localization: velocity from commanded demand '
                                          'when the floor goes blank.'),
        DeclareLaunchArgument('retrodict', default_value='false',
                              description='localization: apply flow/depth/fixes at the '
                                          'instant they describe, replaying later events.'),
        DeclareLaunchArgument('use_yaw', default_value='false',
                              description='localization: fuse the landmark heading anchor.'),
        DeclareLaunchArgument('caustics', default_value='true',
                              description='flow: sun-caustic erosion + bare-floor refusal.'),
        DeclareLaunchArgument('lane_lines', default_value='false',
                              description='flow: lane-line heading (mod 180) yaw bound.'),
        DeclareLaunchArgument('tile_m', default_value='0.0',
                              description='flow: pool tile size in metres; 0 = tile '
                                          'grating (height + yaw bound) OFF.'),
        DeclareLaunchArgument('medium',  default_value='water',
                              choices=['water', 'air'],
                              description='The medium the VEHICLE is in. Read by '
                                          'flow_node, lock_node AND pnp_node -- '
                                          'one value, one argument. `air` for a bench '
                                          'run, or the scale is off by ~1.33.'),
        # An empty lock_class means the ladder follows ANY detected class and
        # `/target_pose` refuses with 'target_width_m unset', because the
        # committed geometry table is keyed by class. `lock:=true` without this
        # is a ladder you switched on and cannot aim -- measured on the vehicle:
        # "[LOCK ] no committed width for 'person'... the 6-DoF pose will refuse".
        DeclareLaunchArgument('lock_class',  default_value='',
                              description='vision_stack:=pi, lock:=true -- the '
                                          'class the ladder follows. Empty = any '
                                          'class, and no 6-DoF pose.'),
        DeclareLaunchArgument('lock',        default_value='true',
                              description='vision_stack:=pi -- start lock_node '
                                          '(follower + XFeat anchor continuity ladder).'),
        # Exposed because the mission path is the one that pays for it. Two
        # live detectors alternate on one Hailo: measured 37.5 Hz per pair
        # against 95.3 Hz for a single resident model, and the chip logs
        # 'has taken the activation N times'. The mission resumes the detector
        # it needs (`use_camera`, a vision verb, or the first `detected()`), so
        # the default costs a mission nothing. Pass paused:=false to watch both
        # streams on the console with no mission running.
        DeclareLaunchArgument('paused',      default_value='true',
                              description='vision_stack:=pi -- start both detectors '
                                          'paused; the mission resumes the one it '
                                          'needs. false = both infer and compete.'),
        DeclareLaunchArgument('camera',     default_value='forward',
                              description='Camera ROLE -- names the topics and nodes '
                                          '(forward|downward). Change `camera_profile`, '
                                          'not this, to point a role at other hardware.'),
        # This default is the PI'S camera. `camera` once defaulted to a
        # generic USB profile, which is not the camera on the vehicle.
        # Measured on the Pi running the documented bringup line: 5.00 Hz and
        # CameraInfo.k all zero, because the `forward` profile carries no
        # `fourcc: MJPG` (YUYV is 3x slower on this unit) and no calibration
        # declares it. Both are silent -- the frame rate looks like "vision is
        # slow" and the zero K only bites when the srot vision uplink is
        # switched on.
        #
        # The role is NOT renamed: /mongla/vision/forward/... and
        # mongla_detector_forward are what 96 call sites and the DSL's own
        # `camera='forward'` default bind to. Only the hardware moves.
        DeclareLaunchArgument('camera_profile', default_value='pi_forward',
                              description='Camera HARDWARE profile (CAMERA_PROFILES in '
                                          'mongla_vision/config.py -- the loaded copy). '
                                          'Default pi_forward is the vehicle\'s Fantech.'),
        DeclareLaunchArgument('model',      default_value='gate_sharks',
                              description='vision_stack:=generic -- model stem in '
                                          'mongla_vision/models/ (a <stem>.yaml '
                                          'sidecar must sit beside it).'),
        DeclareLaunchArgument('models',     default_value='',
                              description='vision_stack:=generic -- multi-model '
                                          'registry, CSV name=stem: '
                                          '"gate=gate_sharks,bins=bin_fire_blood"'),
        DeclareLaunchArgument('active_model', default_value='',
                              description='Registry key to start with (requires models:="..." to be set)'),
        DeclareLaunchArgument('classes',    default_value='gate',
                              description='CSV class names for detector'),
        # 0.45 is gate_sharks' measured bar: 3.2 % false positives there
        # against 40.8 % at the old 0.15 (measured-bars §59, and the model's
        # sidecar `recommended_conf`). A different model needs its own bar --
        # read it from that model's sidecar, do not reuse this one.
        DeclareLaunchArgument('conf',       default_value='0.45',
                              description='Detector confidence floor. 0.45 is '
                                          "gate_sharks' measured bar; another "
                                          "model's bar is in its <stem>.yaml."),
        DeclareLaunchArgument('imgsz',      default_value='640',
                              description='Inference square size for a .pt. A .hef bakes '
                                          'its input size at compile time and ignores this.'),
        DeclareLaunchArgument('max_det',    default_value='100',
                              description='Post-NMS detection cap (runtime; lower toward ~10 if NMS '
                                          'is the FPS bottleneck on busy frames).'),
        # The vehicle is HEADLESS -- vision_display aborts there with 'no Qt
        # platform plugin could be initialized' (exit -6), measured on the Pi.
        # Pass viewer:=true only where there is a display.
        DeclareLaunchArgument('viewer',     default_value='false',
                              description='Open vision_display (OpenCV viewer) alongside vision pipeline'),
        DeclareLaunchArgument('batighor',   default_value='false',
                              description='Batighor, the operator center: foxglove_bridge on '
                                          'foxglove_port and the Lichtblick app on batighor_port '
                                          '(open http://<pi>:<batighor_port>/). Off the mission '
                                          'path; any number of browsers.'),
        DeclareLaunchArgument('batighor_port', default_value='8000',
                              description='Batighor web port'),
        DeclareLaunchArgument('foxglove_port', default_value='8765',
                              description='foxglove_bridge WebSocket port'),
    ]

    # Values from ~/.mongla/bringup.yaml (or $MONGLA_BRINGUP) become the
    # defaults; anything typed as name:=value still wins. Printed at startup so
    # the log says which settings did not come from the command line.
    vision_pi_launch_path = os.path.join(
        get_package_share_directory('mongla_vision'),
        'launch', 'vision_pi.launch.py')
    args += _vision_pi_arguments(vision_pi_launch_path)
    args, from_file, file_path = launch_defaults.apply(args)
    file_note = ([LogInfo(msg=f'[bringup] settings from {file_path}: ' + ', '.join(
        f'{k}={v}' for k, v in sorted(from_file.items())))] if from_file else [])

    manager_node = partial(
        Node,
        package='mongla_manager',
        executable='start',
        name='mongla_manager',
        output='screen',
        # Ctrl-C must outlive the disarm confirmation (issue #21). ros2
        # launch sends SIGTERM 5 s after SIGINT by default, then SIGKILL; the
        # manager's emergency disarm waits up to EMERGENCY_DISARM_S (8 s).
        sigterm_timeout=str(MANAGER_SIGTERM_TIMEOUT_S),
        # Process default warn silences rcl/rmw framework gibberish; the manager's
        # own logger is pinned to info so all Mongla telemetry ([STATE]/[ACT]) and the mission progress lines always show.
        ros_arguments=['--log-level', 'warn', '--log-level', 'mongla_manager:=info'],
        parameters=[{
            'mav_device':           LaunchConfiguration('mav_device'),
            'payload_channels':     LaunchConfiguration('payload_channels'),
            # value_type=bool so a malformed value fails HERE, at launch, with a clear
            # ValueError -- rather than reaching the node as a str/int and dying in
            # declare_parameter with a type error that names no argument. launch_ros
            # coerces the normal spellings either way (false/False/no/off -> False),
            # so there is no silent-True path; this just moves the error somewhere the
            # operator can read it. It matters more here than on the other flags:
            # below firmware behaviour rev 2, MOVE_STOP coasts and this host carries
            # no brake, so a wrongly-true value means `stop` does not decelerate.
            'allow_fw_behaviour_mismatch': ParameterValue(
                LaunchConfiguration('allow_fw_behaviour_mismatch'), value_type=bool),
            'velocity_uplink':  ParameterValue(LaunchConfiguration('velocity_uplink'), value_type=bool),
            'position_uplink':  ParameterValue(LaunchConfiguration('position_uplink'), value_type=bool),
            'vision.mixer_aware': ParameterValue(LaunchConfiguration('mixer_aware'), value_type=bool),
        }],
    )

    vision_launch_path = os.path.join(
        get_package_share_directory('mongla_vision'),
        'launch', 'vision.launch.py')

    # SCOPED, and the scope is load-bearing. `IncludeLaunchDescription` does
    # NOT scope launch configurations: every `name:=value` given to this file
    # leaks into the included one and overrides its default for any argument of
    # the same name. `vision` is a BOOLEAN here and a PROFILE STRING ('fast')
    # inside vision_pi, so `vision:=true` set `vision_profile` to the literal
    # 'true' and detector_dual_node died at startup with
    #   InvalidParameterTypeException: ... to 'True' of type 'BOOL',
    #   expecting type 'STRING': vision_profile
    # -- MEASURED on the vehicle, not reasoned about. Leaving the name out of
    # the dict below is necessary and NOT sufficient; only the scope stops it.
    # Same defect CLAUDE.md records for the sim, where it silently started
    # nothing for a season.
    _stack_is = lambda want: IfCondition(PythonExpression([
        "'", LaunchConfiguration('vision'), "'.lower() in ('true','1') and '",
        LaunchConfiguration('vision_stack'), "' == '", want, "'"]))

    vision_pi_args = {name: LaunchConfiguration(name) for name in VISION_PI_PASSTHROUGH}
    vision_pi_args.update({
            'fwd_model':     LaunchConfiguration('fwd_model'),
            'dwn_model':     LaunchConfiguration('dwn_model'),
            'fwd_classes':   LaunchConfiguration('fwd_classes'),
            'dwn_classes':   LaunchConfiguration('dwn_classes'),
            'conf':          LaunchConfiguration('conf'),
            'imgsz':         LaunchConfiguration('imgsz'),
            'max_det':       LaunchConfiguration('max_det'),
            'viewer':        LaunchConfiguration('viewer'),
            'vision':        LaunchConfiguration('vision_profile'),
            'lock_class':    LaunchConfiguration('lock_class'),
            'flow':          LaunchConfiguration('flow'),
            'pool_depth_m':  LaunchConfiguration('pool_depth_m'),
            'pool_depth_min_m': LaunchConfiguration('pool_depth_min_m'),
            'pool_depth_max_m': LaunchConfiguration('pool_depth_max_m'),
            'medium':        LaunchConfiguration('medium'),
            'lock':          LaunchConfiguration('lock'),
            'paused':        LaunchConfiguration('paused'),
            'caustics':      LaunchConfiguration('caustics'),
            'lane_lines':    LaunchConfiguration('lane_lines'),
            'tile_m':        LaunchConfiguration('tile_m'),
    })
    vision_pi_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(vision_pi_launch_path),
        launch_arguments=vision_pi_args.items(),
        condition=_stack_is('pi'),
    )

    vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(vision_launch_path),
        launch_arguments={
            'camera':        LaunchConfiguration('camera'),
            'profile':       LaunchConfiguration('camera_profile'),
            'model':         LaunchConfiguration('model'),
            'models':        LaunchConfiguration('models'),
            'active_model':  LaunchConfiguration('active_model'),
            'classes':       LaunchConfiguration('classes'),
            'conf':          LaunchConfiguration('conf'),
            'imgsz':         LaunchConfiguration('imgsz'),
            'max_det':       LaunchConfiguration('max_det'),
            'viewer':        LaunchConfiguration('viewer'),
            'lock':          LaunchConfiguration('lock'),
            'lock_class':    LaunchConfiguration('lock_class'),
            # Same meaning in both: start flow_node on the downward camera.
            'flow':          LaunchConfiguration('flow'),
            'pool_depth_m':  LaunchConfiguration('pool_depth_m'),
            # Forwarded EXPLICITLY: both launches declare `paused`, so leaving
            # it out does not keep it out -- an include inherits every parent
            # name:=value. One detector, resumed by the first query or verb.
            'paused':        LaunchConfiguration('paused'),
        }.items(),
        condition=_stack_is('generic'),
    )

    # The outer half of the localization split: the board owns the 500 Hz
    # attitude loop, this owns pool-frame position. Lives beside the manager
    # rather than in the vision launch because its inputs are the manager's
    # (/mongla/imu, /mongla/state) and it must run with vision:=false too.
    localization_node = partial(
        Node,
        package='mongla_localization',
        executable='localization_node',
        name='mongla_localization',
        output='screen',
        ros_arguments=['--log-level', 'warn',
                       '--log-level', 'mongla_localization:=info'],
        parameters=[{
            'flow_camera': 'downward',
            'zupt':       ParameterValue(LaunchConfiguration('zupt'), value_type=bool),
            'demand_aid': ParameterValue(LaunchConfiguration('demand_aid'), value_type=bool),
            'use_yaw':    ParameterValue(LaunchConfiguration('use_yaw'), value_type=bool),
            'retrodict':  ParameterValue(LaunchConfiguration('retrodict'), value_type=bool),
        }],
        condition=IfCondition(LaunchConfiguration('localization')),
    )

    # Batighor (mongla_batighor): the bridge and the web app. Opt-in, off the
    # mission path. The bridge exposes every topic over a WebSocket that
    # Lichtblick, in the browser or on the desktop, renders (our
    # detections/images/state are standard vision_msgs/sensor_msgs). use_compression
    # cuts tether bandwidth; still confirm detection FPS is unperturbed with it up
    # and images being viewed (Mongla is FPS-coupled). See foxglove-and-bags.md.
    foxglove_node = partial(
        Node,
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        ros_arguments=['--log-level', 'warn', '--log-level', 'foxglove_bridge:=info'],
        parameters=[{
            'port':            LaunchConfiguration('foxglove_port'),
            'address':         '0.0.0.0',
            'use_compression': True,
            # /mongla/move/_action/feedback + /status live under the _action
            # namespace => ROS 2 HIDDEN topics. Without this the live err_x_px/
            # err_y_px convergence plot is silently empty (default is false).
            'include_hidden':  True,
            **BRIDGE_LIMITS,
        }],
        condition=IfCondition(LaunchConfiguration('batighor')),
    )
    batighor_node = partial(
        Node,
        package='mongla_batighor',
        executable='serve',
        name='batighor',
        output='screen',
        arguments=['--port', LaunchConfiguration('batighor_port'),
                   '--bridge-port', LaunchConfiguration('foxglove_port')],
        condition=IfCondition(LaunchConfiguration('batighor')),
    )

    # The detector's boxes as annotations the browser draws over the camera's
    # own JPEG (mongla_batighor/overlay.py). Publishes only while watched.
    batighor_overlay = partial(
        Node,
        package='mongla_batighor',
        executable='overlay',
        name='batighor_overlay',
        output='screen',
        condition=IfCondition(LaunchConfiguration('batighor')),
    )

    # The browsers' helm: the pilot token and the command log
    # (mongla_batighor/helm.py). The terminal does not need it.
    batighor_helm = partial(
        Node,
        package='mongla_batighor',
        executable='helm',
        name='batighor_helm',
        output='screen',
        condition=IfCondition(LaunchConfiguration('batighor')),
    )

    # The same file's `nodes:` section: node parameters, applied to every node
    # below. Each node refuses a key it does not declare.
    params = launch_defaults.node_actions()

    return LaunchDescription(args + file_note + params + [
        # One BLAS thread per call, set before any node imports numpy.
        # The vehicle runs OpenBLAS (kutum/doctor.sh checks for it), whose pool
        # defaults to every core in EVERY process. The XFeat matcher already
        # splits its matmul across `MATCH_THREADS` = 2 Python threads on purpose;
        # an OpenBLAS pool under each makes 8 threads on 4 cores beside two
        # detectors and the control loop. Measured on the Pi, 1024x1024x64:
        # 4.76 ms at blas=1/split=2 against 5.33 ms at blas=4/split=2.
        SetEnvironmentVariable('OPENBLAS_NUM_THREADS', '1'),
        OpaqueFunction(function=_warn_vision_only),
        # Every process here restarts 2 s after it dies mid-run, three times,
        # then stays down (`launch_respawn`). The manager too: its teardown
        # still tries the disarm, and a new manager that hears ARMED latches
        # and refuses every verb but disarm/stop/surface (`boot_latch`), so a
        # restart gives back the only host disarm path and moves nothing. A
        # restarted localization publishes in `odom` until re-anchored, which
        # `pose()` already refuses, so a mission reads "unknown", not a wrong
        # pose.
        *respawning(manager_node, 'the manager'),
        *respawning(localization_node, 'localization'),
        # scoped=True keeps anything set INSIDE from escaping. It does not stop
        # the parent leaking IN -- and `forwarding=False`, which does, also
        # hides `vision`, `fwd_model` and friends from the condition and the
        # passthrough dict itself, so nothing launched at all (measured). The
        # leak is therefore fixed where it belongs: at the boundary, by giving
        # the colliding name a valid value instead of an inherited one.
        GroupAction([vision_pi_launch], scoped=True),
        GroupAction([vision_launch], scoped=True),
        *respawning(foxglove_node, 'the Foxglove bridge'),
        *respawning(batighor_node, 'the Batighor web app'),
        *respawning(batighor_overlay, 'the Batighor overlay'),
        *respawning(batighor_helm, 'the Batighor helm'),
    ])
