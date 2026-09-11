"""bringup -- start the AUV control stack (manager + optional vision).

One-command pool-day bringup (defaults: pool mode, DVL auto-connect):

    # Control only (no vision):
    ros2 launch duburi_manager bringup.launch.py

    # With vision -- BOTH cameras, measured calibration, Hailo models:
    ros2 launch duburi_manager bringup.launch.py vision:=true

    # ...and the downward-camera velocity path (still default-off, unvalidated).
    # pool_depth_m is REQUIRED -- without it flow_node refuses and publishes
    # quality 0, deliberately, rather than guessing the scale:
    ros2 launch duburi_manager bringup.launch.py vision:=true flow:=true \\
        pool_depth_m:=1.2

    # Single camera on a dev box / CUDA .pt (what this file used to do ALWAYS):
    ros2 launch duburi_manager bringup.launch.py vision:=true \\
        vision_stack:=generic model:=gate_flare_medium_100ep conf:=0.35

    # Full pool day (BNO085 heading + DVL distance, both cameras, no viewer):
    ros2 launch duburi_manager bringup.launch.py vision:=true \\
        yaw_source:=bno085_dvl viewer:=false

    # Bench / sim -- no DVL, use mavlink AHRS:
    ros2 launch duburi_manager bringup.launch.py mode:=sim yaw_source:=mavlink_ahrs

Multi-model registry (switch models mid-mission without restart):
    ros2 launch duburi_manager bringup.launch.py vision:=true \\
        models:="gate=gate_nano_100ep,flare=flare_medium_100ep,combined=gate_flare_medium_100ep" \\
        active_model:=gate classes:=gate conf:=0.45

    # In the mission DSL:
    #   duburi.use('gate')               # phase: gate
    #   duburi.use('flare', 'flare')     # phase: flare
    #   duburi.use('combined', 'gate')   # phase: combined model, gate filter

DVL connects automatically on startup (dvl_auto_connect:=true by default).
Manual override: ros2 run duburi_planner duburi dvl_connect

Vision commands to test gate detection:
    ros2 run duburi_planner duburi vision_align --camera forward --target_class gate --axes yaw,lat --duration 10
    ros2 run duburi_planner duburi vision_move  --camera forward --target_class gate --fwd_fill 80 --mode area --duration 15

Run the gate+flare mission:
    ros2 run duburi_planner mission gate_flare_prequal
"""

from launch                     import LaunchDescription
from launch.actions             import (DeclareLaunchArgument, GroupAction,
                                        IncludeLaunchDescription)
from launch.conditions          import IfCondition
from launch.substitutions       import PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions       import LaunchConfiguration
from launch_ros.actions         import Node
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    args = [
        DeclareLaunchArgument('mode',       default_value='pool',
                              description='Connection mode: pool|sim|auto|desk|laptop'),
        # Default is mavlink_ahrs since the SROT board became the flight controller.
        #
        # It used to be 'dvl', which is now wrong twice over. The BNO085 moved ONTO
        # the control board (I2C0) and the separate ESP32-C3 + BNO085 USB board was
        # removed from the hull, so 'bno085' opens a device that is not fitted; and
        # 'dvl' made VehicleProfile report has_dvl=True on srot, which routed the FSM
        # into move_forward_dist -- a verb srot_fc hard-refuses.
        #
        # mavlink_ahrs reads the board's own fused ATTITUDE through fc.get_attitude():
        # the same BNO085 part, one layer closer to the thrusters, fused at 500 Hz and
        # now pinnable to ~50 Hz on the wire. Pass yaw_source:=dvl explicitly if a
        # Nucleus is actually fitted and validated on the vehicle you are running.
        DeclareLaunchArgument('yaw_source', default_value='mavlink_ahrs',
                              description='Yaw source: mavlink_ahrs|dvl|bno085|bno085_dvl'),
        # ---- SROT backend -------------------------------------------------- #
        # These four were declared on the NODE but not here, so the documented
        # pool-day command could not configure a SROT vehicle at all: the operator
        # had to drop to `ros2 run duburi_manager start --ros-args -p ...`, which
        # is a different command from the one every doc gives.
        # Default 'srot': this launch is the pool command in every doc, and the
        # hull is the SROT board running Hengla. The ArduSub/BlueOS path is still
        # here -- pass flight_controller:=pixhawk, which is what the sim does --
        # and the 8th-place configuration is preserved whole on the `pixhawk`
        # branch. See the node's declaration for the full reasoning.
        DeclareLaunchArgument('flight_controller', default_value='srot',
                              description='Autopilot backend: srot|pixhawk '
                                          '(srot = the SROT/Hengla board over USB '
                                          'serial; pixhawk = the ArduSub/BlueOS path)'),
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
        DeclareLaunchArgument('dvl_host',        default_value='192.168.2.201'),
        DeclareLaunchArgument('dvl_port',        default_value='9000'),
        # 'auto' scans USB VID/PID for the CH340 payload board, which is the
        # right default on the vehicle. In simulation there is no CH340: point
        # this at the PTY that `payload_sim` creates
        # (payload_port:=/tmp/duburi-$USER/payload) and fire() works there too.
        # 'auto' scans USB VID/PID for the ESP32-C3 (303a:1001). In sim there
        # is no ESP32: point this at the PTY that `bno085_sim` creates
        # (bno085_port:=/tmp/duburi-$USER/bno085) and yaw_source:=bno085 --
        # what the vehicle actually flies -- works there too.
        DeclareLaunchArgument('bno085_port', default_value='auto',
                              description='BNO085 device, or "auto" to scan '
                                          'USB VID/PID.'),
        DeclareLaunchArgument(
            'baro_calibration', default_value='true',
            description='Re-zero the barometer at the surface. TRUE on the pool '
                        'hull. The simulator passes false: SITL ACKs the '
                        'calibration and then stops tracking depth.'),
        DeclareLaunchArgument('payload_port', default_value='auto',
                              description='Payload board device, or "auto" to '
                                          'scan USB VID/PID.'),
        DeclareLaunchArgument('dvl_auto_connect', default_value='true',
                              description='Auto-connect DVL at startup (true|false)'),
        DeclareLaunchArgument('vision',     default_value='false',
                              description='Start camera + detector alongside manager'),
        # WHICH vision stack. `pi` is the VEHICLE path: vision_pi.launch.py runs
        # BOTH cameras through detector_dual_node, wires the measured calibration
        # JSONs by name, and can start flow_node. `generic` is the single-camera
        # dev/CUDA path (vision.launch.py) this file used to include
        # unconditionally -- which meant the documented pool-day command brought
        # up one camera, no calibration, and a model that is not on the vehicle.
        DeclareLaunchArgument('vision_stack', default_value='pi',
                              choices=['pi', 'generic'],
                              description='pi = both cameras + calibration '
                                          '(vision_pi.launch.py, the vehicle); '
                                          'generic = single camera, no calibration '
                                          '(vision.launch.py, dev/CUDA).'),
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
        DeclareLaunchArgument('fwd_model',   default_value='gate_rescue_repair',
                              description='vision_stack:=pi -- forward-camera model stem.'),
        DeclareLaunchArgument('dwn_model',   default_value='bin_fire_blood',
                              description='vision_stack:=pi -- downward-camera model stem.'),
        DeclareLaunchArgument('fwd_classes', default_value='',
                              description='vision_stack:=pi -- forward class allowlist '
                                          '(empty = the model sidecar\'s full set).'),
        DeclareLaunchArgument('dwn_classes', default_value='',
                              description='vision_stack:=pi -- downward class allowlist.'),
        DeclareLaunchArgument('localization', default_value='true',
                              description='Run the invariant filter (duburi_'
                                          'localization). Read-only: it '
                                          'publishes /duburi/odom and commands '
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
        DeclareLaunchArgument('pool_depth_m', default_value='nan',
                              description='flow:=true -- metres from the DOWNWARD '
                                          'camera to the floor. Required: without '
                                          'it flow_node publishes quality 0 and '
                                          'refuses, by design.'),
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
        # ⛔ EXPOSED BECAUSE THE MISSION PATH IS THE ONE THAT PAYS FOR IT. Two
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
        # ⛔ THIS DEFAULT MUST AGREE WITH `flight_controller` ABOVE.
        # `flight_controller` defaults to srot, i.e. the default vehicle on
        # main is the SROT board + Pi box -- but `camera` defaulted to the
        # Jetson's Blue Robotics profile, and the two were never reconciled.
        # Measured on the Pi running the documented bringup line: 5.00 Hz and
        # CameraInfo.k all zero, because the `forward` profile carries no
        # `fourcc: MJPG` (YUYV is 3x slower on this unit) and no calibration
        # declares it. Both are silent -- the frame rate looks like "vision is
        # slow" and the zero K only bites when the srot vision uplink is
        # switched on.
        #
        # The role is NOT renamed: /duburi/vision/forward/... and
        # duburi_detector_forward are what 96 call sites and the DSL's own
        # `camera='forward'` default bind to. Only the hardware moves.
        # On the Jetson, pass camera_profile:=forward.
        DeclareLaunchArgument('camera_profile', default_value='pi_forward',
                              description='Camera HARDWARE profile (CAMERA_PROFILES in '
                                          'duburi_vision/config.py -- the loaded copy). '
                                          'Default pi_forward matches flight_controller:=srot '
                                          '(Pi box). On the Jetson pass forward.'),
        DeclareLaunchArgument('model',      default_value='gate_flare_medium_100ep',
                              description='Single-model: gate_flare_medium_100ep|gate_nano_100ep|gate_medium_100ep|flare_medium_100ep'
                                          '|yolov11n (ROBOSUB-tested pretrained, sim/bench)|yolo26_nano_pretrained'),
        DeclareLaunchArgument('models',     default_value='',
                              description='Multi-model registry (CSV name=stem): '
                                          '"gate=gate_nano_100ep,combined=gate_flare_medium_100ep"'),
        DeclareLaunchArgument('active_model', default_value='',
                              description='Registry key to start with (requires models:="..." to be set)'),
        DeclareLaunchArgument('classes',    default_value='gate',
                              description='CSV class names for detector'),
        # 0.15 is the Hailo INT8 operating point, and the TOP of the measured
        # bar (`measured-bars.md`: 0.08 <= conf <= 0.15; above 0.15 loses the
        # cross-venue gate). It was 0.30 here -- a CUDA-path number, shipped on
        # the path that never ran. On `vision_stack:=generic` (a .pt on CUDA)
        # pass conf:=0.35 explicitly.
        DeclareLaunchArgument('conf',       default_value='0.15',
                              description='Detector confidence floor. 0.15 = Hailo '
                                          'INT8 operating point; pass 0.35 for the '
                                          'generic CUDA stack.'),
        DeclareLaunchArgument('imgsz',      default_value='640',
                              description='Inference square size. NOTE: a TensorRT .engine bakes '
                                          'imgsz at export -- this only re-scales the .pt fallback. '
                                          'For TRT, re-export to match (export_engine --all '
                                          '--imgsz <N>), then imgsz:=<N>.'),
        DeclareLaunchArgument('max_det',    default_value='100',
                              description='Post-NMS detection cap (runtime; lower toward ~10 if NMS '
                                          'is the FPS bottleneck on busy frames).'),
        # The vehicle is HEADLESS -- vision_display aborts there with 'no Qt
        # platform plugin could be initialized' (exit -6), measured on the Pi.
        # Pass viewer:=true only where there is a display.
        DeclareLaunchArgument('viewer',     default_value='false',
                              description='Open vision_display (OpenCV viewer) alongside vision pipeline'),
        DeclareLaunchArgument('foxglove',   default_value='false',
                              description='Start foxglove_bridge (WebSocket telemetry on foxglove_port). '
                                          'Off the mission path -- pure viz. Needs '
                                          'ros-humble-foxglove-bridge installed. Connect the Foxglove '
                                          'desktop app to ws://<jetson-ip>:<foxglove_port>.'),
        DeclareLaunchArgument('foxglove_port', default_value='8765',
                              description='foxglove_bridge WebSocket port'),
    ]

    manager_node = Node(
        package='duburi_manager',
        executable='start',
        name='duburi_manager',
        output='screen',
        # Process default warn silences rcl/rmw framework gibberish; the manager's
        # own logger is pinned to info so all Mongla telemetry ([STATE]/[ARDUB]/
        # [RC ]/[ACT]) and the mission progress lines always show.
        ros_arguments=['--log-level', 'warn', '--log-level', 'duburi_manager:=info'],
        parameters=[{
            'mode':                 LaunchConfiguration('mode'),
            'yaw_source':           LaunchConfiguration('yaw_source'),
            'nucleus_dvl_host':     LaunchConfiguration('dvl_host'),
            'nucleus_dvl_port':     LaunchConfiguration('dvl_port'),
            'nucleus_dvl_password': 'nortek',
            'dvl_auto_connect':     LaunchConfiguration('dvl_auto_connect'),
            'payload_port':         LaunchConfiguration('payload_port'),
            'baro_calibration':     LaunchConfiguration('baro_calibration'),
            'bno085_port':          LaunchConfiguration('bno085_port'),
            'flight_controller':    LaunchConfiguration('flight_controller'),
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
        }],
    )

    vision_launch_path = os.path.join(
        get_package_share_directory('duburi_vision'),
        'launch', 'vision.launch.py')

    vision_pi_launch_path = os.path.join(
        get_package_share_directory('duburi_vision'),
        'launch', 'vision_pi.launch.py')

    # ⛔ SCOPED, and the scope is load-bearing. `IncludeLaunchDescription` does
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

    vision_pi_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(vision_pi_launch_path),
        launch_arguments={
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
            'medium':        LaunchConfiguration('medium'),
            'lock':          LaunchConfiguration('lock'),
            'paused':        LaunchConfiguration('paused'),
        }.items(),
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
    # (/duburi/imu, /duburi/state) and it must run with vision:=false too.
    localization_node = Node(
        package='duburi_localization',
        executable='localization_node',
        name='duburi_localization',
        output='screen',
        ros_arguments=['--log-level', 'warn',
                       '--log-level', 'duburi_localization:=info'],
        parameters=[{
            'flow_camera': 'downward',
        }],
        condition=IfCondition(LaunchConfiguration('localization')),
    )

    # Foxglove telemetry bridge -- opt-in, off the mission path (pure viz). Auto-
    # exposes every topic over a WebSocket the Foxglove desktop app renders (our
    # detections/images/state are standard vision_msgs/sensor_msgs). use_compression
    # cuts tether bandwidth; still confirm detection FPS is unperturbed with it up
    # and images being viewed (Mongla is FPS-coupled). See foxglove-and-bags.md.
    foxglove_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        ros_arguments=['--log-level', 'warn', '--log-level', 'foxglove_bridge:=info'],
        parameters=[{
            'port':            LaunchConfiguration('foxglove_port'),
            'address':         '0.0.0.0',
            'use_compression': True,
            # /duburi/move/_action/feedback + /status live under the _action
            # namespace => ROS 2 HIDDEN topics. Without this the live err_x_px/
            # err_y_px convergence plot is silently empty (default is false).
            'include_hidden':  True,
        }],
        condition=IfCondition(LaunchConfiguration('foxglove')),
    )

    return LaunchDescription(args + [
        manager_node,
        localization_node,
        # scoped=True keeps anything set INSIDE from escaping. It does not stop
        # the parent leaking IN -- and `forwarding=False`, which does, also
        # hides `vision`, `fwd_model` and friends from the condition and the
        # passthrough dict itself, so nothing launched at all (measured). The
        # leak is therefore fixed where it belongs: at the boundary, by giving
        # the colliding name a valid value instead of an inherited one.
        GroupAction([vision_pi_launch], scoped=True),
        GroupAction([vision_launch], scoped=True),
        foxglove_node,
    ])
