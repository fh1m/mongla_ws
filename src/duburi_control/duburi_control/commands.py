"""Single source of truth for every /duburi/move command.

This is the ONE place that knows what commands exist, what fields each
one reads from `Move.Goal`, what the defaults are, and what to print in
`--help`. The action server, the Python client, and the `duburi` CLI
all read from here -- adding a new command is two edits, no more:

    1. Add a row to COMMANDS below.
    2. Add a method on `Duburi` whose name and parameter names match
       the row's `cmd` and `fields`.

Field names map directly to fields on `duburi_interfaces.action.Move`
(`duration`, `gain`, `target`, `target_name`, `timeout`, `settle`,
`yaw_rate_pct`). The dispatch just does
`getattr(duburi, cmd)(**kwargs_built_from_fields)`.

`defaults` covers the case where the operator omits a field. Any field
NOT listed in `defaults` is required -- the CLI marks it `required=True`,
and the action server uses whatever the caller put on the goal (which
rosidl zero-initialises, so the operator must supply it explicitly).
"""


COMMANDS = {
    # ---- Arm / disarm / mode --------------------------------------- #
    'arm': {
        'help':     'Arm the vehicle (motors hot).',
        'fields':   ['timeout'],
        'defaults': {'timeout': 15.0},
    },
    'disarm': {
        'help':     'Disarm safely (mode -> MANUAL, neutral, then disarm).',
        'fields':   ['timeout'],
        'defaults': {'timeout': 20.0},
    },
    'set_mode': {
        'help':     'Switch ArduSub flight mode (MANUAL, ALT_HOLD, STABILIZE, ...).',
        'fields':   ['target_name', 'timeout'],
        'defaults': {'timeout': 8.0},
    },

    # ---- State read-outs ------------------------------------------- #
    'head': {
        'help':     'Read current heading (degrees) at execution time. '
                    'Result is in final_value. Also works as a magic value in '
                    'other CLI commands: --target head resolves to the live '
                    'heading the moment the command runs.',
        'fields':   [],
        'defaults': {},
    },

    # ---- Stop / pause ---------------------------------------------- #
    'stop': {
        'help':     'Active hold: send neutral 1500 PWM to all six channels.',
        'fields':   [],
        'defaults': {},
    },
    'surface': {
        'help':     'Emergency surface: set depth to 0 m and hold until reached. '
                    'Bypasses command_active gate so it works during a running mission.',
        'fields':   [],
        'defaults': {},
    },
    'pause': {
        'help':     'Release RC override for N seconds (autopilot takes over).',
        'fields':   ['duration'],
        'defaults': {'duration': 2.0},
    },

    # ---- Forward / back  (Ch5) ------------------------------------- #
    'move_forward': {
        'help':     'Drive forward for `duration` s at `gain` percent thrust.',
        'fields':   ['duration', 'gain', 'settle'],
        'defaults': {'gain': 80.0, 'settle': 0.0},
    },
    'move_back': {
        'help':     'Drive backward for `duration` s at `gain` percent thrust.',
        'fields':   ['duration', 'gain', 'settle'],
        'defaults': {'gain': 80.0, 'settle': 0.0},
    },

    # ---- Left / right  (Ch6) --------------------------------------- #
    'move_left': {
        'help':     'Strafe left for `duration` s at `gain` percent thrust.',
        'fields':   ['duration', 'gain', 'settle'],
        'defaults': {'gain': 80.0, 'settle': 0.0},
    },
    'move_right': {
        'help':     'Strafe right for `duration` s at `gain` percent thrust.',
        'fields':   ['duration', 'gain', 'settle'],
        'defaults': {'gain': 80.0, 'settle': 0.0},
    },

    # ---- Style maneuvers ------------------------------------------- #
    'style_roll': {
        'help':     'Style: N×360° roll on Ch2 in ACRO mode, one flip per loop '
                    'iteration; `timeout` is per flip, not total. '
                    'BNO085-confirmed (AHRS2 fallback) with direction-locked '
                    'unwrap. Per flip: optional `headroom` m pre-dive, ACRO '
                    'roll guarded by a hard surface-depth abort and '
                    'cos(roll)-modulated Ch3 depth correction, then ALT_HOLD '
                    'recovery to the origin depth before the next flip. '
                    'ACRO_BAL_ROLL + ACRO_TRAINER zeroed before and restored '
                    'after. Cancel mid-flip restores ALT_HOLD then disarms.',
        'fields':   ['gain', 'timeout', 'flips', 'headroom'],
        'defaults': {'gain': 60.0, 'timeout': 20.0, 'flips': 1, 'headroom': 1.0},
    },
    'style_yaw': {
        'help':     'Style: N×360° yaw spin in ALT_HOLD. flips full rotations, '
                    'each as (360/deg_per_step) steps with settle between. '
                    'BNO heading tracking active. No mode change — safest style verb.',
        'fields':   ['flips', 'deg_per_step', 'settle'],
        'defaults': {'flips': 1, 'deg_per_step': 90.0, 'settle': 1.0},
    },

    # ---- Curved (car-style) motion --------------------------------- #
    'arc': {
        'help':     'Curved motion: forward thrust + yaw rate at the same time. '
                    'gain is forward thrust pct; yaw_rate_pct is signed yaw stick.',
        'fields':   ['duration', 'gain', 'yaw_rate_pct', 'settle'],
        'defaults': {'gain': 50.0, 'yaw_rate_pct': 30.0, 'settle': 0.0},
    },

    # ---- Yaw  (sharp pivots) --------------------------------------- #
    'yaw_left': {
        'help':     'Sharp pivot left by `target` degrees within `timeout` s.',
        'fields':   ['target', 'timeout', 'settle'],
        'defaults': {'timeout': 30.0, 'settle': 0.0},
    },
    'yaw_right': {
        'help':     'Sharp pivot right by `target` degrees within `timeout` s.',
        'fields':   ['target', 'timeout', 'settle'],
        'defaults': {'timeout': 30.0, 'settle': 0.0},
    },
    'turn': {
        'help':     'Rotate to absolute heading `target` degrees (0-360) via shortest '
                    'arc. Direction (left/right) is chosen automatically.',
        'fields':   ['target', 'timeout', 'settle'],
        'defaults': {'timeout': 30.0, 'settle': 0.0},
    },

    # ---- Depth ----------------------------------------------------- #
    'set_depth': {
        'help':     'Hold absolute depth (`target` metres, negative below surface).',
        'fields':   ['target', 'timeout', 'settle'],
        'defaults': {'timeout': 30.0, 'settle': 0.0},
    },

    # ---- Heading lock (depth-hold's yaw cousin) -------------------- #
    'lock_heading': {
        'help':     'Stream Ch4 rate-overrides driven by yaw_source until '
                    'unlock_heading. target=0 means lock current heading.',
        'fields':   ['target', 'timeout'],
        'defaults': {'target': 0.0, 'timeout': 300.0},
    },
    'unlock_heading': {
        'help':     'Stop the heading-lock streamer, send neutral.',
        'fields':   [],
        'defaults': {},
    },

    # ---- DVL ------------------------------------------------------- #
    'dvl_connect': {
        'help':     'Connect to the Nortek Nucleus 1000 DVL over TCP and begin '
                    'streaming. Must be called before any move_*_dist command '
                    'when yaw_source is dvl/nucleus_dvl.',
        'fields':   [],
        'defaults': {},
    },

    # ---- DVL distance-based motion --------------------------------- #
    'move_forward_dist': {
        'help':     'Drive forward `distance_m` metres using DVL position feedback. '
                    'Falls back to open-loop timed drive if no DVL position available.',
        'fields':   ['distance_m', 'gain', 'dvl_tolerance', 'settle'],
        'defaults': {'gain': 60.0, 'dvl_tolerance': 0.1, 'settle': 0.0},
    },
    'move_back_dist': {
        'help':     'Drive backward `distance_m` metres using DVL position feedback. '
                    'Falls back to open-loop timed drive if no DVL position available.',
        'fields':   ['distance_m', 'gain', 'dvl_tolerance', 'settle'],
        'defaults': {'gain': 60.0, 'dvl_tolerance': 0.1, 'settle': 0.0},
    },
    'move_lateral_dist': {
        'help':     'Strafe `distance_m` metres (positive=right, negative=left) '
                    'using DVL position feedback.',
        'fields':   ['distance_m', 'gain', 'dvl_tolerance', 'settle'],
        'defaults': {'gain': 36.0, 'dvl_tolerance': 0.1, 'settle': 0.0},
    },

    # ---- Vision verbs --------------------------------------------- #
    # All of these read the latest detection from VisionState; preflight
    # runs once per camera the first time it's hit. Defaults are tuned
    # for a webcam-detected 'person' but every gain is overrideable from
    # the goal so missions can profile them in flight.
    #
    # tracking=True: subscribe to /tracks (tracker_node must be running)
    # instead of /detections. Enables ByteTrack ID stability + Kalman
    # smoothing. Off by default -- requires tracker_node for that camera.
    # Can also be set persistently: ros2 param set /duburi_manager vision.use_tracks true
    'vision_align_3d': {
        'help':     'Hold target_class centred AND at target_bbox_h_frac. '
                    'Active axes via CSV axes. lock_mode: settle/follow/pursue. '
                    'depth_anchor_frac: 0=top, 0.5=centre, 1=bottom of bbox. '
                    'distance_metric: height/area/width/diagonal. '
                    'gate_guard=true: suppress forward when gate appears angled. '
                    'pass_at: freeze lat+depth, drive straight once size>=pass_at. '
                    'tracking=true: use tracker_node (stable IDs + Kalman).',
        'fields':   ['camera', 'target_class', 'axes', 'duration',
                     'deadband', 'kp_yaw', 'kp_lat', 'kp_depth', 'kp_forward',
                     'target_bbox_h_frac', 'visual_pid', 'on_lost',
                     'stale_after', 'depth_anchor_frac', 'lock_mode',
                     'distance_metric', 'gate_guard', 'gate_guard_min_w_frac',
                     'pass_at', 'pass_at_gain', 'tracking',
                     'offset_x', 'offset_y', 'lost_patience_s'],
        'defaults': {'camera': 'laptop', 'target_class': 'person',
                     'axes': 'yaw,forward', 'duration': 30.0,
                     'deadband': 0.18, 'kp_yaw': 60.0, 'kp_lat': 60.0,
                     'kp_depth': 0.05, 'kp_forward': 200.0,
                     'target_bbox_h_frac': 0.30, 'visual_pid': False,
                     'on_lost': 'fail', 'stale_after': 1.5,
                     'depth_anchor_frac': 0.0,  # 0.0 = use ROS param default (0.5)
                     'lock_mode': '', 'distance_metric': '',
                     'gate_guard': False, 'gate_guard_min_w_frac': 0.35,
                     'pass_at': 0.0, 'pass_at_gain': 50.0,
                     'tracking': False,
                     'offset_x': 0.0, 'offset_y': 0.0,
                     'lost_patience_s': 0.0},
    },
    'vision_align_yaw': {
        'help':     'Steer toward horizontal centre via heading channel. '
                    'lock_mode: settle (done when centred) / follow (until duration). '
                    'offset_x/y: keep target this many px from center (0=center). '
                    'tracking=true: use tracker_node.',
        'fields':   ['camera', 'target_class', 'duration', 'deadband',
                     'kp_yaw', 'on_lost', 'stale_after', 'lock_mode', 'tracking',
                     'offset_x', 'offset_y', 'lost_patience_s'],
        'defaults': {'camera': 'laptop', 'target_class': 'person',
                     'duration': 15.0, 'deadband': 0.18,
                     'kp_yaw': 60.0, 'on_lost': 'fail',
                     'stale_after': 1.5, 'lock_mode': '', 'tracking': False,
                     'offset_x': 0.0, 'offset_y': 0.0, 'lost_patience_s': 0.0},
    },
    'vision_align_lat': {
        'help':     'Strafe toward horizontal centre via lateral channel. '
                    'lock_mode: settle / follow. tracking=true: use tracker_node. '
                    'offset_x: keep target this many px to the right of center (slalom pass). '
                    'offset_y: keep target this many px below center.',
        'fields':   ['camera', 'target_class', 'duration', 'deadband',
                     'kp_lat', 'on_lost', 'stale_after', 'lock_mode', 'tracking',
                     'offset_x', 'offset_y', 'lost_patience_s'],
        'defaults': {'camera': 'laptop', 'target_class': 'person',
                     'duration': 15.0, 'deadband': 0.18,
                     'kp_lat': 60.0, 'on_lost': 'fail',
                     'stale_after': 1.5, 'lock_mode': '', 'tracking': False,
                     'offset_x': 0.0, 'offset_y': 0.0, 'lost_patience_s': 0.0},
    },
    'vision_align_depth': {
        'help':     'Nudge depth setpoint to centre target vertically. '
                    'depth_anchor_frac: which vertical point on bbox to align '
                    '(0=top, 0.5=centre, 1=bottom). Use 0.2 for tall objects. '
                    'offset_x/y: keep target this many px from center. '
                    'tracking=true: use tracker_node.',
        'fields':   ['camera', 'target_class', 'duration', 'deadband',
                     'kp_depth', 'on_lost', 'stale_after',
                     'depth_anchor_frac', 'lock_mode', 'tracking',
                     'offset_x', 'offset_y', 'lost_patience_s'],
        'defaults': {'camera': 'laptop', 'target_class': 'person',
                     'duration': 15.0, 'deadband': 0.18,
                     'kp_depth': 0.05, 'on_lost': 'fail',
                     'stale_after': 1.5,
                     'depth_anchor_frac': 0.0,  # 0.0 = use ROS param default (0.5)
                     'lock_mode': '', 'tracking': False,
                     'offset_x': 0.0, 'offset_y': 0.0, 'lost_patience_s': 0.0},
    },
    'vision_hold_distance': {
        'help':     'Drive forward/back to match target_bbox_h_frac. '
                    'distance_metric: height (default) / area / width / diagonal. '
                    'lock_mode: settle / follow / pursue (only approach). '
                    'pass_at: freeze lat+depth, drive straight once size>=pass_at. '
                    'offset_x/y: keep target this many px from center. '
                    'tracking=true: use tracker_node.',
        'fields':   ['camera', 'target_class', 'duration', 'deadband',
                     'kp_forward', 'target_bbox_h_frac', 'on_lost',
                     'stale_after', 'lock_mode', 'distance_metric',
                     'gate_guard', 'gate_guard_min_w_frac',
                     'pass_at', 'pass_at_gain', 'tracking',
                     'offset_x', 'offset_y', 'lost_patience_s'],
        # deadband is tighter here because bbox-height error is naturally
        # smaller than the centring errors on yaw/lat axes.
        'defaults': {'camera': 'laptop', 'target_class': 'person',
                     'duration': 20.0, 'deadband': 0.05,
                     'kp_forward': 200.0, 'target_bbox_h_frac': 0.30,
                     'on_lost': 'fail', 'stale_after': 1.5,
                     'lock_mode': '', 'distance_metric': '',
                     'gate_guard': False, 'gate_guard_min_w_frac': 0.35,
                     'pass_at': 0.0, 'pass_at_gain': 50.0,
                     'tracking': False,
                     'offset_x': 0.0, 'offset_y': 0.0, 'lost_patience_s': 0.0},
    },
    'vis_approach': {
        'help':     'Drive forward using monocular depth (vis_range) from '
                    'depth_estimation_node. target_vis_range is 0..1 (0=far, 1=close). '
                    'Requires depth_estimation_node running on the same camera. '
                    'lock_mode: settle (exit at target), pursue (only approach). '
                    'offset_x/y: keep target this many px from center. '
                    'tracking=true: use tracker_node.',
        'fields':   ['camera', 'target_class', 'duration', 'deadband',
                     'kp_forward', 'target_vis_range', 'on_lost',
                     'stale_after', 'lock_mode', 'tracking',
                     'offset_x', 'offset_y', 'lost_patience_s'],
        'defaults': {'camera': 'laptop', 'target_class': 'person',
                     'duration': 30.0, 'deadband': 0.05,
                     'kp_forward': 200.0, 'target_vis_range': 0.65,
                     'on_lost': 'fail', 'stale_after': 1.5,
                     'lock_mode': '', 'tracking': False,
                     'offset_x': 0.0, 'offset_y': 0.0, 'lost_patience_s': 0.0},
    },
    'vision_acquire': {
        'help':     'Block (optionally driving via target_name verb) until '
                    'target_class is seen at least once. target_name in '
                    "{'', 'yaw_left', 'yaw_right', 'move_forward', 'arc'}. "
                    'tracking=true: use tracker_node.',
        'fields':   ['camera', 'target_class', 'target_name', 'timeout',
                     'gain', 'yaw_rate_pct', 'stale_after', 'tracking'],
        'defaults': {'camera': 'laptop', 'target_class': 'person',
                     'target_name': '', 'timeout': 30.0,
                     'gain': 25.0, 'yaw_rate_pct': 25.0,
                     'stale_after': 1.5, 'tracking': False},
    },
    'vision_lock_fire': {
        'help':     'Align on target in 3D then hold stable for stable_lock_s seconds '
                    'before firing. Retries max_attempts times; fires at last known '
                    'pose on all-fail fallback. '
                    'fire_channel=1/2 torpedo, 3/4 dropper (ESP32 serial). '
                    'fire_channel=0 + fire_aux_channel=0 = log-only stub.',
        'fields':   ['camera', 'target_class', 'axes', 'duration',
                     'deadband', 'kp_yaw', 'kp_lat', 'kp_depth', 'kp_forward',
                     'target_bbox_h_frac', 'on_lost', 'stale_after',
                     'depth_anchor_frac', 'distance_metric',
                     'stable_lock_s', 'max_attempts', 'attempt_timeout',
                     'fire_aux_channel', 'fire_pwm',
                     'offset_x', 'offset_y',
                     'fire_channel', 'lost_patience_s'],
        'defaults': {'camera': 'forward', 'target_class': 'torpedo_hole',
                     'axes': 'yaw,lat,depth', 'duration': 60.0,
                     'deadband': 0.10, 'kp_yaw': 60.0, 'kp_lat': 60.0,
                     'kp_depth': 0.05, 'kp_forward': 0.0,
                     'target_bbox_h_frac': 0.0, 'on_lost': 'hold',
                     'stale_after': 1.5, 'depth_anchor_frac': 0.0,
                     'distance_metric': '',
                     'stable_lock_s': 3.0, 'max_attempts': 3.0,
                     'attempt_timeout': 15.0,
                     'fire_aux_channel': 0.0, 'fire_pwm': 1900.0,
                     'offset_x': 0.0, 'offset_y': 0.0,
                     'fire_channel': 0.0, 'lost_patience_s': 0.0},
    },
    'fire': {
        'help':     'Fire ESP32 payload channel. 1/2 = torpedo, 3/4 = dropper. '
                    'Requires payload_port to be connected (auto-detected at startup).',
        'fields':   ['fire_channel'],
        'defaults': {'fire_channel': 1.0},
    },
    'look_around': {
        'help':     'Switch to POSHOLD, hold position, and rotate incrementally '
                    'searching for target_class. Exits immediately when target '
                    'is detected. Makes a full orbit if duration allows. '
                    'yaw_rate_pct = step degrees per look (positive=CW/right, '
                    'negative=CCW/left). gain = turn speed %%. '
                    'settle = dwell seconds at each step to observe. '
                    'target = override starting yaw (0.0 = current heading). '
                    'Falls back to ALT_HOLD + heading lock if POSHOLD unavailable.',
        'fields':   ['camera', 'target_class', 'duration', 'gain',
                     'yaw_rate_pct', 'settle', 'target', 'stale_after'],
        'defaults': {'camera': 'laptop', 'target_class': 'person',
                     'duration': 60.0,       # enough for a full 360° orbit at 20° steps
                     'gain': 40.0,            # turn speed percent
                     'yaw_rate_pct': 20.0,   # step degrees (positive = right orbit)
                     'settle': 1.5,           # dwell per step to observe
                     'target': 0.0,           # override start yaw; 0.0 = use current
                     'stale_after': 1.0},     # detection freshness threshold
    },
}


# Field names that carry a string instead of a float (everything else is float).
STRING_FIELDS = ('target_name', 'camera', 'target_class', 'axes', 'on_lost',
                 'lock_mode', 'distance_metric')

# Field names that carry a bool. rosidl init these to False.
BOOL_FIELDS = ('visual_pid', 'tracking', 'gate_guard')


def fields_for(cmd, request, *, runtime_defaults=None):
    """Pull the kwargs for `cmd` out of a Move.Goal `request`.

    For each field the spec lists, take the value from the request. If
    the value is the rosidl "unset" default (False for bool, 0.0 for
    floats, '' for strings), substitute in this order:

        1. `runtime_defaults[field]` if set  (live ROS-param override)
        2. `spec['defaults'][field]` if set  (hardcoded fallback)
        3. leave the rosidl zero on the kwargs

    `runtime_defaults` is the manager's `vision.*` ROS-param snapshot
    (or None). It lets pool operators tune `vision.kp_yaw` etc. with
    `ros2 param set` and have the next vision goal pick up the new
    value WITHOUT touching either the spec defaults or the mission's
    Python source.

    The returned dict can be passed straight to the matching `Duburi`
    method as `**kwargs` -- the dispatch never has to know what each
    command is doing.
    """
    spec = COMMANDS[cmd]
    spec_defaults    = spec['defaults']
    runtime_defaults = runtime_defaults or {}
    kwargs = {}
    for field in spec['fields']:
        value = getattr(request, field)
        if field in STRING_FIELDS:
            unset = (value == '')
        elif field in BOOL_FIELDS:
            # bool unset == False; defaults explicitly set True or False.
            unset = (value is False) and (field in spec_defaults) and \
                    (spec_defaults[field] is True)
        else:
            unset = (value == 0.0)
        if unset:
            if field in runtime_defaults:
                value = runtime_defaults[field]
            elif field in spec_defaults:
                value = spec_defaults[field]
        kwargs[field] = value
    return kwargs
