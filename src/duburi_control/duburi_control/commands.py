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
    'mission_reset': {
        'help':     'Stop heading lock, clear abort event, send RC neutral. '
                    'Call at start of every mission run() to clear state '
                    'carried forward from any previous mission.',
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
        'help':     'Drive forward for `duration` s at `gain` percent thrust. '
                    'pass_through=true -> `gain` is a RAW PWM delta (1500+gain) instead of '
                    'a percent, to probe the minimum PWM that moves the 20kg hull.',
        'fields':   ['duration', 'gain', 'settle', 'pass_through'],
        'defaults': {'gain': 80.0, 'settle': 0.0, 'pass_through': False},
    },
    'move_back': {
        'help':     'Drive backward for `duration` s at `gain` percent thrust. '
                    'pass_through=true -> `gain` is a RAW PWM delta (see move_forward).',
        'fields':   ['duration', 'gain', 'settle', 'pass_through'],
        'defaults': {'gain': 80.0, 'settle': 0.0, 'pass_through': False},
    },

    # ---- Left / right  (Ch6) --------------------------------------- #
    'move_left': {
        'help':     'Strafe left for `duration` s at `gain` percent thrust. '
                    'pass_through=true -> `gain` is a RAW PWM delta (see move_forward).',
        'fields':   ['duration', 'gain', 'settle', 'pass_through'],
        'defaults': {'gain': 80.0, 'settle': 0.0, 'pass_through': False},
    },
    'move_right': {
        'help':     'Strafe right for `duration` s at `gain` percent thrust. '
                    'pass_through=true -> `gain` is a RAW PWM delta (see move_forward).',
        'fields':   ['duration', 'gain', 'settle', 'pass_through'],
        'defaults': {'gain': 80.0, 'settle': 0.0, 'pass_through': False},
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
                    'gain is forward thrust pct; yaw_rate_pct is signed yaw stick. '
                    'pass_through=true -> gain + yaw_rate_pct are RAW PWM deltas.',
        'fields':   ['duration', 'gain', 'yaw_rate_pct', 'settle', 'pass_through'],
        'defaults': {'gain': 50.0, 'yaw_rate_pct': 30.0, 'settle': 0.0,
                     'pass_through': False},
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
    # Exactly two, pixel-native, recover-don't-fail. The control loop
    # reads /detections directly (the topic the HUD shows). Both verbs
    # ALWAYS return success=True (the verb ran); the align/move outcome
    # is an integer code in Move.Result.final_value, so the mission DSL
    # branches on it and the action client never raises on a miss.
    #
    #   final_value codes: 0=ALIGNED/REACHED 1=LOST 2=TIMEOUT
    #                      3=NO_CAMERA 4=ABORTED
    #
    # gain = hard max-speed cap (% thrust); the P-controller output is
    # clamped to it so the AUV never exceeds it on any axis.
    'vision_align': {
        'help':     'Centre target_class on the active axes (CSV of lat,yaw,depth) '
                    'each at its signed pixel offset (offset_lat/yaw/depth; 0=centre). '
                    'Aligned when every active axis is within err_px. gain caps speed; '
                    'gain_lat/gain_yaw override the cap on the lat/yaw axis (0 = inherit '
                    'gain) so e.g. yaw can micro-align slowly while lateral stays brisk '
                    '(depth rate is set by depth_step, not a % cap). '
                    'On arrival the lateral inertia is braked (reverse-kick) so the hull '
                    'stops square; brake_off=true coasts, brake_gain scales the kick. A '
                    'gently-converged lock exits with ~0 momentum and is not kicked. '
                    'hold_s>0 turns it into an ACTIVE station-keep: once centred it keeps '
                    'correcting for hold_s seconds (fighting water inertia, e.g. to hold a '
                    'torpedo-hole lock steady for the shot) before exiting -- budget '
                    'duration >= approach + hold_s. '
                    'fire_channels (CSV e.g. "1,2") fires those payload channels ONCE '
                    'mid-hold, fire_t seconds into the hold (0 = at hold start), on a '
                    'background thread so the loop keeps correcting while the shot leaves '
                    '-- so a torpedo launches while still glued to the hole (no '
                    'align-then-fire drift). Requires fire_t < hold_s (else clamped to 0). '
                    'On duration expiry it logs NOT-aligned and the mission continues. '
                    'hold_through_loss=true coasts on target loss (set by the DSL when '
                    'no fallback search is supplied). '
                    'lock_target=true locks onto the acquired target (steers to the box '
                    'nearest the last-accepted centre, not the largest) so a second hole / '
                    'spurious box cannot steal the aim. ctrl_conf is a control-side min '
                    'score to accept a box. range_gain_floor softens lat/depth gain when '
                    'the bbox fills the frame (close) to stop overshoot. ki_lat adds a '
                    'lateral integral during the hold to null a steady current. '
                    'fwd_fill>0 adds a forward range-hold axis (mode=area/width/height, '
                    'kp_forward gain): align ALSO drives forward to that fill %% and holds '
                    'the standoff -- one verb does forward-standoff + lat/depth + hold + '
                    'mid-hold fire (the torpedo standoff shot). The fire is gated on the '
                    'standoff too. fwd_fill=0 (default) = no forward axis. '
                    'settle_px>0 = SETTLE GATE: only declare aligned once the hull is '
                    'in-band AND barely moving (|Δerr|<=settle_px) so it ends settled on '
                    'target like vision_move (not mid-pass through the band); 0=off. '
                    'depth_step = per-update depth-setpoint resolution (m, 0.02..0.10; 0='
                    'default 0.02): depth moves SLOWLY in these steps + freezes in the '
                    'deadband so ArduSub settles (no z-wobble). fire_pass_enabled = fire the '
                    'payload at command end even if never fully aligned, as long as the '
                    'target was seen live+recently (a guaranteed partial-points shot). '
                    'hold_heading = widen the heading-lock deadband during the hold so the '
                    'launcher heading holds steady (no terminal yaw jitter) when yaw is '
                    'released to the lock. '
                    'DOWNWARD CAMERA (camera=downward/sim_bottom): the frame rotates -- '
                    'image-X still drives Ch6 lateral, but the depth axis (image-Y) drives '
                    'Ch5 SURGE fore/aft (two-sided, braked), and fwd_fill DESCENDS the '
                    'ALT_HOLD depth to that bbox fill using the depth_step logic (proportional, '
                    'deadband-frozen, one-sided). Bounded by max_depth_m (deepest) and '
                    'depth_ceiling_m (shallowest -- surface guard so alignment cannot lift the '
                    'hull out of the water). surge_sign (+1/-1) flips fore/aft for the mount '
                    '(verify DISARMED). fire drops droppers (3/4). fire_gap spaces multi-channel '
                    'shots (fire=[1,4]) apart in seconds (solenoid needs the gap). See the bin task.',
        'fields':   ['camera', 'target_class', 'axes',
                     'offset_lat', 'offset_yaw', 'offset_depth',
                     'err_px', 'duration', 'gain',
                     'gain_lat', 'gain_yaw',
                     'brake_off', 'brake_gain', 'hold_s', 'hold_through_loss',
                     'fire_channels', 'fire_t',
                     'kp_lat', 'kp_yaw', 'kp_depth',
                     'lost_grace_s', 'align_stable_frames',
                     'lock_target', 'ctrl_conf', 'range_gain_floor', 'ki_lat',
                     'coast_s', 'fwd_fill', 'mode', 'kp_forward', 'settle_px',
                     'depth_step', 'fire_pass_enabled', 'hold_heading',
                     'surge_sign', 'max_depth_m', 'depth_ceiling_m', 'fire_gap',
                     'use_feature'],
        'defaults': {'camera': 'forward', 'target_class': '',
                     'axes': '', 'offset_lat': 0.0, 'offset_yaw': 0.0,
                     'offset_depth': 0.0, 'err_px': 40.0,
                     'duration': 20.0, 'gain': 30.0,
                     'gain_lat': 0.0, 'gain_yaw': 0.0,
                     'brake_off': False, 'brake_gain': 0.0, 'hold_s': 0.0,
                     'hold_through_loss': False,
                     'fire_channels': '', 'fire_t': 0.0,
                     'kp_lat': 60.0, 'kp_yaw': 60.0, 'kp_depth': 0.05,
                     'lost_grace_s': 1.0, 'align_stable_frames': 3.0,
                     'lock_target': False, 'ctrl_conf': 0.0,
                     'range_gain_floor': 1.0, 'ki_lat': 0.0, 'coast_s': 0.0,
                     'fwd_fill': 0.0, 'mode': 'area', 'kp_forward': 200.0,
                     'settle_px': 0.0, 'depth_step': 0.0,
                     'fire_pass_enabled': False, 'hold_heading': False,
                     'surge_sign': 0.0, 'max_depth_m': 0.0,
                     'depth_ceiling_m': 0.0, 'fire_gap': 0.0,
                     'use_feature': False},
    },
    'vision_move': {
        'help':     'Drive forward toward target_class. fwd_fill > 0 stops once the bbox '
                    'fills that %% of the frame (mode = area/width/height; height for tall '
                    'slalom). fwd_fill <= 0 (e.g. --fwd_fill -1, or DSL move(fwd=None)) is '
                    'PASS-THROUGH: drive until the target is seen and then leaves the frame, '
                    'plus a commit overshoot (hold_s, else ~2s) to carry the hull through a '
                    'gate. maintain_on holds a maintain_px lateral offset while moving; depth '
                    'and yaw are left to ArduSub / heading lock. gain caps forward speed; '
                    'gain_lat overrides the cap on the maintain strafe (0 = inherit gain). '
                    'On a fill-stop arrival the forward (and maintain) inertia is braked so '
                    'the hull halts in front of the target instead of creeping in; '
                    'PASS-THROUGH never brakes (it must coast through the gate). '
                    'brake_off=true coasts, brake_gain scales the kick. Does NOT re-centre.',
        'fields':   ['camera', 'target_class', 'fwd_fill', 'mode',
                     'maintain_px', 'maintain_on', 'hold_s',
                     'err_px', 'duration', 'gain', 'gain_lat',
                     'brake_off', 'brake_gain', 'hold_through_loss',
                     'kp_forward', 'kp_lat', 'lost_grace_s', 'range_gain_floor',
                     'coast_s'],
        'defaults': {'camera': 'forward', 'target_class': '',
                     'fwd_fill': 95.0, 'mode': 'area',
                     'maintain_px': 0.0, 'maintain_on': False,
                     'hold_s': 0.0, 'err_px': 40.0,
                     'duration': 20.0, 'gain': 30.0, 'gain_lat': 0.0,
                     'brake_off': False, 'brake_gain': 0.0,
                     'hold_through_loss': False,
                     'kp_forward': 200.0, 'kp_lat': 60.0, 'lost_grace_s': 1.0,
                     'range_gain_floor': 1.0, 'coast_s': 0.0},
    },
    'fire': {
        'help':     'Fire ESP32 payload channel. 1/2 = torpedo, 3/4 = dropper. '
                    'Requires payload_port to be connected (auto-detected at startup).',
        'fields':   ['fire_channel'],
        'defaults': {'fire_channel': 1.0},
    },
    # ---- Anchor (XFeat + LighterGlue geometric superglue) ------------- #
    # Command name == Duburi method name (manager dispatches getattr(duburi,cmd));
    # the DSL exposes them as duburi.vision.anchor_snap/clear/align.
    'vision_anchor_snap': {
        'help':     'Capture the current camera view as the anchor REFERENCE '
                    '(non-blocking). The anchor_node stores the next frame; '
                    'anchor_align then drives the hull back onto it. Pass ref_name '
                    'to ALSO save it to references/<ref_name>.png so it survives '
                    'restarts and reloads on competition day. With target_class+conf '
                    '(+err_px centring gate), waits up to 3s for that detection and '
                    'snaps JUST its bbox crop (keys the lock on the target, not the '
                    'background); falls back to the whole frame after 3s. Requires '
                    'the anchor node (launch vision.launch.py anchor:=true).',
        'fields':   ['camera', 'ref_name', 'target_class', 'conf', 'err_px'],
        'defaults': {'camera': 'forward', 'ref_name': '', 'target_class': '',
                     'conf': 0.5, 'err_px': 40.0},
    },
    'vision_anchor_clear': {
        'help':     'Drop the stored anchor reference so the next anchor_snap '
                    'starts fresh.',
        'fields':   ['camera'],
        'defaults': {'camera': 'forward'},
    },
    'vision_anchor_align': {
        'help':     'Superglue the hull to the snapped reference: drive lat from '
                    'the homography tx, yaw from theta, depth from ty until the live '
                    'view re-superimposes within err_px / theta_thresh, then hold for '
                    'hold_s (active station-keep). Optionally fire fire_channels (CSV, '
                    'e.g. "1,2") ONCE at first lock -- a torpedo leaves mid-hold while '
                    'the hull is glued. match = min RANSAC inliers to count a tick as '
                    'locked (0 = trust the node). gain caps speed; gain_lat/yaw/depth '
                    'override per axis. No forward axis (homography has no metric range). '
                    'Always returns success=True; outcome code in final_value.',
        'fields':   ['camera', 'err_px', 'theta_thresh', 'duration', 'gain',
                     'gain_lat', 'gain_yaw', 'gain_depth',
                     'brake_off', 'brake_gain', 'hold_s', 'fire_channels',
                     'min_inliers', 'ref_name', 'kp_lat', 'kp_yaw', 'kp_depth',
                     'lost_grace_s', 'align_stable_frames'],   # field names == method params
        'defaults': {'camera': 'forward', 'err_px': 20.0, 'theta_thresh': 0.05,
                     'duration': 30.0, 'gain': 30.0,
                     'gain_lat': 0.0, 'gain_yaw': 0.0, 'gain_depth': 0.0,
                     'brake_off': False, 'brake_gain': 0.0, 'hold_s': 0.0,
                     'fire_channels': '', 'min_inliers': 0.0, 'ref_name': '',
                     'kp_lat': 60.0, 'kp_yaw': 120.0, 'kp_depth': 0.05,
                     'lost_grace_s': 1.0, 'align_stable_frames': 3.0},
    },
}


# Field names that carry a string instead of a float (everything else is float).
STRING_FIELDS = ('target_name', 'camera', 'target_class', 'axes', 'mode',
                 'fire_channels', 'ref_name')

# Field names that carry a bool. rosidl init these to False.
BOOL_FIELDS = ('maintain_on', 'hold_through_loss', 'brake_off', 'pass_through',
               'lock_target', 'fire_pass_enabled', 'hold_heading', 'use_feature')


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
