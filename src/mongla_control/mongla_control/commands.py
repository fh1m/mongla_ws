"""Single source of truth for every /mongla/move command.

This is the ONE place that knows what commands exist, what fields each
one reads from `Move.Goal`, what the defaults are, and what to print in
`--help`. The action server, the Python client, and the `mongla` CLI
all read from here -- adding a new command is two edits, no more:

    1. Add a row to COMMANDS below.
    2. Add a method on `Mongla` whose name and parameter names match
       the row's `cmd` and `fields`.

Field names map directly to fields on `mongla_interfaces.action.Move`
(`duration`, `gain`, `target`, `target_name`, `timeout`, `settle`, ...). The dispatch just does
`getattr(mongla, cmd)(**kwargs_built_from_fields)`.

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
        'help':     'Disarm and confirm it took on the board\'s HEARTBEAT. '
                    'Signals abort first, so a running verb stops.',
        'fields':   ['timeout'],
        'defaults': {'timeout': 20.0},
    },
    'set_mode': {
        'help':     'Switch board mode: STABILIZE|DEPTH_HOLD|SURFACE|MANUAL|ACRO, '
                    'and verify it took (a silent refusal looks like success).',
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
        'help':     'Halt now: the board\'s MOVE_STOP brake.',
        'fields':   [],
        'defaults': {},
    },
    'mission_reset': {
        'help':     'Clear the abort flag, zero the demand, and re-zero depth if '
                    'disarmed at the surface. First line of every mission.',
        'fields':   [],
        'defaults': {},
    },
    'calibrate_depth': {
        'help':     'Re-zero the barometer at the surface '
                    'so depth reads 0 before a dive. DISARMED + surface only; verifies '
                    'the re-zero took. Auto-run by mission_reset; also standalone.',
        'fields':   [],
        'defaults': {},
    },
    'calc_distance': {
        'help':     "Downward optical-flow distance bracket: phase='start' latches "
                    "the axis + resets, 'stop' returns accumulated metres in "
                    "final_value.",
        'fields':   ['phase'],
        'defaults': {'phase': 'start'},
    },
    'surface': {
        'help':     'Emergency surface: brake, then the board\'s SURFACE mode '
                    '(ascend and hold). Bypasses the busy gate, so it works during '
                    'a running mission.',
        # timeout is the ascent budget. It MUST be registered (not just hardcoded in
        # surface()) so the client result-backstop covers the full ascent -- else the
        # 12 s safety floor truncates a deep emergency surface mid-water (a safety bug).
        'fields':   ['timeout'],
        'defaults': {'timeout': 60.0},
    },
    'pause': {
        'help':     'Hold station for `duration` s (the board\'s MOVE_HOLD).',
        'fields':   ['duration'],
        'defaults': {'duration': 2.0},
    },

    # ---- Forward / back -------------------------------------------- #
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

    # ---- Left / right ---------------------------------------------- #
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
        'help':     'Style: `flips` x 360 deg roll, run by the board '
                    '(MOVE_STYLE). `timeout` bounds the whole manoeuvre.',
        'fields':   ['flips', 'timeout'],
        'defaults': {'flips': 1, 'timeout': 20.0},
    },
    # ---- Yaw  (sharp pivots) --------------------------------------- #
    'yaw_left': {
        'help':     'Sharp pivot left by `target` degrees within `timeout` s.',
        'fields':   ['target', 'timeout', 'settle'],
        'defaults': {'timeout': 30.0, 'settle': 0.0},
        # A forgotten target would be a real one (north / the surface / 0 deg)
        # and is refused instead -- issue #14.
        'required': ['target'],
    },
    'yaw_right': {
        'help':     'Sharp pivot right by `target` degrees within `timeout` s.',
        'fields':   ['target', 'timeout', 'settle'],
        'defaults': {'timeout': 30.0, 'settle': 0.0},
        # A forgotten target would be a real one (north / the surface / 0 deg)
        # and is refused instead -- issue #14.
        'required': ['target'],
    },
    'turn': {
        'help':     'Rotate to absolute heading `target` degrees (0-360) via shortest '
                    'arc. Direction (left/right) is chosen automatically.',
        'fields':   ['target', 'timeout', 'settle'],
        'defaults': {'timeout': 30.0, 'settle': 0.0},
        # A forgotten target would be a real one (north / the surface / 0 deg)
        # and is refused instead -- issue #14.
        'required': ['target'],
    },

    # ---- Depth ----------------------------------------------------- #
    'set_depth': {
        'help':     'Hold absolute depth (`target` metres, negative below surface).',
        'fields':   ['target', 'timeout', 'settle'],
        'defaults': {'timeout': 30.0, 'settle': 0.0},
        # A forgotten target would be a real one (north / the surface / 0 deg)
        # and is refused instead -- issue #14.
        'required': ['target'],
    },

    # ---- Vision verbs --------------------------------------------- #
    # Exactly two, pixel-native, recover-don't-fail. The control loop
    # reads /detections directly (the topic the HUD shows). Both verbs
    # ALWAYS return success=True (the verb ran); the align/move outcome
    # is an integer code in Move.Result.final_value, so the mission DSL
    # branches on it and the action client never raises on a miss.
    #
    #   final_value codes: 0=ALIGNED/REACHED 1=LOST 2=TIMEOUT
    #                      3=NO_CAMERA 4=ABORTED 5=DRIFTED
    #
    # gain = hard max-speed cap (% thrust); the P-controller output is
    # clamped to it so the AUV never exceeds it on any axis.
    'vision_align': {
        'help':     'Centre target_class on the active axes (CSV of lat,yaw; on the '
                    'downward camera also depth, which drives SURGE) each at its signed '
                    'pixel offset (offset_lat/yaw/depth; 0=centre). Aligned when every '
                    'active axis is within err_px. gain caps speed; gain_lat/gain_yaw '
                    'override the cap per axis (0 = inherit gain). '
                    'On arrival the lateral inertia is braked (reverse-kick) so the hull '
                    'stops square; brake_off=true coasts, brake_gain scales the kick. '
                    'hold_s>0 keeps correcting for hold_s seconds once centred (station-'
                    'keep for a shot) -- budget duration >= approach + hold_s. '
                    'fire_channels (CSV e.g. "9,10") fires those board channels ONCE, '
                    'fire_t seconds into the hold, on a background thread so the loop '
                    'keeps correcting while the shot leaves. fire_gap spaces multi-channel '
                    'shots apart in seconds. fire_pass_enabled fires at command end even '
                    'if never fully aligned, as long as the target was seen recently. '
                    'hold_through_loss=true coasts on target loss. lock_target=true steers '
                    'to the box nearest the last-accepted centre, not the largest. '
                    'ctrl_conf is a control-side min score. range_gain_floor softens gain '
                    'when the bbox fills the frame. ki_lat adds a lateral integral during '
                    'the hold. fwd_fill>0 (forward camera) adds a forward range-hold axis '
                    '(mode=area/width/height, kp_forward gain) to that fill %% and gates '
                    'the shot on the standoff. settle_px>0 declares aligned only once the '
                    'hull is in-band AND barely moving. VERIFY (verify_off=true skips it) '
                    're-measures on align_stable_frames new frames after braking: DRIFTED '
                    'if out of band, LOST if not re-seen. surge_sign (+1/-1) flips the '
                    'downward surge for the mount. The board holds depth and heading '
                    'itself; a forward depth axis and a downward fwd_fill are REFUSED.',
        'fields':   ['camera', 'target_class', 'axes',
                     'offset_lat', 'offset_yaw', 'offset_depth',
                     'err_px', 'duration', 'gain',
                     'gain_lat', 'gain_yaw',
                     'brake_off', 'brake_gain', 'hold_s', 'hold_through_loss',
                     'fire_channels', 'fire_t',
                     'kp_lat', 'kp_yaw',
                     'lost_grace_s', 'align_stable_frames',
                     'lock_target', 'ctrl_conf', 'range_gain_floor', 'ki_lat',
                     'coast_s', 'lock_s', 'fwd_fill', 'mode', 'kp_forward', 'settle_px',
                     'verify_off', 'fire_pass_enabled',
                     'surge_sign', 'fire_gap',
                     'standoff_max_tilt_deg', 'fire_max_tilt_deg', 'tool'],
        'defaults': {'camera': 'forward', 'target_class': '',
                     'axes': '', 'offset_lat': 0.0, 'offset_yaw': 0.0,
                     'offset_depth': 0.0, 'err_px': 40.0,
                     'duration': 20.0, 'gain': 30.0,
                     'gain_lat': 0.0, 'gain_yaw': 0.0,
                     'brake_off': False, 'brake_gain': 0.0, 'hold_s': 0.0,
                     'hold_through_loss': False,
                     'fire_channels': '', 'fire_t': 0.0,
                     'kp_lat': 60.0, 'kp_yaw': 60.0,
                     'lost_grace_s': 1.0, 'align_stable_frames': 3.0,
                     'lock_target': False, 'ctrl_conf': 0.0,
                     'range_gain_floor': 1.0, 'ki_lat': 0.0, 'coast_s': 0.8, 'lock_s': 1.0,   # a SWITCH: >0 enables the ladder
                     'fwd_fill': 0.0, 'mode': 'area', 'kp_forward': 200.0,
                     'settle_px': 0.0, 'verify_off': False,
                     'fire_pass_enabled': False,
                     'surge_sign': 0.0, 'fire_gap': 0.0,
                     'standoff_max_tilt_deg': 0.0, 'fire_max_tilt_deg': 0.0,
                     'tool': ''},
    },
    'vision_move': {
        'help':     'Drive forward toward target_class. fwd_fill > 0 stops once the bbox '
                    'fills that %% of the frame (mode = area/width/height; height for tall '
                    'slalom). fwd_fill <= 0 (e.g. --fwd_fill -1, or DSL move(fwd=None)) is '
                    'PASS-THROUGH: drive until the target is seen and then leaves the frame, '
                    'plus a commit overshoot (hold_s, else ~2s) to carry the hull through a '
                    'gate. maintain_on holds a maintain_px lateral offset while moving; depth '
                    'and heading are held by the board. gain caps forward speed; '
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
                     'coast_s', 'lock_s'],
        'defaults': {'camera': 'forward', 'target_class': '',
                     'fwd_fill': 95.0, 'mode': 'area',
                     'maintain_px': 0.0, 'maintain_on': False,
                     'hold_s': 0.0, 'err_px': 40.0,
                     'duration': 20.0, 'gain': 30.0, 'gain_lat': 0.0,
                     'brake_off': False, 'brake_gain': 0.0,
                     'hold_through_loss': False,
                     'kp_forward': 200.0, 'kp_lat': 60.0, 'lost_grace_s': 1.0,
                     'range_gain_floor': 1.0, 'coast_s': 0.8,
                     # A SWITCH, not a duration: the ladder owns its own
                     # horizon (`lock_state.ZERO_AUTHORITY_S`) and stops
                     # publishing at zero. Any value > 0 enables it.
                     'lock_s': 1.0},
    },
    # ---- Per-thruster identification ------------------------------- #
    #
    # The board has implemented MAV_CMD_DO_MOTOR_TEST the whole time and
    # `SrotFC.motor_test` has driven it since 2026-09-07, but there was no verb,
    # so the one procedure that can turn ONE thruster and watch what happens was
    # reachable from the ground station and not from a mission, a script or the
    # CLI. It is the primitive underneath thrust-curve identification
    # (k_n_per_rpm2 is still null), dead-thruster detection, and checking that a
    # motor turns the way the mixer believes it does.
    #
    # This turns a thruster. Props off, or the vehicle restrained.
    'motor_test': {
        'help':     'Spin ONE thruster (1-8) at `gain` percent for `duration` s. '
                    'Requires ARMED. The board expires the test and '
                    'DISARMS within 3 s of the last keep-alive, which is the '
                    'safety mechanism -- there is no stop command. '
                    'PROPS OFF OR VEHICLE RESTRAINED.',
        # `target` carries the MOTOR INDEX (1..8). It is already the overloaded
        # axis-quantity field -- metres for set_depth, degrees for yaw_* -- so
        # reusing it costs no wire change and no mongla_interfaces rebuild. A
        # dedicated field would be cleaner and is not worth a wire break for it.
        'fields':   ['target', 'gain', 'duration'],
        'defaults': {'target': 1.0, 'gain': 20.0, 'duration': 2.0},
    },
    'fire': {
        'help':     'Activate payload BOARD channel N (1..16) -- the same n as '
                    'SERVO{n}_ROLE, no host-side map. The board decides: a SWITCH '
                    'channel fires, a PWM/SERVO channel is REFUSED (it is the '
                    'on-board arm). `mongla_manager connect` lists which is which.',
        'fields':   ['fire_channel'],
        'defaults': {'fire_channel': 0.0},
    },
}


# Field names that carry a string instead of a float (everything else is float).
# This mirrors the types in Move.action AND MUST NOT DRIFT FROM IT. The CLI
# builds `--flags` from this: a `string` field missing here is typed as a FLOAT
# and the flag becomes unusable -- `--tool torpedo` died with
# "invalid _float_or_head value: 'torpedo'", which only running it revealed,
# because every static check passes when the field merely EXISTS everywhere.
# `test_field_types_match_the_action` pins the two together.
STRING_FIELDS = ('target_name', 'camera', 'target_class', 'axes', 'mode',
                 'fire_channels', 'phase', 'tool')

# Field names that carry a bool. rosidl init these to False.
BOOL_FIELDS = ('maintain_on', 'hold_through_loss', 'brake_off', 'lock_target',
               'fire_pass_enabled', 'verify_off')


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

    The returned dict can be passed straight to the matching `Mongla`
    method as `**kwargs` -- the dispatch never has to know what each
    command is doing.
    """
    spec = COMMANDS[cmd]
    spec_defaults    = spec['defaults']
    runtime_defaults = runtime_defaults or {}
    # ISSUE #14. When the sender says which fields it SET, that list decides
    # -- a given 0.0 is a zero. Only a legacy sender (empty list) falls back to
    # inferring "unset" from the value, where 0.0 cannot be requested.
    given = set(getattr(request, 'set_fields', None) or ())
    kwargs = {}
    for field in spec['fields']:
        value = getattr(request, field)
        if given:
            unset = field not in given
            if unset and field in spec.get('required', ()) \
                    and field not in runtime_defaults and field not in spec_defaults:
                raise ValueError(f'{cmd}: {field!r} not set -- it has no default, '
                                 f'and 0.0 would be a real value, not a guess')
        elif field in STRING_FIELDS:
            unset = (value == '')
        elif field in BOOL_FIELDS:
            # bool unset == False; defaults explicitly set True or False.
            unset = (value is False) and (field in spec_defaults) and \
                    (spec_defaults[field] is True)
        else:
            unset = (value == 0.0)      # LEGACY sender only -- see above
        if unset:
            if field in runtime_defaults:
                value = runtime_defaults[field]
            elif field in spec_defaults:
                value = spec_defaults[field]
        kwargs[field] = value
    return kwargs
