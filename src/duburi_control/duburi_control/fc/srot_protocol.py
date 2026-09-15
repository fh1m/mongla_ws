#!/usr/bin/env python3
"""SROT wire protocol -- the SINGLE source of the SROT/Hengla MAVLink constants.

These ids and codes are already hand-mirrored in three other places: the ESP32
firmware (`srot-control-board`), Bondor's `bondor/src/shared/protocol.ts`, and the
SROT docs (`JETSON_COMMS.md`). This module is `duburi_ws`'s ONE copy -- every
SROT-specific number lives here so a firmware rev is a one-file update, never a
magic-number hunt across `srot_fc.py`.

Nothing here imports pymavlink or ROS: it is pure data + pure conversion helpers,
so it unit-tests without hardware. `srot_fc.py` is the only consumer.

References (all under Mongla_others/srot-control-board/):
  JETSON_COMMS.md  -- the authoritative wire contract (cmd 31000, modes, telemetry)
  ALGORITHMS.md    -- control semantics behind the numbers
  PARAMETERS.md    -- MOVE_* defaults, GAIN, MAG_YAW_REF
"""

from __future__ import annotations

# ---------------------------------------------------------------------- #
#  Link identity (JETSON_COMMS.md §1)                                     #
# ---------------------------------------------------------------------- #
# The vehicle (SROT board) is sysid 1 / compid 1; commands target it. We
# present as a GCS/companion (255/190, the pymavlink convention). The Jetson
# MUST beat a >=1 Hz HEARTBEAT or the board trips its GCS failsafe and surfaces.
VEHICLE_SYSID   = 1
VEHICLE_COMPID  = 1        # MAV_COMP_ID_AUTOPILOT1
SOURCE_SYSID    = 255
# 191 = MAV_COMP_ID_ONBOARD_COMPUTER. NOT pymavlink's default 190
# (MAV_COMP_ID_MISSIONPLANNER), and the difference is a failsafe, not cosmetics.
#
# The board's LoRa bridge synthesises its filler heartbeat as 255/190 -- byte-identical
# to what we used to send -- so the firmware could not distinguish the companion from
# the ground station. That makes a source-specific GCS failsafe impossible to write:
# a parameter naming "the Jetson" as 255/190 also matches the bridge, so a DEAD JETSON
# with Bondor still connected holds the failsafe open and the vehicle station-keeps
# when it should surface. The firmware team declined to ship a fake fix for this
# (their JETSON_FEEDBACK §4) and asked us to take a distinct id. This is that.
#
# Ordering is safe in both directions -- deliberately, so neither repo has to wait:
# the board counts ANY heartbeat whose id is not its own
# (`msg.compid != MAV_COMPONENT_ID || msg.sysid != MAV_SYSTEM_ID`,
# fw mav_commands.cpp:687), so 191 feeds the current failsafe exactly as 190 did.
#
# FS_GCS_SYSID / FS_GCS_COMPID HAVE LANDED (fw params.cpp:148-149) with defaults
# 255 / 191 -- exactly SOURCE_SYSID / SOURCE_COMPID below. That is now a LIVE
# COUPLING, not a future plan: if either side moves, the board's source-scoped GCS
# failsafe stops recognising us and it SURFACES mid-mission, looking for all the
# world like a link fault. test_srot_protocol_drift pins both against params.cpp.
#
# Applied on the SROT path only (auv_manager_node._setup_mavlink); the pixhawk
# backend keeps pymavlink's defaults so it stays byte-identical to history.
SOURCE_COMPID   = 191
GCS_FAILSAFE_MS = 5000     # board surfaces after this much HEARTBEAT silence
BAUD            = 115200   # ESP32 UART0; used on the direct-serial path

# ---------------------------------------------------------------------- #
#  Command ids (JETSON_COMMS.md §5, §10)                                  #
# ---------------------------------------------------------------------- #
# COMMAND_ACK.result values (MAVLink MAV_RESULT). Pinned here because older
# pymavlink dialects ship without CANCELLED=6, and these are the SROT move
# contract (JETSON_COMMS.md §5). IN_PROGRESS is the only non-terminal one.
ACK_ACCEPTED    = 0
ACK_DENIED      = 2
ACK_FAILED      = 4
ACK_IN_PROGRESS = 5
ACK_CANCELLED   = 6
# TEMPORARILY_REJECTED is the FIFTH dispatch outcome and is NOT in the four-result
# table JETSON_COMMS.md §5 documents -- but the firmware returns it whenever it
# misses a state mutex at dispatch: SROT_MOVE (mav_commands.cpp:329), accel-cal
# (:418) and DO_SET_SERVO (:464) on the payload path. The command never ran, so it
# is terminal for us: treating it as non-terminal means waiting out the whole
# deadline and reporting a bogus TIMEOUT for a move that was simply refused.
# Retrying is the caller's call, not this layer's.
#
# ⚠ FIXED 2026-08-03: this was 3, which is MAV_RESULT_UNSUPPORTED, not
# TEMPORARILY_REJECTED. Verified against the firmware's own vendored enum,
# `lib/mavlink/common/common.h:1151` -> TEMPORARILY_REJECTED=1. The old value
# broke the move path in BOTH directions and neither was visible in a log:
#   * a real mutex miss (wire 1) was not in TERMINAL_ACKS, so `_relay_move_ack`
#     polled until the budget expired and reported a bogus TIMEOUT + braked --
#     exactly the failure the paragraph above says it prevents; and
#   * a genuine UNSUPPORTED (wire 3, what the board answers for a command it does
#     not implement) was reported as "board busy -- safe to retry", which is the
#     worst possible advice for a command that will never be supported.
# `test_srot_protocol_drift` now pins every one of these against common.h.
ACK_TEMPORARILY_REJECTED = 1
ACK_UNSUPPORTED = 3        # command not implemented by this firmware -- never retry
TERMINAL_ACKS   = frozenset({ACK_ACCEPTED, ACK_DENIED, ACK_FAILED, ACK_CANCELLED,
                             ACK_TEMPORARILY_REJECTED, ACK_UNSUPPORTED})

ACK_NAMES = {ACK_ACCEPTED: 'ACCEPTED', ACK_TEMPORARILY_REJECTED: 'TEMPORARILY_REJECTED',
             ACK_DENIED: 'DENIED', ACK_UNSUPPORTED: 'UNSUPPORTED', ACK_FAILED: 'FAILED',
             ACK_IN_PROGRESS: 'IN_PROGRESS', ACK_CANCELLED: 'CANCELLED'}

CMD_SROT_MOVE = 31000      # the one custom verb; rides inside a COMMAND_LONG
CMD_DO_SET_SERVO = 183     # PCA9685 servo channel -> µs (payload release servo)
CMD_DO_SET_RELAY = 181     # PCA9685 MOSFET/switch channel on/off (payload solenoid)
CMD_USER_1    = 31010      # yaw spin (stunt)   -- style_yaw has no STYLE equivalent
CMD_USER_2    = 31011      # pitch spin (stunt)
CMD_USER_3    = 31012      # roll spin (stunt)
CMD_USER_4    = 31013      # pattern
CMD_USER_5    = 31014      # autotune start

# MAV_CMD_DO_MOTOR_TEST -- fully implemented on the board and unused by us until
# now. Standard ArduPilot semantics: p1 = motor (1-BASED), p2 = throttle type
# (0 = percent, 1 = raw PWM us, 2 = pilot -> DENIED), p3 = throttle, p4 = the
# keep-alive window in SECONDS.
#
# THREE PRECONDITIONS, all enforced by the firmware and all worth knowing before
# calling it (mav_commands.cpp:366-402):
#   * MOTORS MUST BE ARMED. Disarmed returns MAV_RESULT_FAILED with
#     "Arm motors before testing motors."
#   * IT IS A KEEP-ALIVE, NOT A ONE-SHOT. The board expires the test when the
#     resends stop -- and expiry AUTO-DISARMS. That is by design (it mirrors
#     ArduSub's verify_motor_test), so "the vehicle disarmed when I let go" is
#     the feature, not a fault.
#   * THE WINDOW IS CLAMPED to 600..3000 ms whatever p4 says, and p4 = 0 gives
#     600. Bondor asks for 1 s and refreshes every 300 ms.
CMD_DO_MOTOR_TEST = 209
MOTOR_TEST_THROTTLE_PERCENT = 0
MOTOR_TEST_THROTTLE_PWM     = 1
MOTOR_TEST_WINDOW_MIN_MS = 600     # the firmware's floor; a 0 or absent p4 lands here
MOTOR_TEST_WINDOW_MAX_MS = 3000    # ...and its ceiling
MOTOR_TEST_KEEPALIVE_HZ  = 3.3     # 300 ms, Bondor's cadence: >= 2 Hz with margin

# ---------------------------------------------------------------------- #
#  SROT_MOVE p1 -- WIRE type codes (JETSON_COMMS.md §5 p1 table)          #
# ---------------------------------------------------------------------- #
# NOTE: these are the 0-based WIRE codes sent in p1. The firmware stores
# movement::Type = p1 + 1 and reports THAT internal enum in MV_TYPE (so a
# `forward` p1=0 reads back as MV_TYPE=1). Any p1 outside 0..9 -> DENIED.
MOVE_FORWARD  = 0   # p2=duration_s   p3=speed 0..1
MOVE_BACK     = 1   # p2=duration_s   p3=speed 0..1
MOVE_STRAFE_L = 2   # p2=duration_s   p3=speed 0..1
MOVE_STRAFE_R = 3   # p2=duration_s   p3=speed 0..1
MOVE_TURN     = 4   # p2=degrees      p3=yaw rate deg/s (0->MOVE_YAW_RATE)  p4=0 rel/1 abs
MOVE_DIVE     = 5   # p2=depth_m (>=0) p3=ignored (rate=MOVE_DEPTH_RATE)
MOVE_STOP     = 6   # brake to a halt
MOVE_HOLD     = 7   # p2=seconds (0=until timeout); station-keep (0 translation, hold depth+heading)
MOVE_STYLE    = 8   # p2=count of 360 rolls (always ROLL at 90 deg/s)
MOVE_ARC      = 9   # p2=duration_s   p3=fwd speed 0..1   p4=signed yaw rate deg/s

# Absolute vs relative for MOVE_TURN p4. Absolute needs MAG_YAW_REF=1 on the board.
TURN_RELATIVE = 0
TURN_ABSOLUTE = 1

VALID_MOVE_TYPES = frozenset(range(0, 10))   # p1 0..9; anything else the board DENIEs

# ---------------------------------------------------------------------- #
#  Flight modes -- HEARTBEAT.custom_mode (JETSON_COMMS.md §3)             #
# ---------------------------------------------------------------------- #
MODE_STABILIZE   = 0
MODE_ACRO        = 1
MODE_DEPTH_HOLD  = 2
MODE_SURFACE     = 9
MODE_MANUAL      = 19
MODE_MOTOR_DETECT = 20
MODE_AUTOTUNE    = 21
MODE_MOTOR_TUNE  = 22
MODE_AUTO        = 23   # the mode SROT_MOVE runs in (auto-entered on a move)
# Command-only modes: DO_SET_MODE cannot select these, but HEARTBEAT.custom_mode
# CAN carry them, so they must still map. A mission never commands them; seeing one
# mid-run means something else put the board there (see MODE_NAMES).
MODE_STUNT       = 100  # N x 360 spin on one axis (CMD_USER_1/2/3)
MODE_PATTERN     = 101  # multi-step pattern (CMD_USER_4)

# custom_mode int -> name. DuburiState.mode carries the name; note DEPTH_HOLD is
# the SROT analogue of ArduSub ALT_HOLD but the NAMES differ -- any mission/FSM
# that branches on a mode string must be audited (set_depth engages ALT_HOLD on
# Pixhawk but a DIVE-in-AUTO on SROT).
#
# This is the COMPLETE FlightMode enum (fw state_types.h:33-45), not a selection of
# the ones we drive. `custom_mode` can carry any of them, and an unmapped value
# surfaces on /duburi/state as 'UNKNOWN(100)' -- which reads like a comms fault when
# it is actually a real, nameable mode. 20/22 spin motors and 100/101 move the hull,
# so a supervisor that cannot name them cannot react to them either.
MODE_NAMES = {
    MODE_STABILIZE:    'STABILIZE',
    MODE_ACRO:         'ACRO',
    MODE_DEPTH_HOLD:   'DEPTH_HOLD',
    MODE_SURFACE:      'SURFACE',
    MODE_MANUAL:       'MANUAL',
    MODE_MOTOR_DETECT: 'MOTOR_DETECT',
    MODE_AUTOTUNE:     'AUTOTUNE',
    MODE_MOTOR_TUNE:   'MOTOR_TUNE',
    MODE_AUTO:         'AUTO',
    MODE_STUNT:        'STUNT',
    MODE_PATTERN:      'PATTERN',
}
MODE_INTS = {name: num for num, name in MODE_NAMES.items()}

# ArduSub mode names accepted as aliases, so a mission written for the Pixhawk
# backend does not have to branch. ALT_HOLD *is* DEPTH_HOLD -- the firmware's own
# header says so verbatim: `DEPTH_HOLD = 2, // ArduSub ALT_HOLD` (state_types.h).
# Without this, every mission with a literal set_mode('ALT_HOLD') -- there are
# seven -- dies on srot with "unknown SROT mode".
#
# Deliberately NOT aliased: POSHOLD and GUIDED. The board has no position
# estimate at all, so silently accepting them would promise station-keeping it
# cannot deliver. Those still fail loudly, which is correct.
MODE_ALIASES = {'ALT_HOLD': MODE_DEPTH_HOLD}

# Modes the board reports but DO_SET_MODE cannot select -- they are entered by
# command only (DUBURI_WS_INTEGRATION.md §4.2 "Settable by DO_SET_MODE?").
#
# They are in MODE_NAMES because custom_mode carries them and telemetry must name
# them; they are excluded from mode_int() because a DO_SET_MODE to one would be
# accepted-and-ignored by the board, leaving set_mode() to burn its whole 8 s
# deadline and report the generic "mode stayed STABILIZE". Refusing up front says
# the true thing: the mode exists, this is not how you enter it.
MODE_NOT_SETTABLE = frozenset({MODE_STUNT, MODE_PATTERN})


def mode_name(custom_mode) -> str:
    """custom_mode int -> mode name, or 'UNKNOWN(<n>)' for an unmapped value."""
    try:
        return MODE_NAMES.get(int(custom_mode), f'UNKNOWN({int(custom_mode)})')
    except (TypeError, ValueError):
        return 'UNKNOWN'


def mode_int(name) -> int | None:
    """Mode name -> custom_mode int for a SETTABLE mode, else None (fail-loud upstream).

    Accepts the ArduSub aliases in MODE_ALIASES (currently just ALT_HOLD, which
    is the same mode as DEPTH_HOLD) so Pixhawk-era missions run unmodified.

    Returns None for MODE_NOT_SETTABLE (STUNT/PATTERN): those are real modes that
    `mode_name` must still name, but DO_SET_MODE cannot enter them -- use the
    CMD_USER_* commands."""
    key = str(name).strip().upper()
    num = MODE_INTS.get(key, MODE_ALIASES.get(key))
    if num is None or num in MODE_NOT_SETTABLE:
        return None
    return num


# ---------------------------------------------------------------------- #
#  MV_STATE -- movement phase (JETSON_COMMS.md §6 NAMED_VALUE_FLOAT)      #
# ---------------------------------------------------------------------- #
# Mirrors the firmware's phase enum EXACTLY:
#   enum { PH_IDLE = 0, PH_CRUISE, PH_BRAKE, PH_TURN, PH_DIVE, PH_STYLE,
#          PH_HOLD, PH_DONE };                       (fw movement.cpp:12)
#
# ⚠ 6 is HOLD and 7 is DONE. Both this file and JETSON_COMMS.md previously stopped
# at `6: done`, so a station-keeping HOLD leg was named 'done' while the vehicle was
# still holding -- and DONE itself fell through to the 'mv7' fallback. Corrected
# 2026-08-01 against the firmware source (fw changelog §6/§10).
#
# MV_STATE is a progress HINT either way: the terminal COMMAND_ACK is the only
# authority on completion, which is why this never surfaced as a stuck mission.
MV_IDLE   = 0
MV_CRUISE = 1
MV_BRAKE  = 2
MV_TURN   = 3
MV_DIVE   = 4
MV_STYLE  = 5
MV_HOLD   = 6
MV_DONE   = 7
MV_STATE_NAMES = {
    MV_IDLE: 'idle', MV_CRUISE: 'cruise', MV_BRAKE: 'brake', MV_TURN: 'turn',
    MV_DIVE: 'dive', MV_STYLE: 'style', MV_HOLD: 'hold', MV_DONE: 'done',
}


def mv_state_name(state) -> str:
    try:
        return MV_STATE_NAMES.get(int(state), f'mv{int(state)}')
    except (TypeError, ValueError):
        return 'idle'


# ---------------------------------------------------------------------- #
#  MANUAL_CONTROL scaling (JETSON_COMMS.md §4, DUBURI_WS_INTEGRATION §4.1)#
# ---------------------------------------------------------------------- #
# x=forward, y=lateral(+starboard), r=yaw all in -1000..1000; z=heave in
# 0..1000 with 500=NEUTRAL and z>500 = UP/ascend. The board CLAMPS these at
# entry (out-of-range is silently truncated, not rejected), so we clamp too --
# a units bug should under-drive, never burst.
MC_AXIS_MAX = 1000    # x/y/r full-scale
MC_Z_MAX    = 1000    # z full-scale
MC_Z_NEUTRAL = 500    # z centre = hold depth


def _clamp(v: float, lo: float, hi: float) -> float:
    """Clamp to [lo, hi]. NaN passes THROUGH -- callers must handle it.

    ⛔ DO NOT "fix" this by clamping NaN to `lo`. Comparisons against NaN are all
    False, so NaN does fall through both branches -- but `lo` is only the safe
    answer where lo happens to BE the neutral value. For `unit_to_mc` lo is
    -1.0, i.e. FULL REVERSE THRUST, and for `unit_to_mc_z` it is full descend.
    That fix was written and caught here during B35 precisely because a
    range check accepts -1000 happily. Each converter names its OWN neutral.
    """
    return lo if v < lo else hi if v > hi else v


def unit_to_mc(u: float) -> int:
    """-1..1 -> -1000..1000 for MANUAL_CONTROL x/y/r (clamped). NaN -> 0 (NEUTRAL).

    Neutral, not the clamp floor: -1000 is full reverse on this axis (B35)."""
    u = float(u)
    if u != u:                     # NaN
        return 0
    return int(round(_clamp(u, -1.0, 1.0) * MC_AXIS_MAX))


def unit_to_mc_z(u: float) -> int:
    """-1..1 (up positive) -> 0..1000 with 500 neutral for MANUAL_CONTROL z (clamped).

    +1 (full ascend) -> 1000, 0 (hold) -> 500, -1 (full descend) -> 0. Matches the
    board's convention that z>500 ascends (heave positive = UP).

    NaN -> MC_Z_NEUTRAL (500 = hold depth). NOT the clamp floor, which is full
    descend (B35)."""
    u = float(u)
    if u != u:                     # NaN
        return MC_Z_NEUTRAL
    return int(round(MC_Z_NEUTRAL + _clamp(u, -1.0, 1.0) * (MC_Z_MAX - MC_Z_NEUTRAL)))


def pct_to_mc(p: float) -> int:
    """-100..100 % -> -1000..1000 (the old percent API -> MANUAL_CONTROL axis).

    NaN -> 0 (neutral), for the same reason as `unit_to_mc` (B35)."""
    p = float(p)
    if p != p:
        return 0
    return int(round(_clamp(p, -100.0, 100.0) * 10))


def pct_to_mc_z(p: float) -> int:
    """-100..100 % (up positive) -> 0..1000, 500 neutral. NaN -> 500 (B35)."""
    p = float(p)
    if p != p:
        return MC_Z_NEUTRAL
    return int(round(MC_Z_NEUTRAL + _clamp(p, -100.0, 100.0) * 5))


# ---------------------------------------------------------------------- #
#  Movement param defaults (PARAMETERS.md / JETSON_COMMS.md §8)           #
# ---------------------------------------------------------------------- #
# For reference + host-side sanity clamps; the board owns the live values.
MOVE_CRUISE_MAX = 0.80    # clamps a move's p3 speed
MOVE_YAW_RATE   = 45.0    # deg/s default turn rate when p3=0
MOVE_DEPTH_RATE = 0.20    # m/s dive/ascend ramp
GAIN_FOR_AUTONOMY = 1.0   # MANUAL_CONTROL is halved until GAIN=1.0 (boots at 0.5)

# --- the LIVE pilot gain, and why setting the parameter cannot reach it ------
#
# `JS_GAIN_DEFAULT` is only the POWER-ON value. The firmware keeps the live gain
# in `s_gain_live`, which lazily adopts the parameter "on first use"
# (mav_commands.cpp:218) -- and `mav_stream.cpp:850` reads `pilotGain()` in the
# telemetry loop to publish the GAIN channel. That loop runs from boot, so the
# latch has ALWAYS happened before a companion finishes connecting.
#
# So `set_default_gain()` cannot affect the session that calls it, by
# construction. Measured on the live board: after our startup writes
# JS_GAIN_DEFAULT = 1.0, every GAIN sample across a 44 s capture read 0.500.
# The firmware's own source records the cost of this trap -- "They lost a whole
# pool session to exactly this: a JS_GAIN_DEFAULT = 1.0 write that never
# persisted, leaving MANUAL_CONTROL at half authority with nothing to indicate
# it."
#
# The one in-session path the firmware offers is a joystick button function
# delivered in MANUAL_CONTROL.buttons, dispatched on the PRESS EDGE
# (`mc.buttons & ~s_prev_buttons`). Verified live: 0.5 -> 1.0 in five presses,
# the board announcing "Gain 60%" ... "Gain 100%" as it went.
# MAV_SYS_STATUS_SENSOR_LEAK, from their vendored common.h:139. The board sets
# `present` always, `enabled` from LEAK_EN, and `health` SET = dry / CLEAR = leak.
# --------------------------------------------------------------------------- #
#  LANDING_TARGET uplink: the class map and the timeout ladder                 #
# --------------------------------------------------------------------------- #
#
# ⚠ APPEND-ONLY, and 0 is reserved. Same discipline as the firmware's
# `SROT_SERVO_FUNC_*` and `movement::Type`: these numbers cross a repo boundary
# by hand, so INSERTING a value silently renames every class after it. 0 must
# stay UNSPECIFIED so an unconfigured sender reads as "unassigned" rather than
# as whichever class happens to be first.
#
# WHY THIS EXISTS AT ALL: our two senders disagreed. `auv_manager_node`'s uplink
# tick sent a hardcoded `target_num=0` while `tools/srot_uplink_check.py` sent
# `d.class_id` -- the detector's own index, which is a property of whichever
# model happens to be loaded and means something different for every one. So
# the field was either meaningless or model-dependent, and neither is a wire
# contract. Proposed upstream in PR #2 (VISION_API §6, answering their Q5).
UPLINK_CLASS_UNSPECIFIED = 0
# LANDING_TARGET.z: which clock `time_usec` is on. 0.0 = host wall clock,
# advisory (VISION_API §1.2, the original contract). 1.0 = microseconds since
# BOARD boot, the frame's capture instant mapped through ClockMap, so the board
# can de-rotate the bearing by its own gyro. Proposed upstream; the board
# ignores z today, so sending it is harmless before the PR merges.
UPLINK_TIME_BOARD = 1.0
# VISION_SPEED_ESTIMATE.covariance[8]: vertical velocity variance. A downward
# camera cannot see vertical motion, so it is sent as unobserved (1e6 m^2/s^2)
# rather than as a confident zero that would fight the board's depth loop.
SPEED_Z_UNOBSERVED_VAR = 1.0e6

# VISION_POSITION_ESTIMATE.covariance is the upper triangle of the 6x6 over
# (x, y, z, roll, pitch, yaw), row-major, 21 entries. These are the diagonal
# slots; (0,1) is the x-y cross term. A mistake here swaps confidence between
# axes with no error anywhere, which is why they are named rather than inlined.
POS_COV_IDX = {'xx': 0, 'xy': 1, 'yy': 6, 'zz': 11, 'rr': 15, 'pp': 18, 'yawyaw': 20}
# A position that jumps further than this between two estimates is a RESET (a
# prop fix, a heading anchor), not motion at any speed the hull can reach at
# 10 Hz. MAVLink's reset_counter tells the board to drop its deltas across it.
POS_RESET_JUMP_M = 0.5
UPLINK_CLASSES = {
    'gate':       1,
    'flare':      2,
    'red_pipe':   3,   # slalom
    'bin':        4,
    'blood':      5,
    'fire':       6,
    'torpedo':    7,
    'hole':       8,
    'rescue':     9,
    'person':    10,
}
UPLINK_CLASS_MAX = 10   # bump when appending; never renumber


def uplink_class_num(name) -> int:
    """Class name -> frozen wire id. Unknown or empty -> 0 = UNSPECIFIED.

    Case-insensitive because the detector's labels and a mission's strings have
    drifted in case before. An unmapped class deliberately sends 0 rather than
    raising: the bearing is still correct and still useful, and refusing to
    uplink a target because nobody has assigned it a number yet would be a
    worse failure than an unlabelled one.
    """
    if not name:
        return UPLINK_CLASS_UNSPECIFIED
    return UPLINK_CLASSES.get(str(name).strip().lower(),
                              UPLINK_CLASS_UNSPECIFIED)


# The timeout ladder, declared as a cross-repo contract in VISION_API.md and
# asserted NOWHERE until now. The ordering is the whole point: each rung must
# expire strictly before the next, so authority reaches zero BEFORE loss is
# declared -- which is what makes a detection dropout a glide rather than a
# lurch. One edit to any of these silently breaks that, and the symptom is a
# vehicle that jerks when a box flickers.
UPLINK_FRESHNESS_S   = 0.10   # a sample older than this is not "this frame"
UPLINK_COAST_S       = 0.40   # steer on the tracker's prediction up to here
UPLINK_LOST_GRACE_S  = 0.80   # authority ramps to zero across this
UPLINK_DECLARE_LOST_S = 1.00  # only now is the target gone
UPLINK_LADDER = (UPLINK_FRESHNESS_S, UPLINK_COAST_S,
                 UPLINK_LOST_GRACE_S, UPLINK_DECLARE_LOST_S)

SYS_STATUS_SENSOR_LEAK = 2

JS_FUNC_NONE      = 0
JS_FUNC_GAIN_INC  = 42
JS_FUNC_GAIN_DEC  = 43
GAIN_MIN          = 0.10   # mav_commands.cpp:38
GAIN_MAX          = 1.00
GAIN_STEP         = 0.10
BTN_FUNC_PARAM    = 'BTN{}_FUNCTION'    # BTN0..BTN15
N_BUTTONS         = 16

# What the board does to a translation demand AFTER the gain, which our control
# loop does not model:
#
#     demand = ((1 - PILOT_EXPO) * sp + PILOT_EXPO * sp**3) * PILOT_SPEED
#
# with DEF_PILOT_EXPO = 0.30 and DEF_PILOT_SPEED = 1.0 (config.h:537-538). So
# the SMALL-SIGNAL gain a vision loop actually sees is
# `GAIN * (1 - PILOT_EXPO)` -- 0.35 at boot, 0.70 even after the gain fix
# above. The expo exists so a human stick has fine resolution near centre; for
# a proportional controller it is an unmodelled 30 % droop exactly in the
# terminal-alignment regime, plus a cubic term that makes one kp wrong at both
# ends of the error range.
PILOT_EXPO_DEFAULT  = 0.30
PILOT_SPEED_DEFAULT = 1.0

# The arming guard's real threshold, in METRES of depth error at the surface.
#
# It used to be the 0.90 above, applied to DEPTH_OUT. Two things changed that:
# fw rev 8 suppresses DEPTH_OUT while the controller is not running (so the guard
# had nothing to read while disarmed, which is the only time it runs), and the
# replacement signal DEPTH_CMD is `clamp(DEPTH_P * (depth - 0.10))` -- a clamped
# OUTPUT, not an error. Comparing a clamped output against a fixed number silently
# means a different physical depth the moment anyone retunes DEPTH_P.
#
# So the guard divides back out: implied_err_m = DEPTH_CMD / DEPTH_P, compared
# against this. 0.30 m reproduces the old behaviour exactly at DEPTH_P = 3.0
# (3.0 * 0.30 = 0.90) and now survives a gain change. The 2026-08-02 phantom baro
# read -3..-6.7 m, i.e. 10x over.
DEPTH_ERR_ARM_LIMIT_M = 0.30

# fw `mav_stream.cpp`: DEPTH_CMD = depth::preview(s.depth, 0.10f), and preview() is
# `clamp(DEPTH_P * (meas - tgt))`. So the 0.10 m target is BAKED INTO the number and a
# perfectly-zeroed barometer sitting in air still reads -0.10 m of "error".
#
# That matters because the quantity this guard actually wants is HOW FAR THE BAROMETER
# IS FROM ZERO at the surface, not how far it is from the preview's target. Comparing
# the raw error against 0.30 m spends a third of the budget on a constant offset --
# measured on the vehicle 2026-08-07, a healthy board at +0.15 m of baro offset read
# -0.25 m and sat 0.05 m from a FAIL it did not deserve.
#
# Recovering the board's own depth is exact while unsaturated:
#     depth = DEPTH_CMD / DEPTH_P + DEPTH_PREVIEW_TARGET_M
DEPTH_PREVIEW_TARGET_M = 0.10

# ...but NOT while clamped, which is the case that matters most: at |DEPTH_CMD| = 1.0
# the true depth is somewhere beyond the clamp and the formula above would report a
# harmless -0.23 m. So saturation is its own refusal, checked FIRST. The 2026-08-02
# phantom baro pinned here.
DEPTH_CMD_SATURATED = 0.99

# Baro offset that earns a WARN rather than a refusal: real, worth a `calibrate_depth`,
# not dangerous. Below the FAIL limit by enough that a normal offset does not cry wolf --
# a guard that fires on a healthy vehicle is a guard that gets overridden by habit.
DEPTH_OFFSET_WARN_M = 0.20

# fw `DEF_DEPTH_P` (include/config.h). Fallback when the board never answered the
# param read -- pinned by the drift test so it cannot rot, and the pin EARNED ITS KEEP:
# fw f533bd2 moved it 3.0 -> 0.5 after the 2026-08-07 water test ("my structural estimate
# of P was right; D was 20x too strong" -- the working set is P 0.5 / I 0.1 / D 0.01).
#
# The direction of this error is what matters. `check_depth_loop_settled` recovers the
# depth error as `DEPTH_CMD / depth_p`, so assuming 3.0 against a board running 0.5
# divides the recovered error by SIX: 1.8 m of real error would read as 0.30 m and pass
# the arming guard. It fails OPEN, on the one check standing between a phantom baro and
# full vertical thrust. Read from the board when it answers; this is only the fallback.
DEPTH_P_DEFAULT = 0.5

# YAW_REF (NAMED_VALUE_FLOAT, fw rev 9) -- `yaw_ref::State`. ONLY 2 means the heading
# is absolute; anything else and ATTITUDE.yaw is relative to wherever the BNO booted,
# which makes an absolute MOVE_TURN (p4=1) turn to a meaningless number.
#
# MAGACC IS NOT A PROXY FOR THIS and must not be used as one: the alignment is
# protected by the |B| band and the sample-agreement test, neither of which depends
# on the sensor's self-assessment, and with a stored calibration `need_acc` drops to
# 0 so accuracy stops correlating with the outcome entirely.
YAW_REF_IDLE          = 0
YAW_REF_SAMPLING      = 1
YAW_REF_LOCKED        = 2
YAW_REF_REFUSED_CAL   = 3
YAW_REF_REFUSED_FIELD = 4
YAW_REF_REFUSED_NOISE = 5
YAW_REF_NAMES = {
    0: 'IDLE (MAG_YAW_REF off, or nothing attempted)',
    1: 'SAMPLING (still collecting)',
    2: 'LOCKED (heading is absolute)',
    3: 'REFUSED_CAL (mag accuracy too low)',
    4: 'REFUSED_FIELD (field magnitude implausible -- hard iron nearby)',
    5: 'REFUSED_NOISE (samples disagreed -- vehicle moving, or interference)',
}

# ---------------------------------------------------------------------- #
#  Firmware behaviour revision -- the cross-repo coordination signal      #
# ---------------------------------------------------------------------- #
# The board bumps SROT_FW_BEHAVIOUR_REV (fw include/config.h) whenever it changes
# OBSERVABLE behaviour that we have written a workaround for. Assert on THIS, never
# on the shape of their C++.
#
# We learned that the hard way. `test_stop_still_does_not_apply_reverse_thrust`
# detected the MOVE_STOP fix by grepping their `Type::STOP` case for `abort()`.
# They fixed it a different way -- restoring the outgoing leg's axis and speed,
# which `start()` had zeroed before the switch -- so our tripwire stayed GREEN
# while the behaviour changed underneath it. Had we trusted it we would have kept
# the host brake and kicked the hull twice.
#
#   1  MOVE_STOP coasts. A move can be stranded on IN_PROGRESS for ever when a
#      failsafe or mode change displaces AUTO. No SET_MESSAGE_INTERVAL. ESC_STATUS
#      only (undecodable). DIVE target not clamped. TURN/DIVE progress constant.
#   2  fw 2026-08-01 (AUDIT.md R35-R44): MOVE_STOP brakes -- HOST BRAKE REMOVED.
#      Every move reaches exactly one terminal ACK, sent once. 511/510 implemented.
#      ESC_TELEMETRY_1_TO_4 / _5_TO_8 emitted. DIVE clamped >= 0. Real TURN/DIVE
#      progress. SURFACE zeroes pilot translation/yaw; MANUAL_CONTROL ages out.
#   3  fw 2026-08-02 (AUDIT.md R46-R50): the Bar30's calibration PROM is validated (CRC-4)
#      and no longer read in a race, so DEPTH and TEMPERATURE can now be ABSENT rather than
#      silently wrong. WTEMP and SCALED_PRESSURE2 are SUPPRESSED when the baro is unhealthy
#      or stale; SCALED_IMU2.temperature sends 0 (MAVLink's "not provided"); a board whose
#      PROM fails CRC REFUSES DEPTH_HOLD/AUTO/PATTERN. SYS_STATUS gains the extended health
#      bitfield with LEAK on MAV_SYS_STATUS_SENSOR_LEAK. FS_GCS_SYSID/COMPID (255/191) scope
#      the GCS failsafe to a named companion. ESPNOW_EN is tri-state, so Battery 2 populates.
#   4  fw 2026-08-02 (AUDIT.md R51-R54): YAW IS NOW ABSOLUTE. MAG_YAW_REF defaults on and
#      is migrated on for boards storing the old 0, so ATTITUDE.yaw / VFR_HUD.heading are a
#      magnetic compass heading instead of relative to wherever the BNO booted. A heading
#      compared across a vehicle RESET will differ from rev <= 3 -- that is the fix, not a
#      regression, and absolute MOVE_TURN (p4=1) finally turns to the heading it is given.
#      The BNO's own calibration is persisted to sensor flash (mag accuracy survives a power
#      cycle), and the mag report stops once the one-shot reference locks.
#   5  fw 2026-08-03: BARO JITTER GATE -- the barometer is judged on the VARIANCE of the
#      stream, not only per-sample plausibility. This is the fix for the bench finding we
#      reported: a failing Bar30 produced 317-874 mbar sample-to-sample while EVERY reading
#      sat inside the wide plausibility band, so depth stayed "healthy" and its phantom value
#      saturated the depth PID into full vertical thrust on arming. Peak-to-peak > 15 mbar
#      over an 8-sample window now marks the baro unhealthy (suppressing WTEMP /
#      SCALED_PRESSURE2 and refusing DEPTH_HOLD/AUTO), and the measure itself is published
#      as NAMED_VALUE_FLOAT "BARO_P2P" -- ungated, deliberately, so it can explain WHY depth
#      withdrew. Observed live at 5.90 mbar, matching our own independently-computed spread.
#   6  fw 2026-08-06: MOTOR_DETECT converges, and can no longer wipe a good calibration.
#      Its pulse is driven THROUGH the direction flag, so it measures an AGREEMENT rather
#      than an absolute direction; storing that as the new absolute cal made the routine
#      converge only when CAL_MDIRn was already +1, so an inverted thruster stayed inverted
#      through any number of runs. It now composes. Separately, an inconclusive detect used
#      to default to +1 and store it, so a run in air silently reset all eight thrusters and
#      reported SUCCESS; it now leaves the value alone and finishes FAIL (which also keeps
#      it out of flash). Nothing for this host to adapt to -- duburi_ws never runs
#      MOTOR_DETECT -- but it changes what an operator should expect from Bondor.
#   7  fw 2026-08-07 (PR #5): PREFLIGHT_STORAGE reports SUCCESS, not only failure. The
#      command has always been ACKed ACCEPTED the instant it parses, but the NVS write is
#      deferred to the Core-0 update() -- so the ACK has never meant "written", only
#      "request received", and the only signal was a CRITICAL statustext on FAILURE with
#      silence on success. Silence is indistinguishable from a dropped request or a wedged
#      board. Rev 7 emits STATUSTEXT "Params saved to flash" (MAV_SEVERITY_INFO) on the
#      success path: if you ever automate a save, wait for the statustext, NOT the ACK.
#
#      ⚠ THE REV NUMBER UNDER-COUNTS THIS RELEASE. Five other commits shipped inside rev 7
#      without a bump of their own, and two change behaviour we care about:
#        * 83ef62e FRAME_REVERSE -- a new param that negates all six axis demands after the
#          controllers and before the mixer. DEFAULT 0, so a stock board is unchanged and
#          the gate is honest; but it is SET TO 1 on our hull, where it inverts the meaning
#          of every MANUAL_CONTROL axis and every SROT_MOVE primitive relative to rev <= 6.
#        * 502eb23 MOT_1/MOT_8_DIRECTION now DEFAULT to -1 (as-flown), the rest +1. Our
#          documented restore was a uniform [-1] x 8, which is no longer the intended
#          configuration and interacts with FRAME_REVERSE. See srot-integration.md.
#      The other three are not ours: 4aaa755 adds a COMP_SEEN NAMED_VALUE_FLOAT (a Bondor
#      signal -- inert here, our reader table is per-name), b447fe5 unswaps the PM1
#      volt/curr pins (a battery reading we only display), 2213c9a is their AGENTS.md.
#      Neither of the two that ARE ours is a host-code change -- the wire is identical, and the drift suite is green
#      against d6f1da5 -- but do not read "rev 7" as "one small additive change".
#   rev 10 (2026-08-07) YAW SENSE IS CORRECTED, AND THIS ONE IS NOT ADDITIVE.
#      BNO_SWAP_ROLL_PITCH was an improper transform (swap x/y, keep z -- determinant -1),
#      leaving the frame left-handed and SILENTLY INVERTING YAW while roll and pitch read
#      correctly. Measured in water 2026-08-07: turning right made yaw DECREASE. MANUAL was
#      fine (open loop); STABILIZE and DEPTH_HOLD span the hull.
#      *** ANY HEADING RECORDED AGAINST REV <= 9 HAS THE OPPOSITE SENSE ***, absolute
#      MOVE_TURN targets included. Every heading constant, pool-day note and recorded
#      competition_config heading taken before this is negated. The mag alignment must be
#      re-run after flashing. This is why the constant below is not merely "bumped".
#      Also: MOTOR_DETECT now decides on the gyro projection onto the expected angular
#      direction (the old "dominant axis" was degenerate for M5-M8 and gave a wrong vertical
#      result) and ends DISARMED in MANUAL rather than armed in STABILIZE.
#   rev 11 autotune limit-cycle validity is median + consensus, not min-vs-max.
#   rev 12 autotune holds depth through the rate/angle phases.
#   rev 13 SROT_MOVE REQUIRES ARMED. The movement state machine never consulted the arm
#      state, so a move sent while disarmed ran its whole profile with the thrusters silent
#      and finished ACCEPTED at 100 %. Measured: a 3 m FORWARD "completed" in 3.5 s having
#      moved nothing -- and the doc names US as the consumer that would advance an entire
#      mission on a dead hull. It is now TEMPORARILY_REJECTED + "SROT_MOVE refused: arm
#      first". See ACK_TEMPORARILY_REJECTED below: our retry-on-that-code path must not
#      spin forever on a disarmed board.
#      Also adds per-axis ATC_RAT_*_FLTD and the previously absent ATC_RAT_*_FLTT.
#   rev 14 the depth refusal names WHY instead of one identical "No depth sensor" for three
#      faults, publishes BARO_HEALTH, and makes the jitter threshold the BARO_JIT_MAX param.
FW_BEHAVIOUR_REV = 14

# The minimum revision this host code assumes. Flashing older firmware than this
# re-opens the coasting MOVE_STOP with no host brake left to cover it.
#
# NOT raised to 7 despite FRAME_REVERSE, deliberately: its default is 0, so a stock rev-7
# board and a rev-6 board command identical axes and there is no host workaround to gate.
# The axis flip is a PARAM this hull sets, not a revision property -- gating it here would
# strand a working rev-2 board while still not catching a rev-7 board with FRAME_REVERSE
# left at 0. The check that would actually catch it is a param read, not a rev compare.
# RAISED 2 -> 10 (2026-09-03). Rev 10 corrected an INVERTED YAW SENSE: below it,
# STABILIZE and DEPTH_HOLD span the hull because turning right makes yaw decrease,
# while roll and pitch read correctly so nothing looks wrong. Any host loop that closes
# on yaw -- which is exactly what the vision path does -- diverges on a rev <= 9 board.
# That is worth refusing hardware for; the older reasoning below is kept because it is
# still the right question to ask of every future rev.
FW_BEHAVIOUR_REV_REQUIRED = 10

# WHERE THE BOARD REPORTS IT: `AUTOPILOT_VERSION.middleware_sw_version`. The board has
# no middleware, so that field was zero and free; request the message with
# MAV_CMD_REQUEST_MESSAGE(148). `SrotFC.check_behaviour_rev()` reads it and REFUSES TO
# ARM below the requirement. Verified on hardware: the board answers 2.
#
# The drift test that checks this same number is NOT a substitute. It reads the firmware
# repo off disk and skips when that repo is not checked out beside the workspace -- i.e.
# it skips on the vehicle, the one place the answer matters. Grepping a sibling repo is a
# developer-workstation convenience; the wire is the contract.
#
# 0 means "firmware older than 2026-08-01" (that build never populated the field), NOT
# "unknown". Fail closed on 0.
MSG_ID_AUTOPILOT_VERSION = 148

# ESC_STATUS msgid, as an INTEGER LITERAL on purpose.
#
# `mavutil.mavlink.MAVLINK_MSG_ID_ESC_STATUS` does not exist: upstream removed the
# WIP messages 290/291 from `common`, so pymavlink has no symbol for it and naming
# it raises AttributeError at import. We still need the number, because it is the
# stream key the board rates its RPM output by -- one SET_MESSAGE_INTERVAL on 291
# paces ESC_STATUS *and* the ESC_TELEMETRY_1_TO_4/5_TO_8 pair we actually decode.
MSG_ID_ESC_STATUS = 291

# ---------------------------------------------------------------------- #
#  Two batteries -- the board sends BATTERY_STATUS twice, with different  #
#  instance ids, and they mean physically different things.               #
# ---------------------------------------------------------------------- #
# MEASURED on the vehicle 2026-08-02: both stream at 2 Hz. PM1 is the electronics
# pack read by the ESP32's own ADC; PM2 is the THRUSTER pack, which the flight
# controller cannot read directly -- it arrives over ESP-NOW from the 2nd board, so
# it is absent whenever that link is down or `PM2_SRC`/`ESPNOW_EN` are misconfigured.
#
# Mixing them up is not cosmetic: on this hull PM1 read 1.35 V (nothing is wired to
# GPIO36) while PM2 read 14.74 V. A consumer that samples pymavlink's single
# per-msgid slot alternates between the two.
BATTERY_ID_MAIN     = 0   # PM1, electronics rail -> DuburiState.battery_voltage
BATTERY_ID_THRUSTER = 1   # PM2, thruster pack, via ESP-NOW

# ⛔ NAMED_VALUE_FLOAT's `name` field is 10 CHARS and the board does not shorten its
# own strings -- so "BARO_HEALTH" arrives as "BARO_HEALT", truncated silently. Looking
# up the full name finds nothing and returns None, which is indistinguishable from
# "the board did not send it". This is the same 10-char trap that produced our worst
# misreading of this signal: BARO_HEALT was read as a health SCORE, so `3` looked
# excellent when it means NOT INITIALISED.
NAME_BARO_HEALTH = 'BARO_HEALT'

# bar30.h:33. A fault CODE, not a score -- larger is worse, and 0 is the good one.
BARO_HEALTH_TEXT = {
    0: 'healthy',
    1: 'jitter (peak-to-peak over BARO_JIT_MAX)',
    2: 'read failures',
    3: 'not initialised',
}


def baro_health_text(value):
    """Human text for a BARO_HEALTH code. None/NaN -> 'not reported'.

    Never invents a reading: an unknown code is reported as unknown rather than
    mapped to the nearest known one.
    """
    if value is None:
        return 'not reported'
    try:
        if value != value:            # NaN
            return 'not reported'
        code = int(round(float(value)))
    except (TypeError, ValueError):
        return 'not reported'
    return BARO_HEALTH_TEXT.get(code, f'unknown code {code}')


# ---------------------------------------------------------------------- #
#  LEAK: why we still read NAMED_VALUE_FLOAT and not SYS_STATUS           #
# ---------------------------------------------------------------------- #
# We asked the board to move LEAK onto the SYS_STATUS extended health bits, because
# NAMED_VALUE_FLOAT multiplexes MV_STATE/LEAK/WTEMP/GAIN onto one msgid and pymavlink caches
# exactly one message per msgid -- so whichever arrived last wins and leak REPORTING was
# probabilistic. The board did it (fw rev 3), and it is verified on the wire:
# present_extended = health_extended = 0x02.
#
# WE STILL CANNOT READ IT, and this is the trap:
#
#   pymavlink 2.4.49's SYS_STATUS message has THIRTEEN fields and no extensions.
#   `MAVLink_sys_status_message.fieldnames` ends at `errors_count4`. The board sends 40
#   bytes; pymavlink parses the first 31 against its own schema and DISCARDS the rest.
#   `getattr(msg, 'onboard_control_sensors_health_extended', None)` is None. Always.
#
# Exactly the ESC_STATUS(291) failure wearing a different hat: the data is on the wire and
# our library cannot see it. So the firmware keeps sending NAMED_VALUE_FLOAT("LEAK") as a
# deprecated duplicate, and the real fix on OUR side is to stop reading LEAK out of
# pymavlink's single-slot cache -- see SrotFC._drain_named / telemetry().
#
# Flip this to True only after confirming `'onboard_control_sensors_health_extended' in
# mavutil.mavlink.MAVLink_sys_status_message.fieldnames` on the pymavlink you deploy.
SYS_STATUS_HAS_EXTENDED_HEALTH = False
MAV_SYS_STATUS_SENSOR_LEAK     = 0x02


def sanitize_speed(speed: float) -> float:
    """Clamp a 0..1 move speed to [0, MOVE_CRUISE_MAX] (the board clamps too).

    NaN FAILS SAFE TO ZERO. `_clamp` is written with comparisons, and every
    comparison against NaN is False, so a NaN fell straight through the clamp
    and onto the wire as p3 (B35). Zero is the fail-safe direction here: no
    motion. `inf` needs no special case -- it compares True and clamps.
    """
    speed = float(speed)
    if speed != speed:            # NaN: the one value that is not equal to itself
        return 0.0
    return _clamp(speed, 0.0, MOVE_CRUISE_MAX)


# ---------------------------------------------------------------------- #
#  Payload -- PCA9685 aux expander on the SROT board (config.h SECTION 5) #
# ---------------------------------------------------------------------- #
# The payload is integrated into SROT now (NO separate USB ESP32): a 16-channel
# PCA9685 servo/MOSFET expander on the board's I2C bus, driven over MAVLink.
#   * channels 0..7  = PCA_SERVO  (PWM µs, SERVO_MIN_US..SERVO_MAX_US) via DO_SET_SERVO
#     (DO_SET_SERVO p1 is 1-BASED, so PCA ch c -> p1 = c+1)
#   * channels 8..15 = PCA_SWITCH (MOSFET on/off) via DO_SET_RELAY
#     (DO_SET_RELAY p1 is a 0-based INSTANCE -> PCA ch = PCA_RELAY_BASE_CH + instance)
PCA9685_NUM_CH      = 16
SERVO_MIN_US        = 1000
SERVO_MAX_US        = 2000
PCA_PAYLOAD_SERVO_CH = 0    # config.h default payload-release servo (0-based)
PCA_RELAY_BASE_CH   = 8     # first MOSFET/switch channel (0-based)

# ---------------------------------------------------------------------- #
#  Channel ROLE -- the board owns it, we only read it                     #
# ---------------------------------------------------------------------- #
# Each PCA9685 channel's role is a FIRMWARE parameter, set in Bondor, exposed as
# `SERVO{n}_ROLE` where n = PCA channel + 1 (so SERVO9_ROLE is PCA channel 8).
#
# duburi_ws drives SWITCH channels ONLY. The PWM/servo channels belong to the
# on-board manipulator arm, and firing one from a mission would move the arm during
# a drop. We therefore READ the role and refuse anything that is not a switch --
# rather than keeping a host-side copy of the wiring, which goes stale SILENTLY the
# moment a channel is re-roled on the board.
#
# ⚠ THE 1-8 / 9-16 SPLIT IS A DEFAULT, NOT THE WIRING. It is only the initial value
# of the per-channel param (`(c < 8) ? 1.0f : 2.0f`, fw params.cpp), freely re-rolled
# from Bondor. Nothing in this file or above it may assume it -- read the role. It is
# recorded here only because it is what this hull happened to read on 2026-08-02, and
# because stating it as folklore is how it got believed in the first place.
PCA_ROLE_PARAM_FMT  = 'SERVO{}_ROLE'
PCA_ROLE_DISABLED   = 0
PCA_ROLE_SERVO      = 1     # PWM -- the arm. duburi_ws MUST NOT drive these.
PCA_ROLE_SWITCH     = 2     # MOSFET/relay HIGH-LOW -- the payload. Ours.
PCA_ROLE_NAMES      = {PCA_ROLE_DISABLED: 'DISABLED',
                       PCA_ROLE_SERVO:    'SERVO (PWM, on-board arm)',
                       PCA_ROLE_SWITCH:   'SWITCH (MOSFET/relay)'}
# A role-2 channel has no pulse width: the firmware reads DO_SET_SERVO's µs as a
# LEVEL for it (>= 1500 = ON), which makes every channel addressable by its own
# number whatever its role (fw mav_commands.cpp:471-479).
PCA_SWITCH_ON_US    = 2000
PCA_SWITCH_OFF_US   = 1000

# ---- SERVOn_FUNCTION -- WHAT is wired there, as opposed to HOW it is driven ---- #
#
# ROLE (above) is the AUTHORITY: it decides whether a channel may be actuated at all,
# and the firmware enforces it. FUNCTION is the IDENTITY: it decides what we CALL the
# channel. The two are orthogonal, and conflating them is the bug to avoid -- setting
# a FUNCTION never makes a channel fireable.
#
# The firmware deliberately does not read `servo_func` (grep over its src/ + include/
# returns only the declaration and the param-table row). It is storage, served by the
# ordinary param protocol, so the payload map lives ON THE BOARD and travels with the
# hull -- instead of in a host-side table that goes stale the moment someone re-wires
# a channel, whose failure mode is firing the manipulator arm during a drop.
#
# ⚠ APPEND-ONLY. Canonical numbers are `SROT_SERVO_FUNC_*` in the firmware's
# include/config.h; Bondor mirrors them in src/shared/protocol.ts. Inserting a value
# silently renames every payload after it. test_srot_protocol_drift pins all three.
PCA_FUNC_PARAM_FMT  = 'SERVO{}_FUNCTION'
PCA_FUNC_NONE       = 0     # param default -- an unconfigured board reads "unassigned"
PCA_FUNC_TORPEDO    = 1
PCA_FUNC_DROPPER    = 2
PCA_FUNC_GRIPPER    = 3
PCA_FUNC_LIGHT      = 4
PCA_FUNC_CAMERA     = 5
PCA_FUNC_AUX        = 6
PCA_FUNC_NAMES      = {PCA_FUNC_NONE:    'unassigned',
                       PCA_FUNC_TORPEDO: 'torpedo',
                       PCA_FUNC_DROPPER: 'dropper',
                       PCA_FUNC_GRIPPER: 'gripper',
                       PCA_FUNC_LIGHT:   'light',
                       PCA_FUNC_CAMERA:  'camera',
                       PCA_FUNC_AUX:     'aux'}


# --------------------------------------------------------------------------- #
#  MOTOR_DETECT (mode 20) -- the GATE 0 procedure, run from here              #
# --------------------------------------------------------------------------- #
# The token an operator must hand `SrotFC.motor_detect()` to actually run it.
# A bool default would make an accidental run one keystroke away, and this
# routine PULSES ALL EIGHT THRUSTERS and then rewrites the direction signs the
# attitude controllers depend on.
MOTOR_DETECT_TOKEN = 'RUN MOTOR DETECT IN WATER'

# 8 motors x (500 ms settle + 500 ms thrust), plus however long the gyro takes
# to go quiet before each one. 60 s is generous; the refusal on timeout says
# what it saw rather than assuming failure.
MOTOR_DETECT_TIMEOUT_S = 60.0

# --------------------------------------------------------------------------- #
#  The safety gates: board params that SILENTLY disable behaviour               #
# --------------------------------------------------------------------------- #
# Twenty-one board parameters default to zero, and several of them gate safety
# behaviour rather than a convenience. A disabled gate is invisible: nothing
# errors, nothing logs, the feature simply never happens -- which is the exact
# defect class that has already cost this project a round three times (the
# vision verbs in SURFACE, the disarmed SROT_MOVE, MTUNE_EN).
#
# Each entry: (param, expected, severity, what it gates)
#   `expected` is what the value must be for the feature to be LIVE.
#   Severity 'CRIT' = a safety behaviour is off; 'WARN' = behaviour differs from
#   what a mission author would assume.
#
# ⛔ THIS TABLE IS NOT A SET OF DEFAULTS TO WRITE. It is what to READ and report.
# Enabling a motor-spinning mode or a failsafe threshold is an operator decision
# on a specific hull, and a host that quietly set them would be worse than one
# that never looked.
SAFETY_GATES = (
    # LEAK_EN gates BOTH the leak failsafe and the leak pre-arm refusal
    # (fw arming.cpp:37). Measured 0 on this vehicle's board 2026-09-03: as
    # configured, a leak neither blocks arming nor surfaces the hull.
    ('LEAK_EN',       1.0, 'CRIT', 'leak failsafe AND the leak pre-arm refusal'),
    # ESPNOW_EN gates the whole 2nd-board link: the THRUSTER pack voltage, the
    # kill-switch state, the mixer's voltage linearisation and the low-thruster-
    # battery failsafe all go dark together (fw second_board/main.cpp:29-31).
    ('ESPNOW_EN',     1.0, 'CRIT', '2nd-board link: thruster volts, kill state, '
                                   'mixer compensation, battery failsafe'),
    # ⛔ ARMING_CHECK == 0 makes canArm() return true immediately
    # (fw arming.cpp:13-14) -- EVERY board-side pre-arm check is skipped,
    # including the leak and battery refusals above.
    ('ARMING_CHECK',  1.0, 'CRIT', 'ALL board-side pre-arm checks'),
    ('FS_BAT_ENABLE', 1.0, 'CRIT', 'low-thruster-battery failsafe'),
    ('FS_GCS_ENABLE', 1.0, 'CRIT', 'GCS-loss failsafe (host silence -> surface)'),
    # Not safety, but it changes how far a timed leg travels: without a pack
    # voltage the mixer cannot linearise thrust, so distance is battery-state
    # dependent. Needs ESPNOW_EN too -- the data has to arrive first.
    ('MOT_BAT_V_MAX', None, 'WARN', 'thrust linearisation (a timed leg is '
                                    'pack-state dependent without it)'),
    ('THR_TRIM_EN',   1.0, 'WARN', 'per-motor thrust normalisation from RPM'),
)


def safety_gate_findings(values):
    """[(severity, param, text)] for every gate that is OFF or UNREADABLE.

    Pure: takes {param: float|None} and returns findings, so the whole table is
    testable with no board attached. `None` means the parameter could not be
    read, which is reported as its own finding -- absence is not a pass.
    """
    out = []
    for name, expected, sev, gates in SAFETY_GATES:
        v = values.get(name)
        if v is None:
            out.append((sev, name, f'UNREADABLE -- gates {gates}'))
        elif expected is None:                 # "must be set to something > 0"
            if v <= 0.0:
                out.append((sev, name, f'is 0 -- {gates}'))
        elif v < expected - 1e-6:
            out.append((sev, name, f'is {v:g}, wanted {expected:g} -- '
                                   f'DISABLED: {gates}'))
    return out


# --------------------------------------------------------------------------- #
#  AUTOTUNE (21) and MOTOR_TUNE (22) -- the other two in-water procedures       #
# --------------------------------------------------------------------------- #
# Separate tokens, because these are NOT the same risk as MOTOR_DETECT. Detect
# pulses each thruster for 500 ms; AUTOTUNE runs a RELAY on the attitude loops
# for ~a minute at full authority and then REWRITES EVERY PID, and MOTOR_TUNE
# spins one motor at a time through a throttle ramp. A single shared token would
# let an operator who meant one reach the others.
AUTOTUNE_TOKEN   = 'RUN AUTOTUNE IN WATER'
MOTOR_TUNE_TOKEN = 'RUN MOTOR TUNE IN WATER'

# ~1 minute quoted by the firmware header (rate loops -> angle loops -> depth),
# doubled: the phases are terminated by limit-cycle CONSENSUS, not by a clock,
# so a marginal plant legitimately takes longer. The refusal on timeout reports
# what it saw.
AUTOTUNE_TIMEOUT_S   = 150.0
# 8 motors x (ramp + level holds + relay), and it averages across motors.
MOTOR_TUNE_TIMEOUT_S = 300.0

# ⛔ MOTOR_TUNE IS GATED ON A PARAM THAT DEFAULTS TO ZERO.
# `mt_active = (mode == MOTOR_TUNE) && (g_params.mtune_en > 0.5f) && armed`
# (fw task_control_loop.cpp:509), and `DEF_MTUNE_EN = 0.0f`
# (fw config.h:588). So SET_MODE alone enters the mode and the tuner never
# starts -- the mode sits there doing nothing, which is the silent-no-op shape
# that has already cost us a round (vision verbs in SURFACE).
MOTOR_TUNE_ENABLE_PARAM = 'MTUNE_EN'

# The PIDs AUTOTUNE rewrites, read before and after so the caller is told what
# actually moved rather than "finished".
AUTOTUNE_PID_PARAMS = (
    'RATE_RLL_P', 'RATE_RLL_I', 'RATE_RLL_D',
    'RATE_PIT_P', 'RATE_PIT_I', 'RATE_PIT_D',
    'RATE_YAW_P', 'RATE_YAW_I', 'RATE_YAW_D',
    'ANG_RLL_P',  'ANG_PIT_P',  'ANG_YAW_P',
    'DEPTH_P',    'DEPTH_I',    'DEPTH_D',
)

# What MOTOR_TUNE writes: the throttle->RPM plant fit, averaged across motors.
MOTOR_TUNE_PARAMS = ('RPM_KP', 'RPM_KI', 'FF_A', 'IDLE', 'MOT_SPIN_MIN')

# The per-motor sign MOTOR_DETECT writes. It MULTIPLIES with MOT_n_DIRECTION;
# neither display shows the product, which is exactly how the 2026-08-07
# confusion happened (fw mav_stream.cpp:457).
CAL_MDIR_PARAMS = tuple(f'CAL_MDIR{i}' for i in range(1, 9))
MOT_DIRECTION_PARAMS = tuple(f'MOT_{i}_DIRECTION' for i in range(1, 9))
