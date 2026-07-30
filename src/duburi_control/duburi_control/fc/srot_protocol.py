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
SOURCE_COMPID   = 190
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
# misses a state mutex at dispatch: SROT_MOVE (mav_commands.cpp:287), accel-cal
# (:376) and, since Round 3, DO_SET_SERVO (:422) on the payload path. The command
# never ran, so it is terminal for us: treating it as non-terminal means waiting
# out the whole deadline and reporting a bogus TIMEOUT for a move that was simply
# refused. Retrying is the caller's call, not this layer's.
ACK_TEMPORARILY_REJECTED = 3
TERMINAL_ACKS   = frozenset({ACK_ACCEPTED, ACK_DENIED, ACK_FAILED, ACK_CANCELLED,
                             ACK_TEMPORARILY_REJECTED})

CMD_SROT_MOVE = 31000      # the one custom verb; rides inside a COMMAND_LONG
CMD_DO_SET_SERVO = 183     # PCA9685 servo channel -> µs (payload release servo)
CMD_DO_SET_RELAY = 181     # PCA9685 MOSFET/switch channel on/off (payload solenoid)
CMD_USER_1    = 31010      # yaw spin (stunt)   -- style_yaw has no STYLE equivalent
CMD_USER_2    = 31011      # pitch spin (stunt)
CMD_USER_3    = 31012      # roll spin (stunt)
CMD_USER_4    = 31013      # pattern
CMD_USER_5    = 31014      # autotune start

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

# custom_mode int -> name. DuburiState.mode carries the name; note DEPTH_HOLD is
# the SROT analogue of ArduSub ALT_HOLD but the NAMES differ -- any mission/FSM
# that branches on a mode string must be audited (set_depth engages ALT_HOLD on
# Pixhawk but a DIVE-in-AUTO on SROT).
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


def mode_name(custom_mode) -> str:
    """custom_mode int -> mode name, or 'UNKNOWN(<n>)' for an unmapped value."""
    try:
        return MODE_NAMES.get(int(custom_mode), f'UNKNOWN({int(custom_mode)})')
    except (TypeError, ValueError):
        return 'UNKNOWN'


def mode_int(name) -> int | None:
    """Mode name -> custom_mode int, or None if unknown (fail-loud upstream).

    Accepts the ArduSub aliases in MODE_ALIASES (currently just ALT_HOLD, which
    is the same mode as DEPTH_HOLD) so Pixhawk-era missions run unmodified."""
    key = str(name).strip().upper()
    if key in MODE_INTS:
        return MODE_INTS[key]
    return MODE_ALIASES.get(key)


# ---------------------------------------------------------------------- #
#  MV_STATE -- movement phase (JETSON_COMMS.md §6 NAMED_VALUE_FLOAT)      #
# ---------------------------------------------------------------------- #
MV_IDLE   = 0
MV_CRUISE = 1
MV_BRAKE  = 2
MV_TURN   = 3
MV_DIVE   = 4
MV_STYLE  = 5
MV_DONE   = 6
MV_STATE_NAMES = {
    MV_IDLE: 'idle', MV_CRUISE: 'cruise', MV_BRAKE: 'brake', MV_TURN: 'turn',
    MV_DIVE: 'dive', MV_STYLE: 'style', MV_DONE: 'done',
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
    return lo if v < lo else hi if v > hi else v


def unit_to_mc(u: float) -> int:
    """-1..1 -> -1000..1000 for MANUAL_CONTROL x/y/r (clamped)."""
    return int(round(_clamp(float(u), -1.0, 1.0) * MC_AXIS_MAX))


def unit_to_mc_z(u: float) -> int:
    """-1..1 (up positive) -> 0..1000 with 500 neutral for MANUAL_CONTROL z (clamped).

    +1 (full ascend) -> 1000, 0 (hold) -> 500, -1 (full descend) -> 0. Matches the
    board's convention that z>500 ascends (heave positive = UP)."""
    return int(round(MC_Z_NEUTRAL + _clamp(float(u), -1.0, 1.0) * (MC_Z_MAX - MC_Z_NEUTRAL)))


def pct_to_mc(p: float) -> int:
    """-100..100 % -> -1000..1000 (the old percent API -> MANUAL_CONTROL axis)."""
    return int(round(_clamp(float(p), -100.0, 100.0) * 10))


def pct_to_mc_z(p: float) -> int:
    """-100..100 % (up positive) -> 0..1000, 500 neutral."""
    return int(round(MC_Z_NEUTRAL + _clamp(float(p), -100.0, 100.0) * 5))


# ---------------------------------------------------------------------- #
#  Movement param defaults (PARAMETERS.md / JETSON_COMMS.md §8)           #
# ---------------------------------------------------------------------- #
# For reference + host-side sanity clamps; the board owns the live values.
MOVE_CRUISE_MAX = 0.80    # clamps a move's p3 speed
MOVE_YAW_RATE   = 45.0    # deg/s default turn rate when p3=0
MOVE_DEPTH_RATE = 0.20    # m/s dive/ascend ramp
GAIN_FOR_AUTONOMY = 1.0   # MANUAL_CONTROL is halved until GAIN=1.0 (boots at 0.5)

# ---------------------------------------------------------------------- #
#  Host-side brake (see SrotFC._brake_last_leg)                          #
# ---------------------------------------------------------------------- #
# The wire-reachable MOVE_STOP does NOT apply reverse thrust: movement::start()
# zeroes s_uf/s_ul/s_speed before the STOP case, so PH_BRAKE computes
# `-0 * gain * 0` == 0 (fw movement.cpp:56,58,90,152). The brake *duration* is
# right, the thrust is nil -- so an abort coasts. The firmware's own
# movement::abort() brakes correctly but is not reachable from MAVLink.
#
# So we brake ourselves: a short REVERSE move leg on the same axis. That stays in
# AUTO (depth hold is preserved -- MANUAL_CONTROL cannot brake here at all, since
# AUTO overwrites fwd/lat with the movement demand, fw task_control_loop.cpp:209),
# uses only documented wire verbs, and the reverse leg ends with its own brake.
# Mirrors the firmware's MOVE_BRAKE_GAIN / MOVE_BRAKE_K shape; tune here, not there.
BRAKE_GAIN     = 0.60   # reverse speed as a fraction of the aborted leg's speed
BRAKE_K        = 0.80   # brake seconds per unit of leg speed
BRAKE_MAX_S    = 1.20   # hard cap -- an abort must never become a long new leg
BRAKE_MIN_SPEED = 0.05  # below this the leg carries no momentum worth nulling

# Reverse of each translation move type. Yaw and depth are deliberately absent:
# Ch4 yaw is a RATE the board's own loop bleeds, and depth is an ArduSub-style
# hold -- reversing either fights the controller instead of helping (same rule as
# motion_vision's inertial brake, which also brakes translation only).
BRAKE_REVERSE = {
    MOVE_FORWARD:  MOVE_BACK,
    MOVE_BACK:     MOVE_FORWARD,
    MOVE_STRAFE_L: MOVE_STRAFE_R,
    MOVE_STRAFE_R: MOVE_STRAFE_L,
}


def sanitize_speed(speed: float) -> float:
    """Clamp a 0..1 move speed to [0, MOVE_CRUISE_MAX] (the board clamps too)."""
    return _clamp(float(speed), 0.0, MOVE_CRUISE_MAX)


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
