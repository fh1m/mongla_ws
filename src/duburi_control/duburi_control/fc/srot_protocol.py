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
# fw mav_commands.cpp:687), so 191 feeds the current failsafe exactly as 190 did,
# and it keeps working once FS_GCS_SYSID/FS_GCS_COMPID land.
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
FW_BEHAVIOUR_REV = 4

# The minimum revision this host code assumes. Flashing older firmware than this
# re-opens the coasting MOVE_STOP with no host brake left to cover it.
FW_BEHAVIOUR_REV_REQUIRED = 2

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
