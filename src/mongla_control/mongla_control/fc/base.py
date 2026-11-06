#!/usr/bin/env python3
"""The result types every board call returns, and the vehicle snapshot.

`MoveResult` for a board verb, `FireResult` for a payload shot, `Telemetry`
for one read of the vehicle. Kept apart from `srot_fc` because nothing here
imports pymavlink: the planner and tools read these codes without a link.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional


# ---- move() terminal codes ------------------------------------------- #
# The SROT board resolves every move to exactly one of four MAVLink results;
# these map onto them, plus the host-side TIMEOUT and ABORTED.
SUCCEEDED = 0   # SROT ACCEPTED   -> Move.Result.success = True
PREEMPTED = 1   # SROT CANCELLED  -> a newer move displaced this one (abort, expected)
FAILED    = 2   # SROT FAILED     -> could not start (e.g. no depth sensor); abort + STATUSTEXT
DENIED    = 3   # SROT DENIED     -> rejected at dispatch (bad type / non-finite param)
TIMEOUT   = 4   # host/client deadline elapsed with no terminal ACK
ABORTED   = 5   # cooperative host abort (goal cancelled)

_CODE_NAME = {SUCCEEDED: 'SUCCEEDED', PREEMPTED: 'PREEMPTED', FAILED: 'FAILED',
              DENIED: 'DENIED', TIMEOUT: 'TIMEOUT', ABORTED: 'ABORTED'}


@dataclass
class MoveResult:
    """Outcome of `SrotFC.move()`.

    ``ok`` is True only on SUCCEEDED. PREEMPTED and FAILED are non-fatal aborts
    (the mission continues); DENIED is a programming/validation error worth
    surfacing loudly. ``reason`` carries the board STATUSTEXT when present.
    """
    code:   int
    reason: str = ''

    @property
    def ok(self) -> bool:
        return self.code == SUCCEEDED

    @property
    def code_name(self) -> str:
        return _CODE_NAME.get(self.code, str(self.code))


# ---- fire() outcomes -------------------------------------------------- #
# A payload fire has more interesting failure modes than a bool can carry, and the
# one that matters most is not "it didn't work" but WHICH kind of no: a channel the
# board has configured as the on-board ARM must read differently from a dead link,
# because the first is a mission-authoring mistake and the second is a comms fault.
FIRE_FIRED       = 0   # board accepted the activation -- see FireResult docstring
FIRE_REJECTED_ARM = 1  # channel role is PWM/servo: it drives the arm, not a payload
FIRE_DISABLED    = 2   # channel role is 0 -- driving it would be a silent no-op
FIRE_DENIED      = 3   # board answered DENIED/FAILED (channel out of range, non-finite)
FIRE_NO_ACK      = 4   # no COMMAND_ACK inside the budget -- outcome UNKNOWN
FIRE_NOT_READY   = 5   # link down, no payload driver, or the role could not be read
FIRE_BUSY        = 6   # another fire is mid-pulse, or the board's state lock was busy
FIRE_THRUSTER_FAULT = 7  # a thruster is KNOWN bad -- the hull cannot hold the shot

_FIRE_CODE_NAME = {FIRE_FIRED: 'FIRED', FIRE_REJECTED_ARM: 'REJECTED_ARM_CHANNEL',
                   FIRE_DISABLED: 'DISABLED_CHANNEL', FIRE_DENIED: 'DENIED',
                   FIRE_NO_ACK: 'NO_ACK', FIRE_NOT_READY: 'NOT_READY',
                   FIRE_BUSY: 'BUSY',
                   FIRE_THRUSTER_FAULT: 'THRUSTER_FAULT'}


@dataclass
class FireResult:
    """Outcome of a payload `fire(channel)`.

    ``channel`` is the BOARD channel that was addressed (MAVLink `DO_SET_SERVO`
    param1, 1-based) -- not an index into any host-side table. There is no table.

    ⚠ ``FIRED`` MEANS THE BOARD ACCEPTED THE COMMAND, NOT THAT A SOLENOID MOVED.
    The firmware streams no actuator readback at all (no `SERVO_OUTPUT_RAW`, no
    `ACTUATOR_OUTPUT_STATUS`; `g_state.aux` is never telemetered) and its PCA9685
    driver sets its health flag unconditionally with no I2C probe, so a physically
    disconnected expander ACKs exactly like a working one. This is the strongest
    statement the wire supports; anything more confident would be a lie a mission
    could branch on.

    ``ok`` is True only on FIRED, so the pre-existing ``if payload.fire(...)``
    call sites keep their meaning.
    """
    code:    int
    channel: int = 0
    reason:  str = ''

    @property
    def ok(self) -> bool:
        return self.code == FIRE_FIRED

    @property
    def code_name(self) -> str:
        return _FIRE_CODE_NAME.get(self.code, str(self.code))

    def __bool__(self) -> bool:
        return self.ok


@dataclass
class Telemetry:
    """One read of the vehicle (`SrotFC.telemetry()`).

    Missing numerics are NaN and missing strings empty, matching MonglaState's
    "no data" convention. ``rpm`` is per-thruster (ESC_TELEMETRY);
    ``leak``/``water_temp_c`` come from `NAMED_VALUE_FLOAT`.
    """
    armed:           bool  = False
    mode:            str   = ''
    yaw_deg:         float = math.nan
    depth_m:         float = math.nan
    battery_voltage: float = math.nan
    roll_deg:        float = math.nan
    pitch_deg:       float = math.nan
    rpm:             tuple = field(default_factory=tuple)
    leak:            bool  = False
    water_temp_c:    float = math.nan
    link_alive:      bool  = False
    # `battery_voltage` above stays the MAIN (PM1) pack so MonglaState is
    # unchanged; the thruster pack is a second, physically separate battery that
    # reaches the board over ESP-NOW and is NaN whenever that link is down.
    thruster_voltage: float = math.nan
    # Depth-controller internals (NAMED_VALUE_FLOAT). `depth_out` saturated while
    # disarmed is the pre-arm tell that arming would command full vertical thrust.
    depth_err_m:      float = math.nan
    depth_out:        float = math.nan
    esc_temp_c:       tuple = field(default_factory=tuple)
    mag_accuracy:     float = math.nan
    # WHY the barometer is (un)healthy, not merely that depth vanished. The board
    # publishes this UNGATED on purpose -- "its whole job is to explain a withdrawal,
    # so gating it on the health it reports would hide it exactly when it matters"
    # (fw mav_stream.cpp:876). Enum, from bar30.h:33 -- and note 3 is the state a bare
    # board sits in, which we once read as a health SCORE:
    #   0 healthy | 1 jitter | 2 read failures | 3 not initialised
    # NaN = the board has not said, which is distinct from 0 = healthy.
    baro_health:      float = math.nan
    # ⛔ TRI-STATE. Has the board heard the companion its GCS failsafe is SCOPED TO
    # (FS_GCS_SYSID/FS_GCS_COMPID, default 255/191 = us) since boot?
    #   None  = the board has not said (older firmware, or no telemetry yet)
    #   False = configured companion NEVER seen -> our heartbeat is not matching, so
    #           the GCS failsafe is not being satisfied by us
    #   True  = seen
    # False while we are connected and heartbeating is a real pre-dive finding: it
    # means the failsafe scoping is wrong, and the vehicle's protection against a dead
    # companion is not actually watching the companion.
    companion_seen:   Optional[bool] = None
    # ⛔ TRI-STATE, AND THE THIRD STATE IS THE POINT. None = the board cannot
    # know. The thruster kill switch is a rotary knob on the SECOND board and
    # its state reaches the control board only over ESP-NOW; on link loss the
    # firmware reports kill=false, justified in its own comment as
    # "link lost -> don't assert kill (display-only)". That is correct for a
    # display and WRONG for anything that gates: `KILL = 0` on the wire means
    # "power is live" OR "nobody is telling us", and a bool cannot hold the
    # difference. `srot_fc` only asserts True/False when the 2nd-board link is
    # proven alive by BATTERY_STATUS instance 1 being present.
    kill_switch:      Optional[bool] = None
