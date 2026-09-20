#!/usr/bin/env python3
"""FlightController -- the hardware-abstraction boundary between the ROS control
stack and whatever autopilot flies the vehicle (Pixhawk/ArduSub or the SROT board).

Why this exists
---------------
Historically `mongla_control` talked to one class, `Pixhawk`, constructed in
`auv_manager_node` and injected into `Mongla`, the `motion_*` functions,
`Heartbeat`, `HeadingLock` and `FeedbackPump`. Migrating to the SROT board is a
transport-and-verbs swap, so we put an interface at that same injection seam:

    make_flight_controller('pixhawk'|'srot', master, log) -> FlightController

The ABC exposes **intent**, not channels -- that is the whole point of moving to a
board that owns the primitives. `manual()` is the low-level servo primitive the
vision + DVL loops stream; `move()` is the one-shot high-level verb the board runs
on-board (SROT) or the host motion loops run (Pixhawk).

This is a duck-typed base (like `mongla_sensors.sources.base.YawSource`), not
`abc.ABC`: subclasses that forget a method fail loudly with NotImplementedError,
and the fakes in tests only need the narrow surface they exercise.

Nothing here imports pymavlink; the backends do.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional


# ---- move() terminal codes ------------------------------------------- #
# The SROT board resolves every move to exactly one of four MAVLink results;
# these map onto them (plus host-side TIMEOUT/ABORTED for the Pixhawk loops).
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
    """Outcome of a `FlightController.move()`.

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
    """A backend-agnostic vehicle snapshot -> `/mongla/state` (`MonglaState`).

    Missing numerics are NaN and missing strings empty, matching MonglaState's
    "no data" convention. ``rpm`` is per-thruster (SROT `ESC_STATUS`; empty on
    Pixhawk, which never had it). ``leak``/``water_temp_c`` come from SROT
    `NAMED_VALUE_FLOAT` and are inert on Pixhawk.
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
    # SROT only. `battery_voltage` above stays the MAIN (PM1) pack so MonglaState is
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


class FlightController:
    """Abstract autopilot backend. See module docstring for the seam it sits at."""

    name: str = 'base'

    # -- lifecycle / mode ------------------------------------------------ #
    def arm(self, timeout: float = 15.0, abort=None):
        """(ok, reason). ``abort`` is a ()->bool cancel hook honoured mid-arm."""
        raise NotImplementedError

    def disarm(self, timeout: float = 15.0):
        """(ok, reason)."""
        raise NotImplementedError

    def set_mode(self, mode: str, timeout: float = 8.0):
        """(ok, reason). ``mode`` is a backend mode name (e.g. 'DEPTH_HOLD')."""
        raise NotImplementedError

    # -- actuation ------------------------------------------------------- #
    def manual(self, fwd: float, lat: float, up: float, yaw: float) -> None:
        """Low-level servo primitive. Each axis -1..1 (up positive = ascend).

        Streamed by the vision + DVL loops. On SROT this is one MANUAL_CONTROL
        frame (all four axes always sent -- there is no per-channel release); on
        Pixhawk it maps to an RC override.
        """
        raise NotImplementedError

    def move(self, verb: str, *, on_progress=None, abort_fn=None, **kw) -> MoveResult:
        """Run one high-level verb to a terminal result (BLOCKING).

        ``on_progress(frac)`` is called ~3 Hz with 0..1 progress (wired to the
        action FeedbackPump). ``abort_fn()`` -> bool is polled to cancel (maps to
        a board stop/brake on SROT). Returns a :class:`MoveResult` -- never raises
        on a miss, so the action server always resolves.
        """
        raise NotImplementedError

    # -- telemetry / liveness ------------------------------------------- #
    def telemetry(self) -> Telemetry:
        raise NotImplementedError

    def send_gcs_heartbeat(self) -> None:
        """Emit the >=1 Hz companion HEARTBEAT. On SROT this is mandatory (5 s of
        silence surfaces the vehicle); on Pixhawk it is a no-op by default (the
        manager already owns its heartbeat timer)."""
        pass

    def close(self) -> None:
        pass

    def __repr__(self) -> str:
        return f'<{self.__class__.__name__} name={self.name}>'
