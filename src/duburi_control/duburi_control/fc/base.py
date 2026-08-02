#!/usr/bin/env python3
"""FlightController -- the hardware-abstraction boundary between the ROS control
stack and whatever autopilot flies the vehicle (Pixhawk/ArduSub or the SROT board).

Why this exists
---------------
Historically `duburi_control` talked to one class, `Pixhawk`, constructed in
`auv_manager_node` and injected into `Duburi`, the `motion_*` functions,
`Heartbeat`, `HeadingLock` and `FeedbackPump`. Migrating to the SROT board is a
transport-and-verbs swap, so we put an interface at that same injection seam:

    make_flight_controller('pixhawk'|'srot', master, log) -> FlightController

The ABC exposes **intent**, not channels -- that is the whole point of moving to a
board that owns the primitives. `manual()` is the low-level servo primitive the
vision + DVL loops stream; `move()` is the one-shot high-level verb the board runs
on-board (SROT) or the host motion loops run (Pixhawk).

This is a duck-typed base (like `duburi_sensors.sources.base.YawSource`), not
`abc.ABC`: subclasses that forget a method fail loudly with NotImplementedError,
and the fakes in tests only need the narrow surface they exercise.

Nothing here imports pymavlink; the backends do.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field


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


@dataclass
class Telemetry:
    """A backend-agnostic vehicle snapshot -> `/duburi/state` (`DuburiState`).

    Missing numerics are NaN and missing strings empty, matching DuburiState's
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
    # SROT only. `battery_voltage` above stays the MAIN (PM1) pack so DuburiState is
    # unchanged; the thruster pack is a second, physically separate battery that
    # reaches the board over ESP-NOW and is NaN whenever that link is down.
    thruster_voltage: float = math.nan
    # Depth-controller internals (NAMED_VALUE_FLOAT). `depth_out` saturated while
    # disarmed is the pre-arm tell that arming would command full vertical thrust.
    depth_err_m:      float = math.nan
    depth_out:        float = math.nan
    esc_temp_c:       tuple = field(default_factory=tuple)
    mag_accuracy:     float = math.nan
    kill_switch:      bool  = False


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
