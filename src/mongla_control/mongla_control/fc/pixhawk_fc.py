#!/usr/bin/env python3
"""PixhawkFC -- the existing Pixhawk/ArduSub backend behind the FlightController HAL.

`PixhawkFC` **is-a** `Pixhawk` (subclass), so it keeps the full historical method
surface (`send_rc_override`, `set_target_depth`, `get_attitude`, ...) that `Mongla`,
the `motion_*` loops, `Heartbeat` and `HeadingLock` still call directly. That makes
`flight_controller=pixhawk` byte-identical to today -- the migration checkpoint
"existing missions run unchanged" is met by construction.

On top of that inherited surface it adds the intent-level ABC methods
(`telemetry`, `manual`, `send_gcs_heartbeat`) so both backends satisfy the same
`FlightController` contract. `move()` is intentionally left to the existing facade
+ `motion_*` path on this backend (the host loops ARE the Pixhawk "move"); only the
SROT backend needs `move()` to send a single on-board primitive.
"""

from __future__ import annotations

import math

from ..pixhawk import Pixhawk
from .base import FlightController, Telemetry


class PixhawkFC(Pixhawk, FlightController):
    """ArduSub backend: the full Pixhawk surface + the intent-level ABC additions."""

    name = 'pixhawk'

    def telemetry(self) -> Telemetry:
        """MonglaState snapshot from the ArduSub cache reads."""
        t = Telemetry()
        att = self.get_attitude()               # AHRS2-backed {'yaw','roll','pitch','depth'}
        if att is not None:
            t.yaw_deg = float(att.get('yaw', math.nan))
            t.roll_deg = float(att.get('roll', math.nan))
            t.pitch_deg = float(att.get('pitch', math.nan))
            t.depth_m = float(att.get('depth', math.nan))
        t.armed = bool(self.is_armed())
        t.mode = self.get_mode()
        batt = self.get_battery()            # {'voltage','current'} or None
        t.battery_voltage = float(batt['voltage']) if batt else math.nan
        t.link_alive = bool(self.link_alive())
        # rpm stays empty -- ArduSub over this stack never exposed per-thruster RPM.
        return t

    def manual(self, fwd: float, lat: float, up: float, yaw: float) -> None:
        """Map the -1..1 intent axes onto an RC override (the ArduSub actuation).

        up>0 = ascend -> throttle >1500 (ArduSub Ch3 up is >1500). Reuses the
        proven percent_to_pwm scaling; the vision/DVL loops keep driving the
        richer send_rc_* directly on this backend, so this is only exercised when
        a caller uses the intent API uniformly across backends.
        """
        pwm = Pixhawk.percent_to_pwm
        self.send_rc_override(
            throttle=pwm(up * 100.0),
            forward=pwm(fwd * 100.0),
            lateral=pwm(lat * 100.0),
            yaw=pwm(yaw * 100.0))

    def send_gcs_heartbeat(self) -> None:
        """No-op: the manager already owns the ArduSub heartbeat timer on this path."""
        pass
