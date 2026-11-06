"""The few names every part of the SROT driver shares.

`SrotFC` is assembled from one file per concern (the map is in
srot_fc.py's docstring). The constants and
helpers they all need live here, so no part imports another part.

pymavlink is imported here, once, after selecting MAVLink 2: the dialect is
fixed at import, so every part must get `mavutil` from this module.
"""
from __future__ import annotations

import math
import os
import time

# MUST precede the pymavlink import.
#
# pymavlink binds ONE dialect module at import time, chosen from the environment, and
# the default is `dialects.v10.ardupilotmega` -- MAVLink *1*. ESC_TELEMETRY_1_TO_4
# (11030) and _5_TO_8 (11031) do not exist there, so `decode()` returns
# `UNKNOWN_11030` and `/mongla/esc_rpm` reads nothing. Silently: no error, no
# callback, just zero RPM that looks exactly like an ESC or wiring fault. This is the
# same trap that cost Bondor every RPM packet via ESC_STATUS.
#
# Anything importing `fc.srot_fc` -- the manager, bringup_check, a test, a script --
# gets MAVLink 2 from this line, with no import-order dependency.
#
# MEASURED on the board (COM19, in-vehicle): with the default dialect the board's own
# ESC frames arrive and are reported as UNKNOWN_291 / UNKNOWN_11030 / UNKNOWN_11031.
os.environ['MAVLINK20'] = '1'
from pymavlink import mavutil                      # noqa: E402

_ARMED_FLAG = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED

_POLL_S = 0.05    # cache-poll granularity for ack/arm/mode loops
_LINK_STALE_S = 3.0


def _rx_age(msg) -> float:
    """Seconds since `msg` ARRIVED, on the monotonic clock (issue #18).

    ⛔ AGES WERE WALL MINUS WALL. pymavlink stamps `_timestamp` with
    `time.time()`, and the Pi has no RTC: the first NTP sync at the pool STEPS
    the clock. A step back of 20 s made a value that stopped arriving 20 s ago
    read FRESH for 20 s -- LEAK and KILL last-known-good standing in for a
    silent sensor -- and a step forward made every live value read absent at
    once. The manager's reader thread now stamps `msg._mono` with
    `time.monotonic()` the moment a message is received, and every AGE here is
    measured from that. Wall time survives only in ordering comparisons between
    two pymavlink stamps, where a step moves both sides together.

    A message this reader never handled (a unit-test fake, a direct
    `recv_match` in a tool) has no `_mono`; it falls back to the old wall-clock
    age rather than to "fresh".
    """
    mono = getattr(msg, '_mono', None)
    if mono is not None:
        return time.monotonic() - float(mono)
    return time.time() - getattr(msg, '_timestamp', 0.0)


def _finite(*vals) -> bool:
    """True iff every value is a finite float (goal validation before send)."""
    return all(isinstance(v, (int, float)) and math.isfinite(v) for v in vals)
