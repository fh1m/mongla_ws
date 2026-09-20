#!/usr/bin/env python3
"""One place that decides how a SROT value is rendered. Pure, no I/O, no pymavlink.

WHY THIS EXISTS. The same telemetry reaches an operator through four different paths --
`connect`'s snapshot, `connect --watch`'s dashboard, `--json`, and the manager's periodic
`[SROT ]` log block. When each formats independently they drift, and the drift is not
cosmetic: a field that reads `--` in one place and `0.00` in another is the exact bug this
module was written to end. Formatting lives here; the four callers only lay it out.

TWO RULES, both learned from the hardware.

1. **Match the board.** The SROT OLED and `VFR_HUD.heading` show a heading in **0..360**
   (`mav_stream.cpp:254-256` wraps it explicitly). `SrotFC.get_attitude` already does the
   same for `/mongla/state`. A tool that renders the raw signed `ATTITUDE.yaw` instead shows
   `-162.23` beside the board's `197` -- the same angle, twice, disagreeing. Roll and pitch
   stay SIGNED, because they genuinely are (`CAL_LVL_R/P` are signed radians and a +3 deg
   list is not a 357 deg list).

2. **Absence is not zero.** Since firmware behaviour rev 3 the board SUPPRESSES values it
   cannot stand behind rather than publishing them, and sends `0` in
   `SCALED_IMU2.temperature` as MAVLink's "not provided" sentinel. Rendering a missing value
   as `0.0` re-creates precisely the failure that suppression was added to fix -- a Bar30
   read during a PROM reset race once published `-51 C` and `+2.87 m` in air with nothing
   marking them wrong. Everything absent renders `--`.

   Rule 2 has teeth on a bare bench: with no Pico, no ESCs and no 2nd board, `0 rpm` and
   `0.00 V` are indistinguishable from "attached and idle" unless we say `--`.
"""
from __future__ import annotations

import math

# ANSI. Kept here so the dashboard and the snapshot cannot pick different colours for the
# same severity.
BOLD, DIM, RESET = '\033[1m', '\033[2m', '\033[0m'
RED, YEL, GRN, CYA = '\033[31m', '\033[33m', '\033[32m', '\033[36m'
ABSENT = '--'

# MAVLink's "field not provided" sentinels, by field. These are REAL VALUES in their own
# encoding, so a consumer that does not special-case them reports a flat battery or a
# freezing hull. UINT16_MAX is the documented no-data value for SYS_STATUS.voltage_battery
# and BATTERY_STATUS.voltages[]; -1 is the same for current_battery.
U16_NODATA = 0xFFFF
I16_NODATA = -1

LEVEL_COLOUR = {'OK': GRN, 'WARN': YEL, 'FAIL': RED}


def fmt(value, spec='{:.2f}', suffix='') -> str:
    """Render a numeric, or `--` when it is absent. Never renders absence as zero."""
    if value is None:
        return ABSENT
    if isinstance(value, float) and math.isnan(value):
        return ABSENT
    return spec.format(value) + suffix


def heading_deg(yaw_rad) -> float | None:
    """ATTITUDE.yaw (radians, +/-pi) -> compass heading in 0..360, matching the board.

    This is THE conversion the whole display bug came down to. The board wraps to 0..360 for
    both its OLED and `VFR_HUD.heading`; so does `SrotFC.get_attitude`. Anything that prints
    the raw signed value disagrees with the vehicle by 360 degrees exactly when the heading
    is negative -- which reads as a broken sensor, not a formatting choice.
    """
    if yaw_rad is None or (isinstance(yaw_rad, float) and math.isnan(yaw_rad)):
        return None
    return math.degrees(yaw_rad) % 360.0


def signed_deg(rad) -> float | None:
    """Radians -> signed degrees. For roll and pitch, which are genuinely +/-."""
    if rad is None or (isinstance(rad, float) and math.isnan(rad)):
        return None
    return math.degrees(rad)


def mv_to_volts(millivolts) -> float | None:
    """Millivolts -> volts, honouring the no-data sentinels. `None` when absent."""
    if millivolts is None:
        return None
    raw = int(millivolts)
    if raw in (0, U16_NODATA):
        return None
    return raw / 1000.0


def centiamps_to_amps(centiamps) -> float | None:
    """BATTERY_STATUS.current_battery (cA, -1 = not provided) -> amps, or None."""
    if centiamps is None or int(centiamps) == I16_NODATA:
        return None
    return int(centiamps) / 100.0


def imu_temp_c(raw) -> float | None:
    """SCALED_IMU2.temperature (centi-degC) -> degC.

    `0` is MAVLink's "not provided" sentinel here, NOT a reading of 0 C -- and the board
    sends exactly that when the barometer is unhealthy. Treating it as a temperature puts a
    freezing hull on the screen of a vehicle sitting in a warm room.
    """
    if raw is None or int(raw) == 0:
        return None
    return int(raw) / 100.0


def rpm_row(values) -> str:
    """Per-thruster RPM as a fixed-width row, or `--` when no ESC telemetry arrived.

    An all-zero row is NOT the same as an absent one: zero is a legitimate RPM for an
    attached, stopped ESC, and on a bare bench there is no Pico and no ESC at all. Printing
    `0 0 0 0 0 0 0 0` for "nothing is connected" invents eight thrusters.
    """
    if not values:
        return ABSENT
    return ' '.join(f'{int(v):>5d}' for v in values)


def esc_temp_row(values) -> str:
    """Per-ESC temperature row. Same absent-vs-zero rule as `rpm_row`.

    An ESC reporting 0 C is implausible, so an all-zero row means the telemetry frame
    arrived with nothing behind it -- render it absent rather than as ice-cold ESCs.
    """
    if not values or not any(int(v) for v in values):
        return ABSENT
    return ' '.join(f'{int(v):>5d}' for v in values)


def bool_word(flag, true_word: str, false_word: str, unknown=ABSENT) -> str:
    """Tri-state boolean: True / False / never-reported. `None` must not read as False.

    LEAK is the case that matters: "no leak reported" and "we never heard about the leak
    sensor" are different facts, and only one of them means the hull is dry.
    """
    if flag is None:
        return unknown
    return true_word if flag else false_word


def fw_version(packed) -> str:
    """AUTOPILOT_VERSION.flight_sw_version (packed MAJ.MIN.PAT.<type>) -> 'v0.2.0'."""
    if not packed:
        return ABSENT
    return f'v{(packed >> 24) & 0xFF}.{(packed >> 16) & 0xFF}.{(packed >> 8) & 0xFF}'


def colour(level: str, text: str) -> str:
    return f'{LEVEL_COLOUR.get(level, "")}{text}{RESET}'
