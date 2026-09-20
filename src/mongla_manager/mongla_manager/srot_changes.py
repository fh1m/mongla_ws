#!/usr/bin/env python3
"""Detect what CHANGED between two SROT telemetry snapshots. Pure, no I/O.

The dashboard answers "what is the board doing now"; this answers "what changed while I
was not looking". They are different questions and a scrolling dump answers neither: it
buries a mode flip or a failsafe under two hundred identical lines.

THREE THINGS COUNT AS A CHANGE, and the third is the one that is usually missed:

  1. a discrete value flipping           (mode, armed, leak, kill)
  2. a continuous value moving far enough to matter   (per-field thresholds below)
  3. a field going ABSENT <-> PRESENT

(3) matters because the board deliberately SUPPRESSES values it cannot stand behind
(firmware behaviour rev 3+). A barometer that stops being reported is the board telling
you something, and a change log that only watches numbers move will never mention it --
the value simply stops updating, which looks identical to a value that is holding steady.

Thresholds are deliberately coarse. A log that fires on sensor noise is a log nobody
reads, and the dashboard is already showing the live number to whoever wants precision.
"""
from __future__ import annotations

import math

# field -> minimum movement worth a line. Chosen from the measured noise floor of each
# signal on a still bench, not from what looks tidy.
_THRESHOLDS = {
    'heading_deg':      2.0,     # BNO yaw is stable to well under a degree when still
    'roll_deg':         2.0,
    'pitch_deg':        2.0,
    'depth_m':          0.10,    # 10 cm -- below this is barometer noise, not motion
    'battery_v':        0.20,    # LiPo sag under load is ~1 V; 0.2 tracks trend not ripple
    'thruster_v':       0.20,
    'water_temp_c':     1.0,
    'depth_out':        0.10,    # controller output; 0.1 of full scale is meaningful
    'depth_err_m':      0.25,
    'gain':             0.05,    # discrete 0.1 steps on the board
    'mag_accuracy':     0.5,     # integer 0..3
}

# Fields whose change is worth shouting about regardless of magnitude.
_CRITICAL = frozenset({'armed', 'leak', 'kill', 'mode', 'baro_healthy'})


def _absent(v) -> bool:
    return v is None or (isinstance(v, float) and math.isnan(v))


def _fmt(v) -> str:
    if _absent(v):
        return '--'
    if isinstance(v, bool):
        return 'yes' if v else 'no'
    if isinstance(v, float):
        return f'{v:.2f}'
    return str(v)


def diff(prev: dict | None, cur: dict) -> list[tuple[str, str, str]]:
    """(level, field, message) for everything that changed. Empty on the first call.

    The first snapshot establishes a baseline and reports nothing -- otherwise every
    field "changes" at startup and the first screen is pure noise.
    """
    if prev is None:
        return []

    out: list[tuple[str, str, str]] = []
    for field, new in cur.items():
        old = prev.get(field, None)
        was, now = _absent(old), _absent(new)

        # (3) appearance / disappearance. Checked FIRST: comparing magnitudes when one
        # side is absent is meaningless, and the transition is the interesting event.
        if was != now:
            if now:
                lvl = 'WARN' if field in _CRITICAL or field in _THRESHOLDS else 'INFO'
                out.append((lvl, field,
                            f'{field} STOPPED being reported (was {_fmt(old)}) -- the '
                            f'board suppresses values it cannot stand behind'))
            else:
                out.append(('INFO', field,
                            f'{field} started being reported: {_fmt(new)}'))
            continue
        if was and now:
            continue                       # absent before and after: nothing happened

        # (1) discrete flips.
        if field in _CRITICAL:
            if old != new:
                lvl = 'CRIT' if field in ('leak', 'armed') else 'WARN'
                out.append((lvl, field, f'{field}: {_fmt(old)} -> {_fmt(new)}'))
            continue

        # (2) continuous movement past the field's own threshold.
        thresh = _THRESHOLDS.get(field)
        if thresh is None:
            if old != new:
                out.append(('INFO', field, f'{field}: {_fmt(old)} -> {_fmt(new)}'))
            continue
        try:
            moved = abs(float(new) - float(old))
        except (TypeError, ValueError):
            continue
        # Heading wraps through 0/360, so a 359 -> 1 turn is 2 degrees of motion, not
        # 358. Taking the shortest arc HERE rather than in a second pass keeps one
        # decision point: whether the entry exists and what it says are the same test.
        suffix = ''
        if field == 'heading_deg':
            moved = min(moved, 360.0 - moved)
            suffix = ' shortest arc'
        if moved >= thresh:
            out.append(('INFO', field,
                        f'{field}: {_fmt(old)} -> {_fmt(new)}  (Δ{moved:.2f}{suffix})'))

    return out
