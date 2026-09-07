#!/usr/bin/env python3
"""Prove the depth loop's SIGN on the bench, disarmed, with nothing spinning.

⛔ WHY THIS EXISTS. `depth_control.cpp` carries this, verbatim:

    "AUDIT R1 has stood since the beginning: this loop has NEVER run closed,
     because the Bar30 was not fitted during development."

The sign WAS inverted -- the old code ran the PID in depth while its output lives
in altitude, so it "commanded the opposite of what it wanted, on every axis of
the depth loop". It is fixed, the fix is well argued and cross-checked against
three independent sources, and it is UNVALIDATED.

**The SURFACE failsafe depends on this sign.** A leak, a low thruster battery or
a GCS loss all route to SURFACE, which calls `depth::update()`. If the sign were
still wrong, the emergency ascent drives the vehicle DOWN.

The firmware team built the tool to check it and we had never used it.
`depth::preview()` runs the same error expression through a SEPARATE,
PROPORTIONAL-ONLY instance and publishes it as NAMED_VALUE_FLOAT `DEPTH_CMD`:
computed on demand, so it is live while disarmed, and P-only so a constant error
cannot wind an integrator to the rail (their first version ran a full PID and
reported "inverted" for a loop that is correct).

    measured DEEPER than target   -> DEPTH_CMD POSITIVE -> ascend   (correct)
    measured SHALLOWER than target -> DEPTH_CMD NEGATIVE -> descend (correct)
    demand moves AWAY from the target -> STILL INVERTED. DO NOT DIVE.

⚠ In AIR you cannot do this by lifting the vehicle: a whole metre of altitude is
~0.12 mbar, about 1.2 mm of equivalent depth. That is why the documented bench
check ("enter DEPTH_HOLD and hand-move the vehicle") could never actually be
performed, and a large part of why it never was. PRESSURISE THE PORT INSTEAD --
a thumb over it is plenty.

USAGE. One phase per invocation, operator prompted, because an unattended
protocol measures an untouched rig.

    # the manager owns the serial port; stop it first
    python3 tools/depth_sign_check.py /dev/ttyUSB0

This tool is READ-ONLY: it sends nothing but parameter/telemetry requests.
"""
import sys
import time

try:
    from pymavlink import mavutil
except ImportError:                                        # pragma: no cover
    sys.exit('pymavlink not available -- source the ROS workspace first')

SETTLE_S = 3.0
# The board's preview target, baked into DEPTH_CMD (depth::preview(depth, 0.10)).
PREVIEW_TARGET_M = 0.10
# A press must move DEPTH_CMD by at least this much to count as a real stimulus.
# Below it we cannot tell a press from baro noise, and reporting a verdict from
# noise is worse than reporting that the operator did not press hard enough.
MIN_DELTA = 0.05
# DEPTH_CMD is clamped to +/-1. A baseline already at the clamp has NO ROOM to
# move in the ascend direction, so a press cannot produce a passing delta and a
# perfectly correct board would read INCONCLUSIVE -- or, on a noise sample,
# FAIL. That is not hypothetical: the Bar30 is uncalibrated above water and read
# +1.21 m on the bench (DEPTH_CMD +0.555, still 0.445 of headroom). A larger
# offset would sit at the rail.
#
# ⚠ The zero offset itself is HARMLESS here -- this test measures a DELTA, so a
# constant offset cancels and calibration is irrelevant to the sign. Only
# proximity to the clamp matters.
SATURATION_HEADROOM = 0.10


def sign_verdict(baseline, pressed):
    """('PASS'|'FAIL'|'INCONCLUSIVE', text) for a baseline/pressed DEPTH_CMD pair.

    Pure, so the decision this tool exists to make is testable with no board and
    no thumb. Pressurising the port raises the pressure, so the board reads
    DEEPER; deeper than target must command ASCEND, which is POSITIVE.

    ⚠ An absent reading is never a pass. The board SUPPRESSES DEPTH_CMD when the
    barometer is unhealthy, so `None` is a finding, not a missing sample.
    """
    if baseline is None or pressed is None:
        return ('INCONCLUSIVE',
                'DEPTH_CMD absent -- the board withdrew the barometer. Check BARO_HEALTH; '
                'absence is a finding, not a pass.')
    # A baseline at the clamp cannot move in the ascend direction. Report that as
    # its own state rather than letting it masquerade as a weak or failing sign.
    if baseline > 1.0 - SATURATION_HEADROOM:
        return ('INCONCLUSIVE',
                f'baseline DEPTH_CMD is {baseline:+.3f}, within {SATURATION_HEADROOM:.2f} of '
                'the +1.0 clamp -- a press has nowhere to go, so this test cannot pass '
                'even on a correct board. Re-zero the barometer first '
                '(`ros2 run duburi_planner duburi calibrate_depth`, disarmed) and re-run. '
                'NOTE the zero offset is otherwise harmless here: this test measures a '
                'DELTA, so a constant offset cancels.')
    delta = pressed - baseline
    if abs(delta) < MIN_DELTA:
        return ('INCONCLUSIVE',
                f'DEPTH_CMD moved {delta:+.3f}, below the {MIN_DELTA:.2f} stimulus floor. '
                'Press harder or seal the port; a stimulus too small to measure cannot '
                'validate a sign.')
    if delta > 0:
        return ('PASS',
                f'pressing the port (deeper) drove DEPTH_CMD {delta:+.3f} = POSITIVE = ASCEND. '
                'The loop pushes back toward its target; SURFACE will ascend.')
    return ('FAIL',
            f'pressing the port (deeper) drove DEPTH_CMD {delta:+.3f} = NEGATIVE = DESCEND. '
            'The loop commands motion AWAY from its target, so the SURFACE failsafe would '
            'drive the vehicle DOWN. DO NOT DIVE.')


def _sample(conn, seconds=SETTLE_S):
    """Median DEPTH_CMD over `seconds`. None if the board never sent it.

    Median, not mean: the Bar30's failure mode is occasional wild samples (the
    2026-08-03 jitter gate exists for 317-874 mbar excursions on a stationary
    bench), and one of those in a short window drags a mean anywhere.
    """
    vals = []
    end = time.time() + seconds
    while time.time() < end:
        m = conn.recv_match(type='NAMED_VALUE_FLOAT', blocking=True, timeout=0.5)
        if m is None:
            continue
        name = m.name.decode() if isinstance(m.name, bytes) else str(m.name)
        if name.strip('\x00') == 'DEPTH_CMD':
            vals.append(float(m.value))
    if not vals:
        return None, 0
    vals.sort()
    return vals[len(vals) // 2], len(vals)


def main(dev='/dev/ttyUSB0', baud=115200):
    conn = mavutil.mavlink_connection(dev, baud=baud)
    print('waiting for heartbeat...', flush=True)
    if conn.wait_heartbeat(timeout=15) is None:
        sys.exit('no heartbeat -- is the manager still holding the port?')
    print(f'connected: sys={conn.target_system} comp={conn.target_component}\n')

    print('PHASE 1 -- baseline. Leave the Bar30 ALONE. Sampling %.0f s...' % SETTLE_S)
    base, n1 = _sample(conn)
    if base is None:
        sys.exit('no DEPTH_CMD in %.0f s. The board suppresses it when the barometer is\n'
                 'unhealthy or stale, so this is a REAL finding: check BARO_HEALTH.' % SETTLE_S)
    print(f'  baseline DEPTH_CMD = {base:+.3f}   (n={n1})\n')

    input('PHASE 2 -- press and HOLD a thumb firmly over the Bar30 port,\n'
          '           then press ENTER while still holding it: ')
    press, n2 = _sample(conn)
    if press is None:
        sys.exit('DEPTH_CMD vanished during the press -- the board withdrew the barometer.')
    print(f'  pressed  DEPTH_CMD = {press:+.3f}   (n={n2})')

    delta = press - base
    print(f'  delta              = {delta:+.3f}\n')

    verdict, text = sign_verdict(base, press)
    banner = {'PASS': 'PASS', 'FAIL': '*** FAIL -- SIGN IS INVERTED. DO NOT DIVE. ***',
              'INCONCLUSIVE': 'INCONCLUSIVE'}[verdict]
    print(banner)
    print(text)
    if verdict == 'FAIL':
        print('\nThe SURFACE failsafe -- reached on leak, low thruster battery or GCS')
        print('loss -- would drive the vehicle DOWN. Report against depth_control.cpp')
        print('before any in-water test.')
    return {'PASS': 0, 'FAIL': 1, 'INCONCLUSIVE': 2}[verdict]


if __name__ == '__main__':
    sys.exit(main(*sys.argv[1:]))
