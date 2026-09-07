#!/usr/bin/env python3
"""GATE 0 from the mission host: identify each thruster, then learn its sign.

Both halves of this procedure have been implemented on the board the whole
time and reachable only from Bondor, which is not where the operator is
standing when the hull is in the water.

  briefing   read the three values that decide what a detect run does. They
             live in three places and NOTHING displays their product.
  test       spin ONE thruster so a human can say which one moved. This is
             how the physical numbering is established, and it is a
             prerequisite for believing anything `detect` reports.
  detect     pulse all eight and CORRECT each sign from the gyro response.

ORDER MATTERS AND THE TOOL DOES NOT ENFORCE IT, because enforcing it would
mean tracking state we cannot verify. `detect` writes signs derived from an
assumed motor->position map; if `test` has not confirmed that map, `detect`
produces a self-consistent calibration for a vehicle wired differently from
the one the mixer believes in.

⚠ `detect` NEEDS WATER and a hull free to rotate. In air the gyro response is
below the 0.05 rad/s gate: the run reports FAIL and writes nothing on fw rev 6
and later -- before that it reset every thruster to +1 and reported SUCCESS,
which is why the firmware revision is checked here rather than assumed.
"""
from __future__ import annotations

import argparse
import os
import sys

WS = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                  'src', 'duburi_control')
if WS not in sys.path:
    sys.path.insert(0, WS)

from pymavlink import mavutil                                   # noqa: E402

from duburi_control.fc import srot_protocol as sp               # noqa: E402
from duburi_control.fc.port_guard import PortGuard              # noqa: E402
from duburi_control.fc.srot_fc import SrotFC                    # noqa: E402


def _connect(dev):
    guard = PortGuard(dev)
    guard.acquire()          # a second opener reboots the board mid-procedure
    conn = mavutil.mavlink_connection(dev, baud=115200,
                                      source_system=sp.SOURCE_SYSID,
                                      source_component=sp.SOURCE_COMPID)
    if conn.wait_heartbeat(timeout=10) is None:
        raise SystemExit(f'no heartbeat on {dev}')
    return guard, conn, SrotFC(conn, log=None)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('action', choices=('briefing', 'test', 'detect'))
    ap.add_argument('--dev', default='/dev/ttyUSB0')
    ap.add_argument('--motor', type=int, default=1, help='test: 1..8')
    ap.add_argument('--throttle', type=float, default=15.0, help='test: -100..100')
    ap.add_argument('--seconds', type=float, default=2.0, help='test: duration')
    ap.add_argument('--yes', action='store_true',
                    help='detect: supply the confirmation token')
    A = ap.parse_args()

    guard, conn, fc = _connect(A.dev)
    try:
        if A.action == 'briefing':
            print(fc.motor_detect_briefing())
            return 0

        rev = fc.read_behaviour_rev()
        if A.action == 'detect' and (rev is None or rev < 6):
            # Not a style check. Before rev 6 an inconclusive detect stored +1
            # for every thruster and reported SUCCESS, so a dry run DESTROYED a
            # good calibration and looked like it had produced one.
            print(f'REFUSED: firmware behaviour rev {rev} -- detect only became '
                  f'safe to run at rev 6 (before that a failed run silently '
                  f'reset every thruster to +1 and reported SUCCESS)')
            return 2

        if not fc.is_armed():
            print('not armed. Both actions drive thrusters and the board '
                  'refuses them disarmed -- arm from Bondor or `duburi arm`, '
                  'with the hull restrained or the props off.')
            return 2

        if A.action == 'test':
            print(f'motor {A.motor} at {A.throttle:.0f}% for {A.seconds:.1f}s '
                  f'-- WATCH WHICH ONE TURNS')
            ok, why = fc.motor_test(A.motor, A.throttle, seconds=A.seconds)
        else:
            # No token on the command line by default: `--yes` is the operator
            # saying it after having been shown the briefing, and without it the
            # driver returns the briefing rather than running.
            ok, why = fc.motor_detect(sp.MOTOR_DETECT_TOKEN if A.yes else None)
        print(why)
        return 0 if ok else 1
    finally:
        conn.close()
        guard.release()


if __name__ == '__main__':
    raise SystemExit(main())
