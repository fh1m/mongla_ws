#!/usr/bin/env python3
"""Is the board armed right now? Read-only; run on Kutum before a deploy.

    python3 - < kutum/armed_probe.py          # over ssh, before anything is synced

Exits 0 when no board is attached or the board's own heartbeat says
DISARMED, 1 when it says ARMED, 2 when a board is attached but cannot be
read. `kutum/release.sh` refuses anything but 0 (#64): a deploy must never
replace the code under an armed vehicle, and "could not tell" is not
"disarmed".

It only listens. Opening the port sends nothing, so it cannot arm, move or
disturb the board. Run it with the manager stopped: the manager holds the
port, and a held port reads as "cannot be read".
"""
from __future__ import annotations

import glob
import os
import sys

MAV_MODE_FLAG_SAFETY_ARMED = 128
MAV_TYPE_GCS = 6
MAV_AUTOPILOT_INVALID = 8
LISTEN_S = 3.0


def candidates():
    """The board's stable name first, then any USB serial device."""
    devs = [d for d in ('/dev/srot',) if os.path.exists(d)]
    devs += sorted(glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*'))
    seen, out = set(), []
    for d in devs:
        real = os.path.realpath(d)
        if real not in seen:
            seen.add(real)
            out.append(d)
    return out


def board_heartbeat(dev, listen_s=LISTEN_S):
    """The vehicle's HEARTBEAT on `dev`, or None. A ground station's
    heartbeat (or another tool's) is not the vehicle's armed state."""
    from pymavlink import mavutil
    m = mavutil.mavlink_connection(dev, baud=115200, autoreconnect=False)
    try:
        import time
        end = time.monotonic() + listen_s
        while time.monotonic() < end:
            hb = m.recv_match(type='HEARTBEAT', blocking=True,
                              timeout=max(0.05, end - time.monotonic()))
            if hb is None:
                continue
            if hb.type == MAV_TYPE_GCS or hb.autopilot == MAV_AUTOPILOT_INVALID:
                continue
            return hb
        return None
    finally:
        m.close()


def main() -> int:
    devs = candidates()
    if not devs:
        print('ok no board attached')
        return 0
    try:
        import pymavlink  # noqa: F401
    except ImportError:
        print(f'FAIL a serial device is attached ({", ".join(devs)}) but pymavlink '
              f'is not installed, so its armed state cannot be read')
        return 2
    unreadable = []
    for dev in devs:
        try:
            hb = board_heartbeat(dev)
        except Exception as e:                         # noqa: BLE001 -- busy, gone
            unreadable.append(f'{dev}: {e}')
            continue
        if hb is None:
            unreadable.append(f'{dev}: no vehicle heartbeat in {LISTEN_S:.0f} s')
            continue
        if hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED:
            print(f'FAIL the board on {dev} reports ARMED')
            return 1
        print(f'ok the board on {dev} reports DISARMED')
        return 0
    print('FAIL a serial device is attached but no board could be read -- '
          + '; '.join(unreadable)
          + '. Power the board so it can report DISARMED, or unplug it.')
    return 2


if __name__ == '__main__':
    sys.exit(main())
