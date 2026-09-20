"""``ros2 run mongla_manager autotune`` -- run the board's relay AUTOTUNE, deliberately.

The board tunes its rate, angle and depth loops by a relay limit cycle and
WRITES AND PERSISTS every PID it tunes, driving all thrusters at full authority.
`SrotFC.autotune` implements it carefully and had no entry point at all. This is
that entry point, and it is an OPERATOR tool, not a mission verb: a mission that
could retune the vehicle mid-run is a mission that can un-tune it.

    autotune                                    # read-only: the live briefing
    autotune --confirm "RUN AUTOTUNE IN WATER"  # needs ARMED, in water, free to rotate

Ctrl-C aborts: the board is put in STABILIZE and disarmed. Refuses to open a port
another process (the manager) already holds, because opening it resets the board.
Exit 0 = tune finished, 1 = refused/failed, 2 = could not reach the board.
"""
from __future__ import annotations

import argparse
import os
import signal
import sys
import threading

os.environ.setdefault('MAVLINK20', '1')

from pymavlink import mavutil                                        # noqa: E402

from mongla_control.fc import srot_protocol as sp                    # noqa: E402
from mongla_control.fc.port_guard import PortGuard, PortBusy         # noqa: E402
from mongla_control.fc.srot_fc import SrotFC                         # noqa: E402

from .connection_config import SROT_BAUD, resolve_srot_profile       # noqa: E402


def _open(path: str, baud: int):
    guard = PortGuard(path)
    guard.acquire()
    kw = {'baud': baud} if path.startswith('/dev/') else {}
    conn = mavutil.mavlink_connection(path, **kw, source_system=sp.SOURCE_SYSID,
                                      source_component=sp.SOURCE_COMPID)
    if conn.wait_heartbeat(timeout=10) is None:
        raise ConnectionError(f'no HEARTBEAT on {path}')
    return guard, conn


def run(fc, confirm: str | None, timeout: float, stop: threading.Event, out=print) -> int:
    if confirm != sp.AUTOTUNE_TOKEN:
        out(fc.autotune_briefing())
        return 1
    ok, why = fc.autotune(confirm, timeout=timeout, abort_fn=stop.is_set)
    out(why)
    return 0 if ok else 1


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--path', default='', help='board device (default: autodetect)')
    ap.add_argument('--baud', type=int, default=SROT_BAUD)
    ap.add_argument('--confirm', default=None,
                    help=f'exactly {sp.AUTOTUNE_TOKEN!r} to run; omit for the briefing')
    ap.add_argument('--timeout', type=float, default=sp.AUTOTUNE_TIMEOUT_S)
    a = ap.parse_args(argv)
    path = a.path or resolve_srot_profile()['conn']
    try:
        guard, conn = _open(path, a.baud)
    except PortBusy as exc:
        print(exc, file=sys.stderr)
        return 2
    except Exception as exc:                                        # noqa: BLE001
        print(f'could not reach the board on {path}: {exc}', file=sys.stderr)
        return 2
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    try:
        return run(SrotFC(conn, log=None), a.confirm, a.timeout, stop)
    finally:
        try:
            conn.close()
        finally:
            guard.release()


if __name__ == '__main__':
    sys.exit(main())
