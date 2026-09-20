#!/usr/bin/env python3
"""What does this board ACTUALLY answer? Measured, not read off a bitmask.

We use 5 of MAVLink's 193 commands and 36 of its 190 messages. The useful
question is not which of the rest exist -- it is which of them this firmware
will answer, and that is a measurement.

WHY NOT JUST READ `AUTOPILOT_VERSION.capabilities`
--------------------------------------------------
Because it is a claim. The board advertises 8206 (MAVLINK2, COMMAND_INT,
MISSION_INT, PARAM_FLOAT) and that is neither a floor nor a ceiling:
`MAV_CMD_SROT_VISION` (31001) is defined in the firmware's own headers and
returns UNSUPPORTED, while things it never advertises may work fine. The
calibration round cost four wrong answers to trusting a plausible number over
an experiment; this asks the board.

TWO INDEPENDENT SIGNALS PER PROBE, and the gap between them is the finding
-------------------------------------------------------------------------
  ack       what COMMAND_ACK says
  arrived   whether the thing actually turned up on the wire

ACCEPTED with nothing arriving is the interesting case -- it is the shape of
every silent-success defect in this project, from the pre-rev-13 disarmed
SROT_MOVE to a vision verb running in SURFACE. Reporting only the ACK would
reproduce it.

MESSAGES ARE REQUESTED ONE-SHOT (`MAV_CMD_REQUEST_MESSAGE`), never by raising
a stream rate: a rate sweep across 190 ids would flood a 115200 line that is
already carrying 5.5 kB/s, and would leave the board reconfigured afterwards.
One-shot is self-limiting and leaves no state behind.

SAFE BY CONSTRUCTION: nothing here arms, moves, writes a parameter, reboots or
calibrates. The command list is an allowlist, and the two commands that would
disturb the vehicle (PREFLIGHT_REBOOT_SHUTDOWN, PREFLIGHT_CALIBRATION) are
deliberately absent.
"""
from __future__ import annotations

import argparse
import os
import sys
import time

from pymavlink import mavutil

M = mavutil.mavlink

_WS = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                   'src', 'mongla_control')
if os.path.isdir(_WS) and _WS not in sys.path:
    sys.path.insert(0, _WS)

# Read-only / introspection commands. Nothing that moves, writes or reboots.
_COMMANDS = [
    ('REQUEST_PROTOCOL_VERSION', 519, 'PROTOCOL_VERSION',
     'exact MAVLink version + library hashes'),
    ('REQUEST_AUTOPILOT_CAPABILITIES', 520, 'AUTOPILOT_VERSION',
     'the capability bitmask, on demand'),
    ('GET_MESSAGE_INTERVAL', 510, 'MESSAGE_INTERVAL',
     'read back a stream rate instead of assuming it took'),
    ('REQUEST_FLIGHT_INFORMATION', 264, 'FLIGHT_INFORMATION',
     'flight uptime / arm time'),
    ('DO_SEND_BANNER', 42428, 'STATUSTEXT', 'firmware banner on demand'),
    ('REQUEST_CAMERA_INFORMATION', 521, 'CAMERA_INFORMATION',
     'negative control -- this board has no camera'),
]

# Raw messages that start a protocol rather than a command.
_PROTOCOLS = [
    ('LOG_REQUEST_LIST', 'LOG_ENTRY',
     'the standard log-download protocol -- would let us pull the board SD log'),
    ('FILE_TRANSFER_PROTOCOL', 'FILE_TRANSFER_PROTOCOL',
     'MAVFTP; not in the advertised bitmask, asked anyway'),
    ('TIMESYNC', 'TIMESYNC',
     'round-trip clock sync -- would put board time and host time on one axis'),
    ('PARAM_REQUEST_LIST', 'PARAM_VALUE',
     'full parameter sweep -- a config snapshot per run'),
    ('MISSION_REQUEST_LIST', 'MISSION_COUNT', 'does it hold a mission at all'),
]


class Probe:
    def __init__(self, dev, baud=115200):
        self.conn = mavutil.mavlink_connection(
            dev, baud=baud, source_system=255, source_component=190)
        print(f'  waiting for a heartbeat on {dev} ...')
        hb = self.conn.wait_heartbeat(timeout=15)
        if hb is None:
            raise SystemExit('no heartbeat -- is the board powered and free?')
        self.sysid = self.conn.target_system
        self.compid = self.conn.target_component
        print(f'  board sysid {self.sysid} compid {self.compid}, '
              f'type {hb.type}, autopilot {hb.autopilot}, '
              f'mavlink v{hb.mavlink_version}\n')

    def drain(self, seconds):
        """Collect everything that arrives in a window.

        Draining EVERY message rather than matching one type is deliberate: a
        board that answers a request with something unexpected -- an unrelated
        message, a STATUSTEXT explaining the refusal -- is exactly what a
        type-matched wait would hide.
        """
        got = {}
        end = time.time() + seconds
        while time.time() < end:
            m = self.conn.recv_match(blocking=False)
            if m is None:
                time.sleep(0.002)
                continue
            got.setdefault(m.get_type(), []).append(m)
        return got

    def command(self, cmd, want, timeout=1.2, **params):
        p = [float(params.get(f'p{i}', 0.0)) for i in range(1, 8)]
        self.conn.mav.command_long_send(self.sysid, self.compid, cmd, 0, *p)
        got = self.drain(timeout)
        acks = [a for a in got.get('COMMAND_ACK', []) if int(a.command) == cmd]
        ack = acks[-1].result if acks else None
        return ack, (want in got), got


_ACK = {0: 'ACCEPTED', 1: 'TEMP_REJECTED', 2: 'DENIED', 3: 'UNSUPPORTED',
        4: 'FAILED', 5: 'IN_PROGRESS', 6: 'CANCELLED'}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--dev', default='/dev/ttyUSB0')
    ap.add_argument('--messages', action='store_true',
                    help='also sweep every message id with REQUEST_MESSAGE')
    a = ap.parse_args()

    print('\n=== SROT / Hengla MAVLink capability probe ===\n')
    pr = Probe(a.dev)

    print('--- commands ------------------------------------------------')
    print(f'  {"command":<34} {"ack":<14} {"replied":<8}  note')
    for name, cmd, want, note in _COMMANDS:
        ack, arrived, _ = pr.command(cmd, want, p1=1.0)
        verdict = _ACK.get(ack, 'NO ACK' if ack is None else str(ack))
        flag = 'yes' if arrived else 'no'
        mark = '  <-- ACK but no reply' if (ack == 0 and not arrived) else ''
        print(f'  {name:<34} {verdict:<14} {flag:<8}  {note}{mark}')

    print('\n--- protocols -----------------------------------------------')
    for name, want, note in _PROTOCOLS:
        try:
            if name == 'LOG_REQUEST_LIST':
                pr.conn.mav.log_request_list_send(pr.sysid, pr.compid, 0, 0xffff)
            elif name == 'FILE_TRANSFER_PROTOCOL':
                pr.conn.mav.file_transfer_protocol_send(
                    0, pr.sysid, pr.compid, [0] * 251)
            elif name == 'TIMESYNC':
                pr.conn.mav.timesync_send(0, int(time.time() * 1e9))
            elif name == 'PARAM_REQUEST_LIST':
                pr.conn.mav.param_request_list_send(pr.sysid, pr.compid)
            elif name == 'MISSION_REQUEST_LIST':
                pr.conn.mav.mission_request_list_send(pr.sysid, pr.compid)
        except Exception as exc:                      # noqa: BLE001
            print(f'  {name:<26} SEND FAILED  {exc!r}')
            continue
        got = pr.drain(2.5)
        n = len(got.get(want, []))
        print(f'  {name:<26} {"REPLIED" if n else "silent":<9} '
              f'{n:>4} {want:<24} {note}')

    if a.messages:
        print('\n--- which messages will it emit on request? ------------------')
        ids = sorted({v for k, v in vars(M).items()
                      if k.startswith('MAVLINK_MSG_ID_') and isinstance(v, int)})
        emit, acked_only = [], []
        for mid in ids:
            pr.drain(0.0)
            ack, _, got = pr.command(512, '__none__', timeout=0.22, p1=float(mid))
            name = M.mavlink_map.get(mid)
            name = name.msgname if name is not None else f'ID {mid}'
            if name in got:
                emit.append((mid, name))
            elif ack == 0:
                acked_only.append((mid, name))
        print(f'  emitted on request ({len(emit)}):')
        for mid, n in emit:
            print(f'    {mid:>5}  {n}')
        if acked_only:
            print(f'\n  ACCEPTED the request and sent nothing ({len(acked_only)}) '
                  f'-- the silent-success shape:')
            for mid, n in acked_only[:20]:
                print(f'    {mid:>5}  {n}')
    print()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
