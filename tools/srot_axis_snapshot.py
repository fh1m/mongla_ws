#!/usr/bin/env python3
"""Snapshot the axis configuration — step 1 of the pre-dive gate sequence.

`MOT_n_DIRECTION` and `CAL_MDIR` MULTIPLY (fw `task_control_loop.cpp:140-145`), and
neither alone tells you what the vehicle will do. Reading only the first is exactly
how mongla_ws concluded "config is correct" on 2026-08-07 while `CAL_MDIR` had
already moved; the firmware team's CFG banner caught it because it prints the
product. This prints the product too.

Run it BEFORE any armed manoeuvre and again AFTER a successful MOTOR_DETECT — detect
writes `CAL_MDIR`, so the two snapshots are how you prove what it changed. Diff them
with `--compare <file>`.

⚠ A SUCCESSFUL MOTOR_DETECT MAKES `FRAME_REVERSE = 1` WRONG. Detect converges every
thruster to agree with its mixer column; `FRAME_REVERSE` then negates all six demands
and re-inverts the corrected frame. After a SUCCESS, `FRAME_REVERSE` must go to 0 in
the same session (`BUGS.md` gate 1).

Read-only. Writes nothing to the board.
"""
import os
os.environ['MAVLINK20'] = '1'          # BEFORE pymavlink imports
import argparse
import json
import sys
import time
from datetime import datetime
from pathlib import Path

_SRC = Path(__file__).resolve().parent.parent / 'src'
for _pkg in ('mongla_control', 'mongla_manager'):
    _p = _SRC / _pkg
    if _p.is_dir():
        sys.path.insert(0, str(_p))

from pymavlink import mavutil
import mongla_control.fc.srot_protocol as sp

# CAL_MDIR is 1-indexed on the wire (CAL_MDIR1..8) while motor_dir[] is 0-indexed
# in firmware. Ask for what the param table actually publishes.
NAMES = (['FRAME_REVERSE']
         + [f'MOT_{n}_DIRECTION' for n in range(1, 9)]
         + [f'CAL_MDIR{n}' for n in range(1, 9)]
         + ['DEPTH_P', 'MAG_YAW_REF', 'FS_GCS_ENABLE', 'FS_GCS_SYSID',
            'FS_GCS_COMPID', 'JS_GAIN_DEFAULT', 'CAL_LVL_R', 'CAL_LVL_P'])


def read_param(m, name, tries=4, timeout_s=1.5):
    """Sequential only — pymavlink keeps ONE slot per msgid, so two requests in
    flight means the second overwrites the first and the first times out looking
    exactly like a dropped frame. Retries cover the bridge's ~8-9% loss."""
    for _ in range(tries):
        m.mav.param_request_read_send(sp.VEHICLE_SYSID, sp.VEHICLE_COMPID,
                                      name.encode(), -1)
        end = time.time() + timeout_s
        while time.time() < end:
            msg = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.4)
            if msg is None:
                continue
            pid = msg.param_id
            if isinstance(pid, bytes):
                pid = pid.decode(errors='ignore')
            if pid.strip('\x00').strip() == name:
                return float(msg.param_value)
    return None


ap = argparse.ArgumentParser()
ap.add_argument('--device', default=None, help='override the auto-detected transport')
ap.add_argument('--save', default=None, help='write the snapshot to this JSON file')
ap.add_argument('--compare', default=None, help='diff against a saved snapshot')
args = ap.parse_args()

baud = None
if args.device:
    conn_str = args.device
    if conn_str.startswith('/dev/'):
        baud = sp.BAUD
else:
    from mongla_manager.connection_config import resolve_srot_profile
    prof = resolve_srot_profile()
    conn_str, baud = prof['conn'], prof['baud']
print(f'transport: {conn_str}' + (f' @ {baud}' if baud else ''))

m = (mavutil.mavlink_connection(conn_str, baud=baud, source_component=sp.SOURCE_COMPID)
     if baud else
     mavutil.mavlink_connection(conn_str, source_component=sp.SOURCE_COMPID))
hb = m.wait_heartbeat(timeout=20)
if hb is None:
    sys.exit('NO HEARTBEAT — link dead. Recreate the Bridget bridge and retry.')
armed = bool(hb.base_mode & 128)
print(f'heartbeat: sys {hb.get_srcSystem()} comp {hb.get_srcComponent()} '
      f'armed={armed}\n')

vals = {}
for n in NAMES:
    # FRAME_REVERSE decides the whole verdict, so it gets more attempts than the
    # rest: the bridge drops ~8-9% of frames and a blank here is indistinguishable
    # from 0 to a reader skimming the output.
    vals[n] = read_param(m, n, tries=8 if n == 'FRAME_REVERSE' else 4)

mot = [vals.get(f'MOT_{n}_DIRECTION') for n in range(1, 9)]
cal = [vals.get(f'CAL_MDIR{n}') for n in range(1, 9)]
fr = vals.get('FRAME_REVERSE')


def _fmt(seq):
    return '[' + ', '.join('  ?' if v is None else f'{int(v):+d}' for v in seq) + ']'


print('--- axis configuration ---')
print(f'  MOT_n_DIRECTION : {_fmt(mot)}')
print(f'  CAL_MDIR        : {_fmt(cal)}')
if None not in mot and None not in cal:
    eff = [int(a) * int(b) for a, b in zip(mot, cal)]
    print(f'  effective       : {_fmt(eff)}      <- what reaches the mixer')
else:
    eff = None
    print('  effective       : UNREADABLE — do not arm on a partial read')
print(f'  FRAME_REVERSE   : {"?" if fr is None else int(fr)}')

print('\n--- everything else ---')
for n in NAMES:
    if n.startswith(('MOT_', 'CAL_MDIR', 'FRAME_')):
        continue
    v = vals[n]
    print(f'  {n:18s} = {"UNREADABLE" if v is None else f"{v:g}"}')

print('\n--- verdict ---')
# NET response, relative to what the mixer intends, is
#     net[m] = frame_reverse_sign * w[m] * effective[m]
# where w is the TRUE per-thruster wiring and frame_reverse_sign is -1 when
# FRAME_REVERSE = 1. Params give us `effective` and the sign; they can NEVER give
# us w. So a uniform `effective` is NOT evidence that the vehicle is fine -- if w
# is non-uniform, net is non-uniform no matter how tidy `effective` looks. That is
# precisely how "config is correct" was concluded on 2026-08-07 from half the
# product, so this block deliberately refuses to reassure.
if eff is None or fr is None:
    print('  INCOMPLETE READ — re-run before arming.')
    if fr is None:
        print('  FRAME_REVERSE did not answer, and the verdict depends on it.')
else:
    fr_sign = -1 if int(fr) == 1 else +1
    print(f'  net[m] = {fr_sign:+d} (FRAME_REVERSE={int(fr)}) x w[m] x '
          f'effective[m]   — and w is UNKNOWN from here.')
    print('  The two hypotheses the parameters cannot separate:')
    for label, w in (('all eight wired normally', [1] * 8),
                     ('M1 and M8 wired backwards', [-1, 1, 1, 1, 1, 1, 1, -1])):
        net = [fr_sign * a * b for a, b in zip(w, eff)]
        uniform = len(set(net)) == 1
        if uniform and net[0] == 1:
            note = 'CORRECT'
        elif uniform:
            note = 'every axis inverted — uniform, so torques intact; FRAME_REVERSE fixes it'
        else:
            note = '⛔ NON-UNIFORM — broken torques, the 2026-08-06 yaw-spin fault'
        print(f'    if w = {_fmt(w)}  ({label:<26}) -> net {_fmt(net)}  {note}')
    print('\n  ONLY PHYSICAL THRUST SEPARATES THESE. Run gate 0 before arming:')
    print('    per-thruster motor test (props on, hull restrained), or')
    print('    in-water MOTOR_DETECT, armed — require SUCCESS.')
    if int(fr) == 1 and len(set(eff)) == 1 and eff[0] == 1:
        print('\n  ⛔ AND NOTE: this is also the POST-DETECT shape. A successful')
        print('  MOTOR_DETECT drives effective to agree with the mixer, after which')
        print('  FRAME_REVERSE = 1 re-inverts every axis. If detect has just')
        print('  SUCCEEDED, set FRAME_REVERSE = 0 now (gate 1).')

snap = {'utc': datetime.utcnow().isoformat() + 'Z', 'armed': armed,
        'transport': conn_str, 'params': vals,
        'effective': eff}
if args.save:
    Path(args.save).write_text(json.dumps(snap, indent=2))
    print(f'\nsaved -> {args.save}')

if args.compare:
    old = json.loads(Path(args.compare).read_text())
    print(f'\n--- diff vs {args.compare} ({old.get("utc")}) ---')
    changed = False
    for n in NAMES:
        a, b = old['params'].get(n), vals.get(n)
        if a != b:
            changed = True
            print(f'  {n:18s} {a} -> {b}')
    print('  (no parameter changed)' if not changed else '')
    if old.get('effective') != eff:
        print(f'  EFFECTIVE {_fmt(old.get("effective") or [])} -> {_fmt(eff or [])}')

sys.exit(0 if eff is not None else 2)
