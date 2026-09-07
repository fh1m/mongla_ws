#!/usr/bin/env python3
"""Set FS_GCS_SYSID/FS_GCS_COMPID to 255/191 and PROVE it reached flash.

WHY THIS IS A SCRIPT AND NOT TWO CLICKS. This param has now reverted to the bench
wildcard (COMPID=0) twice, each time after being "set". 0 means any station satisfies
the GCS failsafe, so a dead Jetson looks like a live GCS and the hull station-keeps
instead of surfacing. The failure is silent in both directions -- a PARAM_SET that
never landed looks exactly like one that did, and on this board so does a save that
never reached NVS.

So every step is confirmed rather than assumed:
  1. read      -- what is actually there now
  2. PARAM_SET -- then re-read; the PARAM_VALUE echo is not trusted on its own
  3. PREFLIGHT_STORAGE
  4. wait for STATUSTEXT "Params saved to flash"  <- rev 7, and the whole point:
     the COMMAND_ACK fires the instant the request PARSES. The NVS write is deferred
     to the Core-0 update(). The ACK has never meant "written".
  5. re-read after the save

Transport is auto-detected exactly the way the manager does it (direct USB serial
first, then the BlueOS bridge over UDP), so this works on the deck or on the bench.
Direct USB is preferable for a write: 0% BAD_DATA vs ~8-9% over the bridge.

Run with --revert to put the bench wildcard back (COMPID=0) if you need bench mode.
"""
import os
os.environ['MAVLINK20'] = '1'          # BEFORE pymavlink imports
import argparse
import sys
import time

# Repo-relative, so this runs on the Jetson and on any clone. Falls back to whatever
# is already importable (a sourced install/) if the src layout is not where we expect.
from pathlib import Path
_SRC = Path(__file__).resolve().parent.parent / 'src'
for _pkg in ('duburi_control', 'duburi_manager'):
    _p = _SRC / _pkg
    if _p.is_dir():
        sys.path.insert(0, str(_p))

from pymavlink import mavutil
import duburi_control.fc.srot_protocol as sp

WANT_SYSID = 255
BENCH_COMPID = 0


def read_param(m, name, tries=4, timeout_s=1.5):
    """Sequential only -- pymavlink keeps ONE slot per msgid."""
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


def set_param(m, name, value, tries=4):
    """PARAM_SET, then VERIFY BY RE-READING. The echo alone is not proof."""
    for attempt in range(1, tries + 1):
        m.mav.param_set_send(sp.VEHICLE_SYSID, sp.VEHICLE_COMPID, name.encode(),
                             float(value), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        time.sleep(0.3)
        got = read_param(m, name)
        if got is not None and abs(got - value) < 0.001:
            return True, got
        print(f'    attempt {attempt}: read back {got!r}, want {value} -- retrying')
    return False, got


def save_to_flash(m, timeout_s=8.0):
    """PREFLIGHT_STORAGE, then wait for the STATUSTEXT -- not the ACK.

    Returns (ack_result, statustext_or_None). On rev >= 7 a successful save emits
    "Params saved to flash" at MAV_SEVERITY_INFO. On rev < 7 success is SILENT, so a
    missing statustext there is inconclusive rather than a failure.
    """
    m.mav.command_long_send(sp.VEHICLE_SYSID, sp.VEHICLE_COMPID,
                            mavutil.mavlink.MAV_CMD_PREFLIGHT_STORAGE, 0,
                            1, 0, 0, 0, 0, 0, 0)
    ack, texts = None, []
    end = time.time() + timeout_s
    while time.time() < end:
        msg = m.recv_match(type=['COMMAND_ACK', 'STATUSTEXT'], blocking=True, timeout=0.5)
        if msg is None:
            continue
        if msg.get_type() == 'COMMAND_ACK':
            if msg.command == mavutil.mavlink.MAV_CMD_PREFLIGHT_STORAGE:
                ack = msg.result
        else:
            t = msg.text
            if isinstance(t, bytes):
                t = t.decode(errors='ignore')
            t = t.strip('\x00').strip()
            texts.append(t)
            # MATCH "Params saved", NOT merely "saved". The board emits TWO lines and
            # the calibration one comes FIRST:
            #     STATUSTEXT: Calibration saved + verified on flash
            #     STATUSTEXT: Params saved to flash          <- the one we need
            # An early return on any "saved" grabs the calibration line and reports a
            # params write that may never have happened -- which is exactly the
            # silence-means-success failure rev 7 was written to remove, recreated on
            # the client side. Measured: this bit on the first run of this script.
            low = t.lower()
            if 'param' in low and 'saved' in low:
                return ack, texts
    return ack, texts


ap = argparse.ArgumentParser()
ap.add_argument('--revert', action='store_true',
                help='put the bench wildcard (COMPID=0) back')
ap.add_argument('--device', default=None, help='override the auto-detected transport')
args = ap.parse_args()

target_compid = BENCH_COMPID if args.revert else sp.SOURCE_COMPID

baud = None
if args.device:
    conn_str = args.device
    if conn_str.startswith('/dev/'):
        baud = sp.BAUD
else:
    from duburi_manager.connection_config import resolve_srot_profile
    prof = resolve_srot_profile()          # -> {'conn': str, 'baud': int|None}
    conn_str, baud = prof['conn'], prof['baud']
print(f'transport: {conn_str}' + (f' @ {baud}' if baud else ''))

m = (mavutil.mavlink_connection(conn_str, baud=baud, source_component=sp.SOURCE_COMPID)
     if baud else
     mavutil.mavlink_connection(conn_str, source_component=sp.SOURCE_COMPID))
hb = m.wait_heartbeat(timeout=20)
if hb is None:
    sys.exit('NO HEARTBEAT -- link dead')
print(f'heartbeat: sys {hb.get_srcSystem()} comp {hb.get_srcComponent()} '
      f'armed={bool(hb.base_mode & 128)}')
if bool(hb.base_mode & 128):
    sys.exit('VEHICLE IS ARMED -- disarm before writing params')

print('\n[1] current')
for n in ('FS_GCS_ENABLE', 'FS_GCS_SYSID', 'FS_GCS_COMPID'):
    print(f'    {n:16s} = {read_param(m, n)}')

print(f'\n[2] write  FS_GCS_SYSID={WANT_SYSID}  FS_GCS_COMPID={target_compid}')
ok_s, _ = set_param(m, 'FS_GCS_SYSID', WANT_SYSID)
ok_c, _ = set_param(m, 'FS_GCS_COMPID', target_compid)
if not (ok_s and ok_c):
    sys.exit('PARAM_SET DID NOT TAKE -- nothing saved, board unchanged')
print('    both confirmed by re-read')

print('\n[3] save to flash')
ack, texts = save_to_flash(m)
print(f'    COMMAND_ACK = {ack}  (0=ACCEPTED -- means REQUEST RECEIVED, not written)')
for t in texts:
    print(f'    STATUSTEXT: {t}')
saved = any('param' in t.lower() and 'saved' in t.lower() for t in texts)
if not saved:
    print('    !! no "saved" statustext. On rev >= 7 that means the write did NOT '
          'complete. POWER-CYCLE AND RE-CHECK before trusting this.')

print('\n[4] verify after save')
final = {n: read_param(m, n) for n in ('FS_GCS_ENABLE', 'FS_GCS_SYSID', 'FS_GCS_COMPID')}
for n, v in final.items():
    print(f'    {n:16s} = {v}')

good = (final['FS_GCS_COMPID'] is not None
        and int(final['FS_GCS_COMPID']) == target_compid)
print('\n' + ('OK -- ' + ('bench wildcard restored' if args.revert else
                          'failsafe scoped to the companion (191)')
             if good else 'FAILED -- value did not stick'))
print('⚠ POWER-CYCLE THE BOARD AND RUN THIS AGAIN (read-only) TO PROVE IT PERSISTED.')
print('   This param has silently reverted twice; a readback before a power cycle')
print('   only proves RAM, not NVS.')
sys.exit(0 if good else 1)
