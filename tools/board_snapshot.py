#!/usr/bin/env python3
"""A live reading off the SROT board, for the site and the docs.

Two published numbers disagreed with the board and with each other: the link
load (18.8 % in one ledger row, 51.8 % in another) and `LEAK_EN` (0 on
2026-09-07, then recorded as set to 1 on 2026-09-15). Prose cannot settle that.
This connects to the board and reads it.

It is PASSIVE by default: the port is opened with DTR and RTS deasserted -- the
way Bondor does it, so attaching does not reset the flight controller -- and
nothing is transmitted. With `--params` it also asks for a short list of
parameters by name, which is a read, never a write, and never an actuation.

    python3 tools/board_snapshot.py                  # 20 s, passive
    python3 tools/board_snapshot.py --seconds 30 --params
    python3 tools/board_snapshot.py --port /dev/ttyUSB0

Writes docs/data/board.json and prints a summary. Every figure is stamped with
the moment it was taken, because a live reading is only true of the board it
came from, on the day it came from it.
"""
from __future__ import annotations

import argparse
import json
import struct
import sys
import time
from collections import Counter, defaultdict
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'docs' / 'data' / 'board.json'
BAUD = 115200
# 8N1: ten bits carry each byte, so this is the wire's real byte ceiling
WIRE_BYTES_PER_S = BAUD / 10

# the parameters that settle a published claim, and nothing else
WANT_PARAMS = ['LEAK_EN', 'FRAME_REVERSE', 'MOT_BAT_V_MAX', 'THR_TRIM_EN',
               'MOT_1_DIRECTION', 'MOT_2_DIRECTION', 'MOT_3_DIRECTION', 'MOT_4_DIRECTION',
               'MOT_5_DIRECTION', 'MOT_6_DIRECTION', 'MOT_7_DIRECTION', 'MOT_8_DIRECTION']

# SYS_STATUS sensor bits worth naming (MAV_SYS_STATUS_SENSOR)
BITS = [(1 << 1, '3D_ACCEL'), (1 << 2, '3D_MAG'), (1 << 5, 'ABSOLUTE_PRESSURE'),
        (1 << 12, 'ANGULAR_RATE_CONTROL'), (1 << 13, 'ATTITUDE_STABILIZATION'),
        (1 << 14, 'YAW_POSITION'), (1 << 15, 'Z_ALTITUDE_CONTROL'),
        (1 << 16, 'XY_POSITION_CONTROL'), (1 << 17, 'MOTOR_OUTPUTS'),
        (1 << 20, 'BATTERY'), (1 << 25, 'PROPULSION')]
EXT_BITS = [(1 << 0, 'RECOVERY_SYSTEM'), (1 << 1, 'LEAK')]


def open_port(port: str):
    """DTR and RTS deasserted before anything is read, so attaching does not
    reset the board (srot-integration.md, and what Bondor does)."""
    import serial
    ser = serial.Serial()
    ser.port, ser.baudrate, ser.timeout = port, BAUD, 0.2
    ser.dsrdtr = False
    ser.rtscts = False
    ser.open()
    ser.dtr = False
    ser.rts = False
    return ser


def decode(mask: int, table) -> list[str]:
    return [name for bit, name in table if mask & bit]


def snapshot(port: str, seconds: float, want_params: bool) -> dict:
    from pymavlink.dialects.v20 import common as mavlink
    ser = open_port(port)
    mav = mavlink.MAVLink(None)
    mav.robust_parsing = True

    counts = Counter()
    named = defaultdict(int)
    named_last: dict[str, float] = {}
    params: dict[str, float] = {}
    batteries: dict[int, dict] = {}
    sys_status = hud = heartbeat = None
    statustexts: list[str] = []
    raw_bytes = 0
    bad = 0

    if want_params:
        req = mavlink.MAVLink(None, srcSystem=255, srcComponent=191)
        for name in WANT_PARAMS:
            msg = req.param_request_read_encode(1, 1, name.encode(), -1)
            ser.write(bytes(msg.pack(req)))
            time.sleep(0.02)

    t_end = time.time() + seconds
    while time.time() < t_end:
        chunk = ser.read(4096)
        if not chunk:
            continue
        raw_bytes += len(chunk)
        for byte in chunk:
            try:
                m = mav.parse_char(bytes([byte]))
            except Exception:
                bad += 1
                continue
            if m is None:
                continue
            t = m.get_type()
            if t == 'BAD_DATA':
                bad += 1
                continue
            counts[t] += 1
            if t == 'NAMED_VALUE_FLOAT':
                nm = m.name.strip('\x00')
                named[nm] += 1
                named_last[nm] = m.value
            elif t == 'PARAM_VALUE':
                params[m.param_id.strip('\x00')] = m.param_value
            elif t == 'SYS_STATUS':
                sys_status = m.to_dict()
                # pymavlink cannot decode the EXTENDED health words (where LEAK
                # lives): v2 truncates the payload, so take the declared length,
                # zero-pad to 43 and unpack three uint32 at 31/35/39 --
                # srot-board-soul.md §4. A naive unpack reads past the end.
                try:
                    buf = bytes(m.get_msgbuf())
                    payload = buf[10:10 + buf[1]].ljust(43, b'\x00')
                    ext = struct.unpack_from('<III', payload, 31)
                    sys_status['present_extended'] = ext[0]
                    sys_status['enabled_extended'] = ext[1]
                    sys_status['health_extended'] = ext[2]
                except Exception:
                    pass
            elif t == 'VFR_HUD':
                hud = m.to_dict()
            elif t == 'HEARTBEAT':
                heartbeat = m.to_dict()
            elif t == 'BATTERY_STATUS':
                batteries[m.id] = {'voltage_mv': m.voltages[0] if m.voltages else None,
                                   'current_ca': m.current_battery}
            elif t == 'STATUSTEXT':
                s = m.text.strip('\x00')
                if s not in statustexts:
                    statustexts.append(s)
    ser.close()

    rates = {k: round(v / seconds, 1) for k, v in sorted(counts.items(), key=lambda kv: -kv[1])}
    out = {
        'taken': datetime.now(timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ'),
        'port': port, 'baud': BAUD, 'seconds': seconds, 'passive': not want_params,
        'link': {
            'bytes_per_s': round(raw_bytes / seconds, 1),
            'wire_bytes_per_s': WIRE_BYTES_PER_S,
            'used_pct': round(raw_bytes / seconds / WIRE_BYTES_PER_S * 100, 1),
            'messages_per_s': round(sum(counts.values()) / seconds, 1),
            'bad_frames': bad,
        },
        'rates_hz': rates,
        'named_value_float': {k: {'hz': round(v / seconds, 1), 'last': named_last.get(k)}
                              for k, v in sorted(named.items(), key=lambda kv: -kv[1])},
        'params': params,
        'batteries': batteries,
        'statustext': statustexts[:12],
    }
    if heartbeat:
        out['heartbeat'] = {'base_mode': heartbeat['base_mode'],
                            'custom_mode': heartbeat['custom_mode'],
                            'system_status': heartbeat['system_status'],
                            'armed': bool(heartbeat['base_mode'] & 128)}
    if hud:
        out['hud'] = {'alt_m': hud['alt'], 'heading_deg': hud['heading'],
                      'groundspeed': hud['groundspeed'], 'throttle': hud['throttle']}
    if sys_status:
        out['sensors'] = {
            'present': decode(sys_status['onboard_control_sensors_present'], BITS),
            'enabled': decode(sys_status['onboard_control_sensors_enabled'], BITS),
            'health': decode(sys_status['onboard_control_sensors_health'], BITS),
            'present_extended': decode(sys_status.get('present_extended', 0), EXT_BITS),
            'enabled_extended': decode(sys_status.get('enabled_extended', 0), EXT_BITS),
            'health_extended': decode(sys_status.get('health_extended', 0), EXT_BITS),
            'extended_raw': {k: sys_status.get(k) for k in
                             ('present_extended', 'enabled_extended', 'health_extended')},
            'voltage_battery_mv': sys_status['voltage_battery'],
            'raw': {k: sys_status[k] for k in sys_status if k.startswith('onboard')},
        }
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--port', default='/dev/ttyUSB0')
    ap.add_argument('--seconds', type=float, default=20.0)
    ap.add_argument('--params', action='store_true', help='also read the named parameters')
    args = ap.parse_args()
    try:
        data = snapshot(args.port, args.seconds, args.params)
    except Exception as exc:                       # a missing board is not a crash
        print(f'no reading: {exc}', file=sys.stderr)
        return 2
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(data, indent=1) + '\n')
    link = data['link']
    print(f'{data["taken"]}  {data["port"]} @ {data["baud"]}')
    print(f'link   {link["bytes_per_s"]:.0f} B/s of {link["wire_bytes_per_s"]:.0f} '
          f'= {link["used_pct"]:.1f} %   {link["messages_per_s"]:.0f} msg/s   bad {link["bad_frames"]}')
    print('rates  ' + ', '.join(f'{k} {v}' for k, v in list(data['rates_hz'].items())[:8]))
    if data.get('sensors'):
        s = data['sensors']
        print(f'leak   present_ext={s["present_extended"]} enabled_ext={s["enabled_extended"]} '
              f'health_ext={s["health_extended"]}')
    if data['params']:
        print('params ' + ', '.join(f'{k}={v:g}' for k, v in data['params'].items()))
    if data['named_value_float']:
        print('names  ' + ', '.join(f'{k}={v["last"]:g}@{v["hz"]}Hz'
                                    for k, v in list(data['named_value_float'].items())[:8]))
    print(f'wrote  {OUT.relative_to(ROOT)}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
