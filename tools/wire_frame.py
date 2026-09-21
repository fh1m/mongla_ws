#!/usr/bin/env python3
"""One verb, on the wire -- the byte figure on the site, generated, not drawn.

`mongla move_forward 3 --gain 40` leaves the Pi as ONE MAVLink 2 frame. This
tool builds that frame with the project's own verb table (`srot_fc._build_params`)
and its own wire constants (`srot_protocol`), packs it with pymavlink exactly as
`SrotFC._command_long` does, decodes it back field by field, and writes the
figure into docs/index.html between the `wire:begin` / `wire:end` markers.

So every byte on the page is a byte this code would send. Change the verb table
or a constant and re-run: the figure follows, or the round-trip assert fails.

    python3 tools/wire_frame.py            # rewrite the figure in place
    python3 tools/wire_frame.py --check    # exit 1 if the page has drifted
"""
from __future__ import annotations

import argparse
import html
import struct
import sys
import types
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _site_splice import splice  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
PKG = ROOT / 'src' / 'mongla_control' / 'mongla_control'

# The example verb. Ordinary on purpose: the figure explains the wire, not an
# exotic command.
VERB, KW = 'move_forward', {'duration': 3.0, 'gain': 40}
PAYLOAD_FULL = 33          # COMMAND_LONG: 7 x f32 + u16 + 3 x u8
HEADER = 10                # MAVLink 2 header, STX through the 3-byte msgid


def _load_fc():
    """Import mongla_control.fc WITHOUT the package __init__, which pulls in the
    ROS action types. The fc subpackage is deliberately import-light."""
    pkg = types.ModuleType('mongla_control')
    pkg.__path__ = [str(PKG)]
    sys.modules.setdefault('mongla_control', pkg)
    from mongla_control.fc import srot_fc, srot_protocol
    return srot_fc, srot_protocol


def build_frame():
    fc, sp = _load_fc()
    from pymavlink.dialects.v20 import common as mavlink
    p1, p2, p3, p4, p5 = fc._build_params(VERB, dict(KW))
    mav = mavlink.MAVLink(None, srcSystem=sp.SOURCE_SYSID, srcComponent=sp.SOURCE_COMPID)
    # same argument order as SrotFC._command_long
    msg = mav.command_long_encode(sp.VEHICLE_SYSID, sp.VEHICLE_COMPID, sp.CMD_SROT_MOVE, 0,
                                  float(p1), float(p2), float(p3), float(p4),
                                  float(p5), 0.0, 0.0)
    raw = bytes(msg.pack(mav))
    back = mavlink.MAVLink(None).parse_char(raw)     # validates the CRC
    if back is None or back.command != sp.CMD_SROT_MOVE:
        raise SystemExit('frame does not round-trip through pymavlink')
    return raw, sp


def fields(raw: bytes, sp):
    """(offset, length, name, decoded value, role, note) for every byte range.

    Decoded from `raw` itself -- nothing here restates a value it did not read."""
    plen = raw[1]
    payload = raw[HEADER:HEADER + plen] + bytes(PAYLOAD_FULL - plen)  # v2 trims zeros
    f = struct.unpack('<7fHBBB', payload)
    msgid = int.from_bytes(raw[7:10], 'little')
    move_names = {getattr(sp, n): n for n in dir(sp)
                  if n.startswith('MOVE_') and isinstance(getattr(sp, n), int)}
    out = [
        (0, 1, 'STX', f'0x{raw[0]:02X}', 'hdr', 'a MAVLink 2 frame starts here'),
        (1, 1, 'LEN', str(plen), 'hdr',
         f'payload length — MAVLink 2 cut {PAYLOAD_FULL - plen} trailing zero byte'),
        (2, 1, 'INCOMPAT', str(raw[2]), 'hdr', 'no flags set: the frame is unsigned'),
        (3, 1, 'COMPAT', str(raw[3]), 'hdr', ''),
        (4, 1, 'SEQ', str(raw[4]), 'hdr', 'counts frames, so the board can see a loss'),
        (5, 1, 'SYSID', str(raw[5]), 'who', 'who is speaking: the Pi'),
        (6, 1, 'COMPID', str(raw[6]), 'who',
         'ONBOARD_COMPUTER — the id the board\'s link failsafe listens for'),
        (7, 3, 'MSGID', str(msgid), 'hdr', 'COMMAND_LONG, little-endian'),
    ]
    labels = ['p1 · type', 'p2 · seconds', 'p3 · speed', 'p4', 'p5 · timeout', 'p6', 'p7']
    for i, label in enumerate(labels):
        off = HEADER + 4 * i
        note = move_names.get(int(f[i]), '') if i == 0 else ''
        out.append((off, 4, label, f'{f[i]:g}', 'verb' if i < 3 else 'idle', note))
    out += [
        (38, 2, 'COMMAND', str(f[7]), 'cmd', 'CMD_SROT_MOVE — the one custom command'),
        (40, 1, 'TGT SYS', str(f[8]), 'who', 'the board'),
        (41, 1, 'TGT COMP', str(f[9]), 'who', 'its autopilot'),
    ]
    out = [r for r in out if r[0] < HEADER + plen]   # past LEN is trimmed
    crc_at = HEADER + plen
    out.append((crc_at, 2, 'CRC', f'0x{int.from_bytes(raw[crc_at:crc_at + 2], "little"):04X}',
                'hdr', 'checksum, seeded by the message definition itself'))
    return out


def render(raw: bytes, sp) -> str:
    rows = fields(raw, sp)
    e = html.escape
    cells = ''.join(
        f'<div class="fld fld--{role}" style="--n:{n}">'
        f'<span class="off">{off}</span>'
        f'<span class="hex">{"".join(f"<i>{raw[k]:02X}</i>" for k in range(off, off + n))}</span>'
        f'<b>{e(name)}</b><em>{e(val)}</em></div>'
        for off, n, name, val, role, _ in rows)
    key = ''.join(
        f'<tr class="fld--{role}"><td><b>{e(name)}</b></td><td class="v">{e(val)}</td>'
        f'<td>{e(note)}</td></tr>'
        for _, _, name, val, role, note in rows if note)
    return (
        f'  <figure class="wire" aria-label="The {len(raw)}-byte MAVLink 2 frame for {VERB}">\n'
        f'   <div class="wire__cmd"><span class="silk">the operator types</span>'
        f'<code>mongla {VERB} {KW["duration"]:g} --gain {KW["gain"]}</code>'
        f'<span class="silk">the cable carries · {len(raw)} bytes</span></div>\n'
        f'   <div class="wire__bytes">{cells}</div>\n'
        f'   <table class="wire__key"><tbody>{key}</tbody></table>\n'
        f'   <figcaption>Every byte above is generated by <code>tools/wire_frame.py</code> from '
        f'the project\'s own verb table, packed by pymavlink exactly as the manager sends it, '
        f'then decoded back &mdash; the tool fails if the frame does not round-trip.</figcaption>\n'
        f'  </figure>')


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--check', action='store_true', help='exit 1 if the page has drifted')
    args = ap.parse_args()
    raw, sp = build_frame()
    rc = splice('wire', render(raw, sp), args.check)
    if rc == 0:
        print(f'{VERB}: {len(raw)} bytes  {raw.hex(" ")}')
    return rc


if __name__ == '__main__':
    sys.exit(main())
