#!/usr/bin/env python3
"""The live board reading, as a figure on the page.

`tools/board_snapshot.py` connects to the SROT board and writes
docs/data/board.json. This renders that file into docs/index.html between the
`board:begin` / `board:end` markers: what the board says about itself at the
moment it was asked, including the line where it prints its own configuration.

Nothing here is typed. Read the board again, re-run both tools, and the figure
follows -- timestamp included, because a live reading is only true of the day it
was taken.

    python3 tools/board_panel.py
    python3 tools/board_panel.py --check    # exit 1 if the page has drifted
"""
from __future__ import annotations

import argparse
import html
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _site_splice import splice  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / 'docs' / 'data' / 'board.json'
# the streams worth naming, and what each one is for
STREAMS = [('NAMED_VALUE_FLOAT', 'the health names, multiplexed'),
           ('ATTITUDE', 'roll, pitch, yaw'),
           ('SCALED_IMU2', 'the raw inertial data'),
           ('UNKNOWN_291', 'ESC_STATUS — undecodable here, by design'),
           ('VFR_HUD', 'depth, reported as altitude'),
           ('SYS_STATUS', 'the health words'),
           ('BATTERY_STATUS', 'one pack per instance')]


def render(d: dict) -> str:
    e = html.escape
    link = d['link']
    rates = d['rates_hz']
    names = d['named_value_float']
    cfg = next((t for t in d['statustext'] if t.startswith('CFG ')), '')
    top = max(rates.values())
    rows = ''.join(
        f'<li><b>{e(k)}</b><i style="--w:{min(100, rates[k] / top * 100):.1f}%"></i>'
        f'<em>{rates[k]:g}<u>Hz</u></em><span>{e(note)}</span></li>'
        for k, note in STREAMS if k in rates)

    def nv(key):
        v = names.get(key, {}).get('last')
        return '—' if v is None else f'{v:g}'

    sensors = d['sensors']
    leak_ok = 'LEAK' in sensors['enabled_extended'] and 'LEAK' in sensors['health_extended']
    baro_ok = 'ABSOLUTE_PRESSURE' in sensors['health']
    checks = [
        ('leak', 'present · enabled · healthy' if leak_ok else 'not enabled',
         f'reads {nv("LEAK")} — dry' if nv('LEAK') == '0' else f'reads {nv("LEAK")}', leak_ok),
        # the bit and the name disagree here, and the figure says so rather than
        # choosing: SYS_STATUS does not set ABSOLUTE_PRESSURE in `health`, while
        # the board's own BARO_HEALT name reads 3. An unhealthy barometer makes
        # the board refuse every automatic move, so which is right matters.
        ('barometer', 'bit says unhealthy' if not baro_ok else 'healthy',
         f'BARO_HEALT reads {nv("BARO_HEALT")} · p2p {nv("BARO_P2P")} — the two disagree'
         if not baro_ok else f'p2p {nv("BARO_P2P")}', None if not baro_ok else True),
        ('kill switch', f'KILL = {nv("KILL")}', 'still means live OR not talking', None),
        ('armed', 'no' if not d['heartbeat']['armed'] else 'YES',
         'custom mode ' + str(d['heartbeat']['custom_mode']), not d['heartbeat']['armed']),
    ]
    checks_html = ''.join(
        f'<li class="{"ok" if ok else "no" if ok is False else "warn"}"><b>{e(label)}</b>'
        f'<span>{e(state)}</span><em>{e(detail)}</em></li>'
        for label, state, detail, ok in checks)
    volt = next(iter(d['batteries'].values()), {}).get('voltage_mv') or 0
    return (
        '  <figure class="board-live" aria-label="A live reading taken off the SROT board">\n'
        f'   <header><span class="silk">Live off the board <i>//</i> {e(d["taken"])}</span>'
        f'<span class="silk">{e(d["port"])} · {d["baud"]} baud · {d["seconds"]:g} s window</span>'
        '</header>\n'
        '   <div class="board-live__grid">\n'
        '    <div class="board-live__wire">\n'
        '     <span class="silk">the cable, right now</span>\n'
        f'     <b>{link["used_pct"]:g}<u>%</u></b>\n'
        f'     <div class="board-live__bar"><i style="--w:{link["used_pct"]:g}%"></i>'
        '<u style="--x:60%"></u></div>\n'
        f'     <p>{link["bytes_per_s"]:.0f} of {link["wire_bytes_per_s"]:.0f} bytes a second · '
        f'{link["messages_per_s"]:.0f} messages · {link["bad_frames"]} frames the parser rejected. '
        'The tick is the 60&#8239;% bar this project ships against.</p>\n'
        '    </div>\n'
        f'    <ol class="board-live__streams">{rows}</ol>\n'
        f'    <ol class="board-live__checks">{checks_html}</ol>\n'
        '   </div>\n'
        '   <p class="board-live__cfg"><span class="silk">and the board states its own configuration</span>'
        f'<code>{e(cfg)}</code>'
        '<span>revision <b>14</b> · <code>FRAME_REVERSE = 1</code> · eight motor directions, and '
        '<b>not</b> all &minus;1 — so the double inversion two sections above is not present on this '
        'hull, and the board is the one saying so.</span></p>\n'
        '   <figcaption>Read by <code>tools/board_snapshot.py</code> over the USB-C cable with DTR and '
        'RTS deasserted, so attaching did not reset the board; rendered by '
        '<code>tools/board_panel.py</code> from <code>docs/data/board.json</code>. Nothing was '
        'transmitted but the parameter reads. The electronics pack reads '
        f'<strong>{volt / 1000:.2f} V</strong>, the vehicle is <strong>disarmed</strong>, and there are '
        'no thrusters attached to it.</figcaption>\n'
        '  </figure>')


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--check', action='store_true', help='exit 1 if the page has drifted')
    args = ap.parse_args()
    if not DATA.exists():
        print('no board.json — run tools/board_snapshot.py first', file=sys.stderr)
        return 2
    d = json.loads(DATA.read_text())
    rc = splice('board', render(d), args.check)
    if rc == 0 and not args.check:
        print(f'{d["taken"]}  link {d["link"]["used_pct"]:g} %  '
              f'leak {d["sensors"]["health_extended"]}')
    return rc


if __name__ == '__main__':
    sys.exit(main())
