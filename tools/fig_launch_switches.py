#!/usr/bin/env python3
"""Draw which processes each bringup switch starts, from the launch files themselves.

Every dot is computed by rendering `bringup.launch.py` (tools/launch_render.py,
the same rendering `test_every_switch_combination_renders` checks), so the
picture cannot disagree with the launch. A switch that changes no process
only changes a parameter inside one, and is listed beside the matrix.

    source install/setup.bash && python3 tools/fig_launch_switches.py
        -> docs/imgs/readme/launch-switches.webp
"""
from __future__ import annotations

import io
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt                       # noqa: E402
from PIL import Image                                 # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
from launch_render import default_of, render, source_tree, switches  # noqa: E402

OUT = ROOT / 'docs' / 'imgs' / 'readme' / 'launch-switches.webp'
INK, MUTED, GRID, BG, ACCENT = '#1b1f24', '#6b7280', '#e5e7eb', '#ffffff', '#2563eb'


def table():
    """(rows, columns, quiet): each row a launch line and the processes it starts."""
    sw = switches()
    rows = [('defaults', set(render({})))]
    with_vision = {'vision': 'true'}
    vis = set(render(with_vision))
    rows.append(('vision:=true', vis))
    quiet = []
    for k in sorted(sw):
        if k == 'vision':
            continue
        for v in sw[k]:
            if v == default_of(k):
                continue
            got = set(render({**with_vision, k: v}))
            if got != vis:
                rows.append((f'vision:=true {k}:={v}', got))
            else:
                quiet.append(f'{k}:={v}')
    cols = sorted(set().union(*(r for _, r in rows)),
                  key=lambda c: (c.split('/')[0] != 'mongla_manager', c.split('/')[0], c))
    return rows, cols, quiet


def draw(rows, cols, quiet):
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 10})
    fig = plt.figure(figsize=(13.5, 0.40 * len(rows) + 2.6), dpi=160)
    fig.patch.set_facecolor(BG)
    ax = fig.add_axes([0.25, 0.24, 0.46, 0.62])
    ax.set_facecolor(BG)
    for i, (_, started) in enumerate(rows):
        for j, c in enumerate(cols):
            on = c in started
            ax.scatter(j, i, s=90 if on else 14, color=ACCENT if on else GRID,
                       zorder=3, ec=BG, lw=1)
    ax.set_xticks(range(len(cols)))
    ax.set_xticklabels([c.split('/')[1] for c in cols], rotation=40, ha='right',
                       color=INK, fontsize=9)
    ax.set_yticks(range(len(rows)))
    ax.set_yticklabels([r for r, _ in rows], color=INK, fontsize=9.5, family='DejaVu Sans Mono')
    ax.invert_yaxis()
    ax.set_xlim(-0.6, len(cols) - 0.4)
    ax.tick_params(length=0)
    for s in ax.spines.values():
        s.set_visible(False)
    ax.grid(color=GRID, lw=0.6)
    ax.set_axisbelow(True)
    fig.text(0.02, 0.95, 'What each bringup switch starts', color=INK, fontsize=14,
             weight='bold', va='top')
    fig.text(0.02, 0.905, 'ros2 launch mongla_manager bringup.launch.py <line>   '
             '-- each dot is a process the launch file starts, rendered from the source',
             color=MUTED, fontsize=9.5, va='top')
    fig.text(0.745, 0.86, f'{len(quiet)} switches start nothing new;\n'
             'they change a parameter\ninside a running node:', color=INK,
             fontsize=9.5, va='top')
    fig.text(0.745, 0.72, '\n'.join(quiet), color=MUTED, fontsize=8.4, va='top',
             family='DejaVu Sans Mono', linespacing=1.35)
    buf = io.BytesIO()
    fig.savefig(buf, format='png', facecolor=BG)
    plt.close(fig)
    buf.seek(0)
    OUT.parent.mkdir(parents=True, exist_ok=True)
    Image.open(buf).save(OUT, 'WEBP', quality=90, method=6)
    return OUT


def main() -> int:
    with source_tree():
        rows, cols, quiet = table()
    print(draw(rows, cols, quiet), f'{len(rows)} lines, {len(cols)} processes, {len(quiet)} parameter-only')
    return 0


if __name__ == '__main__':
    sys.exit(main())
