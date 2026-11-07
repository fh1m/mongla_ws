#!/usr/bin/env python3
"""Draw Kutum as the stack it is: six layers, what each one holds, and which
file in kutum/ owns it.

The ownership column is read from the scripts themselves (a layer is owned by
the file that installs it), so a step moved between scripts moves here too.
The measured column is the machine as kutum/doctor.sh read it on the date
below; rerun the doctor and update SNAPSHOT when the machine changes.

    python3 tools/fig_kutum.py  ->  docs/imgs/readme/kutum-stack.webp
"""
from __future__ import annotations

import io
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt                       # noqa: E402
from matplotlib.patches import FancyBboxPatch         # noqa: E402
from PIL import Image                                 # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
KUTUM = ROOT / 'kutum'
OUT = ROOT / 'docs' / 'imgs' / 'readme' / 'kutum-stack.webp'
INK, MUTED, GRID, BG, ACCENT = '#1b1f24', '#6b7280', '#e5e7eb', '#ffffff', '#2563eb'
WARM = '#b45309'

MEASURED_ON = '2026-11-07'
# (layer, what is there, the marker in a kutum/ file that installs it)
SNAPSHOT = [
    ('Mongla',  '6 packages, clean build 44.4 s; a stamp records which commit',
     ('deploy.sh', 'colcon build')),
    ('Python',  'numpy 1.26.4 and cv2 from apt, OpenBLAS, pinned pip, hailo_platform',
     ('setup.sh', 'requirements.txt')),
    ('ROS 2',   'Jazzy, sourced the same way in zsh and bash',
     ('env.sh', '/opt/ros/jazzy/setup.')),
    ('Hailo',   'driver (DKMS), firmware and HailoRT 4.24.0, from public source',
     ('setup.sh', 'install_dkms')),
    ('Ubuntu',  '24.04.4, kernel 6.8 raspi; governor pinned; udev names the cameras',
     ('setup.sh', 'udevadm control --reload-rules')),
    ('Silicon', 'Raspberry Pi 5, 4 GB  +  Hailo-8 on PCIe Gen 3 x1',
     (None, 'a screwdriver')),
]


def owner(file: str | None, marker: str) -> str:
    """The file, if it really contains the step; refuses to draw a lie."""
    if file is None:
        return marker
    text = (KUTUM / file).read_text()
    if marker not in text:
        raise SystemExit(f'kutum/{file} no longer contains {marker!r}: '
                         'update SNAPSHOT before drawing')
    return f'kutum/{file}'


def draw() -> Path:
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 10})
    fig = plt.figure(figsize=(12.5, 6.3), dpi=160)
    fig.patch.set_facecolor(BG)
    ax = fig.add_axes((0.02, 0.10, 0.96, 0.72))
    ax.set_xlim(0, 100)
    ax.set_ylim(0, len(SNAPSHOT))
    ax.axis('off')
    for i, (layer, what, (f, marker)) in enumerate(reversed(SNAPSHOT)):
        y = i + 0.12
        ax.add_patch(FancyBboxPatch((0.5, y), 73, 0.76, boxstyle='round,pad=0,rounding_size=0.12',
                                    fc='#f8fafc', ec=GRID, lw=1))
        ax.text(2.5, y + 0.38, layer, color=INK, fontsize=12.5, weight='bold', va='center')
        ax.text(16, y + 0.38, what, color=INK, fontsize=10, va='center')
        ax.annotate('', xy=(78.5, y + 0.38), xytext=(74.2, y + 0.38),
                    arrowprops=dict(arrowstyle='-', color=GRID, lw=1.2))
        ax.text(79, y + 0.38, owner(f, marker), color=ACCENT, fontsize=10.5,
                family='DejaVu Sans Mono', va='center')
    fig.text(0.02, 0.95, 'Kutum, layer by layer', color=INK, fontsize=15, weight='bold', va='top')
    fig.text(0.02, 0.895, 'Each layer is put there by one file in kutum/, except the bottom one. '
             'Measured on the vehicle ' + MEASURED_ON + '.',
             color=MUTED, fontsize=10, va='top')
    fig.text(0.80, 0.83, 'owned by', color=MUTED, fontsize=9.5, va='bottom')
    fig.text(0.02, 0.045, 'kutum/doctor.sh reads every layer back, read-only.   '
             'Nothing here grades the board, the cameras or the models: '
             'bringup_check --srot does.', color=WARM, fontsize=9.5)
    buf = io.BytesIO()
    fig.savefig(buf, format='png', facecolor=BG)
    plt.close(fig)
    buf.seek(0)
    OUT.parent.mkdir(parents=True, exist_ok=True)
    Image.open(buf).save(OUT, 'WEBP', quality=90, method=6)
    return OUT


if __name__ == '__main__':
    print(draw())
    sys.exit(0)
