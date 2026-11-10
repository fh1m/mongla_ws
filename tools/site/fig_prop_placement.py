#!/usr/bin/env python3
"""Draw why `prop_placement` adds the hull's error once: one pass, top view.

Every number on the figure is computed here by the shipped `place()` -- the
pass drawn, and the two coverage figures over 2000 passes with the same
settings as `test_prop_placement` -- so the picture cannot drift from the code.

    PYTHONPATH=src/mongla_localization python3 tools/site/fig_prop_placement.py
        -> docs/imgs/readme/prop-placement.webp
"""
from __future__ import annotations

import io
import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt                       # noqa: E402
import numpy as np                                    # noqa: E402
from matplotlib.patches import Ellipse                # noqa: E402
from PIL import Image                                 # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src' / 'mongla_localization'))
from mongla_localization.prop_placement import (CHI2_2DOF_95, Sighting,  # noqa: E402
                                                place)

OUT = ROOT / 'docs' / 'imgs' / 'readme'
TRUTH = np.array([6.0, 2.0])
HULL_SD, YAW_SD, RANGE_SD, BEAR_SD, N = 0.30, 1.5, 0.15, 1.0, 8

THEMES = {
    '':      dict(bg='#ffffff', ink='#1b1f24', muted='#6b7280', grid='#e5e7eb',
                  honest='#2563eb', naive='#d97706', track='#9ca3af'),
}


def one_pass(rng, hull_err=None):
    hull_err = rng.normal(0.0, HULL_SD, 2) if hull_err is None else hull_err
    yaw_err = rng.normal(0.0, YAW_SD)
    true_hulls, sightings = [], []
    for k in range(N):
        h = np.array([1.0 + 0.1 * k, 0.5 - 0.05 * k])
        d = TRUTH - h
        true_hulls.append(h)
        sightings.append(Sighting(
            hull_xy=tuple(h + hull_err),
            range_m=float(np.hypot(*d)) + rng.normal(0.0, RANGE_SD),
            range_sigma_m=RANGE_SD,
            bearing_deg=math.degrees(math.atan2(d[1], d[0])) + yaw_err
            + rng.normal(0.0, BEAR_SD)))
    return np.array(true_hulls), sightings


def coverage(trials=2000):
    rng = np.random.default_rng(7)
    hull_cov = np.eye(2) * HULL_SD ** 2
    hit = naive = 0
    for _ in range(trials):
        _, s = one_pass(rng)
        p = place(s, hull_cov_xy=hull_cov, yaw_sigma_deg=YAW_SD)
        est = np.array([p.x_m, p.y_m])
        for cov, is_naive in ((p.cov, False),
                              ((np.asarray(p.cov_noise) + hull_cov) / N, True)):
            d = TRUTH - est
            inside = float(d @ np.linalg.solve(np.asarray(cov), d)) <= CHI2_2DOF_95
            if is_naive:
                naive += inside
            else:
                hit += inside
    return hit / trials, naive / trials


def ellipse(ax, xy, cov, colour, lw, ls: object = '-'):
    vals, vecs = np.linalg.eigh(np.asarray(cov))
    k = math.sqrt(CHI2_2DOF_95)
    ang = math.degrees(math.atan2(vecs[1, 1], vecs[0, 1]))
    ax.add_patch(Ellipse(xy, 2 * k * math.sqrt(vals[1]), 2 * k * math.sqrt(vals[0]),
                         angle=ang, fill=False, ec=colour, lw=lw, ls=ls))


def draw(theme, cover, naive_cover):
    t = THEMES[theme]
    rng = np.random.default_rng(42)
    # A pass whose pose has drifted 0.35 m back and to port: big enough to see.
    true_hulls, s = one_pass(rng, hull_err=np.array([-0.30, 0.18]))
    p = place(s, hull_cov_xy=np.eye(2) * HULL_SD ** 2, yaw_sigma_deg=YAW_SD)
    est = np.array([p.x_m, p.y_m])
    naive = (np.asarray(p.cov_noise) + np.eye(2) * HULL_SD ** 2) / N

    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 11})
    fig, ax = plt.subplots(figsize=(10.0, 5.4), dpi=160)
    fig.patch.set_facecolor(t['bg'])
    ax.set_facecolor(t['bg'])
    reported = np.array([x.hull_xy for x in s])
    ax.plot(*true_hulls.T, color=t['track'], lw=2, solid_capstyle='round')
    ax.plot(*reported.T, color=t['ink'], lw=2, ls=(0, (4, 3)))
    pts = []
    for x in s:
        b = math.radians(x.bearing_deg)
        end = np.array(x.hull_xy) + x.range_m * np.array([math.cos(b), math.sin(b)])
        pts.append(end)
        ax.plot([x.hull_xy[0], end[0]], [x.hull_xy[1], end[1]],
                color=t['muted'], lw=0.7, alpha=0.55)
    pts = np.array(pts)
    ax.scatter(*pts.T, s=26, color=t['muted'], zorder=3, ec=t['bg'], lw=1)
    ellipse(ax, est, p.cov, t['honest'], 2.2)
    ellipse(ax, est, naive, t['naive'], 2.2, ls=(0, (2, 2)))
    ax.scatter(*TRUTH, marker='*', s=260, color=t['ink'], zorder=5, ec=t['bg'], lw=1.5)

    def note(xy, text, colour, at, ha='left'):
        ax.annotate(text, xy=xy, xytext=at, color=colour, ha=ha, va='center',
                    fontsize=10.5, arrowprops=dict(arrowstyle='-', color=colour, lw=0.8))

    note(TRUTH, 'where the prop is', t['ink'], (7.0, 1.55))
    note(true_hulls[-1], 'where the hull was', t['track'], (2.2, -0.15))
    note(reported[-1], 'where the filter said it was', t['ink'], (2.2, 0.95))
    lo = est - [0.0, math.sqrt(CHI2_2DOF_95 * np.linalg.eigvalsh(p.cov)[1])]
    note(lo, f'shipped 95 % ellipse: holds the truth\nin {cover * 100:.1f} % of 2000 passes',
         t['honest'], (6.6, 0.55))
    note(est + [0.0, 0.25], f'every error divided by {N}:\nholds it in {naive_cover * 100:.1f} %',
         t['naive'], (3.4, 3.0))
    ax.set_aspect('equal')
    ax.set_xlim(0.4, 9.0)
    ax.set_ylim(-0.5, 3.5)
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    for side in ('left', 'bottom'):
        ax.spines[side].set_color(t['grid'])
    ax.tick_params(colors=t['muted'], labelsize=9)
    ax.set_xlabel('pool x, m', color=t['muted'])
    ax.set_ylabel('pool y, m', color=t['muted'])
    ax.grid(color=t['grid'], lw=0.6)
    ax.set_axisbelow(True)
    ax.set_title(f'{N} sightings from one drifted pose: the drift does not average away',
                 color=t['ink'], loc='left', fontsize=13, pad=12)
    fig.tight_layout()
    buf = io.BytesIO()
    fig.savefig(buf, format='png', facecolor=t['bg'])
    plt.close(fig)
    buf.seek(0)
    out = OUT / f'prop-placement{theme}.webp'
    Image.open(buf).save(out, 'WEBP', quality=88, method=6)
    return out


def main() -> int:
    cover, naive_cover = coverage()
    for theme in THEMES:
        print(draw(theme, cover, naive_cover))
    print(f'coverage {cover:.3f}  naive {naive_cover:.3f}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
