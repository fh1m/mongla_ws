#!/usr/bin/env python3
"""What do the camera's OWN automatic loops do to our pixels?

⛔ THE QUESTION NOBODY ASKS. Every measurement in this repo -- detector
confidence, XFeat inliers, the turbidity proxy, bank references -- is computed
on pixels the CAMERA decided the values of. If auto white balance and auto
exposure are running, those pixels change for reasons that have nothing to do
with the scene, and every downstream number inherits it:

  * XFeat descriptors are intensity-derived, so a gain change is an
    appearance change to a bank reference that never moved;
  * the turbidity proxy would be measuring the CAMERA's decisions rather than
    the water;
  * `exposure_dynamic_framerate` lets the camera silently drop frame rate in
    dim water -- and the lock ladder's rungs are sized in SECONDS, so a
    halved rate halves how many detections fit inside coast_s.

Read live from the vehicle on 2026-09-25: white_balance_automatic=1,
auto_exposure=3 (Aperture Priority), exposure_dynamic_framerate=1.

This measures the cost rather than assuming it: hold the camera still, look at
one static scene, and see how much the pixels move on their own.
"""
from __future__ import annotations

import argparse
import subprocess
import sys
import time

import cv2
import numpy as np


def ctl(dev: str, name: str, value=None):
    if value is None:
        out = subprocess.run(['v4l2-ctl', '-d', dev, f'--get-ctrl={name}'],
                             capture_output=True, text=True)
        return out.stdout.strip()
    subprocess.run(['v4l2-ctl', '-d', dev, f'--set-ctrl={name}={value}'],
                   capture_output=True, text=True)
    return None


def sample(dev: str, n: int, warm: float) -> dict:
    cap = cv2.VideoCapture(dev)
    if not cap.isOpened():
        return {}
    t0 = time.monotonic()
    while time.monotonic() - t0 < warm:
        cap.grab()
    means, rb, times = [], [], []
    last = None
    for _ in range(n):
        ok, im = cap.read()
        if not ok:
            continue
        now = time.monotonic()
        if last is not None:
            times.append(now - last)
        last = now
        b, g, r = (float(im[:, :, i].mean()) for i in range(3))
        means.append((b + g + r) / 3.0)
        rb.append(r / max(b, 1e-6))
    cap.release()
    if not means:
        return {}
    m, q = np.array(means), np.array(rb)
    out = {'luma_mean': m.mean(), 'luma_sd': m.std(),
           'luma_ptp': m.max() - m.min(),
           'rb_mean': q.mean(), 'rb_sd': q.std(), 'rb_ptp': q.max() - q.min(),
           'n': len(m)}
    if times:
        t = np.array(times)
        out['fps_mean'] = 1.0 / t.mean()
        out['fps_min'] = 1.0 / t.max()
    return out


def show(tag: str, s: dict) -> None:
    if not s:
        print(f'{tag:<10} no frames')
        return
    line = (f'{tag:<10} luma {s["luma_mean"]:6.2f} sd {s["luma_sd"]:5.3f} '
            f'ptp {s["luma_ptp"]:5.2f}   R/B {s["rb_mean"]:.4f} '
            f'sd {s["rb_sd"]:.4f} ptp {s["rb_ptp"]:.4f}')
    if 'fps_mean' in s:
        line += f'   fps {s["fps_mean"]:5.1f} (min {s["fps_min"]:5.1f})'
    print(line)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--dev', required=True)
    ap.add_argument('--frames', type=int, default=120)
    ap.add_argument('--warm', type=float, default=2.0)
    args = ap.parse_args()

    print('⚠ Keep the camera STILL and the scene UNCHANGED for this run.')
    print('   Anything that moves below is the camera arguing with itself.\n')

    before = {k: ctl(args.dev, k) for k in
              ('white_balance_automatic', 'auto_exposure',
               'exposure_dynamic_framerate', 'exposure_time_absolute', 'gain')}
    print('as found:', {k: v.split(': ')[-1] if v else '?'
                        for k, v in before.items()}, '\n')

    auto = sample(args.dev, args.frames, args.warm)
    show('AUTO', auto)

    # Pin everything the camera is allowed to decide.
    ctl(args.dev, 'white_balance_automatic', 0)
    ctl(args.dev, 'exposure_dynamic_framerate', 0)
    ctl(args.dev, 'auto_exposure', 1)            # 1 = Manual Mode
    time.sleep(0.5)
    fixed = sample(args.dev, args.frames, args.warm)
    show('FIXED', fixed)

    # Leave the camera as it was found: this is a measurement, not a change.
    for k, v in before.items():
        if v and 'value' not in v:
            continue
    ctl(args.dev, 'white_balance_automatic', 1)
    ctl(args.dev, 'auto_exposure', 3)
    ctl(args.dev, 'exposure_dynamic_framerate', 1)

    if auto and fixed:
        print()
        for name, key in (('luma drift (peak-to-peak)', 'luma_ptp'),
                          ('colour drift R/B (p2p)', 'rb_ptp')):
            a, b = auto[key], fixed[key]
            print(f'{name:<28} auto {a:8.4f}   fixed {b:8.4f}   '
                  f'{"x%.1f" % (a / b) if b > 1e-9 else "--"}')
        if auto['rb_ptp'] > max(3 * fixed['rb_ptp'], 0.01):
            print('\n⭐ AWB IS MOVING THE PIXELS on a static scene. Every '
                  'intensity-derived measurement downstream -- XFeat '
                  'descriptors, the turbidity proxy, bank references -- is '
                  'carrying that drift.')
        else:
            print('\n⚠ No large AWB drift on THIS scene. A static indoor '
                  'view is the easy case; the claim needs water.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
