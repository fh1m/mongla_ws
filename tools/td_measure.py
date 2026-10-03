#!/usr/bin/env python3
"""Measure the camera <-> gyro time offset `td` on the bench.

    tools/td_measure.py --self-test             # the scorer, on known truth
    tools/td_measure.py --record 45 --out run.npz   # live: manager must run
    tools/td_measure.py --analyse run.npz

⛔ WHY. `td` is the one timing term in the flow pipeline with no measurement
anywhere (measured-bars, "What this does NOT cover"). The flow node estimates
it online, but only logs every sixth accepted window and only after the gates
it ships with. This records the two series the node correlates -- image yaw
rate from the node's own planar rigid fit, and the board gyro as the manager
publishes it on `/mongla/imu_rates` -- and scores them offline.

THE TWO CLOCKS ARE THE NODE'S CLOCKS. Images carry the mailbox's kernel
capture stamp on the wall clock (`meta.stamp_wall`, what camera_node puts in
the header); gyro samples carry the board time mapped through `ClockMap`
(what the manager stamps). `td` is the offset between exactly those, so the
number is the one the flow node would use.

WHAT IT IS ALLOWED TO CALL A MEASUREMENT, written before the run:
  * the excitation gate passes (a changing rotation rate -- a steady turn is
    degenerate, Li & Mourikis);
  * every window of the run gives an estimate at or above the node's quality
    bar (`time_offset_min_quality`, 0.5);
  * the window estimates agree within SPREAD_MAX_MS of each other.
Otherwise it says NOT MEASURED and why. `--self-test` runs the same scoring
against a synthetic pair with an injected delay of known sign, so a bug in the
scorer cannot be mistaken for a property of the rig.

The camera is mounted however it is mounted: image yaw is correlated against
ALL THREE gyro axes and both signs, and the best match is reported -- which
doubles as a check of which board axis the optical axis lies along.
"""
from __future__ import annotations

import argparse
import pathlib
import sys
import time

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'src' / 'mongla_vision'))

from mongla_vision.flow.flow_timing import TimeOffset  # noqa: E402

QUALITY_MIN = 0.5            # the node's `time_offset_min_quality`
SPREAD_MAX_MS = 2.0          # window estimates must agree this well
WINDOW_S = 15.0
AXES = ('x (pitch rate)', 'y (roll rate)', 'z (yaw rate)')


# --------------------------------------------------------------------------- #
#  scoring
# --------------------------------------------------------------------------- #
def _estimate(img_t, img_v, gyr_t, gyr_v):
    est = TimeOffset(max_lag_s=0.20)
    for t, v in zip(img_t, img_v):
        est.add_image_yaw(float(t), float(v))
    for t, v in zip(gyr_t, gyr_v):
        est.add_gyro_yaw(float(t), float(v))
    td = est.estimate()
    return td, float(est.quality), float(est.peak_correlation)


def score(img_t, img_v, gyr_t, gyr_xyz):
    """Pick the gyro axis/sign the image matches, then window it.

    Returns a dict with the verdict and every number behind it."""
    best = None
    for k in range(3):
        for sign in (1.0, -1.0):
            td, q, peak = _estimate(img_t, sign * np.asarray(img_v),
                                    gyr_t, gyr_xyz[:, k])
            if td is None:
                continue
            if best is None or peak > best['peak']:
                best = dict(axis=k, sign=sign, td=td, quality=q, peak=peak)
    if best is None:
        return dict(measured=False, why='no axis gave an estimate -- not '
                    'enough rotation, or too little overlap')
    k, sign = best['axis'], best['sign']
    t0, t1 = max(img_t[0], gyr_t[0]), min(img_t[-1], gyr_t[-1])
    windows = []
    start = t0
    while start + WINDOW_S <= t1 + 1e-9:
        mi = (img_t >= start) & (img_t < start + WINDOW_S)
        mg = (gyr_t >= start) & (gyr_t < start + WINDOW_S)
        td, q, peak = _estimate(img_t[mi], sign * np.asarray(img_v)[mi],
                                gyr_t[mg], gyr_xyz[mg, k])
        windows.append(dict(start=start - t0, td=td, quality=q, peak=peak))
        start += WINDOW_S
    out = dict(best, windows=windows, axis_name=AXES[k])
    good = [w for w in windows if w['td'] is not None and w['quality'] >= QUALITY_MIN]
    if len(windows) < 2:
        out.update(measured=False, why=f'{len(windows)} window(s) of '
                   f'{WINDOW_S:.0f} s -- record longer to check repeatability')
    elif len(good) < len(windows):
        out.update(measured=False, why=f'{len(windows) - len(good)} of '
                   f'{len(windows)} windows below quality {QUALITY_MIN}')
    else:
        tds = [w['td'] * 1000.0 for w in good]
        spread = max(tds) - min(tds)
        out.update(spread_ms=spread)
        if spread > SPREAD_MAX_MS:
            out.update(measured=False, why=f'windows disagree by '
                       f'{spread:.2f} ms > {SPREAD_MAX_MS} ms')
        else:
            out.update(measured=True, why='')
    return out


def report(r) -> str:
    lines = []
    if 'axis' in r:
        lines.append(f"image yaw matches gyro {r['axis_name']}, sign "
                     f"{'+' if r['sign'] > 0 else '-'}  (peak {r['peak']:.2f})")
        lines.append(f"td over the whole run: {r['td'] * 1000:+.2f} ms "
                     f"(quality {r['quality']:.2f})")
        for w in r['windows']:
            td = 'none' if w['td'] is None else f"{w['td'] * 1000:+.2f} ms"
            lines.append(f"  window @{w['start']:5.1f}s: {td}  quality "
                         f"{w['quality']:.2f}  peak {w['peak']:.2f}")
    lines.append('MEASURED' if r['measured'] else f"NOT MEASURED: {r['why']}")
    if r.get('spread_ms') is not None:
        lines.append(f"window spread {r['spread_ms']:.2f} ms "
                     f"(bar {SPREAD_MAX_MS} ms)")
    return '\n'.join(lines)


# --------------------------------------------------------------------------- #
#  self-test: the scorer against a delay of known sign
# --------------------------------------------------------------------------- #
def self_test() -> int:
    """td > 0 means image stamps are LATE: an image stamped t shows t - td.
    Synthesise that, on the wrong-looking axis and sign, at the camera's real
    rate and jitter, and require the scorer to find axis, sign and td."""
    rng = np.random.default_rng(7)
    fails = 0
    for true_ms, axis, sign in ((+23.0, 0, -1.0), (-11.0, 2, +1.0), (+4.0, 1, +1.0)):
        dur = 45.0
        gyr_t = np.arange(0.0, dur, 0.02)                     # 50 Hz board
        def omega(t):
            return (0.6 * np.sin(2 * np.pi * 0.7 * t) + 0.35 * np.sin(2 * np.pi * 1.9 * t + 1.0)
                    + 0.2 * np.sin(2 * np.pi * 0.23 * t + 2.0))
        gyr = np.zeros((len(gyr_t), 3))
        gyr[:, axis] = omega(gyr_t) + rng.normal(0, 0.01, len(gyr_t))
        # The OTHER axes move too, as a hand wobble does -- independent
        # rotation of similar size, so the axis choice is actually tested
        # rather than decided by the excitation gate rejecting pure noise.
        for j, other in enumerate(sorted({0, 1, 2} - {axis})):
            f1, f2 = (0.41, 1.3) if j == 0 else (0.57, 2.3)
            gyr[:, other] = (0.5 * np.sin(2 * np.pi * f1 * gyr_t + 0.4 + j)
                             + 0.3 * np.sin(2 * np.pi * f2 * gyr_t + 1.7)
                             + rng.normal(0, 0.01, len(gyr_t)))
        img_t = np.cumsum(rng.normal(1 / 58.7, 0.0015, int(dur * 58.7)))
        img_t = img_t[img_t < dur]
        img_v = sign * omega(img_t - true_ms / 1000.0) + rng.normal(0, 0.03, len(img_t))
        r = score(img_t, img_v, gyr_t, gyr)
        ok = (r.get('axis') == axis and r.get('sign') == sign and r['measured']
              and abs(r['td'] * 1000 - true_ms) < 1.0)
        print(f"[self-test] injected {true_ms:+.1f} ms on axis {axis} sign "
              f"{sign:+.0f}: got axis {r.get('axis')} sign {r.get('sign')} "
              f"td {r['td'] * 1000 if r.get('td') is not None else float('nan'):+.2f} ms "
              f"measured={r['measured']} -> {'ok' if ok else 'FAIL'}")
        fails += not ok
    # And the refusal: a steady turn is degenerate and must NOT be measured.
    gyr_t = np.arange(0.0, 45.0, 0.02)
    gyr = np.zeros((len(gyr_t), 3))
    gyr[:, 2] = 0.4 + np.random.default_rng(1).normal(0, 0.005, len(gyr_t))
    img_t = np.arange(0.0, 45.0, 1 / 58.7)
    img_v = 0.4 + np.random.default_rng(2).normal(0, 0.005, len(img_t))
    r = score(img_t, img_v, gyr_t, gyr)
    ok = not r['measured']
    print(f"[self-test] steady turn: measured={r['measured']} -> {'ok' if ok else 'FAIL'}")
    fails += not ok
    print('[self-test] PASS' if not fails else f'[self-test] {fails} FAIL')
    return 1 if fails else 0


# --------------------------------------------------------------------------- #
#  live recording
# --------------------------------------------------------------------------- #
def record(seconds: float, out: str, device: str) -> int:
    import threading
    import cv2
    import rclpy
    from geometry_msgs.msg import Vector3Stamped
    from mongla_vision.cameras.v4l2_mailbox import V4L2MailboxCamera
    from mongla_vision.flow.flow_math import solve_planar_motion

    gyr = []
    rclpy.init()
    node = rclpy.create_node('td_measure')

    def on_rates(m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        gyr.append((t, m.vector.x, m.vector.y, m.vector.z))

    node.create_subscription(Vector3Stamped, '/mongla/imu_rates', on_rates, 50)
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()

    cam = V4L2MailboxCamera(device=device, width=1280, height=720, fps=60)
    img = []
    prev = prev_t = None
    t_end = time.monotonic() + seconds
    print(f'[td] recording {seconds:.0f} s -- wobble the rig about the '
          f'camera axis NOW (irregular, changing speed)')
    while time.monotonic() < t_end:
        frame, meta = cam.read()
        if frame is None or not meta.fresh:
            time.sleep(0.002)
            continue
        g = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        g = cv2.resize(g, (320, 180), interpolation=cv2.INTER_AREA)
        t = float(meta.stamp_wall)
        if prev is not None:
            pts = cv2.goodFeaturesToTrack(prev, 200, 0.01, 6)
            if pts is not None and len(pts) >= 12:
                nxt, st, _ = cv2.calcOpticalFlowPyrLK(prev, g, pts, None)
                pm = solve_planar_motion(pts.reshape(-1, 2), nxt.reshape(-1, 2),
                                         st.reshape(-1), t - prev_t,
                                         cx=160.0, cy=90.0)
                if pm.ok:
                    img.append((0.5 * (t + prev_t), pm.yaw_rate))
        prev, prev_t = g, t
    cam.close()
    node.destroy_node()
    rclpy.shutdown()
    if len(gyr) < 100 or len(img) < 100:
        print(f'[td] too little data: {len(gyr)} gyro, {len(img)} image samples '
              f'-- is the manager running and publishing /mongla/imu_rates?')
        return 1
    g = np.array(gyr)
    i = np.array(img)
    np.savez(out, img_t=i[:, 0], img_v=i[:, 1], gyr_t=g[:, 0], gyr_xyz=g[:, 1:4])
    print(f'[td] saved {out}: {len(i)} image, {len(g)} gyro samples')
    return analyse(out)


def analyse(path: str) -> int:
    d = np.load(path)
    r = score(d['img_t'], d['img_v'], d['gyr_t'], d['gyr_xyz'])
    print(report(r))
    return 0 if r['measured'] else 2


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--self-test', action='store_true')
    ap.add_argument('--record', type=float, metavar='SECONDS')
    ap.add_argument('--out', default='td_run.npz')
    ap.add_argument('--device', default='/dev/video0')
    ap.add_argument('--analyse', metavar='NPZ')
    a = ap.parse_args()
    if a.self_test:
        return self_test()
    if a.record:
        return record(a.record, a.out, a.device)
    if a.analyse:
        return analyse(a.analyse)
    ap.print_help()
    return 0


if __name__ == '__main__':
    sys.exit(main())
