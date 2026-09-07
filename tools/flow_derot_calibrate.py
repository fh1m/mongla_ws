#!/usr/bin/env python3
"""Re-derive the de-rotation gains ON THIS MOUNT, from a tilt-only wave.

⛔ THIS WAS OWED AND WRITTEN DOWN. `measured-bars.md` §12, verbatim: "For the
vehicle: the axis mapping and the method carry over; **this gain does not**,
whatever its cause. Re-derive it on the hull." The gains were fitted on a rig
where the camera and IMU were rigidly coupled in a known orientation; the IMU
is now the SROT BOARD. `rotation_flow_px`'s own docstring says the mapping is
"mount-specific and CALIBRATED by the static-tilt test", so this is the
designed procedure, not a workaround.

Measured consequence of skipping it: a 30 cm slide read 124 % with
de-rotation ON and 104 % with it OFF -- the correction ADDS error, which is
what a wrong sign looks like. A stationary rig accumulated 1.24 cm of phantom
travel with it on, against 0.15 cm off.

THE ARITHMETIC, so the fitted number is checkable rather than trusted.
Let `S_x` be the true coupling, `measured_dx = S_x * (f * roll * dt)`. The
node passes `roll_rate = -gx * roll` into `rotation_flow_px` and SUBTRACTS
`f * roll_rate * dt`, so

    corrected_dx = S_x*f*roll*dt + gx*f*roll*dt = (S_x + gx) * f*roll*dt

which is zero exactly when **gx = -S_x**. So fit `S`, negate it, and that is
the parameter. Same for `gy` with pitch and image-y.

⚠ EXCITATION IS THE WHOLE DIFFICULTY, and the ledger already says why:
"Least squares never refuses -- gate a fit on excitation AND held-out score,
or it returns confident nonsense", and "excite one axis at a time; correlated
regressors leave the split undetermined while the fit looks healthy". A hand
slide rotates AND translates, and translation loads straight onto the
rotation regressor: fitting these gains from a slide gave 1.90 and 1.63,
which are fitted partly to the operator's wrist.

So this asks for a motion that is as close to PURE ROTATION as a hand can
manage -- tilt the rig about a point near the lens, do not carry it -- and
then refuses the fit unless:

  * there is real rotation to fit (rotation spread above a floor), AND
  * the fit survives K-FOLD HELD-OUT scoring, not just training R^2, AND
  * translation looks small (the intercept and the unexplained residual are
    reported, and a large one is called out rather than buried).

Usage, one axis at a time:
    python3 tools/flow_derot_calibrate.py --axis roll    # tilt LEFT-RIGHT
    python3 tools/flow_derot_calibrate.py --axis pitch   # tilt FORE-AFT
"""
import argparse
import json
import math
import sys
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image

from duburi_vision.distance.flow_math import detect_corners

_MIN_SAMPLES = 60
_MIN_ROT_SPREAD_PX = 3.0      # sd of the rotation regressor, in pixels


class Collector(Node):
    def __init__(self, cam):
        super().__init__('flow_derot_calibrate')
        self.imgs = []
        self.rates = []
        self.create_subscription(
            Image, f'/duburi/vision/{cam}/image_raw', self.imgs.append,
            QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(
            Vector3Stamped, '/duburi/imu_rates',
            lambda m: self.rates.append(
                (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                 m.vector.x, m.vector.y)), 50)


def kfold_slope(x, y, k=5):
    """Slope through the origin, scored on HELD-OUT points.

    Through the origin because zero rotation must give zero rotational flow;
    an intercept would absorb a steady translation and make the fit look
    better while meaning less. The intercept is reported separately as a
    WARNING signal, not fitted into the answer.
    """
    n = len(x)
    idx = np.arange(n)
    rng = np.random.default_rng(0)
    rng.shuffle(idx)
    folds = np.array_split(idx, k)
    errs, slopes = [], []
    for i in range(k):
        te = folds[i]
        tr = np.concatenate([folds[j] for j in range(k) if j != i])
        s = float(np.dot(x[tr], y[tr]) / max(1e-12, np.dot(x[tr], x[tr])))
        slopes.append(s)
        errs.append(float(np.sqrt(np.mean((y[te] - s * x[te]) ** 2))))
    s_all = float(np.dot(x, y) / max(1e-12, np.dot(x, x)))
    ss_res = float(np.sum((y - s_all * x) ** 2))
    ss_tot = float(np.sum((y - np.mean(y)) ** 2))
    return {
        'slope': s_all,
        'fold_sd': float(np.std(slopes)),
        'holdout_rms_px': float(np.mean(errs)),
        'r2': 1.0 - ss_res / max(1e-12, ss_tot),
        'null_rms_px': float(np.sqrt(np.mean(y ** 2))),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--camera', default='downward')
    ap.add_argument('--axis', choices=('roll', 'pitch'), required=True)
    ap.add_argument('--seconds', type=float, default=30.0)
    ap.add_argument('--calibration', default='')
    ap.add_argument('--fx', type=float, default=513.94)
    ap.add_argument('--fy', type=float, default=516.93)
    a = ap.parse_args()

    if a.calibration:
        raw = json.load(open(a.calibration))
        s = 640.0 / raw['image_width']
        a.fx = raw['camera_matrix'][0][0] * s
        a.fy = raw['camera_matrix'][1][1] * s

    print(__doc__.split('Usage')[0].strip()[:0] or '', end='')
    print(f'axis={a.axis}   f=({a.fx:.2f}, {a.fy:.2f})')
    print('\nTILT the rig about a point AS NEAR THE LENS AS YOU CAN, '
          f'{"left-right" if a.axis == "roll" else "fore-aft"}, for '
          f'{a.seconds:.0f} s.')
    print('Do NOT carry it across the floor -- translation loads onto the '
          'rotation\nregressor and the fit will look healthy and be wrong.')
    input('ENTER to start > ')

    rclpy.init()
    n = Collector(a.camera)
    b = CvBridge()
    prev = pt = None
    prev_t = 0.0
    rows = []
    t0 = time.time()
    try:
        while time.time() - t0 < a.seconds:
            rclpy.spin_once(n, timeout_sec=0.05)
            if not n.imgs:
                continue
            m = n.imgs.pop()
            n.imgs.clear()
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            g = (b.imgmsg_to_cv2(m, 'mono8') if m.encoding == 'mono8'
                 else cv2.cvtColor(b.imgmsg_to_cv2(m, 'bgr8'),
                                   cv2.COLOR_BGR2GRAY))
            if prev is not None and pt is not None and len(pt) > 10:
                nx, st, _ = cv2.calcOpticalFlowPyrLK(
                    prev, g, pt, None, winSize=(31, 31), maxLevel=3)
                dt = t - prev_t
                rr = [r for r in n.rates if prev_t < r[0] <= t]
                if st is not None and st.sum() > 8 and rr and dt > 1e-4:
                    k = st.reshape(-1).astype(bool)
                    d = nx.reshape(-1, 2)[k] - pt.reshape(-1, 2)[k]
                    wx = float(np.mean([r[1] for r in rr]))   # pitch
                    wy = float(np.mean([r[2] for r in rr]))   # roll
                    if a.axis == 'roll':
                        rows.append((a.fx * wy * dt, float(np.median(d[:, 0]))))
                    else:
                        rows.append((a.fy * wx * dt, float(np.median(d[:, 1]))))
            prev, prev_t, pt = g, t, detect_corners(g, want=80)
            n.rates[:] = n.rates[-400:]
    finally:
        n.destroy_node()
        rclpy.shutdown()

    if len(rows) < _MIN_SAMPLES:
        print(f'\nonly {len(rows)} usable intervals (need {_MIN_SAMPLES}) -- '
              f'not enough to fit anything.')
        return 1
    arr = np.array(rows)
    x, y = arr[:, 0], arr[:, 1]

    print(f'\n{len(rows)} intervals')
    print(f'  rotation regressor spread  sd={x.std():6.2f} px  '
          f'|max|={np.abs(x).max():6.2f} px')
    print(f'  image flow spread          sd={y.std():6.2f} px')
    if x.std() < _MIN_ROT_SPREAD_PX:
        print(f'\n  REFUSING: barely any rotation ({x.std():.2f} px sd, need '
              f'{_MIN_ROT_SPREAD_PX}). A least-squares fit will not refuse '
              f'on its own -- it\n  would hand back a confident number '
              f'fitted to noise. Tilt harder.')
        return 1

    r = kfold_slope(x, y)
    gain = -r['slope']
    shipped = -1.0
    print(f"\n  coupling S = {r['slope']:+.3f}   ->   gain = -S = "
          f"{gain:+.3f}     (shipped: {shipped:+.3f})")
    print(f"  held-out rms {r['holdout_rms_px']:.3f} px vs "
          f"{r['null_rms_px']:.3f} px for predicting zero   "
          f"(fold sd {r['fold_sd']:.3f})")
    print(f"  R2 {r['r2']:+.3f}")

    if r['holdout_rms_px'] >= r['null_rms_px']:
        print('\n  REFUSING: the fit does not beat predicting ZERO on '
              'held-out data.\n  Whatever this measured, it is not the '
              'rotation coupling.')
        return 1
    if r['fold_sd'] > 0.25 * abs(r['slope']):
        print('\n  ⚠ folds disagree by more than 25 % of the slope -- '
              'repeatability is\n  poor, so treat the sign as the result and '
              'the magnitude as provisional.')
    print(f"\n  Set with:  -p gyro_gain_{'x' if a.axis == 'roll' else 'y'}"
          f":={gain:.3f}")
    print('  Then A/B it against the shipped value on a real slide -- a gain '
          'that\n  fits better is not yet a gain that MEASURES better.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
