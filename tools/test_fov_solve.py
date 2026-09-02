#!/usr/bin/env python3
"""Ground-truth verification for fov_solve.

Every metric in a calibration is internal to the optimizer, so a calibration
can be self-consistently wrong -- this camera has already produced fx = 835.7,
969.9 and 1011.2, each with a confident-looking residual. The only way to know
the pipeline recovers the RIGHT answer is to feed it data whose answer we
already know.

So: synthesise views by projecting a board with a KNOWN camera, add realistic
corner noise, and check what comes back.

  1. flat board       -> K recovered to within tolerance, and the held-out test
                         must NOT prefer RO. This is the anti-overfit control:
                         with no real shape to recover, RO's ~3N extra
                         parameters can only fit noise, and held-out error is
                         where that shows.
  2. bowed board      -> RO must win on held-out error AND recover the injected
                         bow to within tolerance. Confirms the win on real data
                         is geometry, not degrees of freedom.
  3. degenerate poses -> near-frontal views only must be flagged
                         UNDER-DETERMINED by the conditioning diagnostic, even
                         though the residual looks fine. This is the failure
                         that produced fx = 835.7 and it was invisible in RMS.

Run:  python3 test_fov_solve.py
"""
from __future__ import annotations

import numpy as np
import cv2

import fov_solve as fs

W, H = 1280, 720
COLS, ROWS, SQ = 8, 6, 0.025
RNG = np.random.default_rng(7)

# a plausible camera for this class of sensor
K_TRUE = np.array([[1010.0, 0, 641.0],
                   [0, 1012.0, 357.0],
                   [0, 0, 1.0]])
D_TRUE = np.array([-0.095, 0.130, -0.0008, 0.0006, -0.070])
NOISE_PX = 0.15   # corner localisation noise, realistic for cornerSubPix


def synth_views(bow_mm: float, n: int = 24, frontal_only: bool = False):
    """Project the board from n poses. `bow_mm` is a cylindrical curl in z."""
    obj = fs.board_points(COLS, ROWS, SQ).astype(np.float64)
    if bow_mm:
        # cylindrical: flat along u (columns), quadratic along v (rows) --
        # the shape a sheet of card takes when it has been stored rolled
        v = obj[:, 1] - obj[:, 1].mean()
        obj[:, 2] = (bow_mm / 1000.0) * (1 - (v / max(abs(v).max(), 1e-9)) ** 2)
        obj[:, 2] -= obj[:, 2].mean()
    ips = []
    for i in range(n):
        if frontal_only:
            rx, ry = RNG.uniform(-0.06, 0.06, 2)          # ~3 deg: degenerate
        else:
            rx, ry = RNG.uniform(-0.55, 0.55, 2)          # ~30 deg: healthy
        rz = RNG.uniform(-0.3, 0.3)
        rvec = np.array([rx, ry, rz])
        tvec = np.array([RNG.uniform(-0.05, 0.05),
                         RNG.uniform(-0.04, 0.04),
                         RNG.uniform(0.28, 0.55)])
        proj, _ = cv2.projectPoints(obj, rvec, tvec, K_TRUE, D_TRUE)
        p = proj.reshape(-1, 2) + RNG.normal(0, NOISE_PX, (len(obj), 2))
        if p[:, 0].min() < 0 or p[:, 0].max() > W or p[:, 1].min() < 0 or p[:, 1].max() > H:
            continue                                       # board left the frame
        ips.append(p.astype(np.float32).reshape(-1, 1, 2))
    return ips


def evaluate(ips, label):
    objp = fs.board_points(COLS, ROWS, SQ)
    size = (W, H)
    i_fixed = COLS - 1
    std = fs.fit_standard(objp, ips, size)
    ro = fs.fit_ro(objp, ips, size, i_fixed)
    recs = fs.kfold(objp, ips, size, i_fixed, 5)
    sh = np.mean([r['std_hold'] for r in recs])
    rh = np.mean([r['ro_hold'] for r in recs])
    wins = sum(r['ro_hold'] < r['std_hold'] for r in recs)
    use_ro = rh < sh and wins > len(recs) / 2
    fit = ro if use_ro else std
    pinned = (fs.fit_ro(objp, ips, size, i_fixed, flags=cv2.CALIB_FIX_PRINCIPAL_POINT)
              if use_ro else
              fs.fit_standard(objp, ips, size, flags=cv2.CALIB_FIX_PRINCIPAL_POINT))
    dfx = 100 * abs(pinned['K'][0, 0] - fit['K'][0, 0]) / fit['K'][0, 0]
    tilts = fs.pose_spread(fit['obj'], ips, fit['K'], fit['D'])
    tilt_ok = tilts.max() >= 25.0 and (tilts.max() - tilts.min()) >= 15.0
    _,_,_,_,_,sd,_,_ = cv2.calibrateCameraExtended(
        [fit['obj'].copy() for _ in ips], ips, size,
        fit['K'].copy(), fit['D'].copy(), flags=cv2.CALIB_USE_INTRINSIC_GUESS)
    sigma_fx = float(np.asarray(sd).ravel()[0])
    z = ro['obj'][:, 2]
    print(f"\n{label}  ({len(ips)} views)")
    print(f"  std  train {std['rms']:.3f}  hold {sh:.3f}  fx {std['K'][0,0]:8.2f}")
    print(f"  RO   train {ro['rms']:.3f}  hold {rh:.3f}  fx {ro['K'][0,0]:8.2f}"
          f"   (RO wins {wins}/{len(recs)})")
    print(f"  chosen: {'RO' if use_ro else 'standard'}   fx err vs truth "
          f"{100*(fit['K'][0,0]-K_TRUE[0,0])/K_TRUE[0,0]:+.2f} %"
          f"   conditioning shift {dfx:.2f} %")
    print(f"  tilt {tilts.min():.1f}..{tilts.max():.1f} deg -> "
          f"{'OK' if tilt_ok else 'TOO LITTLE TILT'};  "
          f"sigma(fx) {sigma_fx:.2f} px ({100*sigma_fx/fit['K'][0,0]:.2f} %)")
    return {'use_ro': use_ro, 'fit': fit, 'dfx': dfx,
            'tilt_ok': tilt_ok, 'sigma_fx': sigma_fx,
            'bow_mm': 1000 * (z.max() - z.min()), 'sh': sh, 'rh': rh}


def main() -> int:
    fails = []

    # ---- 1. flat board: must recover K, must NOT prefer RO ----------------
    r = evaluate(synth_views(0.0), "FLAT board, varied poses")
    err = abs(r['fit']['K'][0, 0] - K_TRUE[0, 0]) / K_TRUE[0, 0]
    if err > 0.01:
        fails.append(f"flat: fx off by {100*err:.2f} % (>1 %)")
    if r['use_ro']:
        fails.append("flat: chose RO on a genuinely flat board -- the held-out "
                     "test is NOT catching the extra degrees of freedom")
    if not r['tilt_ok']:
        fails.append("flat: healthy varied poses wrongly flagged as too little tilt")

    # ---- 2. bowed board: RO must win and recover the bow -------------------
    BOW = 2.0
    r = evaluate(synth_views(BOW), f"BOWED board ({BOW} mm), varied poses")
    err = abs(r['fit']['K'][0, 0] - K_TRUE[0, 0]) / K_TRUE[0, 0]
    if not r['use_ro']:
        fails.append("bowed: did not choose RO despite real board bow")
    if err > 0.01:
        fails.append(f"bowed: fx off by {100*err:.2f} % (>1 %)")
    if abs(r['bow_mm'] - BOW) > 0.6:
        fails.append(f"bowed: recovered {r['bow_mm']:.2f} mm, injected {BOW} mm")

    # ---- 3. degenerate capture: must be FLAGGED, not silently wrong --------
    r = evaluate(synth_views(0.0, n=24, frontal_only=True),
                 "FLAT board, NEAR-FRONTAL only (degenerate)")
    # The principal-point shift was VERIFIED INSUFFICIENT here (0.20 % on a
    # capture whose fx was 4.3 % wrong), which is why the tool now also
    # measures pose spread and OpenCV's own sigma(fx).
    if r['tilt_ok']:
        fails.append("degenerate: near-frontal-only capture was NOT flagged "
                     "by the tilt-spread diagnostic")
    if abs(r['fit']['K'][0,0]-K_TRUE[0,0])/K_TRUE[0,0] < 0.02:
        fails.append("degenerate: synthetic case is not actually degenerate; "
                     "the test has stopped testing anything")
    print(f"  (this is the failure mode that produced fx=835.7 on the real "
          f"camera; its training RMS looked fine)")

    print("\n" + "=" * 60)
    if fails:
        print("FAILED:")
        for f in fails:
            print(f"  - {f}")
        return 1
    print("ALL CHECKS PASSED -- pipeline recovers known ground truth,")
    print("resists overfitting on a flat board, and flags a degenerate capture.")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
