#!/usr/bin/env python3
"""Camera calibration that PROVES its own answer instead of asserting it.

Why this exists rather than a twelve-line calibrateCamera call
--------------------------------------------------------------
Three attempts at this camera returned fx = 835.7, 969.9 and 1011.2 -- a 21 %
spread -- and every one of them came with a confident-looking metric. The
training reprojection RMS is not a sufficient check, and the reasons are
specific:

  * RMS is not comparable across models. `calibrateCameraRO` releases the
    board's 3D point coordinates, adding ~3N free parameters. It will ALWAYS
    beat `calibrateCamera` on training RMS whether or not the shape it
    recovered is real. Reading that drop as "better" is the same error as
    preferring a 20th-order polynomial because it fits the training points.
  * RMS is not comparable across captures either. It scales with how far the
    board was and where it sat in the frame, so two runs' numbers do not mean
    the same thing (OpenCV forum, "How to evaluate camera calibration quality").
  * A degenerate capture -- too little tilt -- produces a LOW RMS and a wrong
    focal length, because focal length and distance stay confounded. That is
    exactly what attempt one did.

So this tool answers the question the way it has to be answered:

  1. HELD-OUT VALIDATION. K-fold: fit on most views, then for each held-out
     view re-fit ONLY the pose (solvePnP) and reproject. The extra parameters
     get no second chance on data they never saw, so a shape that is real
     geometry transfers and a shape that absorbed noise does not. This is the
     only test here that can distinguish the two.
  2. CONDITIONING DIAGNOSTIC. Re-fit with the principal point pinned to frame
     centre. If fx barely moves, fx is well determined by the data; if it
     swings, the solve is under-determined and no residual will tell you.
  3. FOLD SPREAD AS UNCERTAINTY. The scatter of fx across folds is an honest
     error bar. A single number with no uncertainty is what produced three
     mutually contradictory answers.
  4. AN EXTERNAL CHECK (`--external W_m Z_m`). Everything above is internal to
     the optimizer and can be self-consistently wrong. One measurement that
     does not come from the optimizer -- a board of known width at a tape-
     measured distance -- catches a gross error. It is only ~1 % accurate, so
     it arbitrates "roughly right", never the last 2 %.

The RO arm implements OpenCV's own recommendation for our situation: "if your
calibration board is inaccurate, unmeasured, roughly planar targets
(Checkerboard patterns on paper using off-the-shelf printers ... are not
accurate enough), a method from [Strobl & Hirzinger, ICCV 2011] can be
utilized to dramatically improve the accuracies of the estimated camera
intrinsic parameters."  -- docs/tutorials/calib3d/camera_calibration

Usage
-----
    fov_solve.py <dir> [--grid 8x6] [--square 0.025] [--folds 5]
                       [--external <board_width_m> <distance_m>]

The square size does NOT affect the intrinsics (verified: fx identical to
three decimals across 20/25/30/40 mm). It only scales the reported board
geometry, so leave it at the default if the board is unmeasured.
"""
from __future__ import annotations

import glob
import json
import math
import os
import time
import sys

import cv2
import numpy as np

# The classic detector plus cornerSubPix, with the tutorial's own criteria.
# findChessboardCornersSB was tested against this on two real capture sets and
# REJECTED: it matched to within 0.06 % of fx while finding 10/25 boards where
# the classic path found 23/25.
FIND = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
CRIT = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
SUBPIX_WIN = (11, 11)
N_WATER = 1.333  # sea/fresh water; a flat port refracts by Snell's law


# --------------------------------------------------------------------------
# geometry helpers
# --------------------------------------------------------------------------
def board_points(cols: int, rows: int, square: float) -> np.ndarray:
    """The ideal planar grid, (N,3) float32, Z=0 -- as the OpenCV tutorial builds it."""
    objp = np.zeros((rows * cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square
    return objp


def fov_from_K(K: np.ndarray, w: int, h: int) -> dict:
    fx, fy = float(K[0, 0]), float(K[1, 1])
    hfov = 2 * np.degrees(np.arctan(w / (2 * fx)))
    vfov = 2 * np.degrees(np.arctan(h / (2 * fy)))
    dfov = 2 * np.degrees(np.arctan(np.hypot(w, h) / (fx + fy)))

    def refract(a):
        # Snell through a flat port: the half-angle in water is
        # asin(sin(half-angle in air) / n). This is why an 80 deg air lens is
        # ~58 deg underwater -- the single biggest surprise for anyone sizing
        # a search pattern off a datasheet.
        return 2 * np.degrees(np.arcsin(min(1.0, np.sin(np.radians(a / 2)) / N_WATER)))

    return {
        'hfov_air': hfov, 'vfov_air': vfov, 'dfov_air': dfov,
        'hfov_water': refract(hfov), 'vfov_water': refract(vfov),
    }


# --------------------------------------------------------------------------
# detection
# --------------------------------------------------------------------------
def detect(files, cols, rows):
    """Return (image_points, names, size). One entry per view where the board was found."""
    ips, names, size = [], [], None
    for f in files:
        img = cv2.imread(f)
        if img is None:
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        size = gray.shape[::-1]
        ok, c = cv2.findChessboardCorners(gray, (cols, rows), FIND)
        if not ok:
            continue
        ips.append(cv2.cornerSubPix(gray, c, SUBPIX_WIN, (-1, -1), CRIT))
        names.append(os.path.basename(f))
    return ips, names, size


# --------------------------------------------------------------------------
# the two calibration arms
# --------------------------------------------------------------------------
def pose_spread(objp, ips, K, D):
    """How much board TILT the capture actually spans, in degrees.

    This is the diagnostic that matters and the one the principal-point check
    misses. Focal length and distance are confounded in a near-frontal view --
    a board twice as far with twice the focal length projects almost
    identically -- so a capture with little tilt is under-determined no matter
    how clean its residual. Verified against synthetic ground truth: a
    near-frontal-only capture returns fx 4.3 % wrong with a 0.20 px RMS.
    Perspective foreshortening is what breaks the degeneracy, and its
    magnitude is exactly the board's obliquity to the optical axis.
    """
    angles = []
    for ip in ips:
        ok, rvec, tvec = cv2.solvePnP(objp, ip, K, D)
        if not ok:
            continue
        R, _ = cv2.Rodrigues(rvec)
        # angle between the board normal (its local +z) and the viewing ray
        n = R[:, 2]
        angles.append(np.degrees(np.arccos(min(1.0, abs(float(n[2]))))))
    return np.array(angles)


def fit_standard(objp, ips, size, flags=0):
    ops = [objp.copy() for _ in ips]
    rms, K, D, rv, tv = cv2.calibrateCamera(ops, ips, size, None, None, flags=flags)
    return {'rms': rms, 'K': K, 'D': D, 'obj': objp.copy()}


def fit_ro(objp, ips, size, i_fixed, flags=0):
    """Object-releasing method: the board's 3D shape is estimated, not assumed.

    `i_fixed` is the index of the top-right grid corner (cols-1). Points 0 and
    i_fixed are held to set position and scale; every other point's 3D
    coordinate is free. That is what absorbs a bowed printed target.
    """
    ops = [objp.copy() for _ in ips]
    out = cv2.calibrateCameraRO(ops, ips, size, i_fixed, None, None, flags=flags)
    rms, K, D, new_obj = out[0], out[1], out[2], np.asarray(out[5]).reshape(-1, 3)
    return {'rms': rms, 'K': K, 'D': D, 'obj': new_obj.astype(np.float32)}


def holdout_rms(fit, ips_test):
    """Reproject held-out views, re-fitting ONLY the 6-DOF pose.

    The crucial detail: the RO arm must be evaluated against the object points
    IT recovered, not against the ideal grid. Scoring RO's K on the ideal grid
    tests a model nobody fitted and quietly penalises the method.
    """
    obj = fit['obj']
    errs = []
    for ip in ips_test:
        ok, rvec, tvec = cv2.solvePnP(obj, ip, fit['K'], fit['D'])
        if not ok:
            return float('nan')
        proj, _ = cv2.projectPoints(obj, rvec, tvec, fit['K'], fit['D'])
        # RMS over points: sqrt(mean(squared distance)). Dividing an L2 norm by
        # N instead of sqrt(N) is a mean-of-norms and reads ~sqrt(N) times too
        # small -- it once made a 1.57 px calibration look like 0.19 px.
        errs.append(cv2.norm(ip, proj, cv2.NORM_L2) / np.sqrt(len(proj)))
    return float(np.sqrt(np.mean(np.square(errs))))


def kfold(objp, ips, size, i_fixed, k):
    """K-fold held-out comparison of the two arms. Returns per-fold records."""
    n = len(ips)
    order = np.arange(n)
    np.random.default_rng(0).shuffle(order)   # fixed seed: reruns are comparable
    folds = np.array_split(order, min(k, n))
    recs = []
    for i, test_idx in enumerate(folds):
        fit_idx = [j for j in order if j not in set(test_idx.tolist())]
        if len(fit_idx) < 6 or len(test_idx) == 0:
            continue
        tr = [ips[j] for j in fit_idx]
        te = [ips[j] for j in test_idx]
        std = fit_standard(objp, tr, size)
        ro = fit_ro(objp, tr, size, i_fixed)
        recs.append({
            'fold': i, 'n_fit': len(tr), 'n_test': len(te),
            'std_train': std['rms'], 'std_hold': holdout_rms(std, te),
            'std_fx': float(std['K'][0, 0]),
            'ro_train': ro['rms'], 'ro_hold': holdout_rms(ro, te),
            'ro_fx': float(ro['K'][0, 0]),
            # Kept so the folds can serve as an empirical POSTERIOR SAMPLE
            # over the calibration, which is what Max ERE needs.
            'std_K': std['K'], 'std_D': std['D'],
            'ro_K': ro['K'], 'ro_D': ro['D'],
        })
    return recs


def max_ere(samples, w, h, grid=5, depth_m=1.0):
    """AprilCal's MAX EXPECTED REPROJECTION ERROR (Richardson et al., IROS 2013).

    ⛔ WHY THIS AND NOT REPROJECTION RMS. RMS is an average over the points
    you happened to photograph, so it is quiet exactly where you have no
    data -- and calib.io says the same thing in one line: "low reprojection
    error does not equal a good camera calibration". This project has three
    instances of it: fx of 835.7, 969.9 and 1011.2, each with a comfortable
    residual.

    Max ERE asks a different question -- **how much would two equally
    plausible calibrations disagree about where a point lands?** Sample
    calibrations from the posterior, project a grid of fixed 3D test points
    through each, and take the WORST point's spread. It therefore reports the
    part of the image the data has not constrained, which is the part that
    silently ruins a bearing or a scale.

    Our posterior sample is the k-fold fits. That is coarser than AprilCal's
    (they sample the parameter covariance directly) and it is honest about
    what it is: k draws, so treat it as a lower bound on the true spread
    rather than a tight estimate.

    AprilCal's reported bar, and the reason this is worth having: **6-8
    images to reach < 1 px**, and with it novice users beat expert-free
    OpenCV runs 23x on WORST-case reprojection error (1.651 px vs 38.646).
    """
    if len(samples) < 2:
        return float('nan'), None
    # A grid of 3D points spanning the field of view at a working distance,
    # placed using the MEAN calibration so the grid is the same for all draws.
    Km = np.mean([K for K, _ in samples], axis=0)
    us = np.linspace(0.05 * w, 0.95 * w, grid)
    vs = np.linspace(0.05 * h, 0.95 * h, grid)
    pts3d = []
    for v in vs:
        for u in us:
            x = (u - Km[0, 2]) / Km[0, 0] * depth_m
            y = (v - Km[1, 2]) / Km[1, 1] * depth_m
            pts3d.append((x, y, depth_m))
    pts3d = np.array(pts3d, dtype=np.float64).reshape(-1, 1, 3)
    rvec = np.zeros(3); tvec = np.zeros(3)
    proj = []
    for K, D in samples:
        p, _ = cv2.projectPoints(pts3d, rvec, tvec, K, D)
        proj.append(p.reshape(-1, 2))
    proj = np.array(proj)                       # (samples, points, 2)
    mean = proj.mean(axis=0)
    ere = np.linalg.norm(proj - mean, axis=2).mean(axis=0)   # per point
    i = int(np.argmax(ere))
    return float(ere[i]), (float(pts3d[i, 0, 0]), float(pts3d[i, 0, 1]))


def kfold_std(objp, ips, size, k=4):
    """Standard-arm-only folds, as a posterior sample.

    Separate from `kfold` because the suggestion loop runs this ~36 times and
    does not need the RO arm at all -- and `calibrateCameraRO` additionally
    REFUSES some synthetic-augmented sets (it returned a scalar where the
    released object points belong), so including it made the suggester crash
    on exactly the inputs it exists to evaluate.
    """
    n = len(ips)
    order = np.arange(n)
    np.random.default_rng(0).shuffle(order)
    out = []
    for test_idx in np.array_split(order, min(k, n)):
        fit_idx = [j for j in order if j not in set(test_idx.tolist())]
        if len(fit_idx) < 6:
            continue
        f = fit_standard(objp, [ips[j] for j in fit_idx], size)
        out.append((f['K'], f['D']))
    return out


def pose_for(cell, tilt_deg, objp, K, w, h, grid=3):
    """A board pose that lands in `cell` of the frame at `tilt_deg`.

    Returns (rvec, tvec). Used to SYNTHESISE a candidate observation so its
    value can be scored before the operator is asked to go and make it --
    which is the whole idea of a next-best-pose suggestion.
    """
    gy, gx = cell
    # Depth chosen so the board fills a reasonable share of the frame.
    span = float(np.ptp(objp[:, 0])) or 0.2
    Z = 1.15 * span * K[0, 0] / (0.45 * w)
    u = (gx + 0.5) * w / grid
    v = (gy + 0.5) * h / grid
    tvec = np.array([(u - K[0, 2]) / K[0, 0] * Z,
                     (v - K[1, 2]) / K[1, 1] * Z, Z], float)
    # Tilt about the image-x axis, then a little about y, so the pose is not
    # degenerate in one direction only.
    t = math.radians(tilt_deg)
    rvec = np.array([t * 0.85, t * 0.53, 0.0], float)
    return rvec, tvec


def synth_view(objp, K, D, rvec, tvec, w, h, noise_px=0.3, rng=None):
    """Project the board at a pose, as a candidate observation.

    Noise is added on purpose: a NOISELESS synthetic view makes the fit look
    better than any real one ever will, and the suggestion would then be
    scored against a view that cannot be captured.
    """
    rng = rng or np.random.default_rng(0)
    p, _ = cv2.projectPoints(objp.reshape(-1, 1, 3), rvec, tvec, K, D)
    p = p.reshape(-1, 2)
    if (p[:, 0] < 0).any() or (p[:, 0] > w).any() or \
       (p[:, 1] < 0).any() or (p[:, 1] > h).any():
        return None                                  # would not fit in frame
    p = p + rng.normal(0.0, noise_px, p.shape)
    return p.reshape(-1, 1, 2).astype(np.float32)


def suggest_next_pose(objp, ips, size, candidates, i_fixed=0, k=4):
    """AprilCal's next-best-pose, scored by predicted Max ERE.

    ⛔ THIS IS THE PART THAT MAKES A NOVICE'S CALIBRATION GOOD. Richardson et
    al. (IROS 2013) measured novices using guided suggestion against novices
    using plain OpenCV: mean reprojection error 0.229 vs 0.728 px, and
    WORST-case 1.651 vs 38.646 px -- a 23x difference on the number that
    actually breaks a bearing. Focal length spread 1.2 vs 9.0. Thirteen of
    their sixteen subjects had never calibrated anything.

    Method, following theirs but with our own posterior: for each candidate
    pose, SYNTHESISE the observation it would produce under the current
    calibration, add it to the set, refit, and recompute Max ERE. The
    candidate with the lowest predicted Max ERE is the one to ask for. They
    evaluate ~60 candidates; ours is the 9x4 cell/tilt grid, which keeps this
    real-time on a Pi.

    Returns (best_candidate, predicted_ere, table) or (None, nan, []) when
    there is not yet enough data to fit at all.
    """
    w, h = size
    if len(ips) < 6:
        return None, float('nan'), []
    base = fit_standard(objp, ips, size)
    K, D = base['K'], base['D']
    rng = np.random.default_rng(7)
    table = []
    for cand in candidates:
        cell, tilt = cand
        rvec, tvec = pose_for(cell, tilt, objp, K, w, h)
        obs = synth_view(objp, K, D, rvec, tvec, w, h, rng=rng)
        if obs is None:
            continue
        try:
            samples = kfold_std(objp, list(ips) + [obs], size, k)
            if len(samples) < 2:
                continue
            ere, _ = max_ere(samples, w, h)
        except cv2.error:
            continue
        if not math.isfinite(ere):
            continue
        table.append((ere, cand))
    if not table:
        return None, float('nan'), []
    table.sort(key=lambda t: t[0])
    return table[0][1], table[0][0], table


# --------------------------------------------------------------------------
def main() -> int:
    args = sys.argv[1:]
    if not args:
        print(__doc__)
        return 2
    outdir = args[0]
    cols, rows, square, k = 8, 6, 0.025, 5
    external = None
    applies_to = None       # e.g. 'pi_forward' -- the camera PROFILE this describes
    install_dir = None      # write straight into the package's calibration dir
    camera_desc = ''
    for i, a in enumerate(args):
        if a == '--grid':
            cols, rows = (int(x) for x in args[i + 1].lower().split('x'))
        elif a == '--square':
            square = float(args[i + 1])
        elif a == '--folds':
            k = int(args[i + 1])
        elif a == '--external':
            external = (float(args[i + 1]), float(args[i + 2]))
        elif a == '--applies-to':
            applies_to = args[i + 1]
        elif a == '--install':
            install_dir = args[i + 1]
        elif a == '--camera':
            camera_desc = args[i + 1]

    files = sorted(glob.glob(os.path.join(outdir, '*.png')))
    ips, names, size = detect(files, cols, rows)
    if size is None or len(ips) < 8:
        print(f"found {len(ips)} boards in {len(files)} images -- need >= 8 "
              f"(OpenCV's own floor is 10 for a well-posed system)")
        return 1
    w, h = size
    objp = board_points(cols, rows, square)
    i_fixed = cols - 1
    print(f"\n{len(ips)}/{len(files)} boards detected at {w}x{h}, grid {cols}x{rows}")

    # ---- 1. both arms on all data (training numbers -- NOT a comparison) ----
    std = fit_standard(objp, ips, size)
    ro = fit_ro(objp, ips, size, i_fixed)
    print("\n-- fits on all views (training RMS; NOT comparable between arms) --")
    for nm, f in (('standard', std), ('RO', ro)):
        v = fov_from_K(f['K'], w, h)
        print(f"  {nm:9s} RMS {f['rms']:6.3f}  fx {f['K'][0,0]:8.2f} fy {f['K'][1,1]:8.2f}"
              f"  cx {f['K'][0,2]:7.2f} cy {f['K'][1,2]:7.2f}"
              f"  HFOV air {v['hfov_air']:5.2f} water {v['hfov_water']:5.2f}")
    z = ro['obj'][:, 2]
    print(f"  RO recovered board bow: {1000*(z.max()-z.min()):.2f} mm peak-to-peak, "
          f"{1000*z.std():.2f} mm rms   (scales with --square; unmeasured board = indicative)")

    # ---- 2. held-out validation: the test that actually decides ----
    print(f"\n-- {k}-fold HELD-OUT validation (this is what decides) --")
    recs = kfold(objp, ips, size, i_fixed, k)
    if not recs:
        print("  too few views to hold any out")
        return 1
    print(f"  {'fold':>4} {'fit/test':>9} {'std train':>10} {'std HOLD':>9}"
          f" {'RO train':>9} {'RO HOLD':>8}   winner")
    for r in recs:
        win = 'RO' if r['ro_hold'] < r['std_hold'] else 'standard'
        print(f"  {r['fold']:>4} {r['n_fit']:>4}/{r['n_test']:<4} {r['std_train']:10.3f}"
              f" {r['std_hold']:9.3f} {r['ro_train']:9.3f} {r['ro_hold']:8.3f}   {win}")
    sh = np.array([r['std_hold'] for r in recs])
    rh = np.array([r['ro_hold'] for r in recs])
    ro_wins = int((rh < sh).sum())
    print(f"\n  mean held-out RMS   standard {sh.mean():.3f}   RO {rh.mean():.3f}"
          f"   -> RO wins {ro_wins}/{len(recs)} folds")
    use_ro = rh.mean() < sh.mean() and ro_wins > len(recs) / 2
    print(f"  VERDICT: use {'calibrateCameraRO' if use_ro else 'calibrateCamera (standard)'}"
          f" -- chosen on held-out error, not training RMS")

    # ---- 3. uncertainty from fold spread ----
    # ---- Max ERE: what the calibration does NOT know, in pixels ----
    kk = 'ro' if use_ro else 'std'
    samples = [(r[kk + '_K'], r[kk + '_D']) for r in recs]
    ere, worst = max_ere(samples, w, h)
    print(f"\n-- Max ERE (AprilCal, IROS 2013) --")
    print(f"  worst-case disagreement between equally plausible calibrations:"
          f" {ere:.3f} px")
    print(f"  bar: < 1.0 px  ->  {'PASS' if ere < 1.0 else 'NOT YET'}"
          f"   (their result: 6-8 images reach this)")
    if ere >= 1.0:
        print(f"  the worst point is at ray ({worst[0]:+.2f}, {worst[1]:+.2f}) "
              f"-- more views THERE constrain it fastest")
    print(f"  ⚠ our posterior is {len(samples)} k-fold draws, not a sampled "
          f"covariance, so read this as a LOWER bound on the spread")

    fxs = np.array([r['ro_fx' if use_ro else 'std_fx'] for r in recs])
    fx_sd = float(fxs.std(ddof=1)) if len(fxs) > 1 else float('nan')
    print(f"\n-- stability --")
    print(f"  fx across folds: {fxs.mean():.2f} +/- {fx_sd:.2f} px "
          f"(min {fxs.min():.2f}, max {fxs.max():.2f})")

    # ---- 4. conditioning: is fx actually determined by the data? ----
    # Two independent measures, because the principal-point check alone was
    # verified INSUFFICIENT -- it passed a synthetic capture whose fx was
    # 4.3 % wrong (see test_fov_solve.py case 3).
    fit_now = ro if use_ro else std
    tilts = pose_spread(fit_now['obj'], ips, fit_now['K'], fit_now['D'])
    # CALIB_USE_INTRINSIC_GUESS is REQUIRED here: once RO releases the object
    # points the rig is non-planar (z != 0), and OpenCV refuses to auto-init
    # intrinsics for a non-planar rig.
    _, _, _, _, _, sd_int, _, _ = cv2.calibrateCameraExtended(
        [fit_now['obj'].copy() for _ in ips], ips, size,
        fit_now['K'].copy(), fit_now['D'].copy(),
        flags=cv2.CALIB_USE_INTRINSIC_GUESS)
    sigma_fx = float(np.asarray(sd_int).ravel()[0])
    print(f"\n-- conditioning --")
    print(f"  board tilt spanned: {tilts.min():.1f} .. {tilts.max():.1f} deg "
          f"(median {np.median(tilts):.1f})")
    tilt_ok = tilts.max() >= 25.0 and (tilts.max() - tilts.min()) >= 15.0
    print(f"    {'OK' if tilt_ok else 'TOO LITTLE TILT -- fx and distance stay confounded'}"
          f"   (want max >= 25 deg, range >= 15 deg)")
    print(f"  sigma(fx) from OpenCV: +/- {sigma_fx:.2f} px "
          f"({100*sigma_fx/fit_now['K'][0,0]:.2f} %)   "
          f"{'OK' if sigma_fx < 0.01*fit_now['K'][0,0] else 'HIGH -- solve is loose'}")
    pinned = (fit_ro if use_ro else fit_standard)(
        objp, ips, size, *( (i_fixed,) if use_ro else () ),
        flags=cv2.CALIB_FIX_PRINCIPAL_POINT)
    free = ro if use_ro else std
    dfx = 100 * abs(pinned['K'][0, 0] - free['K'][0, 0]) / free['K'][0, 0]
    print(f"  principal point pinned to centre -> fx {pinned['K'][0,0]:.2f} "
          f"({dfx:.2f} % shift)   "
          f"{'WELL CONDITIONED' if dfx < 1.0 else 'UNDER-DETERMINED: capture more tilt'}")
    off = np.hypot(free['K'][0, 2] - w / 2, free['K'][1, 2] - h / 2)
    print(f"  principal point {off:.1f} px from frame centre "
          f"({'plausible' if off < 0.08 * w else 'SUSPICIOUS -- usually within a few % of centre'})")

    # ---- 5. external sanity check ----
    fit = ro if use_ro else std
    v = fov_from_K(fit['K'], w, h)
    if external:
        board_w, dist_z = external
        # widest observed board span in pixels, over the most frontal view
        best, best_px = None, 0.0
        for ip in ips:
            p = ip.reshape(-1, 2)
            span = p[:, 0].max() - p[:, 0].min()
            if span > best_px:
                best_px, best = span, p
        fx_ext = best_px * dist_z / board_w
        err = 100 * (fx_ext - fit['K'][0, 0]) / fit['K'][0, 0]
        print(f"\n-- external check (does not come from the optimizer) --")
        print(f"  board {board_w*100:.1f} cm at {dist_z*100:.1f} cm spans {best_px:.1f} px"
              f"  ->  fx ~= {fx_ext:.1f}   ({err:+.1f} % vs solve)")
        print(f"  {'consistent' if abs(err) < 5 else 'DISAGREES -- investigate before shipping'}"
              f"   (this check is only ~1-2 % accurate; it catches gross error, not the last 2 %)")

    # ---- 6. the answer, with an error bar ----
    fx = float(fit['K'][0, 0])
    hf_lo = 2 * np.degrees(np.arctan(w / (2 * (fx + fx_sd))))
    hf_hi = 2 * np.degrees(np.arctan(w / (2 * (fx - fx_sd))))
    print(f"\n{'='*66}\n  ANSWER  ({'RO' if use_ro else 'standard'}, "
          f"{len(ips)} views, held-out validated)")
    print(f"    fx {fit['K'][0,0]:.2f}   fy {fit['K'][1,1]:.2f}   "
          f"cx {fit['K'][0,2]:.2f}   cy {fit['K'][1,2]:.2f}")
    print(f"    HFOV in air    {v['hfov_air']:.2f} deg   "
          f"(fold spread {hf_lo:.2f} .. {hf_hi:.2f})")
    print(f"    VFOV in air    {v['vfov_air']:.2f} deg")
    print(f"    HFOV in water  {v['hfov_water']:.2f} deg   <- the number missions need")
    print(f"    VFOV in water  {v['vfov_water']:.2f} deg")
    print('='*66)

    out = {
        'image_width': w, 'image_height': h,
        'camera_matrix': fit['K'].tolist(),
        'distortion_coefficients': fit['D'].ravel().tolist(),
        'method': 'calibrateCameraRO' if use_ro else 'calibrateCamera',
        'chosen_by': 'k-fold held-out reprojection error',
        'holdout_rms_px': float(rh.mean() if use_ro else sh.mean()),
        'holdout_rms_other_arm_px': float(sh.mean() if use_ro else rh.mean()),
        'train_rms_px': float(fit['rms']),
        'fx_fold_sd_px': fx_sd,
        # AprilCal's Max Expected Reprojection Error -- the worst-case
        # disagreement between equally plausible calibrations. Their bar is
        # < 1 px, reached in 6-8 guided images.
        'max_ere_px': float(ere),
        'max_ere_bar_px': 1.0,
        'max_ere_note': ('AprilCal (Richardson et al., IROS 2013). Posterior '
                         'is k-fold draws, so this is a LOWER bound. RMS is '
                         'quiet where there is no data; this is not.'),
        'views_used': len(ips), 'views_captured': len(files),
        'grid': [cols, rows], 'square_m': square,
        'board_bow_p2p_mm': float(1000 * (z.max() - z.min())),
        # Key names must match camera_node._load_calibration, which reads
        # 'hfov_deg_air'. It uses .get(..., 0.0), so a mismatch does not crash
        # -- it silently logs HFOV=0.0 forever.
        **{f"{key.split('_')[0]}_deg_{key.split('_')[1]}": float(val)
           for key, val in v.items()},
        'note': ('FOV is invariant to square size; board bow figure is not. '
                 'Water FOV is Snell through a flat port, n=1.333.'),
    }
    # ⛔ `applies_to` IS NOT OPTIONAL METADATA -- IT IS THE GUARD.
    # The one calibration this project held was named for the wrong camera
    # and wired to the wrong camera for four days, in both directions, and
    # nothing could catch it because the file did not say what it described.
    # `test_calibration_binding.py` now refuses any calibration without this
    # field, so a file emitted without it is not installable.
    if applies_to:
        out = {'applies_to': [applies_to],
               'applies_to_note': (
                   'The camera PROFILE this describes. Present because a '
                   'calibration named for one camera was wired to the other, '
                   'both ways, and no test could see it: a file that does not '
                   'say what it describes cannot be checked against what it '
                   'is wired to.'),
               **out}
        if camera_desc:
            out['camera'] = camera_desc
        out['captured'] = time.strftime('%Y-%m-%d')

    path = os.path.join(outdir, 'calibration.json')
    with open(path, 'w') as fh:
        json.dump(out, fh, indent=2)
    print(f"\nwrote {path}")

    if install_dir:
        if not applies_to:
            print("  --install needs --applies-to: an installed calibration "
                  "without\n  `applies_to` fails test_calibration_binding "
                  "and cannot be wired.")
            return 1
        name = f'{applies_to}_{w}x{h}.json'
        dest = os.path.join(install_dir, name)
        with open(dest, 'w') as fh:
            json.dump(out, fh, indent=2)
        print(f"installed {dest}")
        print(f"  vision_pi.launch.py already names {name}, so it goes live "
              f"on the next\n  colcon build -- no launch edit needed.")
    return 0


if __name__ == '__main__':
    sys.exit(main())
