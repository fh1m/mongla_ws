#!/usr/bin/env python3
"""Measure a camera's intrinsics and FIELD OF VIEW from checkerboard images.

Why this exists: duburi_ws publishes CameraInfo with K and D EMPTY
(`camera_node.py:9` -- "size only"), and there is no HFOV anywhere in the tree.
`vision.align` steers on pixel offsets, which is fine on its own, but anything
that turns a pixel into an ANGLE (a bearing, a LANDING_TARGET, a range estimate)
needs a measured focal length. The simulator has a calibrated 57.7 deg; the
vehicle has nothing.

NOTE ON THE BOARD: square size does NOT affect the intrinsics. It only scales
translation, which we do not use. So no ruler is needed for FOV -- pass the
default and the focal lengths and angles are still correct.

Usage:
  fov_calibrate.py capture <device> <outdir> [n]   # grab frames, live feedback
  fov_calibrate.py solve   <outdir> [--square M]   # calibrate + report FOV
"""
import glob, json, os, sys, time
import numpy as np, cv2

# The refractive index and both Snell transforms live in ONE place (B22).
# tools/ runs from a source checkout without the package installed, so fall back
# to the literal ONLY if that import fails -- and say so, rather than letting a
# second silent copy of a physical constant exist.
try:
    from duburi_vision.optics import N_WATER, fov_air_to_water
except ImportError:                                    # pragma: no cover
    import sys as _sys, os as _os
    _sys.path.insert(0, _os.path.join(_os.path.dirname(_os.path.abspath(__file__)),
                                      '..', 'src', 'duburi_vision'))
    from duburi_vision.optics import N_WATER, fov_air_to_water


# INNER corners. ⛔ THIS MUST MATCH THE BOARD AND `fov_solve.py`, which
# defaults to 8x6 -- the grid the shipped downward calibration was actually
# measured on (`pi_downward_1280x720.json: grid [8, 6]`). This file said 9x6,
# so `capture` would have found NOTHING with the board we own and reported it
# as "no board in view" -- an operator sent to re-print a board that is
# already correct. Override with --grid CxR.
COLS, ROWS = 8, 6
CRIT = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


def find(gray):
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    ok, c = cv2.findChessboardCorners(gray, (COLS, ROWS), flags)
    if ok:
        c = cv2.cornerSubPix(gray, c, (11, 11), (-1, -1), CRIT)
    return ok, c


def _grid_from_argv():
    """--grid CxR, shared by capture and solve so the two cannot disagree."""
    global COLS, ROWS
    for i, a in enumerate(sys.argv):
        if a == '--grid' and i + 1 < len(sys.argv):
            COLS, ROWS = (int(x) for x in sys.argv[i + 1].lower().split('x'))
    return COLS, ROWS


def capture(dev, outdir, want=25):
    os.makedirs(outdir, exist_ok=True)
    cap = cv2.VideoCapture(int(dev), cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    # Calibrate at the HIGHEST resolution the camera gives, then scale the
    # result: intrinsics scale linearly with resolution, and more pixels means
    # better corner localisation.
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_FPS, 30)
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)); h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"capturing {w}x{h} from /dev/video{dev} -> {outdir}")
    print("hold the board at DIFFERENT angles and distances, fill different")
    print("parts of the frame -- corners and edges matter most for distortion\n")
    kept, last = 0, 0.0
    try:
        while kept < want:
            ok, f = cap.read()
            if not ok:
                continue
            g = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
            found, c = find(g)
            now = time.time()
            if found and now - last > 1.0:       # 1 s apart: avoid near-duplicates
                cv2.imwrite(f'{outdir}/cal_{kept:03d}.png', f)
                kept += 1; last = now
                cx, cy = c.reshape(-1, 2).mean(0)
                print(f"  [{kept:2d}/{want}] board found, centre ({cx:4.0f},{cy:4.0f})")
            elif not found and now - last > 3.0:
                print("  ...no board in view", end='\r'); last = now - 2.0
    except KeyboardInterrupt:
        print("\n  stopped by user")
    cap.release()
    print(f"\n{kept} images in {outdir}")


def solve(outdir, square=0.025):
    files = sorted(glob.glob(f'{outdir}/*.png'))
    if not files:
        sys.exit(f"no images in {outdir}")
    objp = np.zeros((ROWS * COLS, 3), np.float32)
    objp[:, :2] = np.mgrid[0:COLS, 0:ROWS].T.reshape(-1, 2) * square
    op, ip, shape = [], [], None
    for f in files:
        img = cv2.imread(f); g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        shape = g.shape[::-1]
        ok, c = find(g)
        if ok:
            op.append(objp); ip.append(c)
    print(f"board found in {len(ip)}/{len(files)} images at {shape[0]}x{shape[1]}")
    if len(ip) < 8:
        sys.exit("need >= 8 good views; capture more, at more angles")

    rms, K, D, rv, tv = cv2.calibrateCamera(op, ip, shape, None, None)

    # per-view reprojection error -- the honest quality signal
    errs = []
    for i in range(len(op)):
        proj, _ = cv2.projectPoints(op[i], rv[i], tv[i], K, D)
        errs.append(cv2.norm(ip[i], proj, cv2.NORM_L2) / len(proj))
    w, h = shape
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    hfov = 2 * np.degrees(np.arctan(w / (2 * fx)))
    vfov = 2 * np.degrees(np.arctan(h / (2 * fy)))
    dfov = 2 * np.degrees(np.arctan(np.hypot(w, h) / (2 * (fx + fy) / 2)))
    # Snell through a flat port: what this lens becomes UNDERWATER
    n = N_WATER
    hfov_w = fov_air_to_water(hfov)
    vfov_w = fov_air_to_water(vfov)

    print(f"\n  RMS reprojection error : {rms:.4f} px   "
          f"({'good' if rms < 0.5 else 'usable' if rms < 1.0 else 'POOR -- recapture'})")
    print(f"  per-view err  min/med/max: {min(errs):.3f} / {np.median(errs):.3f} / {max(errs):.3f} px")
    print(f"  fx, fy                 : {fx:.2f}, {fy:.2f} px")
    print(f"  cx, cy                 : {cx:.2f}, {cy:.2f}  (frame centre {w/2:.0f},{h/2:.0f})")
    print(f"  distortion k1,k2,p1,p2,k3: {', '.join(f'{v:+.4f}' for v in D.ravel()[:5])}")
    print(f"\n  IN AIR    HFOV {hfov:6.2f} deg   VFOV {vfov:6.2f} deg   DFOV {dfov:6.2f} deg")
    print(f"  UNDERWATER (flat port, n={N_WATER})")
    print(f"            HFOV {hfov_w:6.2f} deg   VFOV {vfov_w:6.2f} deg"
          f"   <- {100*(1-hfov_w/hfov):.0f} % narrower")

    out = {
        'image_width': w, 'image_height': h,
        'camera_matrix': K.tolist(), 'distortion_coefficients': D.ravel().tolist(),
        'rms_reprojection_error_px': float(rms),
        'views_used': len(ip), 'views_captured': len(files),
        'hfov_deg_air': float(hfov), 'vfov_deg_air': float(vfov),
        'hfov_deg_water': float(hfov_w), 'vfov_deg_water': float(vfov_w),
        'note': ('Square size does not affect intrinsics; only translation scales '
                 'with it. Underwater values are Snell through a flat port and are '
                 'DERIVED, not measured in water.'),
    }
    p = f'{outdir}/calibration.json'
    json.dump(out, open(p, 'w'), indent=1)
    print(f"\n  wrote {p}")


if __name__ == '__main__':
    _grid_from_argv()
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    if sys.argv[1] == 'capture':
        capture(sys.argv[2], sys.argv[3], int(sys.argv[4]) if len(sys.argv) > 4 else 25)
    elif sys.argv[1] == 'solve':
        sq = 0.025
        if '--square' in sys.argv:
            sq = float(sys.argv[sys.argv.index('--square') + 1])
        solve(sys.argv[2], sq)
