"""The capture half of the guided calibration: the pose script and its threads.

`calibrate_node.py` feeds `State.raw` from the camera node's frames;
`detect_loop` matches the live pose against the current step on a downscale
and saves the full-resolution frame when it is held, and `assess` refits from
what has been captured and suggests the next pose. Both loops end when
`State.stopped` is set, so the node can start a new capture without a restart.
"""
import glob
import os
import subprocess
import threading
import time

import cv2
import numpy as np


FIND = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
CRIT = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
GRID = 3
# Tilt bands are ros's skew, and the old metric was broken.
#
# The first version compared the board's two DIAGONALS. That is zero for the
# motion an operator actually makes: tilting about a horizontal or vertical
# axis projects a rectangle to a SYMMETRIC TRAPEZOID, whose diagonals are
# equal by symmetry. Measured on synthetic ground truth:
#
#     tilt     old diagonal metric      ROS skew
#      15 deg        0.0000              0.054
#      30 deg        0.0000              0.115
#      55 deg        0.0000              0.284
#
# EXACTLY ZERO at 55 degrees. The operator reported being unable to reach
# "steeply tilted" no matter how far they tilted, and they were right -- the
# band was unreachable by construction, not by technique.
#
# The replacement is the measure `ros-perception/image_pipeline`'s
# `camera_calibration` has used for years: how far a corner of the projected
# quad departs from 90 degrees,
#     skew = min(1, 2*|pi/2 - angle(up_left, up_right, down_right)|)
# which is sensitive to exactly the symmetric tilt the diagonals cannot see.
#
# The band edges come from that mapping, and the top band stops at ~45 deg
# On purpose. calib.io's guidance is explicit: use up to +/-45 degrees,
# because "tilting more is usually not a good idea as feature localization
# accuracy suffers and can become biased". Asking for "as steep as you can"
# is asking for a worse calibration.
#
#     skew  0.03 = 10 deg    0.075 = 20 deg    0.13 = 33 deg    0.20 = 45 deg
TILT_BINS = (0.00, 0.03, 0.075, 0.13, 1.00)
TILT_NAME = ('square-on', 'slightly tilted (~15 deg)',
             'clearly tilted (~25 deg)', 'steeply tilted (~40 deg)')
CELL_NAME = (('top-left', 'top-centre', 'top-right'),
             ('middle-left', 'CENTRE', 'middle-right'),
             ('bottom-left', 'bottom-centre', 'bottom-right'))
# OpenCV's own chessboard sharpness metric; its documented target is < 3 px.
# Measured on 21 frames captured under the old auto-exposure: median 7.30 px,
# and only 3 of 13 under target. That is the blur, quantified by the
# library's own yardstick rather than by eye.
MAX_SHARPNESS_PX = 3.0
# AprilCal's stopping bar: they reach it in 6-8 guided images.
ERE_BAR_PX = 1.0
HOLD_S = 0.5          # the pose must persist this long before it is taken


def build_steps():
    """The pose script. Coverage x tilt by construction, not by hope.

    Ordered so the easy poses come first and the operator learns the
    interaction before being asked for a steep corner. Every one of the nine
    regions appears at two different tilts, then the extremes are swept: the
    four corners steeply (where distortion has its only evidence) and the
    centre square-on twice (which anchors the principal point).
    """
    cells = [(y, x) for y in range(GRID) for x in range(GRID)]
    steps = []
    for band in (1, 2):                       # slight, then clear
        for (y, x) in cells:
            steps.append((y, x, band))
    for (y, x) in ((0, 0), (0, 2), (2, 0), (2, 2)):
        steps.append((y, x, 3))               # corners, steep
    steps.append((1, 1, 0))                   # centre, square-on
    steps.append((1, 1, 0))
    return steps


class State:
    def __init__(self, cols, rows, outdir, steps):
        self.lock = threading.Lock()
        self.cols, self.rows, self.outdir = cols, rows, outdir
        self.square = None
        self.steps = steps
        self.i = 0                     # current step index
        self.raw = None                # the newest camera frame, full size
        self.corners = None
        self.taken = []                # (y, x, band) actually captured
        self.msg = 'starting...'
        self.err = None
        self.det_ms = 0.0
        self.cell = None               # live: which cell the board is in
        self.band = None               # live: which tilt band
        self.hold = 0.0                # 0..1 progress of the hold timer
        self.sharp = None              # OpenCV chessboard sharpness, px
        # AIR or WATER. Recorded, shown, and part of the filename, because
        # the two differ by ~1.44x and mixing them is a silent 44 % scale
        # error on every range the vehicle computes.
        self.medium = 'air'
        self.K = self.D = None     # latest fit, for the undistorted preview
        self.solve = 'idle'
        self.solve_out = ''
        # AprilCal state: how uncertain the calibration still is, and where
        # to point the operator next.
        self.ere = None            # Max ERE, px
        self.ere_pred = None       # predicted Max ERE after the suggestion
        self.suggest = None        # (cell, tilt_deg) the solver wants next
        self.assessed_n = 0
        self.n_boards = 0
        self.assess_err = None
        self.fx = self.fy = self.cx = self.cy = None
        self.fx_sd = self.hfov_air = self.hfov_water = None
        self.worst_px = None
        self.stopped = False       # set to end both loops


def tilt_of(corners, cols):
    """Foreshortening as ROS `camera_calibration` measures it.

    How far the corner at `up_right` departs from a right angle. Scale-free,
    so it does not confuse "further away" with "more tilted", and -- unlike
    the diagonal comparison this replaces -- it responds to tilt about a
    horizontal or vertical axis, which is the tilt a person actually makes.
    """
    c = corners.reshape(-1, 2)
    up_left, up_right, down_right = c[0], c[cols - 1], c[-1]
    ab = up_left - up_right
    cb = down_right - up_right
    na, nc = np.linalg.norm(ab), np.linalg.norm(cb)
    if na < 1e-6 or nc < 1e-6:
        return 0.0
    ang = np.arccos(np.clip(float(np.dot(ab, cb)) / (na * nc), -1.0, 1.0))
    return float(min(1.0, 2.0 * abs(np.pi / 2.0 - ang)))


def band_of(t):
    return max(0, min(len(TILT_NAME) - 1, int(np.digitize(t, TILT_BINS) - 1)))


def set_camera(dev, exposure, brightness):
    """Force a SHORT manual exposure. This is the motion-blur fix.

    Measured, and it is not a preference. The camera sits in Aperture
    Priority auto by default, and in this room that chose
    `exposure_time_absolute = 2000` -- a 200 ms shutter. Every hand movement
    smears, the board stops being findable, and the operator concludes the
    detector is bad. It is not: on 21 real frames captured under that
    exposure, redetection was only 76 % at the BEST setting and 29-43 % with
    every alternative detector tried.

    Measured on this camera, manual, by shutter (mean brightness / clipped
    pixels / Laplacian sharpness):

        auto (exp 2000)                    26.6 mean          163 sharp
        exp 50, brightness 150            135.0 mean  6.5 %   122 sharp
        exp 50, brightness 255            220.2 mean 53.5 %    57 sharp

    So exp 50 (5 ms) with brightness 150 is the operating point: bright
    enough, barely clipping, sharp, and a shutter 40x shorter than auto
    chose. `gain` is INERT on this unit -- 20, 50 and 100 give identical
    frames -- so it is not offered as a knob.

    Applied with v4l2-ctl rather than cv2 properties because the ordering
    matters: `exposure_time_absolute` is ignored while auto is engaged, so
    auto must be turned off FIRST.
    """
    ok = True
    for k, v in (('auto_exposure', 1),
                 ('exposure_time_absolute', exposure),
                 ('brightness', brightness)):
        r = subprocess.run(['v4l2-ctl', '-d', str(dev),
                            '-c', f'{k}={v}'], capture_output=True)
        ok = ok and r.returncode == 0
    return ok


def detect_loop(st, det_w):
    """Match the live pose against the current step, and take it when held.

    Three measurements that settle how this is tuned, so the next person
    does not re-derive them:

    **480 px is the optimum, not a compromise.** Hit rate on 21 real board
    frames, with the per-frame cost:

        280px  38.1 %   41.5 ms      480px  76.2 %   49.9 ms   <- shipped
        320px  42.9 %   45.9 ms      560px  71.4 %   65.6 ms
        400px  47.6 %   61.7 ms     1280px  61.9 %  223.5 ms

    It beats everything smaller AND everything larger. Going finer to "see
    more detail" halves the hit rate.

    **Limiting OpenCV threads does nothing, MEASURED.** Detection at ~1.5 s
    on a busy scene looked like it was starving the capture loop of all four
    cores. A/B with `cv2.setNumThreads(2)`: **15.0 fps in both arms**. The
    hypothesis was wrong and the change is not shipped.

    **The frame rate is the CAMERA.** Raw capture with nothing else running
    at all is **15.0 fps** while the driver advertises 30 -- the Fantech's
    rate wanders (30.3 / 15.0 / 7.5 measured across sessions) and a replug is
    the known remedy. Carried ledger item 9. No amount of tuning here moves
    it.

    **`CALIB_CB_FAST_CHECK` is a wash**: 306.8 vs 307.4 ms on no-board
    frames at an identical 76.2 % hit rate. Not used -- a flag that buys
    nothing still costs the next reader an explanation.
    """
    held_since = None
    while not st.stopped:
        with st.lock:
            f = st.raw
            i = st.i
        if f is None or i >= len(st.steps):
            time.sleep(0.05)
            continue
        w, h = f.shape[1], f.shape[0]
        small = cv2.resize(f, (det_w, int(det_w * h / w)))
        gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
        t1 = time.time()
        found, c = cv2.findChessboardCorners(gray, (st.cols, st.rows), FIND)
        dt = time.time() - t1
        ty, tx, tb = st.steps[i]
        if not found:
            held_since = None
            with st.lock:
                st.corners, st.cell, st.band, st.hold = None, None, None, 0.0
                st.msg = 'show the whole board to the camera'
                st.det_ms = 1000 * dt
            continue
        c = cv2.cornerSubPix(gray, c, (7, 7), (-1, -1), CRIT)
        full = c / (det_w / w)
        cc = full.reshape(-1, 2)
        cx, cy = float(cc[:, 0].mean()), float(cc[:, 1].mean())
        band = band_of(tilt_of(full, st.cols))
        gx = min(GRID - 1, int(cx / w * GRID))
        gy = min(GRID - 1, int(cy / h * GRID))
        # Tilt is "at least", not "exactly". Demanding an exact band made
        # the steps very fiddly to satisfy -- the operator reported being
        # rejected repeatedly -- without buying anything: the script asks for
        # increasing tilt as it goes, so accepting MORE tilt than asked still
        # yields the spread the fit needs. The square-on steps stay exact,
        # because for those "more" is precisely what must not be accepted.
        okc = (gy, gx) == (ty, tx)
        okt = (band == 0) if tb == 0 else (band >= tb)
        # Sharpness is a capture gate, not something to discover in the
        # solve. Measured on 21 frames taken under the old auto-exposure:
        # median 7.30 px against OpenCV's documented < 3 px target, only 3
        # of 13 in spec. A blurred view is not a cheap view -- it biases the
        # corner positions the whole calibration is built from, and nothing
        # downstream can tell it apart from a good one.
        sharp = None
        if okc and okt:
            try:
                sharp = float(cv2.estimateChessboardSharpness(
                    gray, (st.cols, st.rows), c)[0])
            except Exception:
                sharp = None
        oks = sharp is None or sharp <= MAX_SHARPNESS_PX
        with st.lock:
            st.sharp = sharp
        if okc and okt and not oks:
            held_since = None
            prog = 0.0
            msg = (f'too blurred ({sharp:.1f} px, want <{MAX_SHARPNESS_PX:.0f})'
                   f' -- hold still, or add light')
        elif okc and okt:
            held_since = held_since or time.time()
            prog = min(1.0, (time.time() - held_since) / HOLD_S)
            msg = 'hold it...'
            if prog >= 1.0:
                fn = os.path.join(st.outdir, f'cal_{i:03d}.png')
                cv2.imwrite(fn, f)
                held_since = None
                prog = 0.0
                with st.lock:
                    st.taken.append((ty, tx, tb))
                    st.i += 1
                    msg = 'got it'
        else:
            held_since = None
            prog = 0.0
            msg = ('move the board to the highlighted box' if not okc
                   else ('tilt it MORE' if band < tb else
                         'hold it FLATTER (square-on to the camera)'))
        with st.lock:
            st.corners, st.cell, st.band = full, (gy, gx), band
            st.hold, st.msg, st.det_ms = prog, msg, 1000 * dt


def assess(st, cols, rows, square, size, fs):
    """Refit from what is captured, report Max ERE, and suggest the next pose.

    This is the aprilcal loop, and it replaces "capture 24 poses because a
    script says so" with "capture until the calibration stops being
    uncertain". Their measured result is 6-8 images to under 1 px, so a fixed
    24 is both slower than necessary and no guarantee.

    Runs in a thread: a fit plus ~12 candidate evaluations is ~6 s on the Pi,
    and the video must not stall for it. `fs` is the solver module.
    """
    objp = fs.board_points(cols, rows, square)
    while not st.stopped:
        with st.lock:
            files = sorted(glob.glob(os.path.join(st.outdir, 'cal_*.png')))
            n_seen = st.assessed_n
        if len(files) < 6 or len(files) == n_seen:
            time.sleep(1.0)
            continue
        try:
            ips, _, sz = fs.detect(files, cols, rows)
            if len(ips) < 6:
                raise ValueError('too few boards')
            samples = fs.kfold_std(objp, ips, sz, 4)
            ere, worst = fs.max_ere(samples, sz[0], sz[1])
            # Live intrinsics + the conditioning checks, so the operator can
            # see the answer FORM rather than waiting for a verdict at the
            # end. A calibration that is going wrong is visible early: fx
            # wandering between folds is the tell.
            fit = fs.fit_standard(objp, ips, sz)
            Kf = fit['K']
            fxs = np.array([K[0, 0] for K, _ in samples])
            v = fs.fov_from_K(Kf, sz[0], sz[1])
            # Where the model is least certain, as a PIXEL, so it can be
            # drawn. A metric you can point at is an instruction.
            wpx = None
            if worst is not None:
                wpx = (float(worst[0] * Kf[0, 0] + Kf[0, 2]),
                       float(worst[1] * Kf[1, 1] + Kf[1, 2]))
            cands = [((y, x), t) for y in range(GRID) for x in range(GRID)
                     for t in (10, 25, 40)]
            best, pred, _ = fs.suggest_next_pose(objp, ips, sz, cands)
            with st.lock:
                st.ere, st.ere_pred = ere, pred
                st.K, st.D = Kf, fit['D']
                st.fx, st.fy = float(Kf[0, 0]), float(Kf[1, 1])
                st.cx, st.cy = float(Kf[0, 2]), float(Kf[1, 2])
                st.fx_sd = float(fxs.std(ddof=1)) if len(fxs) > 1 else None
                st.hfov_air = float(v['hfov_air'])
                st.hfov_water = float(v['hfov_water'])
                st.worst_px = wpx
                st.assessed_n = len(files)
                st.n_boards = len(ips)
                if best is not None:
                    st.suggest = best
        except Exception as exc:
            with st.lock:
                st.assessed_n = len(files)
                st.assess_err = str(exc)[:120]
        time.sleep(0.5)


def resume(st):
    """Continue from frames already on disk, so a restart is not destructive.

    The files ARE the state -- `cal_<step>.png` is named for the step that
    produced it, so resuming is just "which step numbers exist". Without
    this, a restart begins at step 0 and overwrites the operator's work one
    frame at a time while the page counts up from zero, with no error.
    """
    done = {int(os.path.basename(p)[4:7])
            for p in glob.glob(os.path.join(st.outdir, 'cal_*.png'))}
    i = 0
    while i in done:
        i += 1
    with st.lock:
        st.i = i
        if i:
            st.msg = f'resumed at step {i + 1}'
