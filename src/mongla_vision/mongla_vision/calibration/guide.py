#!/usr/bin/env python3
"""Step-by-step guided camera calibration in a browser.

⛔ WHY A SCRIPT OF POSES AND NOT A FREEFORM CAPTURE. A calibration fails in a
way that LOOKS LIKE SUCCESS: too little tilt leaves focal length and distance
confounded, and the fit then returns a LOW reprojection RMS with a WRONG
focal length. That is how this project's downward camera produced fx = 835.7,
969.9 and 1011.2 on three consecutive tries, each with a confident metric.

A freeform version of this tool was tried first, showing coverage and tilt as
bars for the operator to fill. Watched live it reached **9/9 coverage with 15
of 17 views FLAT** -- the operator naturally holds a board square-on, and the
one axis that decides correctness is the one that goes unfilled. Bars report
the problem; they do not prevent it.

So the capture is a SCRIPT. Each step names one pose -- a frame region and a
tilt -- checks the live detection against it, and only advances when it is
actually met. Coverage and tilt spread are then guaranteed by construction,
and the operator never has to work out what is missing.

Layout: streaming and detection run on separate threads, and detection works
on a downscale. Inline full-resolution detection made the video unusable
(with no board in frame the detector searches exhaustively before failing --
142 s per call at 1280x720 on random noise, ~112 ms at 480 px on a real
scene). The saved frame is full resolution and `fov_solve` re-detects it
there, so the downscale costs nothing.

ThreadingHTTPServer, not HTTPServer: an MJPEG handler never returns and would
starve every other route.

Usage (stop the vision launch first -- it holds the cameras):
    python3 tools/fov_calibrate_web.py --device 3 --out ~/fantech_cal
"""
import argparse
import glob
import re
import signal
import socket
import json
import os
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

def _load_solver():
    """The solver, as a sibling module.

    It used to be loaded by file path because both halves lived in `tools/`.
    Inside the package it is an ordinary import, and the point stands either
    way: ONE implementation, two callers. A second copy of the geometry is
    how the simulator's scorer came to grade a board that no longer existed.
    """
    try:
        from . import solver                      # installed package
    except ImportError:
        # Running the file directly (the tools/ shim): load the sibling by
        # path rather than by package, so the calibration maths never needs
        # a sourced ROS to run.
        import importlib.util
        here = os.path.dirname(os.path.abspath(__file__))
        spec = importlib.util.spec_from_file_location(
            'mongla_calib_solver', os.path.join(here, 'solver.py'))
        solver = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(solver)
    return solver


FIND = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
CRIT = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
GRID = 3
# ⛔ TILT BANDS ARE ROS'S SKEW, AND THE OLD METRIC WAS BROKEN.
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
# ON PURPOSE. calib.io's guidance is explicit: use up to +/-45 degrees,
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
        self.frame = None
        self.raw = None
        self.corners = None
        self.taken = []                # (y, x, band) actually captured
        self.msg = 'starting...'
        self.err = None
        self.fps = 0.0
        self.det_ms = 0.0
        self.cell = None               # live: which cell the board is in
        self.band = None               # live: which tilt band
        self.hold = 0.0                # 0..1 progress of the hold timer
        self.sharp = None              # OpenCV chessboard sharpness, px
        self.device = None
        # AIR or WATER. Recorded, shown, and part of the filename, because
        # the two differ by ~1.44x and mixing them is a silent 44 % scale
        # error on every range the vehicle computes.
        self.medium = 'air'
        self.undistort = False     # preview the correction, live
        self.K = self.D = None     # latest fit, for that preview
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

    ⛔ MEASURED, AND IT IS NOT A PREFERENCE. The camera sits in Aperture
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
        r = subprocess.run(['v4l2-ctl', '-d', f'/dev/video{dev}',
                            '-c', f'{k}={v}'], capture_output=True)
        ok = ok and r.returncode == 0
    return ok


def capture_loop(st, dev, width, height, jpeg_w, exposure, brightness):
    """Read and stream only. Detection lives in its own thread, deliberately."""
    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    if not cap.isOpened():
        with st.lock:
            st.err = (f'cannot open camera {dev} -- the vision launch holds '
                      f'the cameras, stop it first.')
        return
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    # AFTER opening: cv2 resets the controls when it configures the stream,
    # so setting them before this point is silently undone.
    if exposure:
        set_camera(dev, exposure, brightness)
        time.sleep(0.5)
        for _ in range(5):
            cap.read()
    n, t0 = 0, time.time()
    while True:
        ok, f = cap.read()
        if not ok:
            time.sleep(0.01)
            continue
        with st.lock:
            st.raw = f
            corners = st.corners
            i = st.i
            worst = st.worst_px
            undist = st.undistort
            K, D = st.K, st.D
        shown = f
        if undist and K is not None:
            # ⛔ THE ONLY CHECK AN OPERATOR CAN MAKE WITH THEIR EYES. Every
            # other number here is a statistic; this is the calibration doing
            # its job. Straight edges in the world must come out straight --
            # if the frame edges still bow, the distortion model is wrong and
            # no reprojection RMS will say so.
            try:
                shown = cv2.undistort(f, K, D)
            except Exception:
                shown = f
        vis = cv2.resize(shown, (jpeg_w, int(jpeg_w * h / w)))
        vh = vis.shape[0]
        sc = jpeg_w / w
        if corners is not None:
            cv2.drawChessboardCorners(vis, (st.cols, st.rows),
                                      (corners * sc).astype(np.float32), True)
        # Only the TARGET region is drawn. Nine boxes was a puzzle to read;
        # one box is an instruction.
        if i < len(st.steps):
            ty, tx, _ = st.steps[i]
            x0, y0 = int(tx * jpeg_w / GRID), int(ty * vh / GRID)
            x1, y1 = int((tx + 1) * jpeg_w / GRID), int((ty + 1) * vh / GRID)
            cv2.rectangle(vis, (x0 + 3, y0 + 3), (x1 - 3, y1 - 3),
                          (80, 220, 255), 3)
        # The least-certain point, drawn. Max ERE says WHERE the model is
        # weakest; a number in a sidebar is a fact, a target on the video is
        # an instruction.
        if worst is not None:
            wx, wy = int(worst[0] * sc), int(worst[1] * sc)
            if 0 <= wx < jpeg_w and 0 <= wy < vh:
                cv2.circle(vis, (wx, wy), 16, (60, 120, 255), 2)
                cv2.line(vis, (wx - 22, wy), (wx - 8, wy), (60, 120, 255), 2)
                cv2.line(vis, (wx + 8, wy), (wx + 22, wy), (60, 120, 255), 2)
                cv2.putText(vis, 'least certain', (wx - 44, wy - 22),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (60, 120, 255), 1)
        if undist and K is not None:
            cv2.putText(vis, 'UNDISTORTED', (10, 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (60, 220, 60), 2)
        ok, jpg = cv2.imencode('.jpg', vis, [cv2.IMWRITE_JPEG_QUALITY, 70])
        n += 1
        if ok:
            with st.lock:
                st.frame = jpg.tobytes()
                if time.time() - t0 >= 1.0:
                    st.fps = n / (time.time() - t0)
                    n, t0 = 0, time.time()


def detect_loop(st, det_w):
    """Match the live pose against the current step, and take it when held.

    ⛔ THREE MEASUREMENTS THAT SETTLE HOW THIS IS TUNED, so the next person
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
    while True:
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
        # ⛔ TILT IS "AT LEAST", NOT "EXACTLY". Demanding an exact band made
        # the steps very fiddly to satisfy -- the operator reported being
        # rejected repeatedly -- without buying anything: the script asks for
        # increasing tilt as it goes, so accepting MORE tilt than asked still
        # yields the spread the fit needs. The square-on steps stay exact,
        # because for those "more" is precisely what must not be accepted.
        okc = (gy, gx) == (ty, tx)
        okt = (band == 0) if tb == 0 else (band >= tb)
        # ⛔ SHARPNESS IS A CAPTURE GATE, not something to discover in the
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


def assess(st, cols, rows, square, size):
    """Refit from what is captured, report Max ERE, and suggest the next pose.

    ⛔ THIS IS THE APRILCAL LOOP, and it replaces "capture 24 poses because a
    script says so" with "capture until the calibration stops being
    uncertain". Their measured result is 6-8 images to under 1 px, so a fixed
    24 is both slower than necessary and no guarantee.

    Runs in a thread: a fit plus ~12 candidate evaluations is ~6 s on the Pi,
    and the video must not stall for it.
    """
    fs = _load_solver()
    objp = fs.board_points(cols, rows, square)
    while True:
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


def run_solve(st, argv, dest):
    with st.lock:
        st.solve, st.solve_out = 'running', 'solving...'
    try:
        r = subprocess.run(argv, capture_output=True, text=True, timeout=900)
        out = (r.stdout or '') + (r.stderr or '')
        ok = r.returncode == 0 and os.path.exists(dest)
        with st.lock:
            st.solve = 'done' if ok else 'failed'
            st.solve_out = out[-4000:]
    except Exception as exc:
        with st.lock:
            st.solve, st.solve_out = 'failed', str(exc)


PAGE = """<!doctype html><meta charset=utf-8>
<title>Camera calibration</title>
<style>
 :root{--bg:#0e1116;--fg:#e6edf3;--dim:#8b949e;--ok:#3fb950;--no:#f85149;
       --line:#21262d;--acc:#58a6ff;--warn:#d29922}
 *{box-sizing:border-box}
 body{margin:0;background:var(--bg);color:var(--fg);
      font:14px/1.55 ui-monospace,SFMono-Regular,Menlo,monospace}
 header{padding:12px 20px;border-bottom:1px solid var(--line);display:flex;
        gap:16px;align-items:baseline}
 h1{font-size:14px;margin:0;letter-spacing:.09em;text-transform:uppercase}
 .sub{color:var(--dim);font-size:12px}
 main{display:grid;grid-template-columns:minmax(0,1fr) 340px}
 .vid{padding:16px}
 img{width:100%;display:block;border:1px solid var(--line);border-radius:4px}
 aside{padding:18px;border-left:1px solid var(--line)}
 .step{font-size:12px;color:var(--dim);letter-spacing:.1em;
       text-transform:uppercase}
 .todo{font-size:21px;line-height:1.35;margin:6px 0 14px}
 .todo b{color:var(--acc)}
 .chk{display:flex;gap:10px;align-items:center;margin:6px 0;font-size:13px}
 .dot{width:9px;height:9px;border-radius:50%;background:#30363d;flex:none}
 .dot.on{background:var(--ok)} .dot.off{background:var(--no)}
 .msg{margin:14px 0;padding:9px 11px;border-radius:4px;background:#161b22;
      border-left:3px solid var(--warn);font-size:13px;min-height:38px}
 .bar{height:6px;background:#161b22;border-radius:3px;overflow:hidden;
      margin:10px 0 4px}
 .bar i{display:block;height:100%;background:var(--acc);width:0;
        transition:width .12s linear}
 .prog{height:6px;background:#161b22;border-radius:3px;overflow:hidden}
 .prog i{display:block;height:100%;background:var(--ok);width:0}
 button{width:100%;padding:10px;border:0;border-radius:4px;background:var(--acc);
        color:#04121f;font:inherit;font-weight:700;cursor:pointer;margin-top:6px}
 button:disabled{background:#21262d;color:var(--dim);cursor:not-allowed}
 button.ghost{background:#21262d;color:var(--dim);font-weight:400}
 table#ans{width:100%;border-collapse:collapse;font-size:12px}
 table#ans td{padding:2px 0;vertical-align:top}
 table#ans td:first-child{color:var(--dim);white-space:nowrap;padding-right:10px}
 table#ans td:last-child{text-align:right;font-variant-numeric:tabular-nums}
 .good{color:var(--ok)} .bad{color:var(--no)} .warn2{color:var(--warn)}
 .med{font:700 15px/1.6 ui-monospace,monospace;letter-spacing:.16em;
 padding:6px 10px;border-radius:6px;text-align:center;margin-bottom:8px}
.med.air{background:#12331e;color:#7fe3a1;border:1px solid #2c6b41}
.med.water{background:#0d2740;color:#79c6ff;border:1px solid #2b5f8c}
.lib{border:1px solid #333;border-radius:6px;padding:7px 9px;margin:6px 0}
.cam{border:1px solid var(--line);border-radius:4px;padding:8px 10px;
      margin-bottom:8px;font-size:12px}
 .cam.act{border-color:var(--acc)}
 .cam b{font-weight:600} .cam .d{color:var(--dim)}
 .cam button{width:auto;padding:4px 10px;margin-top:6px;font-size:11px}
 .btnrow{display:flex;gap:6px;margin-top:6px}
 .btnrow button{margin-top:0}
 .btnrow a{flex:1;text-decoration:none}
 .btnrow input{flex:1;min-width:0;background:#161b22;border:1px solid var(--line);
               color:var(--fg);border-radius:4px;padding:6px;font:inherit;font-size:12px}
 pre{background:#161b22;padding:10px;border-radius:4px;font-size:11px;
     max-height:34vh;overflow:auto;white-space:pre-wrap;margin-top:8px}
 h2{font-size:11px;letter-spacing:.12em;text-transform:uppercase;
    color:var(--dim);margin:20px 0 6px}
</style>
<header><h1>Calibration</h1>
 <span class=sub>each step is checked before it counts</span>
 <span class=sub id=perf style="margin-left:auto"></span></header>
<main>
 <div class=vid><img src="/video"></div>
 <aside>
  <div class=step id=stepno></div>
  <div class=todo id=todo></div>
  <div class=chk><span class=dot id=dc></span><span id=tc></span></div>
  <div class=chk><span class=dot id=dt></span><span id=tt></span></div>
  <div class=chk><span class=dot id=ds></span><span id=ts></span></div>
  <div class=prog><i id=hold></i></div>
  <div class=msg id=msg></div>
  <h2>Medium</h2>
  <div id=medbar class=med>AIR</div>
  <div class=btnrow>
    <button class=ghost id=mair>Calibrate in AIR</button>
    <button class=ghost id=mwat>Calibrate in WATER</button>
  </div>
  <div class=sub>air and water differ by ~1.44&times; on this hull. The
   medium is written into the file and onto its name, so the two can never
   be confused. Calibrate in air unless you are validating.</div>
  <h2>Cameras on this vehicle</h2>
  <div id=cams class=sub>scanning…</div>
  <h2>Saved calibrations</h2>
  <div class=d id=libdir></div>
  <div id=lib class=sub>scanning…</div>
  <h2>The answer so far</h2>
  <table id=ans><tr><td colspan=2 class=sub>needs 6 usable views</td></tr></table>
  <h2>Certainty <span class=sub>(AprilCal Max ERE)</span></h2>
  <div class=bar><i id=eb></i></div>
  <div class=sub id=et></div>
  <div class=sub id=sg style="margin-top:6px"></div>
  <h2>Progress</h2>
  <div class=bar><i id=pb></i></div>
  <div class=sub id=pt></div>
  <button class=ghost id=skip>Skip this pose</button>
  <div class=btnrow>
    <button class=ghost id=undo>Delete last</button>
    <button class=ghost id=undist>Undistorted view</button>
  </div>
  <h2>Board</h2>
  <div class=btnrow>
    <input id=gr size=6 title="inner corners, e.g. 8x6">
    <input id=sq size=7 title="square size in metres">
    <button class=ghost id=setb>Apply</button>
  </div>
  <div class=sub>a wrong grid detects NOTHING and looks like bad lighting</div>
  <h2>Finish</h2>
  <button id=go>Run calibration</button>
  <div class=btnrow>
    <a id=dl href="/calibration.json" download><button class=ghost
       style="width:100%">Download result</button></a>
    <button class=ghost id=rst>Reset all captures</button>
  </div>
  <div class=sub id=sv style="margin-top:6px"></div>
  <pre id=out></pre>
 </aside>
</main>
<script>
const $=i=>document.getElementById(i);
async function tick(){
 try{
  const s=await (await fetch('/status')).json();
  $('perf').textContent=s.fps.toFixed(0)+' fps · detect '+s.det_ms.toFixed(0)+' ms';
  if(s.done){
    $('stepno').textContent='all poses captured';
    $('todo').innerHTML='Press <b>Run calibration</b>.';
    $('tc').textContent='';$('tt').textContent='';$('ts').textContent='';
  }else{
    $('stepno').textContent='Step '+(s.i+1)+' of '+s.n;
    $('todo').innerHTML='Put the board in the <b>'+s.want_cell+
      '</b> of the frame,<br>and hold it <b>'+s.want_tilt+'</b>.';
    $('tc').textContent='position: '+(s.cell? s.cell : 'no board');
    $('tt').textContent='tilt: '+(s.band!==null? s.band : '-');
    $('ts').textContent = s.sharp===null ? 'sharpness: -'
        : 'sharpness: '+s.sharp.toFixed(1)+' px (want <'+s.sharp_max+')';
    $('ds').className='dot '+(s.sharp===null?'':(s.sharp<=s.sharp_max?'on':'off'));
    $('dc').className='dot '+(s.ok_cell?'on':'off');
    $('dt').className='dot '+(s.ok_tilt?'on':'off');
  }
  $('hold').style.width=(100*s.hold)+'%';
  $('msg').textContent=s.err||s.msg;
  $('pb').style.width=(100*s.i/s.n)+'%';
  $('pt').textContent=s.i+' of '+s.n+' poses captured ('+s.n_boards+' usable)';
  // Certainty bar: full when Max ERE is under the bar. Log scale, because
  // it starts in the tens of pixels and the last factor of two is the part
  // that matters.
  let pct=0, txt='need 6 usable views before this can be computed';
  if (s.ere!==null && isFinite(s.ere)) {
    pct = Math.max(0, Math.min(100, 100*(1 - Math.log10(Math.max(s.ere,s.ere_bar))
                                          /Math.log10(40))));
    txt = 'Max ERE '+s.ere.toFixed(2)+' px  (bar <'+s.ere_bar.toFixed(1)+')';
    if (s.ere<=s.ere_bar) txt += '  — DONE, calibration is certain';
  }
  $('eb').style.width=pct+'%';
  $('et').textContent=txt;
  // The answer as it forms, with a verdict per line, so a calibration that
  // is going wrong is visible EARLY rather than at the end.
  const rows=[];
  if (s.fx!==null){
    const sdpct = s.fx_sd!==null ? 100*s.fx_sd/s.fx : null;
    const cls = sdpct===null ? '' : (sdpct<0.5?'good':(sdpct<1.5?'warn2':'bad'));
    rows.push(['focal fx / fy', s.fx.toFixed(1)+' / '+s.fy.toFixed(1)+' px','']);
    rows.push(['centre cx / cy', s.cx.toFixed(0)+' / '+s.cy.toFixed(0)+' px','']);
    if (sdpct!==null)
      rows.push(['fx spread across folds', sdpct.toFixed(2)+' %', cls]);
    rows.push(['HFOV in air', s.hfov_air.toFixed(2)+'°','']);
    rows.push(['HFOV in water', s.hfov_water.toFixed(2)+'°','good']);
    rows.push(['usable views', s.n_boards, s.n_boards>=10?'good':'warn2']);
  }
  $('ans').innerHTML = rows.length
    ? rows.map(r=>'<tr><td>'+r[0]+'</td><td class="'+r[2]+'">'+r[1]+'</td></tr>').join('')
    : '<tr><td colspan=2 class=sub>needs 6 usable views</td></tr>';
  $('sg').textContent = s.suggest
      ? 'solver suggests next: '+s.suggest+
        (s.ere_pred? '  (predicts '+s.ere_pred.toFixed(1)+' px)':'')
      : '';
  const enough = (s.ere!==null && isFinite(s.ere) && s.ere<=s.ere_bar) || s.done;
  const b=$('go'); b.disabled=(s.solve==='running')||!enough;
  b.textContent=s.solve==='running'?'solving...':
    (!enough?'keep capturing — not certain yet':
     (s.solve==='done'?'Re-run calibration':'Run calibration'));
  $('sv').textContent = s.solve==='done'?'installed — rebuild to use it':
    s.solve==='failed'?'solve failed, see below':'';
  $('out').textContent=s.solve_out||'';
  $('skip').style.display=s.done?'none':'block';
  const mb=$('medbar'); mb.textContent=(s.medium||'air').toUpperCase();
  mb.className='med '+(s.medium||'air');
  $('undist').textContent = s.undistort?'Raw view':'Undistorted view';
  $('undist').disabled = s.fx===null;
  $('dl').style.pointerEvents = s.have_result?'auto':'none';
  $('dl').style.opacity = s.have_result?1:0.4;
  if(!$('gr').matches(':focus') && !$('gr').value) $('gr').value=s.grid;
  if(!$('sq').matches(':focus') && !$('sq').value) $('sq').value=s.square||'';
 }catch(e){}
}
$('go').onclick=async()=>{await fetch('/solve',{method:'POST'});tick();};
$('undo').onclick=async()=>{await fetch('/undo',{method:'POST'});tick();};
$('undist').onclick=async()=>{await fetch('/undistort',{method:'POST'});tick();};
$('rst').onclick=async()=>{
  if(confirm('Delete every capture and start over?')){
    await fetch('/reset',{method:'POST'});tick();}};
$('setb').onclick=async()=>{
  const g=$('gr').value.trim(), q=$('sq').value.trim();
  if(!g&&!q) return;
  $('msg').textContent='restarting with the new board…';
  await fetch('/board?grid='+encodeURIComponent(g)+'&square='+encodeURIComponent(q),
              {method:'POST'});
  setTimeout(()=>location.reload(), 5000);};
$('skip').onclick=async()=>{await fetch('/skip',{method:'POST'});tick();};
async function cams(){
 try{
  const r=await (await fetch('/cameras')).json();
  window._cams=r.cameras;
  document.getElementById('cams').innerHTML = r.cameras.length ? r.cameras.map(c=>{
    const active = String(c.device).endsWith(String(r.active));
    const cal=c.calibration;
    let line, cls;
    if(!cal){ line='<span class=bad>NO CALIBRATION</span>'; cls='bad'; }
    else if(c.match==='confirmed'){
      line='<span class=good>calibrated</span> · fx '+cal.fx.toFixed(0)+
           ' · HFOV '+cal.hfov_air.toFixed(1)+'° air / '+cal.hfov_water.toFixed(1)+'° water'+
           (cal.max_ere_px?' · ERE '+cal.max_ere_px.toFixed(1)+'px':'')+
           ' · '+(cal.captured||'?'); }
    else { line='<span class=warn2>UNVERIFIED</span> — '+cal.file+
           ' fits by name but records no camera identity, so it cannot be'+
           ' proven to be this unit'; }
    const prof = cal && cal.applies_to ? cal.applies_to[0]
               : (String(c.card||'').toLowerCase().includes('fantech')
                  ? 'pi_forward' : 'pi_downward');
    return '<div class="cam'+(active?' act':'')+'">'+
      '<b>'+(c.card||c.device)+'</b> <span class=d>'+c.device+
      ' · usb '+(c.usb_vid||'?')+':'+(c.usb_pid||'?')+
      (active?' · ACTIVE':'')+'</span><br>'+line+
      (active?'':'<button onclick="sw(\\''+c.device+'\\',\\''+prof+'\\')">'+
        (cal?'Re-calibrate':'Calibrate')+' this camera</button>')+'</div>';
  }).join('') : '<span class=bad>no USB cameras found</span>';
 }catch(e){}
}
$('mair').onclick=()=>setmed('air');
$('mwat').onclick=()=>setmed('water');
async function setmed(m){
  if(m==='water' && !confirm(
     'Calibrate IN WATER?\\n\\nThe normal path is to calibrate in air once '+
     'and correct refraction analytically. An in-water calibration is a '+
     'VALIDATION of that correction. It is saved separately and never '+
     'overwrites the air one.')) return;
  $('msg').textContent='restarting in '+m+'…';
  await fetch('/switch?medium='+m,{method:'POST'});
  setTimeout(()=>location.reload(), 5000);
}
window._cams=[];
async function lib(){
 try{
  const r=await (await fetch('/library')).json();
  document.getElementById('lib').innerHTML = r.library.length ? r.library.map(c=>{
    // Only offer to apply a calibration to a camera that is actually here.
    // Applying to an absent device would write a file claiming a unit
    // nobody can check.
    const btns=(window._cams||[]).map(cam=>
      '<button onclick="ap(\\''+c.file+'\\',\\''+cam.device+'\\',\\''+
      ((c.applies_to&&c.applies_to[0])||'pi_forward')+'\\')">apply to '+
      (cam.card||cam.device)+'</button>').join(' ');
    return '<div class=lib><b>'+c.file+'</b> <span class=d>'+
      (c.medium||'air').toUpperCase()+' · '+c.width+'x'+c.height+
      (c.fx?' · fx '+c.fx.toFixed(0):'')+
      (c.views?' · '+c.views+' views':'')+
      (c.max_ere_px?' · ERE '+c.max_ere_px.toFixed(1)+'px':'')+
      '</span><br><span class=sub>'+(c.camera||'no camera recorded')+
      ' · '+(c.captured||'?')+'</span><br>'+btns+'</div>';
  }).join('') : '<span class=sub>none saved yet</span>';
  // Name the directory on the page. It is resolved by walking up for
  // `src/mongla_vision/config/calibration`, and if that walk ever fails the
  // fallback is a folder OUTSIDE the package -- which would look exactly
  // like an empty library.
  document.getElementById('libdir').textContent = r.dir;
 }catch(e){}
}
async function ap(file,dev,prof){
  if(!confirm('Apply '+file+' to '+dev+'?\\n\\nThis asserts they are the SAME '+
    'physical camera. The file records that YOU asserted it, not that it '+
    'was measured here.')) return;
  await fetch('/apply?file='+encodeURIComponent(file)+'&device='+
    encodeURIComponent(dev)+'&applies_to='+encodeURIComponent(prof),
    {method:'POST'});
  setTimeout(()=>{lib();cams();}, 500);
}
async function sw(dev,prof){
  const n=dev.replace(/\\D+/g,'');
  document.getElementById('cams').innerHTML='<span class=warn2>switching to '+dev+'…</span>';
  await fetch('/switch?device='+n+'&applies_to='+prof,{method:'POST'});
  setTimeout(()=>location.reload(), 5000);
}
setInterval(tick,300);tick();
setInterval(cams,4000);cams();
setInterval(lib,6000);lib();
</script>
"""


def make_handler(st, solve_argv, solve_dest, cal_dir, restart):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _json(self):
            with st.lock:
                i, n = st.i, len(st.steps)
                done = i >= n
                ty, tx, tb = st.steps[min(i, n - 1)]
                cell, band = st.cell, st.band
                d = {
                    'i': i, 'n': n, 'done': done,
                    'want_cell': CELL_NAME[ty][tx], 'want_tilt': TILT_NAME[tb],
                    'cell': CELL_NAME[cell[0]][cell[1]] if cell else None,
                    'band': TILT_NAME[band] if band is not None else None,
                    'ok_cell': bool(cell == (ty, tx)),
                    'ok_tilt': bool(band == tb),
                    'hold': st.hold, 'msg': st.msg, 'err': st.err,
                    'sharp': st.sharp, 'sharp_max': MAX_SHARPNESS_PX,
                    'undistort': st.undistort, 'medium': st.medium,
                    'grid': f'{st.cols}x{st.rows}',
                    'square': st.square,
                    'have_result': os.path.exists(solve_dest),
                    'ere': st.ere, 'ere_pred': st.ere_pred,
                    'ere_bar': ERE_BAR_PX, 'n_boards': st.n_boards,
                    'fx': st.fx, 'fy': st.fy, 'cx': st.cx, 'cy': st.cy,
                    'fx_sd': st.fx_sd, 'hfov_air': st.hfov_air,
                    'hfov_water': st.hfov_water,
                    'suggest': (f'{CELL_NAME[st.suggest[0][0]][st.suggest[0][1]]}'
                                f' at ~{st.suggest[1]} deg'
                                if st.suggest else None),
                    'fps': round(st.fps, 1), 'det_ms': round(st.det_ms, 1),
                    'solve': st.solve, 'solve_out': st.solve_out,
                }
            return json.dumps(d).encode()

        def do_GET(self):
            if self.path == '/':
                b = PAGE.encode()
                self.send_response(200)
                self.send_header('Content-Type', 'text/html; charset=utf-8')
                self.send_header('Content-Length', str(len(b)))
                self.end_headers()
                self.wfile.write(b)
            elif self.path == '/calibration.json':
                # Download the installed calibration. A calibration is
                # expensive to make and trivial to lose; it should be one
                # click to keep a copy off the vehicle.
                try:
                    b = open(solve_dest, 'rb').read()
                except Exception:
                    self.send_response(404); self.end_headers(); return
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Disposition',
                                 'attachment; filename="'
                                 + os.path.basename(solve_dest) + '"')
                self.send_header('Content-Length', str(len(b)))
                self.end_headers()
                self.wfile.write(b)
            elif self.path == '/library':
                b = json.dumps({'library': library(cal_dir),
                                'dir': cal_dir}).encode()
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(b)))
                self.end_headers()
                self.wfile.write(b)
            elif self.path == '/cameras':
                b = json.dumps({
                    'cameras': list_cameras(cal_dir),
                    'active': st.device,
                }).encode()
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(b)))
                self.end_headers()
                self.wfile.write(b)
            elif self.path == '/status':
                b = self._json()
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(b)))
                self.end_headers()
                self.wfile.write(b)
            elif self.path == '/video':
                self.send_response(200)
                self.send_header('Content-Type',
                                 'multipart/x-mixed-replace; boundary=f')
                self.end_headers()
                try:
                    while True:
                        with st.lock:
                            f = st.frame
                        if f:
                            self.wfile.write(
                                b'--f\r\nContent-Type: image/jpeg\r\n'
                                b'Content-Length: ' + str(len(f)).encode()
                                + b'\r\n\r\n' + f + b'\r\n')
                        time.sleep(0.02)
                except Exception:
                    pass
            else:
                self.send_response(404)
                self.end_headers()

        def do_POST(self):
            if self.path.startswith('/switch'):
                # ⛔ RE-EXEC RATHER THAN RETARGET THE THREADS. The capture
                # thread owns an open V4L2 handle and the detector owns an
                # anchor; unwinding both live is a race for no gain. The
                # process replaces itself with the same argv bar the camera,
                # which reuses the reaper, the resume, and every check that
                # runs at startup -- one code path, not two.
                from urllib.parse import urlparse, parse_qs
                q = parse_qs(urlparse(self.path).query)
                dev = q.get('device', [None])[0]
                prof = q.get('applies_to', [None])[0]
                med = q.get('medium', [None])[0]
                self.send_response(204)
                self.end_headers()
                # A medium change alone is a valid switch -- the camera does
                # not move when the hull goes in the water, the refraction
                # does.
                if dev is not None or med is not None:
                    threading.Thread(
                        target=restart,
                        kwargs={'device': dev, 'applies_to': prof,
                                'medium': med},
                        daemon=True).start()
                return
            if self.path.startswith('/apply'):
                from urllib.parse import urlparse, parse_qs
                q = parse_qs(urlparse(self.path).query)
                name, err = apply_calibration(
                    cal_dir, q.get('file', [''])[0],
                    q.get('device', ['/dev/video0'])[0],
                    q.get('applies_to', ['pi_forward'])[0])
                with st.lock:
                    st.msg = (
                        f'applied -> {name}. The RUNNING vision node still '
                        f'holds the old intrinsics -- colcon build and '
                        f'relaunch before trusting a bearing.'
                        if name else f'apply refused: {err}')
                self.send_response(204); self.end_headers(); return
            if self.path == '/undistort':
                with st.lock:
                    st.undistort = not st.undistort
                self.send_response(204); self.end_headers(); return
            if self.path == '/undo':
                # Drop the LAST capture. A view you know was bad -- a hand in
                # frame, a wobble -- is worth deleting on the spot; leaving it
                # in and hoping RANSAC eats it is how a quiet bias gets in.
                with st.lock:
                    fs_ = sorted(glob.glob(os.path.join(st.outdir,
                                                        'cal_*.png')))
                    if fs_:
                        os.remove(fs_[-1])
                        st.i = max(0, st.i - 1)
                        st.assessed_n = -1        # force a refit
                        st.msg = f'deleted {os.path.basename(fs_[-1])}'
                self.send_response(204); self.end_headers(); return
            if self.path == '/reset':
                with st.lock:
                    for f_ in glob.glob(os.path.join(st.outdir, 'cal_*.png')):
                        os.remove(f_)
                    st.i = 0
                    st.ere = st.ere_pred = st.K = None
                    st.assessed_n = -1
                    st.n_boards = 0
                    st.msg = 'reset -- all captures deleted'
                self.send_response(204); self.end_headers(); return
            if self.path.startswith('/board'):
                # The board can differ on the day. A tool that hardcodes the
                # grid is a tool that cannot be used with the board you
                # actually brought -- and a WRONG grid detects nothing while
                # looking like a lighting problem.
                from urllib.parse import urlparse, parse_qs
                q = parse_qs(urlparse(self.path).query)
                self.send_response(204); self.end_headers()
                threading.Thread(
                    target=restart,
                    kwargs={'device': None,
                            'grid': q.get('grid', [None])[0],
                            'square': q.get('square', [None])[0]},
                    daemon=True).start()
                return
            if self.path == '/skip':
                with st.lock:
                    if st.i < len(st.steps):
                        st.i += 1
                        st.msg = 'skipped -- that pose is missing from the set'
                self.send_response(204)
                self.end_headers()
            elif self.path == '/solve':
                with st.lock:
                    busy = st.solve == 'running'
                if not busy:
                    threading.Thread(target=run_solve,
                                     args=(st, solve_argv, solve_dest),
                                     daemon=True).start()
                self.send_response(204)
                self.end_headers()
            else:
                self.send_response(404)
                self.end_headers()
    return H


def camera_identity(dev):
    """Who this camera actually IS -- card name and USB VID:PID.

    ⛔ THIS IS THE FIELD THAT WOULD HAVE CAUGHT THE WRONG-CAMERA BUG. The one
    calibration this project held was named for one camera, described a
    second, and was wired to a third, for four days, and nothing could see it
    because the file did not record what it was taken with. `applies_to`
    fixed the wiring half; this fixes the provenance half -- a calibration
    now says which physical unit produced it, so a swapped camera is
    DETECTABLE rather than merely unlucky.
    """
    out = {'device': dev}
    try:
        r = subprocess.run(['v4l2-ctl', '-d', dev, '--info'],
                           capture_output=True, text=True, timeout=5)
        for line in (r.stdout or '').splitlines():
            if 'Card type' in line:
                out['card'] = line.split(':', 1)[1].strip()
            elif 'Bus info' in line:
                out['bus'] = line.split(':', 1)[1].strip()
    except Exception:
        pass
    try:
        real = os.path.realpath(dev)
        node = os.path.basename(real)
        base = f'/sys/class/video4linux/{node}/device/../'
        for key, fn in (('usb_vid', 'idVendor'), ('usb_pid', 'idProduct'),
                        ('usb_serial', 'serial')):
            try:
                out[key] = open(base + fn).read().strip()
            except Exception:
                pass
    except Exception:
        pass
    return out


def list_cameras(cal_dir):
    """Every capture-capable camera, with the calibration it has (or lacks).

    The competition-day question is not "what is my focal length", it is
    "which of my cameras can I trust right now". This answers that in one
    place: the units present, what each one is, whether a calibration exists
    for it, and whether that calibration was taken on THIS unit.
    """
    cams = []
    for dev in sorted(glob.glob('/dev/video*'),
                      key=lambda d: int(''.join(c for c in d if c.isdigit()) or 0)):
        try:
            r = subprocess.run(['v4l2-ctl', '-d', dev, '--list-formats'],
                               capture_output=True, text=True, timeout=5)
            txt = r.stdout or ''
            # Version-robust: this v4l2-ctl prints "[0]: 'MJPG' (...)" and no
            # "Pixel Format" string at all, which an earlier check assumed --
            # it found ZERO cameras on a machine with one plugged in. Require
            # a capture type AND at least one enumerated format.
            if 'Video Capture' not in txt or not re.search(r'\[\d+\]:', txt):
                continue                     # metadata node, not a stream
        except Exception:
            continue
        ident = camera_identity(dev)
        # USB ONLY. The Pi exposes ~18 `pispbe` ISP pipeline nodes that
        # enumerate formats and are not cameras; listing them buries the two
        # devices that matter under a page of noise. A USB vendor id is the
        # thing every real camera here has and no ISP node does.
        if not ident.get('usb_vid'):
            continue
        entry = {**ident, 'calibration': None, 'match': None}
        best, best_match = None, None
        for f in sorted(glob.glob(os.path.join(cal_dir, '*.json'))):
            try:
                d = json.load(open(f))
            except Exception:
                continue
            info = {
                'file': os.path.basename(f),
                'applies_to': d.get('applies_to'),
                'captured': d.get('captured'),
                'fx': (d.get('camera_matrix') or [[None]])[0][0],
                'hfov_air': d.get('hfov_deg_air'),
                'hfov_water': d.get('hfov_deg_water'),
                'max_ere_px': d.get('max_ere_px'),
                'views': d.get('views_used'),
            }
            # ⛔ THREE STATES, NOT TWO. "confirmed" needs the USB id recorded
            # AT CAPTURE and matching now. "unverified" is a file that plausibly
            # belongs but cannot prove it -- which is every calibration taken
            # before identity was recorded, and is exactly the state the
            # wrong-camera bug lived in for four days. Reporting it as
            # "calibrated" would repeat that; reporting it as "none" would
            # throw away a good calibration. So it gets its own word.
            if d.get('usb_vid'):
                if (d['usb_vid'] == ident.get('usb_vid') and
                        d.get('usb_pid') == ident.get('usb_pid')):
                    best, best_match = info, 'confirmed'
                    break
                continue                      # identity recorded and DIFFERENT
            card = (ident.get('card') or '').split(':')[0].strip().lower()
            blob = (str(d.get('camera') or '') + ' ' +
                    ' '.join(d.get('applies_to') or [])).lower()
            if card and card in blob and best is None:
                best, best_match = info, 'unverified'
        entry['calibration'], entry['match'] = best, best_match
        cams.append(entry)
    return cams


def default_calibration_dir():
    """Where calibrations live, found robustly.

    ⛔ THIS WAS WRONG WHEN INSTALLED, TWICE, AND FAILED QUIETLY BOTH TIMES.

    First version built `<this file>/../../src/mongla_vision/config/
    calibration`, which is right in the source tree and nonsense in
    `install/.../site-packages`. Download 404'd and the camera panel read NO
    CALIBRATION for a camera that has one.

    Second version walked upward looking for `src/mongla_vision/config/
    calibration` -- and under `ros2 run` on the vehicle it resolved to

        <ws>/build/mongla_vision/mongla_vision/src/mongla_vision/config/calibration

    an EMPTY directory **the tool had created itself** on an earlier run
    whose cwd happened to sit there. `main()` does `os.makedirs` on whatever
    this returns, so a resolver that creates the directory it searches for
    will find its own mistake forever after, and every symptom (empty
    library, 404 download, cameras showing NO CALIBRATION) points at missing
    calibrations rather than at a wrong path.

    So: **a candidate inside a generated tree is never valid.** `build/` and
    `install/` are wiped and rebuilt; a calibration written there is deleted
    by the next `colcon build` and can never be committed, which is the one
    thing this file must not allow. The source tree is the target.
    """
    def usable(cand):
        # Reject anything under a generated tree, however plausible.
        parts = os.path.abspath(cand).split(os.sep)
        return (os.path.isdir(cand)
                and 'build' not in parts and 'install' not in parts)

    here = os.path.dirname(os.path.abspath(__file__))
    starts = [os.getcwd(), here]
    # Under `ros2 run` this file lives inside <ws>/build/... or
    # <ws>/install/...; the workspace root is the parent of that, so name it
    # directly rather than hoping the upward walk survives the decoys.
    parts = here.split(os.sep)
    for gen in ('build', 'install'):
        if gen in parts:
            starts.insert(0, os.sep.join(parts[:parts.index(gen)]))
    for start in starts:
        d = start
        for _ in range(8):
            cand = os.path.join(d, 'src', 'mongla_vision', 'config',
                                'calibration')
            if usable(cand):
                return cand
            nd = os.path.dirname(d)
            if nd == d:
                break
            d = nd
    # Nothing found: a real directory the operator can find, never a path
    # that only looks plausible.
    fallback = os.path.expanduser('~/mongla_calibrations')
    os.makedirs(fallback, exist_ok=True)
    return fallback


def capture_dir(applies_to, medium):
    """Where this camera's frames for THIS medium live.

    ⛔ THE MEDIUM IS PART OF THE PATH, NOT JUST THE FILENAME. Switching to
    water re-execs and `resume()` picks up whatever is in `--out`. With one
    shared folder that is the AIR capture set, and Solve would install it as
    `..._water.json`: a filename saying water over pixels taken in air --
    the exact 1.44x confusion the whole medium flag exists to prevent,
    arriving through the front door.

    Same reasoning as the per-profile split that came before it: a switch
    must not resume into another capture's frames.
    """
    suffix = '' if medium == 'air' else f'_{medium}'
    return os.path.expanduser(f'~/calib_{applies_to}{suffix}')


def restart_argv(argv, cur_profile, cur_medium, device=None,
                 applies_to=None, grid=None, square=None, medium=None):
    """The argv a re-exec should run with. Pure, so it can be driven.

    ⛔ `--out` IS ALWAYS REWRITTEN, from BOTH the profile and the medium.
    An earlier version rewrote it only when the profile changed, so a
    medium-only switch re-execed pointing at the previous capture folder,
    `resume()` picked up the AIR frames, and Solve installed them as
    `..._water.json` -- a filename saying water over pixels taken in air,
    which is the exact 1.44x confusion the medium flag exists to prevent.
    Nothing in the naming or file-content tests can see that, because the
    artifact is correct and the INPUT is wrong.
    """
    new = list(argv)

    def setarg(flag, val):
        if val is None:
            return
        if flag in new:
            new[new.index(flag) + 1] = str(val)
        else:
            new.extend([flag, str(val)])

    setarg('--device', device)
    setarg('--applies-to', applies_to)
    setarg('--grid', grid)
    setarg('--square', square)
    setarg('--medium', medium)
    setarg('--out', capture_dir(applies_to or cur_profile,
                                medium or cur_medium))
    return new


def _solver():
    """The solver module, loaded BY PATH.

    ⛔ NOT `from mongla_vision.calibration.solver import ...`. `tools/` loads
    this file by path precisely so the calibration tool does not need a ROS
    environment; a package import re-enters `mongla_vision/__init__.py`,
    which imports `preflight`, which imports `rclpy` -- and the tool dies on
    a dev box with `ModuleNotFoundError: rclpy`. This is the same trap the
    shims were written to avoid, walked into from the other side.
    """
    import importlib.util
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        'solver.py')
    spec = importlib.util.spec_from_file_location('_calib_solver', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def library(cal_dir):
    """Every calibration this vehicle holds, with what it is good for.

    A calibration costs an operator ten minutes and a board. Keeping them all
    and being able to re-apply one is the difference between swapping a
    camera in two minutes and recalibrating on competition ground.
    """
    out = []
    for f in sorted(glob.glob(os.path.join(cal_dir, '*.json'))):
        try:
            d = json.load(open(f))
        except Exception:
            continue
        K = d.get('camera_matrix') or [[None]]
        out.append({
            'file': os.path.basename(f),
            'applies_to': d.get('applies_to'),
            'medium': d.get('medium', 'air'),
            'captured': d.get('captured'),
            'camera': d.get('camera'),
            'usb_vid': d.get('usb_vid'), 'usb_pid': d.get('usb_pid'),
            'width': d.get('image_width'), 'height': d.get('image_height'),
            'fx': K[0][0],
            'hfov_air': d.get('hfov_deg_air'),
            'hfov_water': d.get('hfov_deg_water'),
            'views': d.get('views_used'),
            'max_ere_px': d.get('max_ere_px'),
            'holdout_rms_px': d.get('holdout_rms_px'),
        })
    return out


def apply_calibration(cal_dir, src_file, dev, applies_to):
    """Bind an existing calibration to a connected camera.

    ⛔ APPLYING IS THE OPERATOR ASSERTING "this is the same physical unit",
    and the file must say so rather than pretend it was measured here. So the
    connected camera's identity is written in ALONGSIDE
    `identity_source: operator asserted at apply time` -- never as though it
    had been captured on this unit. The difference matters the day someone
    asks why a calibration claims a camera it never saw.

    Refuses to apply across MEDIA. An in-water calibration standing in for an
    in-air one is a ~1.44x scale error on every range and velocity, which is
    both silent and exactly the size that still looks plausible.
    """
    # A bad request must come back as a REFUSAL, not as a dropped
    # connection. The first live test of this route passed a file that was
    # not there and got an unhandled FileNotFoundError inside do_POST, which
    # the browser sees as the page hanging -- a failure mode indis-
    # tinguishable from the tool being broken.
    src = os.path.join(cal_dir, os.path.basename(src_file or ''))
    try:
        d = json.load(open(src))
    except Exception as e:
        return None, f'{src_file!r}: {e}'
    medium = d.get('medium', 'air')
    ident = camera_identity(dev)
    if not ident.get('usb_vid'):
        return None, f'{dev}: no USB identity, refusing to bind blindly'
    if not applies_to:
        return None, 'no camera profile given'
    d = dict(d)
    d['applies_to'] = [applies_to]
    # State the medium EXPLICITLY even when the source predates the field.
    # A file that merely omits it is read as air by convention, and a
    # convention is not evidence.
    d['medium'] = medium
    d.update({k: v for k, v in ident.items()
              if k in ('usb_vid', 'usb_pid', 'usb_serial', 'card', 'bus')})
    d['identity_source'] = 'operator asserted at apply time'
    d['applied_from'] = os.path.basename(src_file)
    d['applied_on'] = time.strftime('%Y-%m-%d %H:%M')
    dest = os.path.join(cal_dir, _solver().calibration_filename(
        applies_to, d.get('image_width'), d.get('image_height'), medium))
    if os.path.exists(dest) and os.path.abspath(dest) != os.path.abspath(src):
        # Never overwrite a calibration without keeping the old one. It cost
        # somebody a board and ten minutes.
        os.replace(dest, dest + '.bak-' + time.strftime('%H%M%S'))
    json.dump(d, open(dest, 'w'), indent=2)
    return os.path.basename(dest), None


def reap_previous(port, wait_s=6.0):
    """Kill any EARLIER instance of this tool, then wait for it to let go.

    ⛔ ON COMPETITION GROUND YOU DO NOT GO PID HUNTING. A second `ros2 run
    mongla_vision calibrate` used to die on `OSError: [Errno 98] Address
    already in use`, and the fix -- find the process, work out that there are
    TWO of them, kill both -- is exactly the wrong thing to be doing beside a
    pool with a run slot ticking. Restarting a tool must just work.

    TWO processes per instance, which is the trap: `ros2 run` execs the node
    as a CHILD, so killing the wrapper leaves the child holding both the port
    and the camera. That is the same defect that put three duplicate copies
    of every A/B arm on this machine earlier today, met a third time.

    Found two ways, because either alone has a hole: whoever holds the PORT
    (catches an instance started under a different name), and whoever LOOKS
    like this tool (catches one that has not bound yet, mid-startup).

    ⚠ Never kills anything in our OWN process group -- that includes the
    `ros2 run` wrapper that launched us, and killing it would take us with
    it.
    """
    me = os.getpid()
    try:
        my_pg = os.getpgid(0)
    except OSError:
        my_pg = None
    victims = set()

    out = subprocess.run(['ss', '-lntpH', f'sport = :{port}'],
                         capture_output=True, text=True)
    for m in re.finditer(r'pid=(\d+)', out.stdout or ''):
        victims.add(int(m.group(1)))

    for cl in glob.glob('/proc/[0-9]*/cmdline'):
        try:
            pid = int(cl.split('/')[2])
            cmd = open(cl, 'rb').read().replace(b'\x00', b' ').decode(
                'utf-8', 'replace')
        except Exception:
            continue
        if any(k in cmd for k in ('mongla_vision/calibrate',
                                  'mongla_vision calibrate',
                                  'calibration/guide.py',
                                  'fov_calibrate_web')):
            victims.add(pid)

    kept = []
    for v in list(victims):
        if v == me:
            continue
        try:
            if my_pg is not None and os.getpgid(v) == my_pg:
                continue                       # our own wrapper -- leave it
        except OSError:
            pass
        kept.append(v)
    if not kept:
        return []

    for sig in (signal.SIGTERM, signal.SIGKILL):
        alive = []
        for v in kept:
            try:
                os.kill(v, sig)
                alive.append(v)
            except OSError:
                pass
        if not alive:
            break
        t0 = time.time()
        while time.time() - t0 < wait_s / 2:
            if not any(os.path.exists(f'/proc/{v}') for v in alive):
                break
            time.sleep(0.2)
        if not any(os.path.exists(f'/proc/{v}') for v in alive):
            break
    # The port lingers briefly in TIME_WAIT even after the holder is gone.
    t0 = time.time()
    while time.time() - t0 < wait_s:
        probe = socket.socket()
        try:
            probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            probe.bind(('0.0.0.0', port))
            probe.close()
            break
        except OSError:
            probe.close()
            time.sleep(0.3)
    return kept


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--device', type=int, default=3)
    # Derived from the profile AND the medium unless given explicitly --
    # see `capture_dir`.
    ap.add_argument('--out', default=None)
    ap.add_argument('--grid', default='8x6',
                    help='INNER corners; must match the board AND fov_solve')
    ap.add_argument('--square', type=float, default=0.025)
    ap.add_argument('--width', type=int, default=1280)
    ap.add_argument('--height', type=int, default=720)
    ap.add_argument('--port', type=int, default=8099)
    ap.add_argument('--detect-width', type=int, default=480)
    ap.add_argument('--stream-width', type=int, default=800)
    ap.add_argument('--exposure', type=int, default=50,
                    help='manual shutter in units of 0.1 ms. 0 leaves the '
                         'camera on auto, which measured a 200 ms shutter '
                         'here and smeared every hand movement.')
    ap.add_argument('--brightness', type=int, default=150)
    ap.add_argument('--applies-to', default='pi_forward')
    ap.add_argument('--medium', choices=('air', 'water'), default='air',
                    help='the medium the board is being viewed THROUGH. '
                         'Air and water differ by ~1.44x on this hull; the '
                         'medium is recorded in the file, shown on the page '
                         'and suffixed onto the filename so an in-water '
                         'calibration can never be mistaken for an in-air '
                         'one.')
    ap.add_argument('--install', default=None)
    ap.add_argument('--no-reap', action='store_true',
                    help='do NOT stop an earlier instance first. Only for '
                         'deliberately running two at once; the default is '
                         'to take over, because hunting a PID beside a pool '
                         'is the wrong job at the wrong moment.')
    a = ap.parse_args()

    if not a.no_reap:
        killed = reap_previous(a.port)
        if killed:
            print(f'stopped {len(killed)} earlier instance(s): '
                  f'{", ".join(str(k) for k in killed)}')

    cols, rows = (int(x) for x in a.grid.lower().split('x'))
    if a.out is None:
        a.out = capture_dir(a.applies_to, a.medium)
    os.makedirs(a.out, exist_ok=True)
    st = State(cols, rows, a.out, build_steps())
    st.square = a.square
    resume(st)

    here = os.path.dirname(os.path.abspath(__file__))
    install = os.path.abspath(a.install or default_calibration_dir())
    os.makedirs(install, exist_ok=True)
    print(f'calibrations directory: {install}')
    st.medium = a.medium
    # Same rule the solver installs by -- imported, not restated, so the
    # Download button can never serve a different file than Solve wrote.
    dest = os.path.join(install, _solver().calibration_filename(
        a.applies_to, a.width, a.height, a.medium))
    # ⛔ `solver.py`, NOT `fov_solve.py`. The name changed when calibration
    # moved into the package and this path did not, so the Solve button would
    # have launched a file that does not exist -- a stale string surviving a
    # rename, which is the same defect class as the calibration filename that
    # outlived the camera it was named for.
    argv = [sys.executable, os.path.join(here, 'solver.py'), a.out,
            '--grid', a.grid, '--square', str(a.square),
            '--applies-to', a.applies_to, '--install', install,
            '--medium', a.medium,
            '--identity', json.dumps(camera_identity(f'/dev/video{a.device}'))]

    threading.Thread(target=capture_loop,
                     args=(st, a.device, a.width, a.height, a.stream_width,
                           a.exposure, a.brightness),
                     daemon=True).start()
    threading.Thread(target=detect_loop, args=(st, a.detect_width),
                     daemon=True).start()
    threading.Thread(target=assess,
                     args=(st, cols, rows, a.square, (a.width, a.height)),
                     daemon=True).start()

    st.device = a.device

    def restart(device=None, applies_to=None, grid=None, square=None,
                medium=None):
        """Re-exec on the chosen camera/medium. One startup path, not two."""
        new = restart_argv(sys.argv, a.applies_to, a.medium, device,
                           applies_to, grid, square, medium)
        time.sleep(0.4)
        # `sys.argv[0]` is a Python file either way -- a .py when run
        # directly, and the console-script wrapper under `ros2 run` -- so one
        # form covers both. The earlier version branched on the name and was
        # wrong for the wrapper, which is the case that actually ships.
        os.execv(sys.executable, [sys.executable] + new)

    srv = ThreadingHTTPServer(('0.0.0.0', a.port),
                              make_handler(st, argv, dest, install, restart))
    print(f'{len(st.steps)} guided poses -> {a.out}  (resuming at step '
          f'{st.i + 1})')
    # Never a hardcoded IP: the vehicle's address changes with the link,
    # and a printed URL that quietly stopped being true reads to an operator
    # as "the server did not start". The mDNS name is the stable one.
    hosts = [socket.gethostname().split('.')[0] + '.local']
    try:
        hosts += [ip for ip in subprocess.run(
            ['hostname', '-I'], capture_output=True, text=True,
            timeout=2).stdout.split()
            if ':' not in ip and not ip.startswith(('127.', '169.254.'))
            and not (ip.split('.')[0] == '172'
                     and 16 <= int(ip.split('.')[1]) <= 31)]
    except Exception:
        pass
    print('open  ' + '   or   '.join(f'http://{h}:{a.port}/' for h in hosts))
    print(f'solve installs -> {dest}')
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
