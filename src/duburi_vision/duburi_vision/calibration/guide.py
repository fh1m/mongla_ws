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
    from . import solver
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
        vis = cv2.resize(f, (jpeg_w, int(jpeg_w * h / w)))
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
            cands = [((y, x), t) for y in range(GRID) for x in range(GRID)
                     for t in (10, 25, 40)]
            best, pred, _ = fs.suggest_next_pose(objp, ips, sz, cands)
            with st.lock:
                st.ere, st.ere_pred = ere, pred
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
  <h2>Certainty <span class=sub>(AprilCal Max ERE)</span></h2>
  <div class=bar><i id=eb></i></div>
  <div class=sub id=et></div>
  <div class=sub id=sg style="margin-top:6px"></div>
  <h2>Progress</h2>
  <div class=bar><i id=pb></i></div>
  <div class=sub id=pt></div>
  <button class=ghost id=skip>Skip this pose</button>
  <h2>Finish</h2>
  <button id=go>Run calibration</button>
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
 }catch(e){}
}
$('go').onclick=async()=>{await fetch('/solve',{method:'POST'});tick();};
$('skip').onclick=async()=>{await fetch('/skip',{method:'POST'});tick();};
setInterval(tick,300);tick();
</script>
"""


def make_handler(st, solve_argv, solve_dest):
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
                    'ere': st.ere, 'ere_pred': st.ere_pred,
                    'ere_bar': ERE_BAR_PX, 'n_boards': st.n_boards,
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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--device', type=int, default=3)
    ap.add_argument('--out', default=os.path.expanduser('~/fantech_cal'))
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
    ap.add_argument('--install', default=None)
    a = ap.parse_args()

    cols, rows = (int(x) for x in a.grid.lower().split('x'))
    os.makedirs(a.out, exist_ok=True)
    st = State(cols, rows, a.out, build_steps())
    resume(st)

    here = os.path.dirname(os.path.abspath(__file__))
    install = os.path.abspath(a.install or os.path.join(
        here, '..', 'src', 'duburi_vision', 'config', 'calibration'))
    os.makedirs(install, exist_ok=True)
    dest = os.path.join(install, f'{a.applies_to}_{a.width}x{a.height}.json')
    argv = [sys.executable, os.path.join(here, 'fov_solve.py'), a.out,
            '--grid', a.grid, '--square', str(a.square),
            '--applies-to', a.applies_to, '--install', install]

    threading.Thread(target=capture_loop,
                     args=(st, a.device, a.width, a.height, a.stream_width,
                           a.exposure, a.brightness),
                     daemon=True).start()
    threading.Thread(target=detect_loop, args=(st, a.detect_width),
                     daemon=True).start()
    threading.Thread(target=assess,
                     args=(st, cols, rows, a.square, (a.width, a.height)),
                     daemon=True).start()

    srv = ThreadingHTTPServer(('0.0.0.0', a.port),
                              make_handler(st, argv, dest))
    print(f'{len(st.steps)} guided poses -> {a.out}  (resuming at step '
          f'{st.i + 1})')
    print(f'open http://<this-host>:{a.port}/')
    print(f'solve installs -> {dest}')
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
