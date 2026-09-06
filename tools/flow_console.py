#!/usr/bin/env python3
"""flow_console -- watch the bottom camera work as a DVL, live, in a browser.

The same algorithm `flow_node` ships -- adaptive keyframe baseline, forward-
backward rejection, planar rigid fit, gyro de-rotation -- with the frame, the
surviving correspondences and the accumulating distance all visible at once.

⛔ WHY A CONSOLE AND NOT A COUNTDOWN. A blind run over ssh asks the operator to
slide a rig against a stopwatch they cannot see, and then reports one number
they have to trust. The first live attempt caught 0.57 cm of a 30 cm slide and
NOTHING in the output said whether the rig moved late, moved outside the
window, or the sensor missed it. Seeing the points move settles that in one
glance.

    python3 tools/flow_console.py --height 0.72
    then open http://<pi>:8092/

⚠ ThreadingHTTPServer, never HTTPServer: an MJPEG handler never returns, and a
single-threaded server therefore serves the video and NOTHING else -- every
other endpoint hangs with no error anywhere. That has already cost this project
an evening.
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import math
import os
import sys
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)


def _load(name, rel):
    spec = importlib.util.spec_from_file_location(name, os.path.join(_ROOT, rel))
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


_fm = _load('_fc_flow_math',
            'src/duburi_vision/duburi_vision/distance/flow_math.py')
_fv = _load('_fc_flow_velocity',
            'src/duburi_vision/duburi_vision/distance/flow_velocity.py')
_ne = _load('_fc_nav', 'src/duburi_manager/duburi_manager/estimator/nav_estimator.py')

_FEATURE_PARAMS = dict(maxCorners=160, qualityLevel=0.01, minDistance=8,
                       blockSize=7)
_LK_PARAMS = dict(winSize=(31, 31), maxLevel=3,
                  criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,
                            30, 0.01))
_MIN_TRACKS = 6
F_AIR, F_WATER = 513.94, 741.0

STATE = {
    'x': 0.0, 'y': 0.0, 'vx': 0.0, 'vy': 0.0, 'q': 0, 'pts': 0,
    'px': 0.0, 'used': 0, 'refused': 0, 'fallback': 0, 'reason': 'starting',
    'yaw_img': 0.0, 'yaw_gyro': 0.0, 'gyro_rms': 0.0, 'hz': 0.0,
    'peak': 0.0, 'path': 0.0, 'inliers': 0, 'resid': 0.0, 'h': 0.72,
    'moving': False, 'span': 0.0,
}
LOCK = threading.Lock()
JPEG = {'buf': None}
RESET = threading.Event()
SIM_RESET = threading.Event()

# CAPTURE. The operator should not have to read a number off a screen at the
# exact moment they stop sliding, nor should anyone have to relay it by hand:
# both are places for a transcription error to enter a measurement. Arming a
# phase makes the console watch for the move, score it against the tape by
# itself, and append the result to a file.
RUNS_PATH = '/tmp/flow_runs.json'
ARM = {'phase': None, 'truth': 0.30, 'state': 'idle'}
SCORE_NOW = threading.Event()
RUNS = []


def _load_runs():
    global RUNS
    try:
        with open(RUNS_PATH) as fh:
            RUNS = json.load(fh)
    except Exception:
        RUNS = []


def _save_runs():
    try:
        with open(RUNS_PATH, 'w') as fh:
            json.dump(RUNS, fh, indent=1)
    except Exception:
        pass


class Gyro:
    def __init__(self, port):
        from pymavlink import mavutil
        self._m = mavutil.mavlink_connection(port, baud=115200)
        self.buf = deque(maxlen=4096)
        self.yaw_buf = deque(maxlen=4096)
        self.yaw_deg = 0.0
        self.n = 0
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while True:
            try:
                m = self._m.recv_match(blocking=True, timeout=0.5)
            except Exception:
                continue
            if m is None or m.get_type() != 'ATTITUDE':
                continue
            t = time.monotonic()
            self.buf.append((t, float(m.pitchspeed), float(m.rollspeed)))
            self.yaw_buf.append((t, float(m.yawspeed)))
            self.yaw_deg = math.degrees(float(m.yaw))
            self.n += 1

    def rms(self, since=1.0):
        t0 = time.monotonic() - since
        s = [(p, r) for (t, p, r) in self.buf if t >= t0]
        return float(np.sqrt(np.mean([p * p + r * r for p, r in s]))) if s else 0.0


def worker(args):
    gyro = None
    if not args.no_gyro:
        try:
            gyro = Gyro(args.port)
            time.sleep(0.8)
        except Exception as exc:
            print(f'gyro unavailable: {exc}')

    f_px = F_WATER if args.medium == 'water' else F_AIR
    h = args.height
    cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height_px)
    cap.set(cv2.CAP_PROP_FPS, args.fps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not cap.isOpened():
        raise SystemExit(f'cannot open {args.device}')

    # SELF-TEST SOURCE. Not a mock of the pipeline -- the pipeline is
    # untouched; only the frames are synthesised, by warping ONE REAL frame
    # from this camera by a known amount. That exercises capture, scoring,
    # logging and the HTTP path end to end against EXACT truth, which a still
    # rig cannot do (every interval is correctly refused, so the capture never
    # triggers) and a hand slide cannot do either (its truth is a tape).
    sim = None
    if args.sim_slide > 0.0:
        for _ in range(6):
            ok, fr = cap.read()
        sim = dict(base=cv2.cvtColor(fr, cv2.COLOR_BGR2GRAY),
                   travelled=0.0, per_frame=None, hold_until=0.0)
        print(f'SELF-TEST: synthesising a {args.sim_slide * 100:.0f} cm slide '
              f'at {args.sim_speed} m/s through the real pipeline')

    anchor = anchor_pts = None
    anchor_t = None
    cap_t0 = cap_last = None
    pos_hist = deque()
    cap_x0 = cap_y0 = 0.0
    cap_pts = []
    cap_resid = []
    x = y = path = 0.0
    used = refused = fallback = 0
    peak = 0.0
    reason = 'waiting'
    last_v = (0.0, 0.0)
    move_t = 0.0
    t_hz = time.monotonic()
    n_hz = 0
    hz = 0.0
    draw = None

    while True:
        ok, frame = cap.read()
        if not ok:
            continue
        t = time.monotonic()
        if sim is not None:
            # The synthetic slide WAITS FOR THE ARM, exactly as an operator
            # does. Advancing from startup ran the whole 30 cm out in 0.6 s,
            # before anything was armed, and then presented a static scene --
            # which the pipeline correctly reported as no motion, and which
            # looked like a capture failure rather than a timing one.
            if (ARM['state'] == 'armed'
                    and t >= sim.get('hold_until', 0.0)
                    and sim['travelled'] < args.sim_slide):
                sim['travelled'] = min(
                    args.sim_slide,
                    sim['travelled'] + args.sim_speed / max(hz, 30.0))
            shift = f_px * sim['travelled'] / h
            # The synthetic motion follows the ARMED AXIS, so back really is
            # the opposite of forward and lateral really is perpendicular --
            # otherwise the self-test would score three runs of the same move
            # and prove nothing about the axis mapping.
            ph = ARM.get('phase') or 'fwd'
            if ph == 'back':
                M = np.float32([[1, 0, 0], [0, 1, -shift]])
            elif ph == 'lat':
                M = np.float32([[1, 0, shift], [0, 1, 0]])
            else:
                M = np.float32([[1, 0, 0], [0, 1, shift]])
            g = cv2.warpAffine(sim['base'], M,
                               (sim['base'].shape[1], sim['base'].shape[0]),
                               flags=cv2.INTER_LINEAR,
                               borderMode=cv2.BORDER_REFLECT)
            frame = cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)
        n_hz += 1
        if t - t_hz >= 1.0:
            hz, n_hz, t_hz = n_hz / (t - t_hz), 0, t

        if SIM_RESET.is_set():
            SIM_RESET.clear()
            if sim is not None:
                # HOLD BEFORE MOVING. Re-arming rewinds the synthetic travel,
                # which teleports the scene by the whole previous slide (214 px
                # for 30 cm) in one frame. That is an artifact of the test
                # source -- a real operator cannot teleport a floor -- but it
                # lands inside the capture window and scored a 30 cm lateral
                # slide as 0.01 cm. The hold lets the anchor re-establish on
                # the rewound scene before any motion is synthesised.
                sim['travelled'] = 0.0
                sim['hold_until'] = time.monotonic() + 0.8
        if RESET.is_set():
            RESET.clear()
            x = y = path = peak = 0.0
            used = refused = fallback = 0
            anchor = None
            move_t = 0.0
            cap_t0 = cap_last = None

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        H, W = gray.shape[:2]
        vis = frame.copy()

        if anchor is None or anchor_pts is None or len(anchor_pts) < _MIN_TRACKS:
            anchor, anchor_t = gray, t
            anchor_pts = _fm.detect_corners(gray, want=args.want_points)
            _publish(vis, anchor_pts, None, STATE)
            continue

        nxt, status, _ = cv2.calcOpticalFlowPyrLK(anchor, gray, anchor_pts,
                                                  None, **_LK_PARAMS)
        flow = _fm.robust_flow(anchor_pts, nxt, status, min_tracks=_MIN_TRACKS)
        draw = (anchor_pts, nxt, status)
        if flow is None:
            anchor, anchor_t = gray, t
            anchor_pts = _fm.detect_corners(gray, want=args.want_points)
            refused += 1
            reason = 'LK lost the anchor'
            _publish(vis, anchor_pts, None, STATE)
            continue

        n_ok = int(np.asarray(status).reshape(-1).astype(bool).sum())
        mag = math.hypot(flow[0], flow[1])
        dt = t - anchor_t
        if mag < args.target_px and n_ok >= _MIN_TRACKS and dt < args.max_baseline:
            _publish(vis, anchor_pts, draw, STATE)
            continue

        # Forward-backward only now: it is a second full LK pass and the
        # ripeness check above does not need it. Cull immediately before the
        # estimate.
        if args.fb_px > 0 and nxt is not None and status is not None:
            fb = _fm.forward_backward_error(anchor, gray, anchor_pts, nxt,
                                            _LK_PARAMS)
            if fb is not None:
                st = np.asarray(status).reshape(-1).astype(bool)
                st &= (fb <= args.fb_px)
                status = st.astype(np.uint8).reshape(-1, 1)
                n_ok = int(st.sum())

        disp = _fm.flow_dispersion(anchor_pts, nxt, status)
        yaw_img = 0.0
        inliers, resid = n_ok, disp or 0.0
        pm = _fm.solve_planar_motion(anchor_pts, nxt, status, dt,
                                     cx=W / 2.0, cy=H / 2.0,
                                     ransac_px=args.ransac_px)
        if pm.ok:
            flow = (pm.dx_px, pm.dy_px)
            n_ok, disp = pm.n_inliers, pm.residual_px
            inliers, resid = pm.n_inliers, pm.residual_px
            yaw_img = -pm.yaw_rate
        else:
            fallback += 1

        pr = rr = 0.0
        if gyro is not None and gyro.buf:
            r = _fm.integrate_rate(list(gyro.buf), t - dt, t)
            if r:
                pr, rr = r
        anchor, anchor_t = gray, t
        anchor_pts = _fm.detect_corners(gray, want=args.want_points)

        v = _fv.flow_velocity(flow[0], flow[1], dt, f_px=f_px, height_m=h,
                              pitch_rate=-pr, roll_rate=-rr,
                              dispersion_px=disp,
                              min_net_flow_px=args.min_flow,
                              rot_fraction_max=args.rot_max,
                              max_dispersion_ratio=args.max_disp)
        if not v.ok:
            refused += 1
            reason = v.reason
        else:
            used += 1
            reason = 'ok'
            x += v.vx * dt
            y += v.vy * dt
            path += math.hypot(v.vx, v.vy) * dt
            peak = max(peak, math.hypot(x, y))
            last_v = (v.vx, v.vy)
            if math.hypot(v.vx, v.vy) > 0.03:
                move_t = t


        # ---- armed capture: find the move, score it, log it ----------------
        #
        # ⛔ STILLNESS IS JUDGED ON POSITION, NOT ON INSTANTANEOUS VELOCITY,
        # and the reason is the absence-is-not-zero rule applied to my own
        # code. The first version read `speed = hypot(vx,vy) if v.ok else 0`,
        # so a REFUSED interval counted as zero speed -- and a burst of
        # refusals in the middle of a slide therefore looked exactly like the
        # rig stopping. Measured live: a capture closed at 12 cm while the
        # console's own trace carried on to 24.9 cm.
        #
        # A refusal means "I could not measure", never "it did not move". The
        # position accumulator already encodes the distinction correctly: a
        # refused interval contributes nothing to it, so a genuinely still rig
        # and a temporarily unmeasurable one both leave the position ALONE.
        # Asking whether the position has changed is therefore the question
        # that survives both cases, and it is also the quantity the operator
        # is watching.
        if ARM['state'] == 'armed':
            pos_hist.append((t, x, y))
            while pos_hist and t - pos_hist[0][0] > args.still_s:
                pos_hist.popleft()
            moved_recently = args.still_s
            if len(pos_hist) >= 2:
                _, x0h, y0h = pos_hist[0]
                moved_recently = math.hypot(x - x0h, y - y0h)
            else:
                moved_recently = float('inf')
            travelled = math.hypot(x - cap_x0, y - cap_y0)

            # THE OPERATOR ENDS THE RUN. No stillness heuristic beats the
            # person who just stopped sliding, and three of them have now been
            # wrong here in a row -- each one closing the capture mid-slide and
            # reporting a correct sensor as short. Auto-stop is kept as a
            # convenience with settings loose enough for a slow hand
            # (2 s below 6 mm), but the button is the ground truth for WHEN.
            forced = SCORE_NOW.is_set()
            if forced:
                SCORE_NOW.clear()
            if travelled >= args.min_capture_m and (
                    forced or moved_recently < args.still_m):
                dx, dy = x - cap_x0, y - cap_y0
                ph = ARM['phase']
                truth = float(ARM['truth'])
                primary = dy if ph == 'lat' else dx
                cross = dx if ph == 'lat' else dy
                mag = math.hypot(dx, dy)
                rec = {
                    'phase': ph, 'truth_m': truth,
                    'measured_m': primary, 'cross_m': cross,
                    'dx_m': dx, 'dy_m': dy, 'abs_m': mag,
                    'axis_deg': math.degrees(math.atan2(dy, dx)),
                    'on_named_axis_pct': (100.0 * abs(primary) / mag
                                          if mag > 1e-6 else 0.0),
                    'error_m': mag - truth,
                    'pct': 100.0 * mag / truth if truth else 0.0,
                    'implied_h_m': (h * truth / mag) if mag > 1e-6 else None,
                    'span_s': t - (cap_t0 or t),
                    'points_med': float(np.median(cap_pts)) if cap_pts else 0,
                    'resid_med': float(np.median(cap_resid)) if cap_resid else 0.0,
                    'used': used, 'refused': refused, 'fallback': fallback,
                    'h_m': h, 'when': time.strftime('%H:%M:%S'),
                }
                RUNS.append(rec)
                _save_runs()
                ARM['state'] = 'done'
                cap_t0 = None
                pos_hist.clear()
            else:
                if cap_t0 is None and travelled > 0.005:
                    cap_t0 = t
                if v.ok:
                    cap_pts.append(n_ok)
                    cap_resid.append(resid)

        gy = 0.0
        if gyro is not None and gyro.yaw_buf:
            g = _fm.interp_rate([(a, b, 0.0) for (a, b) in gyro.yaw_buf],
                                t - dt * 0.5)
            if g:
                gy = g[0]

        with LOCK:
            STATE.update(x=x, y=y, vx=last_v[0], vy=last_v[1],
                         q=int(255 * max(0.0, 1.0 - v.rot_fraction))
                         if v.ok else 0,
                         pts=n_ok, px=mag, used=used, refused=refused,
                         fallback=fallback, reason=reason, yaw_img=yaw_img,
                         yaw_gyro=gy, gyro_rms=gyro.rms() if gyro else 0.0,
                         hz=hz, peak=peak, path=path, inliers=inliers,
                         resid=resid, h=h, moving=(t - move_t) < 0.5,
                         span=math.hypot(x, y),
                         arm_phase=ARM['phase'], arm_state=ARM['state'],
                         n_runs=len(RUNS))
        _publish(vis, anchor_pts, draw, STATE)


def _publish(vis, pts, draw, st):
    """Overlay the correspondences and the numbers, then encode once."""
    if draw is not None:
        p0, p1, status = draw
        if p0 is not None and p1 is not None and status is not None:
            s = np.asarray(status).reshape(-1).astype(bool)
            a = np.asarray(p0).reshape(-1, 2)
            b = np.asarray(p1).reshape(-1, 2)
            for i in range(min(len(a), len(b))):
                if not s[i]:
                    continue
                cv2.line(vis, (int(a[i][0]), int(a[i][1])),
                         (int(b[i][0]), int(b[i][1])), (0, 235, 255), 1,
                         cv2.LINE_AA)
                cv2.circle(vis, (int(b[i][0]), int(b[i][1])), 2,
                           (0, 255, 120), -1, cv2.LINE_AA)
    elif pts is not None:
        for q in np.asarray(pts).reshape(-1, 2):
            cv2.circle(vis, (int(q[0]), int(q[1])), 2, (90, 90, 90), -1)
    h, w = vis.shape[:2]
    cv2.line(vis, (w // 2 - 12, h // 2), (w // 2 + 12, h // 2), (255, 90, 40), 1)
    cv2.line(vis, (w // 2, h // 2 - 12), (w // 2, h // 2 + 12), (255, 90, 40), 1)
    ok, buf = cv2.imencode('.jpg', vis, [cv2.IMWRITE_JPEG_QUALITY, 70])
    if ok:
        JPEG['buf'] = buf.tobytes()


PAGE = r"""<!doctype html><html><head><meta charset=utf-8>
<title>bottom camera :: DVL</title>
<style>
*{box-sizing:border-box}
body{margin:0;background:#0a0c10;color:#d8dee9;
 font:13px/1.45 ui-monospace,SFMono-Regular,Menlo,monospace}
header{display:flex;align-items:baseline;gap:14px;padding:10px 18px;
 border-bottom:1px solid #1b2029;background:#0d1016}
h1{font-size:13px;margin:0;letter-spacing:.14em;text-transform:uppercase;
 color:#7d8799;font-weight:600}
.tag{font-size:11px;color:#4e5666}
main{display:grid;grid-template-columns:minmax(420px,1fr) 400px;gap:0;
 height:calc(100vh - 41px)}
.view{position:relative;background:#05070a;display:flex;align-items:center;
 justify-content:center;border-right:1px solid #1b2029}
.view img{max-width:100%;max-height:100%;display:block}
.side{overflow:auto;padding:14px 16px}
.big{font-size:64px;line-height:1;font-weight:600;letter-spacing:-.03em;
 font-variant-numeric:tabular-nums}
.unit{font-size:18px;color:#5a6474;margin-left:6px;font-weight:400}
.axis{margin-bottom:14px;padding:12px 14px;background:#0e1218;
 border:1px solid #1b2029;border-radius:8px;position:relative;overflow:hidden}
.lab{font-size:10px;letter-spacing:.16em;text-transform:uppercase;color:#5f6a7d;
 margin-bottom:6px}
.fwd .big{color:#5ee2a0}.lat .big{color:#63b3ff}
.bar{height:3px;background:#1b2029;margin-top:10px;border-radius:2px;
 overflow:hidden}
.bar i{display:block;height:100%;background:#5ee2a0;width:0;
 transition:width .12s linear}
.lat .bar i{background:#63b3ff}
table{width:100%;border-collapse:collapse;margin-top:4px}
td{padding:3px 0;font-size:12px}
td:first-child{color:#67718a}
td:last-child{text-align:right;font-variant-numeric:tabular-nums}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-bottom:12px}
.card{background:#0e1218;border:1px solid #1b2029;border-radius:8px;
 padding:10px 12px}
.card .v{font-size:22px;font-weight:600;font-variant-numeric:tabular-nums}
button{width:100%;padding:11px;background:#182030;color:#cbd4e1;
 border:1px solid #263041;border-radius:8px;
 font:600 12px/1 ui-monospace,monospace;letter-spacing:.12em;cursor:pointer;
 text-transform:uppercase;margin-bottom:12px}
button:hover{background:#1e2739;border-color:#37455e}
button:active{transform:translateY(1px)}
.runs{background:#0e1218;border:1px solid #1b2029;border-radius:8px;
 padding:12px 14px;margin-bottom:14px}
.rbtns{display:grid;grid-template-columns:1fr 1fr 1fr;gap:6px}
.rb{margin:0;padding:9px 4px;font-size:11px;letter-spacing:.08em}
.rb.on{background:#33290f;border-color:#8a6a1e;color:#ffc861}
.rb.ok{background:#10331f;border-color:#2c6b45;color:#5ee2a0}
.armed{margin-top:10px;font-size:12px;color:#ffc861;min-height:16px}
.score{margin-top:8px;margin-bottom:0;background:#1d2a1f;border-color:#2c6b45;
 color:#5ee2a0}
.score:hover{background:#24371f;border-color:#3d8a5b}
#rtab{margin-top:8px}
#rtab td{border-top:1px solid #161b23;padding:5px 0;font-size:11px}
.good{color:#5ee2a0}.bad{color:#ff8080}.meh{color:#ffc861}
.rfoot{margin-top:8px;font-size:10px;color:#3f4756}
.rfoot a{color:#5a6474;text-decoration:none}
.rfoot a:hover{color:#8b95a8}
.pill{display:inline-block;padding:2px 8px;border-radius:20px;font-size:10px;
 letter-spacing:.1em;text-transform:uppercase}
.ok{background:#10331f;color:#5ee2a0}
.no{background:#331414;color:#ff8080}
.mv{background:#33290f;color:#ffc861}
.warn{color:#ffc861}
h2{font-size:10px;letter-spacing:.18em;text-transform:uppercase;color:#4e5666;
 margin:18px 0 6px;font-weight:600}
</style></head><body>
<header><h1>Bottom camera &rarr; DVL</h1>
<span class=tag id=hdr></span></header>
<main>
 <div class=view><img id=v src="/stream"></div>
 <div class=side>
  <div class=runs>
   <div class=lab style="margin-bottom:8px">Guided 30 cm verification</div>
   <div class=rbtns>
    <button class=rb onclick="arm('fwd')" id=b_fwd>Forward</button>
    <button class=rb onclick="arm('back')" id=b_back>Back</button>
    <button class=rb onclick="arm('lat')" id=b_lat>Lateral</button>
   </div>
   <button class=score onclick="fetch('/score')" id=bscore>I have stopped &mdash; score it</button>
   <div id=armed class=armed></div>
   <table id=rtab></table>
   <div class=rfoot>
    <a href="#" onclick="fetch('/clearruns');return false">clear</a> &middot;
    <a href="#" onclick="fetch('/reset');return false">zero the counters</a>
   </div>
  </div>
  <div class="axis fwd"><div class=lab>Forward / back</div>
    <div><span class=big id=x>0.0</span><span class=unit>cm</span></div>
    <div class=bar><i id=bx></i></div></div>
  <div class="axis lat"><div class=lab>Lateral</div>
    <div><span class=big id=y>0.0</span><span class=unit>cm</span></div>
    <div class=bar><i id=by></i></div></div>
  <div class=grid>
   <div class=card><div class=lab>Speed</div>
     <div class=v id=spd>0.00</div></div>
   <div class=card><div class=lab>Points</div>
     <div class=v id=pts>0</div></div>
  </div>
  <div id=st></div>
  <h2>Measurement</h2>
  <table>
   <tr><td>intervals used</td><td id=used>0</td></tr>
   <tr><td>refused</td><td id=ref>0</td></tr>
   <tr><td>median fallback</td><td id=fb>0</td></tr>
   <tr><td>inliers / residual</td><td id=inl>-</td></tr>
   <tr><td>px this interval</td><td id=px>0.00</td></tr>
   <tr><td>path length</td><td id=path>0.0 cm</td></tr>
   <tr><td>furthest from zero</td><td id=peak>0.0 cm</td></tr>
  </table>
  <h2>Cross-check</h2>
  <table>
   <tr><td>yaw from image</td><td id=yi>0.000</td></tr>
   <tr><td>yaw from gyro</td><td id=yg>0.000</td></tr>
   <tr><td>gyro rms</td><td id=gr>0.000</td></tr>
  </table>
  <h2>Setup</h2>
  <table>
   <tr><td>lens to floor</td><td id=hh>-</td></tr>
   <tr><td>capture</td><td id=hz>0 Hz</td></tr>
   <tr><td>last reason</td><td id=rsn>-</td></tr>
  </table>
 </div>
</main>
<script>
const $=i=>document.getElementById(i);
const TRUTH=0.30;
let armedPhase=null;
function arm(p){armedPhase=p;fetch('/arm?phase='+p+'&truth='+TRUTH);}
function fmt(r){
 const e=r.error_m*100, pct=r.pct;
 const cls = Math.abs(e)<=1.0?'good':(Math.abs(e)<=2.5?'meh':'bad');
 const dx=(r.dx_m!==undefined?r.dx_m:0)*100, dy=(r.dy_m!==undefined?r.dy_m:0)*100;
 return '<tr><td style="text-transform:uppercase;letter-spacing:.1em">'
  +r.phase+'</td><td>'+(r.abs_m*100).toFixed(1)+' cm</td>'
  +'<td class="'+cls+'">'+(e>=0?'+':'')+e.toFixed(1)+' cm</td>'
  +'<td class="'+cls+'">'+pct.toFixed(1)+'%</td>'
  +'<td style="color:#5a6474">'+dx.toFixed(0)+','+dy.toFixed(0)+'</td>'
  +'<td style="color:#5a6474">h&rarr;'
  +(r.implied_h_m?r.implied_h_m.toFixed(2):'-')+'</td></tr>';
}
async function runs(){
 try{
  const d=await (await fetch('/runs')).json();
  $('rtab').innerHTML = d.runs.length
   ? '<tr style="color:#4e5666"><td>run</td><td>distance</td><td>error</td>'
     +'<td>of truth</td><td>dx,dy cm</td><td>implied</td></tr>'+d.runs.map(fmt).join('')
   : '';
  const st=d.arm.state, ph=d.arm.phase;
  ['fwd','back','lat'].forEach(p=>{
    const b=$('b_'+p); b.className='rb';
    if(st==='armed'&&ph===p) b.className='rb on';
    if(d.runs.some(r=>r.phase===p)) b.className='rb ok';
  });
  $('armed').textContent = st==='armed'
    ? 'ARMED for '+ph.toUpperCase()+' — slide, then press score.'
    : (st==='done' ? 'captured. arm the next axis.' : '');
  $('bscore').style.opacity = st==='armed' ? '1' : '0.35';
 }catch(e){}
 setTimeout(runs,400);
}
runs();
async function tick(){
 try{
  const d=await (await fetch('/data')).json();
  $('x').textContent=(d.x*100).toFixed(1);
  $('y').textContent=(d.y*100).toFixed(1);
  $('bx').style.width=Math.min(100,Math.abs(d.x*100)/30*100)+'%';
  $('by').style.width=Math.min(100,Math.abs(d.y*100)/30*100)+'%';
  $('spd').textContent=Math.hypot(d.vx,d.vy).toFixed(2);
  $('pts').textContent=d.pts;
  $('used').textContent=d.used;$('ref').textContent=d.refused;
  $('fb').textContent=d.fallback;
  $('inl').textContent=d.inliers+' / '+d.resid.toFixed(2)+' px';
  $('px').textContent=d.px.toFixed(2);
  $('path').textContent=(d.path*100).toFixed(1)+' cm';
  $('peak').textContent=(d.peak*100).toFixed(1)+' cm';
  $('yi').textContent=d.yaw_img.toFixed(3);
  $('yg').textContent=d.yaw_gyro.toFixed(3);
  $('gr').textContent=d.gyro_rms.toFixed(3);
  $('hh').textContent=d.h.toFixed(2)+' m';
  $('hz').textContent=d.hz.toFixed(0)+' Hz';
  $('rsn').textContent=d.reason;
  $('st').innerHTML= d.moving
    ? '<span class="pill mv">moving</span>'
    : (d.reason==='ok'?'<span class="pill ok">tracking</span>'
                      :'<span class="pill no">'+d.reason.slice(0,28)+'</span>');
  $('hdr').textContent='slide 30 cm and read the number';
 }catch(e){}
 setTimeout(tick,120);
}
tick();
</script></body></html>"""


class H(BaseHTTPRequestHandler):
    protocol_version = 'HTTP/1.1'

    def log_message(self, *a):
        pass

    def _json(self, obj):
        body = json.dumps(obj).encode()
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path.startswith('/stream'):
            self.send_response(200)
            self.send_header('Content-Type',
                             'multipart/x-mixed-replace; boundary=f')
            self.end_headers()
            try:
                while True:
                    buf = JPEG['buf']
                    if buf:
                        self.wfile.write(b'--f\r\nContent-Type: image/jpeg\r\n'
                                         b'Content-Length: '
                                         + str(len(buf)).encode() + b'\r\n\r\n'
                                         + buf + b'\r\n')
                    time.sleep(0.04)
            except Exception:
                return
        if self.path.startswith('/data'):
            with LOCK:
                body = json.dumps(STATE).encode()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if self.path.startswith('/arm'):
            from urllib.parse import parse_qs, urlparse
            q = parse_qs(urlparse(self.path).query)
            ARM['phase'] = (q.get('phase', ['fwd'])[0])
            ARM['truth'] = float(q.get('truth', ['0.30'])[0])
            ARM['state'] = 'armed'
            SIM_RESET.set()
            RESET.set()
            return self._json({'ok': True, 'phase': ARM['phase']})
        if self.path.startswith('/score'):
            SCORE_NOW.set()
            return self._json({'ok': True})
        if self.path.startswith('/disarm'):
            ARM['state'] = 'idle'
            return self._json({'ok': True})
        if self.path.startswith('/runs'):
            return self._json({'runs': RUNS, 'arm': ARM})
        if self.path.startswith('/clearruns'):
            RUNS.clear()
            _save_runs()
            return self._json({'ok': True})
        if self.path.startswith('/reset'):
            RESET.set()
            body = b'{"ok":true}'
            self.send_response(200)
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        body = PAGE.encode()
        self.send_response(200)
        self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--height', type=float, required=True)
    p.add_argument('--device', default='/dev/duburi_cam_downward')
    p.add_argument('--width', type=int, default=640)
    p.add_argument('--height-px', type=int, default=360)
    p.add_argument('--fps', type=int, default=210)
    p.add_argument('--port-http', type=int, default=8092)
    p.add_argument('--port', default='/dev/ttyUSB0')
    p.add_argument('--medium', choices=('air', 'water'), default='air')
    p.add_argument('--no-gyro', action='store_true')
    p.add_argument('--target-px', type=float, default=8.0)
    p.add_argument('--max-baseline', type=float, default=0.75)
    p.add_argument('--ransac-px', type=float, default=2.0)
    p.add_argument('--fb-px', type=float, default=2.0)
    p.add_argument('--min-flow', type=float, default=0.5)
    p.add_argument('--rot-max', type=float, default=0.80)
    p.add_argument('--max-disp', type=float, default=5.0)
    p.add_argument('--move-thresh', type=float, default=0.03)
    p.add_argument('--still-s', type=float, default=2.0)
    p.add_argument('--still-m', type=float, default=0.006,
                   help='position change over still_s that counts as stopped')
    p.add_argument('--min-capture-m', type=float, default=0.08,
                   help='a move smaller than this is a nudge, not a run')
    p.add_argument('--sim-slide', type=float, default=0.0,
                   help='self-test: synthesise this slide, metres')
    p.add_argument('--sim-speed', type=float, default=0.15)
    p.add_argument('--want-points', type=int, default=80)
    a = p.parse_args()
    _load_runs()
    threading.Thread(target=worker, args=(a,), daemon=True).start()
    srv = ThreadingHTTPServer(('0.0.0.0', a.port_http), H)
    print(f'flow console on http://0.0.0.0:{a.port_http}/  '
          f'(h={a.height} m, {a.medium})')
    srv.serve_forever()


if __name__ == '__main__':
    main()
