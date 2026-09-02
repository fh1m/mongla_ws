#!/usr/bin/env python3
"""Guided camera calibration with live video in a browser.

The Pi is headless, so cv2.imshow is not available -- the annotated feed is
served as MJPEG over HTTP and watched from the laptop.

Why guided rather than "wave the board around": the first attempt produced
RMS 1.57 px with tilt spanning only 9-32 deg and cx 98 px off centre. Without
strong, VARIED tilts the solve is ill-conditioned -- focal length and distance
stay partly confounded and you get a confident, wrong answer. This tool refuses
to accept a pose it already has, so the 25 views are 25 DIFFERENT views.

  cal_guide.py <device> <outdir> --grid WxH [--port 8088]
  then open  http://<pi-ip>:8088  in a browser
"""
import os, sys, threading, time
from http.server import BaseHTTPRequestHandler, HTTPServer
import numpy as np, cv2
cv2.setNumThreads(2)

DEV     = int(sys.argv[1])
OUTDIR  = sys.argv[2]
GRID    = (8, 6)
PORT    = 8088
for i, a in enumerate(sys.argv):
    if a == '--grid': GRID = tuple(int(x) for x in sys.argv[i+1].lower().split('x'))
    if a == '--port': PORT = int(sys.argv[i+1])
COLS, ROWS = GRID
IDEAL = np.mgrid[0:COLS, 0:ROWS].T.reshape(-1, 2).astype(np.float32)
DW, DH = 480, 270      # detection size -- small enough to run in real time
VW, VH = 960, 540      # streamed size -- plenty to aim by, a quarter the JPEG
CRIT = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# Per-capture quality gate. The board was screened at 0.265 px median
# homography residual, so anything materially above that in a single frame is
# the FRAME going wrong (motion blur, partial occlusion, soft focus), not the
# target. Checking at capture time means a bad view is retaken while the
# operator is still standing there, instead of surfacing as a poor RMS after
# all 25 are done -- which is what happened on the first two attempts.
QUALITY_GATE = 0.45   # px at zero tilt, full resolution
# ...but the gate must SCALE WITH TILT. Two effects make an oblique view
# legitimately noisier without anything being wrong with the frame:
#   - depth of field: at 40 deg the far half of the board is at a different
#     distance from the near half, so part of it is genuinely softer;
#   - foreshortening: the compressed side has fewer pixels per square, and
#     cornerSubPix error in pixels is roughly inversely proportional to that.
# A flat threshold therefore rejects hard tilts for being hard tilts -- which
# is exactly the obliquity the calibration needs most. Allowance is linear in
# the measured tilt strength; every attempt is logged so the slope can be
# checked against data instead of trusted.
GATE_TILT_K  = 1.6
ATTEMPT_LOG  = None   # set from OUTDIR below


def gate_for(strength):
    return QUALITY_GATE + GATE_TILT_K * float(strength)
os.makedirs(OUTDIR, exist_ok=True)
ATTEMPT_LOG = os.path.join(OUTDIR, 'attempts.csv')
REJDIR = os.path.join(OUTDIR, 'rejects'); os.makedirs(REJDIR, exist_ok=True)
# A gate must never be able to stall the run forever. After this many
# consecutive rejections on one pose, take the BEST frame seen for that pose
# and move on, flagged in the log so the solve can weight or drop it. A capture
# session that cannot terminate is worse than one view of known lower quality.
MAX_REJECTS = 12

# The pose plan. Each entry is a REQUIRED, distinct view -- the calibration is
# only as good as the variety here.
PLAN = [
    ('FRONTAL   near   -- fill the frame, square to the lens', 'front', 'near',  None),
    ('FRONTAL   mid',                                          'front', 'mid',   None),
    ('FRONTAL   far    -- board small but whole',              'front', 'far',   None),
    ('TILT LEFT  edge toward camera, ~40 deg',                 'left',  'any',   None),
    ('TILT LEFT  again, different distance',                   'left',  'any',   None),
    ('TILT RIGHT edge toward camera, ~40 deg',                 'right', 'any',   None),
    ('TILT RIGHT again, different distance',                   'right', 'any',   None),
    ('TILT TOP   edge toward camera, ~40 deg',                 'up',    'any',   None),
    ('TILT TOP   again, different distance',                   'up',    'any',   None),
    ('TILT BOTTOM edge toward camera, ~40 deg',                'down',  'any',   None),
    ('TILT BOTTOM again, different distance',                  'down',  'any',   None),
    ('DIAGONAL tilt -- top-left corner toward camera',         'any',   'any',   'diag'),
    ('DIAGONAL tilt -- the other diagonal',                    'any',   'any',   'diag'),
    ('CORNER  top-left of frame',                              'any',   'any',   'tl'),
    ('CORNER  top-right of frame',                             'any',   'any',   'tr'),
    ('CORNER  bottom-left of frame',                           'any',   'any',   'bl'),
    ('CORNER  bottom-right of frame',                          'any',   'any',   'br'),
    ('EDGE    hard against the LEFT edge',                     'any',   'any',   'l'),
    ('EDGE    hard against the RIGHT edge',                    'any',   'any',   'r'),
    ('EDGE    hard against the TOP edge',                      'any',   'any',   't'),
    ('EDGE    hard against the BOTTOM edge',                   'any',   'any',   'b'),
    ('Any strong tilt, centre of frame',                       'any',   'any',   'tilt'),
    ('Any strong tilt, different from the last',               'any',   'any',   'tilt'),
    ('FRONTAL close -- fill as much of the frame as you can',  'front', 'near',  None),
    ('Free pose -- anywhere you have not been',                'any',   'any',   None),
]

# Resume: a restart must not throw away poses already captured. The pose plan
# is positional, so the count of saved frames IS the index to resume at.
_done = len([f for f in os.listdir(OUTDIR) if f.endswith('.png')])
if _done:
    print(f"  resuming at pose {_done+1} -- {_done} already captured", flush=True)

state = {'frame': None, 'idx': _done, 'msg': 'starting', 'shots': _done,
         'rms': None, 'lock': threading.Lock(), 'run': True, 'reject': 0, 'note': '', 'qsum': []}


def classify(corners, W, H):
    """Rough pose from the corner quad -- no calibration needed."""
    p = corners.reshape(ROWS, COLS, 2)
    tl, tr, bl, br = p[0, 0], p[0, -1], p[-1, 0], p[-1, -1]
    # ratio of opposite edge lengths reveals which side is nearer
    lft = np.linalg.norm(tl - bl); rgt = np.linalg.norm(tr - br)
    top = np.linalg.norm(tl - tr); bot = np.linalg.norm(bl - br)
    hor = (lft - rgt) / max(lft, rgt)     # + => left edge bigger => left nearer
    ver = (top - bot) / max(top, bot)
    tilt = 'front'
    if abs(hor) > 0.12 or abs(ver) > 0.12:
        tilt = ('left' if hor > 0 else 'right') if abs(hor) >= abs(ver) else \
               ('up' if ver > 0 else 'down')
    strength = max(abs(hor), abs(ver))
    xs, ys = corners.reshape(-1, 2).T
    area = (xs.max()-xs.min()) * (ys.max()-ys.min()) / (W*H)
    scale = 'near' if area > 0.20 else 'mid' if area > 0.07 else 'far'
    cx, cy = xs.mean()/W, ys.mean()/H
    zone = ('t' if cy < 0.35 else 'b' if cy > 0.65 else '') + \
           ('l' if cx < 0.35 else 'r' if cx > 0.65 else '')
    return tilt, scale, strength, area, zone, (cx, cy)


def wants(req_tilt, req_scale, req_zone, tilt, scale, strength, zone):
    if req_tilt != 'any' and tilt != req_tilt: return False
    if req_tilt != 'any' and req_tilt != 'front' and strength < 0.20: return False
    if req_scale != 'any' and scale != req_scale: return False
    if req_zone in ('tl','tr','bl','br'):
        return zone == req_zone
    if req_zone in ('l','r','t','b'):
        return req_zone in zone
    if req_zone == 'diag':
        return strength > 0.15
    if req_zone == 'tilt':
        return strength > 0.22
    return True


def frame_quality(frame):
    """Full-res residual of this one frame, or None if the board is not found.

    A planar target maps to the ideal grid by a homography exactly, so the
    leftover is corner-localisation error for THIS frame. Blur and occlusion
    inflate it; a clean frame sits at the board's screened noise floor.
    """
    g = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ok, c = cv2.findChessboardCorners(
        g, GRID, cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not ok:
        return None
    c = cv2.cornerSubPix(g, c, (11, 11), (-1, -1), CRIT).reshape(-1, 2)
    H, _ = cv2.findHomography(IDEAL, c, 0)
    if H is None:
        return None
    p = cv2.perspectiveTransform(IDEAL.reshape(-1, 1, 2), H).reshape(-1, 2)
    return float(np.linalg.norm(c - p, axis=1).mean())


def worker():
    cap = cv2.VideoCapture(DEV, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_FPS, 30)
    W = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)); H = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK
    prev = None; steady = 0; last_save = 0.0
    runrej = 0; best = (None, None, None)
    heat = np.zeros((3, 3), int)
    fps_t, fps_n, fps = time.time(), 0, 0.0
    while state['run']:
        ok, f = cap.read()
        if not ok: continue
        # DETECT SMALL, SAVE BIG. findChessboardCorners at 1280x720 is so slow
        # on this Pi that it made the guide unusable -- and the guidance only
        # needs the corner QUAD to classify a pose, not sub-pixel accuracy.
        # The saved frame is still full resolution, and `solve` re-detects it
        # properly there, so nothing is lost from the calibration itself.
        small = cv2.resize(f, (DW, DH), interpolation=cv2.INTER_AREA)
        g = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
        vis = cv2.resize(f, (VW, VH), interpolation=cv2.INTER_AREA)
        i = state['idx']
        done = i >= len(PLAN)
        found, c = (False, None) if done else cv2.findChessboardCorners(g, GRID, flags)
        msg, colour = 'show the board', (0, 200, 255)

        if found:
            # no cornerSubPix here: it costs time and the pose classifier only
            # uses edge-length ratios, which sub-pixel refinement does not change
            cv2.drawChessboardCorners(vis, GRID, c * (VW / DW), True)
            tilt, scale, strength, area, zone, (ncx, ncy) = classify(c, DW, DH)
            want_t, want_s, want_z = PLAN[i][1], PLAN[i][2], PLAN[i][3]
            hit = wants(want_t, want_s, want_z, tilt, scale, strength, zone)
            # stillness: corners must stop moving, or the frame is blurred
            if prev is not None and prev.shape == c.shape:
                motion = float(np.abs(c - prev).mean())
            else:
                motion = 99.0
            prev = c.copy()
            still = motion < 0.8      # detect-space px
            steady = steady + 1 if (hit and still) else 0
            msg = (f'{tilt.upper():5s} {scale:4s} tilt{strength*100:3.0f}% '
                   f'fill{area*100:4.1f}%  {"STEADY" if still else "hold still"}')
            colour = (0, 255, 0) if hit else (0, 165, 255)
            if steady >= 4 and time.time() - last_save > 0.8:
                q = frame_quality(f)
                gate = gate_for(strength)
                last_save = time.time(); steady = 0
                if q is not None and (best[0] is None or q < best[0]):
                    best = (q, f.copy(), gate)
                with open(ATTEMPT_LOG, 'a') as _lg:
                    _lg.write(f'{i},{tilt},{strength:.4f},{area:.4f},'
                              f'{"" if q is None else f"{q:.4f}"},{gate:.4f},'
                              f'{0 if (q is None or q > gate) else 1}\n')
                if q is None or q > gate:
                    state['reject'] += 1; runrej += 1
                    if q is not None:
                        cv2.imwrite(f'{REJDIR}/p{i:02d}_{runrej:02d}_{q:.3f}.png', f)
                    if runrej >= MAX_REJECTS and best[0] is not None:
                        cv2.imwrite(f'{OUTDIR}/cal_{state["shots"]:03d}.png', best[1])
                        with open(ATTEMPT_LOG, 'a') as _lg:
                            _lg.write(f'{i},BESTOF,{strength:.4f},{area:.4f},'
                                      f'{best[0]:.4f},{best[2]:.4f},2\n')
                        state['qsum'].append(best[0])
                        state['note'] = f'accepted BEST-OF {best[0]:.3f} px -- moving on'
                        state['shots'] += 1; state['idx'] += 1
                        runrej = 0; best = (None, None, None)
                        heat[min(2,int(ncy*3)), min(2,int(ncx*3))] += 1
                    else:
                        state['note'] = ('REJECTED: board lost at full res'
                                         if q is None else
                                         f'REJECTED {q:.3f} > {gate:.3f} px '
                                         f'({runrej}/{MAX_REJECTS})')
                else:
                    cv2.imwrite(f'{OUTDIR}/cal_{state["shots"]:03d}.png', f)
                    state['qsum'].append(q)
                    state['note'] = f'saved  {q:.3f} / {gate:.3f} px'
                    state['shots'] += 1; state['idx'] += 1
                    runrej = 0; best = (None, None, None)
                    heat[min(2,int(ncy*3)), min(2,int(ncx*3))] += 1
        else:
            prev = None; steady = 0

        # ---- overlay -------------------------------------------------------
        bar_h = 118
        W = VW
        pad = np.zeros((bar_h, W, 3), np.uint8)
        vis = np.vstack([pad, vis])
        if done:
            cv2.putText(vis, 'ALL CAPTURED -- tell Claude', (12, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)
        else:
            cv2.putText(vis, f'[{i+1}/{len(PLAN)}]  {PLAN[i][0]}', (12, 34),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255,255,255), 2)
            cv2.putText(vis, msg, (12, 66), cv2.FONT_HERSHEY_SIMPLEX, 0.58, colour, 2)
            for s_ in range(steady):
                cv2.circle(vis, (24 + s_*22, 94), 8, (0,255,0), -1)
            if state['note']:
                nc = (0,255,0) if state['note'].startswith('saved') else (60,60,255)
                cv2.putText(vis, state['note'], (300, 100),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, nc, 1)
            if state['qsum']:
                cv2.putText(vis, f"quality med {float(np.median(state['qsum'])):.3f} px"
                                 f"  rejected {state['reject']}", (12, 112),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (170,170,170), 1)
            if steady: cv2.putText(vis, 'capturing', (128, 100),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)
        # progress
        cv2.rectangle(vis, (W-250, 18), (W-16, 36), (70,70,70), -1)
        w = int(234 * state['shots'] / len(PLAN))
        cv2.rectangle(vis, (W-250, 18), (W-250+w, 36), (0,220,0), -1)
        cv2.putText(vis, f"{state['shots']}/{len(PLAN)}", (W-250, 58),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255,255,255), 2)
        # coverage 3x3
        for r in range(3):
            for cc in range(3):
                x, y = W-110+cc*30, 68+r*13
                on = heat[r, cc] > 0
                cv2.rectangle(vis, (x, y), (x+26, y+10),
                              (0,200,0) if on else (60,60,60), -1)
        cv2.putText(vis, 'coverage', (W-110, 64),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (180,180,180), 1)
        fps_n += 1
        if time.time() - fps_t > 1.0:
            fps = fps_n / (time.time() - fps_t); fps_t, fps_n = time.time(), 0
        cv2.putText(vis, f'{fps:.0f} fps', (W-70, VH+bar_h-10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (120,120,120), 1)
        with state['lock']:
            state['frame'] = vis
        if done:
            state['run'] = False
    cap.release()


class H(BaseHTTPRequestHandler):
    def log_message(self, *a): pass
    def do_GET(self):
        if self.path != '/stream':
            self.send_response(200); self.send_header('Content-Type','text/html'); self.end_headers()
            self.wfile.write(b"<html><body style='margin:0;background:#111;text-align:center'>"
                             b"<img src='/stream' style='width:100%;max-width:1280px'></body></html>")
            return
        self.send_response(200)
        self.send_header('Content-Type','multipart/x-mixed-replace; boundary=f')
        self.end_headers()
        try:
            while state['run'] or state['frame'] is not None:
                with state['lock']:
                    f = state['frame']
                if f is None: time.sleep(0.03); continue
                ok, j = cv2.imencode('.jpg', f, [cv2.IMWRITE_JPEG_QUALITY, 55])
                if not ok: continue
                self.wfile.write(b'--f\r\nContent-Type: image/jpeg\r\n\r\n' + j.tobytes() + b'\r\n')
                time.sleep(0.01)
        except Exception:
            pass


t = threading.Thread(target=worker, daemon=True); t.start()
srv = HTTPServer(('0.0.0.0', PORT), H)
threading.Thread(target=srv.serve_forever, daemon=True).start()
print(f"  open  http://10.42.0.28:{PORT}  in a browser on the laptop")
print(f"  capturing to {OUTDIR}, {len(PLAN)} guided poses\n")
try:
    while state['run']:
        time.sleep(0.5)
except KeyboardInterrupt:
    state['run'] = False
time.sleep(0.5)
print(f"  {state['shots']} images captured -> {OUTDIR}")
