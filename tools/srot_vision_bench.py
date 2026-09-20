#!/usr/bin/env python3
"""Live bench: camera -> Hailo -> control law -> srot board -> thrusters.

Open the page, stand in front of the camera, move. The panel shows the whole
chain at once -- the box, the pixel error, the bearing in degrees, the four
MANUAL_CONTROL axes going out, and the board's own ESC RPM coming back. If the
loop is wired correctly, moving right makes the lateral command go positive and
the horizontal thrusters change together.

WHY A PAGE AND NOT A LOG. The claim being tested is "vision drives thrusters",
and that is a claim about a CORRELATION between two things that move. A log of
numbers cannot show you that; two columns updating side by side while you move
can, and a disagreement is obvious rather than something you reconstruct
afterwards from timestamps.

SAFETY
  * Arming is explicit and per-session -- the page opens DISARMED.
  * Disarm is unconditional on this board and is the universal stop. The big
    button, closing the page, and Ctrl-C all disarm.
  * STABILIZE only. This path never enters AUTO, so it never touches the
    depth-loop gate that every SROT_MOVE goes through.
  * `up` is never commanded. The depth axis is not ported.

    srot_vision_bench.py [--class person] [--gain 25] [--port 8091]
                         [--model yolov11n] [--device 0]
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np

WS = os.path.expanduser('~/mongla_ws/src')
for pkg in ('mongla_control', 'mongla_vision', 'mongla_manager'):
    p = os.path.join(WS, pkg)
    if p not in sys.path:
        sys.path.insert(0, p)

import cv2                                                   # noqa: E402
cv2.setNumThreads(0)
from pymavlink import mavutil                                # noqa: E402

from mongla_control.bearing import (                         # noqa: E402
    BearingFilter, bearing_from_pixels)
from mongla_control.fc import srot_protocol as sp            # noqa: E402
from mongla_control.fc.port_guard import PortGuard           # noqa: E402
from mongla_control.fc.srot_fc import SrotFC                 # noqa: E402
from mongla_vision.detection.factory import make_detector    # noqa: E402
import os as _os, sys as _sys
_sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))
from _where import where   # never print a hardcoded IP

ap = argparse.ArgumentParser()
ap.add_argument('--class', dest='klass', default='person')
ap.add_argument('--gain', type=float, default=25.0, help='speed cap, %% thrust')
ap.add_argument('--port', type=int, default=8091)
ap.add_argument('--model', default='yolov11n')
ap.add_argument('--device', type=int, default=0)
ap.add_argument('--dev', default='/dev/ttyUSB0')
ap.add_argument('--conf', type=float, default=0.35)
ap.add_argument('--hz', type=float, default=50.0)
ap.add_argument('--cal', default=os.path.expanduser(
    '~/mongla_ws/src/mongla_vision/config/calibration/pi_downward_1280x720.json'))
A = ap.parse_args()

W, H = 640, 360
DEADBAND_DEG = 2.5        # command zero inside this bearing error
# Kept only for the overlay: the control deadband is angular now, and the two
# would drift apart if the drawing derived from a second constant.
DEADBAND_PX = 25.0

# ---- calibration, rescaled to the streamed resolution ---------------------
K = D = None
try:
    c = json.load(open(A.cal))
    Kc, D = c['camera_matrix'], c['distortion_coefficients']
    sx, sy = W / c['image_width'], H / c['image_height']
    K = [Kc[0][0] * sx, 0.0, Kc[0][2] * sx,
         0.0, Kc[1][1] * sy, Kc[1][2] * sy, 0.0, 0.0, 1.0]
except Exception as exc:                                     # noqa: BLE001
    print(f'no calibration ({exc}); bearings will be unavailable', file=sys.stderr)

# The board's thrust allocation, copied from srot-control-board
# `src/control/mixer.cpp:25-35`. Mirrored here ONLY to visualise what the board
# is about to do with our demand -- the actual allocation happens on the board
# and the panel shows measured ESC RPM beside this prediction, so a divergence
# between the two is visible rather than assumed away.
#            roll pitch  yaw  thr  fwd  lat
MIX = (
    (0.0,  0.0,  1.0,  0.0, -1.0,  1.0),   # 1 FR horiz
    (0.0,  0.0, -1.0,  0.0, -1.0, -1.0),   # 2 FL horiz
    (0.0,  0.0, -1.0,  0.0,  1.0,  1.0),   # 3 RR horiz
    (0.0,  0.0,  1.0,  0.0,  1.0, -1.0),   # 4 RL horiz
    (1.0, -1.0,  0.0, -1.0,  0.0,  0.0),   # 5 FR vert
    (-1.0, -1.0, 0.0, -1.0,  0.0,  0.0),   # 6 FL vert
    (1.0,  1.0,  0.0, -1.0,  0.0,  0.0),   # 7 RR vert
    (-1.0,  1.0, 0.0, -1.0,  0.0,  0.0),   # 8 RL vert
)


def mix(fwd, lat, up, yaw, frame_reverse=False):
    """demand -> per-thruster -1..1, including the board's two saturation groups.

    The horizontal (1-4) and vertical (5-8) groups scale down INDEPENDENTLY --
    the matrix is block-diagonal, so a saturated yaw must not bleed authority
    out of the vertical thrusters. Getting that wrong makes the panel disagree
    with the vehicle only when it matters, i.e. at full deflection.
    """
    if frame_reverse:
        fwd, lat, up, yaw = -fwd, -lat, -up, -yaw
    demand = (0.0, 0.0, yaw, up, fwd, lat)
    out = [sum(M[c] * demand[c] for c in range(6)) for M in MIX]
    for lo, hi in ((0, 4), (4, 8)):
        peak = max(1.0, max(abs(v) for v in out[lo:hi]))
        for i in range(lo, hi):
            out[i] /= peak
    return out


state = {'mix': [0.0] * 8, 'esc_msgs': 0, 'braw': None, 'bfilt_resets': 0, 'frame': None, 'draw': (None, 0.0, 0.0), 'frame_reverse': False, 'hist': [],
    'jpg': None, 'armed': False, 'mode': '?', 'want_arm': None,
    'det': None, 'axes': (0.0, 0.0, 0.0, 0.0), 'rpm': [0] * 8,
    'bearing': None, 'loop_hz': 0.0, 'det_hz': 0.0, 'sent': 0, 'msg': '',
    'depth': float('nan'), 'yaw': float('nan'),
}
_stop = threading.Event()


def worker():
    guard = PortGuard(A.dev)
    guard.acquire()
    m = mavutil.mavlink_connection(A.dev, baud=115200,
                                   source_system=sp.SOURCE_SYSID,
                                   source_component=sp.SOURCE_COMPID)
    m.wait_heartbeat(timeout=10)
    fc = SrotFC(m, log=None)

    # The 200 Hz demuxing reader, exactly as auv_manager_node runs it. Without
    # it every NAMED_VALUE_FLOAT but the last of each 500 ms bundle is lost --
    # pymavlink keeps ONE message per msgid and all 20 names share msgid 251.
    def reader():
        while not _stop.is_set():
            while True:
                msg = m.recv_match(blocking=False)
                if msg is None:
                    break
                t = msg.get_type()
                if t == 'NAMED_VALUE_FLOAT':
                    fc.note_named_value(msg)
                elif t == 'BATTERY_STATUS':
                    fc.note_battery(msg)
                elif t in ('ESC_TELEMETRY_1_TO_4', 'ESC_TELEMETRY_5_TO_8'):
                    state['esc_msgs'] += 1
                    base = 0 if t.endswith('1_TO_4') else 4
                    rpm = list(state['rpm'])
                    for i, v in enumerate(getattr(msg, 'rpm', [])[:4]):
                        rpm[base + i] = int(v)
                    state['rpm'] = rpm
            time.sleep(0.005)
    threading.Thread(target=reader, daemon=True).start()

    det = make_detector(model_path=A.model, conf=A.conf,
                        class_allowlist=[A.klass], device='cuda:0',
                        iou=0.5, imgsz=640, half=True, max_det=20)
    cap = cv2.VideoCapture(A.device, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
    cap.set(cv2.CAP_PROP_FPS, 210)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    for _ in range(15):
        cap.read()

    bfilt = BearingFilter()
    period = 1.0 / max(A.hz, 1.0)
    t_loop = time.perf_counter()
    hz_ema = det_ema = 0.0
    n_det = 0
    while not _stop.is_set():
        t0 = time.perf_counter()

        want = state.pop('want_arm', None)
        if want is True:
            ok, why = fc.set_mode('STABILIZE')
            ok, why = fc.arm(timeout=12.0)
            state['msg'] = f'arm: {"OK" if ok else why}'
        elif want is False:
            fc.disarm(timeout=8.0)
            state['msg'] = 'disarmed'
        state['want_arm'] = None

        ok, frame = cap.read()
        if not ok:
            time.sleep(0.01)
            continue
        dets = det.infer(frame)
        n_det += 1
        target = max(dets, key=lambda d: d.area) if dets else None

        fwd = lat = up = yaw = 0.0
        ex_px = 0.0
        b = b_raw = None
        if target is not None:
            ex_px = target.cx - W / 2.0
            if K:
                b_raw = bearing_from_pixels(target.cx, target.cy,
                                            target.width, target.height,
                                            width=W, height=H, K=K, D=D)
        # Filter even when the target is gone -- passing None is what clears
        # the velocity state, and a filter that keeps extrapolating through a
        # dropout walks the bearing off the frame while looking confident.
        b = bfilt.update(b_raw, time.perf_counter())

        # Steer on the FILTERED BEARING, not the raw pixel error. Two separate
        # reasons, and the second is the one that matters to the vehicle:
        # filtering removes 78 % of the measured jitter at no lag cost, and a
        # bearing has UNITS -- so this gain is thrust-per-radian rather than
        # thrust-per-this-particular-camera.
        #
        # A deadband, because without one the hull hunts forever on residual
        # noise, and on a bench that reads as "the loop is unstable" when it is
        # doing exactly what it was told. 2.5 deg is just under the raw jitter
        # sd (2.96 deg) and comfortably above the filtered one (0.77).
        if b is not None:
            err_rad = b.angle_x
            if abs(err_rad) > math.radians(DEADBAND_DEG):
                lat = max(-1.0, min(1.0, err_rad / math.radians(20.0)))
                lat *= A.gain / 100.0
        # Send EVERY tick, including the neutral one. MANUAL_CONTROL authority
        # ramps to zero between MANUAL_FRESH_MS (1000) and MANUAL_DECAY_MS
        # (1500), so a loop that only sends when it sees something would decay
        # mid-track and read as the vehicle giving up.
        fc.manual(fwd=fwd, lat=lat, up=up, yaw=yaw)
        state['sent'] += 1
        state['axes'] = (fwd, lat, up, yaw)
        state['mix'] = mix(fwd, lat, up, yaw, state['frame_reverse'])
        # Short rolling history so the page can draw pixel-error against
        # lateral-command over time. That correlation IS the claim being
        # tested; a pair of instantaneous numbers cannot show it.
        h = state['hist']
        h.append((round(math.degrees(b_raw.angle_x) if b_raw else 0.0, 2),
                  round(lat, 3), 1 if target is not None else 0,
                  round(math.degrees(b.angle_x) if b else 0.0, 2)))
        del h[:-240]
        state['bearing'] = b
        state['braw'] = b_raw
        state['bfilt_resets'] = bfilt.resets
        state['det'] = target

        t = fc.telemetry()
        if state['sent'] % 200 == 1:
            # FRAME_REVERSE negates all six axis demands on the board, so the
            # panel would draw every thruster backwards without it. This hull
            # ships FR=1 (see the CFG banner), which is exactly the sort of
            # thing that is invisible until you watch a thruster.
            try:
                fr = fc.get_param('FRAME_REVERSE', timeout=0.4)
                if fr is not None:
                    state['frame_reverse'] = bool(fr > 0.5)
            except Exception:                                # noqa: BLE001
                pass
        state['armed'], state['mode'] = bool(t.armed), t.mode or '?'
        state['depth'], state['yaw'] = t.depth_m, t.yaw_deg

        # DRAWING IS NOT IN THIS LOOP. Measured: with the 2x resize, overlay
        # and JPEG encode inline, a 50 Hz request delivered 31.9 Hz -- the
        # control rate was being set by the image encoder. That is exactly
        # backwards, and it is the kind of coupling that only shows up as
        # "the loop is a bit slow" rather than as an error. The renderer now
        # runs in its own thread off the latest frame; it can drop frames
        # freely, the control loop cannot.
        state['frame'] = frame
        state['draw'] = (target, ex_px, lat)

        dt = time.perf_counter() - t0
        inst = 1.0 / max(time.perf_counter() - t_loop, 1e-6)
        t_loop = time.perf_counter()
        hz_ema = inst if hz_ema == 0 else hz_ema * 0.9 + inst * 0.1
        state['loop_hz'] = hz_ema
        time.sleep(max(0.0, period - dt))

    try:
        fc.disarm(timeout=5.0)
    except Exception:                                        # noqa: BLE001
        pass
    cap.release()
    det.close()
    m.close()
    guard.release()


def renderer():
    """Encode the operator view off the latest frame, at its own pace.

    Deliberately decoupled: this thread may skip frames, the control loop may
    not. It reads a snapshot rather than locking, because a torn read here costs
    one slightly-stale overlay and a lock would put the encoder back in the
    control loop's path by another route.
    """
    EYE, HOT, DIM = (255, 168, 74), (138, 217, 57), (70, 82, 96)
    s2 = 2.0
    cxp, cyp = W, H
    db = int(DEADBAND_PX * s2)
    while not _stop.is_set():
        frame = state.get('frame')
        if frame is None:
            time.sleep(0.02)
            continue
        target, ex_px, lat = state.get('draw', (None, 0.0, 0.0))
        vis = cv2.resize(frame, (W * 2, H * 2), interpolation=cv2.INTER_LINEAR)
        vis = cv2.convertScaleAbs(vis, alpha=0.82, beta=6)   # calm the whites

        shade = vis.copy()
        cv2.rectangle(shade, (cxp - db, 0), (cxp + db, H * 2), (40, 46, 54), -1)
        cv2.addWeighted(shade, 0.35, vis, 0.65, 0, vis)
        cv2.line(vis, (cxp, 0), (cxp, H * 2), DIM, 1)
        cv2.line(vis, (0, cyp), (W * 2, cyp), DIM, 1)
        for tick in range(-3, 4):
            x = cxp + int(tick * (W / 3.5))
            cv2.line(vis, (x, cyp - 7), (x, cyp + 7), DIM, 1)

        if target is not None:
            x1, y1, x2, y2 = [int(v * s2) for v in target.xyxy]
            tx, ty = int(target.cx * s2), int(target.cy * s2)
            # corner brackets, not a full box -- they occlude less of the thing
            # you are trying to look at
            L = max(14, min(46, (x2 - x1) // 4))
            for (px, py, dx, dy) in ((x1, y1, 1, 1), (x2, y1, -1, 1),
                                     (x1, y2, 1, -1), (x2, y2, -1, -1)):
                cv2.line(vis, (px, py), (px + dx * L, py), EYE, 2)
                cv2.line(vis, (px, py), (px, py + dy * L), EYE, 2)
            cv2.line(vis, (cxp, cyp), (tx, ty), EYE, 2, cv2.LINE_AA)
            cv2.circle(vis, (tx, ty), 5, EYE, -1, cv2.LINE_AA)
            live = abs(ex_px) > DEADBAND_PX
            cv2.putText(vis, f'{target.class_name} {target.score:.2f}',
                        (x1, max(18, y1 - 9)), cv2.FONT_HERSHEY_SIMPLEX,
                        0.55, EYE, 2, cv2.LINE_AA)
            cv2.putText(vis, f'{ex_px:+.0f} px', (tx + 10, ty - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        HOT if live else DIM, 2, cv2.LINE_AA)
            if abs(lat) > 1e-3:
                bl = int(lat * (W * 0.9))
                cv2.rectangle(vis, (cxp, H * 2 - 26),
                              (cxp + bl, H * 2 - 14), HOT, -1)
            cv2.putText(vis, f'lat {lat:+.3f}', (14, H * 2 - 16),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, HOT, 1, cv2.LINE_AA)
        else:
            cv2.putText(vis, 'no target', (14, H * 2 - 16),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, DIM, 1, cv2.LINE_AA)

        enc, buf = cv2.imencode('.jpg', vis, [cv2.IMWRITE_JPEG_QUALITY, 72])
        if enc:
            state['jpg'] = buf.tobytes()
        time.sleep(0.028)          # ~35 Hz of video is plenty for an eye


PAGE = ("""<title>Mongla bench</title>
<style>
:root{
  --bg:#07090c; --panel:#0e1218; --line:#1b222c; --ink:#c9d4e2; --dim:#5d6b7d;
  --hot:#39d98a;      /* commanded / forward       */
  --cold:#ff5b6e;     /* reverse                   */
  --eye:#4aa8ff;      /* what the camera sees      */
  --warn:#ffb84a;
  --mono:ui-monospace,"SF Mono",Menlo,Consolas,monospace;
}
*{box-sizing:border-box}
body{margin:0;background:
   radial-gradient(1200px 700px at 20% -10%,#101722 0%,var(--bg) 60%);
   color:var(--ink);font:13px/1.45 system-ui,sans-serif;-webkit-font-smoothing:antialiased}
header{display:flex;align-items:baseline;gap:16px;padding:14px 20px 10px;
  border-bottom:1px solid var(--line)}
h1{margin:0;font-size:15px;letter-spacing:.14em;text-transform:uppercase;font-weight:600}
header .sub{color:var(--dim);font:11px var(--mono);letter-spacing:.06em}
.grid{display:grid;grid-template-columns:minmax(420px,1.35fr) minmax(360px,1fr);
  gap:14px;padding:14px 20px 24px;align-items:start}
.card{background:linear-gradient(180deg,#0f141b,#0b0f14);border:1px solid var(--line);
  border-radius:10px;overflow:hidden;box-shadow:0 10px 30px -18px #000}
.card h2{margin:0;padding:9px 13px;font-size:10px;letter-spacing:.16em;
  text-transform:uppercase;color:var(--dim);border-bottom:1px solid var(--line);
  font-weight:600}
.card .body{padding:12px 13px}
img{display:block;width:100%}
canvas{display:block;width:100%}
.rows{display:grid;grid-template-columns:auto 1fr;gap:5px 12px;align-items:center;
  font:12px var(--mono)}
.rows .k{color:var(--dim);letter-spacing:.04em}
.val{font:13px var(--mono);color:var(--ink)}
.state{font-size:19px;font-weight:700;letter-spacing:.1em}
.armed{color:var(--hot)}.safe{color:var(--dim)}
.btns{display:flex;gap:9px;padding:12px 13px 13px}
button{flex:1;font:600 13px system-ui;letter-spacing:.09em;text-transform:uppercase;
  padding:13px;border:1px solid var(--line);border-radius:8px;cursor:pointer;
  background:#121821;color:var(--ink);transition:.14s}
button:hover{transform:translateY(-1px)}
button.go{background:#10301f;border-color:#1d5a38;color:var(--hot)}
button.go:hover{background:#164028}
button.stop{background:#2c1114;border-color:#5a1d24;color:var(--cold)}
button.stop:hover{background:#3a171b}
.chip{display:inline-block;padding:2px 7px;border-radius:20px;font:10px var(--mono);
  border:1px solid var(--line);color:var(--dim)}
.chip.on{color:var(--hot);border-color:#1d5a38}
.legend{display:flex;gap:14px;padding:0 13px 11px;font:10px var(--mono);color:var(--dim)}
.sw{display:inline-block;width:9px;height:9px;border-radius:2px;margin-right:5px;
   vertical-align:-1px}
</style>

<header>
  <h1>Mongla &mdash; vision to thrust</h1>
  <div class=sub>camera &rarr; hailo-8 &rarr; control law &rarr; srot &rarr; thrusters</div>
</header>

<div class=grid>
  <div>
    <div class=card>
      <h2>Forward camera <span id=cls class=chip></span></h2>
      <img src='/stream'>
      <div class=legend>
        <span><i class=sw style="background:var(--eye)"></i>target</span>
        <span><i class=sw style="background:#4a5260"></i>deadband</span>
        <span>&plusmn;<span id=dbtxt>25</span>&nbsp;px</span>
      </div>
    </div>
    <div class=card style="margin-top:14px">
      <h2>Correlation &mdash; does the command follow the target?</h2>
      <canvas id=trace height=170></canvas>
      <div class=legend>
        <span><i class=sw style="background:rgba(74,168,255,.35)"></i>bearing raw</span>
        <span><i class=sw style="background:var(--eye)"></i>bearing filtered</span>
        <span><i class=sw style="background:var(--hot)"></i>lateral command</span>
        <span id=fnote></span>
      </div>
    </div>
  </div>

  <div>
    <div class=card>
      <h2>Vehicle &mdash; top down</h2>
      <canvas id=auv height=380></canvas>
      <div class=legend>
        <span><i class=sw style="background:var(--hot)"></i>thrust fwd</span>
        <span><i class=sw style="background:var(--cold)"></i>thrust reverse</span>
        <span><i class=sw style="background:var(--eye)"></i>bearing to target</span>
        <span id=frchip class=chip></span>
      </div>
      <div class=body style="border-top:1px solid var(--line)">
        <div style="display:flex;justify-content:space-between;
                    font:10px var(--mono);color:var(--dim);letter-spacing:.1em;
                    text-transform:uppercase;margin-bottom:7px">
          <span>thruster &mdash; commanded vs measured</span>
          <span id=escnote></span>
        </div>
        <canvas id=thr height=112></canvas>
      </div>
    </div>

    <div class=card style="margin-top:14px">
      <h2>Board</h2>
      <div class=body>
        <div class=rows>
          <div class=k>state</div><div><span id=arm class="state safe">&mdash;</span>
            <span id=mode class=chip></span></div>
          <div class=k>target</div><div class=val id=tgt>&mdash;</div>
          <div class=k>pixel error</div><div class=val id=ex>&mdash;</div>
          <div class=k>bearing</div><div class=val id=bear>&mdash;</div>
          <div class=k>loop</div><div class=val id=hz>&mdash;</div>
          <div class=k>depth / heading</div><div class=val id=nav>&mdash;</div>
          <div class=k>note</div><div class=val id=msg style="color:var(--warn)"></div>
        </div>
      </div>
      <div class=btns>
        <button class=go onclick="fetch('/arm')">Arm &middot; stabilize</button>
        <button class=stop onclick="fetch('/disarm')">Disarm</button>
      </div>
    </div>
  </div>
</div>

<script>
const $=id=>document.getElementById(id);
let S={armed:false,mode:'',cls:'',score:0,ex:0,bear:null,fwd:0,lat:0,up:0,yaw:0,
       hz:0,sent:0,rpm:[0,0,0,0,0,0,0,0],mix:[0,0,0,0,0,0,0,0],fr:false,
       hist:[],depth:0,yawdeg:0,msg:''};
// R is the RENDERED state: it chases S every frame instead of snapping to it.
// Polling at 25 Hz and repainting on arrival looks stuttery no matter how fast
// the poll is, because the eye sees the STEP. Interpolating decouples the
// visual frame rate from the data rate.
let R={lat:0,ex:0,mix:[0,0,0,0,0,0,0,0],rpm:[0,0,0,0,0,0,0,0],bear:0,hasB:0};
const lerp=(a,b,t)=>a+(b-a)*t;

async function poll(){
  try{
    const d=await (await fetch('/j')).json();
    S=d;
  }catch(e){}
  setTimeout(poll,40);
}
poll();

function fitDPR(c){
  const dpr=window.devicePixelRatio||1, r=c.getBoundingClientRect();
  if(c.width!==Math.round(r.width*dpr)||c.height!==Math.round(r.height*dpr)){
    c.width=Math.round(r.width*dpr); c.height=Math.round(r.height*dpr);
  }
  const x=c.getContext('2d'); x.setTransform(dpr,0,0,dpr,0,0); return x;
}

// ---- the vehicle, drawn the way it is actually built ---------------------
// vectored_6dof: four horizontal thrusters at the corners at 45 deg, four
// vertical in the same corners. Positions and angles match mixer.cpp's columns,
// so an arrow pointing the wrong way here means the MIX is wrong, not the art.
const HORIZ=[ {x: 1,y:-1,a:-45}, {x:-1,y:-1,a:+45},
              {x: 1,y: 1,a:+45}, {x:-1,y: 1,a:-45} ];   // FR FL RR RL
function drawAUV(){
  const c=$('auv'), g=fitDPR(c);
  const w=c.getBoundingClientRect().width, h=c.getBoundingClientRect().height;
  g.clearRect(0,0,w,h);
  const cx=w/2, cy=h/2+6, S1=Math.min(w,h)*0.30;

  // camera field of view, so "what it can see" is part of the picture
  const half=(63.82/2)*Math.PI/180;
  const gr=g.createLinearGradient(cx,cy-S1,cx,cy-S1*3.1);
  gr.addColorStop(0,'rgba(74,168,255,.16)'); gr.addColorStop(1,'rgba(74,168,255,0)');
  g.beginPath(); g.moveTo(cx,cy-S1*0.2);
  g.lineTo(cx+Math.sin(-half)*S1*3.1, cy-Math.cos(-half)*S1*3.1);
  g.lineTo(cx+Math.sin( half)*S1*3.1, cy-Math.cos( half)*S1*3.1);
  g.closePath(); g.fillStyle=gr; g.fill();

  // hull
  g.save(); g.translate(cx,cy);
  g.beginPath();
  const R1=S1*0.92, oct=8;
  for(let i=0;i<oct;i++){const a=Math.PI/oct+i*2*Math.PI/oct;
    const px=Math.cos(a)*R1*0.86, py=Math.sin(a)*R1;
    i?g.lineTo(px,py):g.moveTo(px,py);} g.closePath();
  g.fillStyle='#121922'; g.fill();
  g.strokeStyle='#2b3644'; g.lineWidth=1.5; g.stroke();
  // nose marker
  g.beginPath(); g.moveTo(0,-R1*0.99); g.lineTo(-7,-R1*0.72); g.lineTo(7,-R1*0.72);
  g.closePath(); g.fillStyle='#2b3644'; g.fill();

  // horizontal thrusters: nacelle + thrust arrow along its own axis
  HORIZ.forEach((t,i)=>{
    const px=t.x*R1*0.70, py=t.y*R1*0.62, v=R.mix[i];
    g.save(); g.translate(px,py); g.rotate(t.a*Math.PI/180);
    g.beginPath(); g.roundRect(-8,-15,16,30,5);
    g.fillStyle='#0d131a'; g.fill(); g.strokeStyle='#2b3644'; g.stroke();
    const L=Math.abs(v)*46, dir=v>=0?-1:1;
    if(L>1.5){
      g.beginPath(); g.moveTo(0,dir*14); g.lineTo(0,dir*(14+L));
      g.strokeStyle=v>=0?'#39d98a':'#ff5b6e'; g.lineWidth=4.5;
      g.lineCap='round'; g.stroke();
      g.beginPath(); g.moveTo(0,dir*(14+L+5));
      g.lineTo(-4.5,dir*(14+L-2)); g.lineTo(4.5,dir*(14+L-2)); g.closePath();
      g.fillStyle=v>=0?'#39d98a':'#ff5b6e'; g.fill();
    }
    g.restore();
    g.fillStyle='#4a5260'; g.font='9px ui-monospace';
    g.textAlign='center'; g.fillText('M'+(i+1),px,py+t.y*30+3);
  });

  // vertical thrusters as rings whose fill is |command|
  [4,5,6,7].forEach((i,k)=>{
    const t=HORIZ[k], px=t.x*R1*0.30, py=t.y*R1*0.28, v=R.mix[i];
    g.beginPath(); g.arc(px,py,9,0,7); g.strokeStyle='#2b3644'; g.lineWidth=1.4; g.stroke();
    if(Math.abs(v)>0.02){
      g.beginPath(); g.arc(px,py,9,-Math.PI/2,-Math.PI/2+Math.abs(v)*6.283);
      g.strokeStyle=v>=0?'#39d98a':'#ff5b6e'; g.lineWidth=3; g.stroke();
    }
  });

  // Resultant translation demand -- the single arrow that says "this way".
  //
  // Written as a direct vector, NOT as a rotation. The first version composed
  // atan2 with a rotate() and got the axes SWAPPED and forward NEGATED: a
  // command to port drew an arrow pointing aft. It looked completely
  // plausible -- an arrow of the right length, moving when the target moved --
  // which is exactly why it survived being looked at. In screen space the
  // mapping is one line and cannot be got subtly wrong:
  //     +lat (starboard) -> +x (right)      +fwd -> -y (up, toward the nose)
  const vx = R.lat, vy = -S.fwd, RL = Math.hypot(vx,vy)*S1*1.6;
  if(RL>3){
    const ux=vx/Math.hypot(vx,vy), uy=vy/Math.hypot(vx,vy);
    const ex=ux*RL, ey=uy*RL;
    g.beginPath(); g.moveTo(0,0); g.lineTo(ex,ey);
    g.strokeStyle='rgba(57,217,138,.95)'; g.lineWidth=3; g.lineCap='round'; g.stroke();
    // arrowhead built from the unit vector and its perpendicular
    const px=-uy, py=ux;
    g.beginPath(); g.moveTo(ex+ux*9, ey+uy*9);
    g.lineTo(ex+px*6, ey+py*6); g.lineTo(ex-px*6, ey-py*6);
    g.closePath(); g.fillStyle='rgba(57,217,138,.95)'; g.fill();
  }
  g.restore();

  // bearing to target, drawn from the hull out through the FOV cone
  if(R.hasB>0.02){
    g.save(); g.translate(cx,cy);
    const a=R.bear, L=S1*2.6;
    g.beginPath(); g.moveTo(0,0);
    g.lineTo(Math.sin(a)*L,-Math.cos(a)*L);
    g.strokeStyle='rgba(74,168,255,'+(0.35+0.5*R.hasB)+')';
    g.lineWidth=2; g.setLineDash([7,5]); g.stroke(); g.setLineDash([]);
    g.beginPath(); g.arc(Math.sin(a)*L,-Math.cos(a)*L,4.5,0,7);
    g.fillStyle='#4aa8ff'; g.fill();
    g.restore();
  }
}

// ---- the correlation trace ----------------------------------------------
function drawTrace(){
  const c=$('trace'), g=fitDPR(c);
  const w=c.getBoundingClientRect().width, h=c.getBoundingClientRect().height;
  g.clearRect(0,0,w,h);
  const mid=h/2;
  g.strokeStyle='#1b222c'; g.lineWidth=1;
  g.beginPath(); g.moveTo(0,mid); g.lineTo(w,mid); g.stroke();
  const H=S.hist||[]; if(H.length<2) return;
  const n=H.length, dx=w/(n-1);
  // RAW bearing, thin. Drawn behind the filtered one so the difference between
  // them IS the visualisation -- "78 % jitter reduction" is a number; a hairy
  // line with a clean one through it is the same fact you can see.
  g.beginPath();
  H.forEach((p,i)=>{const y=mid-(p[0]/32)*(h*0.42);
    i?g.lineTo(i*dx,y):g.moveTo(i*dx,y);});
  g.strokeStyle='rgba(74,168,255,.30)'; g.lineWidth=1; g.stroke();
  // FILTERED bearing -- what the loop actually steers on
  g.beginPath();
  H.forEach((p,i)=>{const y=mid-((p[3]||0)/32)*(h*0.42);
    i?g.lineTo(i*dx,y):g.moveTo(i*dx,y);});
  g.strokeStyle='rgba(74,168,255,.95)'; g.lineWidth=1.8; g.stroke();
  // lateral command, scaled to full authority
  g.beginPath();
  H.forEach((p,i)=>{const y=mid-p[1]*(h*0.42);
    i?g.lineTo(i*dx,y):g.moveTo(i*dx,y);});
  g.strokeStyle='rgba(57,217,138,.95)'; g.lineWidth=2; g.stroke();
  // shade the gaps where nothing was detected -- absence is information here
  g.fillStyle='rgba(255,91,110,.07)';
  let run=-1;
  H.forEach((p,i)=>{ if(!p[2]&&run<0) run=i;
    else if(p[2]&&run>=0){ g.fillRect(run*dx,0,(i-run)*dx,h); run=-1; }});
  if(run>=0) g.fillRect(run*dx,0,(n-run)*dx,h);
}

function drawThrusters(){
  const c=$('thr'), g=fitDPR(c);
  const w=c.getBoundingClientRect().width, h=c.getBoundingClientRect().height;
  g.clearRect(0,0,w,h);
  const n=8, pad=6, bw=(w-pad*(n-1))/n, mid=h*0.55;
  for(let i=0;i<n;i++){
    const x=i*(bw+pad), v=R.mix[i], rp=R.rpm[i]/3600;
    g.strokeStyle='#1b222c'; g.lineWidth=1;
    g.beginPath(); g.moveTo(x,mid); g.lineTo(x+bw,mid); g.stroke();
    // COMMANDED: a hollow bar. It is a prediction of what the board will do,
    // computed from the same mixer the firmware runs.
    const ch=Math.abs(v)*(h*0.42);
    if(ch>0.8){
      g.strokeStyle=v>=0?'#39d98a':'#ff5b6e'; g.lineWidth=1.6;
      g.strokeRect(x+0.5, v>=0?mid-ch:mid, bw-1, ch);
    }
    // MEASURED: solid fill. Zero with no ESCs attached, which is the truth and
    // must not look like the command failed.
    const mh=Math.abs(rp)*(h*0.42);
    if(mh>0.8){
      g.fillStyle=rp>=0?'rgba(57,217,138,.55)':'rgba(255,91,110,.55)';
      g.fillRect(x+2, rp>=0?mid-mh:mid, bw-4, mh);
    }
    g.fillStyle='#4a5260'; g.font='9px ui-monospace'; g.textAlign='center';
    g.fillText('M'+(i+1), x+bw/2, h-3);
  }
}
function frame(){
  const k=0.25;                       // chase constant -> ~4 frame settle
  R.lat=lerp(R.lat,S.lat,k);
  R.ex=lerp(R.ex,S.ex,k);
  for(let i=0;i<8;i++){
    R.mix[i]=lerp(R.mix[i],S.mix[i]||0,k);
    R.rpm[i]=lerp(R.rpm[i],S.rpm[i]||0,0.18);
  }
  if(S.bear!=null){R.bear=lerp(R.bear,S.bear*Math.PI/180,k);R.hasB=lerp(R.hasB,1,0.2);}
  else R.hasB=lerp(R.hasB,0,0.12);

  $('arm').textContent=S.armed?'ARMED':'DISARMED';
  $('arm').className='state '+(S.armed?'armed':'safe');
  $('mode').textContent=S.mode||'';
  $('mode').className='chip'+(S.armed?' on':'');
  $('cls').textContent=S.cls?S.cls+' '+S.score.toFixed(2):'no target';
  $('cls').className='chip'+(S.cls?' on':'');
  $('tgt').textContent=S.cls?S.cls+'  '+S.score.toFixed(2):'—';
  $('ex').textContent=S.cls?S.ex.toFixed(0)+' px':'—';
  $('bear').textContent=S.bear==null?'—':S.bear.toFixed(2)+'°';
  $('hz').textContent=S.hz.toFixed(1)+' Hz · '+S.sent+' frames';
  $('nav').textContent=S.depth.toFixed(2)+' m · '+S.yawdeg.toFixed(0)+'°';
  $('msg').textContent=S.msg||'';
  $('frchip').textContent='FRAME_REVERSE '+(S.fr?'1':'0');
  $('frchip').className='chip'+(S.fr?' on':'');

  drawAUV(); drawTrace(); drawThrusters();
  // Zeros here are honest only if the stream is alive. Say which, because a
  // dead stream and a stopped thruster look identical in a number.
  $('fnote').textContent='α-β 0.25/0.02 · gate 12° · '+S.resets+' snaps';
  $('escnote').textContent = S.esc_msgs>0
    ? (R.rpm.some(v=>Math.abs(v)>1) ? 'esc telemetry live'
                                    : 'esc stream live \u00b7 0 rpm (no escs attached)')
    : 'no esc telemetry';
  requestAnimationFrame(frame);
}
requestAnimationFrame(frame);
</script>""").encode()


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _send(self, body, ctype='text/plain'):
        self.send_response(200)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path == '/arm':
            state['want_arm'] = True
            return self._send(b'arming')
        if self.path == '/disarm':
            state['want_arm'] = False
            return self._send(b'disarming')
        if self.path == '/j':
            d = state['det']
            b = state['bearing']
            fwd, lat, up, yaw = state['axes']
            payload = {
                'armed': state['armed'], 'mode': state['mode'],
                'cls': d.class_name if d else '', 'score': d.score if d else 0.0,
                'ex': (d.cx - W / 2.0) if d else 0.0,
                'bear': (math.degrees(b.angle_x) if b else None),
                'fwd': fwd, 'lat': lat, 'up': up, 'yaw': yaw,
                'hz': state['loop_hz'], 'sent': state['sent'],
                'rpm': state['rpm'], 'msg': state['msg'],
                'mix': [round(v, 3) for v in state['mix']],
                'fr': state['frame_reverse'],
                'esc_msgs': state['esc_msgs'],
                'braw': (math.degrees(state['braw'].angle_x)
                         if state.get('braw') else None),
                'resets': state['bfilt_resets'],
                'hist': state['hist'][-160:],
                'depth': state['depth'] if state['depth'] == state['depth'] else 0.0,
                'yawdeg': state['yaw'] if state['yaw'] == state['yaw'] else 0.0,
            }
            return self._send(json.dumps(payload).encode(), 'application/json')
        if self.path != '/stream':
            return self._send(PAGE, 'text/html')
        # ThreadingHTTPServer is required: this handler never returns, so a
        # single-threaded server could never answer /j or /disarm.
        self.send_response(200)
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=f')
        self.end_headers()
        try:
            while not _stop.is_set():
                j = state['jpg']
                if j:
                    self.wfile.write(b'--f\r\nContent-Type: image/jpeg\r\n'
                                     b'Content-Length: ' + str(len(j)).encode()
                                     + b'\r\n\r\n' + j + b'\r\n')
                time.sleep(0.02)
        except Exception:                                    # noqa: BLE001
            pass


threading.Thread(target=worker, daemon=True).start()
threading.Thread(target=renderer, daemon=True).start()
print(where(A.port) + f'   class={A.klass} gain={A.gain}%',
      flush=True)
try:
    ThreadingHTTPServer(('0.0.0.0', A.port), Handler).serve_forever()
except KeyboardInterrupt:
    _stop.set()
    time.sleep(0.7)
