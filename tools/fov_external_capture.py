#!/usr/bin/env python3
"""Capture a frontal board shot for the EXTERNAL calibration check.

Everything else in this pipeline is internal to the optimizer -- reprojection
error, held-out error, fold spread -- and can be self-consistently wrong. Two
independent calibrations of this camera disagreed by 1.93 % while each claimed
+/-0.1 %. This is the one measurement that does not come from the optimizer:

    fx = pixel_span * distance / board_width

Three things bias it, so the overlay gates on all three:

  * TILT. A board rotated away from the lens is foreshortened, so its pixel
    span shrinks and fx reads LOW. Wanted: obliquity ~= 1.00 (square to lens).
  * OFF-CENTRE. Radial distortion is strongest at the edges, so a board in the
    corner has a span the pinhole formula does not describe. Wanted: centred.
  * DISTORTION itself. Even centred, the raw span carries k1. The tool
    therefore reports BOTH the raw span and the span after undistorting the
    corners with the existing calibration -- the difference is the size of
    that effect, stated rather than assumed.

  ext_capture.py <device> <outfile> [--grid 8x6] [--port 8090] [--calib path]
"""
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

cv2.setNumThreads(2)

DEV = int(sys.argv[1])
OUT = sys.argv[2]
GRID, PORT, CALIB = (8, 6), 8090, None
for i, a in enumerate(sys.argv):
    if a == '--grid':
        GRID = tuple(int(x) for x in sys.argv[i + 1].lower().split('x'))
    if a == '--port':
        PORT = int(sys.argv[i + 1])
    if a == '--calib':
        CALIB = sys.argv[i + 1]
COLS, ROWS = GRID
DW, DH = 640, 360
VW, VH = 960, 540
FIND = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
CRIT = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

state = {'jpg': None, 'frame': None, 'ok': False, 'shot': 0, 'msg': ''}


def metrics(p, w, h):
    """Return (obliquity, centre offset frac, long-axis span px, diag span px)."""
    q = p.reshape(ROWS, COLS, 2)
    tl, tr, bl, br = q[0, 0], q[0, -1], q[-1, 0], q[-1, -1]
    top, bot = np.linalg.norm(tr - tl), np.linalg.norm(br - bl)
    lft, rgt = np.linalg.norm(bl - tl), np.linalg.norm(br - tr)
    obl = (max(top, bot) / max(min(top, bot), 1e-6)) * \
          (max(lft, rgt) / max(min(lft, rgt), 1e-6))
    cx, cy = p[:, 0].mean(), p[:, 1].mean()
    off = np.hypot(cx - w / 2, cy - h / 2) / (w / 2)
    long_px = max(top, bot, lft, rgt)
    diag = max(np.linalg.norm(br - tl), np.linalg.norm(bl - tr))
    return obl, off, long_px, diag


def worker():
    cap = cv2.VideoCapture(DEV, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    while True:
        got, f = cap.read()
        if not got:
            time.sleep(0.05)
            continue
        state['frame'] = f
        small = cv2.resize(f, (DW, DH))
        g = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
        vis = cv2.resize(f, (VW, VH))
        found, c = cv2.findChessboardCorners(g, GRID, FIND)
        lines, col = ['show the whole board, SQUARE to the lens'], (60, 180, 255)
        state['ok'] = False
        if found:
            cv2.drawChessboardCorners(vis, GRID, c * (VW / DW), True)
            p = c.reshape(-1, 2) * (1280.0 / DW)
            obl, off, long_px, diag = metrics(p, 1280, 720)
            square = obl < 1.04
            centred = off < 0.22
            state['ok'] = square and centred
            lines = [
                f"tilt {obl:.3f}  {'SQUARE' if square else 'TOO TILTED - face the lens'}",
                f"offset {off*100:.0f}% from centre  "
                f"{'CENTRED' if centred else 'MOVE TO CENTRE'}",
                f"span {long_px:.1f} px   diagonal {diag:.1f} px",
            ]
            col = (80, 240, 80) if state['ok'] else (60, 180, 255)
        bar = np.zeros((120, VW, 3), np.uint8)
        for i, t in enumerate(lines):
            cv2.putText(bar, t, (12, 26 + i * 28), cv2.FONT_HERSHEY_SIMPLEX,
                        0.62, col, 2)
        tag = 'READY - press SHOOT' if state['ok'] else 'not ready'
        cv2.putText(bar, f"{tag}    shots {state['shot']}  {state['msg']}",
                    (12, 112), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (80, 240, 80) if state['ok'] else (150, 150, 150), 1)
        enc, buf = cv2.imencode('.jpg', np.vstack([vis, bar]),
                                [cv2.IMWRITE_JPEG_QUALITY, 55])
        if enc:
            state['jpg'] = buf.tobytes()


PAGE = b"""<html><body style='margin:0;background:#111;color:#ddd;
font-family:system-ui;text-align:center'>
<img src='/stream' style='width:100%;max-width:1000px'><br>
<button onclick="fetch('/shoot').then(r=>r.text()).then(t=>{
  document.getElementById('s').innerText=t})"
 style='font-size:26px;padding:16px 52px;margin:14px;background:#2b7;
 border:0;border-radius:8px;color:#fff;cursor:pointer'>SHOOT</button>
<div id='s' style='font-size:17px;padding:8px'></div>
</body></html>"""


class H(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        if self.path == '/shoot':
            f = state['frame']
            if f is None:
                body = b'no frame'
            elif not state['ok']:
                # refuse rather than save a shot the check cannot use --
                # a tilted or off-centre frame biases fx and looks fine
                body = b'NOT READY - board must be square to the lens and centred'
            else:
                state['shot'] += 1
                path = OUT if state['shot'] == 1 else \
                    OUT.replace('.png', f'_{state["shot"]}.png')
                cv2.imwrite(path, f)
                state['msg'] = f'saved {os.path.basename(path)}'
                body = f'SAVED {path} - tell Claude'.encode()
            self.send_response(200)
            self.send_header('Content-Type', 'text/plain')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if self.path != '/stream':
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.send_header('Content-Length', str(len(PAGE)))
            self.end_headers()
            self.wfile.write(PAGE)
            return
        self.send_response(200)
        self.send_header('Content-Type',
                         'multipart/x-mixed-replace; boundary=f')
        self.end_headers()
        try:
            while True:
                j = state['jpg']
                if j:
                    self.wfile.write(b'--f\r\nContent-Type: image/jpeg\r\n'
                                     b'Content-Length: ' +
                                     str(len(j)).encode() + b'\r\n\r\n' + j + b'\r\n')
                time.sleep(0.01)
        except Exception:
            pass


threading.Thread(target=worker, daemon=True).start()
print(f"  open http://10.42.0.28:{PORT}   -> aim, then press SHOOT", flush=True)
# ThreadingHTTPServer, NOT HTTPServer: the MJPEG handler never returns (it
# streams in a while-loop forever), so a single-threaded server can never
# accept a second connection and /shoot hangs with no error anywhere.
ThreadingHTTPServer(('0.0.0.0', PORT), H).serve_forever()
