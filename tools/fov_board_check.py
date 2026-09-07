#!/usr/bin/env python3
"""Live board screening in a browser: aim, identify the grid, measure flatness.

Combines three things that all have to be right before calibration is worth
running, and shows them live so none of them is a guess:

  AIM      -- the Pi is headless; MJPEG over HTTP is the only view.
  GRID     -- findChessboardCorners needs the exact inner-corner count and
              gives no warning on a wrong guess, so sweep candidates until one
              locks, then stop sweeping.
  FLATNESS -- a flat board maps to the ideal grid by a HOMOGRAPHY exactly,
              from any angle and through any lens. Residual above corner noise
              IS bow. Only readings at real obliquity count -- bow is
              invisible head-on.

  board_live.py <device> [--port 8089]
"""
import sys, threading, time
from http.server import BaseHTTPRequestHandler, HTTPServer
import numpy as np, cv2
import os as _os, sys as _sys
_sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))
from _where import where   # never print a hardcoded IP
cv2.setNumThreads(2)

DEV = int(sys.argv[1]); PORT = 8089
for i,a in enumerate(sys.argv):
    if a == '--port': PORT = int(sys.argv[i+1])
DW, DH = 640, 360
VW, VH = 960, 540
CRIT  = (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
SWEEP = cv2.CALIB_CB_ADAPTIVE_THRESH|cv2.CALIB_CB_NORMALIZE_IMAGE|cv2.CALIB_CB_FAST_CHECK
FIND  = cv2.CALIB_CB_ADAPTIVE_THRESH|cv2.CALIB_CB_NORMALIZE_IMAGE
CANDS = [(11,8),(10,7),(9,7),(9,6),(8,6),(8,5),(7,6),(7,5),(7,4),(6,5),(6,4),(5,4),(4,3)]

state = {'jpg': None}

def worker():
    cap = cv2.VideoCapture(DEV, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    grid_wh, obj, ci = None, None, 0
    res, tf, t0 = [], 0, time.time()
    while True:
        ok, f = cap.read()
        if not ok: time.sleep(0.05); continue
        small = cv2.resize(f, (DW, DH))
        g = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
        vis = cv2.resize(f, (VW, VH))
        msg, col = "aim the board at the camera", (60,180,255)

        if grid_wh is None:                      # sweep one candidate per frame
            cw, ch = CANDS[ci % len(CANDS)]; ci += 1
            found, _ = cv2.findChessboardCorners(g, (cw, ch), SWEEP)
            if found:
                grid_wh = (cw, ch)
                obj = np.mgrid[0:cw,0:ch].T.reshape(-1,2).astype(np.float32)
            msg = f"searching for grid... trying {cw}x{ch}"
        else:
            cw, ch = grid_wh
            found, c = cv2.findChessboardCorners(g, (cw, ch), FIND)
            if found:
                c = cv2.cornerSubPix(g, c, (7,7), (-1,-1), CRIT)
                cv2.drawChessboardCorners(vis, (cw,ch), c*(VW/DW), True)
                p2 = c.reshape(-1,2)
                H, _ = cv2.findHomography(obj, p2, 0)
                pr = cv2.perspectiveTransform(obj.reshape(-1,1,2), H).reshape(-1,2)
                r  = float(np.linalg.norm(p2-pr, axis=1).mean()) * (1280.0/DW)
                d  = p2 - pr
                du = pr[obj[:,0].argmax()] - pr[obj[:,0].argmin()]
                du = du/max(np.linalg.norm(du),1e-9)
                uu = obj[:,0]-obj[:,0].mean(); vv = obj[:,1]-obj[:,1].mean()
                A  = np.column_stack([np.ones_like(uu),uu,vv,uu*uu,vv*vv,uu*vv])
                co,*_ = np.linalg.lstsq(A, d@du, rcond=None)
                qu, qv = abs(co[3])*(1280.0/DW), abs(co[4])*(1280.0/DW)
                q  = p2.reshape(ch, cw, 2)
                tl,tr,bl,br = q[0,0],q[0,-1],q[-1,0],q[-1,-1]
                top,bot = np.linalg.norm(tr-tl), np.linalg.norm(br-bl)
                lft,rgt = np.linalg.norm(bl-tl), np.linalg.norm(br-tr)
                obl = (max(top,bot)/max(min(top,bot),1e-6))*(max(lft,rgt)/max(min(lft,rgt),1e-6))
                if obl < 1.15:
                    msg, col = f"{cw}x{ch} FOUND -- now TILT the board ~40 deg", (60,180,255)
                else:
                    res.append(r); res[:] = res[-40:]
                    ax = ('curls TOP-BOTTOM' if qv > 3*max(qu,1e-6) else
                          'curls LEFT-RIGHT'  if qu > 3*max(qv,1e-6) else
                          'even' )
                    msg, col = (f"{cw}x{ch}  resid {r:.3f} px  tilt {obl:.2f}  "
                                f"curve u {qu:.3f} v {qv:.3f}  {ax}", (80,240,80))
            else:
                msg, col = f"{cw}x{ch} locked -- board not visible", (60,140,255)

        tf += 1; fps = tf/max(time.time()-t0, 1e-6)
        bar = np.zeros((96, VW, 3), np.uint8)
        cv2.putText(bar, msg, (12,30), cv2.FONT_HERSHEY_SIMPLEX, 0.75, col, 2)
        if res:
            m = float(np.median(res))
            v,vc = (("FLAT - calibrate with this",(80,240,80)) if m<0.35 else
                    ("USABLE, slight bow",(80,220,220))       if m<0.60 else
                    ("BOWED - mount it flatter",(60,180,255)) if m<1.20 else
                    ("BADLY BOWED",(60,60,240)))
            cv2.putText(bar, f"median {m:.3f} px over {len(res)}  ->  {v}",
                        (12,64), cv2.FONT_HERSHEY_SIMPLEX, 0.7, vc, 2)
        cv2.putText(bar, f"prev board 1.365 | flat<0.35 slight<0.60 bowed<1.20 | {fps:.0f} fps",
                    (12,88), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (170,170,170), 1)
        ok2, buf = cv2.imencode('.jpg', np.vstack([vis, bar]),
                                [cv2.IMWRITE_JPEG_QUALITY, 55])
        if ok2: state['jpg'] = buf.tobytes()

class H(BaseHTTPRequestHandler):
    def log_message(self, *a): pass
    def do_GET(self):
        if self.path != '/stream':
            self.send_response(200); self.send_header('Content-Type','text/html')
            self.end_headers()
            self.wfile.write(b"<html><body style='margin:0;background:#111;"
                             b"text-align:center'><img src='/stream' "
                             b"style='width:100%;max-width:1100px'></body></html>")
            return
        self.send_response(200)
        self.send_header('Content-Type','multipart/x-mixed-replace; boundary=f')
        self.end_headers()
        try:
            while True:
                j = state['jpg']
                if j:
                    self.wfile.write(b'--f\r\nContent-Type: image/jpeg\r\n'
                                     b'Content-Length: '+str(len(j)).encode()+b'\r\n\r\n'+j+b'\r\n')
                time.sleep(0.01)
        except Exception: pass

threading.Thread(target=worker, daemon=True).start()
print(where(PORT), flush=True)
HTTPServer(('0.0.0.0', PORT), H).serve_forever()
