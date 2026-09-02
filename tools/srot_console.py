#!/usr/bin/env python3
"""Mongla console: the whole srot + Pi + AI HAT stack, live, in one page.

    srot_console.py [--class person] [--gain 25] [--port 8092]

Three threads, and the split is the point:

    control    camera -> Hailo -> bearing -> filter -> MANUAL_CONTROL, 50 Hz
    render     the operator view, ~35 Hz, may drop frames
    reader     200 Hz MAVLink drain + de-multiplex, in BoardTelemetry

The control thread must never be paced by either of the others. That is not a
style preference: the first version of the bench ran the JPEG encode inline and
delivered 31.9 Hz against a 50 Hz request, with nothing in any log to say so.

Everything the page shows is measured on this hardware. Where a value is not
available it is reported ABSENT rather than zero -- on this board that
distinction is load-bearing, because DEPTH_ERR and DEPTH_OUT are SUPPRESSED
while the depth controller is not running and a zero there would read as
"settled" for a loop that never started.
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
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
WS = os.path.expanduser('~/duburi_ws/src')
for _p in ('duburi_control', 'duburi_vision', 'duburi_manager'):
    _q = os.path.join(WS, _p)
    if _q not in sys.path:
        sys.path.insert(0, _q)

import cv2                                                      # noqa: E402
cv2.setNumThreads(0)
from pymavlink import mavutil                                   # noqa: E402

from duburi_control.bearing import (                            # noqa: E402
    BearingFilter, bearing_from_pixels)
from duburi_control.fc import srot_protocol as sp               # noqa: E402
from duburi_control.fc.port_guard import PortGuard              # noqa: E402
from duburi_control.fc.srot_fc import SrotFC                    # noqa: E402
from duburi_vision.detection.factory import make_detector       # noqa: E402
from srot_console_server import (                               # noqa: E402
    SERIAL_CAPACITY_BPS, BoardTelemetry, HostStats)

ap = argparse.ArgumentParser()
ap.add_argument('--class', dest='klass', default='person')
ap.add_argument('--gain', type=float, default=25.0)
ap.add_argument('--port', type=int, default=8092)
ap.add_argument('--model', default='yolov11n')
ap.add_argument('--device', type=int, default=0)
ap.add_argument('--dev', default='/dev/ttyUSB0')
ap.add_argument('--conf', type=float, default=0.35)
ap.add_argument('--hz', type=float, default=50.0)
A = ap.parse_args()

W, H = 640, 360
DEADBAND_DEG = 2.5
CAL = os.path.expanduser('~/duburi_ws/src/duburi_vision/config/'
                         'calibration/pi_forward_1280x720.json')

K = D = None
try:
    _c = json.load(open(CAL))
    _Kc, D = _c['camera_matrix'], _c['distortion_coefficients']
    _sx, _sy = W / _c['image_width'], H / _c['image_height']
    K = [_Kc[0][0] * _sx, 0.0, _Kc[0][2] * _sx,
         0.0, _Kc[1][1] * _sy, _Kc[1][2] * _sy, 0.0, 0.0, 1.0]
except Exception as exc:                                        # noqa: BLE001
    print(f'no calibration ({exc}) -- bearings unavailable', file=sys.stderr)

# The board's own allocation (srot mixer.cpp:25-35), mirrored to visualise what
# it is about to do. Measured RPM is shown beside it, so a divergence between
# prediction and hardware is visible rather than assumed away.
MIX = ((0, 0, 1, 0, -1, 1), (0, 0, -1, 0, -1, -1), (0, 0, -1, 0, 1, 1),
       (0, 0, 1, 0, 1, -1), (1, -1, 0, -1, 0, 0), (-1, -1, 0, -1, 0, 0),
       (1, 1, 0, -1, 0, 0), (-1, 1, 0, -1, 0, 0))


def mix(fwd, lat, up, yaw, frame_reverse=False):
    if frame_reverse:
        fwd, lat, up, yaw = -fwd, -lat, -up, -yaw
    dm = (0.0, 0.0, yaw, up, fwd, lat)
    out = [sum(M[c] * dm[c] for c in range(6)) for M in MIX]
    # Horizontal (1-4) and vertical (5-8) saturate INDEPENDENTLY: the matrix is
    # block-diagonal, so a saturated yaw must not steal vertical authority.
    for lo, hi in ((0, 4), (4, 8)):
        peak = max(1.0, max(abs(v) for v in out[lo:hi]))
        for i in range(lo, hi):
            out[i] /= peak
    return out


ST = {'jpg': None, 'frame': None, 'draw': (None, 0.0, 0.0), 'want': None,
      'vision': {}, 'board': {}, 'host': {}, 'link': {}, 'loop_hz': 0.0,
      'fw_rev': None, 'mode': '?', 'fr': False}
STOP = threading.Event()


def control():
    guard = PortGuard(A.dev)
    guard.acquire()
    conn = mavutil.mavlink_connection(A.dev, baud=115200,
                                      source_system=sp.SOURCE_SYSID,
                                      source_component=sp.SOURCE_COMPID)
    conn.wait_heartbeat(timeout=10)
    fc = SrotFC(conn, log=None)
    tel = BoardTelemetry(fc, conn)
    host = HostStats()
    # 50 Hz ATTITUDE: the board's floor is 20 ms, and a below-floor request is
    # ACCEPTED and silently clamped -- the ACK cannot tell you the rate you got.
    fc.set_message_rate(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 50)
    ST['fw_rev'] = fc.read_behaviour_rev()

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

    bf = BearingFilter()
    hist = []
    period = 1.0 / max(A.hz, 1.0)
    t_prev = time.perf_counter()
    hz = det_hz = 0.0
    t_g = t_i = t_d = t_c = 0.0
    sent = 0
    rx0, trx = tel.rx_bytes, time.time()

    while not STOP.is_set():
        t0 = time.perf_counter()
        want = ST.pop('want', None)
        ST['want'] = None
        if want is True:
            fc.set_mode('STABILIZE')
            ok, why = fc.arm(timeout=12.0)
            ST['msg'] = 'armed' if ok else str(why)
        elif want is False:
            fc.disarm(timeout=8.0)
            ST['msg'] = 'disarmed'

        ok, frame = cap.read()
        t1 = time.perf_counter()
        if not ok:
            time.sleep(0.005)
            continue
        dets = det.infer(frame)
        t2 = time.perf_counter()
        target = max(dets, key=lambda d: d.area) if dets else None

        b_raw = None
        if target is not None and K:
            b_raw = bearing_from_pixels(target.cx, target.cy,
                                        target.width, target.height,
                                        width=W, height=H, K=K, D=D)
        # Filter even on loss: passing None is what clears the velocity state.
        b = bf.update(b_raw, time.perf_counter())
        t3 = time.perf_counter()

        fwd = lat = up = yaw = 0.0
        if b is not None and abs(b.angle_x) > math.radians(DEADBAND_DEG):
            lat = max(-1.0, min(1.0, b.angle_x / math.radians(20.0)))
            lat *= A.gain / 100.0
        # Sent EVERY tick, neutral included: MANUAL_CONTROL authority ramps to
        # zero between MANUAL_FRESH_MS (1000) and MANUAL_DECAY_MS (1500), so a
        # loop that only sends when it sees something decays mid-track.
        fc.manual(fwd=fwd, lat=lat, up=up, yaw=yaw)
        sent += 1
        t4 = time.perf_counter()

        if sent % 250 == 1:
            try:
                fr = fc.get_param('FRAME_REVERSE', timeout=0.4)
                if fr is not None:
                    ST['fr'] = bool(fr > 0.5)
            except Exception:                                   # noqa: BLE001
                pass

        a = 0.12
        t_g = t_g * (1 - a) + (t1 - t0) * 1000 * a
        t_i = t_i * (1 - a) + (t2 - t1) * 1000 * a
        t_d = t_d * (1 - a) + (t3 - t2) * 1000 * a
        t_c = t_c * (1 - a) + (t4 - t3) * 1000 * a
        inst = 1.0 / max(time.perf_counter() - t_prev, 1e-6)
        t_prev = time.perf_counter()
        hz = inst if hz == 0 else hz * .92 + inst * .08
        det_hz = hz

        hist.append((round(math.degrees(b_raw.angle_x), 2) if b_raw else None,
                     round(lat, 4), 1 if target else 0,
                     round(math.degrees(b.angle_x), 2) if b else None))
        del hist[:-200]

        t = fc.telemetry()
        now = time.time()
        if now - trx >= 1.0:
            ST['link'] = {'bps': (tel.rx_bytes - rx0) / (now - trx),
                          'util': ((tel.rx_bytes - rx0) / (now - trx))
                                  / SERIAL_CAPACITY_BPS}
            rx0, trx = tel.rx_bytes, now
        ST['mode'] = t.mode or '?'
        ST['loop_hz'] = hz
        ST['vision'] = {
            'armed': bool(t.armed), 'cls': target.class_name if target else '',
            'score': float(target.score) if target else 0.0,
            'ex': (target.cx - W / 2.0) if target else 0.0,
            'bear': math.degrees(b.angle_x) if b else None,
            'braw': math.degrees(b_raw.angle_x) if b_raw else None,
            'fwd': fwd, 'lat': lat, 'up': up, 'yaw': yaw,
            'mix': [round(v, 3) for v in mix(fwd, lat, up, yaw, ST['fr'])],
            'resets': bf.resets, 'det_hz': det_hz, 'sent': sent, 'hist': hist,
            't_grab': t_g, 't_inf': t_i, 't_dec': t_d, 't_ctl': t_c}
        ST['board'] = tel.snapshot(now)
        ST['board']['mode'] = t.mode
        ST['host'] = host.poll(now)
        ST['frame'] = frame
        ST['draw'] = (target, (target.cx - W / 2.0) if target else 0.0, lat)

        time.sleep(max(0.0, period - (time.perf_counter() - t0)))

    try:
        fc.disarm(timeout=5.0)
    except Exception:                                           # noqa: BLE001
        pass
    tel.stop()
    cap.release()
    det.close()
    conn.close()
    guard.release()


def render():
    """Operator view, at its own pace. May drop frames; control may not."""
    EYE, HOT, DIM = (255, 168, 74), (138, 217, 57), (70, 82, 96)
    s2, cxp, cyp = 2.0, W, H
    while not STOP.is_set():
        frame = ST.get('frame')
        if frame is None:
            time.sleep(0.02)
            continue
        target, ex_px, lat = ST.get('draw', (None, 0.0, 0.0))
        vis = cv2.resize(frame, (W * 2, H * 2), interpolation=cv2.INTER_LINEAR)
        vis = cv2.convertScaleAbs(vis, alpha=0.82, beta=6)
        # deadband drawn in DEGREES converted to pixels, from the same fx the
        # bearing uses -- so the band on screen is the band the loop applies.
        db = int(math.tan(math.radians(DEADBAND_DEG)) * (K[0] if K else 500) * s2)
        shade = vis.copy()
        cv2.rectangle(shade, (cxp - db, 0), (cxp + db, H * 2), (40, 46, 54), -1)
        cv2.addWeighted(shade, 0.35, vis, 0.65, 0, vis)
        cv2.line(vis, (cxp, 0), (cxp, H * 2), DIM, 1)
        cv2.line(vis, (0, cyp), (W * 2, cyp), DIM, 1)
        if target is not None:
            x1, y1, x2, y2 = [int(v * s2) for v in target.xyxy]
            tx, ty = int(target.cx * s2), int(target.cy * s2)
            L = max(14, min(46, (x2 - x1) // 4))
            for (px, py, dx, dy) in ((x1, y1, 1, 1), (x2, y1, -1, 1),
                                     (x1, y2, 1, -1), (x2, y2, -1, -1)):
                cv2.line(vis, (px, py), (px + dx * L, py), EYE, 2)
                cv2.line(vis, (px, py), (px, py + dy * L), EYE, 2)
            cv2.line(vis, (cxp, cyp), (tx, ty), EYE, 2, cv2.LINE_AA)
            cv2.circle(vis, (tx, ty), 5, EYE, -1, cv2.LINE_AA)
            cv2.putText(vis, f'{target.class_name} {target.score:.2f}',
                        (x1, max(18, y1 - 9)), cv2.FONT_HERSHEY_SIMPLEX,
                        0.55, EYE, 2, cv2.LINE_AA)
            if abs(lat) > 1e-3:
                cv2.rectangle(vis, (cxp, H * 2 - 26),
                              (cxp + int(lat * W * 0.9), H * 2 - 14), HOT, -1)
        cv2.putText(vis, f'lat {lat:+.3f}', (14, H * 2 - 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    HOT if abs(lat) > 1e-3 else DIM, 1, cv2.LINE_AA)
        enc, buf = cv2.imencode('.jpg', vis, [cv2.IMWRITE_JPEG_QUALITY, 72])
        if enc:
            ST['jpg'] = buf.tobytes()
        time.sleep(0.028)


CT = {'.html': 'text/html', '.css': 'text/css', '.js': 'application/javascript'}


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
        p = self.path.split('?')[0]
        if p == '/arm':
            ST['want'] = True
            return self._send(b'ok')
        if p == '/disarm':
            ST['want'] = False
            return self._send(b'ok')
        if p == '/j':
            payload = {k: ST[k] for k in
                       ('vision', 'board', 'host', 'link', 'loop_hz',
                        'fw_rev', 'mode', 'fr')}
            return self._send(json.dumps(payload, default=str).encode(),
                              'application/json')
        if p in ('/console.css', '/console.js'):
            f = HERE / 'console' / p.lstrip('/')
            return self._send(f.read_bytes(), CT[f.suffix])
        if p != '/stream':
            return self._send((HERE / 'console' / 'console.html').read_bytes(),
                              'text/html')
        # ThreadingHTTPServer required: this handler never returns, so a
        # single-threaded server could not answer /j or /disarm while streaming.
        self.send_response(200)
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=f')
        self.end_headers()
        try:
            while not STOP.is_set():
                j = ST['jpg']
                if j:
                    self.wfile.write(b'--f\r\nContent-Type: image/jpeg\r\n'
                                     b'Content-Length: ' + str(len(j)).encode()
                                     + b'\r\n\r\n' + j + b'\r\n')
                time.sleep(0.02)
        except Exception:                                       # noqa: BLE001
            pass


threading.Thread(target=control, daemon=True).start()
threading.Thread(target=render, daemon=True).start()
print(f'  console  http://10.42.0.28:{A.port}   class={A.klass} gain={A.gain}%',
      flush=True)
try:
    ThreadingHTTPServer(('0.0.0.0', A.port), Handler).serve_forever()
except KeyboardInterrupt:
    STOP.set()
    time.sleep(0.8)
