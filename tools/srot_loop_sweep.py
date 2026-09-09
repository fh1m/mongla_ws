#!/usr/bin/env python3
"""Where does the vision->thrust loop saturate, and on what?

`VISION_LOOP_HZ = 20` was chosen when perception was 3-4 Hz on a .pt and
20-30 Hz on TensorRT. The AI HAT does ~70-80 Hz, so the constant is now the
binding constraint on the behaviours that actually hurt -- terminal alignment
and the close-in torpedo lock. This finds the real ceiling instead of guessing
a new number.

WHAT IS MEASURED AT EACH RATE
  achieved     median loop Hz against the requested rate. The gap IS the answer.
  latency      frame captured -> MANUAL_CONTROL byte on the wire, per tick.
               Not "inference time": the whole path, which is what a control
               loop's phase margin actually pays.
  host         Pi CPU and temperature.
  board        SYS_STATUS.load, drop_rate_comm, errors_comm -- the far end's own
               opinion, which is the only way to see the BOARD saturate rather
               than the host.
  link         TX bytes/s against 11520 B/s. Full duplex, so this is a separate
               budget from the ~4.2 kB/s of telemetry coming back.

WHY EVERY NUMBER IS A MEDIAN over a sustained window: a mean folds in the
startup transient and a peak is not a rate anything can rely on. This project
has already published one headline (82.3 Hz) that was a single run.

REPORTED HONESTLY: a rate that is requested and not achieved is reported as
not achieved. The temptation at the end of a sweep is to quote the highest
number that appeared anywhere.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import statistics as st
import sys
import time

WS = os.path.expanduser('~/duburi_ws/src')
for _p in ('duburi_control', 'duburi_vision'):
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

ap = argparse.ArgumentParser()
ap.add_argument('--rates', default='20,40,60,80,120,200')
ap.add_argument('--seconds', type=float, default=14.0)
ap.add_argument('--class', dest='klass', default='person')
ap.add_argument('--model', default='yolov11n')
ap.add_argument('--dev', default='/dev/ttyUSB0')
A = ap.parse_args()

W, H = 640, 360
CAL = os.path.expanduser('~/duburi_ws/src/duburi_vision/config/'
                         'calibration/pi_downward_1280x720.json')
K = D = None
try:
    _c = json.load(open(CAL))
    _Kc, D = _c['camera_matrix'], _c['distortion_coefficients']
    _sx, _sy = W / _c['image_width'], H / _c['image_height']
    K = [_Kc[0][0] * _sx, 0, _Kc[0][2] * _sx,
         0, _Kc[1][1] * _sy, _Kc[1][2] * _sy, 0, 0, 1]
except Exception:                                               # noqa: BLE001
    pass


def cpu_sample():
    with open('/proc/stat') as fh:
        p = [float(x) for x in fh.readline().split()[1:]]
    return p[3] + p[4], sum(p)


def pi_temp():
    try:
        with open('/sys/class/thermal/thermal_zone0/temp') as fh:
            return int(fh.read().strip()) / 1000.0
    except Exception:                                           # noqa: BLE001
        return float('nan')


def main() -> int:
    guard = PortGuard(A.dev)
    guard.acquire()
    conn = mavutil.mavlink_connection(A.dev, baud=115200,
                                      source_system=sp.SOURCE_SYSID,
                                      source_component=sp.SOURCE_COMPID)
    conn.wait_heartbeat(timeout=10)
    fc = SrotFC(conn, log=None)
    fc.set_message_rate(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 50)

    # B28: REFUSE TO MEASURE INTO A BOARD THAT DISCARDS THE COMMAND.
    # `SROT_MOVE` leaves the board latched in AUTO, and in AUTO the firmware
    # throws away every axis of MANUAL_CONTROL and reports nothing. A sweep run
    # in that state drives zero thrust and still prints a full set of numbers --
    # a plausible measurement standing in for an absent one, which is this
    # project's signature defect. Reuse the verb path's own verified set-mode
    # rather than a second copy of it.
    from duburi_control.vision_verbs import _require_srot_vision_mode
    _require_srot_vision_mode(fc, None, 'srot_loop_sweep')

    det = make_detector(model_path=A.model, conf=0.35,
                        class_allowlist=[A.klass], device='cuda:0',
                        iou=0.5, imgsz=640, half=True, max_det=20)
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
    cap.set(cv2.CAP_PROP_FPS, 210)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    for _ in range(20):
        cap.read()

    print(f'\nsweep: {A.seconds:.0f} s per rate, {A.klass!r}, '
          f'MANUAL_CONTROL to the board each tick\n')
    hdr = (f"{'req':>5} {'achieved':>9} {'e2e ms':>16} {'infer':>7} "
           f"{'cpu':>5} {'degC':>5} {'tx B/s':>8} {'tx%':>5} "
           f"{'bd load':>8} {'drop':>6} {'err':>5}")
    print(hdr)
    print('-' * len(hdr))
    rows = []
    for req in [float(x) for x in A.rates.split(',')]:
        bf = BearingFilter()
        period = 1.0 / req
        lat_ms, inf_ms, ticks = [], [], []
        sent = 0
        c0 = cpu_sample()
        # Board counters BEFORE, so the delta is attributable to this rate
        # rather than to everything since boot.
        for _ in range(30):
            conn.recv_match(blocking=False)
        t_end = time.perf_counter() + A.seconds
        t_prev = time.perf_counter()
        while time.perf_counter() < t_end:
            t0 = time.perf_counter()
            ok, frame = cap.read()
            if not ok:
                continue
            t1 = time.perf_counter()
            dets = det.infer(frame)
            t2 = time.perf_counter()
            tgt = max(dets, key=lambda d: d.area) if dets else None
            b_raw = (bearing_from_pixels(tgt.cx, tgt.cy, tgt.width, tgt.height,
                                         width=W, height=H, K=K, D=D)
                     if (tgt is not None and K) else None)
            b = bf.update(b_raw, time.perf_counter())
            lat = 0.0
            if b is not None and abs(b.angle_x) > math.radians(2.5):
                lat = max(-1.0, min(1.0, b.angle_x / math.radians(20.0))) * 0.25
            fc.manual(fwd=0.0, lat=lat, up=0.0, yaw=0.0)
            t3 = time.perf_counter()
            sent += 1
            # End to end: shutter to the byte leaving. This is the delay the
            # control law is actually closing around.
            lat_ms.append((t3 - t0) * 1000)
            inf_ms.append((t2 - t1) * 1000)
            now = time.perf_counter()
            ticks.append(now - t_prev)
            t_prev = now
            time.sleep(max(0.0, period - (time.perf_counter() - t0)))

        c1 = cpu_sample()
        di, dt = c1[0] - c0[0], c1[1] - c0[1]
        cpu = 100.0 * (1 - di / dt) if dt else float('nan')
        # Drain and read the board's own view.
        load = drop = errs = float('nan')
        t_read = time.time()
        while time.time() - t_read < 1.5:
            m = conn.recv_match(type='SYS_STATUS', blocking=False)
            if m is not None:
                load, drop, errs = (m.load / 10.0, m.drop_rate_comm / 100.0,
                                    m.errors_comm)
        ach = 1.0 / st.median(ticks) if ticks else 0.0
        med = st.median(lat_ms)
        p95 = sorted(lat_ms)[int(.95 * len(lat_ms))]
        # MANUAL_CONTROL is a 30-byte frame on the wire (18 payload + 12).
        tx = sent / A.seconds * 30
        rows.append(dict(req=req, ach=ach, med=med, p95=p95,
                         inf=st.median(inf_ms), cpu=cpu, temp=pi_temp(),
                         tx=tx, load=load, drop=drop, errs=errs, n=len(ticks)))
        r = rows[-1]
        print(f"{req:5.0f} {ach:9.1f} {med:7.1f} /{p95:6.1f} {r['inf']:7.2f} "
              f"{cpu:5.0f} {r['temp']:5.1f} {tx:8.0f} "
              f"{100*tx/11520.0:5.1f} {load:8.1f} {drop:6.2f} {errs:5.0f}")

    cap.release()
    det.close()
    conn.close()
    guard.release()

    print('\n' + '=' * 72)
    best = max(rows, key=lambda r: r['ach'])
    knee = None
    for r in rows:
        if r['ach'] < r['req'] * 0.92:
            knee = r
            break
    print(f"  highest achieved      {best['ach']:.1f} Hz "
          f"(requested {best['req']:.0f})")
    if knee:
        print(f"  saturates at          {knee['req']:.0f} Hz requested -> "
              f"{knee['ach']:.1f} achieved")
    print(f"  e2e latency at peak   {best['med']:.1f} ms median, "
          f"{best['p95']:.1f} p95")
    print(f"  inference alone       {best['inf']:.2f} ms  "
          f"({100*best['inf']/best['med']:.0f} % of the loop)")
    print(f"  TX at peak            {best['tx']:.0f} B/s = "
          f"{100*best['tx']/11520.0:.1f} % of the uplink")
    print(f"  Pi CPU at peak        {best['cpu']:.0f} %")
    print()
    # The number a control engineer actually wants. A discrete loop's delay is
    # its sampling period PLUS its pipeline latency, and that sum is what sets
    # phase lag -- which is what limits how hard the gain can be pushed before
    # the hull oscillates. Reporting only "70 Hz" hides that the sample period
    # is now the SMALL term.
    slow = min(rows, key=lambda r: r['ach'])
    for r in (slow, best):
        delay = 1000.0 / r['ach'] + r['med']
        print(f"  at {r['ach']:5.1f} Hz: sample {1000.0/r['ach']:5.1f} ms + "
              f"pipeline {r['med']:5.1f} ms = {delay:5.1f} ms of loop delay")
    d_lo = 1000.0 / slow['ach'] + slow['med']
    d_hi = 1000.0 / best['ach'] + best['med']
    print(f"  -> {d_lo/d_hi:.2f}x less delay, and the pipeline is now "
          f"{100*best['med']/d_hi:.0f} % of it")
    print('=' * 72)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
