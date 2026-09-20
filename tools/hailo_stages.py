#!/usr/bin/env python3
"""Where does a Hailo frame actually go? Per-stage, on real frames.

The published numbers cannot answer this. `hailortcli benchmark` feeds
SYNTHETIC data with no host pipeline and reported ~1036 FPS on a model whose
real pipeline ran 40-45 FPS at 15-20% chip utilisation -- a 25x gap that was
entirely host preprocessing, host NMS and PCIe transfer.

So this splits the frame time into the four things that can own it:

    grab       camera read (MJPEG decode inside OpenCV)
    letterbox  host resize + pad into the 640x640 canvas  (a FRESH allocation)
    infer      DMA in + chip + baked NMS + DMA out        (blocking call)
    decode     per-class Python loop, un-letterbox, clamp

Reported as MEDIANS over a sustained window. A mean folds in the startup
transient and a peak is not a rate anything can rely on -- this project has
already published one headline (82.3 Hz) that was a single run.
"""
import argparse, os, sys, time, statistics as st
import numpy as np

sys.path.insert(0, os.path.expanduser('~/mongla_ws/src/mongla_vision'))
import cv2
cv2.setNumThreads(1)          # the pipeline is serial; extra threads only add jitter

from mongla_vision.detection.factory import make_detector
from mongla_vision.detection import hailo as H

ap = argparse.ArgumentParser()
ap.add_argument('--model', default='gate_rescue_repair')
ap.add_argument('--conf', type=float, default=0.15)
ap.add_argument('--seconds', type=float, default=12.0)
ap.add_argument('--width', type=int, default=640)
ap.add_argument('--height', type=int, default=360)
ap.add_argument('--fps', type=int, default=210)
ap.add_argument('--device', default='0')
ap.add_argument('--synthetic', action='store_true',
                help='no camera: isolates the chip+host cost from the grab')
A = ap.parse_args()

path = A.model if A.model.endswith('.hef') else os.path.expanduser(
    f'~/mongla_ws/src/mongla_vision/models/{A.model}.hef')
det = make_detector(model_path=path, conf=A.conf, class_allowlist=None,
                    max_det=100, logger=None)
size = det._size

cap = None
if not A.synthetic:
    dev = int(A.device) if A.device.isdigit() else A.device
    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  A.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, A.height)
    cap.set(cv2.CAP_PROP_FPS,          A.fps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)
    for _ in range(20): cap.read()
    print(f'  camera {cap.get(cv2.CAP_PROP_FRAME_WIDTH):.0f}x'
          f'{cap.get(cv2.CAP_PROP_FRAME_HEIGHT):.0f} @ '
          f'{cap.get(cv2.CAP_PROP_FPS):.0f} requested')
else:
    print('  synthetic frames (no camera)')

frame0 = np.random.randint(0, 255, (A.height, A.width, 3), np.uint8)

g, l, i, d, tot, ndet = [], [], [], [], [], []
end = time.perf_counter() + A.seconds
while time.perf_counter() < end:
    t0 = time.perf_counter()
    if cap is not None:
        ok, frame = cap.read()
        if not ok: continue
    else:
        frame = frame0
    t1 = time.perf_counter()

    buf, scale, px, py = H.letterbox(frame, size)
    t2 = time.perf_counter()

    res = det._pipe.infer({det._in_name: np.expand_dims(buf, 0)})
    t3 = time.perf_counter()

    raw = res[det._out_name]
    per_class = raw[0] if len(raw) else []
    n = 0
    h, w = frame.shape[:2]
    for cid, boxes in enumerate(per_class):
        if boxes is None or len(boxes) == 0: continue
        for b in boxes:
            if float(b[4]) < A.conf: continue
            y1, x1, y2, x2 = float(b[0]), float(b[1]), float(b[2]), float(b[3])
            x1 = (x1 * size - px) / scale; x2 = (x2 * size - px) / scale
            y1 = (y1 * size - py) / scale; y2 = (y2 * size - py) / scale
            n += 1
    t4 = time.perf_counter()

    g.append((t1-t0)*1e3); l.append((t2-t1)*1e3)
    i.append((t3-t2)*1e3); d.append((t4-t3)*1e3)
    tot.append((t4-t0)*1e3); ndet.append(n)

def row(name, xs):
    xs = sorted(xs)
    p95 = xs[int(0.95*(len(xs)-1))]
    print(f'  {name:<12} {st.median(xs):7.2f} {p95:8.2f} '
          f'{100*st.median(xs)/st.median(tot):7.1f} %')

print(f'\n  {A.model}  conf={A.conf}  n={len(tot)} frames\n')
print(f'  {"stage":<12} {"median":>7} {"p95":>8} {"share":>9}')
print('  ' + '-'*40)
row('grab', g); row('letterbox', l); row('infer', i); row('decode', d)
print('  ' + '-'*40)
row('TOTAL', tot)
print(f'\n  achieved       {1000.0/st.median(tot):6.1f} Hz')
if cap is not None:
    print(f'  without grab   {1000.0/(st.median(tot)-st.median(g)):6.1f} Hz'
          f'   <- ceiling if the camera were free')
print(f'  detections/frame  median {st.median(ndet):.1f}  max {max(ndet)}')
if cap: cap.release()
det.close()
