#!/usr/bin/env python3
"""Where does a Hailo detector call go? Per stage, one or two detectors at once.

`hailortcli benchmark` feeds synthetic data with no host pipeline and reported
~1036 FPS on a model whose real pipeline ran 40-45 FPS, so the frame time has
to be split on the path the node runs: `HailoDetector.infer`, InferModel
async. The stages, in the order that method runs them:

    lock       waiting for the detector's own lock (zero with one caller)
    letterbox  resize + pad into the bound input buffer
    ready      wait_for_async_ready (a slot on the chip's queue)
    chip       run_async + job.wait: DMA in, the chip, the baked NMS, DMA out,
               and any time the scheduler gives the chip to the other network
    nms        decode of the flat NMS buffer
    boxes      un-letterbox, clamp, allowlist

`--pair DWN_MODEL` runs a second detector in its own thread on the same
process VDevice, each paced at its own rate, which is how the composed vision
process runs the two cameras (measured-bars 95: the downward detector held
44.9 Hz of the 60 asked for, its call 20-33 ms against ~10 ms of chip).

Bar, written before the first two-detector run (bead po7.15.6): the stage
whose median grows by at least half of the downward call's growth from alone
to paired owns the ceiling. If none does, the report says "no single owner"
and nothing is changed on its strength. Medians and p90 over a sustained
window; one real frame feeds every call (the grab is not the subject).
"""
import argparse
import os
import statistics as st
import sys
import threading
import time

import numpy as np

sys.path.insert(0, os.path.expanduser('~/mongla_ws/src/mongla_vision'))
import cv2  # noqa: E402

from mongla_vision.detection.hailo import HailoDetector  # noqa: E402
from mongla_vision.detection import hailo as H  # noqa: E402

STAGES = ('lock', 'letterbox', 'ready', 'chip', 'nms', 'boxes')


def timed_infer(det, frame, out):
    """`HailoDetector.infer`'s async path, step for step, with a clock between
    steps. Kept beside the method it mirrors: if `infer` changes, this must."""
    h, w = frame.shape[:2]
    t = [time.perf_counter()]
    with det._infer_lock:
        t.append(time.perf_counter())
        cim = det._configured()
        scale, px, py = det._letterbox_into_bound(frame)
        t.append(time.perf_counter())
        cim.wait_for_async_ready(timeout_ms=H._ASYNC_READY_MS)
        t.append(time.perf_counter())
        job = cim.run_async([det._bindings])
        job.wait(H._ASYNC_WAIT_MS)
        t.append(time.perf_counter())
        per_class = det._decode_nms(det._out_buf)
        t.append(time.perf_counter())
    det._boxes_to_detections(per_class, w, h, scale, px, py)
    t.append(time.perf_counter())
    for name, a, b in zip(STAGES, t, t[1:]):
        out[name].append(1000.0 * (b - a))
    out['total'].append(1000.0 * (t[-1] - t[0]))


def paced(det, frame, hz, stop, out):
    period, nxt = (1.0 / hz if hz else 0.0), time.perf_counter()
    while not stop.is_set():
        timed_infer(det, frame, out)
        if period:
            nxt += period
            time.sleep(max(0.0, nxt - time.perf_counter()))


def run(dets, seconds):
    stop = threading.Event()
    outs = {n: {s: [] for s in STAGES + ('total',)} for n, *_ in dets}
    th = [threading.Thread(target=paced, args=(d, f, hz, stop, outs[n])) for n, d, f, hz in dets]
    for x in th:
        x.start()
    time.sleep(seconds)
    stop.set()
    for x in th:
        x.join()
    return outs


def p(xs, q):
    xs = sorted(xs)
    return xs[int(q * (len(xs) - 1))]


def report(title, outs, seconds):
    print(f'\n  {title}')
    print(f'  {"network":10s} {"stage":10s} {"p50 ms":>7s} {"p90 ms":>7s}')
    for n, o in outs.items():
        for s in STAGES + ('total',):
            print(f'  {n:10s} {s:10s} {st.median(o[s]):7.2f} {p(o[s], 0.9):7.2f}')
        print(f'  {n:10s} rate       {len(o["total"]) / seconds:7.1f} Hz')


def frame_from(path, device):
    if path:
        f = cv2.imread(path)
        if f is None:
            raise SystemExit(f'cannot read {path}')
        return f
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)
    ok, f = False, None
    for _ in range(20):
        ok, f = cap.read()
    cap.release()
    if not ok:
        raise SystemExit('no frame: pass --frame')
    return f


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--model', default='gate_sharks', help='stem or .hef (the forward detector)')
    ap.add_argument('--pair', default='', help='a second model, run beside it (the downward detector)')
    ap.add_argument('--hz', type=float, default=30.0, help='first detector pace (0 = back to back)')
    ap.add_argument('--pair-hz', type=float, default=60.0)
    ap.add_argument('--seconds', type=float, default=15.0)
    ap.add_argument('--frame', default='', help='an image; default one frame of --device')
    ap.add_argument('--device', default='/dev/mongla_cam_downward')
    a = ap.parse_args()

    def hef(m):
        return m if m.endswith('.hef') else os.path.expanduser(
            f'~/mongla_ws/src/mongla_vision/models/{m}.hef')
    f = frame_from(a.frame, a.device)
    first = HailoDetector(model_path=hef(a.model), conf=0.15)
    dets = [('first', first, f, a.hz)]
    if a.pair:
        second = HailoDetector(model_path=hef(a.pair), conf=0.15)
        dets.append(('second', second, f, a.pair_hz))
    for _, d, fr, _ in dets:
        for _ in range(5):
            d.infer(fr)
    alone = {}
    for n, d, fr, hz in dets:
        alone.update(run([(n, d, fr, hz)], a.seconds))
    report('each alone', alone, a.seconds)
    if a.pair:
        both = run(dets, a.seconds)
        report('both at once', both, a.seconds)
        growth = {s: st.median(both['second'][s]) - st.median(alone['second'][s]) for s in STAGES}
        total = st.median(both['second']['total']) - st.median(alone['second']['total'])
        owner = [s for s, g in growth.items() if total > 0 and g >= 0.5 * total]
        print(f'\n  second detector, alone -> paired: total {total:+.2f} ms; '
              + ', '.join(f'{s} {g:+.2f}' for s, g in growth.items()))
        print(f'  VERDICT the ceiling is owned by: {owner[0] if owner else "no single owner"}')
    for _, d, _, _ in dets:
        d.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
