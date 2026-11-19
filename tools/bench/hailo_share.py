#!/usr/bin/env python3
"""What XFeat costs the detectors when the three share one Hailo-8.

The shipped classes (`HailoDetector`, `XFeatHailo`) on the process's one
VDevice under ROUND_ROBIN, each in its own thread, each paced at its own rate
the way the nodes run them: forward detector, downward detector, XFeat. Per
network it reports the achieved rate and the call latency (submit to result,
host post-process included), p50 and p90, in each condition:

    detectors alone            the baseline
    + XFeat at --xfeat-hz      the anchor's cycle
    + XFeat flat out           the worst case a busy anchor can make
    flat out, detectors first  the same, detectors at scheduler priority 31

Bar, written before the first run: sharing is acceptable when, with XFeat at
the anchor's rate, both detectors keep their rate (within 2 %) and neither
detector's p90 rises by more than 5 ms. Priority is worth setting when, under
XFeat flat out, it brings the detectors' p90 back within 5 ms of the baseline.

One real downward frame feeds every network (`--frame`, or the downward camera);
detection content only changes host decode, which is timed but not the subject.
"""
import argparse
import os
import sys
import threading
import time

import numpy as np

sys.path.insert(0, os.path.expanduser('~/mongla_ws/src/mongla_vision'))
import cv2  # noqa: E402

from mongla_vision.anchor.xfeat_hailo import XFeatHailo  # noqa: E402
from mongla_vision.detection.hailo import HailoDetector  # noqa: E402


def frame(path):
    if path:
        return cv2.imread(path)
    cap = cv2.VideoCapture('/dev/mongla_cam_downward')
    for _ in range(10):
        ok, f = cap.read()
    cap.release()
    if not ok:
        raise SystemExit('no frame: pass --frame')
    return f


def paced(name, fn, hz, stop, out):
    """Call `fn` at `hz` (0 = back to back) until `stop`; latencies in ms."""
    lat, period, nxt = [], (1.0 / hz if hz else 0.0), time.perf_counter()
    while not stop.is_set():
        t = time.perf_counter()
        fn()
        lat.append(1000 * (time.perf_counter() - t))
        if period:
            nxt += period
            time.sleep(max(0.0, nxt - time.perf_counter()))
    out[name] = lat


def run(nets, seconds):
    stop, out = threading.Event(), {}
    th = [threading.Thread(target=paced, args=(n, f, hz, stop, out)) for n, f, hz in nets]
    for t in th:
        t.start()
    time.sleep(seconds)
    stop.set()
    for t in th:
        t.join()
    return {n: (len(v) / seconds, np.percentile(v, 50), np.percentile(v, 90)) for n, v in out.items()}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--fwd', required=True, help='forward detector .hef')
    ap.add_argument('--dwn', required=True, help='downward detector .hef')
    ap.add_argument('--xfeat', default=os.path.expanduser('~/hailo_models/xfeat_320x240.hef'))
    ap.add_argument('--fwd-hz', type=float, default=30.0)
    ap.add_argument('--dwn-hz', type=float, default=60.0)
    ap.add_argument('--xfeat-hz', type=float, default=3.0)
    ap.add_argument('--seconds', type=float, default=20.0)
    ap.add_argument('--frame', default='')
    a = ap.parse_args()

    f = frame(a.frame)
    g = cv2.resize(cv2.cvtColor(f, cv2.COLOR_BGR2GRAY), (320, 240))
    fwd, dwn = HailoDetector(model_path=a.fwd), HailoDetector(model_path=a.dwn)
    xf = XFeatHailo(a.xfeat, top_k=1024)
    for _ in range(5):
        fwd.infer(f), dwn.infer(f), xf.detect(g)
    det = [('forward', lambda: fwd.infer(f), a.fwd_hz), ('downward', lambda: dwn.infer(f), a.dwn_hz)]

    rows = [('detectors alone', run(det, a.seconds)),
            (f'+ XFeat at {a.xfeat_hz:g} Hz', run(det + [('xfeat', lambda: xf.detect(g), a.xfeat_hz)], a.seconds)),
            ('+ XFeat flat out', run(det + [('xfeat', lambda: xf.detect(g), 0)], a.seconds))]
    for d in (fwd, dwn):
        d._cim.set_scheduler_priority(31)
    rows.append(('flat out, detectors first', run(det + [('xfeat', lambda: xf.detect(g), 0)], a.seconds)))

    print(f'{"condition":28s} {"network":9s} {"rate Hz":>8s} {"p50 ms":>7s} {"p90 ms":>7s}')
    for cond, res in rows:
        for n in ('forward', 'downward', 'xfeat'):
            if n in res:
                hz, p50, p90 = res[n]
                print(f'{cond:28s} {n:9s} {hz:8.1f} {p50:7.2f} {p90:7.2f}')
    base = rows[0][1]

    def verdict(res):
        return all(res[n][0] >= 0.98 * base[n][0] and res[n][2] - base[n][2] <= 5.0
                   for n in ('forward', 'downward'))
    print(f'VERDICT sharing at the anchor rate: {"PASS" if verdict(rows[1][1]) else "FAIL"}; '
          f'flat out: {"PASS" if verdict(rows[2][1]) else "FAIL"}; '
          f'flat out with detector priority: {"PASS" if verdict(rows[3][1]) else "FAIL"}')
    for o in (fwd, dwn, xf):
        o.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
