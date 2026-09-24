#!/usr/bin/env python3
"""Re-derive the identity bar for the references PRODUCTION actually builds.

⛔ WHY THIS EXISTS. §22 swept the identity bar to 60 inliers using WHOLE-FRAME
references -- 1024 keypoints spread over water, pool edge and prop. But
`lock_node` enrols with `roi=det_box`, so a shipped reference holds only the
keypoints inside the detection box, which is a fraction of that. A bar derived
from one configuration and applied to another is the recurring defect in this
codebase, and it fails in the worst direction: 60 may be UNREACHABLE for a true
match, so re-acquisition would never fire and nothing would log a fault.

PRODUCTION SHAPE, EXACTLY. References are cropped to a real detector box;
queries are FULL FRAMES, because `locate()` matches the live frame whole. Boxes
come from our own YOLO graph via onnxruntime rather than from hand-drawn
rectangles, so the crops are the ones the vehicle would really get.

Reports the same three populations as §22 -- same run, same prop other run,
other prop same venue -- and sweeps the bar across them.
"""
from __future__ import annotations

import argparse
import importlib.util
import os

import cv2
import numpy as np
import onnxruntime as ort

HERE = os.path.dirname(os.path.abspath(__file__))
import sys as _sys, os as _os
_sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))
from archive_root import archive_root
A = archive_root()
M = f'{A}/Mirpur/Sun_June_21'


def backend(model, threads):
    p = os.path.join(HERE, '..', 'src', 'mongla_vision', 'mongla_vision',
                     'anchor', 'xfeat_onnx.py')
    spec = importlib.util.spec_from_file_location('xfeat_onnx', p)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def letterbox(img, size=640):
    h, w = img.shape[:2]
    r = min(size / h, size / w)
    nh, nw = int(round(h * r)), int(round(w * r))
    out = np.full((size, size, 3), 114, np.uint8)
    out[:nh, :nw] = cv2.resize(img, (nw, nh))
    return out, r


def best_box(sess, iname, img, conf_th):
    lb, r = letterbox(img)
    x = lb[:, :, ::-1].transpose(2, 0, 1)[None].astype(np.float32) / 255.0
    y = sess.run(None, {iname: x})[0][0]
    conf = y[4:].max(axis=0)
    i = int(conf.argmax())
    if conf[i] < conf_th:
        return None, 0.0
    cx, cy, bw, bh = y[:4, i] / r
    return (cx - bw / 2, cy - bh / 2, cx + bw / 2, cy + bh / 2), float(conf[i])


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--xfeat', required=True)
    ap.add_argument('--detector', required=True)
    ap.add_argument('--threads', type=int, default=2)
    ap.add_argument('--refs', type=int, default=10)
    ap.add_argument('--queries', type=int, default=14)
    ap.add_argument('--conf', type=float, default=0.25)
    a = ap.parse_args()

    X = backend(a.xfeat, a.threads)
    import sys
    sys.path.insert(0, os.path.abspath(os.path.join(HERE, '..', 'src',
                                                    'mongla_vision')))
    sys.modules.setdefault('xfeat_onnx', X)
    from mongla_vision.anchor.bank import CheckpointBank
    net = X.XFeatONNX(a.xfeat, top_k=1024, threads=a.threads)
    sess = ort.InferenceSession(a.detector, providers=['CPUExecutionProvider'])
    iname = sess.get_inputs()[0].name

    def frames(path, n):
        cap = cv2.VideoCapture(path)
        N = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
        out = []
        for j in range(n):
            cap.set(cv2.CAP_PROP_POS_FRAMES,
                    int(N * (0.12 + 0.74 * j / max(1, n - 1))))
            ok, f = cap.read()
            if ok:
                out.append(f)
        cap.release()
        return out

    bank = CheckpointBank(net, capacity=max(a.refs, 8))
    kp = []
    for f in frames(f'{M}/torpedo.mkv', a.refs):
        box, c = best_box(sess, iname, f, a.conf)
        if box is None:
            continue
        g = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
        r = bank.enrol(g, roi=box, det_conf=c, label='torpedo', force=True)
        if r.accepted:
            kp.append(r.keypoints)
    if not bank.size:
        raise SystemExit('no detections to crop -- lower --conf')
    print(f'references: {bank.size} ROI-cropped, keypoints '
          f'min/med/max {min(kp)}/{int(np.median(kp))}/{max(kp)} '
          f'(whole-frame was 1024)')

    pops = {}
    for name, path in (('same run', f'{M}/torpedo.mkv'),
                       ('same prop, other run', f'{M}/torpedo_1.mkv'),
                       ('OTHER prop', f'{M}/gate.mkv')):
        v = [bank.locate(cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)).inliers
             for f in frames(path, a.queries)]
        pops[name] = np.array(v, float)
        w = pops[name]
        print(f'  {name:<22} p50 {np.percentile(w,50):>5.0f}  '
              f'p90 {np.percentile(w,90):>5.0f}  max {w.max():>5.0f}')

    print('\nsweeping the bar (true = same prop other run, false = other prop):')
    pos, neg = pops['same prop, other run'], pops['OTHER prop']
    for t in (10, 15, 20, 25, 30, 40, 60, 80):
        print(f'  bar {t:>3}: keeps {100*(pos>=t).mean():>3.0f}% of true, '
              f'admits {100*(neg>=t).mean():>3.0f}% of false')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
