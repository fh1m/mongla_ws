#!/usr/bin/env python3
"""Blocking vs async on the SAME frames: the boxes must be identical.

Switching the detector from `InferVStreams` to `InferModel`/`run_async` was
done for one reason -- the blocking call holds the GIL for its whole ~10 ms
and freezes every other Python thread in the process. That is a transport
change and it must not move a single pixel.

It very nearly did. The two APIs return DIFFERENT SHAPES: the blocking one a
dict of ragged per-class arrays, the async one a FLAT float32 buffer
(measured: (1503,) for a 3-class model = 3 * (1 + 100 * 5), fixed per-class
stride with a leading count). Misreading that layout does not raise -- it
reads scores as coordinates and hands the control loop a box that tracks
nothing, which on a vision-servoed hull means driving at the wrong thing.

So this is not a smoke test, it is the equivalence proof. Run:

    python3 tools/hailo_api_equivalence.py --mode blocking --out /tmp/b.json
    python3 tools/hailo_api_equivalence.py --mode async    --out /tmp/a.json
    python3 tools/hailo_api_equivalence.py --compare /tmp/b.json /tmp/a.json

Two processes because one VDevice cannot serve both APIs -- the second raises
HAILO_STREAM_NOT_ACTIVATED (72).
"""
import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..',
                                'src', 'duburi_vision'))

_SEED = 20260904
_N = 24


def _frames():
    """Deterministic pseudo-images. Structured, not pure noise -- a detector
    finds nothing in uniform static, and 'both APIs agreed on zero boxes' is
    not evidence of anything."""
    rng = np.random.default_rng(_SEED)
    out = []
    for i in range(_N):
        f = rng.integers(0, 60, (360, 640, 3), dtype=np.uint8)
        # a few bright rectangles, moving, so there is something to detect
        for k in range(4):
            x = (60 * k + 13 * i) % 560
            y = (40 * k + 7 * i) % 300
            f[y:y + 55, x:x + 70] = rng.integers(150, 255, 3, dtype=np.uint8)
        out.append(f)
    return out


def _run(mode, path, model, conf):
    if mode == 'blocking':
        os.environ['DUBURI_HAILO_FORCE_BLOCKING'] = '1'
    from duburi_vision.detection.factory import make_detector
    det = make_detector(model_path=model, conf=conf, max_det=100,
                        class_allowlist=None, device='cpu', half=False)
    rows = []
    for f in _frames():
        rows.append([[d.class_id, round(d.score, 4)]
                     + [round(v, 2) for v in d.xyxy]
                     for d in det.infer(f)])
    with open(path, 'w') as fh:
        json.dump(rows, fh)
    total = sum(len(r) for r in rows)
    print(f'  {mode}: {total} detections over {_N} frames -> {path}')
    if total == 0:
        print('  WARNING: zero detections. Agreement would be vacuous; '
              'lower --conf or use real footage.')


def _compare(a_path, b_path):
    a = json.load(open(a_path))
    b = json.load(open(b_path))
    if len(a) != len(b):
        print(f'  FAIL: {len(a)} frames vs {len(b)}')
        return 1
    bad = 0
    for i, (x, y) in enumerate(zip(a, b)):
        if x != y:
            bad += 1
            if bad <= 3:
                print(f'  frame {i} differs:\n    {x}\n    {y}')
    total = sum(len(r) for r in a)
    if bad:
        print(f'  FAIL: {bad}/{len(a)} frames differ ({total} detections)')
        return 1
    print(f'  PASS: identical on all {len(a)} frames, {total} detections')
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--mode', choices=('blocking', 'async'))
    ap.add_argument('--out')
    ap.add_argument('--model', default='gate_rescue_repair')
    ap.add_argument('--conf', type=float, default=0.05)
    ap.add_argument('--compare', nargs=2)
    a = ap.parse_args()
    if a.compare:
        sys.exit(_compare(*a.compare))
    _run(a.mode, a.out, a.model, a.conf)


if __name__ == '__main__':
    main()
