#!/usr/bin/env python3
"""Build a checkpoint bank BEFORE the run, from practice footage or stills.

⭐ WHY. On the run there may be no confident detection to enrol a checkpoint
from at the moment the anchor is needed -- which is exactly when the detector
is failing, i.e. the case the anchor rung exists for. A bank prepared in advance
does not have that dependency.

Measured 2026-09-24 (`measured-bars.md` §21.3): references snapped on one run
clear the trust bar on 92 % and 100 % of frames of a DIFFERENT run of the same
prop. ⛔ And on a generic structural view, 8 %. So this is worth doing for a
DISTINCTIVE RIGID PROP and is not worth doing for "the pool".

    python3 tools/build_practice_bank.py --model ~/hailo_models/xfeat_320x240.onnx \
        --label gate --out ~/missions/banks/gate.npz \
        --video practice/gate_run.mkv --stride 2.0

    python3 tools/build_practice_bank.py --model ... --label torpedo \
        --out torpedo.npz --images practice/torpedo/*.png

⛔ THE BANK IS SAVED AT THE BACKEND'S RESOLUTION and refuses to load into a
different one, because keypoints are stored in backend pixels. Build it with
the model the vehicle runs.
"""
from __future__ import annotations

import argparse
import glob
import importlib.util
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, '..', 'src', 'mongla_vision')


def load_modules(model, threads):
    sys.path.insert(0, os.path.abspath(SRC))
    def mod(name, rel):
        spec = importlib.util.spec_from_file_location(
            name, os.path.join(SRC, 'mongla_vision', 'anchor', rel))
        m = importlib.util.module_from_spec(spec)
        sys.modules[name] = m
        spec.loader.exec_module(m)
        return m
    x = mod('xfeat_onnx', 'xfeat_onnx.py')
    from mongla_vision.anchor.bank import CheckpointBank
    return CheckpointBank, x.XFeatONNX(model, top_k=1024, threads=threads)


def frames_from_video(path, stride_s, limit):
    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        raise SystemExit(f'cannot open {path}')
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
    step = max(1, int(round(fps * stride_s)))
    out = []
    for i in range(0, n, step):
        if len(out) >= limit:
            break
        cap.set(cv2.CAP_PROP_POS_FRAMES, i)
        ok, f = cap.read()
        if ok:
            out.append((f'{os.path.basename(path)}@{i/fps:.1f}s',
                        cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)))
    cap.release()
    return out


def frames_from_images(patterns, limit):
    out = []
    for pat in patterns:
        for p in sorted(glob.glob(os.path.expanduser(pat))):
            if len(out) >= limit:
                break
            img = cv2.imread(p, cv2.IMREAD_GRAYSCALE)
            if img is not None:
                out.append((os.path.basename(p), img))
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--label', default='')
    ap.add_argument('--video', default='')
    ap.add_argument('--images', nargs='*', default=[])
    ap.add_argument('--stride', type=float, default=2.0)
    ap.add_argument('--max', type=int, default=32)
    ap.add_argument('--threads', type=int, default=2)
    ap.add_argument('--append', action='store_true',
                    help='add to an existing bank instead of replacing it')
    a = ap.parse_args()

    Bank, be = load_modules(a.model, a.threads)
    src = (frames_from_video(a.video, a.stride, a.max) if a.video
           else frames_from_images(a.images, a.max))
    if not src:
        raise SystemExit('no frames: pass --video or --images')

    bank = Bank(be, capacity=max(a.max, 8))
    if a.append and os.path.exists(a.out):
        print(f'  appending to {a.out} ({bank.load(a.out)} existing)')

    print(f'  backend {be.w}x{be.h}  label={a.label!r}  {len(src)} candidates')
    kept = 0
    for name, gray in src:
        # `force`: these are hand-chosen practice frames, so the detector's
        # confidence gate does not apply -- but the EMPTY-reference check still
        # does, which is the one that matters for a still.
        r = bank.enrol(gray, roi=None, det_conf=1.0, label=a.label, force=True)
        flag = 'ok ' if r.accepted else f'skip({r.reason})'
        print(f'    {flag:<14} {r.keypoints:>5} kp  {name}')
        kept += int(r.accepted)

    if not kept:
        raise SystemExit('nothing enrolled -- refusing to write an empty bank')
    n = bank.save(a.out)
    size = os.path.getsize(a.out) / 1e6
    print(f'\n  wrote {n} references to {a.out} ({size:.1f} MB)')
    print('  ⚠ a preloaded reference still has to clear MIN_INLIERS on the '
          'run; it is not trusted because it was prepared.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
