#!/usr/bin/env python3
"""Gap distribution on REAL UNDERWATER FOOTAGE, with a real competition model.

Section 50 measured the quantity the lock ladder is sized on -- how long a real
detection gap lasts -- but only on a PERSON walking in and out of a room. Gap
durations there belong to the subject and its occlusions, not to underwater
detection, which is exactly why those constants were shipped as a candidate
rather than as defaults.

This closes the distance the archive can close: the same `continuity.analyse`,
run over real competition footage with the model that actually flies. It is
still not a pool day -- the vehicle is not moving under its own control and
nothing closes a loop -- but the DETECTOR's own dropout behaviour on real
props in real water is precisely what the archive can answer and a person in a
room cannot.
"""
from __future__ import annotations

import argparse
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(os.path.dirname(HERE), 'src', 'mongla_vision'))
from archive_root import archive_root                        # noqa: E402

A = archive_root()

CLIPS = {
    'gate (mirpur)':      f'{A}/Mirpur/Sun_June_21/gate.mkv',
    'torpedo (mirpur)':   f'{A}/Mirpur/Sun_June_21/torpedo.mkv',
    'torpedo_1 (mirpur)': f'{A}/Mirpur/Sun_June_21/torpedo_1.mkv',
    'octagon (robosub)':  f'{A}/robosub/clips/octagon/octagon_1.mp4',
    'torpedo (robosub)':  f'{A}/robosub/clips/torpedo/torpedo_shark_up_1.mp4',
    'bin (final_run)':    f'{A}/final_run/bin.mkv',
}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', default='/home/fh1m/hailo/work/gate_rescue_repair.onnx')
    ap.add_argument('--conf', type=float, default=0.25)
    ap.add_argument('--seconds', type=float, default=60.0)
    ap.add_argument('--stride', type=int, default=2)
    args = ap.parse_args()

    from ultralytics import YOLO
    from mongla_vision.continuity import Obs, analyse

    model = YOLO(args.model, task='detect')
    print(f'model: {os.path.basename(args.model)}   conf {args.conf}\n')

    allrep = []
    for name, path in CLIPS.items():
        if not os.path.exists(path):
            continue
        cap = cv2.VideoCapture(path)
        fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
        total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
        if total < 30:
            cap.release()
            continue
        # ⛔ REFUSE AN IMPLAUSIBLE FRAME RATE. The Mirpur MKVs report fps=250
        # and fps=1000 -- variable-frame-rate containers that OpenCV cannot
        # read honestly. EVERY number here is a duration, so a wrong time base
        # silently compresses the gaps: those clips produced a p50 of 6 ms and
        # contributed 469 of 507 pooled gaps, which would have been published
        # as "real water says the ladder is generous". A gap measured on a
        # fictional clock is not a small gap, it is not a gap at all.
        if not (5.0 <= fps <= 120.0):
            print(f'  === {name} ===\n    ⛔ REFUSED: fps reads {fps:.0f}, '
                  f'which is not a real rate. Every measurement here is a '
                  f'DURATION, so this clip cannot be timed.')
            cap.release()
            continue
        # A window from the middle: the start of a clip is often the deck.
        start = int(total * 0.35)
        n = min(int(args.seconds * fps), total - start)
        cap.set(cv2.CAP_PROP_POS_FRAMES, start)
        obs = []
        for k in range(0, n, args.stride):
            ok, im = cap.read()
            if not ok:
                break
            for _ in range(args.stride - 1):
                cap.grab()
            # ⛔ TIME IS THE FRAME'S OWN, from the clip's rate -- not the wall
            # clock of this script. A gap measured in processing time would
            # measure this laptop, not the water.
            t = (start + k) / fps
            r = model.predict(im, conf=args.conf, verbose=False)[0]
            if r.boxes is None or len(r.boxes) == 0:
                obs.append(Obs(t=t, seen=False))
                continue
            confs = r.boxes.conf.cpu().numpy()
            i = int(np.argmax(confs))
            xywh = r.boxes.xywh.cpu().numpy()[i]
            h_, w_ = im.shape[:2]
            obs.append(Obs(t=t, seen=True, score=float(confs[i]),
                           cx=float(xywh[0]) / w_, cy=float(xywh[1]) / h_))
        cap.release()
        if len(obs) < 20:
            continue
        rep = analyse(obs, name)
        print(rep.text())
        allrep.append(rep)

    if not allrep:
        print('no clips scored')
        return 1

    gaps = sorted(g.duration for r in allrep for g in r.gaps)
    if gaps:
        def pct(q):
            k = max(0, min(len(gaps) - 1, int(round(q / 100.0 * (len(gaps) - 1)))))
            return gaps[k]
        print('\n  === ALL CLIPS POOLED: real props, real water ===')
        print(f'    gaps n={len(gaps)}   p50 {pct(50):.2f}  p75 {pct(75):.2f}  '
              f'p90 {pct(90):.2f}  p95 {pct(95):.2f}  p99 {pct(99):.2f}  '
              f'max {gaps[-1]:.2f} s')
        print('\n    coverage of each SHIPPED rung:')
        for nm, thr in (('freshness 0.20', 0.20), ('coast_s 0.80', 0.80),
                        ('lost_grace 1.00', 1.00), ('kalman 1.50', 1.50),
                        ('track_buffer 5.00', 5.00)):
            cov = 100.0 * sum(1 for g in gaps if g <= thr) / len(gaps)
            print(f'        {nm:<20} covers {cov:5.1f} %')
    return 0


if __name__ == '__main__':
    sys.exit(main())
