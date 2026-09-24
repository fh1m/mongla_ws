#!/usr/bin/env python3
"""Run `continuity.analyse` on a RECORDED session -- the measurement it waits for.

⛔ WHY THIS EXISTS. `mongla_vision/continuity.py` says, in its own docstring,
that six constants protect a target lock -- the freshness ramp, `coast_s`,
`lost_grace_s`, the Kalman `max_predict_s` and the tracker's `track_buffer` --
and that every one is sized from the detection RATE and from the others'
ordering, **not one from the quantity they actually defend against: how long a
real gap lasts.** It then says nothing in this repo has ever recorded that.

The module has sat unimported ever since, because the measurement did not
exist. As of 2026-09-24 it does: a 253 s vehicle recording with 108 real
detection gaps. This is the adapter between them.

⚠ ONE HONEST LIMIT, STATED UP FRONT. The recording is a PERSON walking in and
out of a room, not a prop in water. Gap DURATIONS are set by how long the
target was actually absent, which is a property of the subject and the
occlusions, not of the detector's underwater performance. So this sizes the
constants against a real gap distribution -- which is strictly better than
sizing them against each other -- and it is not a competition-water number.
The clip is a fixture, and `presence` and `score` in particular belong to a
person in air.
"""
from __future__ import annotations

import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), 'src', 'mongla_vision'))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--topic', default='/mongla/vision/forward/detections')
    ap.add_argument('--label', default='')
    args = ap.parse_args()

    from rclpy.serialization import deserialize_message
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
    from vision_msgs.msg import Detection2DArray

    from mongla_vision.continuity import Obs, analyse

    r = SequentialReader()
    r.open(StorageOptions(uri=args.bag, storage_id='mcap'),
           ConverterOptions('cdr', 'cdr'))

    obs = []
    while r.has_next():
        topic, data, stamp = r.read_next()
        if topic != args.topic:
            continue
        m = deserialize_message(data, Detection2DArray)
        # ⛔ THE FRAME'S OWN STAMP, not the bag's receive time. A gap measured
        # on arrival times folds transport jitter into the very quantity the
        # ladder is sized from.
        h = m.header.stamp
        t = h.sec + h.nanosec * 1e-9
        if t <= 0.0:
            t = stamp / 1e9
        if not m.detections:
            obs.append(Obs(t=t, seen=False))
            continue
        best, score = None, -1.0
        for d in m.detections:
            s = max((float(h_.hypothesis.score) for h_ in d.results),
                    default=0.0)
            if s > score:
                best, score = d, s
        bb = best.bbox
        # Normalised, so jitter is comparable across sources. The recording is
        # 640x360; read it from the box rather than assuming.
        w = float(os.environ.get('MONGLA_FRAME_W', 640))
        hgt = float(os.environ.get('MONGLA_FRAME_H', 360))
        obs.append(Obs(t=t, seen=True, score=score,
                       cx=float(bb.center.position.x) / w,
                       cy=float(bb.center.position.y) / hgt,
                       track_id=-1))

    if not obs:
        print(f'no messages on {args.topic}')
        return 1

    rep = analyse(obs, args.label or os.path.basename(args.bag))
    print(rep.text())

    # ⭐ THE POINT: what each constant would have to be to cover a stated
    # fraction of the REAL gaps, instead of being sized against the others.
    d = sorted(g.duration for g in rep.gaps)
    if d:
        def pct(q):
            k = max(0, min(len(d) - 1, int(round(q / 100.0 * (len(d) - 1)))))
            return d[k]
        print('\n    === what the MEASURED gaps imply ===')
        for q in (50, 75, 90, 95, 99):
            print(f'        cover {q:>2} % of gaps  ->  {pct(q):5.2f} s')
        print(f'        cover every gap    ->  {d[-1]:5.2f} s')
    return 0


if __name__ == '__main__':
    sys.exit(main())
