#!/usr/bin/env python3
"""Is there anything to recover inside a gap? Measured, before building on it.

The prior-guided idea says: lower the bar where the target is expected, and
weak detections that were being discarded become recovered targets. Every
paper that proposes it reports a gain.

⛔ NONE OF THAT IS EVIDENCE FOR OUR PIPELINE. The idea only pays if weak
detections ACTUALLY EXIST during our gaps. If the detector produces literally
nothing while the target is out of view, a lower bar recovers nothing and the
whole mechanism is decoration.

So this asks the recorded session directly: during the gaps, what did the
detector produce below the acting threshold, and where was it?
"""
from __future__ import annotations

import argparse
import os
import sys


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--topic', default='/mongla/vision/forward/detections')
    ap.add_argument('--bar', type=float, default=0.50,
                    help='the ACTING threshold; below this is "weak"')
    args = ap.parse_args()

    from rclpy.serialization import deserialize_message
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
    from vision_msgs.msg import Detection2DArray

    r = SequentialReader()
    r.open(StorageOptions(uri=args.bag, storage_id='mcap'),
           ConverterOptions('cdr', 'cdr'))

    frames = []           # (t, [(score, cx, cy)])
    while r.has_next():
        topic, data, stamp = r.read_next()
        if topic != args.topic:
            continue
        m = deserialize_message(data, Detection2DArray)
        hs = m.header.stamp
        t = hs.sec + hs.nanosec * 1e-9
        if t <= 0:
            t = stamp / 1e9
        dets = []
        for d in m.detections:
            sc = max((float(x.hypothesis.score) for x in d.results), default=0.0)
            dets.append((sc, float(d.bbox.center.position.x),
                         float(d.bbox.center.position.y)))
        frames.append((t, dets))

    if not frames:
        print('no detection messages')
        return 1

    strong = [(t, [d for d in ds if d[0] >= args.bar]) for t, ds in frames]

    # A gap is a maximal run with NO strong detection.
    gaps, run = [], None
    for i, (t, ds) in enumerate(strong):
        if not ds:
            run = i if run is None else run
        elif run is not None:
            gaps.append((run, i - 1))
            run = None
    if run is not None:
        gaps.append((run, len(strong) - 1))

    real = [(a, b) for a, b in gaps
            if (frames[b][0] - frames[a][0]) >= 0.2]

    n_with_weak = 0
    weak_frames = 0
    gap_frames = 0
    best_weak = []
    for a, b in real:
        got = False
        for i in range(a, b + 1):
            gap_frames += 1
            ds = frames[i][1]
            w = [d for d in ds if 0.0 < d[0] < args.bar]
            if w:
                weak_frames += 1
                got = True
                best_weak.append(max(d[0] for d in w))
        n_with_weak += 1 if got else 0

    dur = frames[-1][0] - frames[0][0]
    print(f'frames            {len(frames)}  over {dur:.1f}s')
    print(f'acting threshold  {args.bar:.2f}')
    print(f'gaps >= 0.2 s     {len(real)}   ({gap_frames} frames inside them)')
    print()
    print(f'gaps containing ANY weak detection: {n_with_weak}/{len(real)} '
          f'({100.0 * n_with_weak / max(len(real), 1):.0f} %)')
    print(f'gap frames with a weak detection  : {weak_frames}/{gap_frames} '
          f'({100.0 * weak_frames / max(gap_frames, 1):.0f} %)')
    if best_weak:
        best_weak.sort()
        n = len(best_weak)
        print(f'their scores: p50 {best_weak[n // 2]:.3f}   '
              f'p90 {best_weak[int(0.9 * (n - 1))]:.3f}   '
              f'max {best_weak[-1]:.3f}')
        print()
        print('⭐ There IS something to recover: a region-aware bar could '
              'accept these where the target was expected.')
    else:
        print()
        print('⛔ NOTHING to recover. The detector produced no weak box at '
              'all inside the gaps, so a lower bar recovers nothing and the '
              'prior-guided idea does not apply to this pipeline.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
