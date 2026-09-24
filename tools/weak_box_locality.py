#!/usr/bin/env python3
"""Are the weak boxes inside a gap WHERE THE TARGET WAS, or scattered noise?

`gap_recovery_potential.py` showed every gap contains weak detections. That is
necessary and not sufficient: a region-aware bar only helps if those boxes sit
near where the target was last seen. If they are uniformly scattered, lowering
the bar anywhere admits noise at the same rate it admits targets, and the
mechanism is worse than useless -- it would feed the tracker exactly the
false positives the bar exists to reject.

⛔ THE CONTROL MATTERS MORE THAN THE MEASUREMENT. "Near the last sighting" is
meaningless without knowing how near a RANDOM box would be, so the same
distance is computed against uniformly-drawn points over the frame. If the two
distributions agree, there is no locality and the prior is worthless.
"""
from __future__ import annotations

import argparse
import math
import random
import sys


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--topic', default='/mongla/vision/forward/detections')
    ap.add_argument('--bar', type=float, default=0.50)
    ap.add_argument('--w', type=float, default=640.0)
    ap.add_argument('--h', type=float, default=360.0)
    args = ap.parse_args()

    from rclpy.serialization import deserialize_message
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
    from vision_msgs.msg import Detection2DArray

    r = SequentialReader()
    r.open(StorageOptions(uri=args.bag, storage_id='mcap'),
           ConverterOptions('cdr', 'cdr'))
    frames = []
    while r.has_next():
        topic, data, stamp = r.read_next()
        if topic != args.topic:
            continue
        m = deserialize_message(data, Detection2DArray)
        hs = m.header.stamp
        t = hs.sec + hs.nanosec * 1e-9
        if t <= 0:
            t = stamp / 1e9
        frames.append((t, [(max((float(x.hypothesis.score)
                                 for x in d.results), default=0.0),
                            float(d.bbox.center.position.x),
                            float(d.bbox.center.position.y))
                           for d in m.detections]))
    if not frames:
        print('no detections')
        return 1

    rng = random.Random(0)
    near, rand = [], []
    last_strong = None
    for t, ds in frames:
        strong = [d for d in ds if d[0] >= args.bar]
        if strong:
            b = max(strong, key=lambda d: d[0])
            last_strong = (b[1], b[2])
            continue
        if last_strong is None:
            continue
        for sc, cx, cy in ds:
            if 0.0 < sc < args.bar:
                near.append(math.hypot(cx - last_strong[0],
                                       cy - last_strong[1]))
                rand.append(math.hypot(rng.uniform(0, args.w) - last_strong[0],
                                       rng.uniform(0, args.h) - last_strong[1]))
    if not near:
        print('no weak boxes after a sighting')
        return 1

    near.sort()
    rand.sort()

    def pct(v, q):
        return v[max(0, min(len(v) - 1, int(q / 100.0 * (len(v) - 1))))]

    print(f'weak boxes measured      {len(near)}')
    print(f'{"":22}{"WEAK":>10}{"RANDOM":>10}   (px from last sighting)')
    for q in (25, 50, 75, 90):
        print(f'  p{q:<20}{pct(near, q):10.0f}{pct(rand, q):10.0f}')
    r_frac = 0.25 * args.w
    a = 100.0 * sum(1 for d in near if d <= r_frac) / len(near)
    b = 100.0 * sum(1 for d in rand if d <= r_frac) / len(rand)
    print()
    print(f'within a {r_frac:.0f} px prior radius:  '
          f'weak {a:.0f} %   random {b:.0f} %')
    if a > b * 1.3:
        print(f'\n⭐ LOCALITY IS REAL: {a / max(b, 1e-9):.2f}x more of the weak '
              f'boxes fall near the last sighting than chance. A region-aware '
              f'bar admits targets faster than it admits noise.')
    else:
        print('\n⛔ NO LOCALITY. The weak boxes are no closer to the last '
              'sighting than random, so a region-aware bar would admit noise '
              'at the same rate as targets. The mechanism does not apply.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
