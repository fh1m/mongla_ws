#!/usr/bin/env python3
"""Does sub-threshold evidence persist WHILE THE TARGET IS STILL IN SIGHT?

Track-before-detect postpones the threshold: a target too dim to clear a
per-frame bar is accumulated across frames until the ACCUMULATION clears one.
Radar and infrared small-target work have done this since the 1980s and report
tracking down to SNR 1 dB. Our ladder begins AT the detector's threshold --
FOLLOW, ANCHOR and the place bank are all ways of surviving after a detection
is lost, and none of them can start without one. This is the only idea we have
found that would extend the ladder BELOW its first rung.

⛔ THE REGIME IS THE WHOLE QUESTION, AND THE FIRST VERSION OF THIS TOOL GOT IT
WRONG. Most gaps in a walk-in/walk-out recording are the subject LEAVING THE
FRAME. The target is absent, so the weak boxes in those gaps are noise by
construction, and measuring them answers a question nobody asked. What matters
is the dropout that happens while the target is still in view. Those two
populations are mixed together in every bag we own and must be separated
before any statistic means anything:

    IN-VIEW    both bracketing detections sit in the frame interior
               (visibility margin >= MARGIN_ENTER) and near each other
    EXIT/ENTRY a bracket sits in the margin band, or the brackets are far apart

⭐ EXIT/ENTRY IS THE FREE NEGATIVE CONTROL. The target really is gone there, so
its link probability is what pure noise scores on our own hardware, in our own
water, with our own detector. Track-before-detect applies only if IN-VIEW
stands well above shuffle AND EXIT/ENTRY sits near it.

WHY LINK PROBABILITY AND NOT LONGEST RUN. Longest run is capped by the length
of the stretch it is measured in, and our median gap is a few frames, so the
statistic saturates and every population converges toward the same ceiling.
Worse, shuffling a 4-frame stretch barely destroys time -- every frame stays
within about two frames of where it started. Link probability has no ceiling:
P(a weak box lies within `gate` of the previous frame's weak box), over
consecutive frame pairs. The control pairs frames from DIFFERENT gaps, which
removes time completely instead of merely stirring it.

Class-filtered throughout: a sub-threshold chair must not count toward a
person's track.
"""
from __future__ import annotations

import argparse
import math
import random
import sys
from collections import Counter

# The frame interior, borrowed from tracking/visibility.py so there is one
# definition of "in sight" and not two.
MARGIN_ENTER = 0.18
# Brackets further apart than this did not lose the target in place; the
# subject travelled, which is the exit/entry regime under another name.
BRACKET_NEAR_PX = 220.0


def _score_and_class(det):
    best, cls = 0.0, ''
    for h in det.results:
        s = float(h.hypothesis.score)
        if s >= best:
            best, cls = s, str(h.hypothesis.class_id)
    return best, cls


def read_bag(path, topic, bar, floor):
    """Split every frame's detections into strong (>= bar) and weak
    (floor <= score < bar). Returns per-frame lists of (score, x, y, cls)."""
    from rclpy.serialization import deserialize_message
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
    from vision_msgs.msg import Detection2DArray

    reader = SequentialReader()
    reader.open(StorageOptions(uri=path, storage_id='mcap'),
                ConverterOptions('cdr', 'cdr'))
    strong, weak, seen = [], [], []
    while reader.has_next():
        name, data, _stamp = reader.read_next()
        if name != topic:
            continue
        msg = deserialize_message(data, Detection2DArray)
        s, w = [], []
        for d in msg.detections:
            score, cls = _score_and_class(d)
            seen.append(score)
            box = (score, float(d.bbox.center.position.x),
                   float(d.bbox.center.position.y), cls)
            if score >= bar:
                s.append(box)
            elif score >= floor:
                w.append(box)
        strong.append(s)
        weak.append(w)
    return strong, weak, seen


def gap_stretches(strong, min_len):
    idx = [i for i, s in enumerate(strong) if not s]
    if not idx:
        return []
    out, run = [], [idx[0]]
    for a, b in zip(idx, idx[1:]):
        if b == a + 1:
            run.append(b)
        else:
            out.append(run)
            run = [b]
    out.append(run)
    return [r for r in out if len(r) >= min_len]


def margin(x, y, w, h):
    """visibility.py's rectangular barrier: 1 at the centre, 0 at any edge."""
    hw, hh = 0.5 * w, 0.5 * h
    return min(1.0 - abs(x - hw) / hw, 1.0 - abs(y - hh) / hh)


def classify(stretch, strong, w, h):
    """IN-VIEW or EXIT/ENTRY, decided by the detections bracketing the gap."""
    i, j = stretch[0] - 1, stretch[-1] + 1
    if i < 0 or j >= len(strong) or not strong[i] or not strong[j]:
        return None, None
    a = max(strong[i], key=lambda b: b[0])
    b = max(strong[j], key=lambda b: b[0])
    if a[3] != b[3]:
        return None, None
    interior = (margin(a[1], a[2], w, h) >= MARGIN_ENTER
                and margin(b[1], b[2], w, h) >= MARGIN_ENTER)
    near = math.hypot(b[1] - a[1], b[2] - a[2]) <= BRACKET_NEAR_PX
    return ('in_view' if (interior and near) else 'exit_entry'), a[3]


def pick(boxes, cls):
    """Strongest weak box of the bracketing class, or None."""
    c = [b for b in boxes if b[3] == cls]
    return max(c, key=lambda b: b[0]) if c else None


def link_rate(pairs, gate):
    """P(the two frames each hold a weak box of the class, within gate)."""
    if not pairs:
        return float('nan'), 0
    hit = sum(1 for p, q in pairs
              if math.hypot(q[1] - p[1], q[2] - p[2]) <= gate)
    return hit / len(pairs), len(pairs)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--topic', default='/mongla/vision/forward/detections')
    ap.add_argument('--bar', type=float, default=0.45,
                    help='shipped acting bar (detector.yaml conf)')
    ap.add_argument('--floor', type=float, default=0.0,
                    help='lowest score the detector publishes')
    ap.add_argument('--gate', type=float, default=40.0)
    ap.add_argument('--size', default='640x480')
    ap.add_argument('--min-len', type=int, default=3)
    ap.add_argument('--trials', type=int, default=200)
    args = ap.parse_args()

    W, H = (int(v) for v in args.size.lower().split('x'))
    strong, weak, seen = read_bag(args.bag, args.topic, args.bar, args.floor)
    if not weak:
        print('no detections on that topic')
        return 1

    print(f'frames {len(weak)}   detections {len(seen)}   '
          f'published score range {min(seen):.3f}..{max(seen):.3f}')
    print(f'acting bar {args.bar:.2f}   gate {args.gate:.0f} px   '
          f'frame {W}x{H}   interior margin >= {MARGIN_ENTER}\n')

    stretches = gap_stretches(strong, args.min_len)
    lens = Counter(min(len(s), 10) for s in stretches)
    print('gap length (frames, 10 = 10+): '
          + '  '.join(f'{k}:{lens[k]}' for k in sorted(lens)))

    groups = {'in_view': [], 'exit_entry': []}
    for s in stretches:
        kind, cls = classify(s, strong, W, H)
        if kind:
            groups[kind].append((s, cls))
    print(f'classified: in-view {len(groups["in_view"])}   '
          f'exit/entry {len(groups["exit_entry"])}   '
          f'unclassifiable {len(stretches) - sum(len(v) for v in groups.values())}\n')

    rng = random.Random(0)
    print(f'{"":12}{"stretches":>10}{"pairs":>8}{"linked":>9}'
          f'{"control":>9}{"ratio":>8}')
    verdict = {}
    for kind, items in groups.items():
        # Consecutive frame pairs, both holding a weak box of the class.
        pairs, pool = [], []
        for s, cls in items:
            picks = [(i, pick(weak[i], cls)) for i in s]
            for (i, p), (j, q) in zip(picks, picks[1:]):
                if p and q and j == i + 1:
                    pairs.append((p, q))
            pool.extend(p for _i, p in picks if p)
        rate, n = link_rate(pairs, args.gate)
        # ⭐ THE CONTROL: boxes drawn from DIFFERENT gaps, so time is removed
        # entirely rather than stirred inside a four-frame stretch.
        ctrl = 0.0
        if len(pool) >= 2 and n:
            for _ in range(args.trials):
                a, b = rng.sample(pool, 2)
                ctrl += math.hypot(b[1] - a[1], b[2] - a[2]) <= args.gate
            ctrl /= args.trials
        ratio = rate / ctrl if ctrl > 0 else float('nan')
        verdict[kind] = ratio
        print(f'{kind:<12}{len(items):>10}{n:>8}{rate:>9.3f}'
              f'{ctrl:>9.3f}{ratio:>8.2f}x')

    iv, ee = verdict.get('in_view'), verdict.get('exit_entry')
    print()
    if iv and iv == iv and iv >= 1.5 and (not ee or ee != ee or iv >= 1.4 * ee):
        print(f'⭐ IN-SIGHT EVIDENCE PERSISTS at {iv:.2f}x above a time-free '
              f'control, while the exit/entry regime -- where the target is '
              f'genuinely gone -- scores {ee:.2f}x. Sub-threshold energy is '
              f'temporally structured exactly where the ladder needs it, so '
              f'accumulating it accumulates signal. Track-before-detect '
              f'applies.')
        return 0
    print(f'⛔ IN-SIGHT {iv:.2f}x vs EXIT/ENTRY {ee:.2f}x. The in-sight regime '
          f'is not separated from the regime where the target is absent, so '
          f'accumulating post-NMS weak boxes would manufacture a track out of '
          f'noise. Track-before-detect does not apply TO THESE BOXES -- which '
          f'is not the same as saying it does not apply to the raw pre-NMS '
          f'field, a quantity this bag does not contain.')
    return 2


if __name__ == '__main__':
    sys.exit(main())
