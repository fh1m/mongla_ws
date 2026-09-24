#!/usr/bin/env python3
"""Does a loop closure fire at a REVISIT, and nowhere else?

The gates in `anchor/loop_closure.py` are unit-tested against synthetic poses.
This asks the only question that matters on real water: run the real bank over
real downward footage and count what would have been handed to the filter.

TWO ARMS, because a closure has two ways to be wrong and they are opposites:

  SAME-CLIP   build a bank from the first part of a clip, then replay the rest.
              A closure here is either a genuine revisit or the floor aliasing,
              and the clip's own geometry decides which -- so this arm reports
              a RATE, not a verdict.

  CROSS-CLIP  build a bank from clip A and replay clip B. Every closure here is
              FALSE by construction: the vehicle has never been in venue B's
              pool while holding venue A's map. ⛔ THIS ARM IS THE TEST. Any
              non-zero count is a bar that is too low, and the number to report
              is the threshold at which it reaches zero.

⚠ WHAT THIS CANNOT DO. There is no ground-truth trajectory for these clips, so
a same-clip closure cannot be proven to be a real revisit. The cross-clip arm
needs no ground truth at all, which is why the bar is set from it.
"""
from __future__ import annotations

import argparse
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from archive_root import archive_root                        # noqa: E402

A = archive_root()
# ⛔ IMPORTED AS A PACKAGE, not loaded by file path. `bank.py` uses relative
# imports (`from .anchor import ...`), so file-loading it raises "attempted
# relative import with no known parent package". Put the package's parent on
# sys.path and import normally, which also guarantees this measures the SAME
# module the node runs rather than a second copy of it.
_PKG = os.path.join(os.path.dirname(HERE), 'src', 'mongla_vision')
sys.path.insert(0, _PKG)

DOWNWARD = {
    'octagon_1': f'{A}/robosub/clips/octagon/octagon_1.mp4',
    'octagon_Bottom': f'{A}/final_run/octagon_Bottom.mkv',
    'bin': f'{A}/final_run/bin.mkv',
}


def frames(path, n, start=0.0, end=1.0):
    cap = cv2.VideoCapture(path)
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
    if total <= 0:
        cap.release()
        return []
    lo, hi = int(total * start), int(total * end)
    idx = np.linspace(lo, max(lo + 1, hi - 1), n).astype(int)
    out = []
    for i in idx:
        cap.set(cv2.CAP_PROP_POS_FRAMES, int(i))
        ok, img = cap.read()
        if ok:
            out.append(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))
    cap.release()
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--xfeat', required=True)
    ap.add_argument('--enrol', type=int, default=12)
    ap.add_argument('--probe', type=int, default=40)
    args = ap.parse_args()

    from mongla_vision.anchor import bank as bank_mod
    from mongla_vision.anchor import loop_closure as lc
    from mongla_vision.anchor import xfeat_onnx as X

    def build(path, n):
        """A bank of PLACES from the first half of a clip, each with a
        position. The positions are synthetic -- there is no ground truth --
        but they only have to be DISTINCT, because what is being measured is
        whether the wrong place is recognised, not where it is."""
        b = bank_mod.CheckpointBank(X.XFeatONNX(args.xfeat, top_k=1024),
                                    capacity=64)
        for k, g in enumerate(frames(path, n, 0.0, 0.5)):
            b.enrol(g, roi=None, det_conf=1.0, label=lc.PLACE_LABEL,
                    position=(float(k) * 2.0, 0.0))
        return b

    def run(bank, path, n, start, end, tag):
        """Count closures at a sweep of bars. `consider` is given generous
        age and travel so ONLY the evidence bar is under test -- the temporal
        gates are unit-tested and would otherwise mask the number wanted."""
        bars = (15, 40, 60, 80, 100, 120, 150)
        hits = {b: 0 for b in bars}
        best = 0
        seen = 0
        for g in frames(path, n, start, end):
            bp = bank.locate(g, label=lc.PLACE_LABEL)
            seen += 1
            best = max(best, int(bp.inliers))
            for b in bars:
                # ⛔ exclude_newest=0 HERE, and it is not a loosening.
                # With a 12-reference bank, the production value of 2 removes
                # indices 10-11 -- and for a clip probed just after its bank
                # was built, those are exactly the references the frames
                # resemble. The first run of this tool reported
                # `best=511, closures at bar 40: 0`, which is impossible
                # unless the winner was being excluded, and it was. The
                # self-closure guard is unit-tested; what is under test here
                # is the EVIDENCE bar, so the temporal gates are stood down
                # rather than left to mask the number being measured.
                c = lc.consider(bp, ref_age_s=1e6, travel_m=1e6,
                                ref_sigma_m=0.3, m_per_px=0.002,
                                bank_size=bank.size, min_inliers=b,
                                max_offset_m=1e6, exclude_newest=0)
                if c.ok:
                    hits[b] += 1
        print(f'{tag:<34} n={seen:<4} best={best:<5} '
              + '  '.join(f'{b}:{hits[b]}' for b in bars))
        return hits, best

    names = [k for k in DOWNWARD if os.path.exists(DOWNWARD[k])]
    if not names:
        print('no downward clips found under', A)
        return 1

    print(f'bars:                              '
          f'{"":22}' + '  '.join(f'{b}' for b in (15, 40, 60, 80, 100, 120, 150)))
    print('\n=== SAME CLIP: bank from the first half, probe the second ===')
    banks = {}
    for nm in names:
        banks[nm] = build(DOWNWARD[nm], args.enrol)
        run(banks[nm], DOWNWARD[nm], args.probe, 0.5, 1.0, f'{nm} -> itself')

    print('\n=== CROSS CLIP: every closure here is FALSE by construction ===')
    worst = {b: 0 for b in (15, 40, 60, 80, 100, 120, 150)}
    for a in names:
        for b in names:
            if a == b:
                continue
            hits, _ = run(banks[a], DOWNWARD[b], args.probe, 0.0, 1.0,
                          f'{a} -> {b}')
            for k, v in hits.items():
                worst[k] = max(worst[k], v)

    print('\n=== THE BAR ===')
    clean = [b for b in sorted(worst) if worst[b] == 0]
    print('worst false count per bar:',
          '  '.join(f'{b}:{worst[b]}' for b in sorted(worst)))
    if clean:
        print(f'⭐ lowest bar with ZERO false closures: {min(clean)}   '
              f'(shipped bar is {lc.CLOSURE_INLIERS})')
        if min(clean) > lc.CLOSURE_INLIERS:
            print(f'⛔ THE SHIPPED BAR IS TOO LOW on this footage.')
    else:
        print('⛔ NO BAR TESTED reaches zero false closures.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
