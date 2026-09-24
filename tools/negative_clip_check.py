#!/usr/bin/env python3
"""What does the model say when the prop is NOT there?

⛔ THE GAP THIS CLOSES. Every detection bar in `measured-bars.md` section 1 was
derived from labelled held-out PAIRS -- frames that contain the prop. A set
built that way cannot measure a false positive on open water, because it
contains no open water. So a model that fires confidently on empty pool scores
perfectly on it, and the bar comes out far too low.

Measured 2026-09-24 with `gate_rescue_repair.pt`, 1 200 frames per clip:

    conf    gate.mkv (gate)   torpedo.mkv (none)   torpedo_1.mkv (none)
    0.15         100.0 %             90.8 %               33.3 %
    0.45          98.9 %             46.8 %                6.8 %
    0.60          83.2 %              8.6 %                1.7 %

At the shipped 0.15 the model claimed a gate on 90.8 % of gate-free frames,
with half-frame boxes over empty turquoise at 0.44-0.56. That is B-59, and it
was invisible to every check we had.

WHAT IT REPORTS. Per-frame detection rate for one class on clips you assert
CONTAIN the prop and clips you assert do NOT. The separation between those two
columns is the only honest basis for a confidence bar.

⚠ IT CANNOT CHECK YOUR LABELS. "negative" means you looked at the clip and the
prop is not in it. Get that wrong and this reports nonsense with total
confidence -- so render the boxes and look, the way B-59 was actually caught.

    python3 tools/negative_clip_check.py \\
        --model src/mongla_vision/models/gate_rescue_repair.pt --class gate \\
        --positive Mirpur/Sun_June_21/gate.mkv \\
        --negative Mirpur/Sun_June_21/torpedo.mkv Mirpur/Sun_June_21/torpedo_1.mkv
"""
from __future__ import annotations

import argparse
import os
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

BARS = (0.15, 0.30, 0.45, 0.60, 0.75)


def class_index(model_path: pathlib.Path, want: str) -> int:
    import yaml
    side = model_path.with_suffix('.yaml')
    if not side.exists():
        raise SystemExit(f'no sidecar {side.name}')
    names = yaml.safe_load(side.read_text()).get('names', {})
    for k, v in names.items():
        if str(v) == want:
            return int(k)
    raise SystemExit(f'class {want!r} not in {sorted(names.values())}')


def rate(model, clip: pathlib.Path, idx: int, limit: int) -> tuple:
    """Fraction of frames carrying a detection of `idx` at or above each bar."""
    import cv2
    cap = cv2.VideoCapture(str(clip))
    hits = {b: 0 for b in BARS}
    n = 0
    while n < limit:
        ok, frame = cap.read()
        if not ok:
            break
        n += 1
        r = model.predict(frame, conf=min(BARS), verbose=False)[0]
        best = max((float(b.conf[0]) for b in r.boxes
                    if int(b.cls[0]) == idx), default=0.0)
        for b in BARS:
            if best >= b:
                hits[b] += 1
    cap.release()
    return n, hits


def resolve(spec: str) -> pathlib.Path:
    p = pathlib.Path(spec)
    if p.is_file():
        return p
    from archive_root import archive_root
    q = pathlib.Path(archive_root()) / spec
    if not q.is_file():
        raise SystemExit(f'no clip {spec}')
    return q


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', required=True)
    ap.add_argument('--class', dest='cls', required=True)
    ap.add_argument('--positive', nargs='*', default=[])
    ap.add_argument('--negative', nargs='*', default=[])
    ap.add_argument('--limit', type=int, default=1200)
    a = ap.parse_args()

    if not a.negative:
        raise SystemExit(
            'REFUSING: no --negative clips. Without footage where the prop is '
            'ABSENT this measures nothing the existing held-out sweep did not '
            'already measure, and it was that sweep missing negatives which '
            'hid B-59.')

    from ultralytics import YOLO
    model_path = pathlib.Path(a.model)
    idx = class_index(model_path, a.cls)
    model = YOLO(str(model_path), task='detect')

    print(f'{model_path.name}   class {a.cls!r} (index {idx})\n')
    head = '  '.join(f'>={b:.2f}' for b in BARS)
    print(f'{"clip":<34}{"truth":<10}{"n":>6}   {head}')

    rows = []
    for spec, truth in ([(s, 'present') for s in a.positive]
                        + [(s, 'ABSENT') for s in a.negative]):
        clip = resolve(spec)
        n, hits = rate(model, clip, idx, a.limit)
        pct = {b: 100.0 * hits[b] / max(n, 1) for b in BARS}
        rows.append((truth, pct))
        cells = '  '.join(f'{pct[b]:6.1f}%' for b in BARS)
        print(f'{os.path.basename(spec):<34}{truth:<10}{n:>6}   {cells}')

    pos = [p for t, p in rows if t == 'present']
    neg = [p for t, p in rows if t == 'ABSENT']
    if not pos:
        print('\n⚠ no positive clip given, so this shows the false-positive '
              'rate alone and cannot recommend a bar.')
        return 0

    print('\n     bar   worst positive   worst negative   separation')
    best = None
    for b in BARS:
        lo = min(p[b] for p in pos)
        hi = max(p[b] for p in neg)
        sep = lo - hi
        print(f'    {b:.2f}         {lo:6.1f}%          {hi:6.1f}%      '
              f'{sep:+7.1f} pts')
        if best is None or sep > best[1]:
            best = (b, sep)
    print(f'\n⭐ widest separation at conf {best[0]:.2f} ({best[1]:+.1f} pts). '
          f'⚠ This is a per-FRAME detection rate on the clips you named, not '
          f'a recall figure: a frame where the prop is out of view counts '
          f'against the positive column. Treat it as where to look, then '
          f'confirm by rendering the boxes.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
