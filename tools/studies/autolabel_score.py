#!/usr/bin/env python3
"""Score an auto-labeller's boxes against hand labels it never saw.

Bead mongla_ws-qqd. Written BEFORE any candidate runs, with its bar:

  An auto-labeller passes on a held-out hand-labelled set when, at IoU 0.5,
  precision >= 0.90 and recall >= 0.85 over every class together, and no
  class with >= 20 truth boxes has recall under 0.70. Labels that clear this
  are cheaper to correct than to draw. The second, decisive test is
  downstream: a model trained on its labels scores within 0.03 mAP50 of the
  same model trained on hand labels, on hand-labelled frames neither saw.

  Falsified if: --self-test does not read back each planted error.

Both folders hold YOLO txt files with the same stems (one per image). A
class map (`--map their=ours,...`) unifies names when the two disagree;
boxes are never moved. Matching is greedy by IoU within a class, highest
first, which is what mAP50 counts.

    tools/studies/autolabel_score.py TRUTH_DIR CANDIDATE_DIR [--names a,b,c]
    tools/studies/autolabel_score.py --self-test
"""
from __future__ import annotations

import argparse
import collections
import json
import pathlib
import random
import sys

IOU = 0.5
BAR_PRECISION, BAR_RECALL, BAR_CLASS_RECALL, CLASS_MIN_BOXES = 0.90, 0.85, 0.70, 20


def read_labels(path: pathlib.Path) -> list:
    out = []
    if not path.exists():
        return out
    for line in path.read_text().splitlines():
        parts = line.split()
        if len(parts) >= 5:
            c, x, y, w, h = int(parts[0]), *map(float, parts[1:5])
            out.append((c, x - w / 2, y - h / 2, x + w / 2, y + h / 2))
    return out


def iou(a, b) -> float:
    ix = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    iy = max(0.0, min(a[4], b[4]) - max(a[2], b[2]))
    inter = ix * iy
    union = (a[3] - a[1]) * (a[4] - a[2]) + (b[3] - b[1]) * (b[4] - b[2]) - inter
    return inter / union if union > 0 else 0.0


def score(pairs) -> dict:
    """`pairs`: iterable of (truth boxes, candidate boxes) per image."""
    tp, fp, fn = collections.Counter(), collections.Counter(), collections.Counter()
    ious = []
    for truth, cand in pairs:
        used = set()
        for t in truth:
            best, j = 0.0, -1
            for k, c in enumerate(cand):
                if k in used or c[0] != t[0]:
                    continue
                v = iou(t, c)
                if v > best:
                    best, j = v, k
            if best >= IOU:
                used.add(j)
                tp[t[0]] += 1
                ious.append(best)
            else:
                fn[t[0]] += 1
        for k, c in enumerate(cand):
            if k not in used:
                fp[c[0]] += 1
    T, F, N = sum(tp.values()), sum(fp.values()), sum(fn.values())
    classes = sorted(set(tp) | set(fn) | set(fp))
    per = {c: {'truth': tp[c] + fn[c],
               'recall': tp[c] / (tp[c] + fn[c]) if tp[c] + fn[c] else None,
               'precision': tp[c] / (tp[c] + fp[c]) if tp[c] + fp[c] else None}
           for c in classes}
    out = {'precision': T / (T + F) if T + F else None,
           'recall': T / (T + N) if T + N else None,
           'mean_iou': sum(ious) / len(ious) if ious else None,
           'truth_boxes': T + N, 'candidate_boxes': T + F, 'per_class': per}
    weak = [c for c, p in per.items()
            if p['truth'] >= CLASS_MIN_BOXES and (p['recall'] or 0.0) < BAR_CLASS_RECALL]
    out['passes'] = (out['precision'] is not None and out['precision'] >= BAR_PRECISION
                     and (out['recall'] or 0.0) >= BAR_RECALL and not weak)
    out['weak_classes'] = weak
    return out


def _folder_pairs(truth: pathlib.Path, cand: pathlib.Path, remap: dict):
    for t in sorted(truth.glob('*.txt')):
        boxes = [(remap.get(c[0], c[0]), *c[1:]) for c in read_labels(cand / t.name)]
        yield read_labels(t), boxes


def self_test() -> int:
    """Planted errors of known size must read back exactly."""
    rnd = random.Random(7)
    # One box per quadrant, so a planted error cannot land on a neighbour and
    # match it by chance (a first version with free placement read 0.002
    # where 0 was planted: the self-test caught its own fixture).
    truth = [[(rnd.randrange(3), qx + rnd.uniform(0, .3), qy + rnd.uniform(0, .3),
               0, 0) for qx, qy in ((0, 0), (.5, 0), (0, .5), (.5, .5))]
             for _ in range(250)]                      # 1000 boxes
    truth = [[(c, x, y, x + 0.1, y + 0.1) for c, x, y, _, _ in img] for img in truth]
    ok = True

    def check(name, cand, p, r):
        nonlocal ok
        s = score(zip(truth, cand))
        good = abs(s['precision'] - p) < 1e-9 and abs(s['recall'] - r) < 1e-9
        ok &= good
        print(f"{'ok  ' if good else 'FAIL'} {name}: precision {s['precision']:.3f} "
              f"(want {p:.3f}) recall {s['recall']:.3f} (want {r:.3f})")

    check('identical', [list(b) for b in truth], 1.0, 1.0)
    # Drop every 10th box: recall 0.9, precision 1.
    flat = [[b for i, b in enumerate(img) if (k * 4 + i) % 10] for k, img in enumerate(truth)]
    check('10 % missed', flat, 1.0, 0.9)
    # One false box per image far from the truth: 1000 tp, 250 fp.
    check('a false box per image', [img + [(0, .95, .95, .99, .99)] for img in truth], 0.8, 1.0)
    # Shift every box half its width: IoU 1/3 < 0.5, so all miss and all are false.
    check('shifted half a box', [[(c, a + .05, b, d + .05, e) for c, a, b, d, e in img]
                                 for img in truth], 0.0, 0.0)
    # Wrong class on every box: nothing matches.
    check('wrong class', [[((c + 1) % 3, *r) for c, *r in img] for img in truth], 0.0, 0.0)
    print('self-test', 'passed' if ok else 'FAILED')
    return 0 if ok else 1


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=(__doc__ or '').split('\n')[0])
    ap.add_argument('truth', nargs='?', type=pathlib.Path)
    ap.add_argument('candidate', nargs='?', type=pathlib.Path)
    ap.add_argument('--map', default='', help='candidate class id -> truth id, e.g. 3=0,4=1')
    ap.add_argument('--self-test', action='store_true')
    a = ap.parse_args(argv)
    if a.self_test:
        return self_test()
    if not (a.truth and a.candidate):
        ap.error('TRUTH_DIR and CANDIDATE_DIR, or --self-test')
    remap = {int(k): int(v) for k, v in (p.split('=') for p in a.map.split(',') if p)}
    s = score(_folder_pairs(a.truth, a.candidate, remap))
    print(json.dumps(s, indent=1))
    return 0 if s['passes'] else 2


if __name__ == '__main__':
    sys.exit(main())
