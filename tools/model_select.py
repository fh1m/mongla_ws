#!/usr/bin/env python3
"""Rank candidate models by what deployment actually needs.

THE PROBLEM, MEASURED ON OUR OWN ARCHIVE. Three octagon models, all
reporting mAP50 = 0.9950 on their own validation split, evaluated on two
sessions none of them trained on:

    model                          val mAP50   val mAP50-95   session A   session B
    octagon_n_200_v1                 0.9950       0.8470        29.2 %      23.3 %
    octagon_n_200_final              0.9950       0.9278        72.7 %      71.4 %
    octagon_n_200_final_again3       0.9950       0.9570        68.3 %      65.4 %

A 2.5-3x spread in real recall, reproducible across both unseen sessions,
that `mAP50` reports as EXACTLY IDENTICAL. Picking on the number the team
recorded and quoted is picking at random.

And the first two have LITERALLY IDENTICAL training configs -- same base
weights, same dataset, same epochs, same lr, `seed: 0`, `deterministic:
true`. The 43-point recall gap is run-to-run variance that nothing in the
training log surfaces.

WHAT TO USE INSTEAD

  * `mAP50` is SATURATED. Every one of the 25 archived runs reports 0.995,
    most reaching it by epoch 27-38 of 200-300. It cannot rank anything.
  * `mAP50-95` retains signal WITHIN one dataset -- it ranked the two
    identical-config runs correctly. Across different datasets it is not
    comparable.
  * Cross-session recall is the only quantity that predicts deployment, and
    it needs a session the model did not train on.

So: train several, and rank them HERE. The cost is one evaluation pass per
model per session, which is minutes.

    python3 tools/model_select.py --models m1.pt m2.pt --sessions dirA dirB
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

from recall_matrix import evaluate                      # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--models', nargs='+', required=True)
    ap.add_argument('--sessions', nargs='+', required=True,
                    help='dataset dirs the models did NOT train on')
    ap.add_argument('--conf', type=float, default=0.15)
    ap.add_argument('--imgsz', type=int, default=640)
    ap.add_argument('--samples', type=int, default=80)
    a = ap.parse_args()

    from ultralytics import YOLO

    print(f'\n  ranking {len(a.models)} models on {len(a.sessions)} unseen '
          f'sessions  (conf {a.conf}, {a.samples} imgs each)\n')
    hdr = f'  {"model":<38}'
    for s in a.sessions:
        hdr += f'{os.path.basename(s)[:11]:>13}'
    print(hdr + f'{"MEAN":>9}{"WORST":>9}')

    rows = []
    for mp in a.models:
        m = YOLO(mp)
        tag = os.path.basename(os.path.dirname(os.path.dirname(mp)))[:37]
        recs = []
        line = f'  {tag:<38}'
        for s in a.sessions:
            r = evaluate(m, s, a.conf, a.imgsz, a.samples)
            recs.append(r['recall'])
            line += f'{100 * r["recall"]:12.1f}%'
        mean = sum(recs) / len(recs)
        worst = min(recs)
        rows.append((worst, mean, tag))
        print(line + f'{100 * mean:8.1f}%{100 * worst:8.1f}%', flush=True)

    rows.sort(reverse=True)
    print(f'\n  PICK: {rows[0][2]}  (worst-session recall '
          f'{100 * rows[0][0]:.1f} %)')
    print(f'  Ranked on the WORST session, not the mean: a model that is '
          f'excellent\n  on one venue and useless on another loses the run '
          f'it is used in.\n')
    if len(rows) > 1:
        spread = rows[0][0] - rows[-1][0]
        print(f'  spread between best and worst candidate: '
              f'{100 * spread:.1f} points of recall -- and validation mAP50 '
              f'would\n  likely rank them all equal.\n')
    return 0


if __name__ == '__main__':
    sys.exit(main())
