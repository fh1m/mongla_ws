#!/usr/bin/env python3
"""Characterise every labelled dataset by its WATER, not by its task.

WHY. We name datasets after the prop -- gate, bin, torpedo -- and then tune
against them as if the prop were the variable. It is not. Two clips of the
SAME competition differ in their optics: sharpness 315 vs 1241, saturation
160 vs 28, and a model trained on one is being asked about the other.

So the useful question is not "do we have gate data" but "what CONDITIONS do
we have data for, and which are we blind to". A model trained only on
saturated, blurry water will meet a clear-water venue and fail, and no amount
of validation mAP on the original set predicts it.

Prints one row per dataset with the five optical statistics, sorted by
sharpness. Where the rows cluster is the honest map of what the archive
covers; there is deliberately no verdict column (see mongla_vision/underwater.py).

    python3 tools/bench/dataset_survey.py --root ~/Work/.../2025/datasets
"""
import argparse
import glob
import os
import random
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..',
                                'src', 'mongla_vision'))

import cv2                                                  # noqa: E402
import yaml                                                 # noqa: E402

from mongla_vision.underwater import analyse_frames      # noqa: E402

_IMG = ('.png', '.jpg', '.jpeg', '.bmp', '.webp')


def _sample(root, k, seed=11):
    """Frames spread across the whole set, not the first k.

    Datasets are usually written in capture order, so the first k images are
    one moment of one run -- which is how a survey comes to describe a
    lighting condition that lasted thirty seconds.
    """
    files = []
    for split in ('train', 'valid', 'val', 'test'):
        d = os.path.join(root, split, 'images')
        if os.path.isdir(d):
            files += [os.path.join(d, f) for f in os.listdir(d)
                      if f.lower().endswith(_IMG)]
    if not files:
        return []
    files.sort()
    rng = random.Random(seed)
    return rng.sample(files, min(k, len(files)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--root', required=True)
    ap.add_argument('--samples', type=int, default=40)
    a = ap.parse_args()

    rows = []
    for dy in sorted(glob.glob(f'{a.root}/**/data.yaml', recursive=True)):
        root = os.path.dirname(dy)
        paths = _sample(root, a.samples)
        if len(paths) < 5:
            continue
        try:
            cfg = yaml.safe_load(open(dy)) or {}
        except Exception:
            cfg = {}
        names = cfg.get('names') or []
        if isinstance(names, dict):
            names = list(names.values())
        frames = [cv2.imread(p) for p in paths]
        st = analyse_frames([f for f in frames if f is not None])
        rows.append((os.path.relpath(root, a.root), st, names))

    rows.sort(key=lambda r: r[1].sharpness)
    print(f'\n  {len(rows)} datasets, sampled {a.samples} frames each, '
          f'sorted by sharpness (blurriest first)\n')
    print(f'  {"dataset":<46}{"sharp":>8}{"contr":>7}{"sat":>7}'
          f'{"cast":>7}{"bright":>8}  classes')
    for rel, st, names in rows:
        print(f'  {rel[:45]:<46}{st.sharpness:8.0f}{st.contrast:7.1f}'
              f'{st.saturation:7.1f}{st.cast:+7.1f}{st.brightness:8.1f}  '
              f'{",".join(map(str, names))[:40]}')
    print()
    return 0


if __name__ == '__main__':
    sys.exit(main())
