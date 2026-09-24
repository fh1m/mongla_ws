#!/usr/bin/env python3
"""Fine-tune XFeat on OUR water. Round 7.

Every XFeat number this project has -- the 4/4 murky table, the ROOT-SIFT
control, the INT8 survival -- is STOCK weights trained on MegaDepth: clear,
natural, above-water imagery. We own gigabytes of the one domain it was never
trained on.

⭐ IT NEEDS NO LABELS. Upstream's `xfeat_synthetic` scheme builds training
pairs from a plain folder of images by warping them (homography + thin-plate
spline) and keeping the warp as exact correspondence ground truth. So the whole
input is: a folder of frames from our own archive.

⭐ AND THE SECOND HALF IS A PHYSICS MODEL, not more augmentation for its own
sake. Beer-Lambert attenuation -- per-channel exp(-k*d) -- is how water
actually removes light: red first, then green, blue last. Randomising k across
a plausible range generates the turbidity variation our archive does not
happen to contain.

⛔ AND THIS DOES NOT CONTRADICT THE PREPROCESSING BAN. We ban enhancement at
INFERENCE (17 configurations, 95 % of gate detections destroyed). Degrading
TRAINING data to match the test domain is the opposite operation: enhancement
pretends the water is clear, augmentation teaches the network it is not.

⛔ THE HELD-OUT VENUE IS NOT OPTIONAL. Our own detector recall swings
29.2 / 72.7 / 68.3 % across venues, so a descriptor fine-tuned on our archive
can overfit to OUR POOLS. Train on a venue subset, score on a venue never
trained on, and report THAT number -- a fine-tune scored on its training
venues would be the most flattering and least true number in the project.
"""
from __future__ import annotations

import argparse
import os
import random
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from archive_root import archive_root                        # noqa: E402

A = archive_root()

# Venues, kept apart on purpose. `mirpur` is the HELD-OUT venue: the murky
# table's two hardest clips come from it, so scoring there after training
# elsewhere is the honest test.
# ⭐ EVERY ARCHIVE WE OWN, because a descriptor's training set is the one
# place where more real domain data is unambiguously better. These are five
# separate physical archives (see the footage memo); a `find` rooted at the
# repo reaches none of them.
VENUES = {
    'robosub': f'{A}/robosub',
    'final_run': f'{A}/final_run',
    'detect': '/home/fh1m/Music/detect',
    # ⛔ DERIVED, not written down. These sit beside the archive root, whose
    # path carries the retired project name that a contract test bans from
    # live code -- and hardcoding them also assumed one machine. The root
    # already comes from $MONGLA_ARCHIVE or ~/.mongla/archive_root.
    'season_2026': os.path.join(os.path.dirname(A), '2026'),
    'this_year': os.path.join(os.path.dirname(A), 'this_year'),
    'pendrive_1': '/home/fh1m/tmp/smol backups/pendrive_1',
    'pendrive_2': '/home/fh1m/tmp/smol backups/pendrive_2',
    # ⛔ HELD OUT. The murky table's two hardest clips come from here, so
    # scoring on mirpur after training everywhere else is the honest test --
    # and the only one that can detect overfitting to our own pools.
    'mirpur': f'{A}/Mirpur',
}
HELD_OUT = 'mirpur'

# Upstream's AugmentationPipe reserves `max_num_imgs = 3_000` plus a small
# test split from this same folder and REFUSES to start below that, so a small
# extraction fails with "test set overlaps with training set". Aim well past
# it rather than shrinking the pipe: more real frames is the point.
MIN_FRAMES = 3_200

VIDEO_EXT = ('.mp4', '.mkv', '.avi', '.mov', '.MP4', '.MKV')


def clips(root: str) -> list:
    out = []
    for dirpath, _dirs, files in os.walk(root):
        for f in files:
            if f.endswith(VIDEO_EXT):
                out.append(os.path.join(dirpath, f))
    return sorted(out)


def beer_lambert(img: np.ndarray, k, depth_m: float) -> np.ndarray:
    """Per-channel attenuation, BGR order.

    `k` is the attenuation coefficient per metre for (B, G, R). Red is removed
    fastest, which is why underwater imagery goes blue-green long before it
    goes dark -- a plain brightness or contrast jitter cannot produce that and
    would teach the network the wrong invariance.
    """
    f = np.exp(-np.asarray(k, np.float32) * float(depth_m))
    out = img.astype(np.float32) * f.reshape(1, 1, 3)
    # Backscatter: water does not only subtract, it ADDS a veiling light that
    # lifts the blacks. Leaving it out makes turbid frames merely dark, which
    # is the part contrast enhancement can undo and therefore the easy half.
    veil = (1.0 - f).reshape(1, 1, 3) * np.float32([90.0, 75.0, 40.0])
    return np.clip(out + veil, 0, 255).astype(np.uint8)


def extract(paths, n_per_clip, size, out_dir, turbid, seed=0, tag=''):
    rng = random.Random(seed)
    os.makedirs(out_dir, exist_ok=True)
    written = 0
    for p in paths:
        cap = cv2.VideoCapture(p)
        total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
        if total <= 10:
            cap.release()
            continue
        idx = np.linspace(total * 0.05, total * 0.95, n_per_clip).astype(int)
        for i in idx:
            cap.set(cv2.CAP_PROP_POS_FRAMES, int(i))
            ok, img = cap.read()
            if not ok or img is None:
                continue
            img = cv2.resize(img, size, interpolation=cv2.INTER_AREA)
            if turbid and rng.random() < 0.6:
                # k for clear coastal water, roughly. Randomised WIDE, because
                # the point is a range the archive does not contain, not a
                # faithful single water.
                k = (rng.uniform(0.05, 0.25),      # B, attenuates least
                     rng.uniform(0.15, 0.45),      # G
                     rng.uniform(0.40, 1.20))      # R, attenuates most
                img = beer_lambert(img, k, rng.uniform(0.5, 6.0))
            name = f'{tag}{os.path.basename(p)}_{i:07d}.png'.replace(' ', '_')
            cv2.imwrite(os.path.join(out_dir, name), img)
            written += 1
        cap.release()
    return written


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='/tmp/xfeat_ourwater')
    ap.add_argument('--per-clip', type=int, default=40)
    ap.add_argument('--min-frames', type=int, default=MIN_FRAMES)
    ap.add_argument('--width', type=int, default=800)
    ap.add_argument('--height', type=int, default=608)
    ap.add_argument('--no-turbid', action='store_true')
    args = ap.parse_args()

    size = (args.width, args.height)
    train_dir = os.path.join(args.out, 'train')
    total = 0
    print(f'HELD-OUT venue (never trained on): {HELD_OUT}')
    for venue, root in VENUES.items():
        if venue == HELD_OUT:
            print(f'  {venue:<12} SKIPPED -- held out for scoring')
            continue
        if not os.path.isdir(root):
            print(f'  {venue:<12} missing at {root}')
            continue
        cs = clips(root)
        n = extract(cs, args.per_clip, size, train_dir,
                    not args.no_turbid, seed=len(venue), tag=f'{venue}_')
        total += n
        print(f'  {venue:<12} {len(cs):>3} clips -> {n:>5} frames')
    print(f'\n{total} training frames in {train_dir}')
    if total < args.min_frames:
        print(f'⛔ {total} < {args.min_frames}: upstream reserves 3000 images '
              f'plus a test split from this folder and refuses to start '
              f'below that. Raise --per-clip.')
        return 1
    print('\nnext:')
    print(f'  python3 /tmp/xfeat_src/modules/training/train.py \\')
    print(f'      --training_type xfeat_synthetic \\')
    print(f'      --synthetic_root_path {train_dir} \\')
    print(f'      --ckpt_save_path /tmp/xfeat_ft --batch_size 4 --n_steps 4000')
    return 0 if total else 1


if __name__ == '__main__':
    sys.exit(main())
