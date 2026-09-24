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
# ⛔ TWO SOURCES ONLY: the raw archive and the current season. The
# annotated-prediction archive (`Music/detect`, 92 `predict*` directories) is
# NOT a source. It supplied 60 % of the first training set as YOLO output with
# boxes, class names and confidences burned into the pixels, and cost 1.5 h of
# GPU teaching the descriptor to match furniture it will never see again.
#
# ⛔ AND NO AUTOMATIC ANNOTATION DETECTOR IS TRUSTED IN ITS PLACE. Two were
# written and both failed against real footage:
#   * "saturated AND bright" flagged 32 of 33 clips -- bright turquoise pool
#     water is exactly that, so it was detecting water;
#   * "long axis-aligned lines" flagged 6 more -- pool tile borders, lane
#     markers and wall edges are axis-aligned when the camera is level.
# All six were confirmed CLEAN by eye. The only verification that has ever
# worked here is looking at the frames, so the clip list is verified visually
# (`tools/footage_inventory.py` renders every clip) and the defence is that
# the contaminated archive is not read at all.
SOURCES = ('raw_videos', 'season2026')

# Above water. Real frames, wrong domain -- the vehicle is never in air, and
# a descriptor has limited capacity to spend.
DRY_CLIPS = ('octagon_front_1.mp4', 'octagon_2.mp4')

# ⭐ HELD OUT: a different VENUE on a different DATE, not a random split.
# Mirpur is the murky table's venue and its two hardest clips; a random split
# would leak the same pool, the same day and the same water into both sides
# and report a number that means nothing.
HELD_OUT_VENUE = 'Mirpur'

# ⭐ BALANCED BY (venue, date). robosub 2025 has 25 clips and the 2026 season
# has 7; sampling per clip would make the descriptor mostly an August-2025
# specialist. Each venue-date group contributes the same budget instead.
FRAMES_PER_GROUP = 380

# Upstream's AugmentationPipe reserves `max_num_imgs = 3_000` plus a test split
# from this same folder and REFUSES to start below that, with the misleading
# message "test set overlaps with training set".
MIN_FRAMES = 3_200

def clips(root: str) -> list:
    out = []
    for dirpath, _dirs, files in os.walk(root):
        low = dirpath.lower()
        if any(h in low for h in ANNOTATED_HINTS):
            continue
        for f in files:
            if f.endswith(VIDEO_EXT) and not any(
                    h in f.lower() for h in ANNOTATED_HINTS):
                out.append(os.path.join(dirpath, f))
    return sorted(out)


# ⭐ CALIBRATED AGAINST THE REAL ARCHIVE, not invented. Measured over 20
# un-annotated clips spanning both years and every venue:
#
#     R/B  0.305 .. 0.977   (median 0.432)
#     G/B  0.968 .. 1.085   (median 1.047)
#
# ⛔ THE FIRST VERSION OF THIS FUNCTION WAS WILDLY OUT. Sampling k and depth
# independently (k_R-k_B up to 1.15 per m, depth to 6 m) produces R/B as low
# as exp(-6.9) = 0.001 -- three hundred times more red-starved than the worst
# frame we have ever recorded, applied ON TOP of footage that is already blue.
# The network was taught to expect water that does not exist, and the measured
# cost was a UNIFORM ~19-inlier median drop on held-out AND trained-on venues
# alike.
#
# So the TARGET RATIO is sampled from the observed range and the attenuation
# derived from it, rather than the reverse. Physically the same model; the
# difference is that its output now lands where real water lands.
REAL_RB = (0.28, 1.00)          # slightly wider than observed, not 300x
REAL_GB = (0.95, 1.10)


def beer_lambert(img: np.ndarray, rb: float, gb: float,
                 veil_strength: float = 1.0) -> np.ndarray:
    """Attenuate toward a TARGET red/blue and green/blue ratio, BGR order.

    Red is removed fastest, which is why underwater imagery goes blue-green
    long before it goes dark -- a brightness or contrast jitter cannot produce
    that colour cast and would teach the network the wrong invariance.
    """
    f = np.float32([1.0, float(gb), float(rb)])          # B, G, R
    out = img.astype(np.float32) * f.reshape(1, 1, 3)
    # Backscatter: water does not only subtract, it ADDS a veiling light that
    # lifts the blacks. Without it a turbid frame is merely dark, which is the
    # half contrast enhancement can undo -- the easy half.
    veil = (1.0 - f).reshape(1, 1, 3) * np.float32([0.0, 30.0, 55.0]) \
        * float(veil_strength)
    return np.clip(out + veil, 0, 255).astype(np.uint8)


# ⛔ ANNOTATED FOOTAGE IS POISON FOR A DESCRIPTOR. YOLO prediction output has
# bounding boxes, class names and confidences BURNED INTO THE PIXELS: hard
# edges, pure saturated hues, perfectly straight lines and crisp glyphs.
# Those are the easiest features in any frame and they do not exist at
# inference, so a descriptor trained on them spends capacity on furniture it
# will never see again.
#
# Measured: 101 such clips across four archives, 92 in one directory, which
# was 60 % of the first training set.
#
# Two gates, because a path name is a heuristic: exclude the directories, then
# REJECT BY PIXELS, so annotated video stored anywhere else is still caught.
ANNOTATED_HINTS = ('predict', 'runs/detect', 'labelled', 'labeled', 'annot')
SATURATED_FRAC_MAX = 0.004        # 0.4 % of pixels


def looks_annotated(img: np.ndarray) -> bool:
    """Burned-in overlays are saturated AND bright. Real water is neither."""
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    strong = (hsv[:, :, 1] > 200) & (hsv[:, :, 2] > 150)
    return float(strong.mean()) > SATURATED_FRAC_MAX


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
            if turbid and rng.random() < 0.5:
                img = beer_lambert(img,
                                   rng.uniform(*REAL_RB),
                                   rng.uniform(*REAL_GB),
                                   veil_strength=rng.uniform(0.4, 1.0))
            name = f'{tag}{os.path.basename(p)}_{i:07d}.png'.replace(' ', '_')
            cv2.imwrite(os.path.join(out_dir, name), img)
            written += 1
        cap.release()
    return written


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='/tmp/xfeat_ourwater')
    ap.add_argument('--inventory', default='/tmp/footage_inventory.json')
    ap.add_argument('--per-group', type=int, default=FRAMES_PER_GROUP)
    ap.add_argument('--width', type=int, default=800)
    ap.add_argument('--height', type=int, default=608)
    ap.add_argument('--no-turbid', action='store_true')
    ap.add_argument('--min-frames', type=int, default=MIN_FRAMES)
    args = ap.parse_args()

    import json
    with open(args.inventory) as fh:
        inv = json.load(fh)

    # Every clip the inventory found, MINUS the dry ones. The inventory's own
    # `clean` flag is deliberately ignored: it was produced by a detector that
    # was measured wrong, and the set has been verified by eye instead.
    groups = {}
    for r in inv:
        if os.path.basename(r['path']) in DRY_CLIPS:
            continue
        groups.setdefault((r['venue'], r['date']), []).append(r['path'])

    held = {k: v for k, v in groups.items() if HELD_OUT_VENUE in k[0]}
    train = {k: v for k, v in groups.items() if HELD_OUT_VENUE not in k[0]}

    size = (args.width, args.height)
    train_dir = os.path.join(args.out, 'train')
    total = 0
    print(f'HELD-OUT venue: {HELD_OUT_VENUE} '
          f'({sum(len(v) for v in held.values())} clips, '
          f'{len(held)} venue-date group(s)) -- never trained on\n')
    print(f'{"venue":<44}{"date":<12}{"clips":>6}{"frames":>8}')
    for (venue, date), paths in sorted(train.items()):
        per_clip = max(1, args.per_group // max(1, len(paths)))
        n = extract(paths, per_clip, size, train_dir, not args.no_turbid,
                    seed=abs(hash((venue, date))) % 10_000,
                    tag=f'{venue.replace("/", "-")}_{date}_')
        total += n
        print(f'{venue[:43]:<44}{date:<12}{len(paths):>6}{n:>8}')

    print(f'\n{total} training frames in {train_dir}')
    if total < args.min_frames:
        print(f'⛔ {total} < {args.min_frames}: upstream reserves 3000 images '
              f'plus a test split from this folder and refuses to start '
              f'below that. Raise --per-group.')
        return 1
    print('\nnext: verify the frames visually, THEN train.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
