#!/usr/bin/env python3
"""Colour in the detector, or read after it? Scores both arms one way.

Arm A is a detector trained with colour in its classes (pool_holdout.py
--classes fine). Arm B is the coarse gate/flare/drum detector, and each flare
or drum box is given a colour by a hue reader. The reader is learned, not
tuned: its centroids are the mean hue-saturation histograms of the fine TRAIN
split's own boxes, so it sees no held-out frame and no constant is chosen by
hand. Both arms' boxes are scored by the same AP50 routine against the fine
labels of the same held-out images. The bar is in
mongla_data/holdout/colour_ab/BAR.md, written before either model trained.

    python3 tools/studies/colour_ab.py --self-test
    python3 tools/studies/colour_ab.py --root $MONGLA_DATA/holdout/colour_ab

The self-test is the falsifier for the scorer: perfect boxes must give AP 1.0
and boxes with the right place and the wrong colour must give 0 for the
colours they claim, or a bug here would rank the arms.
"""
from __future__ import annotations

import argparse
import json
import pathlib
import sys

import numpy as np

SHARED = ('gate', 'flare_red', 'flare_yellow', 'flare_blue', 'flare_orange',
          'drum_red', 'drum_blue')
HUE_BINS, SAT_BINS = 18, 4
CONF_FLOOR = 0.001          # the AP curve needs the whole ranking


def iou(a, b) -> float:
    """Boxes as (x1, y1, x2, y2)."""
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    u = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / u if u > 0 else 0.0


def ap50(gt: dict, preds: list, cls: str) -> tuple:
    """AP at IoU 0.5 for one class, all-point interpolation.

    gt: {image: [(cls, box), ...]}; preds: [(image, cls, conf, box)].
    Returns (ap, n_gt); ap is NaN when the class has no ground truth.
    """
    g = {im: [b for c, b in boxes if c == cls] for im, boxes in gt.items()}
    n_gt = sum(len(v) for v in g.values())
    if n_gt == 0:
        return float('nan'), 0
    used = {im: [False] * len(v) for im, v in g.items()}
    p = sorted((x for x in preds if x[1] == cls), key=lambda x: -x[2])
    tp = np.zeros(len(p))
    for i, (im, _, _, box) in enumerate(p):
        best, j = 0.0, -1
        for k, gb in enumerate(g.get(im, ())):
            o = iou(box, gb)
            if o > best:
                best, j = o, k
        if best >= 0.5 and not used[im][j]:
            used[im][j] = True
            tp[i] = 1
    if not len(p):
        return 0.0, n_gt
    ctp = np.cumsum(tp)
    rec = ctp / n_gt
    prec = ctp / np.arange(1, len(p) + 1)
    mrec = np.concatenate(([0.0], rec, [1.0]))
    mpre = np.concatenate(([1.0], prec, [0.0]))
    for i in range(len(mpre) - 2, -1, -1):
        mpre[i] = max(mpre[i], mpre[i + 1])
    idx = np.where(mrec[1:] != mrec[:-1])[0]
    return float(np.sum((mrec[idx + 1] - mrec[idx]) * mpre[idx + 1])), n_gt


def hue_hist(img, box) -> np.ndarray | None:
    """Hue x saturation histogram of a box's middle half, weighted by value.

    The middle half because a flare is a thin pole and its box is mostly
    water; the edges are the background the box was drawn around.
    """
    import cv2
    x1, y1, x2, y2 = (int(round(v)) for v in box)
    w, h = x2 - x1, y2 - y1
    x1, x2 = x1 + w // 4, x2 - w // 4
    y1, y2 = y1 + h // 4, y2 - h // 4
    crop = img[max(0, y1):max(0, y2), max(0, x1):max(0, x2)]
    if crop.size == 0:
        return None
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV).reshape(-1, 3).astype(np.float32)
    hb = np.minimum((hsv[:, 0] / 180.0 * HUE_BINS).astype(int), HUE_BINS - 1)
    sb = np.minimum((hsv[:, 1] / 256.0 * SAT_BINS).astype(int), SAT_BINS - 1)
    hist = np.zeros(HUE_BINS * SAT_BINS)
    np.add.at(hist, hb * SAT_BINS + sb, hsv[:, 2] + 1.0)
    s = hist.sum()
    return hist / s if s > 0 else None


class HueReader:
    """Nearest centroid in histogram space, per coarse kind (flare, drum)."""

    def __init__(self):
        self.centroids: dict = {}

    def fit(self, samples):
        """samples: [(fine class, hist)]."""
        acc: dict = {}
        for c, h in samples:
            acc.setdefault(c, []).append(h)
        self.centroids = {c: np.mean(v, axis=0) for c, v in acc.items()}
        return self

    def read(self, kind: str, hist) -> str | None:
        best, out = None, None
        for c, m in self.centroids.items():
            if not c.startswith(kind + '_') or hist is None:
                continue
            d = float(np.abs(hist - m).sum())      # L1 between distributions
            if best is None or d < best:
                best, out = d, c
        return out


def yolo_boxes(label: pathlib.Path, names, w, h):
    out = []
    if label.exists():
        for line in label.read_text().splitlines():
            p = line.split()
            if len(p) != 5:
                continue
            c, cx, cy, bw, bh = int(p[0]), *map(float, p[1:])
            out.append((names[c], (
                (cx - bw / 2) * w, (cy - bh / 2) * h, (cx + bw / 2) * w, (cy + bh / 2) * h)))
    return out


def _self_test() -> int:
    ok = True

    def check(cond, what):
        nonlocal ok
        print(('ok   ' if cond else 'FAIL ') + what)
        ok &= bool(cond)

    gt = {'a': [('flare_red', (0, 0, 10, 10)), ('drum_blue', (20, 20, 40, 40))],
          'b': [('flare_red', (5, 5, 15, 15))]}
    perfect = [(im, c, 0.9, b) for im, bs in gt.items() for c, b in bs]
    check(ap50(gt, perfect, 'flare_red')[0] == 1.0 and ap50(gt, perfect, 'drum_blue')[0] == 1.0,
          'perfect boxes give AP 1.0')
    swapped = [(im, 'flare_yellow' if c == 'flare_red' else c, 0.9, b) for im, c, _, b in perfect]
    check(ap50(gt, swapped, 'flare_red')[0] == 0.0 and ap50(gt, swapped, 'flare_yellow')[0] != 1.0,
          'right place, wrong colour scores 0 for the colour it should have been')
    half = [x for x in perfect if x[0] == 'a']
    check(abs(ap50(gt, half, 'flare_red')[0] - 0.5) < 1e-9, 'half the boxes found gives AP 0.5')
    dup = perfect + [('a', 'flare_red', 0.95, (0, 0, 10, 10))]
    check(ap50(gt, dup, 'flare_red')[0] < 1.0, 'a duplicate box is a false positive')

    red = np.zeros(HUE_BINS * SAT_BINS)
    red[0 * SAT_BINS + 3] = 1.0
    blue = np.zeros(HUE_BINS * SAT_BINS)
    blue[12 * SAT_BINS + 3] = 1.0
    r = HueReader().fit([('drum_red', red), ('drum_blue', blue), ('flare_blue', blue)])
    check(r.read('drum', red * 0.9 + blue * 0.1) == 'drum_red'
          and r.read('flare', red) == 'flare_blue',
          'the reader picks the nearest centroid, only among its own kind')
    print('self-test', 'PASSED' if ok else 'FAILED')
    return 0 if ok else 1


def run(root: pathlib.Path, conf_neg: float = 0.25) -> dict:
    import cv2
    import yaml
    from ultralytics import YOLO

    fine_dir = root / 'fine'
    fnames = yaml.safe_load((fine_dir / 'data.yaml').read_text())['names']
    fnames = [fnames[k] for k in sorted(fnames)] if isinstance(fnames, dict) else fnames

    # The reader learns from the fine TRAIN split only.
    samples = []
    rng = np.random.default_rng(0)
    labels = sorted((fine_dir / 'train' / 'labels').glob('*.txt'))
    for lab in (labels[i] for i in rng.permutation(len(labels))[:4000]):
        img_path = next((fine_dir / 'train' / 'images').glob(lab.stem + '.*'), None)
        im = cv2.imread(str(img_path)) if img_path else None
        if im is None:
            continue
        for c, b in yolo_boxes(lab, fnames, im.shape[1], im.shape[0]):
            if c.startswith(('flare_', 'drum_')):
                h = hue_hist(im, b)
                if h is not None:
                    samples.append((c, h))
    reader = HueReader().fit(samples)

    test_imgs = sorted((fine_dir / 'test' / 'images').iterdir())
    gt, frames = {}, {}
    for p in test_imgs:
        im = cv2.imread(str(p))
        if im is None:
            raise SystemExit(f'unreadable held-out image {p}: both arms must see the same set')
        frames[p.name] = im
        gt[p.name] = yolo_boxes(fine_dir / 'test' / 'labels' / (p.stem + '.txt'),
                                fnames, im.shape[1], im.shape[0])

    def predict(weights):
        m = YOLO(str(weights))
        out = []
        for name, im in frames.items():
            r = m.predict(im, conf=CONF_FLOOR, verbose=False)[0]
            for b, c, s in zip(r.boxes.xyxy.tolist(), r.boxes.cls.tolist(), r.boxes.conf.tolist()):
                out.append((name, m.names[int(c)], float(s), tuple(b)))
        return out, m

    a, ma = predict(root / 'runs/fine/weights/best.pt')
    raw_b, mb = predict(root / 'runs/coarse/weights/best.pt')
    b = []
    for name, c, s, box in raw_b:
        if c in ('flare', 'drum'):
            col = reader.read(c, hue_hist(frames[name], box))
            if col:
                b.append((name, col, s, box))
        else:
            b.append((name, c, s, box))

    res = {'n_test_images': len(frames), 'reader_samples': len(samples), 'per_class': {}}
    for cls in SHARED + ('ball',):
        ap_a, n = ap50(gt, a, cls)
        ap_b = ap50(gt, b, cls)[0] if cls in SHARED else float('nan')
        res['per_class'][cls] = {'n_gt': n, 'A_fine': ap_a, 'B_coarse_hue': ap_b}
    for arm in ('A_fine', 'B_coarse_hue'):
        v = [res['per_class'][c][arm] for c in SHARED if res['per_class'][c]['n_gt']]
        res[f'mean_{arm}'] = float(np.mean(v))

    # Secondary: detections per empty-water frame at the vehicle's confidence.
    negs = sorted(p for p in pathlib.Path(root).parents[1].joinpath('negatives').rglob('*')
                  if p.suffix.lower() in ('.jpg', '.jpeg', '.png'))
    if negs:
        sample = [negs[i] for i in np.linspace(0, len(negs) - 1, min(500, len(negs))).astype(int)]
        for arm, m in (('A_fine', ma), ('B_coarse_hue', mb)):
            n = sum(len(m.predict(str(p), conf=conf_neg, verbose=False)[0].boxes) for p in sample)
            res[f'negatives_per_frame_{arm}'] = n / len(sample)
        res['negative_frames'] = len(sample)
    return res


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--root', help='mongla_data/holdout/colour_ab')
    ap.add_argument('--self-test', action='store_true')
    a = ap.parse_args(argv)
    if a.self_test:
        return _self_test()
    if not a.root:
        ap.error('--root is required')
    res = run(pathlib.Path(a.root))
    print(json.dumps(res, indent=1))
    (pathlib.Path(a.root) / 'result.json').write_text(json.dumps(res, indent=1))
    return 0


if __name__ == '__main__':
    sys.exit(main())
