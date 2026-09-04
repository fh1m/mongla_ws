#!/usr/bin/env python3
"""REAL RECALL -- against ground truth labels, per water regime.

Everything measured before this was PRESENCE: "did a box appear". That
cannot distinguish a hit from a false positive, and it cannot see a miss on a
frame where something else was detected. With 10,188 labelled images in the
archive the honest quantity is available, and this measures it:

    recall     labelled objects the detector found (IoU >= thresh)
    precision  boxes it emitted that were real

Both per DATASET, so the answer is per water condition rather than averaged
into meaninglessness across two different venues.

The point is generalisation. A model that scores 0.995 mAP on its own
validation split and collapses on another pool has learned the pool. Running
every model against every dataset shows which -- and the diagonal (model on
its own data) versus the off-diagonal (model on someone else's water) is the
number that predicts a competition we have never seen.

    python3 tools/recall_matrix.py --model M.pt --data DIR --conf 0.15
    python3 tools/recall_matrix.py --matrix models.txt datasets.txt
"""
import argparse
import glob
import os
import random
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..',
                                'src', 'duburi_vision'))

_IMG = ('.png', '.jpg', '.jpeg', '.bmp', '.webp')


def _iou(a, b):
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    ua = (ax2 - ax1) * (ay2 - ay1) + (bx2 - bx1) * (by2 - by1) - inter
    return inter / ua if ua > 0 else 0.0


def _labels(path, w, h):
    """YOLO txt -> pixel xyxy. Class id is deliberately IGNORED.

    Class names differ between datasets for the same physical object --
    `shark`, `bin_shark`, `shark_torpedo` are all the same prop. Matching on
    the id would report a cross-dataset miss that is really a naming
    difference, which would make every off-diagonal look worse than it is and
    lead us to conclude models do not transfer when the labels simply
    disagree. Localisation is the question here.
    """
    out = []
    if not os.path.exists(path):
        return out
    for line in open(path):
        p = line.split()
        if len(p) < 5:
            continue
        vals = [float(v) for v in p[1:]]
        if len(vals) >= 8:
            # ORIENTED BOXES (YOLO-OBB): 4 corner points, not xywh. Parsing
            # them as xywh does not raise -- it yields a plausible-looking
            # box, sometimes with NEGATIVE area, and reports the model as
            # having 3.6 % recall when it really has ~69 %. Found because a
            # dataset with the same classes and nearly identical water scored
            # 20x worse than its neighbour, which is not something a model
            # does. Take the axis-aligned hull.
            xs = [vals[i] for i in range(0, 8, 2)]
            ys = [vals[i] for i in range(1, 8, 2)]
            out.append((min(xs) * w, min(ys) * h, max(xs) * w, max(ys) * h))
        else:
            cx, cy, bw, bh = vals[:4]
            out.append(((cx - bw / 2) * w, (cy - bh / 2) * h,
                        (cx + bw / 2) * w, (cy + bh / 2) * h))
    return out


def _pairs(root, k, seed=11):
    files = []
    for split in ('train', 'valid', 'val', 'test'):
        d = os.path.join(root, split, 'images')
        if os.path.isdir(d):
            files += [os.path.join(d, f) for f in os.listdir(d)
                      if f.lower().endswith(_IMG)]
    files.sort()
    rng = random.Random(seed)
    files = rng.sample(files, min(k, len(files)))
    out = []
    for f in files:
        lp = f.replace('/images/', '/labels/')
        lp = os.path.splitext(lp)[0] + '.txt'
        if os.path.exists(lp):
            out.append((f, lp))
    return out


def evaluate(model, root, conf, imgsz, k, iou_thresh=0.3, preprocess=None):
    import cv2
    pairs = _pairs(root, k)
    tp = fp = fn = 0
    scores = []
    for img_p, lbl_p in pairs:
        img = cv2.imread(img_p)
        if img is None:
            continue
        h, w = img.shape[:2]
        gts = _labels(lbl_p, w, h)
        src = preprocess(img) if preprocess else img
        r = model.predict(src, conf=conf, imgsz=imgsz, verbose=False)[0]
        dets = [tuple(float(v) for v in b.xyxy[0]) for b in r.boxes]
        scores += [float(b.conf[0]) for b in r.boxes]
        used = set()
        for g in gts:
            best, bi = 0.0, -1
            for i, d in enumerate(dets):
                if i in used:
                    continue
                v = _iou(g, d)
                if v > best:
                    best, bi = v, i
            if best >= iou_thresh:
                tp += 1
                used.add(bi)
            else:
                fn += 1
        fp += len(dets) - len(used)
    rec = tp / (tp + fn) if (tp + fn) else float('nan')
    prec = tp / (tp + fp) if (tp + fp) else float('nan')
    return dict(recall=rec, precision=prec, tp=tp, fp=fp, fn=fn,
                n=len(pairs), scores=scores)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model')
    ap.add_argument('--data')
    ap.add_argument('--matrix', nargs=2, metavar=('MODELS', 'DATASETS'))
    ap.add_argument('--conf', type=float, default=0.15)
    ap.add_argument('--imgsz', type=int, default=640)
    ap.add_argument('--samples', type=int, default=120)
    ap.add_argument('--clahe', action='store_true')
    a = ap.parse_args()

    from ultralytics import YOLO
    pre = None
    if a.clahe:
        from duburi_vision.detection.preprocess import make_clahe
        pre = make_clahe()

    if a.matrix:
        models = [l.strip() for l in open(a.matrix[0]) if l.strip()]
        datas = [l.strip() for l in open(a.matrix[1]) if l.strip()]
        print(f'\n  recall @ conf {a.conf}, IoU 0.3, {a.samples} imgs/set'
              f'{"  [CLAHE]" if a.clahe else ""}\n')
        hdr = "  " + " " * 30 + ''.join(f'{os.path.basename(d)[:11]:>12}'
                                        for d in datas)
        print(hdr)
        for mp in models:
            m = YOLO(mp)
            # The run name, not 'weights' -- every path ends in
            # weights/best.pt and the basename of its dirname is
            # therefore always the same string.
            tag = os.path.basename(os.path.dirname(os.path.dirname(mp)))
            row = f'  {tag[:29]:<30}'
            for d in datas:
                r = evaluate(m, d, a.conf, a.imgsz, a.samples, preprocess=pre)
                row += f'{100 * r["recall"]:11.1f}%'
            print(row, flush=True)
        print()
        return 0

    m = YOLO(a.model)
    r = evaluate(m, a.data, a.conf, a.imgsz, a.samples, preprocess=pre)
    print(f'\n  {os.path.basename(a.model)} on {os.path.basename(a.data)}'
          f'{"  [CLAHE]" if a.clahe else ""}')
    print(f'    {r["n"]} labelled images   tp {r["tp"]}  fp {r["fp"]}  '
          f'fn {r["fn"]}')
    print(f'    RECALL {100 * r["recall"]:.1f} %   '
          f'PRECISION {100 * r["precision"]:.1f} %\n')
    return 0


if __name__ == '__main__':
    sys.exit(main())
