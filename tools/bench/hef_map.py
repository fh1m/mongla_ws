"""mAP50 of a compiled HEF on a labelled YOLO split, run on the real Hailo.

A compile that succeeded proves nothing about accuracy after INT8. This scores
the HEF itself, on the chip, against the labels -- a truth test, not agreement
with the .pt. Run it on Kutum after every compile a model swap depends on:

    python3 tools/bench/hef_map.py --self-test
    python3 tools/bench/hef_map.py --hef <stem>.hef --split <dataset>/test

The <stem>.yaml sidecar must sit beside the HEF (the class names). Scoring is
COCO 101-point AP at IoU 0.5, so a perfect prediction reads 0.995, as in
ultralytics. --self-test is the falsifier: labels fed back must read >= 0.99
and boxes shifted one width below 0.5. First used for measured-bars 98.
"""
import argparse
import glob
import os
import sys

import numpy as np


def iou(a, b):
    x1, y1 = max(a[0], b[0]), max(a[1], b[1])
    x2, y2 = min(a[2], b[2]), min(a[3], b[3])
    inter = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    u = (a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1]) - inter
    return inter / u if u > 0 else 0.0


def ap(rec, prec):
    mrec = np.concatenate(([0.], rec, [1.]))
    mpre = np.concatenate(([1.], prec, [0.]))
    for i in range(len(mpre) - 2, -1, -1):
        mpre[i] = max(mpre[i], mpre[i + 1])
    x = np.linspace(0, 1, 101)
    return float(np.trapz(np.interp(x, mrec, mpre), x))


def map50(gts, preds, ncls):
    """gts: {img: [(cls, xyxy)]}; preds: {img: [(cls, score, xyxy)]}. COCO-style 101-pt."""
    aps = {}
    for c in range(ncls):
        n_gt = sum(1 for g in gts.values() for k, _ in g if k == c)
        if n_gt == 0:
            continue
        dets = sorted(((s, img, b) for img, p in preds.items() for k, s, b in p if k == c),
                      key=lambda t: -t[0])
        used = {img: [False] * len(g) for img, g in gts.items()}
        tp = []
        for s, img, b in dets:
            best, bi = 0.5, -1
            for j, (k, gb) in enumerate(gts.get(img, [])):
                if k == c and not used[img][j]:
                    v = iou(b, gb)
                    if v >= best:
                        best, bi = v, j
            if bi >= 0:
                used[img][bi] = True
            tp.append(bi >= 0)
        tp = np.array(tp, float)
        fp = 1 - tp
        ctp, cfp = np.cumsum(tp), np.cumsum(fp)
        rec = ctp / n_gt if len(tp) else np.array([0.])
        prec = ctp / np.maximum(ctp + cfp, 1e-9) if len(tp) else np.array([0.])
        aps[c] = ap(rec, prec)
    return float(np.mean(list(aps.values()))), aps


def load_gt(split, w_h, seen=lambda s: True):
    gts = {}
    for lbl in sorted(glob.glob(os.path.join(split, 'labels', '*.txt'))):
        stem = os.path.splitext(os.path.basename(lbl))[0]
        if not seen(stem):
            continue
        w, h = w_h(stem)
        rows = []
        for ln in open(lbl):
            p = ln.split()
            if len(p) < 5:
                continue
            c, cx, cy, bw, bh = int(p[0]), *map(float, p[1:5])
            rows.append((c, ((cx-bw/2)*w, (cy-bh/2)*h, (cx+bw/2)*w, (cy+bh/2)*h)))
        gts[stem] = rows
    return gts


def self_test():
    gts = {'a': [(0, (10, 10, 50, 50)), (1, (60, 60, 90, 90))], 'b': [(0, (0, 0, 20, 30))]}
    same = {k: [(c, 0.9, b) for c, b in v] for k, v in gts.items()}
    m1, _ = map50(gts, same, 2)
    shifted = {k: [(c, 0.9, (b[0]+(b[2]-b[0]), b[1], b[2]+(b[2]-b[0]), b[3])) for c, b in v]
               for k, v in gts.items()}
    m2, _ = map50(gts, shifted, 2)
    print(f'self-test: exact {m1:.3f} (want >= 0.99: 101-point AP tops out at 0.995), '
          f'shifted {m2:.3f} (want < 0.5)')
    return m1 >= 0.99 and m2 < 0.5


if __name__ == '__main__':
    ap_ = argparse.ArgumentParser()
    ap_.add_argument('--self-test', action='store_true')
    ap_.add_argument('--hef')
    ap_.add_argument('--split')
    ap_.add_argument('--ncls', type=int, default=5)
    a = ap_.parse_args()
    if a.self_test:
        sys.exit(0 if self_test() else 1)
    import cv2
    from mongla_vision.detection.hailo import HailoDetector
    det = HailoDetector(model_path=a.hef, conf=0.001, max_det=100)
    imgs = {os.path.splitext(os.path.basename(p))[0]: p
            for p in glob.glob(os.path.join(a.split, 'images', '*'))}
    frames = {k: f for k, f in ((k, cv2.imread(p)) for k, p in imgs.items()) if f is not None}
    gts = load_gt(a.split, lambda s: (frames[s].shape[1], frames[s].shape[0]),
                  seen=lambda s: s in frames)
    print(f'{len(imgs) - len(frames)} unreadable image(s) skipped')
    preds = {k: [(d.class_id, d.score, d.xyxy) for d in det.infer(f)] for k, f in frames.items()}
    m, aps = map50(gts, preds, a.ncls)
    print(f'{os.path.basename(a.hef)} {os.path.basename(a.split.rstrip("/"))}: mAP50 {m:.3f} '
          f'per class {{{", ".join(f"{c}: {v:.3f}" for c, v in aps.items())}}} on {len(frames)} images')
