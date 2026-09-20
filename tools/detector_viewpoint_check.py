"""Score a detector on a viewpoint it was not trained for -- honestly.

Built for the return leg (does the gate detector see the gate from BEHIND?)
and kept general, because the trap it exists to avoid is not specific to
that question.

⛔ ALWAYS SCORE THE TRIVIAL MODEL. When ground-truth boxes are a large
fraction of the frame -- 41.9 % on the return set -- a single box covering
the whole image clears IoU 0.3 on EVERY target. Measured: the null model
scores 100 % at IoU 0.3, and the shipping detector scored 73.6 % there and
looked like it worked. At IoU 0.5 the null scores 15.3 % and the detector
9.7 %: worse than guessing "everything".

So this prints the null row FIRST and at three thresholds, because a
threshold at which the null already wins cannot rank anything. Also splits
class-agnostic ("did a box land on it") from class-specific ("did the model
call it a gate"), which on the return set differ by 73.6 points.

Usage: point IMGDIR/LABDIR at a YOLO-format set and MP at a .hef.
"""
import argparse
import glob
import os
import sys

import cv2

# The vehicle's source tree, so this runs from anywhere on the Pi. Import
# the BACKEND MODULE, never the package -- `mongla_vision/__init__` imports
# `preflight`, which imports `rclpy`, and this tool must run without ROS.
sys.path.insert(0, os.path.expanduser("~/mongla_ws/src/mongla_vision"))
from mongla_vision.detection.hailo import HailoDetector


def load(lp):
    out = []
    for ln in open(lp):
        p = ln.split()
        if len(p) >= 5:
            _, x, y, w, h = [float(v) for v in p[:5]]
            out.append((x - w/2, y - h/2, x + w/2, y + h/2))
    return out


def iou(a, b):
    ix = max(0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0, min(a[3], b[3]) - max(a[1], b[1]))
    i = ix * iy
    u = (a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1]) - i
    return i / u if u > 0 else 0.0


ap = argparse.ArgumentParser()
ap.add_argument('--model', required=True, help='path to a .hef')
ap.add_argument('--images', required=True)
ap.add_argument('--labels', required=True)
ap.add_argument('--klass', default=None,
                help='the class the target SHOULD be called; the '
                     'class-specific rows measure only this one')
A = ap.parse_args()

# conf 0.02, deliberately: the HEF bakes its own floor and a runtime conf
# can only tighten, so start under every threshold the sweep reports.
d = HailoDetector(model_path=A.model, conf=0.02, class_allowlist=None)
res = []
areas = []
for f in sorted(glob.glob(os.path.join(A.images, "*"))):
    im = cv2.imread(f)
    if im is None:
        continue
    H, W = im.shape[:2]
    stem = os.path.splitext(os.path.basename(f))[0]
    lp = os.path.join(A.labels, stem + ".txt")
    if not os.path.exists(lp):
        continue
    gt = load(lp)
    areas += [(g[2]-g[0])*(g[3]-g[1]) for g in gt]
    res.append((d.infer(im), gt, W, H))
print("frames %d   mean GT box area = %.1f%% of frame"
      % (len(res), 100*sum(areas)/max(len(areas), 1)), flush=True)

# the null model: one box covering the whole frame, every frame
for thr in (0.3, 0.5, 0.7):
    hit = tot = 0
    for _, gt, W, H in res:
        for g in gt:
            tot += 1
            hit += iou(g, (0.0, 0.0, 1.0, 1.0)) >= thr
    print("  FULL-FRAME null   IoU>=%.1f  R=%5.1f%%" % (thr, 100*hit/max(tot, 1)),
          flush=True)

print(flush=True)
print("  %-6s %-5s %6s %7s %7s %7s  %s"
      % ("conf", "cls", "pred", "R@.3", "R@.5", "R@.7", "by class"), flush=True)
for cf in (0.05, 0.10, 0.15, 0.25, 0.35):
    for merge in (True, False):
        r = {}
        npred = 0
        percls = {}
        for thr in (0.3, 0.5, 0.7):
            hit = tot = 0
            np_ = 0
            for dets, gt, W, H in res:
                bs = []
                for x in dets:
                    if x.score < cf:
                        continue
                    if not merge and A.klass and x.class_name != A.klass:
                        continue
                    percls[x.class_name] = percls.get(x.class_name, 0) + 1
                    b = x.xyxy
                    bs.append((b[0]/W, b[1]/H, b[2]/W, b[3]/H))
                np_ += len(bs)
                for g in gt:
                    tot += 1
                    hit += max([iou(g, b) for b in bs], default=0.0) >= thr
            r[thr] = 100*hit/max(tot, 1)
            npred = np_
        # percls counted 3x (once per threshold loop)
        percls = {k: v//3 for k, v in percls.items()}
        print("  %-6.2f %-5s %6d %6.1f%% %6.1f%% %6.1f%%  %s"
              % (cf, "any" if merge else (A.klass or "cls"), npred,
                 r[0.3], r[0.5], r[0.7], percls), flush=True)
