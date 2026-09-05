#!/usr/bin/env python3
"""Export XFeat to ONNX for the vehicle. Run on a box WITH torch; the Pi has none.

    python3 tools/xfeat_export.py --src <accelerated_features> --out ~/hailo_models

WHY ONNX AND NOT TORCH ON THE PI: the vehicle has no torch and adding it for one
model is a large dependency for a 2.7 MB network. onnxruntime has a prebuilt
aarch64 wheel and measured, on the Pi with the vision stack running:

    640x480   145.6 ms (1 thread)  /  88.0 ms (3)
    320x240    33.1 ms (1 thread)  /  18.2 ms (3)   <- 30 Hz on ONE core

WHY 320x240 IS THE DEFAULT: it is the size the anchor can afford at rate, and it
was verified to KEEP the lock -- every murky archive clip still reaches a trusted
homography at 320x240 (56-260 inliers). Downscaling is the cheapest possible
speedup and also the easiest way to silently destroy the thing being measured,
so it was measured rather than assumed.

THE EXPORT IS FIXED-SHAPE. Feeding another size throws rather than resizing,
which is deliberate: a silent resize would change the pixel scale of every pose
the anchor reports.
"""
import argparse
import os
import sys


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--src', required=True,
                    help='path to a checkout of verlab/accelerated_features')
    ap.add_argument('--out', default=os.path.expanduser('~/hailo_models'))
    ap.add_argument('--sizes', default='320x240,640x480')
    a = ap.parse_args()

    sys.path.insert(0, a.src)
    import torch
    from modules.xfeat import XFeat

    w = os.path.join(a.src, 'weights', 'xfeat.pt')
    net = XFeat(weights=w, top_k=4096).net.eval().cpu()
    os.makedirs(a.out, exist_ok=True)

    for spec in a.sizes.split(','):
        W, H = (int(v) for v in spec.lower().split('x'))
        dst = os.path.join(a.out, f'xfeat_{W}x{H}.onnx')
        torch.onnx.export(
            net, torch.randn(1, 1, H, W), dst, opset_version=16,
            input_names=['image'],
            output_names=['feats', 'keypoints', 'heatmap'], dynamo=False)
        print(f'  {dst}  ({os.path.getsize(dst) / 1e6:.1f} MB)')
    print('\n  Copy to the vehicle\'s model dir; the anchor loads it by name.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
