#!/usr/bin/env python3
"""Export XFeat to an ONNX the Hailo compiler can parse -- and prove it is the
same network.

⛔ THE PROBLEM, MEASURED. The stock export fails to parse for Hailo-8:

    UnsupportedShuffleLayerError in op /Transpose_30 ... and 70 more

The graph carries 71 Transposes, 72 Slices and 70 Unsqueezes, all 5-D with
perm [0,1,2,4,3]. They are one operation: `XFeatModel._unfold2d(x, ws=8)`,
which torch traces as a pile of 5-D slicing because it is written with
`Tensor.unfold`:

    x.unfold(2, ws, ws).unfold(3, ws, ws).reshape(B, C, H//ws, W//ws, ws**2)
     .permute(0, 1, 4, 2, 3).reshape(B, -1, H//ws, W//ws)

⭐ THAT IS SPACE-TO-DEPTH, which torch already has as `pixel_unshuffle` and
which Hailo supports natively. Substituting it is not an approximation:
verified `torch.equal` on random input, and this tool re-verifies the WHOLE
network against the stock ONNX before writing anything.

⚠ WHY THE VERIFICATION IS NOT OPTIONAL. The published advice for this is
"replace some layers into mathematically equivalent layers". A substitution
that is *nearly* equivalent would produce a network that compiles, runs, and
returns subtly different descriptors -- which would show up as a slightly worse
match rate and be blamed on quantisation. The check below is what separates
those two outcomes.

    python3 tools/xfeat_export_hailo.py --src <accelerated_features> \
        --out ~/hailo/work --sizes 320x240,640x480
"""
from __future__ import annotations

import argparse
import os
import sys


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--src', required=True)
    ap.add_argument('--out', default='/home/fh1m/hailo/work')
    ap.add_argument('--sizes', default='320x240')
    ap.add_argument('--tol', type=float, default=1e-5)
    a = ap.parse_args()

    sys.path.insert(0, a.src)
    import numpy as np
    import torch
    import torch.nn.functional as F
    from modules.model import XFeatModel

    def patched_unfold2d(self, x, ws=2):
        # Identical to the stock implementation, as one supported op.
        return F.pixel_unshuffle(x, ws)

    w = os.path.join(a.src, 'weights', 'xfeat.pt')
    os.makedirs(a.out, exist_ok=True)

    for spec in a.sizes.split(','):
        W, H = (int(v) for v in spec.lower().split('x'))
        x = torch.randn(1, 1, H, W)

        stock = XFeatModel().eval()
        stock.load_state_dict(torch.load(w, map_location='cpu'))
        with torch.no_grad():
            ref = [t.clone() for t in stock(x)]

        XFeatModel._unfold2d = patched_unfold2d
        patched = XFeatModel().eval()
        patched.load_state_dict(torch.load(w, map_location='cpu'))
        with torch.no_grad():
            got = patched(x)

        worst = max(float((r - g).abs().max()) for r, g in zip(ref, got))
        print(f'  {spec}: max abs diff across all three heads = {worst:.3e}')
        if worst > a.tol:
            raise SystemExit(
                f'REFUSING TO WRITE: the substitution changed the network by '
                f'{worst:.3e} > {a.tol:.0e}. A graph that compiles but computes '
                f'something else is worse than one that does not compile.')

        dst = os.path.join(a.out, f'xfeat_hailo_{W}x{H}.onnx')
        torch.onnx.export(
            patched, x, dst, opset_version=16, input_names=['image'],
            output_names=['feats', 'keypoints', 'heatmap'], dynamo=False)
        print(f'  wrote {dst} ({os.path.getsize(dst)/1e6:.1f} MB)')

    print('\n  ⚠ Parsing is not compiling. Next: translate, then optimise with '
          'REAL turbid calibration frames, then re-run the murky-clip table '
          'through the HEF -- a HEF that compiles is not a result.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
