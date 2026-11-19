"""Step 1 of 2 for LighterGlue on the Hailo-8: ONNX halves and real calibration.

Writes, into --out: half1.onnx and half2.onnx (tools/bench/lighterglue_hailo.py,
fixed N = 1024 on a 32 x 32 grid), and pairs.npz with real XFeat keypoint and
descriptor pairs from archive footage, each pair a frame and a warped, degraded
copy of it as in tools/studies/xfeat_reliability.py, together with the
float outputs of half 1 (the calibration input of half 2) and of the whole
network (what the quantised emulator is compared with). The DFC venv has no
torch, so step 2 (lighterglue_hailo_dfc.py) reads these files.

Calibration must be REAL descriptors: with synthetic ones the softmax over N
tokens stays flat near 1/N and the optimiser aborts (forum, SuperGlue on Hailo-8).
"""
from __future__ import annotations

import argparse
import os
import sys

import cv2
import numpy as np
import torch

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(1, os.path.join(HERE, '..', 'studies'))
sys.path.insert(2, os.path.join(HERE, '..'))
import lighterglue_export as ex  # noqa: E402
import lighterglue_hailo as lh   # noqa: E402
import xfeat_reliability as xr   # noqa: E402
from archive_root import archive_root  # noqa: E402

N = lh.GRID[0] * lh.GRID[1]


def pad(k, d, n=N):
    """Exactly N points: repeat the last ones if there are fewer. Matches that
    land on a repeat are dropped by the caller (index >= real count)."""
    real = len(k)
    if real >= n:
        return k[:n], d[:n], n
    extra = np.arange(n - real) % max(real, 1)
    return np.concatenate([k, k[extra]]), np.concatenate([d, d[extra]]), real


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--model', required=True, help='xfeat_<w>x<h>.onnx')
    ap.add_argument('--reference', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--frames', type=int, default=6, help='per clip')
    ap.add_argument('--seed', type=int, default=3)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)

    lg, _ = ex.build(a.reference)
    net = lg.net.cpu().eval()
    h1, h2 = lh.Half(net, 0, 3).eval(), lh.Half(net, 3, 6).eval()

    xf = xr._xfeat_module()
    heads = xr.Heads(a.model)
    w, h = heads.w, heads.h
    rng = np.random.default_rng(a.seed)
    A = archive_root()
    size = torch.tensor([float(w), float(h)])
    rec = {k: [] for k in ('k0', 'k1', 'd0', 'd1', 'n0', 'n1', 'H', 'x0', 'x1', 'ref', 'c0', 's0', 'c1', 's1')}
    for rel in xr.CLIPS:
        for g in xr.frames(f'{A}/{rel}', a.frames, w, h):
            H = xr.random_h(rng, w, h)
            g1 = xr.degrade(cv2.warpPerspective(g, H, (w, h)), rng)
            k0, d0 = xr.detect(xf, heads(g), True)
            k1, d1 = xr.detect(xf, heads(g1), True)
            k0, d0, n0 = pad(k0, d0)
            k1, d1, n1 = pad(k1, d1)
            kk0 = torch.from_numpy(k0)[None].float()
            kk1 = torch.from_numpy(k1)[None].float()
            dd0 = torch.from_numpy(d0)[None].float()
            dd1 = torch.from_numpy(d1)[None].float()
            with torch.inference_mode():
                c0, s0 = lh.host_encoding(net, kk0, size)
                c1, s1 = lh.host_encoding(net, kk1, size)
                x0, x1 = h1(lh.maps(dd0), lh.maps(dd1), c0, s0, c1, s1)
                f0, f1, z0, z1 = h2(x0, x1, c0, s0, c1, s1)
                sc = lh.assign(f0, f1, z0, z1)
            for key, v in (('k0', k0), ('k1', k1), ('d0', d0), ('d1', d1), ('n0', n0),
                           ('n1', n1), ('H', H), ('x0', x0.numpy()), ('x1', x1.numpy()),
                           ('ref', sc), ('c0', c0.numpy()), ('s0', s0.numpy()),
                           ('c1', c1.numpy()), ('s1', s1.numpy())):
                rec[key].append(v)
    np.savez_compressed(os.path.join(a.out, 'pairs.npz'),
                        **{k: np.stack(v) if k not in ('n0', 'n1') else np.array(v)
                           for k, v in rec.items()})

    # Fixed-shape ONNX, no dynamic axes: the compiler wants static shapes.
    c = torch.zeros(1, lh.D, *lh.GRID)
    x = torch.zeros(1, 64, *lh.GRID)
    y = torch.zeros(1, lh.D, *lh.GRID)
    for name, m, ins, names in (
            ('half1', h1, (x, x, c, c, c, c), ['x0', 'x1', 'c0', 's0', 'c1', 's1']),
            ('half2', h2, (y, y, c, c, c, c), ['x0', 'x1', 'c0', 's0', 'c1', 's1'])):
        outs = ['y0', 'y1'] if name == 'half1' else ['f0', 'f1', 'z0', 'z1']
        torch.onnx.export(m, ins, os.path.join(a.out, f'{name}.onnx'), dynamo=False,
                          input_names=names, output_names=outs, opset_version=13)
    print(f'wrote {len(rec["k0"])} real pairs, half1.onnx, half2.onnx to {a.out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
