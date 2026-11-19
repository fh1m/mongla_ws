"""Export XFeat's LighterGlue matcher to ONNX, for the anchor's CPU.

LighterGlue (verlab/accelerated_features, `modules/lighterglue.py`) is
LightGlue at 6 layers, 96 dimensions and one head, trained on XFeat's
64-D descriptors. Its two speed tricks, early stopping and point pruning,
change the graph per input and do not export (fabio-sim/LightGlue-ONNX says
the same), so this graph runs all six layers on every point. What it
returns is the log-assignment matrix without its dustbins; picking mutual best matches over a
confidence is a dozen lines of numpy and stays outside, where the bar can
change without a re-export.

Inputs: kpts0 (1, N, 2) and kpts1 (1, M, 2) in pixels, desc0 (1, N, 64) and
desc1 (1, M, 64), and the image size (w, h), which the network normalises
by. N and M are dynamic.

    python3 tools/bench/lighterglue_export.py --reference <accelerated_features> --out xfeat_lighterglue.onnx
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np


def build(reference: str):
    import torch
    sys.path.insert(0, reference)
    from modules.lighterglue import LighterGlue

    lg = LighterGlue(os.path.join(reference, 'weights', 'xfeat-lighterglue.pt'))
    net = lg.net.cpu().eval()
    net.conf.depth_confidence = -1
    net.conf.width_confidence = -1

    assert net.conf.num_heads == 1, net.conf.num_heads

    def rotary(freqs, t):
        # kornia's apply_cached_rotary_emb: rotate adjacent pairs.
        a, b = t[..., 0::2], t[..., 1::2]
        rot = torch.stack((-b, a), dim=3).reshape(t.shape)
        return t * freqs[0] + rot * freqs[1]

    class Core(torch.nn.Module):
        """kornia's LightGlue forward for one head, every layer, no padding.

        Written out rather than traced through kornia, whose unflatten and
        head transposes the TorchScript exporter cannot follow; the weights
        are kornia's own, and main() compares the two."""

        def __init__(self, n):
            super().__init__()
            self.n = n

        def self_block(self, blk, x, enc):
            qkv = blk.Wqkv(x)
            q, k, v = qkv[..., 0::3], qkv[..., 1::3], qkv[..., 2::3]
            q, k = rotary(enc, q), rotary(enc, k)
            att = torch.softmax(q @ k.transpose(1, 2) * q.shape[-1] ** -0.5, 2)
            return x + blk.ffn(torch.cat([x, blk.out_proj(att @ v)], -1))

        def cross_block(self, blk, x0, x1):
            qk0, qk1 = blk.to_qk(x0) * blk.scale ** 0.5, blk.to_qk(x1) * blk.scale ** 0.5
            v0, v1 = blk.to_v(x0), blk.to_v(x1)
            sim = qk0 @ qk1.transpose(1, 2)
            m0 = torch.softmax(sim, 2) @ v1
            m1 = torch.softmax(sim.transpose(1, 2), 2) @ v0
            x0 = x0 + blk.ffn(torch.cat([x0, blk.to_out(m0)], -1))
            x1 = x1 + blk.ffn(torch.cat([x1, blk.to_out(m1)], -1))
            return x0, x1

        def forward(self, kpts0, kpts1, desc0, desc1, size):
            # normalize_keypoints: centre on the image, divide by half its
            # longer side.
            shift = size[None, None, :] / 2
            scale = size.max() / 2
            n = self.n
            e = []
            for k in ((kpts0 - shift) / scale, (kpts1 - shift) / scale):
                # repeat_interleave(2) spelt as stack + reshape, which the
                # exporter keeps the rank of.
                p = n.posenc.Wr(k)
                p = torch.stack([p, p], -1).reshape(p.shape[0], p.shape[1], -1)
                e.append(torch.stack([torch.cos(p), torch.sin(p)], 0))
            d0, d1 = n.input_proj(desc0), n.input_proj(desc1)
            for layer in n.transformers:
                d0 = self.self_block(layer.self_attn, d0, e[0])
                d1 = self.self_block(layer.self_attn, d1, e[1])
                d0, d1 = self.cross_block(layer.cross_attn, d0, d1)
            la = n.log_assignment[n.conf.n_layers - 1]
            f0, f1 = la.final_proj(d0), la.final_proj(d1)
            dd = f0.shape[-1] ** 0.25
            sim = (f0 / dd) @ (f1 / dd).transpose(1, 2)
            z0, z1 = la.matchability(d0), la.matchability(d1)
            # The match part of sigmoid_log_double_softmax; the dustbin row
            # and column only matter for training.
            return (torch.log_softmax(sim, 2) + torch.log_softmax(sim, 1)
                    + torch.nn.functional.logsigmoid(z0)
                    + torch.nn.functional.logsigmoid(z1).transpose(1, 2))

    return lg, Core(net).eval()


def mutual(scores: np.ndarray, min_conf: float):
    """kornia's filter_matches on one log-assignment matrix, in numpy."""
    s = scores[0]
    if s.size == 0:
        return np.zeros(0, int), np.zeros(0, int), np.zeros(0, np.float32)
    m0 = s.argmax(1)
    m1 = s.argmax(0)
    i0 = np.arange(len(m0))
    ok = (m1[m0] == i0) & (np.exp(s[i0, m0]) > min_conf)
    return i0[ok], m0[ok], np.exp(s[i0[ok], m0[ok]]).astype(np.float32)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--reference', required=True, help='accelerated_features checkout')
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    import torch
    lg, core = build(a.reference)
    g = torch.Generator().manual_seed(0)
    n, m = 700, 900
    k0 = torch.rand(1, n, 2, generator=g) * torch.tensor([320., 240.])
    k1 = torch.rand(1, m, 2, generator=g) * torch.tensor([320., 240.])
    d0 = torch.nn.functional.normalize(torch.randn(1, n, 64, generator=g), dim=-1)
    d1 = torch.nn.functional.normalize(torch.randn(1, m, 64, generator=g), dim=-1)
    size = torch.tensor([320., 240.])
    torch.onnx.export(core, (k0, k1, d0, d1, size), a.out, dynamo=False,
                      input_names=['kpts0', 'kpts1', 'desc0', 'desc1', 'size'],
                      output_names=['scores'], opset_version=17,
                      dynamic_axes={'kpts0': {1: 'n'}, 'desc0': {1: 'n'},
                                    'kpts1': {1: 'm'}, 'desc1': {1: 'm'},
                                    'scores': {1: 'n1', 2: 'm1'}})
    # The export is checked against the network it came from, unpruned,
    # on different point counts than it was traced with.
    import onnxruntime as ort
    s = ort.InferenceSession(a.out, providers=['CPUExecutionProvider'])
    n, m = 1024, 512
    k0 = torch.rand(1, n, 2, generator=g) * size
    k1 = torch.rand(1, m, 2, generator=g) * size
    d0 = torch.nn.functional.normalize(torch.randn(1, n, 64, generator=g), dim=-1)
    d1 = torch.nn.functional.normalize(torch.randn(1, m, 64, generator=g), dim=-1)
    with torch.inference_mode():
        ref = lg.net({'image0': {'keypoints': k0, 'descriptors': d0, 'image_size': size[None]},
                      'image1': {'keypoints': k1, 'descriptors': d1, 'image_size': size[None]}}
                     )['log_assignment'][:, :-1, :-1].numpy()
    got = s.run(None, {'kpts0': k0.numpy(), 'kpts1': k1.numpy(), 'desc0': d0.numpy(),
                       'desc1': d1.numpy(), 'size': size.numpy()})[0]
    err = float(np.abs(ref - got).max())
    print(f'wrote {a.out}: {os.path.getsize(a.out) / 1e6:.1f} MB, max |onnx - torch| {err:.2e}')
    return 0 if err < 1e-3 else 1


if __name__ == '__main__':
    raise SystemExit(main())
