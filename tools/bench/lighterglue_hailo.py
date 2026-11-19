"""LighterGlue rewritten for the Hailo-8 compiler, float first.

Forum evidence (community.hailo.ai, SuperGlue on Hailo-8, DFC 3.33) for what the
parser takes: 1x1 Conv2d in place of Linear or Conv1d; attention as a matmul
in the canonical shape with the transpose inside it; the N points on a real
spatial grid (here 32 x 32 for N = 1024), because a flat axis fails with
'Unexpected zero dimension'; at most 4 transformer layers in one HEF. This
network has 6, so it is two HEFs of 3, chained.

Changes from kornia's LightGlue, none of which change the arithmetic:
  * cos and sin of the positional projection depend only on the keypoints, so
    the HOST computes them and feeds them in as tensors (the chip has no cos);
  * rotate-pairs, rot(t) = (-t[2i+1], t[2i]), is a signed channel permutation,
    so it is folded into a second 1x1 conv, and t*cos + rot(t)*sin stays an
    elementwise multiply-add;
  * the final assignment keeps log-softmax and the mutual check on the host;
    the chip returns the projected descriptors and the matchability logits.

`python3 tools/bench/lighterglue_hailo.py --reference <accelerated_features>`
checks the float rewrite against kornia's network and exits non-zero past 1e-4.
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np
import torch
import torch.nn as nn

GRID = (32, 32)                         # N = 1024
D = 96


def _conv(lin: nn.Linear, perm_rows=None, sign=None) -> nn.Conv2d:
    w, b = lin.weight.detach().clone(), lin.bias.detach().clone() if lin.bias is not None else None
    if perm_rows is not None:
        w = w[perm_rows] * sign[:, None]
        b = b[perm_rows] * sign if b is not None else None
    c = nn.Conv2d(w.shape[1], w.shape[0], 1, bias=b is not None)
    c.weight.data = w[:, :, None, None]
    if b is not None:
        c.bias.data = b
    return c


def _rot_index(d):
    """rot(t)[2i] = -t[2i+1], rot(t)[2i+1] = t[2i], as (row index, sign)."""
    idx = torch.arange(d).view(-1, 2).flip(1).reshape(-1)
    sign = torch.tensor([-1.0, 1.0]).repeat(d // 2)
    return idx, sign


class Ffn(nn.Module):
    def __init__(self, seq: nn.Sequential):
        super().__init__()
        self.l1, self.ln, self.l2 = _conv(seq[0]), seq[1], _conv(seq[3])

    def forward(self, x):
        y = self.l1(x)
        # LayerNorm over the 2D channels at each point: channels-last for the
        # norm, back for the next conv. The DFC takes a channel-axis layer norm.
        y = self.ln(y.permute(0, 2, 3, 1)).permute(0, 3, 1, 2)
        return self.l2(torch.nn.functional.gelu(y))


def attend(q, k, v):
    """softmax(q k^T / sqrt(d)) v on (1, D, H, W) maps, N = H*W, one head."""
    n, d = q.shape[2] * q.shape[3], q.shape[1]
    qf, kf, vf = (t.reshape(1, d, n) for t in (q, k, v))
    prob = torch.softmax((qf.transpose(1, 2) @ kf) * d ** -0.5, dim=-1)
    return (vf @ prob.transpose(1, 2)).reshape(1, d, *q.shape[2:])


class SelfBlock(nn.Module):
    def __init__(self, blk):
        super().__init__()
        W, B = blk.Wqkv.weight.detach(), blk.Wqkv.bias.detach()
        # kornia interleaves q, k, v per channel: rows 3i, 3i+1, 3i+2.
        rows = {n: torch.arange(i, 3 * D, 3) for i, n in enumerate('qkv')}
        idx, sign = _rot_index(D)

        def conv(rows_, rot=False):
            w, b = W[rows_], B[rows_]
            if rot:
                w, b = w[idx] * sign[:, None], b[idx] * sign
            c = nn.Conv2d(D, D, 1)
            c.weight.data, c.bias.data = w[:, :, None, None].clone(), b.clone()
            return c

        self.q, self.qr = conv(rows['q']), conv(rows['q'], True)
        self.k, self.kr = conv(rows['k']), conv(rows['k'], True)
        self.v = conv(rows['v'])
        self.out = _conv(blk.out_proj)
        self.ffn = Ffn(blk.ffn)

    def forward(self, x, cos, sin):
        q = self.q(x) * cos + self.qr(x) * sin
        k = self.k(x) * cos + self.kr(x) * sin
        msg = self.out(attend(q, k, self.v(x)))
        return x + self.ffn(torch.cat([x, msg], 1))


class CrossBlock(nn.Module):
    def __init__(self, blk):
        super().__init__()
        self.qk, self.v, self.out = _conv(blk.to_qk), _conv(blk.to_v), _conv(blk.to_out)
        self.ffn = Ffn(blk.ffn)
        self.s = blk.scale ** 0.5

    def forward(self, x0, x1):
        a0, a1 = self.qk(x0) * self.s, self.qk(x1) * self.s
        v0, v1 = self.v(x0), self.v(x1)
        n, d = a0.shape[2] * a0.shape[3], a0.shape[1]
        sim = a0.reshape(1, d, n).transpose(1, 2) @ a1.reshape(1, d, n)
        m0 = (v1.reshape(1, d, n) @ torch.softmax(sim, -1).transpose(1, 2)).reshape(a0.shape)
        m1 = (v0.reshape(1, d, n) @ torch.softmax(sim.transpose(1, 2), -1).transpose(1, 2)).reshape(a0.shape)
        x0 = x0 + self.ffn(torch.cat([x0, self.out(m0)], 1))
        x1 = x1 + self.ffn(torch.cat([x1, self.out(m1)], 1))
        return x0, x1


class Half(nn.Module):
    """Layers [lo, hi) of LighterGlue; the first half also projects the input,
    the last also produces what the host needs for the assignment."""

    def __init__(self, net, lo, hi):
        super().__init__()
        self.first, self.last = lo == 0, hi == net.conf.n_layers
        self.proj = _conv(net.input_proj) if self.first else None
        self.self_ = nn.ModuleList(SelfBlock(net.transformers[i].self_attn) for i in range(lo, hi))
        self.cross = nn.ModuleList(CrossBlock(net.transformers[i].cross_attn) for i in range(lo, hi))
        if self.last:
            la = net.log_assignment[net.conf.n_layers - 1]
            self.fp, self.mt = _conv(la.final_proj), _conv(la.matchability)
            self.dd = float(D) ** 0.25

    def forward(self, x0, x1, c0, s0, c1, s1):
        if self.first:
            x0, x1 = self.proj(x0), self.proj(x1)
        for sb, cb in zip(self.self_, self.cross):
            x0, x1 = sb(x0, c0, s0), sb(x1, c1, s1)
            x0, x1 = cb(x0, x1)
        if not self.last:
            return x0, x1
        return self.fp(x0) / self.dd, self.fp(x1) / self.dd, self.mt(x0), self.mt(x1)


def host_encoding(net, kpts, size):
    """cos and sin of kornia's positional projection, as (1, D, H, W) maps."""
    shift, scale = size / 2, size.max() / 2
    p = net.posenc.Wr((kpts - shift) / scale)                 # (1, N, 48)
    p = torch.stack([p, p], -1).reshape(1, -1, D)
    h, w = GRID
    f = lambda t: t.transpose(1, 2).reshape(1, D, h, w)
    return f(torch.cos(p)), f(torch.sin(p))


def assign(f0, f1, z0, z1, min_conf=0.1):
    """The host half: kornia's log-assignment match part and mutual filter."""
    n = f0.shape[2] * f0.shape[3]
    a, b = f0.reshape(D, n).T, f1.reshape(D, n).T
    sim = a @ b.T
    sc = (torch.log_softmax(sim, 1) + torch.log_softmax(sim, 0)
          + torch.nn.functional.logsigmoid(z0.reshape(n, 1))
          + torch.nn.functional.logsigmoid(z1.reshape(1, n)))
    return sc[None].numpy()


def maps(desc):                                                  # (1, N, 64) -> (1, 64, H, W)
    return desc.transpose(1, 2).reshape(1, 64, *GRID)


def check(reference: str) -> int:
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import lighterglue_export as ex
    lg, _ = ex.build(reference)
    net = lg.net.cpu().eval()
    h1, h2 = Half(net, 0, 3).eval(), Half(net, 3, 6).eval()
    g = torch.Generator().manual_seed(1)
    size = torch.tensor([320., 240.])
    n = GRID[0] * GRID[1]
    # A second view with real matches: the first set shuffled, jittered and
    # noised, so the mutual filter has something to agree about.
    k0 = torch.rand(1, n, 2, generator=g) * size
    d0 = torch.nn.functional.normalize(torch.randn(1, n, 64, generator=g), dim=-1)
    perm = torch.randperm(n, generator=g)
    k1 = (k0[:, perm] + torch.randn(1, n, 2, generator=g) * 2).clamp(0, 239.9)
    d1 = torch.nn.functional.normalize(d0[:, perm] + 0.15 * torch.randn(1, n, 64, generator=g), dim=-1)
    with torch.inference_mode():
        c0, s0 = host_encoding(net, k0, size)
        c1, s1 = host_encoding(net, k1, size)
        x0, x1 = h1(maps(d0), maps(d1), c0, s0, c1, s1)
        f0, f1, z0, z1 = h2(x0, x1, c0, s0, c1, s1)
        got = assign(f0, f1, z0, z1)
        ref = lg.net({'image0': {'keypoints': k0, 'descriptors': d0, 'image_size': size[None]},
                      'image1': {'keypoints': k1, 'descriptors': d1, 'image_size': size[None]}}
                     )['log_assignment'][:, :-1, :-1].numpy()
    err = float(np.abs(ref - got).max())
    # Where the difference sits: only scores that can matter (above log 1e-4)
    # decide a match; a score of -30 against -30.002 decides nothing.
    live = ref > -9.2
    err_live = float(np.abs(ref - got)[live].max()) if live.any() else 0.0
    # What the anchor consumes is the mutual matches, so they are compared too;
    # a score difference that changes none of them is float32 accumulation over
    # a 1024-wide softmax, the same order as the plain ONNX export's.
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    same = []
    for conf in (0.1, 0.01, 0.001):
        i0, i1, _ = ex.mutual(ref, conf)
        j0, j1, _ = ex.mutual(got, conf)
        same.append(set(zip(i0, i1)) == set(zip(j0, j1)) and len(i0) > 0)
    print(f'Hailo-form float vs kornia, N={n}: max |diff| {err:.2e} '
          f'(scores above log 1e-4: {err_live:.2e}, {int(live.sum())} of them); '
          f'identical mutual matches at 0.1 / 0.01 / 0.001: {same}')
    return 0 if err_live < 1e-3 and all(same) else 1


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--reference', required=True)
    sys.exit(check(ap.parse_args().reference))
