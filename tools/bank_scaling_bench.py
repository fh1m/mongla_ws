#!/usr/bin/env python3
"""How large can the checkpoint bank get before it stops fitting the budget?

`CheckpointBank` ships a capacity of 5, ported from a prototype gallery rather
than measured. The question "why only 5?" deserves a number, and the number is
not memory -- a reference is 1024x64 float32 = 256 kB, so a hundred of them is
25 MB on a box with gigabytes.

The cost is the MATCH, and it is linear in bank size: every reference is one
1024x1024x64 similarity matrix plus a mutual-nearest-neighbour pass. This
measures three things:

  looped    what the shipped `CheckpointBank.locate()` does -- N separate
            matches, the honest baseline
  batched   one (N*1024, 64) @ (64, 1024) GEMM, then partition the rows by
            reference. Identical arithmetic, one BLAS call
  shortlist retrieve first, match second. Each reference carries a 64-D
            signature (its L2-normalised mean descriptor); the current frame's
            signature picks the top-k, and only those k are matched

The shortlist is the only one of the three that breaks the linearity, so it is
the only one that answers "can the bank hold a hundred places?".

⚠ WHAT THIS DOES NOT MEASURE: whether the shortlist RETRIEVES THE RIGHT
reference. That is an accuracy question and it needs real footage, not random
descriptors -- `tools/bank_shortlist_accuracy.py` is that experiment. A fast
shortlist that returns the wrong reference is worse than a slow bank.
"""
from __future__ import annotations

import argparse
import importlib.util
import os
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))


def load_xfeat_module():
    p = os.path.join(HERE, '..', 'src', 'mongla_vision', 'mongla_vision',
                     'anchor', 'xfeat_onnx.py')
    spec = importlib.util.spec_from_file_location('xfeat_onnx', p)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def rand_desc(n, k, seed):
    d = np.random.default_rng(seed).standard_normal((n, k)).astype(np.float32)
    return d / np.linalg.norm(d, axis=1, keepdims=True)


def signature(desc):
    """One 64-D vector per reference: the L2-normalised mean descriptor.

    Not a learned global descriptor -- it is what we already have. The claim it
    encodes is weak but real: two views of one scene share keypoints, so their
    mean descriptors are closer than two views of different scenes. Whether
    that survives real footage is the accuracy experiment, not this one.
    """
    v = desc.mean(axis=0)
    n = np.linalg.norm(v)
    return v / n if n > 0 else v


def timeit(fn, reps):
    fn()                                    # warm
    ts = []
    for _ in range(reps):
        t0 = time.perf_counter()
        fn()
        ts.append((time.perf_counter() - t0) * 1e3)
    return float(np.median(ts))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--top-k', type=int, default=1024)
    ap.add_argument('--dim', type=int, default=64)
    ap.add_argument('--threads', type=int, default=2)
    ap.add_argument('--reps', type=int, default=7)
    ap.add_argument('--sizes', default='1,2,5,10,20,50,100')
    ap.add_argument('--shortlist', type=int, default=5)
    a = ap.parse_args()

    X = load_xfeat_module()
    cur = rand_desc(a.top_k, a.dim, 999)
    sizes = [int(v) for v in a.sizes.split(',')]
    bank = [rand_desc(a.top_k, a.dim, i) for i in range(max(sizes))]
    sigs = np.stack([signature(d) for d in bank])

    print(f'top_k={a.top_k} dim={a.dim} threads={a.threads} '
          f'reps={a.reps} shortlist={a.shortlist}')
    print(f'{"N":>5} {"looped":>9} {"batched":>9} {"shortlist":>10} '
          f'{"desc MB":>8}  budget@3Hz=333ms')

    for n in sizes:
        refs = bank[:n]

        def looped():
            for d in refs:
                X.XFeatONNX.match(d, cur, threads=a.threads)

        cat = np.ascontiguousarray(np.concatenate(refs, axis=0))

        def batched():
            sim = X._simmat(cat, cur, a.threads)
            # Partition rows back per reference; the mutual-NN pass is what
            # makes this more than one matmul, and it is per-reference.
            for i in range(n):
                s = sim[i * a.top_k:(i + 1) * a.top_k]
                i12 = s.argmax(axis=1)
                i21 = X._argmax0(s)
                idx0 = np.arange(len(i12))
                m = i21[i12] == idx0
                _ = s[idx0[m], i12[idx0[m]]]

        def shortlist():
            q = signature(cur)
            order = np.argsort(-(sigs[:n] @ q))[:min(a.shortlist, n)]
            for i in order:
                X.XFeatONNX.match(refs[i], cur, threads=a.threads)

        mb = n * a.top_k * a.dim * 4 / 1e6
        print(f'{n:>5} {timeit(looped, a.reps):>8.1f}ms '
              f'{timeit(batched, a.reps):>8.1f}ms '
              f'{timeit(shortlist, a.reps):>9.1f}ms {mb:>8.1f}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
