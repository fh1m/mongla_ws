#!/usr/bin/env python3
"""What does INT8 cost a compiled model, measured off the chip? (issue #47)

Runs an OPTIMIZED .har through the Dataflow Compiler's emulator twice on the
same real frames -- `SDK_FP_OPTIMIZED` (the network after the DFC's model
modifications, in float) and `SDK_QUANTIZED` (the INT8 numerics the chip
runs) -- and reports how much each frame's best confidence drops. No Hailo
device is needed, so this runs on the dev box's GPU.

    env -u PYTHONPATH PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python \\
      ~/hailo-dfc-venv/bin/python tools/bench/hailo_int8_shift.py \\
      --har ~/hailo_work_gate/gate_candidate_optimized.har \\
      --clip ~/Downloads/gate.mkv --clip <archive>/torpedo.mkv

    ... --self-test      # the scorer against synthetic shifts it must recover

PASS CRITERIA, written before the first run (2026-11-06):

  The question #47 asked: is the INT8 confidence loss a property of the chip,
  or of the DFC running at reduced optimisation? Answer it with a
  COUNTERFACTUAL, not with one build: the same parsed model optimised at
  level 0 (no QFT) and at the default level 2 (QFT on 1024 frames, GPU).

  1. Report, per build and clip, the median and p90 of `fp - int8` over
     frames whose fp best score is >= 0.25 (below that the detection is not
     one a mission acts on, and a floor-squashed score hides the shift).
  2. #47's hypothesis is CONFIRMED if level 2's median shift is smaller than
     level 0's by >= 0.02 on every clip. It is REFUTED if level 2 is no
     better than level 0 (difference < 0.01): the loss is then the
     quantiser's, and no optimisation setting recovers it.
  3. The issue's own falsifier: if level 2's shift stays near 0.08
     (>= 0.07), keep the lowered runtime threshold and close with that
     number.

  The emulator is not the chip. Hailo documents SDK_QUANTIZED as
  emulating the hardware's quantised arithmetic, not as bit-exact. The
  chip-side number (0.04-0.06, measured-bars, HEF vs .pt) stays the one
  shipped thresholds rest on; this ranks builds against each other.
"""
from __future__ import annotations

import argparse
import statistics
import sys

ACT_FLOOR = 0.25


def best_scores(y):
    """(batch, classes, 5, boxes) NMS output -> best score per frame."""
    import numpy as np
    y = np.asarray(y)
    return y[:, :, 4, :].reshape(y.shape[0], -1).max(axis=1)


def shift_stats(fp, q, floor=ACT_FLOOR):
    """Median / p90 of fp - q over frames the fp path scores >= floor."""
    d = sorted(f - g for f, g in zip(fp, q) if f >= floor)
    if not d:
        return {'n': 0, 'median': float('nan'), 'p90': float('nan')}
    return {'n': len(d), 'median': statistics.median(d),
            'p90': d[int(0.9 * (len(d) - 1))]}


def verdict(level0: dict, level2: dict) -> str:
    """Criteria 2 and 3, from per-clip median shifts {clip: median}.

    As first written this had no case for level 2 being WORSE than level 0,
    and the first real run (2026-11-06) was exactly that -- level 2 lost 0.34
    where level 0 lost -0.01 -- which it then labelled "REFUTED: optimisation
    level does not change the INT8 loss". A difference of 0.35 is a change.
    """
    gains = [level0[c] - level2[c] for c in level2]
    if all(g >= 0.02 for g in gains):
        v = 'CONFIRMED: full optimisation recovers confidence'
    elif all(g <= -0.02 for g in gains):
        v = ('INVERTED: full optimisation LOSES confidence that level 0 keeps '
             '-- the optimiser degraded this build')
    elif all(abs(g) < 0.01 for g in gains):
        v = 'REFUTED: optimisation level does not change the INT8 loss'
    else:
        v = 'MIXED: the clips disagree; report both, decide nothing'
    if any(m >= 0.07 for m in level2.values()):
        v += ' -- level-2 shift still >= 0.07: keep the lowered threshold'
    return v


def frames(clip, n, imgsz=640):
    import cv2
    import numpy as np
    cap = cv2.VideoCapture(clip)
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
    out = []
    for k in range(n):
        cap.set(cv2.CAP_PROP_POS_FRAMES, int(total * (k + 0.5) / n))
        ok, im = cap.read()
        if ok:
            out.append(cv2.resize(im, (imgsz, imgsz))[:, :, ::-1])
    cap.release()
    if not out:
        raise SystemExit(f'no readable frames in {clip}')
    return np.stack(out).astype(np.float32)


def int8_vs_fp(runner, x, batch=8):
    """shift_stats of an optimised runner's float vs INT8 best scores on x.
    `hailo_compile.py` calls this as its post-optimise guard."""
    per = {}
    for name in ('SDK_FP_OPTIMIZED', 'SDK_QUANTIZED'):
        from hailo_sdk_client import InferenceContext
        # ONE infer call per context: calling per batch rebuilt the emulator
        # graph every time and ran ~20 s a batch.
        with runner.infer_context(getattr(InferenceContext, name)) as ctx:
            per[name] = [float(v) for v in
                         best_scores(runner.infer(ctx, x, batch_size=batch))]
    out = shift_stats(per['SDK_FP_OPTIMIZED'], per['SDK_QUANTIZED'])
    out['fp_p50'] = statistics.median(per['SDK_FP_OPTIMIZED'])
    return out


def measure(har, clips, n, batch=8):
    from hailo_sdk_client import ClientRunner
    runner = ClientRunner(har=har)
    return {clip: int8_vs_fp(runner, frames(clip, n), batch) for clip in clips}


def self_test() -> int:
    """The scorer must recover shifts it was handed, and ignore sub-floor frames."""
    import random
    rng = random.Random(0)
    fp = [rng.uniform(0.3, 0.9) for _ in range(500)]
    for true in (0.0, 0.03, 0.08):
        q = [f - true - rng.uniform(-0.005, 0.005) for f in fp]
        m = shift_stats(fp + [0.1] * 100, q + [0.0] * 100)['median']
        assert abs(m - true) < 0.004, (true, m)
    assert verdict({'a': 0.08}, {'a': 0.04}).startswith('CONFIRMED')
    assert verdict({'a': 0.05}, {'a': 0.045}).startswith('REFUTED')
    assert verdict({'a': -0.01}, {'a': 0.34}).startswith('INVERTED')
    assert 'keep the lowered' in verdict({'a': 0.12}, {'a': 0.08})
    print('self-test: scorer recovers 0.00 / 0.03 / 0.08 within 0.004; '
          'verdicts as specified')
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--har', action='append', default=[],
                    help='optimized .har; repeat as LABEL=PATH to compare builds')
    ap.add_argument('--clip', action='append', default=[])
    ap.add_argument('--frames', type=int, default=200)
    ap.add_argument('--self-test', action='store_true')
    a = ap.parse_args()
    if a.self_test:
        return self_test()
    if not a.har or not a.clip:
        ap.error('--har and --clip are required')
    results = {}
    for spec in a.har:
        label, _, path = spec.rpartition('=')
        label = label or path
        results[label] = measure(path, a.clip, a.frames)
        for clip, s in results[label].items():
            print(f'{label:>10}  {clip.rsplit("/", 1)[-1]:<14} n={s["n"]:<4} '
                  f'fp p50 {s["fp_p50"]:.3f}  shift median {s["median"]:+.3f}  '
                  f'p90 {s["p90"]:+.3f}')
    if {'level0', 'level2'} <= results.keys():
        print(verdict({c: s['median'] for c, s in results['level0'].items()},
                      {c: s['median'] for c, s in results['level2'].items()}))
    return 0


if __name__ == '__main__':
    sys.exit(main())
