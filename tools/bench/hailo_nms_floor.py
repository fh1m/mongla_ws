#!/usr/bin/env python3
"""Is the score threshold compiled into a HEF a floor, or only a default?

A detection HEF carries an NMS score threshold from its compile. The stack
treated that value as a hard floor: boxes below it were taken to be gone
before the host saw them, so a model compiled at 0.200 could never be run at
0.10 without a recompile. HailoRT's `InferModel` output has
`set_nms_score_threshold`, and the NMS of a YOLOv8-family HEF runs in
libhailort on the host (`engine=cpu` in the model script), so the value may be
changeable when the model is configured. This measures whether it is.

    python3 tools/bench/hailo_nms_floor.py MODEL.hef CLIP.mkv --floor 0.05

Three bars, written before the first run. All three must hold for the floor
to be set at run time:

  reach     with the floor F below the compiled value B, at least one box with
            a score in [F, B) is returned on a clip that shows the prop. None
            over the whole clip means B is a hard floor.
  identity  every box with a score at or above B is the same box in both
            runs: same frame, same class, same score, same corners. A lowered
            floor that moves or drops one confident box is not a threshold, it
            is a different post-process.
  cost      the median time of one inference at the lowered floor is within
            5 % of the compiled one, measured in alternating rounds so load on
            the machine falls on both.

Falsified by: zero boxes under B (reach), any box at or above B that differs
(identity), or a median more than 5 % slower (cost).
"""
import argparse
import json
import statistics
import sys
import time

import numpy as np


def frames_of(clip, count, every):
    import cv2
    cap = cv2.VideoCapture(clip)
    out, i = [], 0
    while len(out) < count:
        ok, frame = cap.read()
        if not ok:
            break
        if i % every == 0:
            out.append(frame)
        i += 1
    cap.release()
    return out


def boxes_of(per_class):
    """One frame's output as sorted (class, score, y0, x0, y1, x1) rows."""
    rows = []
    for cls, dets in enumerate(per_class):
        for d in np.asarray(dets, np.float32).reshape(-1, 5):
            rows.append((cls, float(d[4]), float(d[0]), float(d[1]),
                         float(d[2]), float(d[3])))
    return sorted(rows)


def run(hef, frames, floor):
    """Every frame through the model. `floor=None` leaves the compiled value."""
    import cv2
    from hailo_platform import FormatType, HailoSchedulingAlgorithm, VDevice
    params = VDevice.create_params()
    params.scheduling_algorithm = HailoSchedulingAlgorithm.ROUND_ROBIN
    per_frame, times = [], []
    with VDevice(params) as device:
        model = device.create_infer_model(hef)
        model.input().set_format_type(FormatType.UINT8)
        model.output().set_format_type(FormatType.FLOAT32)
        if floor is not None:
            model.output().set_nms_score_threshold(float(floor))
        h, w = model.input().shape[:2]
        shape = model.output().shape
        with model.configure() as configured:
            bindings = configured.create_bindings()
            for frame in frames:
                rgb = cv2.cvtColor(cv2.resize(frame, (w, h)),
                                   cv2.COLOR_BGR2RGB)
                bindings.input().set_buffer(np.ascontiguousarray(rgb))
                bindings.output().set_buffer(np.empty(shape, np.float32))
                t0 = time.perf_counter()
                configured.run([bindings], 2000)
                times.append(time.perf_counter() - t0)
                per_frame.append(boxes_of(bindings.output().get_buffer()))
    return per_frame, times


def verdict(compiled, lowered, baked, floor, t_compiled, t_lowered):
    """The three bars, from two runs. Pure, so `--self-test` can drive it."""
    under = [r for f in lowered for r in f if floor <= r[1] < baked]
    keep_c = [[r for r in f if r[1] >= baked] for f in compiled]
    keep_l = [[r for r in f if r[1] >= baked] for f in lowered]
    differing = sum(
        1 for a, b in zip(keep_c, keep_l)
        if len(a) != len(b) or any(
            x[0] != y[0] or not np.allclose(x[1:], y[1:], atol=1e-6)
            for x, y in zip(a, b)))
    mc, ml = statistics.median(t_compiled), statistics.median(t_lowered)
    slower = (ml - mc) / mc * 100.0
    return {
        'frames': len(compiled),
        'boxes_compiled': sum(len(f) for f in compiled),
        'boxes_lowered': sum(len(f) for f in lowered),
        'boxes_under_compiled_value': len(under),
        'lowest_score': min((r[1] for f in lowered for r in f), default=None),
        'confident_boxes': sum(len(f) for f in keep_c),
        'frames_whose_confident_boxes_differ': differing,
        'median_ms_compiled': round(mc * 1e3, 3),
        'median_ms_lowered': round(ml * 1e3, 3),
        'slower_percent': round(slower, 2),
        'reach': len(under) > 0,
        'identity': differing == 0,
        'cost': slower <= 5.0,
    }


def self_test():
    """The verdict against runs whose answer is chosen, so a bug in the
    scoring cannot pass for a property of the chip."""
    hi = (0, 0.9, 0.1, 0.1, 0.5, 0.5)
    lo = (0, 0.1, 0.2, 0.2, 0.4, 0.4)
    t = [0.010] * 9
    hard = verdict([[hi]], [[hi]], 0.2, 0.05, t, t)
    assert not hard['reach'] and hard['identity'] and hard['cost'], hard
    soft = verdict([[hi]], [[lo, hi]], 0.2, 0.05, t, t)
    assert soft['reach'] and soft['identity'], soft
    moved = (0, 0.9, 0.1, 0.1, 0.5, 0.6)
    bad = verdict([[hi]], [[lo, moved]], 0.2, 0.05, t, t)
    assert bad['reach'] and not bad['identity'], bad
    dropped = verdict([[hi]], [[lo]], 0.2, 0.05, t, t)
    assert not dropped['identity'], dropped
    slow = verdict([[hi]], [[lo, hi]], 0.2, 0.05, t, [0.011] * 9)
    assert not slow['cost'], slow
    print('self-test ok: hard floor, soft floor, moved box, dropped box, '
          'slow run each read as chosen')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('hef', nargs='?')
    ap.add_argument('clip', nargs='?')
    ap.add_argument('--floor', type=float, default=0.05)
    ap.add_argument('--baked', type=float, default=None,
                    help='the compiled value; read from the HEF when omitted')
    ap.add_argument('--frames', type=int, default=150)
    ap.add_argument('--every', type=int, default=10)
    ap.add_argument('--rounds', type=int, default=3)
    ap.add_argument('--self-test', action='store_true')
    a = ap.parse_args()
    if a.self_test:
        self_test()
        return 0
    if not a.hef or not a.clip:
        ap.error('a HEF and a clip, or --self-test')
    baked = a.baked
    if baked is None:
        import os
        sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..',
                                        'src', 'mongla_vision'))
        from mongla_vision.detection.hef_file import baked_score_threshold
        baked = baked_score_threshold(a.hef)
    if baked is None:
        ap.error('the compiled threshold could not be read; pass --baked')
    frames = frames_of(a.clip, a.frames, a.every)
    t_c, t_l = [], []
    for _ in range(a.rounds):
        compiled, t = run(a.hef, frames, None)
        t_c += t[5:]
        lowered, t = run(a.hef, frames, a.floor)
        t_l += t[5:]
    out = verdict(compiled, lowered, baked, a.floor, t_c, t_l)
    out.update(hef=a.hef, compiled_value=baked, floor=a.floor)
    print(json.dumps(out, indent=1))
    return 0 if out['reach'] and out['identity'] and out['cost'] else 1


if __name__ == '__main__':
    sys.exit(main())
