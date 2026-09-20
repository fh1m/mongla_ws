#!/usr/bin/env python3
"""Where the milliseconds go, stage by stage, on ONE clock.

The composed pipeline runs at 70.8 Hz -- a 14.1 ms period -- while a detection
arrives 31.5 ms after the shutter. Those two numbers are consistent only if
something other than throughput is adding delay, and a rate figure cannot say
what. This walks the real path with the real classes and times each step.

Method note, because guessing cost two round trips on this pipeline before:
every stage is timed against the SAME monotonic clock and the stages are made
to sum to the measured total. A stage that does not appear in the sum is a
stage nobody is measuring.

    python3 tools/vision_stage_profile.py --model gate_rescue_repair --seconds 20
"""
import argparse
import os
import statistics as st
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..',
                                'src', 'mongla_vision'))

from mongla_vision import get_profile, make_camera_from_profile   # noqa: E402
from mongla_vision.detection.factory import make_detector         # noqa: E402


def _p(vals, q):
    s = sorted(vals)
    return s[min(len(s) - 1, int(round(q / 100.0 * (len(s) - 1))))]


def _row(name, vals, total_med):
    if not vals:
        print(f'    {name:<26}  --')
        return
    med = st.median(vals)
    share = 100.0 * med / total_med if total_med else 0.0
    print(f'    {name:<26} {med:7.2f} med  {_p(vals, 95):7.2f} p95  '
          f'{max(vals):7.2f} max   {share:5.1f} %')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--profile', default='pi_forward')
    ap.add_argument('--model', default='gate_rescue_repair')
    ap.add_argument('--conf', type=float, default=0.15)
    ap.add_argument('--seconds', type=float, default=20.0)
    ap.add_argument('--warmup', type=float, default=4.0)
    a = ap.parse_args()

    prof = get_profile(a.profile)
    prof.setdefault('name', a.profile)
    cam = make_camera_from_profile(prof)
    info = cam.info()
    print(f"\n  camera {info.get('width')}x{info.get('height')} "
          f"@ {info.get('fps')} fps  ({a.profile})")

    det = make_detector(model_path=a.model, conf=a.conf, max_det=100,
                        class_allowlist=None, device='cpu', half=False)
    print(f'  model  {a.model}\n')

    age_at_read, decode, infer, loop, gap = [], [], [], [], []
    pump_side, slot_wait = [], []
    last_done = None
    t_end = time.monotonic() + a.warmup + a.seconds
    t_measure = time.monotonic() + a.warmup

    while time.monotonic() < t_end:
        t0 = time.monotonic()
        frame, meta = cam.read()
        t1 = time.monotonic()
        if frame is None or not meta.fresh:
            time.sleep(0.001)
            continue
        # `read()` both waits for a fresh frame AND decodes it, so its
        # duration is decode plus whatever wait was left. The frame's own
        # age separates them: it is the part the pipeline cannot remove.
        t2 = time.monotonic()
        det.infer(frame)
        t3 = time.monotonic()
        if time.monotonic() < t_measure:
            last_done = t3
            continue
        age_at_read.append((t1 - meta.stamp_monotonic) * 1000.0)
        if meta.stamp_store:
            # OF THE FRAME ACTUALLY CONSUMED -- a mean over all stores is
            # a different quantity, and the two differ because the pump is
            # descheduled precisely while the consumer is inferring.
            pump_side.append((meta.stamp_store - meta.stamp_monotonic) * 1000.0)
            slot_wait.append((t1 - meta.stamp_store) * 1000.0)
        decode.append((t1 - t0) * 1000.0)
        infer.append((t3 - t2) * 1000.0)
        loop.append((t3 - meta.stamp_monotonic) * 1000.0)
        if last_done is not None:
            gap.append((t3 - last_done) * 1000.0)
        last_done = t3

    total_med = st.median(loop) if loop else 0.0
    print(f'  {len(loop)} frames, {len(loop) / a.seconds:.1f} Hz\n')
    print('                                  ms')
    _row('age when read returns', age_at_read, total_med)
    _row('  capture -> pump store', pump_side, total_med)
    _row('  waiting in the slot', slot_wait, total_med)
    _row('  of which: read+decode', decode, total_med)
    _row('inference', infer, total_med)
    print('    ' + '-' * 62)
    _row('CAPTURE -> DETECTIONS', loop, total_med)
    _row('period between frames', gap, total_med)
    print()
    if loop:
        unexplained = st.median(loop) - st.median(age_at_read) - st.median(infer)
        print(f'    unaccounted: {unexplained:+.2f} ms '
              f'(should be ~0; anything else is a stage nobody is timing)\n')
    # The pump's own counters. `skipped_to_newest` rising means the pump was
    # descheduled and caught up by discarding; `dropped_by_driver` rising means
    # it was descheduled for longer than the whole buffer queue, and the frames
    # were lost in the kernel before we ever saw them. Together they say
    # whether the age above is OUR backlog or the camera's own delivery.
    inf = cam.info()
    print(f"    pump: captured={inf.get('captured')}  "
          f"skipped_to_newest={inf.get('skipped_to_newest')}  "
          f"dropped_by_driver={inf.get('dropped_by_driver')}")
    print(f"    pump: age at STORE {inf.get('store_age_ms', 0):.2f} ms mean, "
          f"{inf.get('store_age_max_ms', 0):.2f} max  "
          f"<- pump-side. The rest of `age when read returns` is the\n          consumer being busy.\n")
    cam.close()


if __name__ == '__main__':
    main()
