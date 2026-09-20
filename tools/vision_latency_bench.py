#!/usr/bin/env python3
"""How OLD is the picture when the control loop acts on it?

Every FPS number in this repo answers "how many frames per second". That is a
different question from this one, and the two can move in OPPOSITE directions:
uncapping capture made detections 26.0 -> 23.6 ms fresher and detections 47.5
-> 62.6 ms STALER in the same run, because every captured frame was also
published.

WHAT IS MEASURED
----------------
`header.stamp` is the CAPTURE instant -- `camera_node` derives it from the
kernel's `v4l2_buffer.timestamp` and `detector_node` passes the same header
through onto `detections`. So the age of a message here is the full chain:

    sensor -> USB -> kernel -> mailbox -> decode -> [inference] -> DDS -> here

Timing a callback would measure something close to the OPPOSITE: a message
already queued arrives in microseconds and is old, one we waited for arrives
slowly and is new.

WHY ONE TOOL, USED ON BOTH SIDES
--------------------------------
Composition is expected to move this by single-digit milliseconds. Two
different scripts cannot resolve that. Fix the camera count and the pause
state too -- an earlier 26.0-vs-15.0 ms split turned out to be CONTENTION
between two live detectors, not the pipeline, and was found only by pausing
one and re-running.

Also reports the mailbox's own drop counters where the node exposes them: in
a single composed process, a rising `_dropped_by_driver` is how a starved
capture thread would quietly return the staleness the mailbox removes, and
loop rate will not show it.

    python3 tools/vision_latency_bench.py --cameras forward --seconds 60
"""
import argparse
import statistics as st
import sys
import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray

from mongla_vision import qos


def _pct(vals, p):
    if not vals:
        return float('nan')
    s = sorted(vals)
    return s[min(len(s) - 1, int(round(p / 100.0 * (len(s) - 1))))]


class _Bench(Node):
    def __init__(self, cameras, warmup_s):
        super().__init__('mongla_vision_latency_bench')
        self._warmup_until = time.monotonic() + warmup_s
        # camera -> {'image': [ages], 'det': [ages], 'det_t': [arrival monotonic]}
        self.stats = {c: {'image': [], 'det': [], 'det_t': []} for c in cameras}
        for cam in cameras:
            ns = f'/mongla/vision/{cam}'
            self.create_subscription(
                Image, f'{ns}/image_raw', self._mk(cam, 'image'), qos.IMAGE)
            self.create_subscription(
                Detection2DArray, f'{ns}/detections',
                self._mk(cam, 'det'), qos.DETECTIONS)

    def _mk(self, cam, kind):
        def cb(msg):
            now_mono, now_wall = time.monotonic(), time.time()
            if now_mono < self._warmup_until:
                return
            s = msg.header.stamp
            stamp = s.sec + s.nanosec * 1e-9
            if stamp <= 0.0:
                return
            # Wall-clock difference converted to a duration; both clocks are
            # read at the same instant so the offset between them cancels.
            age_ms = (now_wall - stamp) * 1000.0
            # A stamp from the future is clock skew between hosts, not a
            # negative latency. Record it so it is visible rather than
            # silently clamped into the distribution.
            self.stats[cam][kind].append(age_ms)
            if kind == 'det':
                self.stats[cam]['det_t'].append(now_mono)
        return cb


def _report(cam, d, seconds):
    print(f'\n  === {cam} ===')
    for kind, label in (('image', 'image_raw '), ('det', 'detections')):
        v = d[kind]
        if not v:
            print(f'    {label}  NO MESSAGES -- '
                  f'check the node is up and unpaused')
            continue
        print(f'    {label}  n={len(v):5d}  {len(v) / seconds:5.1f} Hz   '
              f'age  med {st.median(v):6.2f}  p95 {_pct(v, 95):6.2f}  '
              f'max {max(v):6.2f} ms')
        neg = [x for x in v if x < 0]
        if neg:
            print(f'    {" " * len(label)}  {len(neg)} message(s) stamped in '
                  f'the FUTURE (min {min(neg):.2f} ms) -- host clock skew')
    t = d['det_t']
    if len(t) > 2:
        gaps = [(b - a) * 1000.0 for a, b in zip(t, t[1:])]
        print(f'    interval    med {st.median(gaps):6.2f}  '
              f'p95 {_pct(gaps, 95):6.2f}  max {max(gaps):6.2f} ms   '
              f'<- what VISION_FRESH_* must be derived from')


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--cameras', default='forward',
                    help='comma-separated (forward,downward)')
    ap.add_argument('--seconds', type=float, default=60.0)
    ap.add_argument('--warmup', type=float, default=5.0,
                    help='discarded: model load and the first-dequeue ramp')
    ap.add_argument('--label', default='',
                    help='printed with the result, e.g. "before" / "composed"')
    args = ap.parse_args(argv if argv is not None else sys.argv[1:])

    cams = [c.strip() for c in args.cameras.split(',') if c.strip()]
    rclpy.init()
    node = _Bench(cams, args.warmup)
    print(f'  warming up {args.warmup:.0f}s, then measuring {args.seconds:.0f}s '
          f'on {cams} ...')
    deadline = time.monotonic() + args.warmup + args.seconds
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass

    if args.label:
        print(f'\n  [{args.label}]')
    for cam in cams:
        _report(cam, node.stats[cam], args.seconds)
    print()
    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == '__main__':
    main()
