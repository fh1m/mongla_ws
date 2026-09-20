#!/usr/bin/env python3
"""Feed the REAL detection stream into the real thresholds and see what happens.

The freshness thresholds are derived at runtime from two things the control
loop observes -- the pipeline latency and the detection interval. Unit tests
pin the invariants against numbers recorded earlier; this measures the LIVE
camera and puts the actual observations through the SHIPPING function.

It runs the same accounting the loop runs: a sample's age at first sight is
the pipeline latency, and the gap between distinct captures is the interval.
Then it asks the question that decides whether the vehicle moves properly:

    at the WORST age this camera actually produces, what authority does the
    control loop grant?

1.0 means the sensor is the limit. Anything less means OUR RULE is the limit,
and that is only acceptable when the camera genuinely cannot say where the
target is -- which the output states either way rather than leaving to a
reading of the constants.

    python3 tools/fresh_bounds_check.py --camera forward --seconds 30
"""
import argparse
import statistics as st
import sys
import time

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray

from mongla_vision import qos
from mongla_control.motion_vision import (
    _fresh_bounds, _freshness,
    VISION_FRESH_FULL_MAX_S, VISION_FRESH_ZERO_MAX_S,
)


class _Watch(Node):
    def __init__(self, camera):
        super().__init__('mongla_fresh_bounds_check')
        self.ages, self.gaps = [], []
        self._last_cap = None
        self.create_subscription(
            Detection2DArray, f'/mongla/vision/{camera}/detections',
            self._cb, qos.DETECTIONS)

    def _cb(self, msg):
        st_ = msg.header.stamp
        cap = st_.sec + st_.nanosec * 1e-9
        if cap <= 0:
            return
        age = time.time() - cap
        if not (0.0 <= age < 5.0):
            return
        self.ages.append(age)
        if self._last_cap is not None and cap > self._last_cap:
            gap = cap - self._last_cap
            if 0.0 < gap <= 0.5:
                self.gaps.append(gap)
        self._last_cap = cap


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--camera', default='forward')
    ap.add_argument('--seconds', type=float, default=30.0)
    a = ap.parse_args()

    rclpy.init()
    n = _Watch(a.camera)
    end = time.monotonic() + a.seconds
    while rclpy.ok() and time.monotonic() < end:
        rclpy.spin_once(n, timeout_sec=0.1)

    if len(n.ages) < 10 or not n.gaps:
        print(f'\n  {a.camera}: only {len(n.ages)} detections -- is the '
              f'detector running and unpaused?\n')
        return

    # What the loop would learn: the EMA of the age at arrival, and of the gap.
    def _pct(v, q):
        v = sorted(v)
        return v[min(len(v) - 1, int(round(q / 100.0 * (len(v) - 1))))]

    pipe = st.median(n.ages)
    interval = st.median(n.gaps)
    # p99, NOT max. Over a few thousand samples the maximum is one outlier --
    # a GC pause, a chip context swap, this tool's own spin_once being late --
    # and a threshold chosen to satisfy it would be chosen by noise. The p99
    # is what the vehicle actually lives with.
    worst = _pct(n.ages, 99)
    absolute_max = max(n.ages)
    full, zero = _fresh_bounds(interval, pipe)
    auth_typ = _freshness(pipe, interval, pipe)
    auth_worst = _freshness(worst, interval, pipe)

    print(f'\n  === {a.camera} -- LIVE, {len(n.ages)} detections in '
          f'{a.seconds:.0f}s ===\n')
    print(f'    rate                 {len(n.ages) / a.seconds:6.1f} Hz')
    print(f'    pipeline age  med    {pipe * 1000:6.2f} ms   '
          f'p95 {_pct(n.ages, 95) * 1000:6.2f}   p99 {worst * 1000:6.2f}   '
          f'max {absolute_max * 1000:6.2f} ms')
    print(f'    interval      med    {interval * 1000:6.2f} ms\n')
    print(f'    -> full authority up to  {full * 1000:6.1f} ms   '
          f'(cap {VISION_FRESH_FULL_MAX_S * 1000:.0f})')
    print(f'    -> zero authority at     {zero * 1000:6.1f} ms   '
          f'(cap {VISION_FRESH_ZERO_MAX_S * 1000:.0f})\n')
    print(f'    authority at the TYPICAL age  {auth_typ:.3f}')
    print(f'    authority at the p99     age  {auth_worst:.3f}')
    print(f'    authority at the max     age  '
          f'{_freshness(absolute_max, interval, pipe):.3f}   <- one outlier, not a design input')
    if auth_worst >= 1.0:
        print('\n    -> the SENSOR is the limit, not the rule. '
              'Full authority throughout.\n')
    else:
        print(f'\n    -> the RULE limits this camera at its worst age. That is '
              f'correct only if\n       {worst * 1000:.0f} ms is genuinely too '
              f'old to steer on '
              f'({worst * 0.65 * 100:.1f} cm of travel at 0.65 m/s).\n')


if __name__ == '__main__':
    main()
