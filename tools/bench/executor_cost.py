#!/usr/bin/env python3
"""What an rclpy executor costs to deliver one callback, per executor.

A node here publishes a small message at a fixed rate and subscribes to it,
with a few timers beside it: the shape of our own nodes (state at 10-50 Hz,
detections at 15-100 Hz, a handful of timers). The same workload runs under
each executor in a fresh process, and the run reports:

    cpu_ms_per_cb   process CPU time divided by callbacks delivered
    lat_p50/p99_ms  publish-to-callback latency (monotonic clock in the message)
    delivered       callbacks delivered / callbacks expected

The bar, written before the first run (2026-11-10):

    PASS for an executor if, against SingleThreadedExecutor on the same box,
      cpu_ms_per_cb is at most half, AND
      lat_p99_ms is no worse than 1.5x, AND
      delivered is at least 0.99.

What would falsify the scorer itself: `--work-ms 5` makes every callback burn
5 ms of CPU. The executor's share then becomes small, so the ratio between
executors must move toward 1. If a "10x" survives real work in the callback,
the bench is measuring something other than the executor.

    python3 tools/bench/executor_cost.py                # all executors
    python3 tools/bench/executor_cost.py --work-ms 5    # the falsifier
    python3 tools/bench/executor_cost.py --self-test    # scorer arithmetic only
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time

EXECUTORS = ('single', 'multi', 'events')
BAR_CPU_RATIO = 0.5
BAR_P99_RATIO = 1.5
BAR_DELIVERED = 0.99


def verdict(base: dict, cand: dict) -> tuple[bool, str]:
    """PASS/FAIL of `cand` against `base`, by the bar in the docstring."""
    if cand.get('error'):
        return False, f"did not run: {cand['error']}"
    cpu = cand['cpu_ms_per_cb'] / base['cpu_ms_per_cb']
    p99 = cand['lat_p99_ms'] / max(base['lat_p99_ms'], 1e-9)
    why = (f"cpu x{cpu:.2f} (bar {BAR_CPU_RATIO}), p99 x{p99:.2f} "
           f"(bar {BAR_P99_RATIO}), delivered {cand['delivered']:.3f}")
    ok = (cpu <= BAR_CPU_RATIO and p99 <= BAR_P99_RATIO
          and cand['delivered'] >= BAR_DELIVERED)
    return ok, why


def _percentile(xs, q):
    xs = sorted(xs)
    return xs[min(len(xs) - 1, int(q * len(xs)))] if xs else float('nan')


def _child(kind: str, hz: float, seconds: float, timers: int, work_ms: float) -> None:
    import rclpy
    from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
    from rclpy.node import Node
    from std_msgs.msg import Float64

    rclpy.init()
    node = Node(f'executor_cost_{kind}')
    lat, n_timer = [], [0]

    def burn():
        end = time.perf_counter() + work_ms / 1000.0
        while time.perf_counter() < end:
            pass

    def on_msg(m):
        lat.append((time.monotonic() - m.data) * 1000.0)
        if work_ms:
            burn()

    pub = node.create_publisher(Float64, 'executor_cost', 10)
    node.create_subscription(Float64, 'executor_cost', on_msg, 10)
    node.create_timer(1.0 / hz, lambda: pub.publish(Float64(data=time.monotonic())))
    for _ in range(timers):
        node.create_timer(0.1, lambda: n_timer.__setitem__(0, n_timer[0] + 1))

    if kind == 'single':
        ex = SingleThreadedExecutor()
    elif kind == 'multi':
        ex = MultiThreadedExecutor()
    else:
        from rclpy.experimental import EventsExecutor
        ex = EventsExecutor()
    ex.add_node(node)

    time.sleep(0.5)                       # discovery, outside the measurement
    c0, t0 = time.process_time(), time.monotonic()
    while time.monotonic() - t0 < seconds:
        ex.spin_once(timeout_sec=0.1)
    cpu, wall = time.process_time() - c0, time.monotonic() - t0
    callbacks = 2 * len(lat) + n_timer[0]          # publish timer + sub + timers
    expected = hz * wall
    print(json.dumps({
        'executor': kind,
        'cpu_ms_per_cb': 1000.0 * cpu / max(callbacks, 1),
        'cpu_pct': 100.0 * cpu / wall,
        'lat_p50_ms': _percentile(lat, 0.50),
        'lat_p99_ms': _percentile(lat, 0.99),
        'delivered': len(lat) / expected,
        'callbacks': callbacks,
    }))


def _self_test() -> int:
    base = {'cpu_ms_per_cb': 1.0, 'lat_p99_ms': 2.0, 'delivered': 1.0}
    cases = [
        ({'cpu_ms_per_cb': 0.1, 'lat_p99_ms': 2.0, 'delivered': 1.0}, True),
        ({'cpu_ms_per_cb': 0.6, 'lat_p99_ms': 2.0, 'delivered': 1.0}, False),   # not cheap enough
        ({'cpu_ms_per_cb': 0.1, 'lat_p99_ms': 4.0, 'delivered': 1.0}, False),   # slower tail
        ({'cpu_ms_per_cb': 0.1, 'lat_p99_ms': 2.0, 'delivered': 0.9}, False),   # drops messages
        ({'error': 'ImportError'}, False),
    ]
    bad = [c for c, want in cases if verdict(base, c)[0] is not want]
    print('self-test', 'FAIL' if bad else 'ok', bad or '')
    return 1 if bad else 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--hz', type=float, default=100.0)
    ap.add_argument('--seconds', type=float, default=10.0)
    ap.add_argument('--timers', type=int, default=4)
    ap.add_argument('--work-ms', type=float, default=0.0)
    ap.add_argument('--self-test', action='store_true')
    ap.add_argument('--child', choices=EXECUTORS)
    a = ap.parse_args()
    if a.self_test:
        return _self_test()
    if a.child:
        _child(a.child, a.hz, a.seconds, a.timers, a.work_ms)
        return 0
    rows = {}
    for kind in EXECUTORS:
        out = subprocess.run(
            [sys.executable, __file__, '--child', kind, '--hz', str(a.hz),
             '--seconds', str(a.seconds), '--timers', str(a.timers),
             '--work-ms', str(a.work_ms)], capture_output=True, text=True)
        try:
            rows[kind] = json.loads(out.stdout.strip().splitlines()[-1])
        except Exception:                                   # noqa: BLE001
            rows[kind] = {'executor': kind,
                          'error': (out.stderr.strip().splitlines() or ['no output'])[-1]}
    print(f'{a.hz:.0f} Hz, {a.timers} timers, {a.seconds:.0f} s, work {a.work_ms} ms/callback')
    for kind, r in rows.items():
        if r.get('error'):
            print(f'  {kind:7s} ERROR: {r["error"]}')
            continue
        print(f'  {kind:7s} {r["cpu_ms_per_cb"]:.4f} ms/cb  cpu {r["cpu_pct"]:5.1f} %  '
              f'p50 {r["lat_p50_ms"]:.3f}  p99 {r["lat_p99_ms"]:.3f} ms  '
              f'delivered {r["delivered"]:.3f}')
    base = rows['single']
    if base.get('error'):
        print('ERROR: the baseline did not run')
        return 2
    for kind in ('multi', 'events'):
        ok, why = verdict(base, rows[kind])
        print(f'  {kind:7s} {"PASS" if ok else "FAIL"}: {why}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
