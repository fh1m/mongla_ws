#!/usr/bin/env python3
"""Score the lock ladder against the detector, on a running stack (read-only).

    python3 tools/bench/lock_score.py --selftest
    python3 tools/bench/lock_score.py 90        # on Kutum, beside bringup vision:=true

Reports per camera: detection and lock rates, the lock's age, the share of time
on each rung, and the seconds the lock was `lost` within 2 s of a detection (the
bar: zero). It scores the ladder against the detector, not against truth: a
target the detector never sees is invisible to it. Truth needs footage with the
target labelled in every frame.
"""
# (details)
# BAR (written before the run): while a target was detected within the last 2 s,
# the lock is never 'lost' (a gap is bridged by held/follow/anchor).
# FALSIFIER: --selftest feeds a synthetic stream with a known 3 s unbridged gap.
import sys, time, collections
import rclpy
from vision_msgs.msg import Detection2DArray

BRIDGE_WINDOW_S = 2.0

def score(events, dur):
    """events: sorted (t, kind, value). kind 'det' (n detections) or 'lock' (rung)."""
    last_det = None; rung = None; t_prev = None
    share = collections.Counter(); unbridged = 0.0; gaps = []; gap_start = None
    for t, kind, v in events:
        if t_prev is not None and rung is not None:
            share[rung] += t - t_prev
            if rung == 'lost' and last_det is not None and t_prev - last_det <= BRIDGE_WINDOW_S:
                unbridged += t - t_prev
        if kind == 'det':
            if v > 0:
                if gap_start is not None: gaps.append(t - gap_start); gap_start = None
                last_det = t
            elif gap_start is None and last_det is not None:
                gap_start = t
        else:
            rung = v
        t_prev = t
    return {'share': {k: round(x / max(1e-9, sum(share.values())), 3) for k, x in share.items()},
            'unbridged_s': round(unbridged, 3), 'det_gaps': len(gaps),
            'longest_gap_s': round(max(gaps), 2) if gaps else 0.0}

def selftest():
    ev = [(0, 'lock', 'detection'), (0, 'det', 1), (1, 'det', 0), (1.1, 'lock', 'held'),
          (2, 'lock', 'lost'), (5, 'det', 1), (5, 'lock', 'detection'), (6, 'det', 1)]
    r = score(sorted(ev, key=lambda e: e[0]), 6)
    ok = r['unbridged_s'] >= 0.9 and r['det_gaps'] == 1
    print('selftest', 'PASS' if ok else 'FAIL', r); return 0 if ok else 1

def main(secs):
    rclpy.init(); n = rclpy.create_node('lock_score')
    ev = {c: [] for c in ('forward', 'downward')}; ages = {c: [] for c in ev}
    for c in ev:
        def mk(c):
            def det(m): ev[c].append((time.time(), 'det', len(m.detections)))
            def lock(m):
                now = time.time()
                if m.detections and m.detections[0].results:
                    ev[c].append((now, 'lock', m.detections[0].results[0].hypothesis.class_id))
                    ages[c].append(now - (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9))
            return det, lock
        d, l = mk(c)
        n.create_subscription(Detection2DArray, f'/mongla/vision/{c}/detections', d, 10)
        n.create_subscription(Detection2DArray, f'/mongla/vision/{c}/lock', l, 10)
    end = time.time() + secs
    while time.time() < end: rclpy.spin_once(n, timeout_sec=0.1)
    for c in ev:
        e = sorted(ev[c], key=lambda x: x[0])
        nd = sum(1 for x in e if x[1] == 'det'); nl = sum(1 for x in e if x[1] == 'lock')
        a = sorted(ages[c]); p = lambda q: round(1000 * a[int(q * (len(a) - 1))], 1) if a else float('nan')
        print(c, f'det {nd/secs:.1f} Hz  lock {nl/secs:.1f} Hz  lock age p50 {p(.5)} ms p95 {p(.95)} ms', score(e, secs))
if __name__ == '__main__':
    sys.exit(selftest() if '--selftest' in sys.argv else main(float(sys.argv[1]) if len(sys.argv) > 1 else 90) or 0)
