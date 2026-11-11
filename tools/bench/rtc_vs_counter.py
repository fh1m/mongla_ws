#!/usr/bin/env python3
"""Is the RTC a better clock for control than the system counter?

Kutum has a dedicated real-time clock with its own cell. This asks whether it
should time anything in the control path, by watching its seconds turn over
against the kernel's counter.

    python3 tools/bench/rtc_vs_counter.py 900

Bar, written before the run: the RTC earns a place in control timing only if
it is finer than the loop it would time. The fastest host loop is 20 ms and a
move is timed on the board's 2 ms tick, so: a resolution finer than 1 ms, AND
a rate within 5 ppm of the network-disciplined counter. Otherwise it stays
what it is, wall time across a power cycle.

Falsified by: the residual of the fitted rate. Above the polling step (1 ms)
the turn-overs were not caught cleanly, and the fitted rate is not evidence.
"""
import time, json, sys
P = '/sys/class/rtc/rtc0/since_epoch'
secs = float(sys.argv[1]) if len(sys.argv) > 1 else 600
edges = []
last = open(P).read()
t_end = time.monotonic() + secs
while time.monotonic() < t_end:
    v = open(P).read()
    if v != last:
        edges.append((int(v), time.clock_gettime(time.CLOCK_MONOTONIC_RAW), time.time()))
        last = v
    time.sleep(0.0005)
n = len(edges)
r0, m0, w0 = edges[0]
xs = [e[0] - r0 for e in edges]
def fit(col):
    ys = [e[col] - edges[0][col] for e in edges]
    mx, my = sum(xs) / n, sum(ys) / n
    b = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sum((x - mx) ** 2 for x in xs)
    res = max(abs(y - my - b * (x - mx)) for x, y in zip(xs, ys))
    return (b - 1) * 1e6, res * 1e3
raw_ppm, raw_res = fit(1); wall_ppm, wall_res = fit(2)
print(json.dumps({'edges': n, 'span_s': xs[-1], 'rtc_resolution_s': 1,
                  'counter_raw_vs_rtc_ppm': round(raw_ppm, 2), 'raw_residual_ms': round(raw_res, 3),
                  'ntp_clock_vs_rtc_ppm': round(wall_ppm, 2), 'wall_residual_ms': round(wall_res, 3),
                  'rtc_phase_vs_wall_ms': round((w0 - r0) * 1e3, 1)}))
