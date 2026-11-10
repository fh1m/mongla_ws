#!/usr/bin/env python3
"""Throughput is not latency, and a control loop pays latency.

Async raises FPS by keeping several frames in flight. But a vision-servoed hull
does not care how many frames per second the chip retires -- it cares how stale
the box is that it is steering on RIGHT NOW. Those are different numbers, and
queueing improves the first while making the second worse.

So this measures, per depth: submit-to-result for ONE frame (what the control
loop actually waits for) alongside the throughput. The useful depth is the one
that hides transfer behind compute WITHOUT adding queue delay.
"""
import argparse, time, statistics as st, os
import numpy as np
from hailo_platform import HEF, VDevice

ap = argparse.ArgumentParser()
ap.add_argument('--hef', required=True)
ap.add_argument('--depths', default='1,2,4,8')
ap.add_argument('--seconds', type=float, default=4.0)
A = ap.parse_args()

hef = HEF(A.hef)
size = int(hef.get_input_vstream_infos()[0].shape[0])
frame = np.random.randint(0, 255, (size, size, 3), np.uint8)
print(f'\n  {os.path.basename(A.hef)}  {size}x{size}\n')
print(f'  {"depth":>5} {"throughput":>11} {"age of the box you steer on":>30}')
print(f'  {"":>5} {"":>11} {"median":>10} {"p95":>9} {"max":>9}')
print('  ' + '-'*58)

for d in [int(x) for x in A.depths.split(',')]:
    tgt = VDevice()
    model = tgt.create_infer_model(A.hef)
    with model.configure() as cm:
        pool = []
        for _ in range(d):
            b = cm.create_bindings()
            b.input().set_buffer(frame)
            for on in model.output_names:
                b.output(on).set_buffer(np.empty(model.output(on).shape, np.float32))
            pool.append(b)
        cm.activate()
        try:
            cm.run([pool[0]], 10000)
            lat, n = [], 0
            t_start = time.perf_counter()
            end = t_start + A.seconds
            while time.perf_counter() < end:
                # Submit d frames, stamping each; then drain in order. The
                # LATENCY of frame k is submit_k -> its own completion, which
                # is what a controller consuming that frame actually waited.
                stamps, jobs = [], []
                for b in pool:
                    cm.wait_for_async_ready(timeout_ms=10000)
                    stamps.append(time.perf_counter())
                    jobs.append(cm.run_async([b]))
                for s, j in zip(stamps, jobs):
                    j.wait(10000)
                    lat.append((time.perf_counter() - s) * 1e3)
                    n += 1
            elapsed = time.perf_counter() - t_start
        finally:
            cm.deactivate()
    tgt.release()
    lat.sort()
    hz = n / elapsed
    print(f'  {d:>5} {hz:8.1f} Hz {st.median(lat):9.2f} ms '
          f'{lat[int(0.95*(len(lat)-1))]:7.2f} ms {lat[-1]:7.2f} ms')
print()
