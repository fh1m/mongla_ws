#!/usr/bin/env python3
"""Blocking InferVStreams vs the async InferModel API, batch-1, same HEF.

The research could not find a published number for this anywhere. It matters
because the per-call cost of the blocking API is roughly CONSTANT: measured
~4.7 ms on this box, which is invisible behind a slow multi-context graph and
is 66 % of the frame on a fast single-context one.

Async lets the next frame's DMA start while the current one is still on the
chip. If the 4.7 ms is transfer and call overhead rather than compute,
pipelining should recover most of it. If it does not, the overhead is
per-inference and no amount of restructuring helps -- which is equally worth
knowing, and is the honest outcome to report.
"""
import argparse, os, sys, time, statistics as st
import numpy as np
from hailo_platform import (HEF, VDevice, HailoStreamInterface, InferVStreams,
                            ConfigureParams, InputVStreamParams,
                            OutputVStreamParams, FormatType)

ap = argparse.ArgumentParser()
ap.add_argument('--hef', required=True)
ap.add_argument('--seconds', type=float, default=8.0)
ap.add_argument('--depth', type=int, default=2, help='async jobs in flight')
A = ap.parse_args()

hef = HEF(A.hef)
size = int(hef.get_input_vstream_infos()[0].shape[0])
frame = np.random.randint(0, 255, (1, size, size, 3), np.uint8)
print(f'\n  {os.path.basename(A.hef)}  input {size}x{size}\n')

# ---------------------------------------------------------------- blocking
tgt = VDevice()
cfg = ConfigureParams.create_from_hef(hef, interface=HailoStreamInterface.PCIe)
ng = tgt.configure(hef, cfg)[0]
ngp = ng.create_params()
ivp = InputVStreamParams.make(ng, format_type=FormatType.UINT8)
ovp = OutputVStreamParams.make(ng, format_type=FormatType.FLOAT32)
in_name = hef.get_input_vstream_infos()[0].name

lat = []
with ng.activate(ngp):
    with InferVStreams(ng, ivp, ovp) as pipe:
        pipe.infer({in_name: frame})                      # warm
        end = time.perf_counter() + A.seconds
        while time.perf_counter() < end:
            t = time.perf_counter()
            pipe.infer({in_name: frame})
            lat.append((time.perf_counter() - t) * 1e3)
tgt.release()
b_med = st.median(lat)
print(f'  blocking InferVStreams   {1000/b_med:7.1f} Hz   '
      f'{b_med:6.2f} ms/frame   n={len(lat)}')

# ------------------------------------------------------------------- async
tgt2 = VDevice()
model = tgt2.create_infer_model(A.hef)
n_done = 0
t0 = time.perf_counter()
with model.configure() as cm:
    bindings_pool = []
    for _ in range(A.depth):
        b = cm.create_bindings()
        b.input().set_buffer(frame[0])
        for on in model.output_names:
            shp = model.output(on).shape
            b.output(on).set_buffer(np.empty(shp, np.float32))
        bindings_pool.append(b)
    cm.activate()                       # HAILO_STREAM_NOT_ACTIVATED without this
    try:
        cm.run([bindings_pool[0]], 10000)          # warm
        q = cm.get_async_queue_size()
        print(f'  async queue size         {q}')

        t0 = time.perf_counter()
        end = t0 + A.seconds
        jobs = []
        while time.perf_counter() < end:
            for b in bindings_pool:
                # Block only until the device can accept another job, then
                # dispatch. This is what lets frame N+1's DMA overlap frame N.
                cm.wait_for_async_ready(timeout_ms=10000)
                jobs.append(cm.run_async([b]))
            for j in jobs:
                j.wait(10000)
                n_done += 1
            jobs.clear()
        elapsed = time.perf_counter() - t0
    finally:
        cm.deactivate()
tgt2.release()
a_hz = n_done / elapsed
print(f'  async  InferModel d={A.depth}  {a_hz:7.1f} Hz   '
      f'{1000/a_hz:6.2f} ms/frame   n={n_done}')
print(f'\n  async is {a_hz/(1000/b_med):.2f}x the blocking path\n')
