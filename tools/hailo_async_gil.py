#!/usr/bin/env python3
"""Does the ASYNC Hailo path release the GIL that the blocking one holds?

The blocking `InferVStreams.infer()` was measured holding the GIL for its
entire ~10 ms (`tools/gil_probe.py`): another Python thread woke 9.21 ms late,
99.6 % of ticks, WORSE than a pure-Python spin control -- because the spin at
least yields on the 5 ms switch interval and a C call that never releases does
not yield at all.

That is the reason a composed camera+detector process measured a frame waiting
13.8 ms in the camera's slot: the capture pump is a Python thread and cannot
publish a newer frame while inference is running.

Round 29 measured the async API and recorded "async inference buys exactly
nothing". That was a THROUGHPUT measurement and it stands. This asks a
different question -- whether it releases the GIL -- and a call that submits
and returns cannot hold it for 10 ms by construction.

Same controls as `gil_probe.py`, and for the same reason: without them a gap
number cannot distinguish the subject from a badly-scheduling machine.
"""
import argparse
import os
import statistics as st
import sys
import threading
import time

import numpy as np

from hailo_platform import HEF, VDevice, FormatType

_TICK_S = 0.001


class _Sampler(threading.Thread):
    daemon = True

    def __init__(self):
        super().__init__()
        self.late = []
        self._run = True

    def run(self):
        while self._run:
            t0 = time.monotonic()
            time.sleep(_TICK_S)
            self.late.append((time.monotonic() - t0 - _TICK_S) * 1000.0)

    def stop(self):
        self._run = False


def _measure(label, work, seconds):
    s = _Sampler()
    s.start()
    time.sleep(0.3)
    s.late.clear()
    t_end = time.monotonic() + seconds
    n = 0
    while time.monotonic() < t_end:
        work()
        n += 1
    s.stop()
    time.sleep(0.05)
    g = sorted(x for x in s.late if x >= 0)
    if not g:
        print(f'  {label:<26} no samples')
        return
    p99 = g[min(len(g) - 1, int(0.99 * (len(g) - 1)))]
    blocked = 100.0 * sum(1 for x in g if x > 4.0) / len(g)
    print(f'  {label:<26} calls={n:5d}  woke late: med {st.median(g):6.2f}  '
          f'p99 {p99:7.2f}  max {max(g):7.2f} ms   >4ms: {blocked:5.1f} %   '
          f'rate {n / seconds:6.1f} Hz')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--hef', default=os.path.expanduser(
        '~/hailo_models/gate_rescue_repair.hef'))
    ap.add_argument('--seconds', type=float, default=5.0)
    # ONE MODE PER PROCESS. Configuring a network group the blocking way and
    # then asking the same VDevice for an InferModel raises
    # HAILO_STREAM_NOT_ACTIVATED (72) -- the chip holds one activation and the
    # two APIs each want to own it. Same family as the two-VDevice and
    # ROUND_ROBIN failures already recorded in `detection/hailo.py`.
    ap.add_argument('--mode', choices=('blocking', 'async'), required=True)
    a = ap.parse_args()

    vdev = VDevice()

    # ---------------- controls -------------------------------------------
    print('\n  controls first -- if these do not separate, the run is void\n')
    _measure('CONTROL sleep (frees)', lambda: time.sleep(0.010), a.seconds)

    def _spin():
        t = time.monotonic() + 0.010
        while time.monotonic() < t:
            pass
    _measure('CONTROL spin (HOLDS)', _spin, a.seconds)

    if a.mode == 'blocking':
        _blocking(vdev, a)
    else:
        _async(vdev, a)

    print('\n  A call that submits and waits cannot hold the GIL for the whole')
    print('  inference unless the WAIT also holds it. Compare against the two')
    print('  controls, not against intuition.\n')


def _blocking(vdev, a):
    from hailo_platform import (ConfigureParams, HailoStreamInterface,
                                InferVStreams, InputVStreamParams,
                                OutputVStreamParams)
    hef = HEF(a.hef)
    cfg = ConfigureParams.create_from_hef(hef,
                                          interface=HailoStreamInterface.PCIe)
    ng = vdev.configure(hef, cfg)[0]
    ivp = InputVStreamParams.make(ng, format_type=FormatType.UINT8)
    ovp = OutputVStreamParams.make(ng, format_type=FormatType.FLOAT32)
    in_name = hef.get_input_vstream_infos()[0].name
    size = int(hef.get_input_vstream_infos()[0].shape[0])
    buf = np.zeros((1, size, size, 3), np.uint8)

    with ng.activate(ng.create_params()):
        with InferVStreams(ng, ivp, ovp) as pipe:
            pipe.infer({in_name: buf})            # warm
            _measure('BLOCKING InferVStreams',
                     lambda: pipe.infer({in_name: buf}), a.seconds)

def _async(vdev, a):
    hef = HEF(a.hef)
    size = int(hef.get_input_vstream_infos()[0].shape[0])
    model = vdev.create_infer_model(a.hef)
    model.input().set_format_type(FormatType.UINT8)
    model.output().set_format_type(FormatType.FLOAT32)
    with model.configure() as cim:
        # `configure()` returns a ConfiguredInferModel; it does NOT activate
        # it. Without this, run_async raises HAILO_STREAM_NOT_ACTIVATED (72)
        # -- which reads like an API-mixing problem and is really a missing
        # call. (Invalid when the HailoRT scheduler is enabled; it is not
        # here, and enabling it is what SIGSEGVs with two graphs.)
        cim.activate()
        bindings = cim.create_bindings()
        ibuf = np.zeros((size, size, 3), np.uint8)
        obuf = np.zeros(model.output().shape, np.float32)
        bindings.input().set_buffer(ibuf)
        bindings.output().set_buffer(obuf)

        def _async_once():
            # wait_for_async_ready blocks until a slot is free; run_async
            # submits and returns a job we then wait on. If EITHER releases
            # the GIL the sampler stays on time.
            cim.wait_for_async_ready(timeout_ms=1000)
            job = cim.run_async([bindings])
            job.wait(1000)

        _async_once()                              # warm
        _measure('ASYNC run_async+wait', _async_once, a.seconds)
        cim.deactivate()


if __name__ == '__main__':
    main()
