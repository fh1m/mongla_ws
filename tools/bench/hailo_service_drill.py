#!/usr/bin/env python3
"""What happens to a vision process when the HailoRT service dies.

    python3 tools/bench/hailo_service_drill.py run MODEL.hef     # infer until it fails
    python3 tools/bench/hailo_service_drill.py none MODEL.hef    # open as the downward process
    python3 tools/bench/hailo_service_drill.py direct MODEL.hef  # open as the forward process

Run `run` in one shell, SIGKILL `hailort_service` in another, and time how
long systemd takes to bring it back. measured-bars 83 is the record.

Bars, written before the run:
  restart  after SIGKILL of hailort_service, systemd has it running again
           within 1 s
  surface  an inference in a process using the service raises within 1 s of
           the kill (it must not hang: a hang never reaches the node's exit)
  rejoin   a process started after the restart opens through the service
  order    with the service stopped, MONGLA_HAILO_FALLBACK=direct opens the
           chip and =none refuses
"""
import os
import sys
import time

import numpy as np
from hailo_platform import FormatType

from mongla_vision.detection import hailo_device as hd

role, HEF = sys.argv[1], sys.argv[2]

def model(dev):
    m = dev.create_infer_model(HEF)
    m.input().set_format_type(FormatType.UINT8)
    m.output().set_format_type(FormatType.FLOAT32)
    c = m.configure(); c.__enter__()
    b = c.create_bindings()
    b.input().set_buffer(np.zeros(m.input().shape, np.uint8))
    b.output().set_buffer(np.empty(m.output().shape, np.float32))
    return c, b

if role == 'run':          # infer until it fails; print when and how
    dev = hd._open(fallback='none'); c, b = model(dev)
    print('opened', hd.opened_via, flush=True)
    n = 0
    while True:
        t = time.monotonic()
        try:
            c.run([b], 5000); n += 1
        except Exception as e:
            print(f'FAILED after {n} at {time.time():.3f} took {time.monotonic()-t:.3f}s: {e!r}'[:300], flush=True)
            os._exit(3)
elif role in ('direct', 'none'):
    t = time.monotonic()
    try:
        dev = hd._open(fallback=role); c, b = model(dev)
        t1 = time.monotonic(); [c.run([b], 5000) for _ in range(50)]
        print(f'{role}: opened via {hd.opened_via} after {t1-t:.2f}s, 50 inferences in {time.monotonic()-t1:.3f}s', flush=True)
    except Exception as e:
        print(f'{role}: refused after {time.monotonic()-t:.2f}s: {e!r}'[:300], flush=True)
    os._exit(0)
