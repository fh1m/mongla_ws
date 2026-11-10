#!/usr/bin/env python3
"""The production configuration: XFeat at the rate the ANCHOR actually wants.

Round-robin gives each model an equal share and halves the detector (95.1 ->
48.7 Hz). But the anchor rung does not want 48 Hz -- `lock_node` evaluates it at
**3 Hz**, because §14 measured that 8 Hz cost +42 % of frame age for no extra
coverage. A model that asks for 3 Hz should not cost half the chip.

This throttles XFeat to a target rate and measures what the DETECTOR keeps.
"""
import time, threading
import numpy as np
from hailo_platform import (VDevice, HEF, ConfigureParams, HailoStreamInterface,
                            InputVStreamParams, OutputVStreamParams, InferVStreams,
                            FormatType, HailoSchedulingAlgorithm)

DET = '/home/fh1m/hailo_models/sauvc_sim.hef'
XF  = '/home/fh1m/hailo_models/xfeat_320x240.hef'


class M:
    def __init__(self, vd, path):
        hef = HEF(path)
        cfg = ConfigureParams.create_from_hef(hef, interface=HailoStreamInterface.PCIe)
        self.ng = vd.configure(hef, cfg)[0]
        self.info = hef.get_input_vstream_infos()[0]
        self.ip = InputVStreamParams.make(self.ng, format_type=FormatType.FLOAT32)
        self.op = OutputVStreamParams.make(self.ng, format_type=FormatType.FLOAT32)
        s = self.info.shape
        self.x = np.zeros((1, s[0], s[1], s[2]), np.float32)

    def flat_out(self, secs, out, key):
        n = 0
        with InferVStreams(self.ng, self.ip, self.op) as pipe:
            pipe.infer({self.info.name: self.x})
            t0 = time.perf_counter()
            while time.perf_counter() - t0 < secs:
                pipe.infer({self.info.name: self.x}); n += 1
            out[key] = n / (time.perf_counter() - t0)

    def at_rate(self, secs, hz, out, key):
        n = 0; period = 1.0 / hz
        with InferVStreams(self.ng, self.ip, self.op) as pipe:
            pipe.infer({self.info.name: self.x})
            t0 = nxt = time.perf_counter()
            while time.perf_counter() - t0 < secs:
                now = time.perf_counter()
                if now < nxt:
                    time.sleep(min(0.005, nxt - now)); continue
                nxt = now + period
                pipe.infer({self.info.name: self.x}); n += 1
            out[key] = n / (time.perf_counter() - t0)


p = VDevice.create_params()
p.scheduling_algorithm = HailoSchedulingAlgorithm.ROUND_ROBIN
SECS = 8
print(f'{"xfeat target":<18} {"detector Hz":>12} {"xfeat Hz":>10} {"detector keeps":>15}')
with VDevice(p) as vd:
    det, xf = M(vd, DET), M(vd, XF)
    r = {}
    det.flat_out(SECS, r, 'base')
    print(f'{"(none)":<18} {r["base"]:>12.1f} {0.0:>10.1f} {"100%":>15}')
    for hz in (3.0, 5.0, 10.0, 30.0):
        rr = {}
        t1 = threading.Thread(target=det.flat_out, args=(SECS, rr, 'd'))
        t2 = threading.Thread(target=xf.at_rate, args=(SECS, hz, rr, 'x'))
        t1.start(); t2.start(); t1.join(); t2.join()
        print(f'{hz:<18.0f} {rr["d"]:>12.1f} {rr["x"]:>10.1f} '
              f'{100*rr["d"]/r["base"]:>14.0f}%')
