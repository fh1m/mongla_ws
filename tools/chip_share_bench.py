#!/usr/bin/env python3
"""XFeat + detector on one Hailo-8, via the scheduler and the sync API.

⛔ TWO PROCESSES CANNOT: a second client gets
HAILO_OUT_OF_PHYSICAL_DEVICES(74) while the first keeps full rate. Sharing has
to happen inside one process.

⛔ AND TWO GROUPS CANNOT BOTH BE `activate()`d: that raises
HailoRTInvalidOperationException. With the SCHEDULER enabled you do not
activate at all -- HailoRT time-slices the configured groups itself, which is
the supported mechanism and the one this measures.
"""
import time, threading
import numpy as np
from hailo_platform import (VDevice, HEF, ConfigureParams, HailoStreamInterface,
                            InputVStreamParams, OutputVStreamParams, InferVStreams,
                            FormatType, HailoSchedulingAlgorithm)

DET = '/home/fh1m/hailo_models/sauvc_sim.hef'
XF  = '/home/fh1m/hailo_models/xfeat_320x240.hef'
N = 60


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

    def run(self, n):
        # NO activate(): the scheduler owns activation when it is enabled.
        with InferVStreams(self.ng, self.ip, self.op) as pipe:
            for _ in range(5):
                pipe.infer({self.info.name: self.x})
            t0 = time.perf_counter()
            for _ in range(n):
                pipe.infer({self.info.name: self.x})
            return n / (time.perf_counter() - t0)


p = VDevice.create_params()
p.scheduling_algorithm = HailoSchedulingAlgorithm.ROUND_ROBIN
print(f'{"case":<30} {"detector Hz":>12} {"xfeat Hz":>10}')
with VDevice(p) as vd:
    det, xf = M(vd, DET), M(vd, XF)
    a_d = det.run(N); a_x = xf.run(N)
    print(f'{"alone (scheduler on)":<30} {a_d:>12.1f} {a_x:>10.1f}')
    r = {}
    t1 = threading.Thread(target=lambda: r.__setitem__('d', det.run(N)))
    t2 = threading.Thread(target=lambda: r.__setitem__('x', xf.run(N)))
    t1.start(); t2.start(); t1.join(); t2.join()
    print(f'{"shared, round-robin":<30} {r["d"]:>12.1f} {r["x"]:>10.1f}')
    print(f'{"keeps":<30} {100*r["d"]/a_d:>11.0f}% {100*r["x"]/a_x:>9.0f}%')
