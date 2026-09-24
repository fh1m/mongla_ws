#!/usr/bin/env python3
"""Does XFeat still MATCH after INT8? The gate that makes the HEF a result.

A HEF that compiles is not a result. The bar is section 19.1's murky table,
re-run with the chip as the backbone and everything else identical: same
post-processing (the numpy port in xfeat_onnx.py), same MAGSAC, same
15-inlier trust bar, same reference-at-40 % / query-at-+1/3/5/8 s protocol.

Both arms run on the Pi so the only difference is float32-on-CPU versus
INT8-on-Hailo.
"""
import sys, glob, os, time
import numpy as np, cv2

sys.path.insert(0, '/home/fh1m/mongla_ws/src/mongla_vision')
import importlib.util
spec = importlib.util.spec_from_file_location(
    'xfeat_onnx',
    '/home/fh1m/mongla_ws/src/mongla_vision/mongla_vision/anchor/xfeat_onnx.py')
X = importlib.util.module_from_spec(spec); sys.modules['xfeat_onnx'] = X
spec.loader.exec_module(X)

from hailo_platform import (VDevice, HEF, ConfigureParams, HailoStreamInterface,
                            InputVStreamParams, OutputVStreamParams, InferVStreams,
                            FormatType)

HEF_PATH = '/home/fh1m/hailo_models/xfeat_320x240.hef'
ONNX     = '/home/fh1m/hailo_models/xfeat_320x240.onnx'
CLIPS = ['mirpur_torpedo', 'mirpur_torpedo_1', 'mirpur_gate', 'octagon', 'torpedo_clear']
OFFS  = ['q1', 'q3', 'q5', 'q8']
BAR   = 15


class HailoBackend:
    """Same interface as XFeatONNX; only the three head tensors come from the
    chip. Post-processing is the SHIPPED numpy port, unchanged -- otherwise
    this would compare two different algorithms and call it quantisation."""
    w, h = 320, 240
    top_k, det_thresh = 1024, 0.05

    def __init__(self, hef_path):
        self.hef = HEF(hef_path)
        self.target = VDevice()
        cfg = ConfigureParams.create_from_hef(self.hef, interface=HailoStreamInterface.PCIe)
        self.ng = self.target.configure(self.hef, cfg)[0]
        self.ng_params = self.ng.create_params()
        self.in_info = self.hef.get_input_vstream_infos()[0]
        self.in_params = InputVStreamParams.make(self.ng, format_type=FormatType.FLOAT32)
        self.out_params = OutputVStreamParams.make(self.ng, format_type=FormatType.FLOAT32)
        self.out_names = [o.name for o in self.hef.get_output_vstream_infos()]

    def detect(self, gray):
        if gray.shape[:2] != (self.h, self.w):
            gray = cv2.resize(gray, (self.w, self.h))
        x = (gray.astype(np.float32) / 255.0)[None, :, :, None]   # NHWC
        with self.ng.activate(self.ng_params):
            with InferVStreams(self.ng, self.in_params, self.out_params) as pipe:
                out = pipe.infer({self.in_info.name: x})
        # Identify heads by channel count: 64 feats, 65 keypoint logits, 1 heatmap
        feats = kpts = None
        for n, v in out.items():
            a = v[0]
            if a.shape[-1] == 64:   feats = a.transpose(2, 0, 1)[None]
            elif a.shape[-1] == 65: kpts  = a.transpose(2, 0, 1)[None]
        return self._post(feats, kpts)

    def _post(self, feats, kpt_logits):
        score = X._pixel_shuffle_scores(kpt_logits[0])
        k = X._nms(score, self.det_thresh)
        if len(k) == 0:
            return np.zeros((0, 2), np.float32), np.zeros((0, 64), np.float32)
        if len(k) > self.top_k:
            s = score[k[:, 1], k[:, 0]]
            k = k[np.argsort(-s)[:self.top_k]]
        f = feats[0]; _, fh, fw = f.shape
        gx = np.clip(k[:, 0] / 8.0 - 0.5, 0, fw - 1)
        gy = np.clip(k[:, 1] / 8.0 - 0.5, 0, fh - 1)
        x0 = np.floor(gx).astype(np.int32); x1 = np.minimum(x0 + 1, fw - 1)
        y0 = np.floor(gy).astype(np.int32); y1 = np.minimum(y0 + 1, fh - 1)
        wx = (gx - x0)[None, :]; wy = (gy - y0)[None, :]
        d = (f[:, y0, x0] * (1 - wx) * (1 - wy) + f[:, y0, x1] * wx * (1 - wy)
             + f[:, y1, x0] * (1 - wx) * wy + f[:, y1, x1] * wx * wy).T
        n = np.linalg.norm(d, axis=1, keepdims=True); n[n == 0] = 1
        return k.astype(np.float32), (d / n).astype(np.float32)

    match = staticmethod(X.XFeatONNX.match)


def inliers(be, k0, d0, k1, d1):
    if len(d0) < 4 or len(d1) < 4: return 0
    i0, i1 = be.match(d0, d1)
    if len(i0) < 8: return 0
    src = k0[i0].astype(np.float32).reshape(-1, 1, 2)
    dst = k1[i1].astype(np.float32).reshape(-1, 1, 2)
    H, m = cv2.findHomography(src, dst, cv2.USAC_MAGSAC, 3.0)
    return 0 if H is None or m is None else int(m.sum())


def run(be, tag):
    rows, times = {}, []
    for c in CLIPS:
        rp = f'/home/fh1m/murky/{c}__ref.png'
        if not os.path.exists(rp): continue
        t0 = time.perf_counter(); k0, d0 = be.detect(cv2.imread(rp, 0))
        times.append((time.perf_counter() - t0) * 1000)
        v = []
        for o in OFFS:
            qp = f'/home/fh1m/murky/{c}__{o}.png'
            if not os.path.exists(qp): v.append(None); continue
            k1, d1 = be.detect(cv2.imread(qp, 0))
            v.append(inliers(be, k0, d0, k1, d1))
        rows[c] = (len(k0), v)
    return rows, float(np.median(times))


print('INT8-on-Hailo vs float32-on-CPU, identical post-processing and protocol\n')
res = {}
for tag, be in (('CPU', X.XFeatONNX(ONNX, top_k=1024, threads=2)),
                ('HEF', HailoBackend(HEF_PATH))):
    res[tag], ms = run(be, tag)
    print(f'{tag}: median detect {ms:.1f} ms')

print(f'\n{"clip":<18} {"ref kp CPU/HEF":>15} {"CPU +1/3/5/8":>22} {"HEF +1/3/5/8":>22}  ok')
for c in CLIPS:
    if c not in res['CPU'] or c not in res['HEF']: continue
    (n0c, vc), (n0h, vh) = res['CPU'][c], res['HEF'][c]
    okc = sum(1 for x in vc if x and x >= BAR); okh = sum(1 for x in vh if x and x >= BAR)
    print(f'{c:<18} {n0c:>7}/{n0h:<7} {str(vc):>22} {str(vh):>22}  {okc}/4 -> {okh}/4')
