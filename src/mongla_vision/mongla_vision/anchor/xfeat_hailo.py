"""XFeat on the Hailo-8, so the anchor rung stops burning the Pi's CPU.

⛔ WHAT THIS FIXES (B-62). `lock_node` resolves an empty `anchor_model` by
globbing `xfeat_*.onnx`, so the `.hef` sitting beside it in `~/hailo_models/`
was never a candidate, and `xfeat_onnx.py` hardcodes
`providers=['CPUExecutionProvider']`. Measured on the vehicle, 320x240:

    ONNX on the Pi CPU   32.9 ms  (p95 33.4)   30.4 FPS   <- what ran
    HEF on the Hailo-8    1.43 ms              701  FPS   <- what was measured

A 23x gap, and the anchor was on the wrong side of it -- spending 32.9 ms per
call on the CPU that the camera pump and the rclpy executor need, which is the
exact contention the async detector API exists to avoid.

⭐ IT ALSO UNBLOCKS RE-ID. `tracking/reid.py` is deferred on the premise that
"XFeat is ALREADY loaded at 701 FPS, so identity is nearly free". That premise
was false while this ran on the CPU; it becomes true here.

THE POST-PROCESSING IS NOT REWRITTEN. `_pixel_shuffle_scores`, `_nms` and
`match` are reused from `xfeat_onnx`, because two copies of the descriptor
maths is how the two backends come to disagree about what a feature is -- and
the whole point of the anchor is that a checkpoint stored by one path is
matchable by the other. The ONLY thing that differs here is the transport.

⚠ THE CHIP IS SHARED WITH THE DETECTOR. Both graphs contend for one device and
HailoRT serialises them; the detector must keep priority, because a late
detection costs a lock while a late anchor costs only a slower re-anchor.
Measure the contention before trusting the 1.43 ms figure in a live graph --
701 FPS is throughput measured ALONE, which is exactly the kind of number this
file exists to stop us quoting.
"""
from __future__ import annotations

from typing import Tuple

import numpy as np

from . import xfeat_onnx as _x

# Channel counts identify the heads. The HEF's output NAMES are compiler
# artifacts and have changed between builds; the shapes are a property of the
# network, so they are what this keys on.
_FEAT_CH, _KPT_CH = 64, 65


class XFeatHailo:
    """Same interface as `XFeatONNX`: `detect(gray) -> (keypoints, descriptors)`."""

    w, h = 320, 240

    def __init__(self, hef_path: str, *, top_k: int = 1024,
                 det_thresh: float = 0.05):
        import hailo_platform as hp

        # ⛔ THE PROCESS'S ONE VDevice, NEVER A SECOND (measured, B-62).
        # Opening our own device while the detector holds one fails with
        #     HAILO_OUT_OF_PHYSICAL_DEVICES(74)
        # and the loser is whichever component asks second -- on the vehicle
        # that was the DETECTOR, so a faster anchor bought no detections at
        # all. `detection/hailo.py` already documents this race, including its
        # other face (a silent hang during concurrent construction), and
        # `_shared_device()` is the answer it arrived at: two models CAN be
        # resident on one VDevice and take turns, measured at 98.2 Hz.
        from ..detection.hailo import _DEVICE_LOCK, _shared_device

        self._lock = _DEVICE_LOCK
        self._InferVStreams = hp.InferVStreams
        self.top_k, self.det_thresh = int(top_k), float(det_thresh)
        self._hef = hp.HEF(hef_path)
        self._dev = _shared_device()
        cfg = hp.ConfigureParams.create_from_hef(
            self._hef, interface=hp.HailoStreamInterface.PCIe)
        with self._lock:
            self._ng = self._dev.configure(self._hef, cfg)[0]
        self._ng_params = self._ng.create_params()
        self._in = self._hef.get_input_vstream_infos()[0]
        self._in_params = hp.InputVStreamParams.make(
            self._ng, format_type=hp.FormatType.FLOAT32)
        self._out_params = hp.OutputVStreamParams.make(
            self._ng, format_type=hp.FormatType.FLOAT32)
        self.path = hef_path

    def detect(self, gray: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        import cv2
        if gray.shape[:2] != (self.h, self.w):
            gray = cv2.resize(gray, (self.w, self.h))
        # ⚠ GREY, normalised to [0, 1] exactly as the ONNX path does. XFeat
        # takes one channel, which is why B-61's BGR/RGB defect in the
        # DETECTOR path does not touch this one.
        x = (gray.astype(np.float32) / 255.0)[None, :, :, None]   # NHWC
        # ⛔ THE SAME LOCK THE DETECTOR USES, spanning activate AND infer.
        # The chip runs one network group at a time whatever we do; releasing
        # across the wait would let the detector evict this activation
        # mid-flight, which `detection/hailo.py` recorded as a measured
        # SIGSEGV. Holding it means the ANCHOR waits for the detector rather
        # than the other way round -- the correct priority, since a late
        # detection costs a lock and a late anchor costs a slower re-anchor.
        with self._lock:
            with self._ng.activate(self._ng_params):
                with self._InferVStreams(self._ng, self._in_params,
                                         self._out_params) as pipe:
                    out = pipe.infer({self._in.name: x})
        feats = kpts = None
        for _name, v in out.items():
            a = v[0]
            if a.shape[-1] == _FEAT_CH:
                feats = a.transpose(2, 0, 1)[None]
            elif a.shape[-1] == _KPT_CH:
                kpts = a.transpose(2, 0, 1)[None]
        if feats is None or kpts is None:
            raise RuntimeError(
                f'{self.path}: expected heads with {_FEAT_CH} and {_KPT_CH} '
                f'channels, got {[v[0].shape for v in out.values()]}. This HEF '
                f'is not an XFeat build.')
        return self._post(feats, kpts)

    def _post(self, feats, kpt_logits):
        """Identical arithmetic to the ONNX path, reusing its helpers."""
        score = _x._pixel_shuffle_scores(kpt_logits[0])
        k = _x._nms(score, self.det_thresh)
        if len(k) == 0:
            return (np.zeros((0, 2), np.float32),
                    np.zeros((0, 64), np.float32))
        if len(k) > self.top_k:
            s = score[k[:, 1], k[:, 0]]
            k = k[np.argsort(-s)[:self.top_k]]
        f = feats[0]
        _, fh, fw = f.shape
        gx = np.clip(k[:, 0] / 8.0 - 0.5, 0, fw - 1)
        gy = np.clip(k[:, 1] / 8.0 - 0.5, 0, fh - 1)
        x0 = np.floor(gx).astype(np.int32)
        x1 = np.minimum(x0 + 1, fw - 1)
        y0 = np.floor(gy).astype(np.int32)
        y1 = np.minimum(y0 + 1, fh - 1)
        wx = (gx - x0)[None, :]
        wy = (gy - y0)[None, :]
        d = (f[:, y0, x0] * (1 - wx) * (1 - wy)
             + f[:, y0, x1] * wx * (1 - wy)
             + f[:, y1, x0] * (1 - wx) * wy
             + f[:, y1, x1] * wx * wy).T
        n = np.linalg.norm(d, axis=1, keepdims=True)
        n[n == 0] = 1
        return k.astype(np.float32), (d / n).astype(np.float32)

    # ⭐ THE SAME MATCHER, DELIBERATELY. A checkpoint enrolled through one
    # backend must be matchable through the other, or the bank silently
    # partitions by whichever transport happened to store each entry.
    match = staticmethod(_x.XFeatONNX.match)
