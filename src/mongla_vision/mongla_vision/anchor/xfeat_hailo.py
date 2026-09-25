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
_FEAT_CH, _KPT_CH, _REL_CH = 64, 65, 1

# ⭐ THE THIRD HEAD. The XFeat paper emits a keypoint heatmap, a 64-D
# descriptor map AND a reliability heatmap; `hailortcli parse-hef` confirms
# our own HEF carries all three (conv20 30x40x64, conv27 30x40x65,
# conv23 30x40x1). We computed the reliability on-chip and dropped it on the
# floor, while `min_cossim = 0.82` did the same job as ONE GLOBAL CONSTANT for
# every keypoint in every frame.
#
# ⚠ It is exposed as `last_reliability` rather than being applied here: the
# matcher is shared with the ONNX path, and changing what a match means on one
# backend only is how the two would come to disagree. Measure what it buys
# before any rung acts on it.

# Deadlock guards, not latency knobs -- same rationale as detection/hailo.py:
# an inference that takes a second has already broken the mission, and a tight
# bound would turn a slow frame into an exception.
_ASYNC_READY_MS = 1000
_ASYNC_WAIT_MS = 1000


class XFeatHailo:
    """Same interface as `XFeatONNX`: `detect(gray) -> (keypoints, descriptors)`."""

    w, h = 320, 240

    def __init__(self, hef_path: str, *, top_k: int = 1024,
                 det_thresh: float = 0.05):
        # ⛔ THE PROCESS'S ONE VDevice, AND THE SAME ARBITRATION THE DETECTORS
        # USE (B-62, measured twice on the vehicle).
        #
        #   attempt 1, own VDevice:  HAILO_OUT_OF_PHYSICAL_DEVICES(74)
        #   attempt 2, shared VDevice + lock:
        #       "Cant activate network because a network is already activated"
        #       HAILO_INVALID_OPERATION(6)
        #
        # The second failure is not a scheduling problem and no lock fixes it:
        # HailoRT permits ONE activated network group at a time, the detector
        # holds its activation for the process's life, and the old
        # `activate()` + `InferVStreams` idiom asks for a second one per call.
        #
        # ⭐ `detection/hailo.py` had already solved this for two detectors on
        # one chip: a module-level `_ACTIVE` registry where the next claimant
        # RELEASES the incumbent before activating. Joining that registry --
        # rather than inventing a parallel one -- is what lets the anchor and
        # the detector coexist, and it is why this class now speaks
        # `InferModel` like they do instead of the legacy API.
        from ..detection import hailo as _hd

        self._hd = _hd
        self._lock = _hd._DEVICE_LOCK
        self.top_k, self.det_thresh = int(top_k), float(det_thresh)
        self.path = hef_path
        self._target = _hd._shared_device()
        self._model = self._target.create_infer_model(hef_path)
        self._cim = None
        self._bindings = None
        self._in_buf = None
        self._out_bufs = {}

    def detect(self, gray: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        import cv2
        if gray.shape[:2] != (self.h, self.w):
            gray = cv2.resize(gray, (self.w, self.h))
        # ⚠ GREY, normalised to [0, 1] exactly as the ONNX path does. XFeat
        # takes one channel, which is why B-61's BGR/RGB defect in the
        # DETECTOR path does not touch this one.
        # ⛔ THE SAME LOCK THE DETECTOR USES, spanning acquire AND infer.
        # The chip runs one network group at a time whatever we do; releasing
        # across the wait would let the detector evict this activation
        # mid-flight, which `detection/hailo.py` recorded as a measured
        # SIGSEGV. Holding it means the ANCHOR waits for the detector rather
        # than the other way round -- the correct priority, since a late
        # detection costs a lock and a late anchor costs a slower re-anchor.
        with self._lock:
            cim = self._acquire_locked()
            # Into the bound buffer: no per-call allocation, and no per-call
            # activation either, which was most of the 14.5 ms the legacy
            # idiom cost.
            self._in_buf[...] = gray[:, :, None]
            cim.wait_for_async_ready(timeout_ms=_ASYNC_READY_MS)
            job = cim.run_async([self._bindings])
            job.wait(_ASYNC_WAIT_MS)
            raw = {k: v.copy() for k, v in self._out_bufs.items()}
        feats = kpts = rel = None
        for _name, a in raw.items():
            if a.shape[-1] == _FEAT_CH:
                feats = a.transpose(2, 0, 1)[None]
            elif a.shape[-1] == _KPT_CH:
                kpts = a.transpose(2, 0, 1)[None]
            elif a.shape[-1] == _REL_CH:
                rel = a[..., 0]
        if feats is None or kpts is None:
            raise RuntimeError(
                f'{self.path}: expected heads with {_FEAT_CH} and {_KPT_CH} '
                f'channels, got {[v[0].shape for v in out.values()]}. This HEF '
                f'is not an XFeat build.')
        self.last_reliability = rel
        return self._post(feats, kpts)

    def _acquire_locked(self):
        """Take the chip's single activation, evicting whoever holds it.

        ⛔ THE CALLER MUST HOLD `_DEVICE_LOCK`. This is the detector's own
        protocol, reused rather than reimplemented: `_ACTIVE` names whoever
        currently owns the activation, and the next claimant releases it
        first. Two registries would race each other and reintroduce exactly
        the `HAILO_INVALID_OPERATION(6)` this class was rewritten to fix.
        """
        from hailo_platform import FormatType

        hd = self._hd
        if hd._ACTIVE is self and self._cim is not None:
            return self._cim
        if hd._ACTIVE is not None and hd._ACTIVE is not self:
            hd._ACTIVE._release_locked()
        if self._cim is None:
            # ⛔ UINT8 IN, NOT FLOAT32. Measured: binding a float32 input gave
            #     Input buffer size 307200 is different than expected 76800
            # -- exactly 4x, which is the whole diagnosis. The HEF's input is
            # quantised uint8 (320*240*1 = 76 800 bytes) and HailoRT scales on
            # the way in, so the /255.0 the ONNX path does must NOT be
            # repeated here; doing it in float and then casting would quantise
            # the image twice.
            self._model.input().set_format_type(FormatType.UINT8)
            for name in self._out_names():
                self._model.output(name).set_format_type(FormatType.FLOAT32)
            self._cim = self._model.configure()
            self._cim.__enter__()
            self._in_buf = np.zeros((self.h, self.w, 1), np.uint8)
            self._bindings = self._cim.create_bindings()
            self._bindings.input().set_buffer(self._in_buf)
            # ⚠ ONE BUFFER PER OUTPUT, keyed by name. XFeat emits three heads
            # where the detector emits one, so the single-output binding the
            # detector uses does not transfer.
            for name in self._out_names():
                buf = np.zeros(tuple(self._model.output(name).shape),
                               np.float32)
                self._out_bufs[name] = buf
                self._bindings.output(name).set_buffer(buf)
        self._cim.activate()
        hd._ACTIVE = self
        return self._cim

    def _out_names(self):
        return [o.name for o in self._model.outputs]

    def _release_locked(self) -> None:
        """Give up the activation so a detector can take it. Caller holds the
        lock. Mirrors `HailoDetector._release_locked` -- the registry calls
        this on whichever object it finds, so the two must agree."""
        if self._cim is not None:
            try:
                self._cim.deactivate()
            except Exception:                                    # noqa: BLE001
                pass
        if self._hd._ACTIVE is self:
            self._hd._ACTIVE = None

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
