"""XFeat on the Hailo-8, so the anchor rung stops burning the Pi's CPU.

What this fixes (B-62). `lock_node` resolves an empty `anchor_model` by
globbing `xfeat_*.onnx`, so the `.hef` sitting beside it in `~/hailo_models/`
was never a candidate, and `xfeat_onnx.py` hardcodes
`providers=['CPUExecutionProvider']`. Measured on the vehicle, 320x240:

    ONNX on the Pi CPU   32.9 ms  (p95 33.4)   30.4 FPS   <- what ran
    HEF on the Hailo-8    1.43 ms              701  FPS   <- what was measured

A 23x gap, and the anchor was on the wrong side of it -- spending 32.9 ms per
call on the CPU that the camera pump and the rclpy executor need, which is the
exact contention the async detector API exists to avoid.

The post-processing is not rewritten. `_pixel_shuffle_scores`, `_nms` and
`match` are reused from `xfeat_onnx`, because two copies of the descriptor
maths is how the two backends come to disagree about what a feature is -- and
the whole point of the anchor is that a checkpoint stored by one path is
matchable by the other. The ONLY thing that differs here is the transport.

The chip is shared with the detector. Both graphs contend for one device and
HailoRT serialises them; the detector must keep priority, because a late
detection costs a lock while a late anchor costs only a slower re-anchor.
Measure the contention before trusting the 1.43 ms figure in a live graph --
701 FPS is throughput measured ALONE, which is exactly the kind of number this
file exists to stop us quoting.
"""
from __future__ import annotations

import threading
from typing import Tuple

import numpy as np

from . import xfeat_onnx as _x

# Channel counts identify the heads. The HEF's output NAMES are compiler
# artifacts and have changed between builds; the shapes are a property of the
# network, so they are what this keys on.
_FEAT_CH, _KPT_CH, _REL_CH = 64, 65, 1

# The third head, the reliability heatmap (conv23 30x40x1), ranks the
# keypoints with their probability, as upstream does. Both backends hand it to
# the shared `keypoints_and_descriptors`, so they pick the same set
# (measured-bars section 92).

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
        # The process's one VDevice (B-62): a VDevice of its own gave
        # HAILO_OUT_OF_PHYSICAL_DEVICES(74). The device runs HailoRT's
        # ROUND_ROBIN scheduler (see `detection/hailo_device.py`), so this group and
        # the detectors' share the chip with no Python lock around an
        # inference: measured, two detectors keep 44.4 Hz each with both
        # anchors running ~5 Hz beside them.
        from ..detection import hailo_device

        self._hd = hailo_device
        # This anchor's own buffers only -- never the chip.
        self._lock = threading.Lock()
        self.top_k, self.det_thresh = int(top_k), float(det_thresh)
        self._rel_warned = False
        self.path = hef_path
        self._target = hailo_device.shared_device()
        self._model = self._target.create_infer_model(hef_path)
        # One registry for every network group on the chip, so the exit hook
        # in `detection/hailo_device.py` hands this one back too (see `close_all`).
        hailo_device.register(self)
        self._closed = False
        self._cim = None
        self._bindings = None
        self._in_buf = None
        self._out_bufs = {}

    def detect(self, gray: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        import cv2
        if gray.shape[:2] != (self.h, self.w):
            gray = cv2.resize(gray, (self.w, self.h))
        # GREY as raw uint8, with no division by 255 (the note in _configured
        # says why). XFeat takes one channel, which is why B-61's BGR/RGB defect in the
        # DETECTOR path does not touch this one.
        # This anchor's lock only: its bound buffers would be overwritten by
        # a second call mid-read. The chip is shared by the scheduler.
        with self._lock:
            if self._closed:
                # After `close()` -- the process is exiting and a daemon
                # thread is still calling. Re-acquiring would CONFIGURE a
                # fresh group on a device being released; no features is the
                # honest answer, and the anchor treats it as a miss.
                return (np.zeros((0, 2), np.float32),
                        np.zeros((0, 64), np.float32))
            cim = self._configured()
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
                rel = a.transpose(2, 0, 1)[None]
        if feats is None or kpts is None:
            raise RuntimeError(
                f'{self.path}: expected heads with {_FEAT_CH} and {_KPT_CH} '
                f'channels, got {[v.shape for v in raw.values()]}. This HEF '
                f'is not an XFeat build.')
        if rel is None and not self._rel_warned:
            # An older build without the head still works, ranked as before.
            self._rel_warned = True
            import logging
            logging.getLogger(__name__).warning(
                f'{self.path}: no reliability head; keypoints ranked by '
                f'probability alone (fewer correct matches, section 92)')
        return self._post(feats, kpts, rel)

    def _configured(self):
        """This anchor's network group, configured once. NOT activated.

        Caller holds `self._lock`. The device lock is taken for the configure
        only; with the scheduler on, `activate()` is invalid and unneeded.
        """
        from hailo_platform import FormatType

        if self._cim is None:
            # UINT8 in, not FLOAT32. Measured: binding a float32 input gave
            #     Input buffer size 307200 is different than expected 76800
            # -- exactly 4x, which is the whole diagnosis. The HEF's input is
            # quantised uint8 (320*240*1 = 76 800 bytes) and HailoRT scales on
            # the way in, so the /255.0 the ONNX path does must NOT be
            # repeated here; doing it in float and then casting would quantise
            # the image twice.
            self._model.input().set_format_type(FormatType.UINT8)
            for name in self._out_names():
                self._model.output(name).set_format_type(FormatType.FLOAT32)
            with self._hd.DEVICE_LOCK:
                self._cim = self._model.configure()
                self._cim.__enter__()
            self._in_buf = np.zeros((self.h, self.w, 1), np.uint8)
            self._bindings = self._cim.create_bindings()
            self._bindings.input().set_buffer(self._in_buf)
            # One buffer per output, keyed by name. XFeat emits three heads
            # where the detector emits one, so the single-output binding the
            # detector uses does not transfer.
            for name in self._out_names():
                buf = np.zeros(tuple(self._model.output(name).shape),
                               np.float32)
                self._out_bufs[name] = buf
                self._bindings.output(name).set_buffer(buf)
        return self._cim

    def close(self) -> None:
        """Hand the network group back. Called by `close_all` at exit."""
        with self._lock:
            self._closed = True
            if self._cim is not None:
                try:
                    self._cim.__exit__(None, None, None)
                except Exception:                                # noqa: BLE001
                    pass
                self._cim = self._bindings = None

    def _out_names(self):
        return [o.name for o in self._model.outputs]

    def _post(self, feats, kpt_logits, rel=None):
        return _x.keypoints_and_descriptors(feats, kpt_logits, self.det_thresh,
                                            self.top_k, rel=rel)

    # The same matcher, deliberately. A checkpoint enrolled through one
    # backend must be matchable through the other, or the bank silently
    # partitions by whichever transport happened to store each entry.
    match = staticmethod(_x.XFeatONNX.match)
