"""XFeat on the vehicle: ONNX + numpy, no torch.

WHY XFEAT AND NOT ORB -- measured on our own footage, not chosen from a paper.
Reference snapped at 40 % of each clip, matched FRAME-TO-REFERENCE at +1/3/5/8 s,
`USAC_MAGSAC` homography, counting inliers (a homography needs ~15 to be
trusted):

    clip                    ORB ref kp   ORB     XFeat
    Mirpur torpedo (murky)         71    0/4     4/4   (350 -> 100 inliers)
    Mirpur torpedo_1                7    0/4     4/4   (301 -> 130)
    Mirpur gate                   111    3/4     4/4   (753 -> 682)
    octagon (low texture)        1205    1/4     3/4
    torpedo (clear control)      1260    4/4     4/4

ORB finds SEVEN keypoints in an entire Mirpur frame. EdgePoint2 -- published at
"2x faster than XFeat with competitive IMC2022 results" -- was benched too and
loses on every murky clip (torpedo_1: XFeat 4/4, EP2-S64 1/4). IMC2022 is clear
natural imagery; turbid water is a different domain and the ranking does not
survive the move.

WHY IT FITS. ONNX, 2.7 MB, measured on the Pi with the vision stack running:

    640x480   145.6 ms (1 thread)  /  88.0 ms (3)
    320x240    33.1 ms (1 thread)  /  18.2 ms (3)   <- 30 Hz on ONE core

and 320x240 keeps the lock (every murky clip still 4/4). So this runs at the
anchor's 5-10 Hz inside a fraction of one spare core, while the detector keeps
the Hailo and the fast rung keeps LK.

THE POST-PROCESSING IS XFEAT'S OWN, PORTED. The exported graph stops at the
three heads (`feats` 64xH/8xW/8, `keypoints` 65 channels, `heatmap`); keypoint
decoding, NMS, sampling and normalisation happen here in numpy. They are ported
from `modules/xfeat.py` rather than reinvented -- a softmax over 65 channels
with the dustbin dropped, pixel-shuffled 8x8, is not something to guess at.
"""
from __future__ import annotations

import os
from typing import Optional, Tuple

import numpy as np

# 65 = an 8x8 cell of sub-positions plus one dustbin. Dropping the dustbin
# AFTER the softmax is load-bearing: it is what makes "no keypoint here" a
# real outcome instead of forcing probability onto a pixel.
_CELL = 8


def _softmax(x: np.ndarray, axis: int) -> np.ndarray:
    x = x - x.max(axis=axis, keepdims=True)
    np.exp(x, out=x)
    return x / x.sum(axis=axis, keepdims=True)


def _pixel_shuffle_scores(kpt_logits: np.ndarray) -> np.ndarray:
    """(65, h, w) logits -> (h*8, w*8) keypoint score map. XFeat's own layout."""
    s = _softmax(kpt_logits.astype(np.float32), axis=0)[:_CELL * _CELL]
    c, h, w = s.shape
    # (64,h,w) -> (h,w,8,8) -> (h,8,w,8) -> (h*8, w*8)
    s = s.transpose(1, 2, 0).reshape(h, w, _CELL, _CELL)
    return s.transpose(0, 2, 1, 3).reshape(h * _CELL, w * _CELL)


def _nms(score: np.ndarray, thresh: float, k: int = 5) -> np.ndarray:
    """Local-maximum suppression -> (N,2) xy. Same 5x5 window XFeat uses."""
    import cv2
    mx = cv2.dilate(score, np.ones((k, k), np.uint8))
    ys, xs = np.nonzero((score == mx) & (score > thresh))
    return np.stack([xs, ys], axis=1)


class XFeatONNX:
    """Detect keypoints + 64-D descriptors, and match two views.

    One session, reused. `intra_op_num_threads` is 1 by default so this cannot
    contend with the detector or the camera pumps -- the whole point of putting
    it on the spare cores is that it stays out of the fast path.
    """

    def __init__(self, model_path: str, *, top_k: int = 1024,
                 det_thresh: float = 0.05, threads: int = 1, logger=None):
        import onnxruntime as ort
        if not os.path.exists(model_path):
            raise FileNotFoundError(
                f'XFeat ONNX not found: {model_path}. Export it with '
                f'tools/xfeat_export.py -- the anchor cannot run without it.')
        so = ort.SessionOptions()
        so.intra_op_num_threads = int(max(1, threads))
        so.inter_op_num_threads = 1
        self._s = ort.InferenceSession(model_path, so,
                                       providers=['CPUExecutionProvider'])
        self._in = self._s.get_inputs()[0].name
        shape = self._s.get_inputs()[0].shape
        # The export is FIXED-SHAPE. Feeding another size does not resize, it
        # throws -- so the node must letterbox to this, and knowing it here is
        # what lets it say so instead of failing at the first frame.
        self.h, self.w = int(shape[2]), int(shape[3])
        self.top_k = int(top_k)
        self.det_thresh = float(det_thresh)
        self._log = logger

    # -- detection ---------------------------------------------------------- #
    def detect(self, gray: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """(H,W) uint8 -> (kpts (N,2) xy in INPUT pixels, desc (N,64) L2-normed)."""
        import cv2
        if gray.shape[:2] != (self.h, self.w):
            gray = cv2.resize(gray, (self.w, self.h))
        x = gray.astype(np.float32)[None, None] / 255.0
        feats, kpt_logits, _heat = self._s.run(None, {self._in: x})

        score = _pixel_shuffle_scores(kpt_logits[0])
        kpts = _nms(score, self.det_thresh)
        if len(kpts) == 0:
            return np.zeros((0, 2), np.float32), np.zeros((0, 64), np.float32)
        if len(kpts) > self.top_k:
            s = score[kpts[:, 1], kpts[:, 0]]
            kpts = kpts[np.argsort(-s)[:self.top_k]]

        # Descriptors live at 1/8 resolution; sample them at the keypoints.
        #
        # BILINEAR, NOT NEAREST. Nearest-neighbour was tried first and it is
        # not a rounding detail: measured against torch XFeat on the same
        # frames it returned ~55 % of the inliers (Mirpur gate 119 vs 260,
        # clear torpedo 262 vs 445) while finding the SAME 1024 keypoints and
        # still clearing the 15-inlier bar -- i.e. it degraded quietly, which
        # is the only way this kind of port ever fails. A keypoint sits
        # anywhere inside its 8x8 cell; snapping its descriptor to the cell
        # centre throws away exactly the sub-cell precision the matcher scores
        # on.
        f = feats[0]                                   # (64, h/8, w/8)
        _, fh, fw = f.shape
        # keypoint pixel -> continuous descriptor-grid coordinate (cell centres
        # are at 0.5 in grid units, hence the -0.5 after scaling)
        gxf = np.clip(kpts[:, 0] / _CELL - 0.5, 0, fw - 1)
        gyf = np.clip(kpts[:, 1] / _CELL - 0.5, 0, fh - 1)
        x0 = np.floor(gxf).astype(np.int32); x1 = np.minimum(x0 + 1, fw - 1)
        y0 = np.floor(gyf).astype(np.int32); y1 = np.minimum(y0 + 1, fh - 1)
        wx = (gxf - x0).astype(np.float32); wy = (gyf - y0).astype(np.float32)
        desc = (f[:, y0, x0].T * ((1 - wx) * (1 - wy))[:, None]
                + f[:, y0, x1].T * (wx * (1 - wy))[:, None]
                + f[:, y1, x0].T * ((1 - wx) * wy)[:, None]
                + f[:, y1, x1].T * (wx * wy)[:, None]).astype(np.float32)
        n = np.linalg.norm(desc, axis=1, keepdims=True)
        np.divide(desc, np.maximum(n, 1e-8), out=desc)
        return kpts.astype(np.float32), desc

    # -- matching ----------------------------------------------------------- #
    @staticmethod
    def match(d0: np.ndarray, d1: np.ndarray, min_cossim: float = 0.82):
        """MUTUAL nearest neighbour on cosine similarity.

        Mutual, not one-way: a one-way match will happily map every keypoint of
        a blank frame onto whichever reference point is least dissimilar, which
        produces a full match list and a confident, meaningless homography.
        """
        if len(d0) == 0 or len(d1) == 0:
            return np.zeros(0, np.int64), np.zeros(0, np.int64)
        sim = d0 @ d1.T
        i12 = sim.argmax(axis=1)
        i21 = sim.argmax(axis=0)
        idx0 = np.arange(len(i12))
        mutual = i21[i12] == idx0
        idx0 = idx0[mutual]
        idx1 = i12[idx0]
        keep = sim[idx0, idx1] >= min_cossim
        return idx0[keep], idx1[keep]
