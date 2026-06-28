"""xfeat -- XFeatMatcher: XFeat + LighterGlue keypoint matching -> homography.

Concrete :class:`AnchorMatcher`. Loads XFeat from torch.hub
(``verlab/accelerated_features``); LighterGlue auto-loads lazily the first
time ``match_lighterglue`` is called (there is no separate hub entrypoint).

Constraint: NO module-level ``import torch`` -- it is imported lazily inside
``__init__`` so that ``anchor_node`` startup (and any import of this module)
does not pull in torch/CUDA when ``anchor:=false``.

API used (verified against upstream hubconf.py + modules/xfeat.py):
    xfeat = torch.hub.load('verlab/accelerated_features','XFeat',
                           pretrained=True, top_k=K)
    d = xfeat.detectAndCompute(bgr)[0]        # {'keypoints','scores','descriptors'}
    d['image_size'] = (W, H)
    mkpts0, mkpts1, idx = xfeat.match_lighterglue(d_ref, d_cur, min_conf=...)
"""

from __future__ import annotations

from typing import Optional

import numpy as np

from .anchor import AnchorError, AnchorMatcher
from .homography import extract_error

# RANSAC reprojection threshold (px) for cv2.findHomography.
_RANSAC_REPROJ_PX = 5.0
# Below this many LighterGlue correspondences we can't fit a stable homography.
_MIN_MATCHES_FOR_H = 8


class XFeatMatcher(AnchorMatcher):
    """XFeat + LighterGlue matcher. Stores one reference; matches against it."""

    def __init__(self, *, top_k: int = 2048, device: str = 'cuda:0',
                 min_conf: float = 0.1, hub_repo: str = 'verlab/accelerated_features',
                 logger=None):
        # Lazy import: keep torch/CUDA out of import time so vision launches
        # with anchor:=false never pay for it.
        import torch  # noqa: F401  (kept for side-effect + device probe)

        self._log      = logger
        self._top_k    = int(top_k)
        self._min_conf = float(min_conf)
        self._torch    = torch

        want_cuda = str(device).startswith('cuda') and torch.cuda.is_available()
        self._device = device if want_cuda else 'cpu'

        # torch.hub.load downloads weights on first run (cached after). The pool
        # has no internet -> pre-download on the Jetson; the canary log below
        # makes a silent re-download obvious (it would hang/log a download).
        self._model = torch.hub.load(
            hub_repo, 'XFeat', pretrained=True, top_k=self._top_k)
        try:
            self._model = self._model.to(self._device)
        except Exception:
            pass  # some builds bind device internally; non-fatal

        self._ref: Optional[dict] = None
        self._loaded = True

        if self._log:
            dev_name = ''
            try:
                if want_cuda:
                    dev_name = f' ({torch.cuda.get_device_name(0)})'
            except Exception:
                pass
            self._log.info(
                f'[ANCHOR] loaded XFeat + LighterGlue on {self._device}{dev_name} '
                f'top_k={self._top_k}')

    # ---- AnchorMatcher ------------------------------------------------- #

    def is_loaded(self) -> bool:
        return self._loaded

    def has_reference(self) -> bool:
        return self._ref is not None

    def clear_reference(self) -> None:
        self._ref = None

    def set_reference(self, frame_bgr: np.ndarray, bbox=None) -> bool:
        """Capture ``frame_bgr`` as the reference.

        ``bbox=(x1,y1,x2,y2)`` restricts the reference to that pixel region (a
        detection's bounding box) so the lock keys on the target, not moving
        background. The keypoints are still expressed in FULL-frame coordinates
        (see ``_describe``), so the homography stays single-coordinate-system
        and ``extract_error`` -- including its sign convention -- is unchanged.
        """
        d = self._describe(frame_bgr, bbox=bbox)
        if d is None:
            return False
        self._ref = d
        if self._log:
            n = int(d['keypoints'].shape[0]) if hasattr(d['keypoints'], 'shape') else 0
            tag = f' (crop {tuple(int(v) for v in bbox)})' if bbox is not None else ''
            self._log.info(f'[ANCHOR] reference captured ({n} keypoints){tag}')
        return True

    def match(self, frame_bgr: np.ndarray) -> Optional[AnchorError]:
        if self._ref is None:
            return None
        cur = self._describe(frame_bgr)
        if cur is None:
            return None

        try:
            mkpts_cur, mkpts_ref, _idx = self._model.match_lighterglue(
                cur, self._ref, min_conf=self._min_conf)
        except TypeError:
            # Older signature without min_conf.
            mkpts_cur, mkpts_ref, _idx = self._model.match_lighterglue(cur, self._ref)
        except Exception as exc:   # noqa: BLE001 -- a bad frame must not raise
            if self._log:
                self._log.warning(f'[ANCHOR] match_lighterglue failed: {exc!r}')
            return None

        mkpts_cur = np.asarray(mkpts_cur, dtype=np.float64)
        mkpts_ref = np.asarray(mkpts_ref, dtype=np.float64)
        n_match = int(mkpts_cur.shape[0])
        if n_match < _MIN_MATCHES_FOR_H:
            return AnchorError(0.0, 0.0, 0.0, n_match,
                               0.0 if n_match == 0 else 1.0)

        import cv2
        # H maps REFERENCE points -> LIVE points (src=ref, dst=cur). Pushing the
        # REFERENCE centre through H then yields "where the reference appears in
        # the live frame" -- a direct analog of a YOLO bbox offset, so the pose
        # error has the SAME sign as align_loop's ex/ey and the control laws are
        # sign-identical to (pool-verified) align_loop. NOTE: src/dst order is
        # load-bearing -- swapping it inverts every axis into POSITIVE feedback
        # (the hull drives away from the lock). Do not "simplify" the arg order.
        H, mask = cv2.findHomography(
            mkpts_ref, mkpts_cur, cv2.RANSAC, _RANSAC_REPROJ_PX)
        if H is None or mask is None:
            return AnchorError(0.0, 0.0, 0.0, n_match, 0.0)

        n_inliers = int(mask.sum())
        confidence = n_inliers / max(n_match, 1)
        return extract_error(H, frame_bgr.shape, n_inliers, confidence)

    # ---- internal ------------------------------------------------------ #

    def _describe(self, frame_bgr: np.ndarray, bbox=None) -> Optional[dict]:
        if frame_bgr is None or getattr(frame_bgr, 'size', 0) == 0:
            return None
        h, w = frame_bgr.shape[:2]

        # Crop reference: describe only the bbox region, then SHIFT the keypoints
        # back into full-frame coordinates by (x1,y1). image_size stays the FULL
        # frame, so the reference is "full-frame keypoints confined to the bbox"
        # -- the homography is single-coordinate-system and extract_error (which
        # pushes the full-frame centre) is byte-identical to the whole-frame
        # path. The crop only changes WHICH keypoints exist, never the frame or
        # any sign. (Crop-local coords would mix origins and corrupt the pose.)
        ox = oy = 0
        described = frame_bgr
        if bbox is not None:
            x1, y1, x2, y2 = (int(round(v)) for v in bbox)
            x1 = max(0, min(x1, w - 1)); x2 = max(x1 + 1, min(x2, w))
            y1 = max(0, min(y1, h - 1)); y2 = max(y1 + 1, min(y2, h))
            described = frame_bgr[y1:y2, x1:x2]
            if described.size == 0:
                described = frame_bgr   # degenerate bbox -> whole frame
            else:
                ox, oy = x1, y1

        try:
            out = self._model.detectAndCompute(described, top_k=self._top_k)[0]
        except Exception as exc:   # noqa: BLE001
            if self._log:
                self._log.warning(f'[ANCHOR] detectAndCompute failed: {exc!r}')
            return None

        if ox or oy:
            kp = np.asarray(out['keypoints'], dtype=np.float64).copy()
            kp[:, 0] += ox
            kp[:, 1] += oy
            out['keypoints'] = kp
        out['image_size'] = (w, h)   # FULL frame (W, H) -- LighterGlue position prior
        return out
