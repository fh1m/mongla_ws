"""anchor -- the AnchorMatcher ABC and the AnchorError it returns.

One class per file (no base.py): this module owns the abstract interface and
the small result dataclass. Concrete matchers (``XFeatMatcher``) live in their
own files and subclass ``AnchorMatcher`` -- mirrors the Camera/Detector/Tracker
ABC + per-source-class convention used across ``duburi_vision``.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass
class AnchorError:
    """Pixel-plane pose error of the live view vs the stored reference.

    Recovered from the live->reference homography (NOT metric):
      tx_px / ty_px -- translation of the frame centre through the homography
                       (+tx = reference is to the RIGHT of where we are now,
                        +ty = reference is BELOW -- same image convention as
                        the YOLO align loop's ex/ey).
      theta_rad     -- in-plane rotation extracted from the homography's 2x2.
      n_inliers     -- RANSAC inlier count (match strength / confidence proxy).
      confidence    -- inliers / total matches in [0,1] (0 when no matches).
    """
    tx_px:      float
    ty_px:      float
    theta_rad:  float
    n_inliers:  int   = 0
    confidence: float = 0.0


class AnchorMatcher(ABC):
    """Abstract geometric matcher: store a reference, then match against it.

    Contract:
      * ``set_reference(frame_bgr)`` captures the current view as the anchor.
      * ``match(frame_bgr)`` returns an :class:`AnchorError`, or ``None`` when
        no reference is set or too few correspondences survive (caller treats
        ``None`` as "no lock this tick" -- never raises on a bad frame).
      * ``is_loaded()`` is True once the model/weights are ready (distinct from
        whether a reference has been captured -- see ``has_reference``).
    """

    @abstractmethod
    def set_reference(self, frame_bgr: np.ndarray) -> bool:
        """Capture ``frame_bgr`` as the reference. Returns True on success."""
        raise NotImplementedError

    @abstractmethod
    def match(self, frame_bgr: np.ndarray) -> Optional[AnchorError]:
        """Match ``frame_bgr`` against the stored reference, or None."""
        raise NotImplementedError

    @abstractmethod
    def is_loaded(self) -> bool:
        """True once the matcher's model/weights are ready for inference."""
        raise NotImplementedError

    def has_reference(self) -> bool:
        """True once a reference frame has been captured. Override if stateful."""
        return False

    def clear_reference(self) -> None:
        """Drop the stored reference so the next snap starts fresh. Override."""
        return None

    def last_match(self):
        """Most recent (mkpts_ref, mkpts_cur, inlier_mask), or None. Override.

        Full-frame pixel coords; lets the node draw a match overlay on the HUD
        reference inset. Default None for matchers that don't expose it.
        """
        return None
