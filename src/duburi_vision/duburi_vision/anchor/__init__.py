"""anchor -- XFeat + LighterGlue geometric "superglue" lock.

Snap a reference frame, then match the live view against it (XFeat keypoints
+ LighterGlue -> homography) to recover a pixel-plane pose error the control
loop drives to zero. Holds the hull on a target for a precise shot even when
the YOLO bbox is lost (target filling/clipping the frame up close).

Public surface:
    AnchorError    -- (tx_px, ty_px, theta_rad, n_inliers, confidence)
    AnchorMatcher  -- ABC: set_reference / match / is_loaded
    XFeatMatcher   -- the real matcher (lazy torch import)
"""

from .anchor import AnchorError, AnchorMatcher

__all__ = ['AnchorError', 'AnchorMatcher']
