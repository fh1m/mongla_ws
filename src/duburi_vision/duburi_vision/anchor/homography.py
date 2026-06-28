"""homography -- turn a reference->live homography into an AnchorError.

Pure numpy: no torch, no cv2 model, no ROS. The matcher computes the 3x3
homography ``H`` that maps REFERENCE-frame points onto the LIVE frame
(``cv2.findHomography(mkpts_ref, mkpts_cur)``); this module reads the pose
error out of it so it can be unit-tested with hand-built matrices (identity ->
zero error, translation -> tx, rotation -> theta).

Why ref->live (not live->ref): pushing the reference centre through a
reference->live homography gives "where the reference appears in the live
frame" -- the SAME sign convention as a YOLO bbox offset (+x = right of centre).
That makes the anchor control laws sign-identical to the pool-verified
``align_loop`` (positive feedback that closes the loop, not runs from it).
"""

from __future__ import annotations

import math
from typing import Tuple

import numpy as np

from .anchor import AnchorError


def extract_error(H: np.ndarray, frame_shape: Tuple[int, ...],
                  n_inliers: int = 0, confidence: float = 0.0) -> AnchorError:
    """Read (tx_px, ty_px, theta_rad) out of a reference->live homography.

    ``H`` maps a reference-frame pixel to where it lands in the live frame. We
    push the frame CENTRE through ``H``: the displacement is where the
    reference's centre now appears in the live view -- the pose error the
    control loop drives to zero. ``theta`` is the in-plane rotation in ``H``'s
    upper-left 2x2 (atan2(H[1,0], H[0,0])).

    Sign convention (matches the YOLO align loop's ex/ey): +tx = the reference
    appears to the RIGHT of the live centre -> the target is right -> strafe
    RIGHT (Ch6>1500); +ty = below -> descend. So a rightward drift (AUV moved
    right, reference content now left) yields tx<0 -> strafe LEFT, closing the
    loop. A degenerate/None ``H`` yields a zero error so a bad frame is a no-op.
    """
    if H is None or np.asarray(H).shape != (3, 3):
        return AnchorError(0.0, 0.0, 0.0, int(n_inliers), float(confidence))

    H = np.asarray(H, dtype=np.float64)
    h, w = int(frame_shape[0]), int(frame_shape[1])
    cx, cy = w * 0.5, h * 0.5

    # Map the centre through H (homogeneous divide).
    p = H @ np.array([cx, cy, 1.0], dtype=np.float64)
    if abs(p[2]) < 1e-9:
        return AnchorError(0.0, 0.0, 0.0, int(n_inliers), float(confidence))
    mapped_x = p[0] / p[2]
    mapped_y = p[1] / p[2]

    tx = mapped_x - cx
    ty = mapped_y - cy
    theta = math.atan2(H[1, 0], H[0, 0])

    return AnchorError(
        tx_px=float(tx), ty_px=float(ty), theta_rad=float(theta),
        n_inliers=int(n_inliers), confidence=float(confidence))
