"""overlay -- draw the XFeat match overlay on the HUD reference inset.

Pure (cv2 + numpy only, no ROS): extracted from anchor_node so the green
match-line drawing unit-tests without rclpy / cv_bridge -- mirrors the
crop_gate.py extraction. The operator complaint this answers: the reference
inset showed no green lines, so a snap couldn't be told from a real lock.
"""

from __future__ import annotations

import cv2
import numpy as np


def draw_match_overlay(img, last_match, bbox, min_inliers):
    """Return a copy of the reference frame with the match overlay drawn.

    ``last_match`` = ``(mkpts_ref, mkpts_cur, inlier_mask)`` in FULL-frame pixel
    coords (or None). Each inlier reference keypoint is a green dot with a short
    green vector to its live matched position; outliers are dim. ``bbox`` (a
    crop snap) is outlined so the locked region is visible. An inlier count is
    printed -- green once it clears ``min_inliers`` (a real lock), amber below.
    """
    out = img.copy()
    if bbox is not None:
        x1, y1, x2, y2 = (int(v) for v in bbox)
        cv2.rectangle(out, (x1, y1), (x2, y2), (0, 200, 255), 1)
    n_inliers = 0
    if last_match is not None:
        ref, cur, mask = last_match
        ref = np.asarray(ref); cur = np.asarray(cur)
        m = (np.asarray(mask).reshape(-1).astype(bool)
             if mask is not None else None)
        for i in range(len(ref)):
            inlier = bool(m[i]) if (m is not None and i < len(m)) else True
            rx, ry = int(ref[i][0]), int(ref[i][1])
            if inlier:
                n_inliers += 1
                cv2.circle(out, (rx, ry), 2, (0, 255, 0), -1)
                if i < len(cur):
                    cv2.line(out, (rx, ry),
                             (int(cur[i][0]), int(cur[i][1])), (0, 255, 0), 1)
            else:
                cv2.circle(out, (rx, ry), 2, (120, 120, 120), -1)
    txt = f'matches: {n_inliers} inliers' if last_match is not None else 'no match'
    colour = (0, 255, 0) if n_inliers >= min_inliers else (0, 165, 255)
    cv2.putText(out, txt, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                colour, 1, cv2.LINE_AA)
    return out
