"""draw -- thin dispatcher for the mission-control HUD.

Public surface:
  render_all()  -- annotated video frame (bboxes + alignment overlays)

All colour constants and standalone overlay functions are re-exported from
the implementation modules so that external callers continue to work.

Stats (FPS, detection info, vehicle state) are emitted to the terminal by
VisionDisplayNode at 1 Hz — not rendered on-frame to keep FPS high.

Implementation:
  draw_widgets.py  -- palette constants, micro-widgets
  draw_video.py    -- video overlays (reticle, boxes, hairlines, arrows)
  draw_strip.py    -- (unused; kept for reference)
"""

from __future__ import annotations

from typing import List, Optional

import numpy as np

from .draw_widgets import (
    C_BG, C_ACCENT, C_AMBER, C_OK, C_ERR, C_TEXT, C_DIM, C_BORDER,
)
from .draw_video import render_video_section, draw_track_ids
from .detection.detector import Detection, largest

# Legacy colour aliases kept for any external callers
C_RETICLE       = (50, 55, 55)
COLOR_PRIMARY   = C_ACCENT
COLOR_SECONDARY = C_DIM
COLOR_RETICLE   = C_RETICLE
COLOR_OFFSET    = C_AMBER
COLOR_OK        = C_OK
COLOR_WARN      = C_AMBER
COLOR_ERR       = C_ERR
COLOR_BG        = C_BG
COLOR_FG        = C_TEXT


def render_all(frame_bgr: np.ndarray,
               detections: List[Detection], *,
               show_reticle: bool = True,
               show_alignment: bool = True,
               deadband: float = 0.05,
               primary: Optional[Detection] = None,
               healthy: bool = True,
               track_ids=None,
               vis_range_values: Optional[List[float]] = None,
               depth_map_bgr=None,
               anchor_state=None,
               anchor_error=None,
               anchor_ref_bgr=None,
               **_ignored,
               ) -> np.ndarray:
    """Return annotated video frame with bbox + alignment overlays.

    Strip kwargs (fps, state, sparklines, etc.) are accepted but ignored —
    stats are printed to the terminal by VisionDisplayNode at 1 Hz instead.
    """
    primary = primary or largest(detections)
    return render_video_section(
        frame_bgr, detections,
        show_reticle=show_reticle,
        deadband=deadband,
        primary=primary,
        healthy=healthy,
        track_ids=track_ids,
        vis_range_values=vis_range_values,
        depth_map_bgr=depth_map_bgr,
        anchor_state=anchor_state,
        anchor_error=anchor_error,
        anchor_ref_bgr=anchor_ref_bgr,
    )
