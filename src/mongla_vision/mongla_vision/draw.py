"""draw -- the detector's debug image and the display node's HUD frame.

`render_all()` returns an annotated copy of a frame: boxes, labels, track ids,
the reticle and the alignment overlay. Stats (FPS, vehicle state) are printed
to the terminal by the display node at 1 Hz rather than drawn, to keep the
frame cheap.

Implementation: `draw_widgets.py` (palette, micro-widgets) and
`draw_video.py` (the overlays).
"""

from __future__ import annotations

from typing import List, Optional

import numpy as np

from .draw_video import render_video_section
from .detection.detector import Detection, largest


def render_all(frame_bgr: np.ndarray,
               detections: List[Detection], *,
               show_reticle: bool = True,
               deadband: float = 0.05,
               primary: Optional[Detection] = None,
               healthy: bool = True,
               track_ids=None,
               draw_trail: bool = True,
               trail_key: str = 'main',
               ) -> np.ndarray:
    """Return the annotated frame. ``draw_trail``/``trail_key`` control the
    bounded primary-target trail."""
    primary = primary or largest(detections)
    return render_video_section(
        frame_bgr, detections,
        show_reticle=show_reticle,
        deadband=deadband,
        primary=primary,
        healthy=healthy,
        track_ids=track_ids,
        draw_trail=draw_trail,
        trail_key=trail_key,
    )
