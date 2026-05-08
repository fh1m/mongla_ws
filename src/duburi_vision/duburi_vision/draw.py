"""draw -- thin dispatcher for the mission-control HUD.

Public surface:
  render_all()  -- full frame + strip composite (main entry point)

All colour constants, widget helpers, and standalone overlay functions are
re-exported from the four implementation modules so that callers that import
specific symbols from draw.py continue to work unchanged.

Implementation is split across:
  draw_widgets.py  -- palette constants, micro-widgets (gauges, sparklines, …)
  draw_video.py    -- video-section overlays (reticle, boxes, hairlines, …)
  draw_strip.py    -- 6-row UI strip assembly
"""

from __future__ import annotations

from typing import Dict, List, Optional, Sequence

import numpy as np

from .draw_widgets import (
    C_BG, C_ACCENT, C_AMBER, C_OK, C_ERR, C_TEXT, C_DIM, C_BORDER,
)
from .draw_video import (
    render_video_section,
    draw_detections,
    draw_track_ids,
)
from .draw_strip import render_ui_strip, _STRIP_H
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
               source: str = '?',
               fps: float = 0.0,
               healthy: bool = True,
               show_reticle: bool = True,
               show_alignment: bool = True,
               deadband: float = 0.05,
               primary: Optional[Detection] = None,
               tracking_on: bool = False,
               n_tracks: int = 0,
               primary_track_id=None,
               state=None,
               configured_classes: Optional[List[str]] = None,
               track_ids=None,
               yaw_source: Optional[str] = None,
               err_x_history: Optional[Sequence[float]] = None,
               err_y_history: Optional[Sequence[float]] = None,
               conf_history:  Optional[Sequence[float]] = None,
               video_mode: bool = False,
               is_paused: bool = False,
               video_position: Optional[tuple] = None,
               pipeline_health: Optional[Dict[str, bool]] = None,
               ) -> np.ndarray:
    """Return np.vstack([annotated_video, ui_strip]).

    Output height is frame_h + _STRIP_H (202 px).  The video section carries
    visual overlays only; all panels and instruments live in the strip.
    """
    primary = primary or largest(detections)

    video_out = render_video_section(
        frame_bgr, detections,
        show_reticle=show_reticle,
        deadband=deadband,
        primary=primary,
        healthy=healthy,
        track_ids=track_ids,
    )

    h, w = video_out.shape[:2]

    strip = render_ui_strip(
        w, h,
        detections=detections,
        primary=primary,
        source=source,
        fps=fps,
        healthy=healthy,
        tracking_on=tracking_on,
        n_tracks=n_tracks,
        primary_track_id=primary_track_id,
        configured_classes=configured_classes or [],
        state=state,
        yaw_source=yaw_source or '',
        deadband=deadband,
        show_alignment=show_alignment,
        err_x_history=err_x_history,
        err_y_history=err_y_history,
        conf_history=conf_history,
        video_mode=video_mode,
        is_paused=is_paused,
        video_position=video_position,
        pipeline_health=pipeline_health,
    )

    return np.vstack([video_out, strip])
