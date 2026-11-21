"""Where the floor's tile grid and lane line lie in the picture, for an operator to see.

The tile grating (`tile_grating.measure`) and the lane line (`pool_lines.detect`)
each reduce a frame to a heading, and the flow height to one number. Neither
can be checked by eye from those: a grating locked on a harmonic, or a lane
found on a shadow, publishes a plausible angle. Drawn over the picture, the
same reading is right or wrong at a glance. This turns the two readings back
into image geometry, in the frame's own full-size pixels:

- the grout: two families of parallel lines, each a normal angle and the
  distance along that normal (from the top-left pixel) of one line, a period
  apart. `Grating.phase` belongs to the one carrier the spectrum peaked on,
  and `angle_deg` is folded into [0, 90), so which family that phase was is
  lost; both offsets are measured here by projecting the frame onto each
  family's carrier, the same sum the FFT bin is, at two frequencies only;
- the lane line's centreline, as its direction and its signed distance from
  the frame centre (`pool_lines.LaneLine`), scaled back from the decimated frame.

Called only while a viewer subscribes (flow_floor), at the floor read's
2 Hz, on the frame that reading used. Pure: no ROS here.
"""
from __future__ import annotations

import math

import numpy as np


def grout_lines(gray, period_px: float, angle_deg: float, decimate: int = 1) -> list:
    """[(normal_deg, offset_px)] for each grout family, in full-frame pixels.

    A line of family k is every p with n_k . p = offset + m * period_px.
    `gray` is the full frame; it is decimated here as `measure` does, and the
    two projections run on the small one.
    """
    a = np.asarray(gray, dtype=np.float32)
    d = max(1, int(decimate))
    if d > 1:
        import cv2
        a = cv2.resize(a, None, fx=1.0 / d, fy=1.0 / d, interpolation=cv2.INTER_AREA)
    from mongla_vision.flow.tile_grating import _blank_thick_dark
    a = _blank_thick_dark(a)
    a = a - a.mean()
    h, w = a.shape
    yy, xx = np.mgrid[0:h, 0:w].astype(np.float32)
    p_small = period_px / d
    out = []
    for k in (0.0, 90.0):
        t = math.radians(angle_deg + k)
        nx, ny = math.cos(t), math.sin(t)
        c = np.sum(a * np.exp(-2j * math.pi * (nx * xx + ny * yy) / p_small))
        # Grout is dark: the carrier's minimum, where its phase reaches pi.
        off_small = (p_small * (math.pi - float(np.angle(c))) / (2 * math.pi)) % p_small
        # A small pixel i is the full pixels around (i + 0.5) * d - 0.5 (INTER_AREA).
        off = d * off_small + (0.5 * d - 0.5) * (nx + ny)
        out.append(((angle_deg + k) % 180.0, off % period_px))
    return out


def lane_axis(angle_deg: float, offset_px: float, decimate: int, size) -> tuple:
    """(direction_deg, offset_px from the full frame's centre) of a lane line
    found on the frame decimated by `decimate`. `size` is the full (width, height)."""
    d = max(1, int(decimate))
    t = math.radians(angle_deg)
    nx, ny = -math.sin(t), math.cos(t)
    # The small frame's centre is the full frame's, less the half pixel the
    # resize moves it by, on the normal.
    w, h = size
    cs = ((w // d) * 0.5 + 0.5) * d - 0.5 - w * 0.5, ((h // d) * 0.5 + 0.5) * d - 0.5 - h * 0.5
    return angle_deg % 180.0, offset_px * d + cs[0] * nx + cs[1] * ny


def segment(normal_deg: float, offset: float, size) -> tuple | None:
    """The part of the line n . p = offset inside a (width, height) frame, or None."""
    w, h = size
    t = math.radians(normal_deg)
    nx, ny = math.cos(t), math.sin(t)
    pts = []
    for x in (0.0, float(w - 1)):
        if abs(ny) > 1e-9:
            y = (offset - nx * x) / ny
            if -1e-6 <= y <= h - 1 + 1e-6:
                pts.append((x, y))
    for y in (0.0, float(h - 1)):
        if abs(nx) > 1e-9:
            x = (offset - ny * y) / nx
            if -1e-6 <= x <= w - 1 + 1e-6:
                pts.append((x, y))
    pts = sorted(set((round(x, 3), round(y, 3)) for x, y in pts))
    if len(pts) < 2:
        return None
    return pts[0], pts[-1]


def grid_segments(lines, period_px: float, size) -> list:
    """Every grout line of each family that crosses the frame, as segments."""
    w, h = size
    out = []
    for normal_deg, off in lines:
        t = math.radians(normal_deg)
        nx, ny = math.cos(t), math.sin(t)
        corners = [nx * x + ny * y for x in (0, w - 1) for y in (0, h - 1)]
        lo, hi = min(corners), max(corners)
        m = math.ceil((lo - off) / period_px)
        while off + m * period_px <= hi:
            s = segment(normal_deg, off + m * period_px, size)
            if s:
                out.append(s)
            m += 1
    return out


def lane_segment(direction_deg: float, offset_px: float, size):
    """The lane centreline across a (width, height) frame, or None."""
    w, h = size
    t = math.radians(direction_deg)
    nx, ny = -math.sin(t), math.cos(t)
    return segment(math.degrees(math.atan2(ny, nx)),
                   offset_px + nx * w * 0.5 + ny * h * 0.5, size)
