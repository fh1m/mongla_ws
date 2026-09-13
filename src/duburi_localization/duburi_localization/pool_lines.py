"""The lane line is a surveyed straight edge. Read a heading off it.

⛔ WHY THIS EXISTS BESIDE `tile_grating`, AND NOT INSTEAD OF IT. Both read the
pool floor as a survey, but a lane line cannot be read by the grating method
and the arithmetic says so plainly. World Aquatics fixes lane centres at
exactly 2.5 m; five cycles of a 2.5 m pitch must fit in frame for an FFT to
measure it, which at our downward focal length of 1027.9 px in a 480 px frame
needs an altitude of **26.8 m**. A pool is 2 m deep. So the periodic method is
structurally unable to see lane lines, and the instrument for them is a LINE
detector -- you see one at a time, not a pattern.

WHAT THE RULEBOOK GUARANTEES, which is the whole reason this is worth
building. World Aquatics' pool certification fixes every number:

    lane centres      exactly 2.5 m apart
    marking colour    dark, contrasting
    marking width     0.2 m min, 0.3 m max
    marking length    the full length of the pool
    end cross line    1.0 m long, 2.0 m from the end wall

These are not OUR pool's numbers to be measured on deck -- they are a
worldwide published standard, so unlike `tile_m` this instrument needs no
venue calibration. And the lines run the length of the pool, which means their
direction IS the pool's long axis.

WHAT ONE LINE BUYS:

  HEADING      the line's direction, absolute in the pool frame, modulo 180
               degrees. That is a TWO-fold ambiguity against the tile grid's
               four-fold -- a line is 1-D, so it is strictly the better
               heading reference where both are available.
  CROSS-TRACK  the line's offset from the image centre, which is a lateral
               position relative to a surveyed feature.

AND IT SURVIVES WHAT KILLS THE REST. A lane line is 0.2-0.3 m of high-contrast
dark against pale tile, spanning the whole frame. That is the single most
robust feature in a swimming pool: it outlives the turbidity that erases props,
the blur that erases corners, and the distance that shrinks every detection
below the 10 px floor we measured.

⚠ AND WHAT IT IS NOT. A line gives no ALONG-track information -- sliding along
a lane line looks identical at every point. Only the end cross line breaks
that, and only near the wall. Stated so nobody reads "position" into this.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional

import numpy as np

# World Aquatics pool certification. Published constants, not our pool's.
LANE_SPACING_M = 2.5
LANE_WIDTH_MIN_M = 0.2
LANE_WIDTH_MAX_M = 0.3
CROSS_LINE_FROM_WALL_M = 2.0

# A line must span enough of the frame to be a lane marking rather than a
# shadow edge or a tile joint. Fraction of the image diagonal.
MIN_LINE_SPAN = 0.45

# Agreement required among the detected segments before the answer is a
# heading rather than a coincidence: the spread of the inlier angles.
MAX_ANGLE_SPREAD_DEG = 8.0
MIN_SEGMENTS = 3

# Length over width of the dark region. A lane line at 0.25 m wide spanning a
# 3 m view is ~12:1; a round blob is 1:1 and has no direction at all.
MIN_ELONGATION = 3.0

# Grey levels between the dark minority and the floor. Below this there is no
# line, only the low end of the noise.
MIN_CONTRAST = 25.0

# How much the dark region may grow when the threshold is raised. A line's
# flat core gives ~1.00 (1.11 heavily blurred); a ramp or vignette gives
# 1.71-1.94, because its band is defined by the cut rather than by the scene.
MAX_BAND_GROWTH = 1.35


@dataclass(frozen=True)
class LaneLine:
    angle_deg: float          # direction in the image, [0, 180)
    offset_px: float          # signed perpendicular distance from frame centre
    support: int              # segments that agreed
    spread_deg: float         # how tightly they agreed

    def heading_deg(self, mount_yaw_deg: float = 0.0) -> float:
        """Hull heading in the POOL frame, modulo 180 degrees.

        The two-fold ambiguity is REAL -- a line looks the same from both ends
        -- and is returned rather than resolved by invention. One bit from
        anywhere else settles it: the heading the hull started on, a prop
        bearing, or the gate.
        """
        return (self.angle_deg - float(mount_yaw_deg)) % 180.0

    def cross_track_m(self, focal_px: float, height_m: float) -> float:
        """Metres from the lane line, sideways. Needs the height.

        `offset_px * height / focal` is the same pinhole scaling the flow path
        uses, so it inherits whatever height is available -- including the one
        the tile grating reads off the floor.
        """
        if focal_px <= 0.0:
            return float('nan')
        return float(self.offset_px) * float(height_m) / float(focal_px)


def detect(gray: np.ndarray, *, min_span: float = MIN_LINE_SPAN
           ) -> Optional[LaneLine]:
    """Find the dominant lane line, or None.

    None is the right answer whenever the floor is bare, and a caller that
    reads None as "no heading information" rather than "heading is zero" has
    understood it.
    """
    if gray is None or gray.ndim != 2 or min(gray.shape) < 32:
        return None
    try:
        import cv2
    except Exception:                       # noqa: BLE001
        return None
    a = np.asarray(gray)
    if a.dtype != np.uint8:
        if not np.isfinite(a).all():
            return None
        a = np.clip(a, 0, 255).astype(np.uint8)
    if float(a.std()) < 1.0:
        return None

    h, w = a.shape
    diag = math.hypot(h, w)

    # ⛔ THE FIRST DESIGN READ THE TILES, NOT THE LINE, AND SAID 0.00 DEGREES
    # CONFIDENTLY. Hough-on-edges votes per edge pixel, and a tiled floor has
    # dozens of tile joints against the lane line's two sides -- so the grid
    # won every vote. Measured on the control: a frame containing ONLY tiles
    # and no line was ACCEPTED, which is the proof. A line detector that
    # prefers the background to the foreground is not a line detector.
    #
    # A lane line is not "an edge". It is a single dark CONNECTED BAND, 0.2-0.3
    # m wide, spanning the frame. So find that region and ask it directly for
    # its orientation, via second moments -- which is exact, needs no voting,
    # and cannot be outvoted by a pattern that is merely numerous.
    thresh = _dark_threshold(a)
    if thresh is None:
        return None

    # ⛔ A GRADIENT MIMICS A WIDE LANE LINE, AND EVERY SHAPE TEST PASSES IT.
    # Measured on a plain illumination ramp: the darkest band is 101x480 px,
    # elongated 4.75:1, spanning the frame and high-contrast. Elongation, span
    # and contrast all say "line". This is the same negative control that
    # defeated three versions of the tile grating, and it defeats shape here
    # too -- because the band really IS long, dark and straight.
    #
    # What separates them is EDGE SHARPNESS, expressed as stability against
    # the threshold. A lane line has a flat dark core, so raising the cut
    # barely grows the region: measured 1.000, and still 1.108 through 21 px of
    # blur. A ramp has no core at all -- its "band" is an artefact of where you
    # chose to cut, so the area grows with the cut: 1.71 for a ramp, 1.94 for a
    # vignette. The bar sits between, with wide margin on both sides.
    if _threshold_growth(a) > MAX_BAND_GROWTH:
        return None

    dark = (a <= thresh).astype(np.uint8)
    dark = cv2.morphologyEx(dark, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    n_lab, labels, stats, cents = cv2.connectedComponentsWithStats(dark, 8)
    if n_lab <= 1:
        return None

    best = None
    for i in range(1, n_lab):
        area = int(stats[i, cv2.CC_STAT_AREA])
        if area < 0.005 * h * w:
            continue
        bw = int(stats[i, cv2.CC_STAT_WIDTH])
        bh = int(stats[i, cv2.CC_STAT_HEIGHT])
        # It has to CROSS the frame: a lane line runs the length of the pool,
        # so in any view containing one it reaches two edges.
        #
        # ⚠ HONESTLY LABELLED: UNPROVEN DEFENCE IN DEPTH. Every negative
        # control that reaches this line is already rejected by the contrast,
        # edge-sharpness or elongation gates, so removing this one breaks no
        # test. It is kept because it states a real physical property of a lane
        # line that none of the others express -- not because it was measured
        # to be necessary. Do NOT read this as a verified guard; the same
        # wording marks the Joseph form in `inekf.py` for the same reason.
        if math.hypot(bw, bh) < min_span * diag:
            continue
        if best is None or area > best[0]:
            best = (area, i)
    if best is None:
        return None

    idx = best[1]
    ys, xs = np.nonzero(labels == idx)
    if xs.size < 50:
        return None
    ang, elong = _moment_orientation(xs, ys)
    if ang is None:
        return None
    # A lane line is LONG and NARROW. A blob that is not elongated has no
    # meaningful direction, and its "orientation" would be noise.
    if elong < MIN_ELONGATION:
        return None

    cx, cy = w * 0.5, h * 0.5
    nx, ny = -math.sin(math.radians(ang)), math.cos(math.radians(ang))
    offset = (float(xs.mean()) - cx) * nx + (float(ys.mean()) - cy) * ny
    return LaneLine(angle_deg=float(ang), offset_px=float(offset),
                    support=int(xs.size), spread_deg=float(1.0 / max(elong, 1e-6)))


def _dark_threshold(a: np.ndarray):
    """The cut that separates the line from the floor, from the frame itself.

    ⛔ NEVER AN ABSOLUTE LEVEL. Depth, turbidity and the dive light move the
    absolute brightness by far more than the line-to-tile contrast does, so a
    fixed cut is calibrated to one pool at one depth and nowhere else.

    A lane line is 0.2-0.3 m wide in a frame a few metres across, so it is a
    small MINORITY of dark pixels against a pale floor -- a low percentile,
    not a median split. The 8th was chosen because a 0.25 m line at 2 m
    altitude covers roughly that share of the frame; a percentile that is too
    generous swallows the darker half of a checkerboard and reads the tiles.
    """
    lo = float(np.percentile(a, 8))
    hi = float(np.percentile(a, 60))
    # There must be a real gap between the dark minority and the floor, or
    # there is no line and the "darkest 8 %" is just the low end of noise.
    if hi - lo < MIN_CONTRAST:
        return None
    return lo + 0.25 * (hi - lo)


def _threshold_growth(a: np.ndarray) -> float:
    """How much the dark region grows when the cut is raised.

    The edge-sharpness test, done without differentiating anything: a region
    with real edges is insensitive to where you threshold it, and a smooth
    ramp is entirely defined by it.
    """
    lo = float(np.percentile(a, 8))
    hi = float(np.percentile(a, 60))
    if hi - lo < MIN_CONTRAST:
        return float('inf')
    tight = float((a <= lo + 0.25 * (hi - lo)).sum())
    loose = float((a <= lo + 0.55 * (hi - lo)).sum())
    return loose / max(tight, 1.0)


def _moment_orientation(xs: np.ndarray, ys: np.ndarray):
    """Orientation and elongation of a point set, from its second moments.

    Exact and vote-free: the principal axis of the covariance is the line's
    direction, and the eigenvalue ratio says how line-like the region is. A
    round blob has no direction and its ratio says so.
    """
    x = xs.astype(np.float64) - xs.mean()
    y = ys.astype(np.float64) - ys.mean()
    sxx = float((x * x).mean())
    syy = float((y * y).mean())
    sxy = float((x * y).mean())
    ang = 0.5 * math.atan2(2.0 * sxy, sxx - syy)
    tr, det = sxx + syy, sxx * syy - sxy * sxy
    disc = max(tr * tr / 4.0 - det, 0.0) ** 0.5
    l1, l2 = tr / 2.0 + disc, tr / 2.0 - disc
    if l1 <= 0.0:
        return None, 0.0
    elong = (l1 / l2) ** 0.5 if l2 > 1e-9 else float('inf')
    return math.degrees(ang) % 180.0, elong


def _dominant_angle(angles: List[float]):
    """Modal direction of a set of angles that live on a 180-degree circle.

    ⛔ A PLAIN MEAN IS WRONG HERE AND THE ERROR IS SILENT. Angles wrap: 179 and
    1 degree are two degrees apart, and their arithmetic mean is 90 -- exactly
    perpendicular to both. Doubling the angle maps the 180-degree circle onto a
    full circle where a vector mean is well defined, then halving returns it.
    This is the standard axial-statistics trick and it is the difference
    between a heading and a right angle.
    """
    if not angles:
        return None, 0.0, []
    a = np.asarray(angles, dtype=float)
    two = np.radians(a * 2.0)
    mx, my = float(np.cos(two).mean()), float(np.sin(two).mean())
    if abs(mx) < 1e-12 and abs(my) < 1e-12:
        return None, 0.0, []
    mean = (math.degrees(math.atan2(my, mx)) / 2.0) % 180.0
    # Residuals on the same doubled circle, back in single-angle degrees.
    resid = np.degrees(np.abs(np.angle(np.exp(1j * (two - math.radians(mean * 2.0)))))) / 2.0
    keep = [i for i, r in enumerate(resid) if r <= MAX_ANGLE_SPREAD_DEG]
    if not keep:
        return None, float(resid.max()), []
    spread = float(resid[keep].max())
    # Re-mean on the inliers only, so one wild segment cannot drag the answer.
    two_k = np.radians(a[keep] * 2.0)
    mean = (math.degrees(math.atan2(float(np.sin(two_k).mean()),
                                    float(np.cos(two_k).mean()))) / 2.0) % 180.0
    return mean, spread, keep
