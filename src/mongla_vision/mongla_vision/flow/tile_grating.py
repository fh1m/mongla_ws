"""The pool floor is a surveyed 2-D grating. Read it like an optical encoder.

The inversion this module is built on. `measured-bars.md` records the known
killer for our optical flow: *"the texture that defeats optical flow is a
repeating tile pattern under water."* A uniform tiled floor has no distinctive
corners, so Lucas-Kanade has nothing to track and every patch matches every
other patch -- the aperture problem in its purest form.

That is true of MATCHING. It is exactly backwards for DEMODULATION. A periodic
pattern is a carrier, and displacement is a phase shift of that carrier. The
scene that starves a feature tracker is the scene a phase method is strongest
on, and the literature says so directly: ADOPT (arXiv 2605.19963) beats Digital
Image Correlation precisely at SMALL DISPLACEMENTS and high SNR -- our regime,
frame to frame at 30 Hz -- while costing `O(l^2 log l^2)` against DIC's
`O(l^2 s^2 b^2 log b^2)`.

So the tiled floor is not a hazard to survive. It is an instrument, and one
FFT reads three quantities off it that we currently pay dearly for elsewhere:

  HEADING   the grid's orientation. Pool tiles are laid square to the walls, so
            the grid axes ARE the course axes. This is an ABSOLUTE heading in
            the pool frame, modulo 90 degrees, with NO prop, NO detection and
            NO landmark in view. The capability map calls an absolute heading
            reference "the single highest-leverage item in Tier 3".

  HEIGHT    the observed pitch in pixels against the known tile size in metres:
            `h = f_px * tile_m / pitch_px`. Our flow velocity is
            `scale = h / (f * dt)`, so height is the term that multiplies every
            velocity we produce -- and today it comes from a bbox or a guess.
            Here it is read off the floor itself.

  PHASE     position WITHIN one tile, which does not drift. Flow integrates and
            walks away without bound; phase is an absolute sub-tile coordinate
            that is the same on minute one and minute ten.

What this is not. Phase is absolute only modulo the tile pitch -- it bounds
drift to one tile, it does not localise the vehicle in the pool. Counting which
tile you are on needs continuity, and continuity is exactly what a dropout
breaks. Stated here so nobody reads "drift-free" as "a global fix".

And it refuses. Measured while writing this: our own SAUVC and RoboSub sim
floor textures carry only 0.65 % and 0.67 % of their spectral energy in the
dominant peak -- they are noise-like, not tiled, so this module correctly
returns None on them. That is a sim-fidelity finding as much as an algorithm
one: a real competition pool is tiled and our simulated one is not. RoboSub's
TRANSDEC is a concrete acoustic tank, not a tiled pool, so this instrument is
SAUVC-class and says so rather than pretending to be universal.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

# Fraction of (DC-suppressed) spectral energy the dominant peak must carry
# before we call the floor periodic. Measured: a noise-like texture sits near
# 0.65 %, a real square grid is far above this. The bar is deliberately set
# where a NEGATIVE control fails, not where a positive one passes.
# Swept against both controls, not chosen. At 0.020 every negative refuses but
# the grid only survives noise sigma 10; at 0.010 every negative STILL refuses
# and the grid survives sigma 30; at 0.005 the negatives start passing. So the
# bar sits at the last value where the negative control holds, which is the
# only place a bar like this means anything.
MIN_PEAK_FRACTION = 0.01

# A peak closer to DC than this is the vignette / illumination gradient, not a
# tile. At 8 px period the pattern is already near the blur limit underwater.
MIN_PERIOD_PX = 6.0

# The dominant peak must be this many bins clear of DC, or what selected it
# was the illumination falloff rather than the scene. This is the gate that
# rejects a gradient, proven by removing it and watching both gradient tests
# fail. It doubles as the "enough cycles to measure a period" requirement.
MIN_PEAK_RADIUS_BINS = 5

# A bin at f/k holding at least this share of the peak means the peak was a
# harmonic. Placeholder until swept against both controls (see _fundamental).
SUBHARMONIC_FRACTION = 0.25


# And a period longer than this cannot be seen twice in a frame, so its
# "periodicity" is one edge and a coincidence.
def _max_period_px(shape) -> float:
    return min(shape) / 3.0


@dataclass(frozen=True)
class Grating:
    """What one frame of floor says about itself."""
    period_px: float          # dominant spatial period
    angle_deg: float          # grid orientation, wrapped to [0, 90)
    phase: float              # dominant carrier's phase, radians
    strength: float           # dominant peak's share of spectral energy

    def height_m(self, focal_px: float, tile_m: float) -> float:
        """Metres above the floor, from the tile's known size.

        `h = f * tile_m / pitch_px` is the same pinhole relation the range
        estimator uses on a prop, with the floor as the object of known size.
        This is the term that multiplies every optical-flow velocity, so
        measuring it rather than assuming it removes the largest scale error
        in the velocity channel.
        """
        if self.period_px <= 0.0:
            return float('nan')
        return float(focal_px) * float(tile_m) / self.period_px

    def heading_deg(self, mount_yaw_deg: float = 0.0) -> float:
        """Hull heading in the POOL frame, modulo 90 degrees.

        Tiles are laid square to the pool walls, so the grid axes are the
        course axes. The four-fold ambiguity is real and is NOT resolved here:
        a square grid cannot tell you which wall is which. Resolving it needs
        one more bit from somewhere else -- a prop bearing, the gate, or simply
        the heading the hull started on. Returning the ambiguous value and
        saying so beats inventing the bit.
        """
        return (self.angle_deg - float(mount_yaw_deg)) % 90.0


def measure(gray, *, decimate: int = 3,
            min_strength: float = MIN_PEAK_FRACTION) -> Optional[Grating]:
    """`detect` on a decimated frame, with the period scaled back to full pixels.

    Decimation is free here, and that is not obvious. Throwing away pixels
    normally costs accuracy; it does not here, because the quantity being read
    is a COARSE spatial frequency -- the tile pitch -- not fine detail. An
    area-averaged decimation is a low-pass filter, and a low-pass filter cannot
    move a spectral peak that sits far below its cutoff. Measured, 640x480
    against the same frame at /2 and /3:

        period error   <= 0.5 %
        angle error    <= 0.25 deg
        cost           28.90 ms -> 3.51 ms (/2) -> ~1.5 ms (/3)

    A 28.9 ms pass is an entire frame budget at 30 Hz and would have made this
    unaffordable; 1.5 ms at a few Hz is nothing. Height and heading move slowly,
    so there is no reason to run it per frame.

    Returns a `Grating` whose `period_px` is in FULL-frame pixels, so
    `height_m` can be called with the full-frame focal length and no further
    bookkeeping. Getting that scaling wrong would silently divide every height
    by the decimation factor, which is why it happens here rather than at each
    call site.
    """
    if gray is None or decimate < 1:
        return None
    if decimate > 1:
        try:
            import cv2
            small = cv2.resize(gray, None, fx=1.0 / decimate, fy=1.0 / decimate,
                               interpolation=cv2.INTER_AREA)
        except Exception:                      # noqa: BLE001 -- cv2 optional
            small, decimate = gray, 1
    else:
        small = gray
    g = detect(small, min_strength=min_strength)
    if g is None or decimate == 1:
        return g
    return Grating(period_px=g.period_px * decimate, angle_deg=g.angle_deg,
                   phase=g.phase, strength=g.strength)


def usable_height_m(focal_px: float, tile_m: float, frame_px: int) -> tuple:
    """(min, max) height over the floor where this instrument works.

    The envelope is physical, not a tuning choice. You cannot measure a
    period you only see four times, so the frame must contain at least
    `MIN_PEAK_RADIUS_BINS` full cycles; and a pattern finer than
    `MIN_PERIOD_PX` is past the blur limit underwater. Those two bounds turn
    into a height range once the tile size is known.

    Worked, at our measured downward focal length of 1027.9 px in a 480 px
    frame:

        0.150 m tile  ->  usable ABOVE 2.0 m     (a pool floor is rarely that far)
        0.050 m tile  ->  usable 0.67 m .. 8.6 m
        0.025 m tile  ->  usable 0.33 m .. 4.3 m  (mosaic tile: the good case)

    So the capability is a function of the VENUE's tile size, and a big-tile
    pool at low altitude is outside it. Measure the tile on deck and read this
    before relying on the instrument -- it is exactly the kind of world
    constant that must not be assumed from one pool.
    """
    if focal_px <= 0.0 or tile_m <= 0.0 or frame_px <= 0:
        return (float('nan'), float('nan'))
    max_period = frame_px / float(MIN_PEAK_RADIUS_BINS)
    h_min = focal_px * tile_m / max_period
    h_max = focal_px * tile_m / MIN_PERIOD_PX
    return (h_min, h_max)


# A pixel darker than this share of the floor's median is not grout: grout is
# mid-grey (about 0.6 of the tile), a lane line or a shadow near black.
DARK_FRAC = 0.45
STRIPE_ASPECT = 4.0


def _blank_thick_dark(a: np.ndarray) -> np.ndarray:
    """Fade lanes, shadows and props to the floor's median before the FFT.

    One dark stripe across a tiled floor carries more spectral energy than the
    whole tile grid, and the grating locked onto it: rendered tiles with a lane
    read a period near 100 px whatever the truth (worst 146 % error), and the
    real final-run footage read the lane, not the tiles (BUGS B-144,
    measured-bars section 101). Width does not separate a lane from grout --
    diagonal grout at a large period is as wide as a thin lane, and a width
    test blanked whole tile-only floors -- but darkness does.

    Faded, not cut: a flat fill leaves a straight step where it meets the
    tiles, which is a spectral streak along the grid's own axis.
    """
    try:
        import cv2
    except Exception:                          # noqa: BLE001 -- cv2 optional
        return a
    med = float(np.median(a))
    if med <= 0.0:
        return a
    dark = (a < DARK_FRAC * med).astype(np.uint8)
    dark = cv2.morphologyEx(dark, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    # Only stripes: a dark component at least STRIPE_ASPECT times longer than
    # it is wide. A dark tile cell, a vignette corner or a round shadow is
    # left alone; fading those made a smooth gradient pass as a grating.
    n, lab, st, _ = cv2.connectedComponentsWithStats(dark, connectivity=8)
    stripes = np.zeros_like(dark)
    for k in range(1, n):
        if st[k, cv2.CC_STAT_AREA] < 0.01 * dark.size:
            continue
        pts = np.column_stack(np.nonzero(lab == k)).astype(np.float32)
        (_, _), (w, h), _ = cv2.minAreaRect(pts)
        if min(w, h) > 0 and max(w, h) / min(w, h) >= STRIPE_ASPECT:
            stripes[lab == k] = 1
    if not stripes.any():
        return a
    dark = cv2.dilate(stripes, np.ones((3, 3), np.uint8)).astype(bool)
    if dark.mean() > 0.5:
        return a                               # mostly dark: no floor to protect
    keep = 1.0 - cv2.GaussianBlur(dark.astype(np.float32), (0, 0), 3.0)
    keep = np.clip(keep, 0.0, 1.0) * (~dark)
    floor = float(np.median(a[~dark]))
    return floor + (a - floor) * keep


def detect(gray: np.ndarray, *, min_strength: float = MIN_PEAK_FRACTION
           ) -> Optional[Grating]:
    """Find the floor's carrier, or return None if there isn't one.

    None is the common and correct answer on an untiled floor. A caller that
    treats None as "no motion" rather than "no instrument" has made the same
    mistake as reading a dead sensor's zeros as a measurement.
    """
    if gray is None or gray.ndim != 2 or min(gray.shape) < 32:
        return None
    a = np.asarray(gray, dtype=np.float32)
    if not np.isfinite(a).all():
        return None
    a = _blank_thick_dark(a)
    a = a - a.mean()
    if a.std() < 1e-6:
        return None                     # a flat wall of one colour
    # A window, or the frame's own edges ring across the whole spectrum and
    # the "dominant period" is the image border.
    win = np.outer(np.hanning(a.shape[0]), np.hanning(a.shape[1]))
    spec = np.fft.fftshift(np.fft.fft2(a * win))
    mag = np.abs(spec)
    cy, cx = np.array(mag.shape) // 2

    # Suppress DC and its immediate neighbourhood: illumination falloff is a
    # very low frequency with enormous amplitude and would win every time.
    lo = max(2, int(min(mag.shape) / _max_period_px(mag.shape)))
    mag[cy - lo:cy + lo + 1, cx - lo:cx + lo + 1] = 0.0
    total = float(mag.sum())
    if total <= 0.0:
        return None

    iy, ix = np.unravel_index(int(np.argmax(mag)), mag.shape)
    iy, ix = _fundamental(mag, iy, ix, cy, cx)
    # Sub-bin, or the height is quantised. An FFT bin is an integer, so the
    # recovered period lands on a discrete ladder: measured against known
    # truth, a 48 px grid read back as 51.2 px -- 6.67 % error, which becomes
    # 6.67 % on every velocity derived from the height. A parabolic fit
    # through the peak and its two neighbours recovers the true frequency
    # between bins. Standard, three lines, and it turns the worst case above
    # into a fraction of a percent.
    fy, fx = _centroid(mag, iy, ix, cy, cx)
    radius = math.hypot(fy, fx)
    if radius <= 0.0:
        return None
    # The TRUE 2-D period: the frequency vector's magnitude in cycles per
    # PIXEL, not per row. The two differ on a non-square image, and the row
    # form silently rescales every height estimate read off this.
    fy_cpp = fy / mag.shape[0]
    fx_cpp = fx / mag.shape[1]
    f_cpp = math.hypot(fy_cpp, fx_cpp)
    if f_cpp <= 0.0:
        return None
    period = 1.0 / f_cpp
    if period < MIN_PERIOD_PX or period > _max_period_px(mag.shape):
        return None

    peak = float(mag[max(0, iy - 2):iy + 3, max(0, ix - 2):ix + 3].sum())
    strength = peak / total
    if strength < min_strength:
        return None

    # Which gate does the work -- established by injection, not by argument.
    #
    # A smooth illumination gradient, which is what an underwater scene with
    # light falloff actually looks like, was accepted at 35 % "strength" --
    # higher than a real tiled floor's 5 %. Three discriminants were written
    # against it and the story that finally held is not the one this file
    # first told:
    #
    #   share of energy      ACCEPTED the ramp (35 % vs a real grid's 5 %)
    #   angular prominence   ACCEPTED it (377 vs a rotated grid's 194) -- a
    #                        ramp IS angularly selective, all its energy lies
    #                        along one axis
    #   harmonic at 2f       ACCEPTED it, backwards: the ramp's smooth spectrum
    #                        makes the local background at 2f tiny, so the
    #                        ratio explodes (2164 vs a real grid's 6)
    #   interior maximum     written, believed, and then DELETED -- removing it
    #                        broke NOT ONE TEST. It was decoration.
    #
    # Two cheap checks carry everything, and each was proven by removing it:
    #
    #   PEAK RADIUS   the dominant frequency must be clear of the lowest bins.
    #                 A ramp and a vignette peak at radius 2 because they ARE
    #                 the lowest frequency; a real grid peaks at radius 8.
    #                 Remove this and both gradient tests fail.
    #   PEAK SHARE    rejects white noise and our own sim floor textures, whose
    #                 energy is spread with no peak worth the name. Remove it
    #                 and the noise and sim-texture tests fail.
    if radius < MIN_PEAK_RADIUS_BINS:
        return None
    angle = math.degrees(math.atan2(fy_cpp, fx_cpp)) % 90.0
    ph = float(np.angle(spec[iy, ix]))
    return Grating(period_px=period, angle_deg=angle, phase=ph,
                   strength=strength)


def _fundamental(mag, iy: int, ix: int, cy: int, cx: int) -> Tuple[int, int]:
    """Step from a harmonic down to the grid's fundamental, if one is there.

    Thin grout is an impulse train, and an impulse train's harmonics are as
    Strong as its fundamental. The tallest bin is then a coin toss between f,
    2f and 3f. Measured on synthetic grids with 3 px grout, 80 px period,
    swept over 36 angles: the period came back at 1/2 or 1/3 in 17 of 36 --
    and height is `f * tile / period`, so height and every flow velocity
    scaled by it were 2x or 3x wrong, with nothing refusing. Real archive
    tiles (`final_run/bin.mkv`) have 2-4 px grout at ~85 px, i.e. this regime.

    A harmonic has energy at `f/k`; a fundamental has only noise there. So walk
    down while the sub-harmonic holds a real share of the peak, and stop at the
    DC guard (MIN_PEAK_RADIUS_BINS), which is where the gradient controls live.
    """
    peak = float(mag[iy, ix])
    while True:
        moved = False
        for k in (3, 2):
            sy = cy + (iy - cy) / k
            sx = cx + (ix - cx) / k
            if math.hypot(sy - cy, sx - cx) < MIN_PEAK_RADIUS_BINS:
                continue
            y0, x0 = int(round(sy)), int(round(sx))
            win = mag[max(0, y0 - 1):y0 + 2, max(0, x0 - 1):x0 + 2]
            if win.size == 0:
                continue
            j = int(np.argmax(win))
            by = max(0, y0 - 1) + j // win.shape[1]
            bx = max(0, x0 - 1) + j % win.shape[1]
            if float(mag[by, bx]) >= SUBHARMONIC_FRACTION * peak:
                iy, ix, moved = by, bx, True
                break
        if not moved:
            return iy, ix


def _centroid(mag, iy: int, ix: int, cy: int, cx: int) -> Tuple[float, float]:
    """Frequency of a peak as the power centroid of its main lobe (+-2 bins).

    The three-point parabola it replaced (one row and one column through the
    peak) turned a 48 px grid read as 51.2 px into a fraction of a percent, and
    assumed one clean lobe. A lane blanked
    out of the frame multiplies the grid by a band, which dents the lobe's
    centre symmetrically; the parabola then fit the dent and the search beside
    it landed on a shoulder, and a tiled floor read 1.21x its period
    (measured-bars section 101). A symmetric dent leaves the centroid where it
    was. Under the Hanning window the main lobe is +-2 bins wide.
    """
    y0, y1 = max(0, iy - 2), min(mag.shape[0], iy + 3)
    x0, x1 = max(0, ix - 2), min(mag.shape[1], ix + 3)
    w = mag[y0:y1, x0:x1].astype(np.float64) ** 2
    s = float(w.sum())
    if s <= 0.0:
        return float(iy - cy), float(ix - cx)
    yy, xx = np.mgrid[y0:y1, x0:x1]
    return float((w * yy).sum() / s - cy), float((w * xx).sum() / s - cx)


def shift_from_phase(before: Grating, after: Grating) -> Optional[float]:
    """Displacement along the carrier, in PIXELS, from the phase difference.

    WRAPPED, and the wrap is the whole caveat. The answer lies in
    `[-period/2, +period/2)`; a real motion larger than half a tile between
    frames comes back as a small motion in the opposite direction. At 30 Hz
    and a 0.15 m tile that bound is generous, but it is a bound, and a caller
    must gate on the frame interval rather than trust this blindly.
    """
    if before is None or after is None:
        return None
    if abs(before.period_px - after.period_px) > 0.25 * before.period_px:
        return None                 # the height changed; not the same carrier
    d = after.phase - before.phase
    d = (d + math.pi) % (2.0 * math.pi) - math.pi
    return float(d / (2.0 * math.pi) * before.period_px)
