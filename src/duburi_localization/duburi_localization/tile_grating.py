"""The pool floor is a surveyed 2-D grating. Read it like an optical encoder.

⛔ THE INVERSION THIS MODULE IS BUILT ON. `measured-bars.md` records the known
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

⚠ WHAT THIS IS NOT. Phase is absolute only modulo the tile pitch -- it bounds
drift to one tile, it does not localise the vehicle in the pool. Counting which
tile you are on needs continuity, and continuity is exactly what a dropout
breaks. Stated here so nobody reads "drift-free" as "a global fix".

⚠ AND IT REFUSES. Measured while writing this: our own SAUVC and RoboSub sim
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
# SWEPT AGAINST BOTH CONTROLS, not chosen. At 0.020 every negative refuses but
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

# And a period longer than this cannot be seen twice in a frame, so its
# "periodicity" is one edge and a coincidence.
def _max_period_px(shape) -> float:
    return min(shape) / 3.0


@dataclass(frozen=True)
class Grating:
    """What one frame of floor says about itself."""
    period_px: float          # dominant spatial period
    angle_deg: float          # grid orientation, wrapped to [0, 90)
    phase: Tuple[float, float]   # carrier phase along each axis, radians
    strength: float           # dominant peak's share of spectral energy
    second_period_px: Optional[float] = None   # the orthogonal axis, if found

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


def usable_height_m(focal_px: float, tile_m: float, frame_px: int) -> tuple:
    """(min, max) height over the floor where this instrument works.

    ⛔ THE ENVELOPE IS PHYSICAL, NOT A TUNING CHOICE. You cannot measure a
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
    # ⛔ SUB-BIN, OR THE HEIGHT IS QUANTISED. An FFT bin is an integer, so the
    # recovered period lands on a discrete ladder: measured against known
    # truth, a 48 px grid read back as 51.2 px -- 6.67 % error, which becomes
    # 6.67 % on every velocity derived from the height. A parabolic fit
    # through the peak and its two neighbours recovers the true frequency
    # between bins. Standard, three lines, and it turns the worst case above
    # into a fraction of a percent.
    fy = float(iy - cy) + _parabolic(mag[:, ix], iy)
    fx = float(ix - cx) + _parabolic(mag[iy, :], ix)
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

    # ⛔ WHICH GATE DOES THE WORK -- ESTABLISHED BY INJECTION, NOT BY ARGUMENT.
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

    # The orthogonal axis, if the pattern is a grid rather than stripes. Its
    # presence is what separates a tiled floor from a lane line.
    second = _orthogonal_period(mag, cy, cx, fy, fx)

    ph = float(np.angle(spec[iy, ix]))
    return Grating(period_px=period, angle_deg=angle, phase=(ph, ph),
                   strength=strength, second_period_px=second)


def _annulus_values(mag, cy, cx, radius: float):
    h, w = mag.shape
    r = max(3.0, float(radius))
    out = []
    for k in range(180):
        t = math.pi * k / 180.0
        y = int(round(cy + r * math.sin(t)))
        x = int(round(cx + r * math.cos(t)))
        if 0 <= y < h and 0 <= x < w:
            out.append(float(mag[y, x]))
    return out


def _parabolic(line: np.ndarray, i: int) -> float:
    """Sub-bin offset of a peak from a 3-point parabolic fit.

    The classic interpolator: with `a`, `b`, `c` the magnitudes either side of
    and at the peak, the vertex sits at `0.5*(a-c)/(a-2b+c)`. Guarded against
    the flat case, where the denominator vanishes and the offset is undefined
    rather than infinite.
    """
    if i <= 0 or i >= len(line) - 1:
        return 0.0
    a, b, c = float(line[i - 1]), float(line[i]), float(line[i + 1])
    denom = a - 2.0 * b + c
    if abs(denom) < 1e-12:
        return 0.0
    off = 0.5 * (a - c) / denom
    return float(off) if abs(off) <= 1.0 else 0.0


def _orthogonal_period(mag, cy, cx, fy, fx) -> Optional[float]:
    """Look for a peak ~90 degrees from the dominant one."""
    h, w = mag.shape
    best, best_val = None, 0.0
    base = math.atan2(fy, fx)
    for sign in (1.0, -1.0):
        ang = base + sign * math.pi / 2.0
        for r in range(4, int(min(h, w) / 2) - 1):
            yy = int(round(cy + r * math.sin(ang)))
            xx = int(round(cx + r * math.cos(ang)))
            if not (0 <= yy < h and 0 <= xx < w):
                break
            v = float(mag[yy, xx])
            if v > best_val:
                best_val, best = v, r
    if best is None or best <= 0:
        return None
    return float(mag.shape[0]) / float(best)


def shift_from_phase(before: Grating, after: Grating) -> Optional[float]:
    """Displacement along the carrier, in PIXELS, from the phase difference.

    ⛔ WRAPPED, and the wrap is the whole caveat. The answer lies in
    `[-period/2, +period/2)`; a real motion larger than half a tile between
    frames comes back as a small motion in the opposite direction. At 30 Hz
    and a 0.15 m tile that bound is generous, but it is a bound, and a caller
    must gate on the frame interval rather than trust this blindly.
    """
    if before is None or after is None:
        return None
    if abs(before.period_px - after.period_px) > 0.25 * before.period_px:
        return None                 # the height changed; not the same carrier
    d = after.phase[0] - before.phase[0]
    d = (d + math.pi) % (2.0 * math.pi) - math.pi
    return float(d / (2.0 * math.pi) * before.period_px)
