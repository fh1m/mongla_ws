"""Detection confidence as the vehicle's measure of how much to trust a box.

WHY THIS MODULE EXISTS -- the measurement, not the intuition.

Round 35 set out to measure `vision.range_gain_floor`, on the physics argument
in its own comment: a pixel-normalised P-loop's gain rises ~1/range, so a kp
stable far-field over-drives close-in. The data said the knob is aimed at the
wrong variable. Box jitter tracks DETECTION CONFIDENCE, not bbox fill:

    clip           conf p50   close-in jitter growth
    octagon_1        0.932            0.6x
    bin.mkv          0.906            2.1x
    bin_front_3      0.211            2.9x

A marginal detection wanders at any range, and a confident one is steady even
when it fills the frame. Confirmed by a second, independent probe: across seven
clips, five show confidence RISING close-in with 0-5.5 % missed frames, and the
one clip whose confidence collapses (`bin`, 0.891 -> 0.357) is also the one with
the worst jitter (4.1x).

So confidence is the quality signal the stack already computes on every frame
and then throws away everywhere except a hard threshold.

WHAT THE FIELD DOES: the NSA Kalman filter (GIAOTracker, adopted by StrongSORT)
scales the measurement noise covariance by the detection score,

    R_tilde = (1 - c) * R

so a confident detection is trusted and a doubtful one lets the filter coast on
its own prediction instead of being yanked by a bad box.

WHY WE CANNOT USE THAT FORMULA AS PUBLISHED. It assumes a detector whose scores
live high. Measured against our own distributions:

    MOT benchmark   conf p10 0.55  -> p90 0.95   R factor 0.450 -> 0.050  9.00x
    OUR underwater  conf p10 0.167 -> p90 0.439  R factor 0.833 -> 0.561  1.48x

Raw NSA gives a benchmark a NINE-fold swing in trust and gives us **1.48x** --
it is nearly a no-op in exactly the regime it was added to fix. Underwater
scores are low because the water is hard, not because every detection is bad;
`c` is only meaningful RELATIVE TO WHAT THIS DETECTOR PRODUCES.

Hence `normalise()`: map the detector's own p10..p90 onto [0,1] first, then
apply NSA to the normalised value. A detection at this detector's 90th
percentile is trusted like a 0.95 elsewhere, because for this detector it is
the same statement about quality.

The percentiles are OBSERVED AT RUNTIME, not baked. A number fitted to one
water was the exact mistake `underwater.recommend()` made -- thresholds from
two clips that called a third venue wrong. Here the distribution is estimated
from the stream itself and adapts to whatever pool the vehicle is in.
"""
from __future__ import annotations

from collections import deque
from typing import Deque, Optional, Tuple

# NSA's floor. R is never scaled to zero: a confidence of 1.0 does not mean the
# box is exact, and a zero-noise measurement makes the filter discard its own
# prediction entirely -- one perfect-looking frame would then teleport a track.
NSA_MIN_FACTOR = 0.10
NSA_MAX_FACTOR = 1.60   # a doubtful box may be trusted LESS than the static R

# Enough samples that p10/p90 mean something, short enough to follow the water
# changing during a run.
WINDOW = 240
MIN_SAMPLES = 40

# Fallbacks until the window fills -- our measured underwater distribution.
# Deliberately OUR numbers and not a benchmark's, for the reason above.
FALLBACK_LO = 0.167
FALLBACK_HI = 0.439


def _pct(sorted_vals, q: float) -> float:
    if not sorted_vals:
        return float('nan')
    k = (len(sorted_vals) - 1) * q / 100.0
    f = int(k)
    if f + 1 >= len(sorted_vals):
        return sorted_vals[f]
    return sorted_vals[f] + (sorted_vals[f + 1] - sorted_vals[f]) * (k - f)


class ConfidenceModel:
    """Tracks what this detector's scores look like, and scores a detection.

    One instance per camera. Feed it every accepted detection's score; ask it
    for a trust factor or a trend.
    """

    def __init__(self, window: int = WINDOW,
                 lo_pct: float = 10.0, hi_pct: float = 90.0):
        self._scores: Deque[float] = deque(maxlen=window)
        self._lo_pct = lo_pct
        self._hi_pct = hi_pct

    def observe(self, conf: float) -> None:
        if conf == conf and conf > 0.0:      # NaN-safe
            self._scores.append(float(conf))

    def bounds(self) -> Tuple[float, float]:
        """(lo, hi) percentile bounds of the observed distribution."""
        if len(self._scores) < MIN_SAMPLES:
            return FALLBACK_LO, FALLBACK_HI
        s = sorted(self._scores)
        lo, hi = _pct(s, self._lo_pct), _pct(s, self._hi_pct)
        if hi - lo < 1e-3:                   # a detector pinned at one score
            return FALLBACK_LO, FALLBACK_HI
        return lo, hi

    def normalise(self, conf: float) -> float:
        """Confidence -> [0, 1] against this detector's OWN distribution.

        0 = as bad as this detector's 10th percentile, 1 = as good as its 90th.
        """
        if conf != conf:
            return 0.0
        lo, hi = self.bounds()
        return max(0.0, min(1.0, (conf - lo) / (hi - lo)))

    def nsa_factor(self, conf: float) -> float:
        """R multiplier for the Kalman update -- NSA on the NORMALISED score.

        Returns a value in [NSA_MIN_FACTOR, NSA_MAX_FACTOR]: small means "trust
        this box", large means "trust the filter's prediction instead".
        """
        n = self.normalise(conf)
        f = NSA_MAX_FACTOR - n * (NSA_MAX_FACTOR - NSA_MIN_FACTOR)
        return max(NSA_MIN_FACTOR, min(NSA_MAX_FACTOR, f))

    def authority(self, conf: float, floor: float = 0.25) -> float:
        """Control-authority multiplier in [floor, 1.0] for this detection.

        The control-side twin of `nsa_factor`, and the knob the round's
        measurement actually supports -- the loop should ease off a box it does
        not trust, at ANY range, rather than easing off every close box the way
        a range ramp would.

        Floored rather than allowed to reach zero: a weak detection is still
        the only information available, and a vehicle that stops moving because
        the water got murky has failed the mission just as surely as one that
        drives off target.
        """
        n = self.normalise(conf)
        return floor + n * (1.0 - floor)


class ConfidenceTrend:
    """Falling confidence as an EARLY WARNING that a lock is degrading.

    Measured, and currently unused by anything: on the `bin` clip confidence
    fell 0.891 -> 0.357 BEFORE the box began to wander. The control loop reads
    only an absolute floor, so it learns about the degradation from the error
    signal -- i.e. after the box has already moved.

    Compares a short recent window against a longer baseline. A sustained drop
    is reported; a single bad frame is not, because one frame is what the
    freshness and coast machinery already handles.
    """

    def __init__(self, fast: int = 12, slow: int = 60, drop: float = 0.35):
        self._fast: Deque[float] = deque(maxlen=fast)
        self._slow: Deque[float] = deque(maxlen=slow)
        self._drop = drop

    def observe(self, conf: float) -> None:
        if conf == conf and conf > 0.0:
            self._fast.append(float(conf))
            self._slow.append(float(conf))

    def ratio(self) -> float:
        """recent mean / baseline mean. <1 means confidence is falling."""
        if len(self._fast) < self._fast.maxlen or len(self._slow) < self._slow.maxlen:
            return float('nan')
        f = sum(self._fast) / len(self._fast)
        s = sum(self._slow) / len(self._slow)
        return f / s if s > 1e-6 else float('nan')

    def degrading(self) -> bool:
        """True when recent confidence has fallen materially below baseline."""
        r = self.ratio()
        return r == r and r < (1.0 - self._drop)
