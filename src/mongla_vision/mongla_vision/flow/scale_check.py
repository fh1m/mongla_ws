"""Is the height that scales every flow velocity actually right?

Optical flow gives pixels per second. Metres per second needs a HEIGHT above
the floor, and that height multiplies every velocity the filter receives --
so a 20 % height error is a 20 % velocity error, integrated into position
without limit and with no symptom anywhere.

⭐ WE HAVE TWO INDEPENDENT HEIGHTS AND HAVE NEVER COMPARED THEM.
  * the tile grating measures the floor directly (`floor_height`);
  * `pool_depth_m - |depth|` is arithmetic on the board's barometer.
They share no sensor, so their agreement is evidence and their disagreement is
a fault that is otherwise invisible.

⛔ AND IT REPORTS, IT DOES NOT PICK. A divergence says the two disagree, never
which one is wrong -- and `pool_depth_m` is the one nobody measures. Silently
preferring either is how a stack acquires a number it cannot defend. The same
rule `flow_node` already follows for its optical cross-check.

THE DISCIPLINE comes from the prototype's `MetricCalibrator`: a weighted
median rather than a mean (one bad pair must not move the answer), confidence
from the coefficient of variation, a minimum sample count, and ageing so a
calibration cannot outlive the conditions it was measured in.

⚠ AND THE FRAME CONFUSION THAT PROTOTYPE FLAGGED IN ITSELF: bottom-referenced
ALTITUDE is not surface-referenced DEPTH. They are different quantities with
the same units, and this module never fuses one as the other -- it compares
two estimates of the SAME quantity, altitude.
"""
from dataclasses import dataclass
from typing import Optional
import math
import time

MIN_SAMPLES = 5
MAX_AGE_S = 60.0
# Beyond this the two heights are not describing the same water column.
DISAGREE_FRAC = 0.20


@dataclass(frozen=True)
class ScaleCheck:
    ok: bool
    reason: str
    ratio: float = float('nan')       # grating / barometric
    confidence: float = 0.0
    samples: int = 0
    disagree: bool = False


def _median(xs):
    s = sorted(xs)
    n = len(s)
    return s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])


class ScaleCrossCheck:
    """Accumulates (grating, barometric) altitude pairs and compares them."""

    def __init__(self, min_samples: int = MIN_SAMPLES,
                 max_age_s: float = MAX_AGE_S,
                 disagree_frac: float = DISAGREE_FRAC):
        self.min_samples = int(min_samples)
        self.max_age_s = float(max_age_s)
        self.disagree_frac = float(disagree_frac)
        self._pairs = []              # (t, grating_m, baro_m)

    def add(self, grating_m: float, baro_m: float,
            now: Optional[float] = None) -> None:
        """⛔ A pair with either side absent is NOT stored. Half a comparison
        is not a comparison, and storing it would let the sample count -- the
        thing `min_samples` gates on -- be met by data that cannot answer."""
        if not (grating_m > 0.0 and baro_m > 0.0):
            return
        if not (math.isfinite(grating_m) and math.isfinite(baro_m)):
            return
        self._pairs.append((time.monotonic() if now is None else float(now),
                            float(grating_m), float(baro_m)))

    def _fresh(self, now: Optional[float] = None):
        t = time.monotonic() if now is None else float(now)
        return [p for p in self._pairs if (t - p[0]) <= self.max_age_s]

    def check(self, now: Optional[float] = None) -> ScaleCheck:
        fresh = self._fresh(now)
        if len(fresh) < self.min_samples:
            return ScaleCheck(False, f'{len(fresh)} pairs, need '
                                     f'{self.min_samples}', samples=len(fresh))
        ratios = [g / b for _t, g, b in fresh]
        r = _median(ratios)
        mean = sum(ratios) / len(ratios)
        # Coefficient of variation: spread relative to size, which is the
        # right shape for a RATIO. A standard deviation alone would call a
        # ratio near 2.0 noisy and the same spread near 0.2 tight.
        if mean <= 0:
            return ScaleCheck(False, 'degenerate ratios', samples=len(fresh))
        var = sum((x - mean) ** 2 for x in ratios) / len(ratios)
        cv = math.sqrt(var) / mean
        conf = 1.0 / (1.0 + 10.0 * cv)
        disagree = abs(r - 1.0) > self.disagree_frac
        if disagree:
            return ScaleCheck(
                True,
                f'the two heights DISAGREE by {abs(r - 1.0) * 100:.0f} % '
                f'(grating/barometric = {r:.2f}). This says they differ, NOT '
                f'which is right -- pool_depth_m is the one nobody measures.',
                ratio=r, confidence=conf, samples=len(fresh), disagree=True)
        return ScaleCheck(True, f'agree within {abs(r - 1.0) * 100:.0f} %',
                          ratio=r, confidence=conf, samples=len(fresh))
