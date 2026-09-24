"""A confidence of 0.5 does not mean the same thing in every pool.

Our own detector's recall swings **29.2 / 72.7 / 68.3 %** across three venues.
A threshold chosen where recall is 72.7 % is a different operating point where
it is 29.2 %, so a single shipped number cannot be right in both -- and the
stack has no way to notice which water it is in.

⭐ XFEAT ALREADY MEASURES THE WATER, for free. Descriptor match quality on a
reference the vehicle is already tracking falls as clarity falls: it is a
turbidity proxy computed on every anchor evaluation and currently discarded.

WHAT THIS DOES. Maps a raw detector confidence to a CALIBRATED one, given the
current match quality, so a downstream threshold can stay fixed while the
meaning of the input moves. In clear water it is close to the identity; in
murk it raises the score, because in murk a detection that fires at all is
rarer and therefore stronger evidence.

⛔ WHAT IT MUST NOT DO, AND THIS IS THE WHOLE RISK. A calibration that can
invent confidence would manufacture detections in exactly the conditions where
the stack is least able to check them. So the adjustment is BOUNDED, it never
promotes a detection the detector did not make, and with no turbidity estimate
it returns the input UNCHANGED rather than guessing.

⚠ THE GAIN IS DECLARED, NOT MEASURED. Relating match quality to the recall
curve needs a labelled per-venue set we do not yet have. What is measured is
the recall spread that motivates it. Until that set exists this ships OFF, and
`measured-bars.md` records it as a declared constant.
"""
from dataclasses import dataclass
import math

# Inliers at or above this is clear water: no adjustment.
CLEAR_INLIERS = 200.0
# At or below this the water is as bad as our worst measured venue.
MURKY_INLIERS = 30.0
# ⛔ THE CAP. The most a confidence may be raised, ever. Chosen well below the
# gap between a weak detection and a strong one so calibration can never turn
# a non-detection into a detection.
MAX_GAIN = 0.15


@dataclass(frozen=True)
class Calibrated:
    value: float
    turbidity: float          # 0 clear .. 1 murky
    gain: float
    reason: str


def turbidity(inliers: float,
              clear: float = CLEAR_INLIERS,
              murky: float = MURKY_INLIERS) -> float:
    """0 in clear water, 1 in the worst water we have measured. NaN if unknown."""
    if inliers is None or not math.isfinite(float(inliers)):
        return float('nan')
    x = float(inliers)
    if x >= clear:
        return 0.0
    if x <= murky:
        return 1.0
    return (clear - x) / (clear - murky)


def calibrate(conf: float, inliers: float, *, enabled: bool = False,
              max_gain: float = MAX_GAIN) -> Calibrated:
    """Adjust a detector confidence for the water it was measured in.

    Off by default. A calibration that changes what the vehicle acts on is a
    control-path change, and this one has a declared constant at its centre.
    """
    c = float(conf)
    t = turbidity(inliers)
    if not enabled:
        return Calibrated(c, t, 0.0, 'calibration disabled')
    if not math.isfinite(t):
        # ⛔ No estimate is not "clear water". Guessing here would apply the
        # largest correction exactly when nothing is known.
        return Calibrated(c, t, 0.0, 'no turbidity estimate -- unchanged')
    if c <= 0.0:
        # Never promote something the detector did not detect.
        return Calibrated(c, t, 0.0, 'no detection to calibrate')
    gain = max_gain * t * (1.0 - c)      # least effect on already-sure scores
    out = min(1.0, c + gain)
    return Calibrated(out, t, out - c,
                      f'turbidity {t:.2f}, +{out - c:.3f}')
