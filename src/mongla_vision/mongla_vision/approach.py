"""How close should the vehicle get before it trusts what it sees?

⛔ THE OBVIOUS ANSWER IS WRONG, AND WE MEASURED IT. "Closer is better" fails:
section 23 measured detector confidence against apparent size on real footage
and found it rises and then FALLS AGAIN on several classes. Very close, the
prop overflows the frame, context is lost and the detector does worse than it
did at range. So the approach setpoint is a BAND, not a minimum range: a
controller told to minimise range drives through the band and out the far side.

⛔⛔ AND THE BAND THIS MODULE SHIPPED WAS AN INVENTION WITH A CITATION ATTACHED.
It read `BAND_LO = 0.25, BAND_HI = 0.45` and cited section 23 -- but section 23
measured against sqrt(box area) IN PIXELS, reported by quartile, and this
module compares against `box_height / frame_height`. Those are different
quantities and nothing recorded a conversion between them. Re-measured in the
controller's own unit (`tools/approach_band.py`, 4 archive clips, 13 668
detections through the shipped `gate_rescue_repair` graph, conf >= 0.05):

    class        peak bin (box_h / frame_h)   mean conf at peak   n
    0  gate            0.45 - 0.60                  0.263        7588
    1  rescue          0.10 - 0.20                  0.278        1729
    2  repair          0.80 - 1.20  (monotonic)     0.526        4351

**The shipped band matched none of them.** For `repair` it was catastrophic in
the exact way this module exists to prevent: confidence rises all the way to
the frame edge, and a 0.25-0.45 band would have commanded BACK OFF from the
best view the detector ever gets.

⭐ SO THE BAND IS PER CLASS, AND A CLASS WITH NO MEASUREMENT GETS NO ADVICE.
`advise()` needs the class name. Given one that is not in the table it returns
HOLD and says why -- never a default band, because a default band is exactly
what was wrong here. Re-measure with `tools/approach_band.py` when the model
changes; class ids belong to one graph and do not transfer.

⭐ AND EVIDENCE ACCUMULATES ACROSS VIEWPOINTS, NOT ACROSS FRAMES. Fifty frames
of a stationary vehicle are one observation seen fifty times; multiplying their
likelihoods manufactures certainty out of a single look. The bank can say
whether the viewpoint actually changed, so that is the gate.
"""
from dataclasses import dataclass
import math

# Fraction of frame HEIGHT the target should occupy, per class, from
# `tools/approach_band.py` on the shipped gate_rescue_repair graph. These are
# the measured peak bins -- not a band anyone chose the look of.
#
# ⚠ `repair` is MONOTONIC to the frame edge over the range the archive covers,
# so its upper bound is 1.20 rather than a measured fall. That is "we never saw
# it get worse", not "it never does", and a controller should read the upper
# bound as absent rather than as a peak.
BANDS = {
    'gate':   (0.45, 0.60),
    'rescue': (0.10, 0.20),
    'repair': (0.80, 1.20),
}

# ⛔ NO DEFAULT BAND. The previous global (0.25, 0.45) matched none of the three
# measured classes and would have commanded BACK OFF from `repair`'s best view.
# An unmeasured class gets HOLD and a reason.
# Viewpoint change, in bank inliers, below which two looks are ONE look.
SAME_VIEW_INLIERS = 250

HOLD = 'hold'
CLOSE = 'close'
BACK_OFF = 'back_off'


@dataclass(frozen=True)
class Approach:
    action: str
    reason: str
    size: float = float('nan')


def advise(box_h_px: float, frame_h_px: float, class_name: str = '',
           bands: dict = None) -> Approach:
    """Close, hold, or back off, from apparent size alone -- for a class whose
    band has been MEASURED. Anything else holds and says so."""
    table = BANDS if bands is None else bands
    band = table.get(str(class_name).strip().lower())
    if band is None:
        return Approach(HOLD,
                        f'no measured band for {class_name!r} -- holding. '
                        f'Run tools/approach_band.py; a default band is what '
                        f'was wrong before. Measured: {sorted(table)}')
    lo, hi = band
    if not (frame_h_px > 0) or box_h_px is None or box_h_px <= 0:
        return Approach(HOLD, 'no target size -- holding rather than guessing')
    s = float(box_h_px) / float(frame_h_px)
    if s < lo:
        return Approach(CLOSE, f'{s:.2f} of frame, below the {lo:.2f} band', s)
    if s > hi:
        # ⭐ THE COUNTER-INTUITIVE HALF. Backing off IMPROVES detection here,
        # and a controller that only ever closes cannot express it.
        return Approach(BACK_OFF,
                        f'{s:.2f} of frame, past the {hi:.2f} band -- '
                        f'confidence FALLS beyond it', s)
    return Approach(HOLD, f'{s:.2f} of frame, inside the band', s)


class Evidence:
    """Log-odds accumulated across DECORRELATED views only."""

    def __init__(self, same_view_inliers: int = SAME_VIEW_INLIERS):
        self.same_view_inliers = int(same_view_inliers)
        self.log_odds = 0.0
        self.views = 0
        self.skipped = 0

    def observe(self, conf: float, bank_inliers) -> bool:
        """Fold one detection in. Returns whether it counted.

        ⛔ A look that matches the previous reference too well is the SAME
        look. Counting it is how a vehicle holding station talks itself into
        certainty about one observation.
        """
        if bank_inliers is not None and math.isfinite(float(bank_inliers)) \
                and float(bank_inliers) >= self.same_view_inliers:
            self.skipped += 1
            return False
        c = min(max(float(conf), 1e-4), 1.0 - 1e-4)
        self.log_odds += math.log(c / (1.0 - c))
        self.views += 1
        return True

    @property
    def probability(self) -> float:
        return 1.0 / (1.0 + math.exp(-self.log_odds))
