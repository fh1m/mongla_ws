"""How close should the vehicle get before it trusts what it sees?

⛔ THE OBVIOUS ANSWER IS WRONG, AND WE MEASURED IT. "Closer is better" fails:
§23 measured detector confidence against apparent size on real footage and
found it rises 0.260 -> 0.521 and then FALLS AGAIN. Very close, the prop
overflows the frame, context is lost and the detector does worse than it did
at range.

So the approach setpoint is a BAND -- the apparent size where this class is
seen best -- not a minimum range. A controller told to minimise range drives
through the band and out the far side.

⭐ AND EVIDENCE ACCUMULATES ACROSS VIEWPOINTS, NOT ACROSS FRAMES. Fifty
frames of a stationary vehicle are one observation seen fifty times;
multiplying their likelihoods manufactures certainty out of a single look.
The bank can say whether the viewpoint actually changed, so that is the gate.
"""
from dataclasses import dataclass
import math

# Fraction of frame height the target should occupy. From §23's peak band.
BAND_LO = 0.25
BAND_HI = 0.45
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


def advise(box_h_px: float, frame_h_px: float,
           lo: float = BAND_LO, hi: float = BAND_HI) -> Approach:
    """Close, hold, or back off, from apparent size alone."""
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
