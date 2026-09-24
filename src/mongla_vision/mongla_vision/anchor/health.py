"""A fouled lens and a changed world look identical, and need opposite answers.

When the anchor stops matching, there are two causes and the vehicle should do
different things about them:

  * the CAMERA is degraded -- silt on the port, a bubble, condensation, the
    lights off. Nothing the vehicle sees can be trusted, so the answer is to
    stop trusting vision, not to hunt for a better view.
  * the WORLD moved -- the target left, the vehicle turned, the current pushed
    it off. Vision is fine, and the answer is to search.

⛔ Nothing in this stack can currently tell them apart, so both arrive as "the
anchor is not matching" and get the same response. The signal that separates
them is already computed on every bank lookup and thrown away:

    a fouled lens collapses the yield against EVERY reference at once.
    a changed world collapses it against the one that used to win.

That is a statement about the WHOLE shortlist, not about the best match, which
is why the best-of-bank answer alone cannot carry it. It needs no new
measurement, no new sensor and no new model -- only that the per-reference
yields are looked at instead of maxed over.

⚠ THIS IS A HYPOTHESIS WITH A MECHANISM, NOT A MEASURED RESULT. It has never
been run against a fouled port, because we have never fouled one on purpose.
The thresholds below are therefore DECLARED, not measured, and
`measured-bars.md` records them as such. What is defensible today is the
structure -- that the two causes are separable in principle from data already
in hand -- and the refusal to report a verdict from too little evidence.
"""
from dataclasses import dataclass
from typing import Optional, Sequence

# Fewer references than this and "all of them" means nothing: one reference
# cannot distinguish "every view failed" from "the only view failed".
MIN_REFERENCES = 3

# A yield this far below a reference's own best is a collapse rather than a
# normal viewpoint change. Declared, not measured.
COLLAPSE_FRACTION = 0.25

# If this fraction of the shortlist collapsed together, the common element is
# the camera. Below it, the ones that held up say the camera is fine.
FOULED_FRACTION = 0.8

UNKNOWN = 'unknown'
CAMERA = 'camera'
WORLD = 'world'
HEALTHY = 'healthy'


@dataclass(frozen=True)
class Verdict:
    """What the bank's per-reference yields imply about the cause."""
    state: str
    reason: str
    collapsed: int = 0
    considered: int = 0

    @property
    def trust_vision(self) -> bool:
        """⛔ UNKNOWN MUST NOT DISARM VISION. A verdict of 'I cannot tell' is
        not evidence of a fouled lens, and treating it as one would make every
        quiet moment look like a failure."""
        return self.state != CAMERA


def assess(current: Sequence[int], best: Sequence[int],
           *, min_refs: int = MIN_REFERENCES,
           collapse_fraction: float = COLLAPSE_FRACTION,
           fouled_fraction: float = FOULED_FRACTION) -> Verdict:
    """Compare THIS lookup's per-reference yields against each one's own best.

    `current[i]` is what reference i scored on this frame; `best[i]` is the
    most it has ever scored. Each reference is judged against ITSELF, because
    a whole-frame floor reference and a cropped prop reference are not
    comparable to each other -- comparing them would report the bank's
    composition rather than the camera's state.
    """
    n = min(len(current), len(best))
    if n < min_refs:
        return Verdict(UNKNOWN, f'only {n} reference(s); need {min_refs} '
                                f'before "all of them" means anything',
                       considered=n)

    usable = [(int(current[i]), int(best[i])) for i in range(n)
              if int(best[i]) > 0]
    if len(usable) < min_refs:
        return Verdict(UNKNOWN, 'not enough references have ever scored',
                       considered=len(usable))

    collapsed = sum(1 for c, b in usable if c < collapse_fraction * b)
    frac = collapsed / len(usable)

    if collapsed == 0:
        return Verdict(HEALTHY, 'every reference is still scoring',
                       collapsed=0, considered=len(usable))
    if frac >= fouled_fraction:
        return Verdict(CAMERA,
                       f'{collapsed}/{len(usable)} references collapsed '
                       f'together -- the common element is the camera',
                       collapsed=collapsed, considered=len(usable))
    return Verdict(WORLD,
                   f'{collapsed}/{len(usable)} references collapsed, the rest '
                   f'still score -- the camera is fine and the view changed',
                   collapsed=collapsed, considered=len(usable))
