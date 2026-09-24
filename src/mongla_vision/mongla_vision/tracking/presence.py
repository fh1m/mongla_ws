"""The rung BELOW detection: accumulate evidence the detector refused to call.

WHERE IT SITS. Every rung we had begins with a detection and describes how to
survive losing it:

    PRESENCE   <-- THIS. sub-threshold evidence, no detection required
    DETECTION  the model clears `conf` and names a class
    FOLLOW     LK carries the last box                    frame-to-frame
    ANCHOR     XFeat re-finds the scene                   frame-to-reference
    LOST       authority decays to zero

Nothing started below DETECTION, so "in sight but dim" and "gone" were the
same state to us. This rung separates them.

⭐ THE IDEA IS OLD AND IT IS NOT OURS. Track-before-detect comes out of radar
and infrared small-target work: when one frame cannot clear a threshold,
POSTPONE THE THRESHOLD and accumulate across frames until the accumulation
clears one. Published results track at SNR 1 dB, far below single-frame
detectability. We apply it to the boxes our own detector already emits and
then throws away -- it publishes down to 0.202 while the acting bar is 0.45.

MEASURED ON OUR OWN FOOTAGE, not borrowed from the papers
(`tools/weak_box_persistence.py`, 10 425 frames, bag_person_inout_20260924):

    regime               pairs   linked   time-free control   ratio
    in-view dropout         64    0.938               0.090   10.42x
    exit/entry (gone)       25    1.000               0.440    2.27x

During a dropout with the target still in view, 93.8 % of consecutive frames
carry a weak box of the right class within 40 px of the previous one. The
control -- boxes paired across DIFFERENT gaps, so time is removed rather than
merely stirred -- links at 9.0 %. Stable across a bar sweep that grows the
sample to 200 pairs (8.4x to 13.7x).

⛔ THE TRAP THIS MODULE EXISTS TO AVOID. Look at the exit/entry row: it links
at 1.000 as well. Consecutive linking ALONE does not separate a dim target
from a departed one, because weak boxes pile up against the edge the subject
left through and chain beautifully. Only the control column separates them,
and a control cannot be computed on a vehicle. A rung built on run length
alone would report a confident presence for a target that is already gone --
precisely the failure mode CLAUDE.md section 8.6 names as the one that ends
competition runs.

⭐ WHAT SEPARATES THEM AT RUNTIME IS WHERE THE LAST REAL DETECTION SAT. An
in-view dropout is bracketed by detections in the frame interior; a departure
is bracketed by one in the margin band. That is `visibility.assess`, already
measured and already shipped. So this rung REFUSES to accumulate when the last
detection was leaving, and the refusal is the feature rather than a gap in it.

⛔⛔ READ THIS BEFORE WIRING ANY OF IT. The numbers above are a PERSON IN AIR.
On real underwater footage the premise does not hold: the band below the bar is
not faint signal but confident open-water false positives. `gate_rescue_repair`
claims a gate on **90.8 % of gate-free frames** at the shipped 0.15, drawing
half-frame boxes over empty turquoise at 0.44-0.56, and the separation between
gate-present and gate-absent footage is **+0.0 points at both 0.15 and 0.30**
(B-59). Accumulating that band would manufacture a confident track out of
hallucination -- the exact failure this module was written to prevent, arriving
through the front door.

The constants here inherit the same problem. `MIN_RUN = 3` is justified by the
person bag's 0.090 control giving 0.8 % false chains; the one water control
measured **0.360**, which needs MIN_RUN 6 to stay under 1 % -- longer than the
median gap, so the rung would rarely fire even if it were safe. Precedent for
holding: `continuity.py`, whose person-in-air constants were also not shipped.

So this module is DEFERRED and stays there until B-59 is fixed, the bar is
re-derived WITH negative clips (`tools/negative_clip_check.py`), and these
constants are re-swept on water. It may turn out never to be wired, and that is
an acceptable outcome for it.

WHAT IT NEVER CLAIMS. Not a detection, not an identity, not a box to act on.
It answers one question -- "is there still something of that class where the
target was?" -- with a bearing and a capped confidence the control path weighs
exactly as it weighs a follow. Anything more would be the plausible number
standing in for an absent measurement.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

from .visibility import MARGIN_ENTER, assess

# Pixels a dim target may move between frames. Swept 20-90 px on the recorded
# bag; separation between the in-view and departed regimes peaks here at
# 4.59x (10.42x against 2.27x). Tighter raises both ratios but starts dropping
# real links (0.891 at 20 px); looser washes the regimes together (1.18x at
# 90 px, which is no separation at all).
GATE_PX = 40.0

# Consecutive linked frames before presence is declared. The measured
# time-free link rate is 0.090, so a chain of N arises from unstructured
# boxes with probability 0.09^(N-1): 0.8 % at 3, 0.07 % at 4. Three is the
# first value that makes a false chain rare while still firing inside our
# median gap, which is 3-4 frames.
MIN_RUN = 3

# Below this the detector's output is not evidence, it is the noise floor of
# the NMS stage. Measured: this bag publishes 0.202 at its lowest.
SCORE_FLOOR = 0.20

# A presence older than this is not describing the target any more. The
# measured gap p99 is 2.418 s; this sits below it deliberately, because the
# point is to bridge ordinary dropouts and not to outlive them.
MAX_AGE_S = 1.5

# Ceiling on what this rung may contribute. Evidence the model REFUSED to call
# must never outweigh evidence it called, whatever the chain length.
MAX_CONFIDENCE = 0.45


@dataclass(frozen=True)
class WeakBox:
    """One detection that did not clear the acting bar."""
    score: float
    x: float
    y: float
    class_id: str


@dataclass(frozen=True)
class Presence:
    """What the rung is willing to say. `ok` false is a refusal, and a refusal
    is a real answer -- the control path must be able to tell "nothing there"
    from "not looking"."""
    ok: bool
    px: Tuple[float, float] = (float('nan'), float('nan'))
    run: int = 0
    score: float = 0.0
    reason: str = ''

    @property
    def confidence(self) -> float:
        if not self.ok:
            return 0.0
        depth = min(1.0, (self.run - MIN_RUN + 1) / 4.0)
        return float(min(MAX_CONFIDENCE, 0.15 + 0.30 * depth))


@dataclass
class _Chain:
    x: float
    y: float
    score: float
    run: int
    t: float


class PresenceAccumulator:
    """One target's sub-threshold evidence. No ROS, so the bench and the node
    exercise the same object."""

    def __init__(self, *, gate_px: float = GATE_PX, min_run: int = MIN_RUN,
                 score_floor: float = SCORE_FLOOR,
                 max_age_s: float = MAX_AGE_S):
        self._gate = float(gate_px)
        self._min_run = int(min_run)
        self._floor = float(score_floor)
        self._max_age = float(max_age_s)
        self._chain: Optional[_Chain] = None
        self._armed = False
        self._class = ''
        self._last_det: Optional[Tuple[float, float]] = None

    @property
    def armed(self) -> bool:
        """True when the last detection left the target in the frame interior,
        which is the only regime the measurement supports."""
        return self._armed

    def note_detection(self, x: float, y: float, class_id: str,
                       w: float, h: float, fx: float, fy: float) -> None:
        """Every accepted detection resets the rung and decides whether it may
        run at all.

        ⛔ The margin test is a safety interlock, not a tuning knob: a target
        last seen leaving the frame must not be accumulated, because that is
        the regime where the measurement says weak boxes chain just as well
        while meaning nothing.
        """
        self._chain = None
        self._class = str(class_id)
        self._last_det = (float(x), float(y))
        v = assess(x, y, w, h, fx, fy)
        self._armed = bool(v.h >= MARGIN_ENTER)

    def drop(self) -> None:
        self._chain = None
        self._armed = False
        self._last_det = None

    def observe(self, boxes: Sequence[WeakBox], t: float) -> Presence:
        """Advance one frame on the detections that did NOT clear the bar."""
        if not self._armed:
            return Presence(False,
                            reason='not armed: last detection was leaving frame')
        if self._chain is not None and (t - self._chain.t) > self._max_age:
            self._chain = None
            return Presence(False, reason='stale')

        anchor = ((self._chain.x, self._chain.y) if self._chain is not None
                  else self._last_det)
        if anchor is None:
            return Presence(False, reason='no anchor')

        cand: List[WeakBox] = [
            b for b in boxes
            if b.class_id == self._class and b.score >= self._floor
            and math.hypot(b.x - anchor[0], b.y - anchor[1]) <= self._gate]
        if not cand:
            self._chain = None
            return Presence(False, reason='no linked box')

        # ⛔ STRONGEST, NOT NEAREST. Picking the box closest to our own
        # prediction lets the chain chase itself and manufacture a run out of
        # noise -- the accumulator would be measuring its own extrapolation.
        # The strongest candidate is chosen without reference to where we
        # expect the target, so a long run is earned. This is the same
        # discipline the offline tool used, so the two agree by construction.
        b = max(cand, key=lambda w: w.score)
        run = 1 if self._chain is None else self._chain.run + 1
        self._chain = _Chain(b.x, b.y, b.score, run, t)
        if run < self._min_run:
            return Presence(False, run=run, reason='building')
        return Presence(True, px=(b.x, b.y), run=run, score=b.score,
                        reason='presence')
