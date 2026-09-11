"""Take GEOMETRY from the structure, IDENTITY from the symbol on it.

⛔ THE MISTAKE THIS REPLACES, and it is one line of mission code. "Pass under
the side showing our animal" is usually written as `where('rescue')` -- is the
symbol left or right OF THE FRAME. That is the wrong reference. The frame
centre is where the CAMERA is pointing, not where the gate's divider is, so the
answer flips the moment the vehicle is off-axis: a hull sitting left of the
gate reads a centred symbol as "right" and drives under the wrong side. The
divider is a property of the GATE, so the side must be measured against the
gate's own midline.

The second half is BumblebeeAS's bin insight, which they state in config: every
structure detection is PnP'd, and the symbol topic NEVER is. Bin walls occlude
a symbol from oblique views, so a symbol's box is a poor geometric object and a
fine identity one. Ours is the same shape: `gate` is the structure whose
geometry we trust, `rescue`/`repair` are identity only.

So a symbol contributes exactly two things here: WHICH structure it belongs to
(by overlap, not by proximity) and WHICH SIDE of that structure it sits on.
Never a pose, never a range.

Pure geometry -- no ROS, no cv2. Boxes and optional masks in, an answer with
its support out.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Optional, Sequence

# A symbol overlapping its structure by less than this is not ON it. Set from
# the failure it prevents rather than from a tuning run: a symbol floating in
# open water beside the gate, or one painted on the NEXT prop, would otherwise
# be assigned to the gate and vote on which side to fly through.
MIN_OVERLAP = 0.55

# How far from the midline a symbol must sit before the side is called. Inside
# this band the honest answer is 'centre', because a symbol straddling the
# divider names no side and a mission must not be handed a coin flip as a
# decision. Fraction of the structure's half-width.
SIDE_DEADBAND = 0.10


@dataclass(frozen=True)
class Identity:
    """Which symbol is on the structure, and where on it."""
    label: str = ''
    side: str = 'unknown'       # 'left' | 'right' | 'centre' | 'unknown'
    overlap: float = 0.0        # fraction of the symbol inside the structure
    score: float = 0.0          # the symbol detector's own confidence
    offset: float = 0.0         # -1..+1 across the structure, 0 = its midline
    reason: str = ''


def _xyxy(det) -> tuple:
    return tuple(float(v) for v in det.xyxy)


def _area(box: Sequence[float]) -> float:
    return max(0.0, box[2] - box[0]) * max(0.0, box[3] - box[1])


def overlap_frac(symbol, structure) -> float:
    """Fraction of the SYMBOL that lies inside the structure's box.

    Deliberately not IoU. A gate is large and its placard is small, so IoU
    between them is tiny however perfectly the placard sits on the gate -- an
    IoU gate would reject every correct pairing. What the question actually is:
    how much of the symbol is on the structure.
    """
    s, t = _xyxy(symbol), _xyxy(structure)
    ix1, iy1 = max(s[0], t[0]), max(s[1], t[1])
    ix2, iy2 = min(s[2], t[2]), min(s[3], t[3])
    inter = max(0.0, ix2 - ix1) * max(0.0, iy2 - iy1)
    a = _area(s)
    return inter / a if a > 0.0 else 0.0


def side_of(symbol, structure, *, deadband: float = SIDE_DEADBAND) -> tuple:
    """(side, offset) of the symbol across the STRUCTURE, not the frame.

    `offset` is -1 at the structure's left edge, +1 at its right, 0 at its
    midline -- so it stays meaningful when the vehicle is off-axis, which is
    the whole reason this is not `where()`.
    """
    s, t = _xyxy(symbol), _xyxy(structure)
    mid = 0.5 * (t[0] + t[2])
    half = 0.5 * (t[2] - t[0])
    if half <= 0.0:
        return 'unknown', 0.0
    cx = 0.5 * (s[0] + s[2])
    offset = (cx - mid) / half
    if abs(offset) < deadband:
        return 'centre', offset
    return ('right' if offset > 0.0 else 'left'), offset


def identify(structure, symbols: Iterable, *,
             min_overlap: float = MIN_OVERLAP,
             deadband: float = SIDE_DEADBAND) -> Identity:
    """The best symbol ON this structure, with the side it sits on.

    "Best" is overlap-gated first, then highest detector score -- in that
    order. Score alone would let a confident symbol on the NEXT prop win, and
    overlap alone would let a spurious low-confidence box beat a real one.
    """
    best: Optional[Identity] = None
    for sym in symbols:
        ov = overlap_frac(sym, structure)
        if ov < min_overlap:
            continue
        side, offset = side_of(sym, structure, deadband=deadband)
        cand = Identity(label=str(sym.class_name or ''), side=side,
                        overlap=ov, score=float(sym.score), offset=offset)
        if best is None or cand.score > best.score:
            best = cand
    if best is None:
        return Identity(reason='no symbol overlaps this structure')
    return best


def sides(structure, symbols: Iterable, *,
          min_overlap: float = MIN_OVERLAP,
          deadband: float = SIDE_DEADBAND) -> dict:
    """{label: Identity} -- every symbol on the structure, best per label.

    The gate question is "which side is MY animal on", and both placards are
    usually visible, so the mission wants the map rather than one winner.
    """
    out: dict = {}
    for sym in symbols:
        ov = overlap_frac(sym, structure)
        if ov < min_overlap:
            continue
        label = str(sym.class_name or '')
        side, offset = side_of(sym, structure, deadband=deadband)
        cand = Identity(label=label, side=side, overlap=ov,
                        score=float(sym.score), offset=offset)
        prev = out.get(label)
        if prev is None or cand.score > prev.score:
            out[label] = cand
    return out


def pick_structure(detections: Iterable, structure_class: str):
    """The structure detection to take geometry from: the LARGEST of its class.

    Largest, not most confident. Confidence ranks how sure the detector is that
    something is a gate; area ranks how much of a gate is actually in frame,
    and a pose fitted to a sliver of a prop at the edge is the one that will be
    wrong. Returns None when the structure is not visible -- which is a
    different state from "visible but no symbol on it", and a mission needs to
    tell those apart.
    """
    want = str(structure_class).strip().lower()
    best = None
    best_area = 0.0
    for d in detections:
        if str(d.class_name or '').strip().lower() != want:
            continue
        a = _area(_xyxy(d))
        if a > best_area:
            best, best_area = d, a
    return best
