"""The gate side must be measured against the GATE, not the frame.

The decisive case: a hull sitting off to one side of the gate. Every symbol
then shifts the same way in the image, so a frame-relative answer flips and the
vehicle drives under the wrong side -- with the detector, the pose and the
control loop all working perfectly.
"""
import pytest

from duburi_vision.detection.detector import Detection
from duburi_vision.identity import (
    Identity, identify, overlap_frac, pick_structure, side_of, sides,
)


def _d(name, x1, y1, x2, y2, score=0.9):
    return Detection(class_id=0, class_name=name, score=score,
                     xyxy=(float(x1), float(y1), float(x2), float(y2)))


# A gate spanning x 200..600 in a 640-wide frame: midline at 400.
GATE = _d('gate', 200, 100, 600, 400)


def test_the_side_is_relative_to_the_structure_not_the_frame():
    # Symbol at x~250: LEFT of the gate's midline (400) and also left of the
    # frame centre (320). Both references agree here.
    sym = _d('rescue', 230, 150, 270, 190)
    assert side_of(sym, GATE)[0] == 'left'


def test_an_off_axis_hull_does_not_flip_the_answer():
    # The vehicle has drifted right, so the whole gate sits LEFT in the image:
    # x 0..400, midline 200. The placard at x~300 is still on the gate's RIGHT
    # half -- but it is left of the frame centre (320), which is what a
    # frame-relative `where()` would report.
    gate_left = _d('gate', 0, 100, 400, 400)
    sym = _d('repair', 280, 150, 320, 190)
    side, offset = side_of(sym, gate_left)
    assert side == 'right', 'the divider belongs to the gate, not the camera'
    assert offset > 0


def test_a_symbol_on_the_divider_names_no_side():
    sym = _d('rescue', 390, 150, 410, 190)     # centred on the midline
    assert side_of(sym, GATE)[0] == 'centre'


def test_offset_is_normalised_across_the_structure():
    # At the gate's right edge the offset is +1, whatever the gate's width.
    sym = _d('repair', 590, 150, 610, 190)
    _, offset = side_of(sym, GATE)
    assert 0.9 < offset <= 1.1


def test_a_zero_width_structure_is_unknown_not_a_crash():
    assert side_of(_d('rescue', 10, 10, 20, 20), _d('gate', 5, 5, 5, 40))[0] == 'unknown'


# --- belonging: overlap, not proximity --------------------------------------


def test_overlap_is_the_fraction_of_the_SYMBOL_inside_the_structure():
    # Half the symbol's width is inside the gate.
    sym = _d('rescue', 180, 150, 220, 190)     # gate starts at 200
    assert abs(overlap_frac(sym, GATE) - 0.5) < 1e-6


def test_iou_would_have_rejected_every_correct_pairing():
    # A placard is tiny against a gate. IoU here is ~0.003 -- a threshold on it
    # rejects a perfectly placed symbol, which is why overlap is the measure.
    sym = _d('rescue', 230, 150, 270, 190)
    assert overlap_frac(sym, GATE) == pytest.approx(1.0)


def test_a_symbol_on_the_next_prop_is_not_assigned_to_this_one():
    far = _d('repair', 700, 150, 740, 190)
    assert identify(GATE, [far]).label == ''
    assert 'no symbol' in identify(GATE, [far]).reason


def test_overlap_gates_before_score():
    # A CONFIDENT symbol off the structure must lose to a less confident one on
    # it -- score alone would hand the gate the wrong animal.
    on  = _d('rescue', 230, 150, 270, 190, score=0.40)
    off = _d('repair', 700, 150, 740, 190, score=0.99)
    assert identify(GATE, [off, on]).label == 'rescue'


def test_the_better_detection_of_a_label_wins():
    weak   = _d('rescue', 230, 150, 270, 190, score=0.40)
    strong = _d('rescue', 232, 152, 272, 192, score=0.95)
    got = sides(GATE, [weak, strong])
    assert got['rescue'].score == pytest.approx(0.95)


def test_sides_reports_both_placards():
    got = sides(GATE, [_d('rescue', 230, 150, 270, 190),
                       _d('repair', 530, 150, 570, 190)])
    assert got['rescue'].side == 'left'
    assert got['repair'].side == 'right'


# --- which structure supplies the geometry ----------------------------------


def test_the_largest_structure_supplies_the_geometry_not_the_most_confident():
    # A sliver of a gate at the frame edge can be the most CONFIDENT detection
    # and is the worst thing to fit a pose to.
    sliver = _d('gate', 610, 100, 640, 400, score=0.99)
    full   = _d('gate', 200, 100, 600, 400, score=0.55)
    assert pick_structure([sliver, full], 'gate') is full


def test_no_structure_is_distinct_from_no_symbol():
    # "gate not visible" and "gate visible, no placard on it" are different
    # states and a mission branches differently on them.
    assert pick_structure([_d('rescue', 1, 1, 2, 2)], 'gate') is None
    assert identify(GATE, []).reason != ''
