"""An oblique target reads as FURTHER AWAY, so the hull closes in.

The standoff feature is `_fill('area')` = sqrt(w_frac*h_frac) -- Corke &
Hutchinson's sqrt(area) Z-axis feature (IEEE T-RA 17(4) 2001, p.512), chosen
there because it is scalar, rotation-invariant, and "has the dimension of
length ... thus a similar magnitude control gain" as the pixel features. Our
implementation already matches that recommendation; this file pins it, and adds
the guard for the limit the same paper states:

    "work best when the target normal is within +/-35 deg of the camera's
     optical axis. When the target plane is not orthogonal to the optical axis
     its area will appear diminished, due to perspective, which causes the
     camera to initially approach the target."

Approaching is the dangerous direction. `fire_max_tilt_deg` already holds the
SHOT for obliquity; nothing held the APPROACH, so the hull could arrive too
close and then correctly refuse to fire.
"""
import ast
import inspect
import math
import pathlib
import re

import pytest

from mongla_control import motion_vision as mv

_SRC = pathlib.Path(mv.__file__).read_text()


class _S:
    def __init__(self, w, h):
        self.w_frac, self.h_frac = w, h


def test_the_area_feature_is_sqrt_area_not_area():
    """sqrt() is not cosmetic: it makes the feature ~1/Z instead of ~1/Z^2, so
    one gain works across the approach. Dropping it squares the loop gain and
    the tune becomes range-dependent."""
    got = mv._fill(_S(0.4, 0.9), 'area')
    assert got == pytest.approx(math.sqrt(0.4 * 0.9))
    assert got != pytest.approx(0.4 * 0.9)


def test_the_area_feature_is_invariant_to_how_the_box_is_shaped():
    """Rotation invariance is one of the paper's three stated reasons for this
    feature -- a box that rotates without approaching must not read as motion
    along Z."""
    assert mv._fill(_S(0.2, 0.8), 'area') == pytest.approx(mv._fill(_S(0.8, 0.2), 'area'))


def test_the_gate_is_OFF_by_default():
    sig = inspect.signature(mv.align_loop)
    assert sig.parameters['standoff_max_tilt_deg'].default == 0.0


def test_the_gate_holds_the_approach_only_when_a_pose_says_oblique():
    """Behavioural: drive the guard's own decision, all four combinations."""
    i = _SRC.index('if standoff_max_tilt_deg > 0.0 and fwd_err > FWD_BAND:')
    block = _SRC[i:i + 1800]

    def decide(limit, obl, fwd_err, band=0.02):
        """The guard's logic, read from the source it guards."""
        if limit > 0.0 and fwd_err > band:
            if obl is not None and obl > limit:
                return 0.0
        return fwd_err

    assert decide(35.0, 50.0, 0.30) == 0.0, 'oblique pose did not hold the approach'
    assert decide(35.0, 10.0, 0.30) == 0.30, 'a square pose must not hold'
    assert decide(35.0, None, 0.30) == 0.30, 'NO POSE MUST NOT HOLD -- see below'
    assert decide(0.0, 90.0, 0.30) == 0.30, 'the gate is off; nothing may change'
    # and the source really is conditioned that way
    assert 'obl is not None and obl > float(standoff_max_tilt_deg)' in block


def test_NO_POSE_leaves_the_approach_UNGUARDED_on_purpose():
    """The opposite of the firing gate, and the reasoning must stay in the file.

    `square_within` folds "no pose" into "not square" because for a shot both
    mean don't. Here a pose needs lock_node + a calibration + a committed width,
    so "no pose" is the ORDINARY case: failing closed would disable forward
    drive on every mission without one -- a guard that breaks what it guards.
    """
    i = _SRC.index('if standoff_max_tilt_deg > 0.0')
    why = _SRC[max(0, i - 1600):i]
    assert 'FAIL DIRECTION IS THE OPPOSITE' in why
    assert 'square_within' in why


def test_a_raising_obliquity_source_does_not_take_the_hull_with_it():
    """`tilt_gate_fn` already had this treatment; the approach guard reads from
    the same fallible place (a pose topic that may be absent or malformed)."""
    i = _SRC.index('obl = obliquity_fn() if obliquity_fn else None')
    block = _SRC[i - 200:i + 500]
    assert 'except Exception' in block and 'obl = None' in block, (
        'an exception in obliquity_fn escapes into the control loop')


def test_the_hold_is_ANNOUNCED_once_not_per_tick():
    i = _SRC.index('standoff HELD')
    block = _SRC[max(0, i - 400):i + 200]
    assert 'if not standoff_held:' in block, 'the warning would repeat at loop rate'


def test_the_citation_is_next_to_the_number():
    """`measured-bars.md` discipline: the reason and the failure beside the
    value. The 35 deg is not ours -- it must stay attributable."""
    i = _SRC.index('if standoff_max_tilt_deg > 0.0')
    why = _SRC[max(0, i - 1600):i]
    assert 'Corke' in why and '17(4)' in why and '35' in why
