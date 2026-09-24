"""A fouled lens and a changed world must not arrive as the same event.

They need opposite responses -- stop trusting vision, versus go and search --
and today both reach the ladder as "the anchor is not matching". The separator
needs no new sensor: a dirty port collapses the yield against EVERY reference
at once, while a moved target collapses only the one that used to win, and the
bank already computes both numbers on every lookup.

⚠ These test the SEPARATOR, not the phenomenon. No port has been fouled on
purpose, so the thresholds are declared rather than measured and the tests say
what the logic does, not what silt does. `measured-bars.md` records that
distinction; a test cannot.
"""
import pytest

from mongla_vision.anchor import health


def test_everything_scoring_is_healthy():
    v = health.assess([90, 80, 70, 60], [100, 100, 100, 100])
    assert v.state == health.HEALTHY and v.trust_vision


def test_one_reference_collapsing_is_the_world():
    """The target left, or the vehicle turned. The camera is fine, and the
    other references prove it."""
    v = health.assess([2, 80, 70, 60], [100, 100, 100, 100])
    assert v.state == health.WORLD
    assert v.trust_vision, 'a changed world must not disarm vision'
    assert '3' not in v.reason.split('/')[0]      # 1 collapsed, not 3


def test_every_reference_collapsing_is_the_camera():
    """Silt, a bubble, condensation, the lights off. Nothing the vehicle sees
    can be trusted, and hunting for a better view cannot help."""
    v = health.assess([1, 2, 0, 3], [100, 100, 100, 100])
    assert v.state == health.CAMERA
    assert not v.trust_vision


def test_where_the_boundary_actually_SITS():
    """⚠ The bar is 0.8 of the searched references, and it is DECLARED, not
    measured -- no port has been fouled on purpose.

    So this pins where it sits rather than pretending a measurement chose it.
    3 of 4 is 0.75 and reads WORLD; 4 of 5 is 0.8 and reads CAMERA. The
    strictness is deliberate: calling a fouled lens is an instruction to stop
    believing the cameras, and one surviving reference is real evidence that
    the glass is clear. Sharpen this the first time a real fouled port is
    recorded, and move the number into measured-bars.md when it is.
    """
    three_of_four = health.assess([1, 2, 0, 90], [100, 100, 100, 100])
    assert three_of_four.state == health.WORLD
    four_of_five = health.assess([1, 2, 0, 3, 90],
                                 [100, 100, 100, 100, 100])
    assert four_of_five.state == health.CAMERA
    assert health.FOULED_FRACTION == 0.8, (
        'the bar moved; the boundary cases above describe 0.8 and this test '
        'is the only place that records which number was chosen')


def test_half_collapsing_is_the_world_not_the_camera():
    """⛔ THE ASYMMETRY IS DELIBERATE. Calling a fouled lens is an instruction
    to stop believing the cameras, so it needs most of the evidence, not
    half."""
    v = health.assess([1, 2, 80, 90], [100, 100, 100, 100])
    assert v.state == health.WORLD and v.trust_vision


# --------------------------------------------------------------------------- #
#  Refusing to answer, which matters more than answering
# --------------------------------------------------------------------------- #
def test_too_few_references_refuses():
    """One reference cannot tell 'every view failed' from 'the only view
    failed'."""
    v = health.assess([0, 0], [100, 100])
    assert v.state == health.UNKNOWN


def test_unknown_does_not_disarm_vision():
    """'I cannot tell' is not evidence of a fouled lens. Treating it as one
    makes every quiet moment look like a failure."""
    assert health.assess([0], [100]).trust_vision


def test_references_that_never_scored_are_not_counted_as_collapsed():
    """A reference with no history has not failed -- it has never been asked.
    Counting its silence is how a healthy camera gets reported as fouled."""
    v = health.assess([0, 0, 0, 90, 80, 70],
                      [0, 0, 0, 100, 100, 100])
    assert v.state != health.CAMERA
    assert v.considered == 3, 'the never-scored references were counted'


def test_each_reference_is_judged_against_itself():
    """A whole-frame floor reference and a cropped prop reference are not
    comparable, so an absolute bar would report the bank's COMPOSITION rather
    than the camera's state."""
    # Reference 0 is intrinsically rich (900), reference 1 intrinsically poor
    # (40). Both are holding up at ~80 % of their own best.
    v = health.assess([720, 32, 700], [900, 40, 880])
    assert v.state == health.HEALTHY, (
        'a low-scoring but healthy reference was read as a collapse')


def test_every_verdict_explains_itself():
    for cur, best in (([90, 80, 70], [100, 100, 100]),
                      ([1, 2, 0], [100, 100, 100]),
                      ([1, 80, 70], [100, 100, 100]),
                      ([0], [100])):
        v = health.assess(cur, best)
        assert v.reason.strip(), f'silent verdict for {cur}'


# --------------------------------------------------------------------------- #
#  The bank must actually hand over the numbers
# --------------------------------------------------------------------------- #
def test_the_bank_keeps_the_per_reference_yields():
    """They were computed on every lookup and thrown away."""
    from mongla_vision.anchor.bank import CheckpointBank
    assert hasattr(CheckpointBank, 'last_lookup_scores'), (
        'the bank no longer exposes per-reference yields, so the separator '
        'has nothing to read and silently degrades to "unknown" forever')
