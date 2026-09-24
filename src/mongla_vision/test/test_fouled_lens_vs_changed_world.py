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


# --------------------------------------------------------------------------- #
#  Memory contamination: the bank must not remember a view of the FAULT
# --------------------------------------------------------------------------- #
def test_a_fouled_camera_refuses_enrolment():
    """⛔ A reference snapped while the port is silted is a view of the fault,
    not of the target, and once stored every later lookup matches against it.

    DAM4SAM measures robustness 0.887 -> 0.944 for keeping degraded frames out
    of the reliable memory; we get the same protection without a second bank,
    because health.py already separates a fouled camera from a changed world.
    """
    import numpy as np
    from mongla_vision.anchor.bank import CheckpointBank

    class _Be:
        w, h = 320, 240

        def detect(self, gray):
            n = 64
            d = np.random.default_rng(0).normal(size=(n, 64)).astype(np.float32)
            return np.zeros((n, 2), np.float32), d

    b = CheckpointBank(_Be())
    # Pretend a lookup just happened in which EVERY reference collapsed --
    # the camera verdict.
    b._refs = [object(), object(), object(), object()]
    b._yields = [100, 100, 100, 100]
    b._last_scores = {0: 1, 1: 2, 2: 0, 3: 3}
    r = b.enrol(np.zeros((240, 320), np.uint8), det_conf=1.0)
    assert not r.accepted and r.reason == 'camera degraded'
    assert b._contaminated_refusals == 1


def test_a_changed_world_does_NOT_block_enrolment():
    """The camera is fine and the view moved -- that is exactly when a new
    reference is worth having."""
    import numpy as np
    from mongla_vision.anchor.bank import CheckpointBank

    class _Be:
        w, h = 320, 240

        def detect(self, gray):
            n = 64
            d = np.random.default_rng(1).normal(size=(n, 64)).astype(np.float32)
            return np.zeros((n, 2), np.float32), d

    b = CheckpointBank(_Be())
    b._refs = [object(), object(), object(), object()]
    b._yields = [100, 100, 100, 100]
    b._last_scores = {0: 1, 1: 90, 2: 80, 3: 70}   # one collapsed, rest fine
    r = b.enrol(np.zeros((240, 320), np.uint8), det_conf=1.0, force=True)
    assert r.reason != 'camera degraded'


def test_force_overrides_the_contamination_guard():
    """A deliberate operator snap is a decision, not an accident."""
    import numpy as np
    from mongla_vision.anchor.bank import CheckpointBank

    class _Be:
        w, h = 320, 240

        def detect(self, gray):
            n = 64
            d = np.random.default_rng(2).normal(size=(n, 64)).astype(np.float32)
            return np.zeros((n, 2), np.float32), d

    b = CheckpointBank(_Be())
    b._refs = [object(), object(), object(), object()]
    b._yields = [100, 100, 100, 100]
    b._last_scores = {0: 1, 1: 2, 2: 0, 3: 3}
    r = b.enrol(np.zeros((240, 320), np.uint8), det_conf=1.0, force=True)
    assert r.reason != 'camera degraded'


# --------------------------------------------------------------------------- #
#  ⭐ ANTICIPATION -- predicting a loss instead of reacting to one
# --------------------------------------------------------------------------- #
def test_a_falling_trend_says_prepare():
    """The reference worth having is the one from BEFORE the detection is
    lost. A falling match quality is the only warning available in time."""
    t = health.trend([200, 190, 80, 60])
    assert t.state == health.FALLING and t.prepare


def test_a_steady_trend_does_not_fire():
    assert not health.trend([100, 110, 95, 105]).prepare


def test_an_improving_trend_does_not_fire():
    assert not health.trend([60, 80, 190, 200]).prepare


def test_too_few_samples_is_not_a_trend():
    """Noise with an opinion. Three points can look like anything."""
    t = health.trend([200, 50])
    assert t.state == health.STEADY and 'needed for a trend' in t.reason


def test_an_already_dead_signal_does_not_report_a_fall():
    """There is no fall left to detect, and the health verdict is the right
    instrument for that state."""
    assert not health.trend([0, 0, 0, 0]).prepare


def test_the_trend_never_claims_the_target_is_gone():
    """⛔ It licenses ONE action -- enrol now -- and no other. The ladder's
    rule stands: no rung fabricates a position."""
    t = health.trend([300, 280, 40, 30])
    assert t.prepare
    assert not hasattr(t, 'lost') and not hasattr(t, 'target_gone')


def test_lock_node_uses_the_trend_as_an_enrolment_REASON():
    """Wired, not merely available -- the defect this repo produces most."""
    import pathlib
    src = (pathlib.Path(__file__).resolve().parents[1] / 'mongla_vision'
           / 'lock_node.py').read_text()
    assert '_anchor_trend' in src and '_health.trend(' in src
    assert 'falling or' in src, (
        'the trend is computed but not used as a reason to enrol, so the '
        'anticipation never reaches the bank')
