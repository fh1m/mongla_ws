"""The allocator's scale-down, graded honestly while it does not yet exist.

`mixer::mix()` scales a thruster group down uniformly when its mix exceeds
unity, then discards the factor. From here a vehicle that is not turning looks
identical whether the water is pushing back or the allocator binned a third of
the yaw command -- and the board's own PIDs cannot see it either, because their
anti-windup keys on each PID's own clamp rather than on the mixer.

Asked for in srot-control-board#20 as MIX_SAT_H / MIX_SAT_V. Until that merges
these read None, and the grading of that absence is the whole point of this
file: the reporter must NOT say OK about something it cannot see.
"""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from mongla_manager.health import State                    # noqa: E402
from mongla_manager.health_reporters import allocator      # noqa: E402


def test_absent_telemetry_is_UNKNOWN_not_OK():
    """The names do not exist yet. Saying OK would assert the one thing we
    cannot observe -- the same false-OK the thruster reporter exists to avoid,
    where 958 CRC-valid ESC frames arrived with nothing plugged in."""
    for absent in (None, (None, None)):
        h = allocator(absent)
        assert h.state is State.UNKNOWN
        assert '#20' in h.evidence, 'the reason it is absent should be findable'


def test_a_clean_reading_is_OK():
    assert allocator((1.0, 1.0)).state is State.OK


def test_saturation_is_DEGRADED_not_FAILED():
    """A hard manoeuvre is ALLOWED to ask for more than the hull has. This is
    worth surfacing, not worth refusing to fly over."""
    h = allocator((0.667, 1.0))
    assert h.state is State.DEGRADED
    assert '33 %' in h.evidence, 'the magnitude is the useful part'


def test_ONE_group_reading_clean_does_not_clear_the_other():
    """The subtle one, and it was wrong in the first draft. Reporting OK on
    (1.0, None) asserts the vertical stack is delivering when we never heard
    from it."""
    h = allocator((1.0, None))
    assert h.state is State.UNKNOWN
    assert 'half' in h.evidence


def test_an_observed_saturation_outranks_an_unseen_group():
    """What we DID observe is true regardless of what we did not -- a positive
    finding must not be downgraded to UNKNOWN by a missing sibling."""
    h = allocator((None, 0.5))
    assert h.state is State.DEGRADED
    assert '50 %' in h.evidence


@pytest.mark.parametrize('sat', [None, (None, None), (1.0, 1.0), (0.667, 1.0),
                                 (1.0, None), (None, 0.5)])
def test_every_grading_says_something_and_never_raises(sat):
    h = allocator(sat)
    assert h.state in (State.OK, State.DEGRADED, State.UNKNOWN)
    assert h.evidence.strip()


def test_the_reporter_is_REGISTERED_not_merely_defined():
    """A grader nothing calls is the defect this repo keeps producing -- most
    recently `lock_s`, which was declared, documented, mapped and never passed."""
    src = (Path(__file__).resolve().parents[1] / 'mongla_manager'
           / 'auv_manager_node.py').read_text()
    assert "self._health.register('allocator'" in src
    assert 'allocator_saturation' in src, 'registered but reading nothing'


def test_the_accessor_exists_on_the_backend_with_both_names():
    from mongla_control.fc.srot_fc import SrotFC
    assert hasattr(SrotFC, 'allocator_saturation')
    assert SrotFC.ALLOCATOR_SAT_NAMES == ('MIX_SAT_H', 'MIX_SAT_V')
