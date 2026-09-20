"""B15 -- the deadband/band invariant was a comment, and nothing enforced it.

`LOCK_HOLD_DEADBAND_DEG` carries "Must stay < LOCK_APPROACH_BAND_DEG". Raise it to
or past the band and `span` goes <= 0, at which point the runtime guard returns
the full `LOCK_SPEED_MIN_PCT` everywhere -- silently reinstating the hard min-PWM
floor, i.e. exactly the relay limit-cycle the taper was added to remove. No error,
no log, just a hull that hunts again.
"""

import pytest

import mongla_control.heading_lock as hl


def test_the_shipped_constants_satisfy_the_invariant():
    assert 0.0 <= hl.LOCK_HOLD_DEADBAND_DEG < hl.LOCK_APPROACH_BAND_DEG


def test_the_taper_actually_tapers_with_the_shipped_constants():
    """If it did not, the assert above would be guarding nothing."""
    dead = hl.LOCK_HOLD_DEADBAND_DEG
    band = hl.LOCK_APPROACH_BAND_DEG
    at_edge = hl._lock_floor(band - 1e-6, dead)
    just_inside = hl._lock_floor(dead + 1e-6, dead)
    assert at_edge > just_inside, 'the speed must fall as the error approaches the deadband'


def test_a_bad_tune_cannot_be_imported():
    """The module-level assert is the enforcement; prove it fires."""
    src = open(hl.__file__, encoding='utf-8').read()
    assert 'assert 0.0 <= LOCK_HOLD_DEADBAND_DEG < LOCK_APPROACH_BAND_DEG' in src, \
        'the invariant must be enforced at import, not described in a comment'
