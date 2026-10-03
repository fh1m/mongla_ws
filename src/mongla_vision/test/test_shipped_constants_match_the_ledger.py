"""A shipped constant and the measurement behind it, compared mechanically.

⛔ THE DEFECT THIS CLOSES, AND IT IS THE EXPENSIVE ONE. `measured-bars.md`
holds 104 entries and is the reason anything here is believable. Nothing read
it. Tests checked that the SITE quoted the ledger correctly and that docs did
not go stale — but no test compared a **number in the code** against the
**number the ledger recorded for it**.

So `approach.py` shipped `BAND_LO = 0.25, BAND_HI = 0.45` citing section 23,
while section 23 measured a different quantity entirely (sqrt-area in pixels,
not fraction of frame height) and never stated those numbers. A citation
pointing at a real section, wrapped around an invented value, passed every
check we had for weeks. Re-measured, the peak sat at 0.45-0.60 for `gate` and
0.80-1.20 for `repair`: the shipped band matched **no class**, and for one of
them it would have commanded BACK OFF from the best view the detector gets.

⭐ THE RULE THIS MAKES EXECUTABLE is the repo's own: *one truth, two copies is
the bug -- make one read the other, or have a test compare them.* A constant
and its ledger entry are two copies. This is the comparison.

HOW TO ADD ONE. Put the constant in `LEDGER_BOUND` with the exact digits the
ledger states. The test then fails if either side moves without the other --
in BOTH directions, which is the point: a constant retuned without re-measuring
is as much a defect as a ledger that drifts from the code.

⚠ IT IS OPT-IN AND THAT IS DELIBERATE. Retro-fitting 104 entries would be
busywork and most constants are not ledger-bound. A number earns a row here
when getting it wrong costs a run.
"""
from __future__ import annotations

import importlib
import pathlib
import re

import pytest

_LEDGER = (pathlib.Path(__file__).resolve().parents[3]
           / '.claude' / 'context' / 'measured-bars.md')

# module path -> attribute -> (expected value, a string the ledger must contain
# near it, and why the number is what it is).
LEDGER_BOUND = {
    'mongla_vision.tracking.follower': {
        'MIN_PATCH_STD': (2.0, '86.7',
                          'grey levels; at this bar a refusal is right 86.7 % '
                          'of the time for a 19.4 % refusal rate'),
    },
    'mongla_vision.tracking.lock_state': {
        'DISAGREE_PX': (64.0, None,
                        'px between the two carried rungs before authority is '
                        'cut; generous on purpose so ordinary parallax does '
                        'not trip it'),
    },
    'mongla_vision.flow.flow_node': {
        'FLOW_SCALE_SIGMA_FRAC': (0.036, '1.09 cm on 30 cm — 3.6 %',
                                  'worst of three taped slides; the scale '
                                  'term of the published flow sigma (#23)'),
        'FLOW_TD_MEASURED_S': (-0.0113, 'td = -11.3 ms, sd 0.84 ms',
                               'camera<->gyro offset the online estimator '
                               'starts from, three 60 s runs on the vehicle'),
        'FLOW_ROT_RESIDUAL_CALIBRATED': (0.10, '575.7 → 57.1 mm/s',
                                         'rotation left after a calibrated '
                                         'de-rotation gain (#23)'),
    },
    'mongla_vision.flow.scale_check': {
        'DISAGREE_FRAC': (0.20, None,
                          'the grating and barometric heights may differ by '
                          'this much before it is a fault'),
    },
    'mongla_vision.approach': {
        # ⛔ THE ONE THAT WAS WRONG. Each band is a measured peak bin from
        # tools/approach_band.py, not a chosen range.
        'BANDS': ({'gate': (0.45, 0.60),
                   'rescue': (0.10, 0.20),
                   'repair': (0.80, 1.20)}, '0.45 - 0.60',
                  'per-class peak bins of box_h/frame_h; a global band '
                  'matched none of the three'),
    },
}


def _ledger_text():
    if not _LEDGER.is_file():
        pytest.skip(f'ledger not found at {_LEDGER}')
    return _LEDGER.read_text(encoding='utf-8')


@pytest.mark.parametrize('mod,attr', [(m, a) for m, d in LEDGER_BOUND.items()
                                      for a in d])
def test_the_shipped_value_is_the_one_in_the_register(mod, attr):
    """⛔ BOTH DIRECTIONS. A constant retuned without re-measuring is as much a
    defect as a ledger that drifted from the code -- this cannot tell which
    side moved, and says so rather than guessing."""
    want, _needle, why = LEDGER_BOUND[mod][attr]
    got = getattr(importlib.import_module(mod), attr)
    assert got == want, (
        f'{mod}.{attr} is {got!r}, the register says {want!r}.\n'
        f'  what the number means: {why}\n'
        f'  ONE OF THE TWO MOVED. If the code is right, re-measure and update '
        f'measured-bars.md AND this row together. If the code drifted, put it '
        f'back. Do not update this row alone -- that is the check.')


@pytest.mark.parametrize('mod,attr', [(m, a) for m, d in LEDGER_BOUND.items()
                                      for a in d
                                      if LEDGER_BOUND[m][a][1] is not None])
def test_the_ledger_still_states_the_number(mod, attr):
    """⚠ A value can match a row here and still have no measurement behind it
    -- which is exactly how the approach band survived. Check the LEDGER
    itself still contains the digits, so deleting the entry breaks the build
    rather than quietly orphaning the constant."""
    _want, needle, _why = LEDGER_BOUND[mod][attr]
    text = _ledger_text()
    assert needle in text, (
        f'{mod}.{attr} claims a measurement but "{needle}" is no longer in '
        f'measured-bars.md. Either the entry was deleted or renumbered, or '
        f'the constant is now unsupported -- and an unsupported constant in '
        f'the detection path is the defect this file exists for.')


def test_every_bound_module_actually_imports():
    """A row pointing at a module that moved is a check that silently stops
    checking."""
    for mod in LEDGER_BOUND:
        importlib.import_module(mod)


def test_the_register_is_not_empty_and_names_its_reasons():
    """⛔ A guard that has been emptied is worse than no guard: it still shows
    green. Every row must carry a reason, so nobody adds a bare number."""
    assert LEDGER_BOUND, 'the register was emptied'
    for mod, entries in LEDGER_BOUND.items():
        for attr, (_v, _n, why) in entries.items():
            assert why and len(why) > 20, f'{mod}.{attr} has no stated reason'
