"""The approach band is per class, and an unmeasured class gets no advice.

⛔ THE DEFECT THIS CLOSES. `approach.py` shipped `BAND_LO = 0.25, BAND_HI =
0.45` and cited section 23 for it. Section 23 measured confidence against
sqrt(box area) IN PIXELS, by quartile; this module compares `box_height /
frame_height`. Different quantities, no recorded conversion -- so the shipped
numbers were an invention with a citation attached.

Re-measured in the controller's own unit (`tools/approach_band.py`, 13 668
detections, four archive clips, the shipped gate_rescue_repair graph):

    gate      peak 0.45 - 0.60
    rescue    peak 0.10 - 0.20
    repair    peak 0.80 - 1.20   (monotonic to the frame edge)

The shipped band matched NONE of them, and for `repair` it was wrong in the
exact way this module exists to prevent: it would have commanded BACK OFF from
the best view the detector ever gets.
"""
from __future__ import annotations

import pytest

from mongla_vision.approach import BACK_OFF, BANDS, CLOSE, HOLD, advise


def _at(frac, cls):
    return advise(frac * 480.0, 480.0, cls)


def test_the_old_global_band_is_gone():
    """⛔ A module-level BAND_LO/BAND_HI is the defect itself: one band cannot
    be right for three classes whose peaks are 0.15, 0.52 and 0.80+."""
    import mongla_vision.approach as m

    assert not hasattr(m, 'BAND_LO')
    assert not hasattr(m, 'BAND_HI')


def test_every_shipped_band_is_a_measured_peak():
    """The table is the measurement. Changing a number here means re-running
    tools/approach_band.py, not editing a constant."""
    assert BANDS['gate'] == (0.45, 0.60)
    assert BANDS['rescue'] == (0.10, 0.20)
    assert BANDS['repair'] == (0.80, 1.20)


def test_repair_is_not_told_to_back_off_from_its_best_view():
    """⭐⭐ THE CASE THE OLD BAND GOT BACKWARDS. `repair` confidence rises to
    0.526 in the 0.80-1.20 bin -- the largest the archive shows. Under the old
    global 0.25-0.45 band, 0.90 of frame read as BACK OFF."""
    assert _at(0.90, 'repair').action == HOLD
    assert _at(0.30, 'repair').action == CLOSE


def test_gate_backs_off_past_its_own_peak():
    """⚠ AND THE COUNTER-INTUITIVE HALF IS STILL EXPRESSED, for the class that
    actually measured a fall."""
    assert _at(0.80, 'gate').action == BACK_OFF
    assert _at(0.50, 'gate').action == HOLD
    assert _at(0.10, 'gate').action == CLOSE


def test_rescue_holds_far_earlier_than_gate():
    """The three classes disagree by a factor of five on where to stop, which
    is why one global band could not have been right."""
    assert _at(0.15, 'rescue').action == HOLD
    assert _at(0.15, 'gate').action == CLOSE


def test_an_unmeasured_class_holds_and_says_why():
    """⛔ NO DEFAULT BAND. A default is exactly what was wrong, so an unknown
    class must not silently inherit one."""
    r = _at(0.50, 'shark')
    assert r.action == HOLD
    assert 'no measured band' in r.reason
    assert 'approach_band' in r.reason, 'the reason must name the way to fix it'


def test_an_empty_class_name_also_holds():
    """A caller that forgot to pass the class must not get advice by accident."""
    assert advise(240.0, 480.0).action == HOLD
    assert advise(240.0, 480.0, '').action == HOLD


def test_the_class_name_is_matched_case_insensitively():
    """Detector class strings arrive in whatever case the dataset used."""
    assert _at(0.50, 'GATE').action == HOLD
    assert _at(0.50, ' Gate ').action == HOLD


def test_a_missing_size_still_holds_rather_than_guessing():
    assert advise(0.0, 480.0, 'gate').action == HOLD
    assert advise(None, 480.0, 'gate').action == HOLD
    assert advise(240.0, 0.0, 'gate').action == HOLD


def test_a_caller_may_supply_its_own_measured_table():
    """A new model means new class ids and new peaks; the table is an argument
    so a re-measure does not need a code change."""
    r = advise(0.50 * 480, 480, 'widget', bands={'widget': (0.40, 0.60)})
    assert r.action == HOLD and 'inside the band' in r.reason
