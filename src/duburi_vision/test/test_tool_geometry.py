"""Aim the TOOL, not the camera.

⛔ THE DEFECT. Every vision align centres the target on the CAMERA axis and
then actuates a device mounted somewhere else. The axes are parallel, so

    THE MISS EQUALS THE OFFSET, AT EVERY RANGE.

It does not shrink as the hull closes in -- a 10 cm offset misses a
4.75 cm-radius torpedo opening from 1 m and from 3 m alike. Nothing logged a
fault because from the camera's point of view the shot was perfectly centred
and the fire gate was clean.

The correction is the OPPOSITE of a constant: `du = fx*x/Z`, so it is large up
close. That is why it needs a live range, and why it could not have been
computed correctly before the flat-port refraction fix made `target_pose`
metric.
"""
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import duburi_vision.tool_geometry as tg                       # noqa: E402
from duburi_vision.tool_geometry import (                      # noqa: E402
    camera_for, is_measured, known_tools, offset_m, pixel_offset)

FX, FY = 851.2, 858.2          # measured forward camera


def test_the_table_loads_and_names_the_real_tools():
    t = known_tools()
    assert {'torpedo', 'dropper', 'grabber'} <= set(t), t
    assert 'camera_forward' in t and 'camera_downward' in t


def test_each_tool_declares_which_camera_aims_it():
    """A dropper aimed by the forward camera is a wiring error that would
    produce a confident, wrong correction."""
    assert camera_for('dropper') == 'downward'
    assert camera_for('torpedo') == 'forward'
    assert camera_for('grabber') == 'forward'


def test_an_unmeasured_tool_is_DISTINGUISHABLE_from_a_zero_one():
    """⛔ A zero offset and an unmeasured one are the same NUMBER and not the
    same CLAIM. `camera_forward` is exactly zero from itself; `torpedo` is zero
    because nobody has held a tape to it. Collapsing them makes the warning
    that matters unprintable."""
    assert is_measured('camera_forward') is True
    assert is_measured('torpedo') is False
    assert offset_m('camera_forward') == offset_m('torpedo') == (0.0, 0.0, 0.0)


def test_every_placeholder_reads_as_zero_so_behaviour_is_unchanged():
    """Shipping a guessed offset would move the aim point with false
    confidence -- strictly worse than a known-absent one."""
    for t in known_tools():
        if not is_measured(t):
            assert offset_m(t) == (0.0, 0.0, 0.0), f'{t} guesses an offset'


def test_an_unknown_tool_is_None_not_zero():
    """None means 'aim the camera and say so'. Zero would mean 'I know it is
    centred', which is a different and unearned claim."""
    assert offset_m('no_such_tool') is None
    assert pixel_offset('no_such_tool', fx=FX, fy=FY, range_m=1.5) is None


def test_the_correction_is_range_dependent_and_grows_up_close():
    """The property that makes this more than a constant, and the one most
    likely to be 'simplified' away."""
    tg._CACHE['probe'] = {'camera': 'forward', 'x': 0.04, 'y': 0.12,
                          'z': 0.0, 'unmeasured': True}
    try:
        far = pixel_offset('probe', fx=FX, fy=FY, range_m=1.5)
        near = pixel_offset('probe', fx=FX, fy=FY, range_m=0.8)
        assert far == pytest.approx((FX * 0.04 / 1.5, FY * 0.12 / 1.5))
        assert abs(near[1]) > abs(far[1]) * 1.8, 'the correction must grow up close'
    finally:
        tg._CACHE.pop('probe', None)


def test_no_range_refuses_rather_than_assuming_one():
    """A nominal standoff would be a guess wearing a correction's clothes."""
    tg._CACHE['probe'] = {'camera': 'forward', 'x': 0.04, 'y': 0.12,
                          'z': 0.0, 'unmeasured': True}
    try:
        for bad in (0.0, -1.0, float('nan'), None):
            assert pixel_offset('probe', fx=FX, fy=FY, range_m=bad) is None
    finally:
        tg._CACHE.pop('probe', None)


def test_no_calibration_refuses():
    assert pixel_offset('torpedo', fx=0.0, fy=0.0, range_m=1.5) is None


def test_the_sign_convention_is_written_down_where_the_numbers_are():
    """A sign error here aims the tool the wrong way by twice the offset, and
    is invisible. The convention must live beside the values, not in a commit
    message."""
    y = (Path(__file__).resolve().parents[1] / 'config'
         / 'tool_geometry.yaml').read_text()
    assert '+RIGHT' in y and '+DOWN' in y and '+FORWARD' in y
    assert 'image-Y convention' in y, 'y-down vs world-up must be stated'


def test_z_is_documented_as_NOT_APPLIED():
    """Carrying a field the code ignores is fine; carrying one a reader
    ASSUMES is applied is not."""
    y = (Path(__file__).resolve().parents[1] / 'config'
         / 'tool_geometry.yaml').read_text()
    assert 'NOT currently applied' in y
