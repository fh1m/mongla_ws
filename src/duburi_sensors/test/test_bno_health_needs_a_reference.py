"""B04 -- an uncalibrated BNO reported healthy and steered as if Earth-referenced.

`read_yaw()` falls back to the RAW boot-relative angle when calibration failed
("raw mode (diag only)"), and `is_healthy()` was `read_yaw() is not None`. So a
failed calibration reported HEALTHY while HeadingLock, motion_yaw._YawPID and
heading_error all treated a boot-relative number as an absolute compass heading:
`turn(90)` went to 90 deg in a frame nobody knows, with a fixed unknown bias in
[0,360) that neither looks like noise nor decays.

The guarded failure is the EXPECTED one -- its own comment says "Pixhawk AHRS slow
to warm", i.e. a cold boot, i.e. pool day.
"""

import pytest

from duburi_sensors.sources.bno085 import BNO085Source


def _src(offset, required, raw=42.0):
    s = BNO085Source.__new__(BNO085Source)
    s._offset_deg = offset
    s._calibration_required = required
    s._fresh_raw_yaw = lambda: raw
    return s


def test_calibration_required_but_failed_is_NOT_healthy():
    s = _src(offset=None, required=True)
    assert s.read_yaw() == 42.0, 'raw mode still returns a number for diagnostics'
    assert s.is_healthy() is False, \
        'a boot-relative yaw must not be reported as a usable heading'


def test_calibrated_is_healthy():
    assert _src(offset=10.0, required=True).is_healthy() is True


def test_raw_mode_requested_deliberately_is_still_healthy():
    """`calibrate=False` is a documented diagnostic mode; do not break it."""
    assert _src(offset=None, required=False).is_healthy() is True


def test_calibrated_but_stale_is_not_healthy():
    assert _src(offset=10.0, required=True, raw=None).is_healthy() is False
