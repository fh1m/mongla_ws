"""The baro calibration must be skippable, and ON by default.

ArduSub SITL ACKs MAV_CMD_PREFLIGHT_CALIBRATION as ACCEPTED and then stops
tracking depth -- measured, the readback froze near 0 while the hull sat 1.2 m
down, which made `surface()` CONFIRM while submerged. A false pass is worse
than the hang it replaced, so the simulator turns this off.

The default must stay True: on the pool hull the calibration is real and is
what fixes the pre-dive Bar30 drift.
"""
from unittest.mock import MagicMock

import pytest

from mongla_control.mongla import Mongla


def _mongla(**kw):
    pixhawk = MagicMock()
    pixhawk.is_armed.return_value = False
    return Mongla(pixhawk, log=MagicMock(), **kw), pixhawk


def test_calibration_is_on_by_default():
    """The pool hull must keep calibrating -- nothing here changes that."""
    mongla, pixhawk = _mongla()
    assert mongla._baro_calibration is True


def test_disabled_skips_the_mavlink_command():
    mongla, pixhawk = _mongla(baro_calibration=False)
    mongla._run_baro_calibration(settle_s=0.0, strict=False)
    pixhawk.calibrate_barometer.assert_not_called()


def test_disabled_does_not_fail_mission_reset():
    """mission_reset calls this non-strict; it must never break the reset."""
    mongla, _ = _mongla(baro_calibration=False)
    res = mongla._run_baro_calibration(settle_s=0.0, strict=False)
    assert res.success is True


def test_disabled_is_reported_as_a_failure_to_the_standalone_verb():
    """`mongla calibrate_depth` asked for a calibration and did not get one."""
    mongla, _ = _mongla(baro_calibration=False)
    res = mongla._run_baro_calibration(settle_s=0.0, strict=True)
    assert res.success is False
    assert 'disabled' in res.message


def test_enabled_still_reaches_the_pixhawk():
    mongla, pixhawk = _mongla(baro_calibration=True)
    # A fresh surface reading, or the pre-cal bounds check short-circuits.
    pixhawk.get_attitude.return_value = {'yaw': 0.0, 'roll': 0.0,
                                         'pitch': 0.0, 'depth': -0.02}
    pixhawk.get_attitude_age.return_value = 0.0
    pixhawk.calibrate_barometer.return_value = (True, 'ACCEPTED')
    mongla._run_baro_calibration(settle_s=0.0, strict=False)
    pixhawk.calibrate_barometer.assert_called_once()
