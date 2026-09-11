"""A NaN yaw must arrive at `/duburi/state` as ABSENCE, not as a number.

The producer half is `SrotFC.get_attitude`, gated on the board's own
SYS_STATUS health bits (see `test_srot_ahrs_health.py` for the live
measurement that prompted it). This file is the CONSUMER half, and it exists
because a gate whose value never reaches the thing it protects is a knob wired
to nothing.

The branch under test had never run on the srot backend. On Pixhawk there is a
second AHRS to fall through to; on srot `attitude` is the only source, so
`_effective_yaw_deg` reached `return float(attitude['yaw']), 'AHRS'` and
handed NaN straight out. Every guard downstream compares with `<` or `>`, and
every NaN comparison is silently False -- so the failure mode is not a wrong
heading, it is every heading check quietly declining to fire.
"""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from duburi_manager.auv_manager_node import AUVManagerNode      # noqa: E402

_effective_yaw_deg = AUVManagerNode._effective_yaw_deg


class _NoYawSource:
    """No `yaw_source`, so the AHRS branch is the one exercised."""
    yaw_source = None


def test_nan_yaw_is_reported_as_absent():
    yaw, label = _effective_yaw_deg(_NoYawSource(), {'yaw': float('nan')})
    assert yaw is None, 'NaN must not be published as a heading'
    assert label == 'N/A'


def test_a_real_yaw_still_comes_through():
    yaw, label = _effective_yaw_deg(_NoYawSource(), {'yaw': 137.5})
    assert yaw == 137.5
    assert label == 'AHRS'


def test_zero_is_a_legitimate_heading():
    """0.0 is due north and must survive. The fix is about NaN, not about
    treating a falsy number as missing -- which is how this class of guard is
    usually got wrong."""
    yaw, label = _effective_yaw_deg(_NoYawSource(), {'yaw': 0.0})
    assert yaw == 0.0
    assert label == 'AHRS'


def test_no_attitude_at_all_is_still_absent():
    yaw, label = _effective_yaw_deg(_NoYawSource(), None)
    assert yaw is None and label == 'N/A'
