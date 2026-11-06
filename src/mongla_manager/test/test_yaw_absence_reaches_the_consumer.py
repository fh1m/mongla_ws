"""A NaN yaw must arrive at `/mongla/state` as ABSENCE, not as a number.

The producer half is `SrotFC.get_attitude`, gated on the board's own
SYS_STATUS health bits (see `test_srot_ahrs_health.py` for the live
measurement that prompted it). This file is the CONSUMER half, and it exists
because a gate whose value never reaches the thing it protects is a knob wired
to nothing.

The chain under test is the one the node runs: the board's attitude ->
`BoardYaw.read_yaw()` -> `AUVManagerNode._effective_yaw_deg`. Every guard
downstream compares with `<` or `>`, and every NaN comparison is silently
False -- so the failure mode is not a wrong heading, it is every heading check
quietly declining to fire.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from mongla_manager.auv_manager_node import AUVManagerNode      # noqa: E402
from mongla_manager.yaw_source import BoardYaw, STALE_SECONDS   # noqa: E402

_effective_yaw_deg = AUVManagerNode._effective_yaw_deg


class _Board:
    """The two calls `BoardYaw` makes, nothing else."""

    def __init__(self, attitude, age: "float | None" = 0.01):
        self._attitude, self._age = attitude, age

    def get_attitude(self):
        return self._attitude

    def get_attitude_age(self):
        return self._age


class _Node:
    def __init__(self, attitude, age: "float | None" = 0.01):
        self.yaw_source = BoardYaw(_Board(attitude, age))


def _yaw(attitude, age: "float | None" = 0.01):
    node = _Node(attitude, age)
    return _effective_yaw_deg(node, attitude)


def test_nan_yaw_is_reported_as_absent():
    yaw, label = _yaw({'yaw': float('nan')})
    assert yaw is None, 'NaN must not be published as a heading'
    assert label == 'N/A'


def test_a_real_yaw_still_comes_through():
    assert _yaw({'yaw': 137.5}) == (137.5, 'AHRS')


def test_zero_is_a_legitimate_heading():
    """0.0 is due north and must survive. The fix is about NaN, not about
    treating a falsy number as missing -- which is how this class of guard is
    usually got wrong."""
    assert _yaw({'yaw': 0.0}) == (0.0, 'AHRS')


def test_no_attitude_at_all_is_still_absent():
    assert _yaw(None) == (None, 'N/A')


def test_a_stale_attitude_is_absent_not_the_last_cached_yaw():
    """After a USB drop pymavlink keeps the last ATTITUDE forever. The yaw is
    a perfectly good number -- and it is old, so it is absent."""
    assert _yaw({'yaw': 137.5}, age=STALE_SECONDS + 0.05) == (None, 'N/A')
    assert _yaw({'yaw': 137.5}, age=None) == (None, 'N/A')
