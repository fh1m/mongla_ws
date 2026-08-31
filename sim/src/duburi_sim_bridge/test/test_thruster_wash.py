"""The wash's thrust sum, which is where its sign errors hide.

`_AXES` is a derivation from the model SDF -- joint axis (0, 0, -1) in a child
link posed rpy (-90, 90, yaw), with SDF rpy being Rz.Ry.Rx -- and a derivation
nobody checks is a guess. Getting it wrong points the jet somewhere plausible
and wrong, silently, which is this module's recurring failure mode.
"""

import math

import pytest

from duburi_sim_bridge.thruster_wash import _AXES, net_body_thrust


def _cmd(t1, t2, t3, t4):
    return {1: t1, 2: t2, 3: t3, 4: t4}


def test_four_equal_commands_are_a_yaw_and_make_no_jet():
    """This is the one that bites.

    Four equal positive thrusts LOOK like full-ahead and are a pure yaw: the
    axes cancel. The A/B rig for this module drove exactly that and expected a
    forward drive; the node reported 0.00 N and was right. A scalar sum of
    magnitudes would have claimed 146 N of jet from a vehicle going nowhere.
    """
    bx, by = net_body_thrust(_cmd(36.6, 36.6, 36.6, 36.6))
    assert bx == pytest.approx(0.0, abs=1e-9)
    assert by == pytest.approx(0.0, abs=1e-9)


def test_forward_drive_is_t1_t2_astern_and_lands_on_body_x():
    bx, by = net_body_thrust(_cmd(-36.6, -36.6, 36.6, 36.6))
    assert bx == pytest.approx(4 * 36.6 * math.sqrt(0.5), rel=1e-6)   # +103.52
    assert by == pytest.approx(0.0, abs=1e-9)


def test_lateral_drive_lands_on_body_y_and_not_on_x():
    """Strafe is the case the jet DIRECTION exists for: the wash goes sideways
    while the hull's heading does not, and the two are 90 degrees apart."""
    bx, by = net_body_thrust(_cmd(-36.6, 36.6, -36.6, 36.6))
    assert by == pytest.approx(4 * 36.6 * math.sqrt(0.5), rel=1e-6)
    assert bx == pytest.approx(0.0, abs=1e-9)


def test_every_axis_is_a_unit_vector():
    for i, (ax, ay) in _AXES.items():
        assert math.hypot(ax, ay) == pytest.approx(1.0, rel=1e-9), i
