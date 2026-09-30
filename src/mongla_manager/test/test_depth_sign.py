"""One depth, walked through all three interfaces (issue #19).

`MonglaState.depth_m` is ALTITUDE (negative below the surface); `/mongla/odom`
position.z and the VISION_POSITION_ESTIMATE uplink are NED (positive down).
Both are right where they live. The defect this guards is a fourth place
quietly picking the other sign -- which has happened: `SrotFC.get_attitude`
once negated an already-negative depth, and every depth guard, which compares
against a negative constant, simply stopped firing.

Truth: the hull is 1.2 m below the surface.
"""
import math
import sys
import pathlib

import pytest

from mongla_localization import frames

TRUE_DEPTH_BELOW_SURFACE = 1.2


def test_the_named_conversion_round_trips_and_has_the_right_sign():
    alt = -TRUE_DEPTH_BELOW_SURFACE
    assert frames.ned_z_from_altitude(alt) == pytest.approx(+1.2)
    assert frames.altitude_from_ned_z(frames.ned_z_from_altitude(alt)) == alt


def test_state_to_odom_to_uplink_all_agree_about_one_depth():
    """MonglaState (-1.2) -> the localization node -> /mongla/odom z (+1.2)
    -> the manager's uplink -> VISION_POSITION_ESTIMATE z (+1.2, NED)."""
    loc_tests = pathlib.Path(__file__).resolve().parents[2] / \
        'mongla_localization' / 'test'
    sys.path.insert(0, str(loc_tests))
    import test_localization_node as T
    from mongla_interfaces.msg import MonglaState

    n = T._node()
    T._fly_board(n, 100.0, 1.0, 0.0)
    for k in range(200):                       # 1.2 m down, repeatedly
        s = MonglaState()
        s.depth_m = -TRUE_DEPTH_BELOW_SURFACE
        s.yaw_deg = math.nan
        n._on_state(s)
    odom = []
    n._pub = type('P', (), {'publish': lambda _s, m: odom.append(m)})()
    n._publish()
    z_odom = odom[-1].pose.pose.position.z
    assert z_odom == pytest.approx(+1.2, abs=0.02), (
        f'/mongla/odom z = {z_odom:+.3f}: NED z must be POSITIVE below the '
        f'surface')

    from mongla_manager.auv_manager_node import AUVManagerNode
    sent = {}
    mgr = AUVManagerNode.__new__(AUVManagerNode)
    mgr.fc = type('FC', (), {'send_position_estimate':
                             lambda _s, x, y, z, *a: sent.update(z=z) or True})()
    mgr.get_logger = lambda: type('L', (), {'info': lambda *a: None,
                                            'warn': lambda *a: None})()
    mgr._send_position_uplink(odom[-1], 1.0)
    assert sent['z'] == pytest.approx(z_odom), (
        'the uplink changed the sign or the value of the odometry depth')
    assert sent['z'] > 0.0, 'VISION_POSITION_ESTIMATE is NED: z is +down'


def test_no_new_bare_depth_flip_in_the_estimator():
    """The flip lives in `frames`. A bare `-depth` in the filter is a second
    copy of the sign convention, and the next one to drift."""
    src = (pathlib.Path(__file__).resolve().parents[2] / 'mongla_localization'
           / 'mongla_localization' / 'inekf.py').read_text(encoding='utf-8')
    assert '-float(depth_m)' not in src
    assert 'ned_z_from_altitude(depth_m)' in src
