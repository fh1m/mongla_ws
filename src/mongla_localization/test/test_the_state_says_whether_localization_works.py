"""`/mongla/localization/state` answers "is localization working" on its own.

Before it, the answer was spread over `/mongla/odom`'s frame_id, two latched
strings and a log line every 5 s, and a mission could read none of them in one
place. The node's own timer drives these tests, no rclpy.
"""
import math

import pytest

from mongla_localization import localization_publish as lp

from test_localization_node import _Imu, _node


def _states(n):
    out = []
    n._pub = type('P', (), {'publish': lambda _s, m: None})()
    n._pub_state = type('P', (), {'publish': lambda _s, m: out.append(m)})()
    return out


def test_a_node_with_no_input_says_so_and_publishes_no_number():
    n = _node()
    out = _states(n)
    n._publish()
    s = out[-1]
    assert s.frame == '' and not s.usable
    assert 'no IMU' in s.reason
    assert math.isnan(s.x_m) and math.isnan(s.sigma_xy_m) and math.isnan(s.imu_age_s)


def test_a_live_unanchored_filter_names_the_anchor_as_the_missing_step():
    n = _node(disarmed=True)
    out = _states(n)
    for k in range(20):
        n._on_imu(_Imu(100.0 + 0.02 * k))
    n._publish()
    s = out[-1]
    assert s.frame == 'odom_ned'
    assert math.isfinite(s.x_m) and s.sigma_xy_m >= 0.0
    assert s.imu_age_s < 1.0
    assert not s.usable and 'anchored' in s.reason


@pytest.mark.parametrize('age,anchored,aiding,want', [
    (math.nan, True, 'aided', 'no IMU'),
    (2.0, True, 'aided', 'silent'),
    (0.1, False, 'aided', 'anchored'),
    (0.1, True, 'unaided', 'unaided'),
    (0.1, True, '', 'unaided'),
    (0.1, True, 'aided (flow uncalibrated)', ''),
    (0.1, True, 'aided', ''),
])
def test_the_verdict(age, anchored, aiding, want):
    got = lp.verdict(imu_age_s=age, anchored=anchored, aiding=aiding)
    assert (want in got) if want else got == ''


def test_the_horizontal_sigma_is_the_worse_axis_of_a_rotated_ellipse():
    m = type('O', (), {})()
    m.pose = type('P', (), {})()
    c = [0.0] * 36
    # var 0.04 along one diagonal, 0.01 across it: worse axis sigma 0.2 m.
    c[0] = c[7] = 0.025
    c[1] = c[6] = 0.015
    c[35] = math.radians(3.0) ** 2
    m.pose.covariance = c
    sxy, syaw = lp._horizontal_sigmas(m)
    assert sxy == pytest.approx(0.2) and syaw == pytest.approx(3.0)
