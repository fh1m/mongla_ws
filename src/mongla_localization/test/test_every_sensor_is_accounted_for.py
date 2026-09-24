"""Every signal the vehicle produces is either USED for localisation, or has a
stated reason why not.

⛔ THE DEFECT CLASS. This stack has repeatedly built a capability and left it
unreachable: the lock ladder in no launch file, `lock_s` held at 0, the
checkpoint bank's `position=` never passed, `device_path` read and ignored. A
sensor that nobody consumes fails the same way and is even quieter, because
there is no error -- the filter simply never gets better and nothing says why.

So the ledger below is the claim "all the sensors are helping", written down
where it can fail. Adding a signal without deciding its role breaks this test,
which is the point: the decision is forced at the time the signal appears,
not remembered later.

⚠ Text-level. It proves a topic is SUBSCRIBED and reasoned about, not that the
measurement is any good.
"""
import pathlib
import re

import pytest

_NODE = (pathlib.Path(__file__).resolve().parents[1] / 'mongla_localization'
         / 'localization_node.py')

# ---- used, and what for ---------------------------------------------------
USED = {
    '/mongla/imu':                      'prediction (board IMU, 50 Hz)',
    '/mongla/state':                    'depth, yaw and attitude from the board',
    'velocity':                         'downward optical flow as a DVL',
    '/mongla/localization/fix':         'position: prop resection AND loop closure',
    '/mongla/localization/heading':     'the latched earth-referenced heading',
    '/mongla/demand':                   'the motion model aid',
    'floor_grid_deg':                   'yaw drift bound from the floor grid',
    'lane_heading_deg':                 'yaw drift bound from a lane line',
}

# ---- deliberately NOT a localisation input, with the reason ---------------
# ⚠ Each of these is a DECISION, not an oversight. Re-argue it rather than
# deleting the row.
EXCLUDED = {
    '/mongla/esc_rpm':
        'commanded-side only, and measured 958/958 frames at exactly zero with '
        'nothing attached -- a thruster that is not turning reads identical to '
        'one that is not reporting, so it cannot support an aid',
    '/mongla/imu_rates':
        'derived from the same IMU already used for prediction; feeding a '
        'derived copy of an input back as a measurement double-counts it',
    'distance_traveled':
        'an integral of the flow velocity already consumed; the integral '
        'carries no information the rate did not',
    'vis_range':
        'a per-target range from the detector, not a vehicle observation',
    'low_quality':
        'a quality FLAG. It belongs in the sigma of the measurement it '
        'describes, not as a measurement of its own',
    'target_pose':
        'where a TARGET is. Only useful as a vehicle fix through resection, '
        'which is what /mongla/localization/fix already carries',
    'floor_height':
        'consumed by lock_node to scale a loop closure, and it is an '
        'ALTITUDE -- the filter observes depth from the board, and the two '
        'are different frames (bottom-referenced vs surface-referenced). '
        'Fusing one as the other is the frame confusion flagged in the '
        'prototype we ported this idea from',
}


@pytest.fixture(scope='module')
def src():
    return _NODE.read_text()


@pytest.mark.parametrize('topic,role', sorted(USED.items()))
def test_a_used_sensor_is_actually_subscribed(src, topic, role):
    assert topic in src, (
        f'{topic} is listed as used for {role}, but the node does not '
        f'subscribe to it -- the ledger has drifted from the code')


def test_the_filter_gets_every_kind_of_measurement_it_supports(src):
    """A filter method with no caller is an unreachable capability."""
    for call in ('update_depth', 'update_yaw', 'update_position',
                 'update_body_velocity_xy', 'update_attitude',
                 'update_zero_velocity'):
        assert call in src, (
            f'{call} has no caller in localization_node, so that measurement '
            f'can never reach the filter')


def test_no_sensor_is_silently_unaccounted_for(src):
    """The ledger must cover every topic the node subscribes to."""
    subs = set(re.findall(r"create_subscription\(\s*[A-Za-z0-9_]+,\s*"
                          r"f?['\"]([^'\"]+)['\"]", src))
    known = set(USED) | set(EXCLUDED)
    for topic in subs:
        bare = topic.replace('{cam}', '').rstrip('/')
        if any(k in topic or k in bare for k in known):
            continue
        pytest.fail(
            f'{topic} is subscribed but appears in neither USED nor '
            f'EXCLUDED. Decide its role and record it here -- an undeclared '
            f'input is how a measurement gets consumed for the wrong reason.')


def test_heading_does_not_depend_on_a_magnetometer(src):
    """⭐ THE DESIGN, stated so it cannot be quietly changed.

    Heading is taken as an EARTH REFERENCE once at the start and held by the
    gyro; the magnetometer is not trusted, because a hull full of thrusters
    is a poor place to measure a magnetic field. What bounds the gyro's drift
    is the FLOOR -- the grid and the lane line -- not a compass.

    Section 37 measured the residual gyro drift at 0.1 deg/hour with the
    board's own bias estimate removed, which is what makes this affordable.
    """
    assert '/mongla/localization/heading' in src, (
        'the latched earth-referenced heading is gone; the filter would fall '
        'back to the board attitude, which is boot-relative or magnetic')
    for floor in ('floor_grid_deg', 'lane_heading_deg'):
        assert floor in src, (
            f'{floor} is no longer consumed, so nothing bounds yaw drift '
            f'except a magnetometer this design does not trust')


def test_the_loop_closure_reaches_the_filter(src):
    """The fix topic is the ONLY channel that bounds horizontal drift."""
    i = src.index('/mongla/localization/fix')
    assert 'update_position' in src[i:], (
        'the fix subscription no longer leads to update_position')
