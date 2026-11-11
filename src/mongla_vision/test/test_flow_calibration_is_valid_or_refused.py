"""#80: a calibration flow cannot use refuses velocity; it never degrades silently.

The node used to fall back to one focal length and the frame centre whenever
the calibration was missing, unreadable, for another camera or for another
aspect ratio, and then published with the CALIBRATED sigma. That state is the
one measured at a 3.08 % axis asymmetry (measured-bars), and nothing
downstream could tell. Now each of those is named, velocity is refused unless
`allow_uncalibrated` is set, and when it is, the sigma widens and a latched
`flow_calibration` topic says why.

Built through the real node (`_make`), not through the helpers.
"""
import json
import math
import types
from pathlib import Path

import pytest

from test_flow_node import _cal_path, _make, _published_sigma  # noqa: F401
from test_flow_node import _ros  # noqa: F401  (the rclpy fixture)


def _shipped():
    return json.loads(Path(_cal_path()).read_text())


def _write(tmp_path, name, **changes):
    d = _shipped()
    d.update(changes)
    p = tmp_path / name
    p.write_text(json.dumps(d))
    return str(p)


def _refused(n, frame=(1280, 720)):
    n._admit_frame_size(frame)
    ok, why = n._scale_ready()
    return (not ok), why


def test_the_shipped_calibration_is_accepted_at_its_own_size_and_half_of_it():
    for wh in ((1280, 720), (640, 360)):
        n = _make(calibration=_cal_path(), medium='air', pool_depth_m=4.0)
        try:
            refused, why = _refused(n, wh)
            assert not refused, why
            assert n._cal_problem == ''
        finally:
            n.destroy_node()


@pytest.mark.parametrize('case, needle', [
    ('missing', 'none was passed'),
    ('broken', 'unreadable'),
    ('other_camera', 'not the'),
    ('no_binding', 'names no camera'),
    ('wet', 'calibrated in'),
    ('crop', 'aspect ratio'),
])
def test_every_unusable_calibration_refuses_and_says_why(tmp_path, case, needle):
    frame = (1280, 720)
    if case == 'missing':
        cal = ''
    elif case == 'broken':
        cal = tmp_path / 'broken.json'
        cal.write_text('{ not json')
        cal = str(cal)
    elif case == 'other_camera':
        cal = _write(tmp_path, 'fwd.json', applies_to=['pi_forward'])
    elif case == 'no_binding':
        cal = _write(tmp_path, 'anon.json', applies_to=[])
    elif case == 'wet':
        cal = _write(tmp_path, 'wet.json', medium='water')
    else:
        cal = _cal_path()
        frame = (640, 480)
    n = _make(calibration=cal, medium='air', pool_depth_m=4.0)
    try:
        refused, why = _refused(n, frame)
        assert refused, f'{case}: velocity would publish on a fallback scale'
        assert needle in why
        assert n._intr is None, 'a refused calibration must not be half-loaded'
    finally:
        n.destroy_node()


def test_a_refused_frame_publishes_no_velocity_and_no_distance():
    """The refusal reaches the outputs, not only `_scale_ready`."""
    n = _make(medium='air', pool_depth_m=4.0)
    sent = []
    try:
        n._admit_frame_size((1280, 720))
        n._pub_vel = types.SimpleNamespace(publish=sent.append)
        n._pub_dist = types.SimpleNamespace(publish=sent.append)
        refusals = []
        n._refuse = refusals.append
        n._evaluate(None, None, 0.0, 0.0, 0)
        assert sent == [] and refusals and 'calibration' in refusals[0]
    finally:
        n.destroy_node()


def test_permission_runs_it_with_a_wider_sigma_and_says_so(tmp_path):
    cal = _write(tmp_path, 'fwd.json', applies_to=['pi_forward'])
    good = _make(calibration=_cal_path(), medium='air', pool_depth_m=4.0,
                 estimate_time_offset=False)
    loose = _make(calibration=cal, medium='air', pool_depth_m=4.0,
                  estimate_time_offset=False, allow_uncalibrated=True)
    states = []
    try:
        loose._pub_cal = types.SimpleNamespace(
            publish=lambda m: states.append(m.data))
        for n in (good, loose):
            n._admit_frame_size((1280, 720))
        assert loose._scale_ready()[0]
        assert states and states[-1].startswith('uncalibrated: ')
        s_good = _published_sigma(good, 0.3, 0.0, 0.7, (0.0, 0.0))
        s_loose = _published_sigma(loose, 0.3, 0.0, 0.7, (0.0, 0.0))
        # The calibrated 3.6 % and the uncalibrated 3.08 % in quadrature.
        assert s_loose == pytest.approx(
            math.hypot(s_good, 0.0308 * 0.3), rel=1e-6)
    finally:
        good.destroy_node()
        loose.destroy_node()


def test_with_no_path_the_profile_binds_the_same_file_camera_node_reads():
    """vision.launch.py passes no calibration; camera_node resolves one from
    the profile, and flow now makes the same lookup instead of falling back."""
    from mongla_vision.calibration.binding import calibration_for_profile
    n = _make(profile='pi_downward', medium='air', pool_depth_m=4.0)
    try:
        assert n._cal_path == calibration_for_profile('pi_downward') != ''
        refused, why = _refused(n)
        assert not refused, why
    finally:
        n.destroy_node()


def test_both_launches_hand_flow_its_hardware_profile():
    root = Path(__file__).resolve().parents[1] / 'launch'
    for name in ('vision.launch.py', 'vision_pi.launch.py'):
        src = (root / name).read_text()
        flow = src[src.index("executable='flow_node'"):]
        flow = flow[:flow.index('condition=')]
        assert "'profile':" in flow, f'{name}: flow_node gets no profile'
