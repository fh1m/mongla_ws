"""The closure path must be REACHABLE, and refuse loudly when it is not.

The defect this guards was found by reading: `lock_node` called
`bank.enrol()` with no `position=` and subscribed to no odometry, so every
reference carried `ref_position=None` and no closure could ever have fired --
while the bank, the `near=` search and `BankPose.ref_position` all existed and
looked healthy. A capability with no path to it is indistinguishable from one
that is merely never triggered, so these assert the PATH, not the policy.

Source-level, because standing the node up needs ROS, a camera and a filter.
That is a real limitation: these prove the wiring exists, not that it works.
What it does once reached is covered by `test_loop_closure_refuses.py` and
`mongla_localization/test/test_loop_closure_bounds_drift.py`.
"""
import pathlib
import re

import pytest

SRC = (pathlib.Path(__file__).resolve().parents[1] / 'mongla_vision'
       / 'lock_node.py')
LAUNCH = (pathlib.Path(__file__).resolve().parents[1] / 'launch'
          / 'vision_pi.launch.py')


@pytest.fixture(scope='module')
def src():
    return SRC.read_text()


def test_enrolment_passes_a_position(src):
    """⛔ THE ORIGINAL DEFECT. Without this every reference is unlocalised and
    the closure can never fire."""
    assert re.search(r'enrol\([^)]*position=', src, re.S), (
        'place enrolment no longer passes position= -- every reference will '
        'carry ref_position=None and no closure can ever fire')


def test_the_node_subscribes_to_odometry(src):
    assert '/mongla/odom' in src and '_on_odom' in src, (
        'nothing supplies the vehicle position that enrolment stores')


def test_the_fix_is_published_where_the_filter_listens(src):
    """`localization_node._on_fix` reads PointStamped on this topic, with
    `point.z` carrying the producer's sigma."""
    assert "'/mongla/localization/fix'" in src
    assert 'PointStamped' in src
    assert re.search(r'm\.point\.z\s*=\s*float\(c\.sigma\)', src), (
        'the fix no longer carries its own sigma; localization_node would '
        'substitute a node-wide default and the composed uncertainty -- the '
        'entire point of the closure layer -- would be discarded')


def test_the_fix_rides_the_matched_frames_header(src):
    """Stamping on arrival defeats the filter's retrodiction, which is the one
    thing that makes a late measurement safe."""
    assert re.search(r'm\.header\s*=\s*header', src), (
        'the fix is no longer stamped with the frame it was measured from')
    # Check the CODE, not the prose: the comment above the assignment says
    # "not now()", and matching that was the first version of this test
    # failing on the very explanation of the rule it enforces.
    seg = src[src.index('def _place_tick'):src.index('def _m_per_px')]
    code = '\n'.join(ln.split('#', 1)[0] for ln in seg.splitlines())
    assert 'get_clock' not in code, (
        'the fix is stamped on the host wall clock, which defeats the '
        "filter's retrodiction -- the one thing that makes a late "
        'measurement safe')


def test_recognition_happens_before_enrolment(src):
    """Enrolling first would add THIS frame to the bank and then find it --
    the self-closure the whole path exists to avoid."""
    seg = src[src.index('def _place_tick'):src.index('def _m_per_px')]
    assert seg.index('locate(') < seg.index('enrol('), (
        'the place tick enrols before it recognises, so the bank can match '
        'the frame that was just added to it')


def test_places_are_whole_frame(src):
    """The ~100-inlier bar was derived on whole-frame downward references.
    Cropping would repeat section 25: a bar measured on one configuration,
    shipped against another, silently never firing."""
    seg = src[src.index('def _place_tick'):src.index('def _m_per_px')]
    assert re.search(r'enrol\(\s*gray,\s*roi=None', seg), (
        'places are being enrolled from a crop')


def test_a_forward_camera_is_refused_and_says_so(src):
    seg = src[src.index('self._closer_on = bool'):src.index('self._pub =')]
    assert "!= 'downward'" in seg and 'REFUSED' in seg, (
        'loop closure no longer refuses the forward camera -- a forward bank '
        'holds targets, and a target that moves is not a place')


def test_it_is_off_by_default(src):
    assert re.search(r"declare_parameter\('loop_closure',\s*False\)", src), (
        'loop closure is on by default; it is the only vision path that '
        'reaches the localisation filter')


def test_altitude_has_no_constant_fallback(src):
    """Pixels become metres only with a height. A constant here would put a
    plausible number where a measurement is missing."""
    seg = src[src.index('def _altitude_m'):src.index('def _place_tick')]
    assert seg.count("float('nan')") >= 3, (
        'the altitude path grew a fallback value instead of refusing')


def test_the_scale_uses_the_refraction_corrected_intrinsics(src):
    """The air focal length underwater is a clean 1/n error: every offset 33 %
    short, with a plausible number and no warning."""
    seg = src[src.index('def _m_per_px'):src.index('def _on_info')]
    assert '_K_rect' in seg, 'the offset scale is using the AIR intrinsics'


def test_refusals_are_counted_not_dropped(src):
    seg = src[src.index('def _place_tick'):src.index('def _m_per_px')]
    assert '_closure_refusals' in seg and 'no closure in' in seg, (
        'refusals are silent again, so a path that never fires is '
        'indistinguishable from one that is switched off')


def test_the_launch_exposes_it_and_passes_it_down():
    s = LAUNCH.read_text()
    assert re.search(r"DeclareLaunchArgument\(\s*\n?\s*'loop_closure'", s)
    for key in ('loop_closure', 'place_period_s', 'place_travel_m',
                'pool_depth_m'):
        assert f"'{key}':" in s, (
            f'{key} is declared but never reaches the node -- accepted and '
            f'silently ignored, which is the worst of both')
