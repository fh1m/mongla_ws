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
    """Checked as a VALUE, not as a literal in the source. The default moved
    into the module so the node, the config loader and the parameter
    declaration cannot disagree about it -- and a test reading the literal
    would fail on that refactor while the behaviour was unchanged."""
    from mongla_vision.anchor import loop_closure as lc
    assert lc.defaults()['enabled'] is False, (
        'loop closure is on by default; it is the only vision path that '
        'reaches the localisation filter, and a wrong fix is BELIEVED')
    assert "_lc.defaults()['enabled']" in src, (
        'the node no longer takes its default from the module, so the two '
        'can drift apart')


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


def test_it_is_configured_WITHOUT_launch_plumbing():
    """⛔ DELIBERATELY NOT A LAUNCH ARGUMENT.

    A switch threaded through launch must be declared in every launch file
    that starts the node, and this package has shipped capabilities reachable
    from one launch path and not the one `bringup` includes. Loop closure was
    briefly wired into vision_pi.launch.py while bringup includes
    vision.launch.py -- unreachable from the documented mission command, the
    fifth instance of the same defect.

    The node configures itself instead, so it behaves the same whichever
    launch starts it. This test fails if the plumbing comes back, because two
    sources of truth for one switch is how they disagree.
    """
    from mongla_vision.anchor import loop_closure as lc
    assert hasattr(lc, 'load_config') and hasattr(lc, 'defaults')
    for launch in ('vision_pi.launch.py', 'vision.launch.py'):
        text = (LAUNCH.parent / launch).read_text()
        assert 'loop_closure' not in text, (
            f'{launch} configures loop closure again. Put it in '
            f'{lc.CONFIG_PATH} instead -- a launch argument is only read by '
            f'the launch files that declare it.')


def test_a_broken_config_file_does_not_take_vision_down(tmp_path):
    """On a pool deck a typo must not stop the stack, and must not be silent
    either -- the operator would watch for a change that never loaded."""
    from mongla_vision.anchor import loop_closure as lc
    bad = tmp_path / 'loop_closure.yaml'
    bad.write_text('enabled: [this is not a bool\n')
    cfg = lc.load_config(str(bad))
    assert cfg == lc.defaults(), 'a broken file must fall back to defaults'


def test_an_unset_config_file_is_the_defaults(tmp_path):
    from mongla_vision.anchor import loop_closure as lc
    assert lc.load_config(str(tmp_path / 'nope.yaml')) == lc.defaults()


def test_the_deck_file_actually_changes_the_bars(tmp_path):
    from mongla_vision.anchor import loop_closure as lc
    f = tmp_path / 'loop_closure.yaml'
    f.write_text('enabled: true\nmin_inliers: 140\nplace_travel_m: 2.5\n')
    cfg = lc.load_config(str(f))
    assert cfg['enabled'] is True
    assert cfg['min_inliers'] == 140
    assert cfg['place_travel_m'] == 2.5
    assert cfg['min_age_s'] == lc.MIN_REF_AGE_S, 'unset keys must keep defaults'


def test_the_configured_bars_reach_consider(src):
    """A config that is loaded and then not passed is the same as no config."""
    seg = src[src.index('def _place_tick'):src.index('def _m_per_px')]
    for key in ('min_inliers', 'min_age_s', 'min_travel_m', 'max_offset_m'):
        assert f"self._closure_cfg['{key}']" in seg, (
            f'{key} is configurable but never handed to consider(), so '
            f'editing it changes nothing')


# --------------------------------------------------------------------------- #
#  The altitude that turns pixels into metres
# --------------------------------------------------------------------------- #
def test_the_measured_floor_beats_the_typed_constant(src):
    """`flow_node` publishes floor_height from the tile grating and REFUSES to
    publish `pool_depth_m - |depth|` under that name, because a typed constant
    minus a depth is not a measurement of the floor. The closure must agree,
    or it scales every offset by a number nobody measured."""
    assert 'floor_height' in src and '_on_floor_height' in src, (
        'lock_node no longer reads the measured floor height, so the only '
        'altitude left is pool_depth_m minus depth -- a typed constant')
    seg = src[src.index('def _altitude_m'):src.index('def _place_tick')]
    assert seg.index('_floor_h') < seg.index('_pool_depth'), (
        'the typed constant is consulted before the measurement')


def test_a_stale_floor_height_is_not_used(src):
    """A height from ten seconds ago is a different altitude wearing the right
    units, and it would scale the offset silently."""
    seg = src[src.index('def _altitude_m'):src.index('def _place_tick')]
    assert '_FLOOR_H_MAX_AGE_S' in seg, 'floor height is used regardless of age'


def test_the_typed_constant_fallback_says_what_it_is(src):
    seg = src[src.index('def _altitude_m'):src.index('def _place_tick')]
    assert 'TYPED' in seg and 'warn' in seg, (
        'the pool_depth fallback is silent again, so a run scaled by a typed '
        'constant is indistinguishable from one scaled by a measurement')
