"""A calibration must name the camera it describes, and the launch must agree.

⛔ WHY THIS FILE EXISTS. The only calibration we hold was named
`pi_forward_1280x720.json` and wired to the FORWARD camera, while its own
metadata read `pi_test_global_shutter (Microdia USB)` -- USB vendor 0c45, the
SONIX unit, which is the DOWNWARD camera. It was captured 2026-09-03, four
days before the udev rules were found to have the two cameras SWAPPED, so it
was named for the camera the system then believed it was looking at.

Both halves were live and neither logged anything:

  - the FORWARD Fantech published CameraInfo with a 63.82 deg HFOV belonging
    to a different lens. That is LATENT rather than live: the only consumer
    is the srot vision uplink, which is default-off, so nothing read the bad
    numbers in a default run -- it is wrong the moment it is switched on;
  - the DOWNWARD camera -- the DVL, whose intrinsics round 38 measured a
    3.08 % axis asymmetry to correct -- got NO calibration at all, so
    flow_node fell back to a single focal length and the frame centre. That
    fix was verified in a tool which passed the path by hand, and never
    reached the launch.

This is the FOURTH config in this package measured to reach nothing
(`device_path` into `**_`, the unloaded YAML profile table, `ros2 param set`
on construction-time params), and the round-32 plan predicted a fourth.

READS THE FILES, NEVER IMPORTS THEM: a test that imports in a worktree
resolves against the main workspace's stale `install/` tree, which is how the
CLAHE retraction nearly went the wrong way.
"""
import json
import pathlib
import re

import pytest

_PKG = pathlib.Path(__file__).resolve().parents[1]
_CAL_DIR = _PKG / 'config' / 'calibration'
_LAUNCH = _PKG / 'launch' / 'vision_pi.launch.py'


def _calib_files():
    return sorted(_CAL_DIR.glob('*.json'))


def test_every_calibration_declares_which_camera_it_describes():
    files = _calib_files()
    assert files, 'no calibration files found -- did the path move?'
    for f in files:
        d = json.loads(f.read_text())
        assert d.get('applies_to'), (
            f'{f.name} does not declare `applies_to`. Without it nothing can '
            f'tell which camera it belongs to, which is exactly how it came '
            f'to be wired to the wrong one.')
        assert isinstance(d['applies_to'], list) and d['applies_to']


def test_the_launch_wires_each_calibration_to_a_camera_it_CLAIMS():
    """The guard proper. Parse the launch's own defaults rather than trusting
    the filename -- the filename is the thing that was wrong."""
    src = _LAUNCH.read_text()
    # DeclareLaunchArgument('<side>_calibration', default_value=_calib('X'))
    pat = re.compile(
        r"DeclareLaunchArgument\(\s*\n?\s*'(fwd|dwn)_calibration',\s*\n?\s*"
        r"default_value=(?:_calib\('([^']+)'\)|'')", re.M)
    found = dict((m.group(1), m.group(2)) for m in pat.finditer(src))
    assert set(found) == {'fwd', 'dwn'}, (
        f'could not parse both calibration arguments from '
        f'{_LAUNCH.name}; found {found}')
    side_to_profile = {'fwd': 'pi_forward', 'dwn': 'pi_downward'}
    for side, fname in found.items():
        if not fname:
            continue                      # '' = deliberately uncalibrated
        f = _CAL_DIR / fname
        if not f.exists():
            # ⛔ A NAMED-BUT-ABSENT FILE IS THE PENDING STATE, NOT A BUG.
            # `_calib()` returns '' when the file is missing, so wiring a
            # camera by NAME before it is calibrated is inert today and goes
            # live the moment the file lands. That is deliberate: it removes
            # the step -- "remember to edit the launch default afterwards" --
            # between calibrating a camera and the calibration reaching it,
            # which is the same class of gap that let the wrong file stay
            # wired to the wrong camera for four days.
            #
            # The filename must still be well formed, or the wiring is a
            # typo that will never activate and never complain.
            assert re.fullmatch(r'pi_(forward|downward)_\d+x\d+\.json',
                                fname), (
                f'{side}_calibration names {fname!r}, which does not match '
                f'the pi_<side>_<W>x<H>.json convention -- a wired name that '
                f'no solve will ever produce is silently inert forever')
            assert side_to_profile[side] in fname, (
                f'{side}_calibration is wired to {fname!r}, which names the '
                f'OTHER camera')
            continue
        applies = json.loads(f.read_text()).get('applies_to') or []
        want = side_to_profile[side]
        assert want in applies, (
            f'{_LAUNCH.name} wires {fname} to the {side.upper()} camera, but '
            f'that file declares applies_to={applies} and does NOT list '
            f'{want!r}. This is the exact defect the file exists to catch.')


def test_the_downward_camera_IS_calibrated():
    """The DVL's intrinsics are not optional. Round 38 measured a 3.08 % axis
    asymmetry caused by assuming fx==fy, the frame centre as the principal
    point, and no undistortion -- all three come from this file."""
    src = _LAUNCH.read_text()
    m = re.search(r"'dwn_calibration',\s*\n?\s*default_value=_calib\('([^']+)'\)",
                  src)
    assert m, ('the downward camera has no calibration wired. flow_node then '
               'falls back to one focal length and the frame centre, which '
               'is measured to cost 3.08 % of axis asymmetry.')
    assert (_CAL_DIR / m.group(1)).exists()


def test_the_flow_node_is_actually_LAUNCHABLE():
    """flow_node -- the DVL -- existed only as a setup.py entry point and
    appeared in NO launch file, so the bottom-camera velocity sensor had to
    be started by hand. On a pool deck that means it does not get started.

    It must also receive the SAME calibration the downward camera gets;
    passing them from two places is how they came to disagree in the first
    place (see the module docstring)."""
    src = _LAUNCH.read_text()
    assert "executable='flow_node'" in src, (
        'flow_node is in no launch file -- the DVL cannot be brought up with '
        'the rest of the vision stack.')
    assert "'pool_depth_m'" in src, (
        'flow_node is launched without pool_depth_m. It refuses to publish '
        'velocity without it, so the node would come up and stay silent.')
    # The calibration must be the downward one, by reference not by literal.
    flow = src[src.index("executable='flow_node'"):]
    flow = flow[:flow.index('condition=')]
    assert "LaunchConfiguration('dwn_calibration')" in flow, (
        'flow_node must take the SAME dwn_calibration the downward camera '
        'takes, not its own copy of the path.')


def test_every_tool_naming_a_calibration_names_one_that_EXISTS():
    """The launch is guarded above. Seven tools hardcode the path too.

    `flow_console.py` is the instrument that produced the 30 cm result, and
    the srot_* tools each carry their own copy of the filename -- some as
    absolute paths. This round's rename was propagated to them by `sed`;
    nothing stops the next one from breaking them silently, and a tool that
    cannot find its calibration falls back to a single focal length and the
    frame centre, which is the exact 3.08 %-asymmetry state round 38 fixed.

    Matches the basename only: the tools disagree about the prefix (repo
    relative, ~, and /home/... all appear) and that is not what this guards.
    """
    tools = _PKG.parents[1] / 'tools'
    if not tools.is_dir():
        pytest.skip('tools/ not present')
    missing = []
    seen = 0
    for py in sorted(tools.glob('*.py')):
        for name in re.findall(r"calibration/([A-Za-z0-9_.-]+\.json)",
                               py.read_text()):
            seen += 1
            if not (_CAL_DIR / name).is_file():
                missing.append(f'{py.name} -> {name}')
    assert seen, 'no tool names a calibration -- has the path shape changed?'
    assert not missing, (
        'tools name calibration files that do not exist:\n  ' +
        '\n  '.join(missing) +
        f'\navailable: {sorted(p.name for p in _CAL_DIR.glob("*.json"))}')


# --------------------------------------------------------------------------
# The binding must hold for EVERY launch, not just vision_pi.
#
# Everything above guards `vision_pi.launch.py`, which wires calibrations by
# hand. `vision.launch.py` -- the one `bringup.launch.py` includes, i.e. the
# one an operator is actually told to run -- wired NOTHING, so `camera_node`
# published CameraInfo with k all zero on the mission path and no test here
# could see it, because this file names one launch file.
#
# The fix was NOT to copy the wiring into the second launch (that makes three
# copies of the camera-to-file map and leaves the fourth to be found later).
# `camera_node` resolves its own calibration from its `profile` via each
# file's own `applies_to`. These guard that derivation.
# --------------------------------------------------------------------------

def _load_binding():
    """Import binding.py BY PATH.

    Not `from mongla_vision...` -- see the module docstring: an import in a
    worktree resolves against the main workspace's stale install/ tree, which
    is how the CLAHE retraction nearly went the wrong way. By path, the thing
    under test is the source in this tree and nothing else.
    """
    import importlib.util
    src = _PKG / 'mongla_vision' / 'calibration' / 'binding.py'
    assert src.is_file(), f'{src} is missing -- did the resolver move?'
    spec = importlib.util.spec_from_file_location('_binding_under_test', src)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_every_calibration_resolves_back_from_the_profile_it_CLAIMS():
    """Round-trip. A file that declares `applies_to: [X]` must be what the
    resolver returns for X -- otherwise the declaration is decorative and the
    camera silently gets no calibration, which is the state this whole file
    exists to prevent."""
    resolve = _load_binding().calibration_for_profile
    for f in _calib_files():
        for profile in json.loads(f.read_text())['applies_to']:
            got = resolve(profile)
            assert got, (
                f'{f.name} declares applies_to {profile!r} but the resolver '
                f'returns nothing for it, so that camera comes up with k=0.')
            assert pathlib.Path(got).name == f.name, (
                f'profile {profile!r} resolves to {pathlib.Path(got).name}, '
                f'not {f.name} -- two files claim one camera.')


def test_a_profile_nobody_calibrated_resolves_to_EMPTY_not_a_guess():
    """`forward` and `pi_forward` are DIFFERENT PHYSICAL CAMERAS (Blue
    Robotics on the Jetson vs the Fantech on the Pi). Name similarity must
    never bind one to the other's intrinsics: a wrong calibration is worse
    than none, because it yields confident bearings wrong by a fixed factor
    with nothing logging a fault."""
    resolve = _load_binding().calibration_for_profile
    for profile in ('forward', 'downward', 'laptop', 'sim_front', ''):
        assert resolve(profile) == '', (
            f'{profile!r} resolved to a calibration. No file declares it, so '
            f'this can only be a name-similarity guess -- the round-38 defect '
            f'in a new costume.')


def test_applies_to_names_a_camera_profile_THAT_EXISTS():
    """An `applies_to` naming no real profile binds to nothing, forever, and
    reads as correct. Guards a typo the resolver cannot detect."""
    cams = _PKG / 'config' / 'cameras.yaml'
    if not cams.is_file():
        pytest.skip('cameras.yaml not present')
    text = cams.read_text()
    declared = set(re.findall(r'^\s{4}([a-z_][a-z0-9_]*):\s*$', text, re.M))
    declared |= set(re.findall(r'^\s{4}([a-z_][a-z0-9_]*):\s*\{', text, re.M))
    assert declared, 'parsed no camera profiles -- has cameras.yaml changed?'
    for f in _calib_files():
        for profile in json.loads(f.read_text())['applies_to']:
            assert profile in declared, (
                f'{f.name} declares applies_to {profile!r}, which is not a '
                f'camera profile in cameras.yaml. It will bind to nothing '
                f'forever. Known: {sorted(declared)}')


def test_camera_node_ACTUALLY_calls_the_resolver_when_unset():
    """The resolver existing is not the same as it being reached -- that gap
    is precisely how the four prior 'config reaches nothing' defects in this
    package happened. Read camera_node's source and prove the wiring."""
    src = (_PKG / 'mongla_vision' / 'camera_node.py').read_text()
    assert 'calibration_for_profile' in src, (
        'camera_node does not call calibration_for_profile, so an unset '
        '`calibration` param still publishes k=0 on the bringup path.')
    body = src[src.index('def _load_calibration'):]
    body = body[:body.index('def _fill_calibration')]
    assert 'calibration_for_profile' in body, (
        'calibration_for_profile is imported but not used inside '
        '_load_calibration -- the resolution never happens.')
    assert "get_parameter('profile')" in body, (
        '_load_calibration must resolve from the PROFILE; anything else is a '
        'second copy of the camera-to-file map.')


def test_no_profile_is_claimed_by_TWO_calibrations():
    """`applies_to` is a list and the resolver returns the first match in
    sorted order. Two files claiming one camera would leave the round-trip
    test above passing for whichever sorts first, while the other is silently
    shadowed -- the resolver would be picking a lens alphabetically."""
    owner = {}
    for f in _calib_files():
        for profile in json.loads(f.read_text())['applies_to']:
            assert profile not in owner, (
                f'both {owner[profile]} and {f.name} declare applies_to '
                f'{profile!r}. The resolver takes the first in sorted order, '
                f'so which lens that camera gets is decided by filename.')
            owner[profile] = f.name


def test_setup_py_INSTALLS_the_calibrations_into_the_share_dir():
    """The branch the vehicle actually uses.

    `binding._candidate_dirs()` tries the installed share dir first and the
    source tree second. On the Pi, resolution was measured going through the
    share dir (install/mongla_vision/share/...). The source-tree fallback
    exists for tests and for a `--symlink-install` tree -- it does NOT exist
    on a clean deploy. So if setup.py ever stops installing these files, the
    share dir is empty, the fallback is absent, and every camera comes up
    with k=0 again with only a WARN.
    """
    setup = _PKG / 'setup.py'
    src = setup.read_text()
    assert 'config/calibration' in src, (
        'setup.py does not install config/calibration -- on a clean deploy '
        'the share dir is empty and no calibration resolves.')
    assert re.search(r"glob\(\s*'config/calibration/\*\.json'\s*\)", src), (
        'setup.py names config/calibration but does not glob its *.json. '
        'The calibrations would not reach the installed share dir.')
