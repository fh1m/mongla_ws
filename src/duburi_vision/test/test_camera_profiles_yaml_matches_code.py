"""cameras.yaml and CAMERA_PROFILES are two copies of one truth.

⛔ WHY THIS FILE EXISTS. `config/cameras.yaml` says it "mirrors
CAMERA_PROFILES in config.py". Only the Python dict is ever LOADED -- the
YAML is documentation. Nothing checked that they agree.

Found the hard way on 2026-09-09: raising the forward camera's frame rate was
done in `cameras.yaml`, the package was rebuilt, and the measured rate came
back 22.485 Hz -- byte-identical to the run before the edit. The edit had
changed nothing, and the only reason it was caught is that the number did not
move. Had it been a value with no runtime signal (a `frame_id`, a
`device_path`) the two copies would simply have disagreed, with the YAML --
the one an operator reads -- being the wrong one.

`cameras.yaml` is the FOURTH config in this package measured to reach
nothing, after `device_path` into `**_`, the unloaded YAML profile table, and
`ros2 param set` on construction-time parameters.

This compares the DATA, not the prose: each file's rationale comments differ
deliberately and at different lengths, and that is fine.
"""
import os
import pathlib

import pytest

yaml = pytest.importorskip('yaml')

_PKG = pathlib.Path(__file__).resolve().parents[1]
_YAML = _PKG / 'config' / 'cameras.yaml'

# Compared for every profile present in both. Fields that only make sense in
# one place are ignored rather than forced.
_FIELDS = ('source', 'device', 'device_path', 'width', 'height', 'fps',
           'fourcc', 'frame_id', 'topic')


def _code_profiles():
    """Read CAMERA_PROFILES from the SOURCE tree, by path.

    Never `from duburi_vision import ...`: in a worktree that resolves
    against the main workspace's stale install/ tree, which is how the CLAHE
    retraction nearly went the wrong way.
    """
    import importlib.util
    src = _PKG / 'duburi_vision' / 'config.py'
    spec = importlib.util.spec_from_file_location('_cfg_under_test', src)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.CAMERA_PROFILES


def _yaml_profiles():
    d = yaml.safe_load(_YAML.read_text())
    # the file nests the table under a node/params header
    while isinstance(d, dict) and len(d) == 1:
        only = next(iter(d.values()))
        if not isinstance(only, dict):
            break
        d = only
    for key in ('camera_profiles', 'profiles', 'ros__parameters'):
        if isinstance(d, dict) and key in d and isinstance(d[key], dict):
            d = d[key]
    return d


def test_the_yaml_and_the_code_agree_on_every_shared_profile():
    code, docs = _code_profiles(), _yaml_profiles()
    shared = sorted(set(code) & set(docs))
    assert shared, (
        f'no profile name appears in both files -- the YAML parse shape has '
        f'changed. code={sorted(code)[:6]} yaml={sorted(docs)[:6]}')
    bad = []
    for name in shared:
        c, y = code[name], docs[name] or {}
        if not isinstance(y, dict):
            continue
        for f in _FIELDS:
            if f in c and f in y and c[f] != y[f]:
                bad.append(f'{name}.{f}: code={c[f]!r} yaml={y[f]!r}')
    assert not bad, (
        'cameras.yaml disagrees with the LOADED CAMERA_PROFILES. Only the '
        'Python dict takes effect, so the YAML is what lies:\n  '
        + '\n  '.join(bad))


def test_a_profile_documented_in_the_yaml_EXISTS_in_the_code():
    """The failure mode with no runtime signal: an operator reads a profile
    in the YAML, passes its name, and gets a raise (or the non-profile
    branch). config.py's own comments record this having happened."""
    code, docs = _code_profiles(), _yaml_profiles()
    missing = sorted(set(docs) - set(code))
    assert not missing, (
        f'cameras.yaml documents profiles that CAMERA_PROFILES does not '
        f'define, so `camera:=<name>` fails for them: {missing}')


def test_the_forward_camera_is_not_capped_below_its_measured_ceiling():
    """Guards the specific regression: `fps` was 15 while the unit delivers
    30.18, and the comment beside it asserted 15 was the hardware ceiling.
    Detection rate is what the vision loop steers on."""
    code = _code_profiles()
    fps = code['pi_forward']['fps']
    assert fps >= 30, (
        f'pi_forward fps is {fps}. Measured on the vehicle by distinct header '
        f'stamps: requesting 15 gives 14.63 Hz, requesting 60 gives 30.18 Hz '
        f'and takes detections 14.67 -> 27.11 Hz for ~6 points of one core. '
        f'A value below 30 caps the forward camera under its own ceiling.')
    assert code['pi_forward'].get('fourcc') == 'MJPG', (
        'pi_forward lost fourcc: MJPG. Without it the driver negotiates YUYV '
        'and the measured rate collapses (5.00 Hz through the launch).')
