"""The camera profile table exists TWICE, and only one copy is loaded.

`duburi_vision/config.py` holds the dict `get_profile` resolves against.
`config/cameras.yaml` is the operator-facing copy with the rationale, and
nothing reads it at runtime. The docstring used to ask, politely, that anyone
editing one edit the other.

That is not a mechanism, and it failed the first time it mattered: switching
the Pi cameras to the low-latency source by editing the YAML changed NOTHING.
The launch came up on the old source, published frames, ran detections, and
logged no error -- the only symptom was a latency measurement that did not
move. Same family as the launch default that silently beat every profile's fps
in round 26.

These tests are the mechanism.
"""
import sys
from pathlib import Path

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from duburi_vision.config import CAMERA_PROFILES        # noqa: E402

YAML_PATH = (Path(__file__).resolve().parents[1] / 'config' / 'cameras.yaml')


def _yaml_profiles():
    doc = yaml.safe_load(YAML_PATH.read_text())
    # The file nests the table under a node/ros__parameters block; find the
    # mapping whose values look like profiles rather than assuming a path,
    # so a formatting change does not silently make this test vacuous.
    def walk(node):
        if isinstance(node, dict):
            if any(isinstance(v, dict) and 'source' in v for v in node.values()):
                return {k: v for k, v in node.items()
                        if isinstance(v, dict) and 'source' in v}
            for v in node.values():
                got = walk(v)
                if got:
                    return got
        return {}
    found = walk(doc)
    assert found, f'no profiles found in {YAML_PATH} -- this test would be vacuous'
    return found


def test_the_two_copies_name_the_same_profiles():
    assert set(_yaml_profiles()) == set(CAMERA_PROFILES)


@pytest.mark.parametrize('name', sorted(CAMERA_PROFILES))
def test_the_source_agrees(name):
    """The field that actually decides which class is built. A mismatch here
    means the YAML describes a pipeline the code does not run."""
    assert _yaml_profiles()[name]['source'] == CAMERA_PROFILES[name]['source']


@pytest.mark.parametrize('key', ('width', 'height', 'fps', 'device_path',
                                 'device', 'frame_id', 'topic', 'path'))
def test_every_shared_key_agrees(key):
    """Any key present in BOTH copies must match. Keys present in only one are
    left alone -- the YAML carries operator comments, the dict carries test
    fixtures -- so this catches drift without forbidding either from having
    something the other does not."""
    y = _yaml_profiles()
    bad = []
    for name, prof in CAMERA_PROFILES.items():
        if key in prof and key in y.get(name, {}):
            if str(prof[key]) != str(y[name][key]):
                bad.append(f'{name}.{key}: config.py={prof[key]!r} '
                           f'cameras.yaml={y[name][key]!r}')
    assert not bad, 'profiles disagree:\n  ' + '\n  '.join(bad)


def test_the_pi_cameras_use_the_low_latency_source():
    """Pinned deliberately, because the whole point of the v4l2 source is a
    number nobody looks at day to day. Measured after a 400 ms consumer stall:
    396 ms of staleness on `webcam`, 17 ms on `v4l2`. A revert to `webcam`
    would publish frames, run detections, log nothing, and quietly put a
    third of a second of lag back into the control loop."""
    for name in ('pi_forward', 'pi_downward'):
        assert CAMERA_PROFILES[name]['source'] == 'v4l2', name


def test_every_source_has_a_builder():
    """A typo in `source` is otherwise a runtime error on the vehicle."""
    from duburi_vision.factory import BUILDERS
    for name, prof in CAMERA_PROFILES.items():
        assert prof['source'] in BUILDERS, f'{name}: {prof["source"]!r}'


# --------------------------------------------------------------------------- #
#  A profile key that reaches nothing is worse than an absent one
# --------------------------------------------------------------------------- #
def test_device_path_in_a_profile_reaches_the_builder():
    """`device_path` was passed in by every profile that names one and landed
    in the builder's `**_`, ignored. All four such profiles therefore resolved
    to device index 0, and the operator's explicit launch arg was the only
    thing that had ever made two cameras work at once -- the second died EBUSY
    with a message recommending the very key the profile already set.

    Asserted through `make_camera` rather than on the signature, so a future
    builder that accepts the kwarg and then drops it still fails."""
    import duburi_vision.factory as F

    seen = {}

    class _Spy:
        def __init__(self, **kw):
            seen.update(kw)

        def info(self):
            return {}

    orig = F._build_v4l2.__globals__.get('V4L2MailboxCamera')
    import duburi_vision.cameras.v4l2_mailbox as vm
    vm.V4L2MailboxCamera = _Spy
    try:
        F.make_camera_from_profile(
            {'source': 'v4l2', 'device_path': '/dev/duburi_cam_forward',
             'width': 640, 'height': 360, 'fps': 60, 'name': 'f'})
    finally:
        if orig is not None:
            vm.V4L2MailboxCamera = orig
    assert seen.get('device') == '/dev/duburi_cam_forward', seen


@pytest.mark.parametrize('name', ('pi_forward', 'pi_downward'))
def test_the_pi_profiles_name_a_symlink_this_OS_creates(name):
    """Raspberry Pi OS creates `/dev/v4l/by-id/` for USB video and NEVER
    `by-path`, so the by-path values these profiles used to carry -- which
    udevadm reports correctly -- pointed at nothing. `tools/udev/` supplies
    `/dev/duburi_cam_*` instead, and the rule earned itself immediately: video0
    was the Sonix before a reboot and the Fantech after, so the raw index had
    already swapped the two cameras."""
    dev = CAMERA_PROFILES[name]['device_path']
    assert dev.startswith('/dev/duburi_cam_'), dev
    assert 'by-path' not in dev
