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
def test_device_path_in_a_profile_reaches_the_builder(monkeypatch):
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

    # ⛔ THE RESTORE USED TO NEVER RUN, AND IT LEAKED INTO THE WHOLE SESSION.
    # `orig` was read from `F._build_v4l2.__globals__`, but factory imports
    # V4L2MailboxCamera INSIDE the builder, so the name is not in module globals
    # and `.get()` returned None -- making `if orig is not None:` permanently
    # False. The spy therefore replaced the real class for every test that ran
    # afterwards. Nothing noticed until a test finally introspected the real
    # class (the B23 pump-loop guards), which then failed only in a full-suite
    # run and passed in isolation.
    #
    # A `finally:` gated on a value that is always None is not cleanup. Read the
    # attribute from the module being patched -- which the two sibling tests
    # below already do correctly -- and let monkeypatch own the restore.
    import duburi_vision.cameras.v4l2_mailbox as vm
    monkeypatch.setattr(vm, 'V4L2MailboxCamera', _Spy)
    F.make_camera_from_profile(
        {'source': 'v4l2', 'device_path': '/dev/duburi_cam_forward',
         'width': 640, 'height': 360, 'fps': 60, 'name': 'f'})
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


# --------------------------------------------------------------------------- #
#  fourcc: format is PER-CAMERA, and it used to be unconfigurable
# --------------------------------------------------------------------------- #
def test_fourcc_in_a_profile_reaches_the_builder():
    """`fourcc` did not exist as a profile key and both builders hardcoded
    MJPG -- `_build_webcam` did not even accept the kwarg, so it landed in
    `**_`, and `WebcamCamera` set MJPG unconditionally. A camera that is
    faster in another format could therefore not be configured at all.

    That is not hypothetical. Measured on the vehicle: the Sonix global
    shutter does 210.17 Hz in MJPG against a flat 35.26 in YUYV, while the
    Fantech returns exactly 15.00 Hz in both. Format is worth 6x on one camera
    and nothing on the other, which is precisely why it cannot be a constant.

    Asserted through `make_camera_from_profile` rather than on the signature,
    so a builder that accepts the kwarg and then drops it still fails."""
    import duburi_vision.factory as F

    seen = {}

    class _Spy:
        def __init__(self, **kw):
            seen.update(kw)

        def info(self):
            return {}

    import duburi_vision.cameras.v4l2_mailbox as vm
    orig = vm.V4L2MailboxCamera
    vm.V4L2MailboxCamera = _Spy
    try:
        F.make_camera_from_profile(
            {'source': 'v4l2', 'device_path': '/dev/duburi_cam_downward',
             'width': 640, 'height': 360, 'fps': 210, 'fourcc': 'YUYV',
             'name': 'd'})
    finally:
        vm.V4L2MailboxCamera = orig
    assert seen.get('fourcc') == 'YUYV', seen


def test_the_webcam_fallback_carries_the_format_too():
    """The v4l2 source falls back to OpenCV when the mailbox declines a
    device. If that fallback drops `fourcc`, a profile asking for YUYV
    silently gets MJPG -- the failure is invisible because frames still
    arrive, just at the other format's frame rate."""
    import duburi_vision.factory as F

    seen = {}

    class _Spy:
        def __init__(self, **kw):
            seen.update(kw)

        def info(self):
            return {}

    import duburi_vision.cameras.webcam as wc
    orig = wc.WebcamCamera
    wc.WebcamCamera = _Spy
    try:
        # An int index is not a /dev path, so _build_v4l2 takes the fallback.
        F.make_camera_from_profile(
            {'source': 'v4l2', 'device': 0, 'width': 640, 'height': 360,
             'fps': 30, 'fourcc': 'YUYV', 'name': 'f'})
    finally:
        wc.WebcamCamera = orig
    assert seen.get('fourcc') == 'YUYV', seen


@pytest.mark.parametrize('name,expect_fps', (('pi_forward', 60),
                                             ('pi_downward', 210)))
def test_the_pi_profiles_carry_the_MEASURED_rate(name, expect_fps):
    """Pinned to hardware, because both numbers were wrong and wrong in the
    direction that hides: the profiles asked for 210 and 90.

    The two were also SWAPPED. The global-shutter Sonix is the BOTTOM camera
    (210 Hz, and the optical-flow velocity sensor) and the Fantech is the
    FORWARD one. Asking pi_downward for 90 was asking the wrong camera for a
    rate neither of them has.

    ⛔ pi_forward's 15 IS RETRACTED (2026-09-09). This test asserted 15 was
    "the MEASURED rate" and config.py's comment called it the camera's
    ceiling; the 15 was the config line itself, and each defended the other.
    Re-measured on the vehicle by counting DISTINCT header stamps -- a topic
    `hz` cannot separate a real frame from a republished one, which is how a
    self-imposed cap reads as hardware:

        requested 15 -> 14.63 Hz      requested 60 -> 30.18 Hz
        requested 30 -> 28.03 Hz      requested 90 -> 30.18 Hz

    Identity confirmed against the calibration's USB VID/PID and serial
    (1d6c:0103, YGR80PU1200F23081120), so this is the Fantech answering.
    End to end it took detections 14.67 -> 29.12 Hz for ~6 points of one
    core. 60 rather than 30 because the request saturates at the ceiling."""
    assert CAMERA_PROFILES[name]['fps'] == expect_fps
