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
