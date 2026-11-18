"""Factory unit tests -- no GPU, no cv2 capture, no rclpy."""

import pytest

from mongla_vision.factory import (
    BUILDERS, make_camera, make_camera_from_profile,
)
from mongla_vision.config import CAMERA_PROFILES, get_profile


def test_builders_known_keys():
    assert set(BUILDERS) == {
        'webcam', 'v4l2', 'ros_topic', 'video_file'}


def test_unknown_source_raises_with_helpful_message():
    with pytest.raises(ValueError) as exc:
        make_camera('does_not_exist')
    msg = str(exc.value)
    assert 'unknown camera source' in msg
    assert 'webcam' in msg            # known list cited
    assert 'ros_topic' in msg


def test_ros_topic_requires_node():
    with pytest.raises(ValueError) as exc:
        make_camera('ros_topic', topic='/foo')
    assert 'requires node' in str(exc.value)


def test_ros_topic_requires_topic():
    class _FakeNode:  # never used because we expect failure first
        pass
    with pytest.raises(ValueError):
        make_camera('ros_topic', node=_FakeNode())


def test_get_profile_returns_copy():
    p = get_profile('laptop')
    p['width'] = 9999
    again = get_profile('laptop')
    assert again['width'] != 9999, 'get_profile must return a copy'


def test_get_profile_unknown_lists_known():
    with pytest.raises(ValueError) as exc:
        get_profile('not_a_profile')
    msg = str(exc.value)
    assert 'unknown camera profile' in msg
    assert 'laptop' in msg


def test_camera_profiles_have_source_key():
    for name, profile in CAMERA_PROFILES.items():
        assert 'source' in profile, f'profile {name!r} missing source key'


def test_make_camera_from_profile_requires_source():
    with pytest.raises(ValueError):
        make_camera_from_profile({'width': 640})


def test_a_key_no_builder_reads_is_refused_not_swallowed():
    """`**_` once swallowed `device_path` and both cameras opened index 0.
    A misspelt key now names itself instead of vanishing."""
    with pytest.raises(ValueError, match='exposre_us'):
        make_camera('video_file', path='x.mp4', exposre_us=4000)
    with pytest.raises(ValueError, match='controls'):
        make_camera('ros_topic', node=object(), topic='/t', controls={'gain': 1})


def test_every_profile_control_is_one_the_mailbox_can_set():
    from mongla_vision.cameras.v4l2_mailbox import CONTROLS
    for name, prof in CAMERA_PROFILES.items():
        assert set(prof.get('controls', {})) <= set(CONTROLS), name
