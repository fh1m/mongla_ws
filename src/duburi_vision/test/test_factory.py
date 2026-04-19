"""Factory unit tests -- no GPU, no cv2 capture, no rclpy."""

import pytest

from duburi_vision.factory import (
    BUILDERS, make_camera, make_camera_from_profile,
)
from duburi_vision.config import CAMERA_PROFILES, get_profile


def test_builders_known_keys():
    assert set(BUILDERS) == {'webcam', 'ros_topic', 'jetson', 'blueos', 'mavlink'}


def test_unknown_source_raises_with_helpful_message():
    with pytest.raises(ValueError) as exc:
        make_camera('does_not_exist')
    msg = str(exc.value)
    assert 'unknown camera source' in msg
    assert 'webcam' in msg            # known list cited
    assert 'ros_topic' in msg


def test_jetson_stub_raises_not_implemented():
    with pytest.raises(NotImplementedError) as exc:
        make_camera('jetson')
    assert 'jetson camera not implemented' in str(exc.value).lower()


def test_blueos_stub_raises_not_implemented():
    with pytest.raises(NotImplementedError):
        make_camera('blueos')


def test_mavlink_stub_raises_not_implemented():
    with pytest.raises(NotImplementedError):
        make_camera('mavlink')


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


def test_make_camera_from_profile_jetson_stub_still_raises_not_impl():
    with pytest.raises(NotImplementedError):
        make_camera_from_profile(get_profile('jetson_front'))


def test_make_camera_from_profile_requires_source():
    with pytest.raises(ValueError):
        make_camera_from_profile({'width': 640})
