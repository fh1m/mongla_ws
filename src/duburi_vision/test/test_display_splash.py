"""HUD splash latch (display_node): the startup splash must not re-appear on a
camera switch.

_last_det_t is reset to 0 on a camera switch (detector health goes stale until
the new detector publishes), but the splash is driven by the one-way
_ever_detected latch instead -- so switching cameras keeps showing the live
camera frames, not the "INITIALIZING VISION SYSTEM" splash. No ROS spin: the
real unbound methods run against a MagicMock self.
"""

from unittest.mock import MagicMock, patch

from duburi_vision.utils.display_node import VisionDisplayNode


def test_on_detections_sets_ever_detected_latch():
    fake = MagicMock()
    fake._ever_detected = False
    with patch('duburi_vision.utils.display_node.array_to_detections',
               return_value=[]):
        VisionDisplayNode._on_detections(fake, MagicMock())
    assert fake._ever_detected is True


def test_switch_camera_resets_last_det_t_but_NOT_ever_detected():
    fake = MagicMock()
    fake._active_camera = 'forward'
    fake._ever_detected = True          # we've seen detections on forward
    fake._last_det_t = 123.0
    # MagicMock supports the context-manager + .empty() calls in _switch_camera.
    VisionDisplayNode._switch_camera(fake, 'downward')
    assert fake._last_det_t == 0.0       # health goes stale (correct)
    assert fake._ever_detected is True   # splash latch must survive the switch
    assert fake._active_camera == 'downward'


def test_switch_to_same_camera_is_noop():
    fake = MagicMock()
    fake._active_camera = 'forward'
    fake._ever_detected = True
    VisionDisplayNode._switch_camera(fake, 'forward')
    fake.destroy_subscription.assert_not_called()
    assert fake._ever_detected is True
