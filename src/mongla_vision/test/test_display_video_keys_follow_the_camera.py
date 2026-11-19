"""After a camera switch the display's video keys drive the camera on screen.

The playback publishers and the pause client were built once, for the camera
the display started on, so after `f`/`d` (or the mission's camera) Space and
the arrows paused and seeked a video nobody was watching.
"""
import queue
import threading
import types

from mongla_vision.utils.display_node import VisionDisplayNode


def _fake():
    made, gone = [], []
    n = types.SimpleNamespace(
        _active_camera='forward', _camera='forward', _video_mode=True,
        _cam_subs=[], _det_lock=threading.Lock(), _tracks_lock=threading.Lock(),
        _detections=[], _tracked_dets=[], _track_ids=[], _frame_q=queue.Queue(),
        _pause_client=None, _seek_pub=None, _seek_frame_pub=None, _speed_pub=None,
        get_logger=lambda: types.SimpleNamespace(info=lambda *_: None),
        destroy_subscription=lambda s: None,
        create_client=lambda t, name: made.append(name) or name,
        create_publisher=lambda t, name, q: made.append(name) or name,
        destroy_client=lambda c: gone.append(c),
        destroy_publisher=lambda p: gone.append(p))
    n._create_cam_subs = lambda cam: []
    n._video_controls = lambda cam: VisionDisplayNode._video_controls(n, cam)
    return n, made, gone


def test_a_switch_moves_every_playback_control_to_the_new_camera():
    n, made, gone = _fake()
    n._video_controls('forward')
    VisionDisplayNode._switch_camera(n, 'downward')
    for h in (n._pause_client, n._seek_pub, n._seek_frame_pub, n._speed_pub):
        assert h.startswith('/mongla/vision/downward/'), h
    assert sorted(gone) == sorted(h for h in made if '/forward/' in h)
