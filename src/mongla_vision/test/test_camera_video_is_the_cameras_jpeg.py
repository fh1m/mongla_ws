"""The operator's video is the camera's own JPEG: never re-encoded, rate-capped, and free unwatched.

Batighor shows the cameras in a browser. The camera already compressed every
frame (MJPG over USB), so the cheapest picture Kutum can send is the one it
was given. These hold the three properties that make that true: the bytes
published are the bytes captured, each topic keeps its own rate, and a topic
with no viewer costs nothing.
"""
import cv2
import numpy as np

from mongla_vision import camera_node as CN
from mongla_vision.cameras.camera import FrameMeta


class _Pub:
    def __init__(self, subs=1):
        self.subs, self.sent = subs, []

    def get_subscription_count(self):
        return self.subs

    def publish(self, msg):
        self.sent.append(msg)


def _node(stream_hz=30.0, preview_hz=5.0, subs=(1, 1)):
    node = object.__new__(CN.CameraNode)
    node._jpeg_out = [[_Pub(subs[0]), 1.0 / stream_hz if stream_hz else None, -1e9],
                      [_Pub(subs[1]), 1.0 / preview_hz if preview_hz else None, -1e9]]
    return node


def _jpeg():
    img = np.zeros((36, 64, 3), np.uint8)
    img[:, :32] = (0, 128, 255)
    ok, buf = cv2.imencode('.jpg', img)
    assert ok
    return buf.tobytes()


def _header():
    return CN.Header(frame_id='cam')


def test_the_bytes_published_are_the_bytes_captured():
    node, jpg = _node(), _jpeg()
    node._publish_jpeg(FrameMeta(jpeg=jpg), _header(), now=0.0)
    for out in node._jpeg_out:
        (msg,) = out[0].sent
        assert bytes(msg.data) == jpg and msg.format == 'jpeg'
        assert msg.header.frame_id == 'cam'


def test_each_topic_keeps_its_own_rate():
    """One second of a 210 Hz camera: 30 frames on the stream, 5 on the preview."""
    node, meta = _node(), FrameMeta(jpeg=_jpeg())
    for i in range(210):
        node._publish_jpeg(meta, _header(), now=i / 210.0)
    stream, preview = (len(o[0].sent) for o in node._jpeg_out)
    assert stream == 30, stream
    assert preview == 5, preview


def test_a_cap_holds_its_rate_on_a_camera_that_does_not_divide_it():
    """Stamping each slot with its publish time loses one frame-wait a period:
    this gate gave 27 Hz for 30 on a 210 Hz camera. On a jittered 62.5 Hz
    camera over ten seconds the cap is met to within a frame."""
    rng = np.random.default_rng(0)
    t = np.cumsum(rng.normal(1 / 62.5, 0.0015, 625))
    for hz in (30.0, 20.0, 7.0):
        node, meta = _node(stream_hz=hz, preview_hz=0), FrameMeta(jpeg=_jpeg())
        for now in t:
            node._publish_jpeg(meta, _header(), now=float(now))
        got = len(node._jpeg_out[0][0].sent) / (t[-1] - t[0])
        assert abs(got - hz) <= 0.2, (hz, got)


def test_the_image_gate_keeps_the_same_schedule():
    last, n = 0.0, 0
    for i in range(1, 2101):                    # ten seconds at 210 Hz
        now = 1000.0 + i / 210.0
        if now - last >= 1 / 40.0:
            last, n = CN.advance_slot(last, 1 / 40.0, now), n + 1
    assert 399 <= n <= 401, n


def test_a_topic_nobody_watches_is_never_built():
    node = _node(subs=(0, 0))
    node._publish_jpeg(FrameMeta(jpeg=_jpeg()), _header(), now=0.0)
    assert all(not o[0].sent for o in node._jpeg_out)
    # A viewer arriving later gets the next frame, not one a period later.
    node._jpeg_out[1][0].subs = 1
    node._publish_jpeg(FrameMeta(jpeg=_jpeg()), _header(), now=0.01)
    assert len(node._jpeg_out[1][0].sent) == 1


def test_zero_turns_a_topic_off_and_a_decoded_source_sends_no_video():
    node = _node(preview_hz=0)
    node._publish_jpeg(FrameMeta(jpeg=_jpeg()), _header(), now=0.0)
    assert len(node._jpeg_out[0][0].sent) == 1 and not node._jpeg_out[1][0].sent
    node = _node()
    node._publish_jpeg(FrameMeta(), _header(), now=0.0)    # OpenCV, a file, a topic
    assert all(not o[0].sent for o in node._jpeg_out)


def test_the_mailbox_hands_over_the_payload_it_decoded():
    from mongla_vision.cameras.v4l2_mailbox import V4L2MailboxCamera
    cam = object.__new__(V4L2MailboxCamera)
    jpg = _jpeg()
    cam._slot = (jpg, 1.0, 7, 1.001)
    cam._served, cam._idx, cam._w, cam._h = 6, 0, 64, 36
    cam._fourcc = 'MJPG'
    cam._decoder = cam._decode_mjpeg
    frame, meta = cam.read()
    assert meta.fresh and frame is not None and frame.shape == (36, 64, 3)
    assert meta.jpeg is jpg
    cam._slot, cam._fourcc = (jpg, 2.0, 8, 2.001), 'YUYV'
    cam._decoder = lambda _payload: np.zeros((36, 64, 3), np.uint8)
    assert cam.read()[1].jpeg == b''
