"""Camera -> detector by REFERENCE, in one process, with the header attached.

WHAT COMPOSITION ACTUALLY BUYS
------------------------------
Removing the topic between the two removes, per frame:

    cv_bridge encode   0.845 ms
    serialise          1.469 ms
    transport          1.760 ms
    imgmsg_to_cv2      a second full-frame copy

...and, worth more than all of it, THE PUBLISHER'S CLOCK. On the topic path
the detector acts on whichever frame the publish throttle handed over. Composed,
the camera reads `wants_frame()` and decodes when inference goes idle, so the
picture is the newest one that exists.

`ComposableNodeContainer` would buy none of this -- rclpy has no intra-process
comms, so composed Python nodes still traverse rmw. The gain is the direct
Python reference.

WHAT THESE TESTS PIN
--------------------
The three things that would each fail SILENTLY:

  1. the header travels with the pixels -- without it `detections` carry a
     stamp that is not the capture time, and every freshness gate downstream
     (including the mid-hold torpedo fire) goes back to measuring the wrong
     thing. This is the round-30/32 defect at a third layer.
  2. a composed detector does NOT also subscribe -- it would decode and infer
     the same picture twice, and the topic copy is the SLOWER of the two, so
     it would be the one acted on half the time.
  3. a PAUSED detector stops asking for frames -- the unused camera is paused
     for most of a mission, and decoding for a consumer that discards is the
     defect this round already fixed once.
"""
import sys
import threading
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from duburi_vision import camera_node as CN                    # noqa: E402
from duburi_vision import detector_node as DN                  # noqa: E402


# --------------------------------------------------------------------------- #
#  The payload
# --------------------------------------------------------------------------- #
def test_the_direct_payload_carries_the_header_not_just_pixels():
    """A bare ndarray would compile and run and be wrong -- `detections` would
    be stamped with whatever the detector felt like."""
    f = DN._DirectFrame(frame='PIXELS', header='HEADER')
    assert f.frame == 'PIXELS' and f.header == 'HEADER'
    assert set(DN._DirectFrame._fields) == {'frame', 'header'}


# --------------------------------------------------------------------------- #
#  The camera's side of the handoff
# --------------------------------------------------------------------------- #
class _Meta:
    def __init__(self, wall=None):
        self.fresh = True
        self.stamp_wall = wall if wall is not None else time.time()
        self.stamp_monotonic = time.monotonic()
        self.width, self.height, self.frame_index = 64, 48, 0


class _Cam:
    def __init__(self):
        self.reads = 0

    def read(self):
        self.reads += 1
        return f'frame{self.reads}', _Meta()


class _Sink:
    """Stands in for a DetectorNode: demand flag + submissions."""

    def __init__(self, wants=True):
        self._wants = wants
        self.got = []

    def wants_frame(self):
        return self._wants

    def submit_frame(self, frame, header):
        self.got.append((frame, header))


def _run_loop(node, seconds):
    stop = threading.Event()

    class _Rclpy:
        @staticmethod
        def ok():
            return not stop.is_set()
    real, CN.rclpy = CN.rclpy, _Rclpy()
    try:
        t = threading.Thread(target=node._capture_loop, daemon=True)
        t.start()
        time.sleep(seconds)
        stop.set()
        t.join(timeout=2.0)
    finally:
        CN.rclpy = real


def _camera(sink, publish_hz=40):
    node = object.__new__(CN.CameraNode)
    node._cam = _Cam()
    node._sink = sink
    node._frame_id = 'forward'
    node._min_period = (1.0 / publish_hz) if publish_hz > 0 else 0.0
    node._last_pub = 0.0
    node._IDLE_WAIT_S = CN.CameraNode._IDLE_WAIT_S
    node.published = []
    node._publish = lambda f, m: (
        node.published.append(f),
        setattr(node, '_last_pub', time.monotonic()))
    return node


def test_an_idle_sink_is_fed_regardless_of_the_publish_clock():
    """THE POINT. At 5 Hz publish an always-idle detector must still be fed
    far faster than 5 Hz, or composition bought nothing."""
    sink = _Sink(wants=True)
    node = _camera(sink, publish_hz=5)
    _run_loop(node, 0.5)
    assert len(sink.got) > 20, (
        f'only {len(sink.got)} frames in 0.5 s -- the sink is still gated on '
        f'the publish clock')


def test_a_sink_that_wants_nothing_costs_no_decode():
    """A paused detector plus a 5 Hz topic must decode ~5 Hz, not 200.

    This is the same defect as the rate gate behind the decode, one layer on:
    the unused camera is paused for most of a mission."""
    sink = _Sink(wants=False)
    node = _camera(sink, publish_hz=5)
    _run_loop(node, 0.6)
    assert sink.got == []
    assert node._cam.reads <= 5, (
        f'{node._cam.reads} decodes for a sink that wants nothing')


def test_the_submitted_header_is_the_CAPTURE_stamp():
    """Not `now()`. The whole chain of freshness gates reads this field, and
    it has already been got wrong at two other layers."""
    sink = _Sink(wants=True)
    node = _camera(sink, publish_hz=0)
    captured = time.time() - 0.30
    node._cam.read = lambda: ('f', _Meta(wall=captured))
    _run_loop(node, 0.05)
    assert sink.got, 'nothing submitted'
    _f, header = sink.got[0]
    stamp = header.stamp.sec + header.stamp.nanosec * 1e-9
    assert abs(stamp - captured) < 0.01, (
        f'header stamp is {time.time() - stamp:.3f}s old, expected ~0.30 -- '
        f'the capture time was lost in the handoff')
    assert header.frame_id == 'forward'


def test_one_decode_serves_both_the_sink_and_the_topic():
    """The frame handed to the detector and the frame published must be the
    same object -- decoding twice would give back the CPU this round saved."""
    sink = _Sink(wants=True)
    node = _camera(sink, publish_hz=0)
    _run_loop(node, 0.1)
    assert node.published and sink.got
    assert sink.got[0][0] is node.published[0]


def test_no_sink_behaves_exactly_as_before():
    """The un-composed path -- `vision.launch.py`, the Jetson, sim -- must be
    untouched by any of this."""
    node = _camera(sink=None, publish_hz=0)
    _run_loop(node, 0.1)
    assert node._cam.reads == len(node.published) > 10


# --------------------------------------------------------------------------- #
#  The detector's side
# --------------------------------------------------------------------------- #
def test_the_demand_flag_is_set_only_while_the_worker_waits():
    """`wants_frame()` must mean "would consume NOW", not "exists". A flag that
    is always true makes the camera decode at capture rate again."""
    det = object.__new__(DN.DetectorNode)
    det._want = threading.Event()
    det._paused = False
    det.get_parameter = lambda _n: type('P', (), {'value': False})()
    assert det.wants_frame() is False        # worker not waiting yet
    det._want.set()
    assert det.wants_frame() is True


def test_a_paused_detector_wants_nothing():
    det = object.__new__(DN.DetectorNode)
    det._want = threading.Event()
    det._want.set()
    det.get_parameter = lambda _n: type('P', (), {'value': True})()
    assert det.wants_frame() is False


def test_submit_replaces_rather_than_queues():
    """A MAILBOX. Two submissions while the worker is busy must leave the
    NEWEST, or composition reintroduces exactly the queue it removed."""
    import queue as _q
    det = object.__new__(DN.DetectorNode)
    det._infer_q = _q.SimpleQueue()
    det.submit_frame('old', 'h1')
    det.submit_frame('new', 'h2')
    item = det._infer_q.get_nowait()
    assert item.frame == 'new'
    assert det._infer_q.empty(), 'the old frame is still queued behind it'


def test_a_composed_detector_does_not_also_subscribe():
    """Subscribing as well would decode and infer the same picture twice, and
    the topic copy is the slower of the two -- so it would be the one acted on
    half the time, at random."""
    src = (Path(__file__).resolve().parents[1] / 'duburi_vision'
           / 'detector_node.py').read_text()
    assert "self._sub = None if self._direct else self.create_subscription(" in src


def test_the_composed_launcher_turns_direct_feed_ON():
    """`direct_feed` defaults False so every existing launch is unchanged; the
    composed process is the only thing that sets it. If it stopped doing so,
    the detector would subscribe AND be fed, silently doubling the work."""
    src = (Path(__file__).resolve().parents[1] / 'duburi_vision'
           / 'detector_dual_node.py').read_text()
    assert "Parameter('direct_feed', value=True)" in src
    assert 'frame_sink=det' in src


# --------------------------------------------------------------------------- #
#  Process-wide parameters: two nodes, one namespace
# --------------------------------------------------------------------------- #
def test_no_parameter_name_means_two_things_in_one_process():
    """Launch parameters are PROCESS-wide, so a name declared by BOTH kinds of
    node must be given per-camera explicitly or the process-wide value reaches
    the wrong one.

    `device` is exactly that: the detector's backend (`'cpu'`, a string) and
    the camera's V4L2 index (`-1`, an integer). Composing them killed the whole
    process at startup with InvalidParameterTypeException -- loudly, at least,
    but only on hardware. This test is the cheap version of that discovery.
    """
    from duburi_vision import detector_dual_node as DD

    cam_src = (Path(__file__).resolve().parents[1] / 'duburi_vision'
               / 'camera_node.py').read_text()
    camera_params = {
        line.split("'")[1]
        for line in cam_src.splitlines()
        if "self.declare_parameter('" in line
    }
    assert 'device' in camera_params, 'fixture stale -- re-read camera_node'

    shared = set(DD._SHARED)
    per_cam = set(DD._CAM_PER_CAMERA)
    clash = (shared & camera_params) - per_cam
    assert not clash, (
        f'{sorted(clash)} is declared by BOTH node types and is only set '
        f'process-wide. Add it to _CAM_PER_CAMERA so the camera gets its own '
        f'value, or the process-wide one lands on the wrong node.')


def test_the_camera_override_actually_carries_device():
    """Not just listed -- emitted. A key in the table that never reaches a
    Parameter is the `device_path`-into-`**_` defect again."""
    from duburi_vision import detector_dual_node as DD
    assert 'device' in DD._CAM_PER_CAMERA
    assert DD._CAM_DEFAULTS['device'] == -1, 'must be the camera sentinel'


# --------------------------------------------------------------------------- #
#  direct_feed defaults ON, and cannot leave a detector silently dead
# --------------------------------------------------------------------------- #
def test_direct_feed_defaults_ON():
    """The vehicle runs composed, so the composed arrangement is the default.
    Leaving it off meant the launcher had to remember to switch it on, and
    forgetting cost a double decode+infer with the SLOWER copy winning half
    the time."""
    src = (Path(__file__).resolve().parents[1] / 'duburi_vision'
           / 'detector_node.py').read_text()
    assert "self.declare_parameter('direct_feed',         True)" in src


def test_a_direct_detector_with_no_camera_SUBSCRIBES_rather_than_dying():
    """The risk of defaulting it on: a standalone `detector_node` -- sim,
    replay, the Jetson, `vision.launch.py` -- has no camera in its process and
    would receive NOTHING, for ever, with no error. That is precisely the
    failure mode this round exists to remove, so the default carries its own
    recovery."""
    calls = {'subscribed': 0, 'warned': []}
    det = object.__new__(DN.DetectorNode)
    det._direct = True
    det._fed_direct = False
    det._sub = None
    det._ns_in = '/duburi/vision/forward/image_raw'
    det._on_image = lambda _m: None
    det._fallback_timer = type('T', (), {'cancel': lambda self: None})()

    def _create_sub(*_a, **_k):
        calls['subscribed'] += 1
        return object()
    det.create_subscription = _create_sub
    det.get_logger = lambda: type('L', (), {
        'warn': lambda _s, m: calls['warned'].append(m)})()

    det._check_direct_feed()
    assert calls['subscribed'] == 1, 'no camera and no subscription = dead node'
    assert calls['warned'], 'it must SAY it fell back, not do it quietly'
    assert det._direct is False


def test_a_detector_that_WAS_fed_directly_does_not_subscribe():
    """The composed case: the fallback must not add a redundant subscription
    on top of a working direct feed -- that is the double-work it prevents."""
    calls = {'subscribed': 0}
    det = object.__new__(DN.DetectorNode)
    det._direct = True
    det._fed_direct = True          # a frame arrived by reference
    det._sub = None
    det._ns_in = '/x'
    det._fallback_timer = type('T', (), {'cancel': lambda self: None})()
    det.create_subscription = lambda *_a, **_k: calls.__setitem__(
        'subscribed', calls['subscribed'] + 1)
    det.get_logger = lambda: type('L', (), {'warn': lambda _s, m: None})()

    det._check_direct_feed()
    assert calls['subscribed'] == 0
    assert det._direct is True


def test_submit_frame_records_that_the_direct_path_is_alive():
    """The flag the fallback reads. If `submit_frame` did not set it, a
    perfectly healthy composed detector would add a second, redundant
    subscription two seconds after startup."""
    import queue as _q
    det = object.__new__(DN.DetectorNode)
    det._infer_q = _q.SimpleQueue()
    det._fed_direct = False
    det.submit_frame('f', 'h')
    assert det._fed_direct is True
