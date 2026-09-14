"""BUGS.md D16: a node that cannot infer must stop claiming to be up.

Observed on the vehicle: a Hailo stream abort left the detector ALIVE --
process up, topics up, subscriptions up -- logging a failure every frame and
publishing nothing, indefinitely. Every liveness check we own passed.

These drive the SHIPPING handler on a bare instance, so they test the escalation
that runs rather than a copy of it.
"""
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from duburi_vision import detector_node as D          # noqa: E402


class _Log:
    def __init__(self): self.lines = []
    def info(self, m): self.lines.append(('info', m))
    def warn(self, m): self.lines.append(('warn', m))
    def warning(self, m): self.lines.append(('warn', m))
    def error(self, m): self.lines.append(('error', m))
    def fatal(self, m): self.lines.append(('fatal', m))
    def debug(self, m): pass


def _node(rebuild_works=True):
    n = D.DetectorNode.__new__(D.DetectorNode)
    n._log = _Log()
    n.get_logger = lambda: n._log
    n._infer_fails = 0
    n._det = object()
    n._build_kwargs = {'model_path': 'x.hef'}
    n._rebuilt = []

    def rebuild():
        n._rebuilt.append(1)
        if rebuild_works:
            n._det = object()
            n._infer_fails = 0
    n._rebuild_detector = rebuild
    return n


def test_an_ISOLATED_failure_does_NOT_kill_the_node():
    """One bad frame is a bad frame. Killing the node for it would be worse
    than the fault it is reacting to."""
    n = _node()
    for _ in range(D._INFER_FAIL_REBUILD - 1):
        n._on_infer_failure(RuntimeError('one bad frame'))
    assert n._rebuilt == []
    assert not any(k == 'fatal' for k, _ in n._log.lines)


def test_a_SUCCESS_resets_the_counter():
    """A chip that recovers on its own must never reach the rebuild tier."""
    n = _node()
    for _ in range(D._INFER_FAIL_REBUILD - 1):
        n._on_infer_failure(RuntimeError('x'))
    n._infer_fails = 0                     # what the loop does on success
    for _ in range(D._INFER_FAIL_REBUILD - 1):
        n._on_infer_failure(RuntimeError('x'))
    assert n._rebuilt == [], 'a transient run escalated'


def test_CONSECUTIVE_failures_rebuild_the_detector():
    """A run of failures is the DEVICE, not the frame."""
    n = _node()
    for _ in range(D._INFER_FAIL_REBUILD):
        n._on_infer_failure(RuntimeError('HailoRTStreamAborted'))
    assert n._rebuilt == [1]


def test_the_rebuild_is_attempted_ONCE_not_every_frame():
    """Rebuilding on every frame would hammer a dead device and bury the log
    that says what is wrong."""
    n = _node(rebuild_works=False)
    for _ in range(D._INFER_FAIL_EXIT - 1):
        n._on_infer_failure(RuntimeError('x'))
    assert n._rebuilt == [1]


def test_it_EXITS_when_the_rebuild_did_not_help(monkeypatch):
    """THE FIX. Staying up is the failure mode, not the recovery -- the node
    was indistinguishable from a healthy one to every check we have."""
    n = _node(rebuild_works=False)
    codes = []
    monkeypatch.setattr(D.os, '_exit', lambda c: codes.append(c))
    for _ in range(D._INFER_FAIL_EXIT):
        n._on_infer_failure(RuntimeError('HailoRTStreamAborted'))
    assert codes == [1], 'the node kept claiming to be up'
    assert any(k == 'fatal' for k, _ in n._log.lines)


def test_the_thresholds_are_ordered_and_frame_based():
    """Sized in frames, not seconds, so the behaviour is the same at 3 Hz and
    at 80 -- and the rebuild must come before the exit."""
    assert 0 < D._INFER_FAIL_REBUILD < D._INFER_FAIL_EXIT


# --------------------------------------------------------------------------- #
#  The WIRING -- the handler is useless if the loop does not call it
# --------------------------------------------------------------------------- #
def _loop_node(infer, monkeypatch):
    """A bare node whose `_infer_loop` can be stepped, with the real body.

    Written after two injected defects were MISSED: the tests drove
    `_on_infer_failure` directly, so deleting the CALL to it, and deleting the
    success reset, both passed. The escalation was tested and the wiring was
    not, which is the same shape as a guard that greps the source.
    """
    import threading
    import queue as q
    n = D.DetectorNode.__new__(D.DetectorNode)
    n._log = _Log()
    n.get_logger = lambda: n._log
    n._infer_fails = 0
    n._want = threading.Event()
    n._infer_q = q.Queue()
    n._bridge = None
    n._pre = None
    n._crop = None
    n._publish_dbg = False
    n._det = SimpleNamespace(infer=infer)
    n._build_kwargs = None
    n.get_parameter = lambda _k: SimpleNamespace(value=False)
    n._rebuild_detector = lambda: None
    # everything after the infer is not under test here
    monkeypatch.setattr(D, 'rclpy', SimpleNamespace(ok=lambda: n._infer_q.qsize() > 0))
    return n


def test_the_LOOP_actually_calls_the_failure_handler(monkeypatch):
    """Delete the call and the counter never moves -- which is the D16 zombie
    restored. Driving only the handler could not see it."""
    n = _loop_node(lambda _f: (_ for _ in ()).throw(RuntimeError('abort')),
                   monkeypatch)
    for _ in range(4):
        n._infer_q.put(D._DirectFrame(frame=object(), header=None))
    try:
        n._infer_loop()
    except Exception:
        pass
    assert n._infer_fails == 4, f'loop did not escalate ({n._infer_fails})'


def test_a_SUCCESSFUL_inference_in_the_LOOP_resets_the_counter(monkeypatch):
    """Without the reset a perfectly healthy chip escalates to a rebuild after
    enough scattered bad frames over a whole mission."""
    calls = {'n': 0}

    def flaky(_f):
        calls['n'] += 1
        if calls['n'] <= 3:
            raise RuntimeError('transient')
        return []

    n = _loop_node(flaky, monkeypatch)
    for _ in range(5):
        n._infer_q.put(D._DirectFrame(frame=object(), header=None))
    try:
        n._infer_loop()
    except Exception:
        pass
    assert n._infer_fails == 0, 'a success did not clear the failure run'


# --------------------------------------------------------------------------- #
#  D16's twin: no model at all
# --------------------------------------------------------------------------- #
def test_a_FAILED_MODEL_LOAD_exits_instead_of_running_as_a_no_op(monkeypatch):
    """Observed on the vehicle: `Failed to open device file /dev/hailo0 with
    error 6` after a restart race. The old code logged FATAL and RETURNED,
    leaving `_det` None -- so the loop dropped every frame at `if det is None:
    continue`, silently, forever, while params answered and the stats line
    still printed. Identical signature to D16, one layer earlier."""
    n = D.DetectorNode.__new__(D.DetectorNode)
    n._log = _Log()
    n.get_logger = lambda: n._log
    n._det = None
    n._pending_allowlist = None
    codes = []

    def fake_exit(c):
        # The real `os._exit` NEVER RETURNS. A fake that returns lets execution
        # run on into code that assumed it had stopped -- which is how this
        # test first failed with an UnboundLocalError instead of passing.
        codes.append(c)
        raise SystemExit(c)

    monkeypatch.setattr(D.os, '_exit', fake_exit)
    monkeypatch.setattr(D, 'make_detector',
                        lambda **_k: (_ for _ in ()).throw(RuntimeError('hailo0 busy')))
    with pytest.raises(SystemExit):
        n._load_single_model_async(model_path='x.hef', device='', conf=0.3,
                                   iou=0.5, imgsz=640, half=False, max_det=10,
                                   allowlist=None)
    assert codes == [2], 'a model-less detector kept claiming to be up'
    assert any(k == 'fatal' for k, _ in n._log.lines)


def test_the_inference_counter_does_not_count_frames_it_never_inferred():
    """It reported "300 inferences" during a run in which zero inferences
    happened -- the single reason the dead detector looked busy. A counter
    named for something it does not count is worse than no counter."""
    src = (Path(__file__).resolve().parents[1]
           / 'duburi_vision' / 'detector_node.py').read_text()
    i = src.index('self._infers += 1')
    guard = src[max(0, i - 260):i]
    assert 'self._det is not None' in guard, \
        'the inference counter is not gated on having a detector'


# --------------------------------------------------------------------------- #
#  "Can this camera see?" runs in the LOOP, not beside it
# --------------------------------------------------------------------------- #
def test_the_LOOP_reports_a_blind_camera(monkeypatch):
    """Executed through `_infer_loop`: a covered lens must be published as
    blind, so a mission can tell "cannot see" from "nothing there"."""
    import numpy as np
    from duburi_vision.seeing import Seeing
    # A failing infer keeps the loop stepping frame after frame (the publish
    # path after a successful infer is not stubbed and would end the loop).
    n = _loop_node(lambda _f: (_ for _ in ()).throw(RuntimeError('x')),
                   monkeypatch)
    n._seeing = Seeing()
    n._seeing_state = None
    sent = []
    n._pub_seeing = SimpleNamespace(publish=lambda m: sent.append(m.data))
    rng = np.random.default_rng(0)
    for _ in range(4):
        dark = rng.integers(0, 4, (480, 640, 3), dtype=np.uint8)
        n._infer_q.put(D._DirectFrame(frame=dark, header=None))
    try:
        n._infer_loop()
    except Exception:
        pass
    assert sent and sent[-1] == 'covered', sent


def test_a_FAULT_in_the_seeing_check_never_stops_detection(monkeypatch):
    """The check runs inside the worker loop; any exception there must not end
    the loop. The first draft's log line raised and did exactly that."""
    import numpy as np
    n = _loop_node(lambda _f: (_ for _ in ()).throw(RuntimeError('x')),
                   monkeypatch)
    n._seeing = SimpleNamespace(observe=lambda _f: (_ for _ in ()).throw(ValueError('boom')))
    n._seeing_state = None
    for _ in range(4):
        n._infer_q.put(D._DirectFrame(frame=np.zeros((4, 4, 3), np.uint8), header=None))
    try:
        n._infer_loop()
    except Exception:
        pass
    assert n._infer_fails == 4, 'the seeing fault ended the loop'
