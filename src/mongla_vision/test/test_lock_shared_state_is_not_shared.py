"""Three races and aliases in the lock node and the live-parameter callback
(audit 2026-11-24, vl_13 F13-5, F13-6, F13-7)."""
import inspect
from types import SimpleNamespace

from std_msgs.msg import Header

from mongla_vision import live_params, lock_frames, lock_publish


def test_a_failed_batch_puts_back_what_it_applied(monkeypatch):
    state = {'a': 1.0}
    stored = {'a': 1.0, 'b': 0.0}
    cb = []

    def bad(_v):
        raise ValueError('no')
    node = SimpleNamespace(add_on_set_parameters_callback=cb.append,
                           get_parameter=lambda n: SimpleNamespace(value=stored[n]))
    monkeypatch.setattr(live_params, 'changes', lambda _n, ps: ps)
    live_params.apply_live(node, {'a': lambda v: state.__setitem__('a', v), 'b': bad})
    r = cb[0]([SimpleNamespace(name='a', value=5.0), SimpleNamespace(name='b', value=1.0)])
    assert not r.successful and state['a'] == 1.0


def test_the_publisher_never_writes_into_the_callers_header():
    sent = []
    me = SimpleNamespace(_decomp_pub=SimpleNamespace(publish=sent.append),
                         _cam='downward', _target_w_m=0.6)
    h = Header(frame_id='forward')
    lock_publish.LockPublishMixin._publish_decomposition(
        me, SimpleNamespace(ok=True, theta=1.0, scale=1.0), h)
    assert sent and h.frame_id == 'forward'


def test_the_frame_ring_is_read_under_its_lock():
    src = inspect.getsource(lock_frames.LockFramesMixin._frame_for)
    assert 'with self._lock' in src
