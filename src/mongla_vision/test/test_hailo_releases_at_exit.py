"""Every network group on the chip is handed back BEFORE the VDevice, at exit.

Measured on the vehicle 2026-10-03 (measured-bars §28.5): two detectors and
two XFeat anchors in one process, left to interpreter teardown, die SIGSEGV
(rc 139) or hang -- and a hung detector holds the chip the next launch needs.
Closing each group, then releasing the device, exits 0. No node called
`close()`, so the release has to happen without one: `atexit`.

These run without `hailo_platform`: the objects are fakes that record the
order things happen in, which is the whole property.
"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from mongla_vision.detection import hailo as hd          # noqa: E402
from mongla_vision.anchor.xfeat_hailo import XFeatHailo  # noqa: E402


class _Dev:
    def __init__(self, log): self.log = log
    def release(self): self.log.append('device')


class _Claimant:
    def __init__(self, name, log): self.name, self.log = name, log
    def close(self): self.log.append(self.name)


def _with_fakes(monkeypatch, log, names):
    monkeypatch.setattr(hd, '_DEVICE', _Dev(log))
    monkeypatch.setattr(hd, '_CLAIMANTS', hd.weakref.WeakSet())
    keep = [_Claimant(n, log) for n in names]
    for c in keep:
        hd._register(c)
    return keep


def test_groups_close_BEFORE_the_device_is_released(monkeypatch):
    log = []
    keep = _with_fakes(monkeypatch, log, ['det_a', 'det_b', 'xfeat'])
    hd.close_all()
    assert sorted(log[:-1]) == ['det_a', 'det_b', 'xfeat'], log
    assert log[-1] == 'device', 'the VDevice went before a group it holds'
    assert hd._DEVICE is None
    del keep


def test_it_is_safe_to_call_twice(monkeypatch):
    log = []
    keep = _with_fakes(monkeypatch, log, ['det'])
    hd.close_all()
    hd.close_all()
    assert log.count('device') == 1
    del keep


def test_one_failing_close_does_not_strand_the_rest(monkeypatch):
    log = []
    keep = _with_fakes(monkeypatch, log, ['a', 'b'])

    class _Bad:
        def close(self): raise RuntimeError('boom')
    bad = _Bad()
    hd._register(bad)
    hd.close_all()
    assert 'device' in log and {'a', 'b'} <= set(log)
    del keep, bad


def test_it_runs_at_exit_without_any_node_asking():
    """No node calls close(); that is how the crash shipped. The hook must be
    registered by importing the module, and nowhere else."""
    import inspect
    src = inspect.getsource(hd)
    assert 'atexit.register(close_all)' in src
    # At module level (unindented), so importing the module arms it.
    assert '\natexit.register(close_all)\n' in src


def test_both_backends_register_themselves():
    """A backend that configures a group and is not in the registry is a
    group the exit hook cannot hand back."""
    import inspect
    det = inspect.getsource(hd.HailoDetector.__init__)
    xf = inspect.getsource(XFeatHailo.__init__)
    assert '_register(self)' in det
    assert '_hd._register(self)' in xf


def test_a_closed_anchor_answers_empty_and_does_not_reconfigure():
    """A daemon thread can still call detect() after the exit hook ran.
    Re-acquiring would configure a fresh group on a device being released."""
    import threading
    x = XFeatHailo.__new__(XFeatHailo)
    x._lock = threading.Lock()
    x._closed = True
    x.w, x.h = 320, 240

    def _boom():
        raise AssertionError('reconfigured after close')
    x._configured = _boom
    k, d = x.detect(np.zeros((240, 320), np.uint8))
    assert k.shape == (0, 2) and d.shape == (0, 64)


def test_close_hands_back_the_group_and_marks_closed():
    import threading
    calls = []

    class _Cim:
        def deactivate(self): calls.append('deactivate')
        def __exit__(self, *a): calls.append('exit')
    x = XFeatHailo.__new__(XFeatHailo)
    x._hd, x._lock, x._closed = hd, threading.Lock(), False
    x._cim, x._bindings = _Cim(), object()
    x.close()
    assert x._closed and x._cim is None
    # Scheduler-owned: exit only, never a deactivate.
    assert calls == ['exit'], calls
