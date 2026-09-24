"""The companion HEARTBEAT keeps going while the rest of the node is busy.

The board SURFACES the vehicle after 5 s of heartbeat silence (GCS_FAILSAFE_MS).
The heartbeat used to be a 2 Hz timer in the MutuallyExclusive `timer_group`,
sharing it with callbacks that block for seconds (`_vision_uplink_tick` waits up
to 10 s for a CameraInfo, `_reapply_srot_config` busy-waits up to 3 s), and it
did not exist at all until `executor.spin()` -- so every slow bring-up step after
the port opened was silent too.

This drives the REAL `_setup_mavlink` against a fake master and a real `SrotFC`,
then blocks the calling thread for 6 s -- standing in for a blocked timer-group
callback or a slow preflight -- and reads the heartbeats that reached the wire.
"""
import pathlib
import sys
import threading
import time
from types import SimpleNamespace

from mongla_manager import auv_manager_node as amn

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]
                       / 'mongla_control' / 'test'))
from test_srot_fc import _FakeMaster   # noqa: E402


class _TimedMaster(_FakeMaster):
    """`_FakeMaster` that stamps every heartbeat with the monotonic time it left."""

    def __init__(self):
        super().__init__()
        self.hb_times = []
        orig = self.mav.heartbeat_send

        def _hb(*a):
            self.hb_times.append(time.monotonic())
            orig(*a)
        self.mav.heartbeat_send = _hb

    def wait_heartbeat(self, *a, **k):
        return None


class _Log:
    def info(self, *_a, **_k):
        pass

    warning = warn = error = info


class _StubNode:
    """Just what `_setup_mavlink` reads -- no ROS graph, no port."""

    def __init__(self):
        self._mode_name = 'test'
        self._profile = {'conn': '/dev/null-mongla-test', 'baud': 115200}
        self._is_srot = True
        self._fc_kind = 'srot'
        self._log = _Log()

    def get_logger(self):
        return self._log

    def get_parameter(self, _name):
        return SimpleNamespace(value=False)


class _NoGuard:
    def __init__(self, *a, **k):
        pass

    def acquire(self):
        pass


def test_heartbeats_keep_flowing_while_the_node_is_blocked(monkeypatch):
    master = _TimedMaster()
    monkeypatch.setattr(amn, 'PortGuard', _NoGuard)
    monkeypatch.setattr(amn.mavutil, 'mavlink_connection', lambda *a, **k: master)

    node = _StubNode()
    try:
        amn.AUVManagerNode._setup_mavlink(node)
        t0 = time.monotonic()
        # A timer_group callback (or a preflight read) that holds this thread 6 s
        # -- longer than the board's 5 s failsafe window.
        blocker = threading.Thread(target=time.sleep, args=(6.0,))
        blocker.start()
        blocker.join()
        t1 = time.monotonic()
    finally:
        hb = getattr(node, '_hb_thread', None)
        if hb is not None:
            hb.stop()

    beats = [t for t in master.hb_times if t0 <= t <= t1]
    assert len(beats) >= 10, (
        f'only {len(beats)} heartbeats in 6 s of a blocked node -- the board '
        f'surfaces after 5 s of silence')
    edges = [t0] + beats + [t1]
    worst = max(b - a for a, b in zip(edges, edges[1:]))
    assert worst < 1.5, f'a {worst:.2f} s heartbeat gap while the node was blocked'


def test_the_heartbeat_thread_stops_and_survives_a_failing_send():
    """It must never die on a write error (a USB hiccup), and it must stop."""
    calls = []

    def _send():
        calls.append(time.monotonic())
        if len(calls) == 1:
            raise OSError('write failed')

    hb = amn._HeartbeatThread(_send, period_s=0.05, log=_Log()).start()
    time.sleep(0.4)
    hb.stop()
    n = len(calls)
    assert n >= 4, 'a single failed send killed the heartbeat'
    time.sleep(0.2)
    assert len(calls) == n, 'the heartbeat kept sending after stop()'
