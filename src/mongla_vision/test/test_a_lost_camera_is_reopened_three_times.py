"""A camera that stops delivering is reopened in place, three times, then left down.

Two layers. The policy (`cameras/reopen.CameraWatch`) is checked on a clock we
drive. The wiring is checked on the real `camera_node._watch_camera` with a
fake source whose device "comes back" or never does, so the counts are what
the node actually did, not what the policy says it would.
"""
from __future__ import annotations

import types

from mongla_localization.launch_respawn import MAX_RESTARTS
from mongla_vision import camera_node
from mongla_vision.cameras.reopen import (
    ABSENT, GAVE_UP, LOST_AFTER_S, OK, REOPEN, WAITING, CameraWatch)


def _run(watch: CameraWatch, healthy, t_end: float, dt: float = 0.5,
         present=lambda t: True):
    """Step the watch until `t_end`; `healthy(t)` says what the camera reports."""
    out, t = [], 0.0
    while t <= t_end:
        out.append((t, watch.step(healthy(t), t, present=present(t))))
        t += dt
    return out


def test_the_limit_is_the_owner_policy_shared_with_process_restarts():
    assert MAX_RESTARTS == 3
    assert CameraWatch('v4l2_mailbox').max_reopens == MAX_RESTARTS


def test_a_healthy_camera_is_never_touched():
    verdicts = {v for _, v in _run(CameraWatch('v4l2_mailbox'), lambda t: True, 60)}
    assert verdicts == {OK}


def test_a_short_gap_is_waited_out_not_reopened():
    watch = CameraWatch('webcam')
    blip = lambda t: not (1.0 <= t < 1.0 + LOST_AFTER_S - 0.5)   # noqa: E731
    verdicts = [v for _, v in _run(watch, blip, 20)]
    assert REOPEN not in verdicts and WAITING in verdicts


def test_a_dead_device_is_reopened_three_times_then_given_up():
    verdicts = [v for _, v in _run(CameraWatch('v4l2_mailbox'), lambda t: False, 60)]
    assert verdicts.count(REOPEN) == 3
    assert verdicts[-1] == GAVE_UP
    # Spaced by the grace period: a fresh source is given time to deliver.
    times = [t for t, v in _run(CameraWatch('v4l2_mailbox'), lambda t: False, 60)
             if v == REOPEN]
    assert all(b - a >= LOST_AFTER_S for a, b in zip(times, times[1:]))


def test_a_camera_that_comes_back_is_left_alone_after():
    back_at = LOST_AFTER_S + 1.0
    verdicts = [v for _, v in _run(CameraWatch('v4l2_mailbox'),
                                   lambda t: t >= back_at, 60)]
    assert verdicts.count(REOPEN) == 1 and verdicts[-1] == OK


def test_a_lead_pulled_out_spends_no_attempt_and_comes_back():
    # Measured on the vehicle: three attempts spent in 6 s while the device
    # node did not exist, and the camera stayed LOST after the lead went back.
    back = 30.0
    out = _run(CameraWatch('v4l2_mailbox'), lambda t: t >= back + LOST_AFTER_S + 1,
               60, present=lambda t: t < 2.0 or t >= back)
    verdicts = [v for _, v in out]
    assert GAVE_UP not in verdicts
    assert ABSENT in verdicts
    assert verdicts.count(REOPEN) == 1
    # The reopen waits the grace period after the device reappears.
    first = next(t for t, v in out if v == REOPEN)
    assert first >= back + LOST_AFTER_S
    assert verdicts[-1] == OK


def test_a_video_file_or_topic_is_never_reopened():
    for kind in ('video_file', 'ros_topic'):
        verdicts = {v for _, v in _run(CameraWatch(kind), lambda t: False, 60)}
        assert REOPEN not in verdicts and GAVE_UP not in verdicts, kind


# --- the wiring, on the real node method -----------------------------------

class _Source:
    def __init__(self, healthy: bool, kind: str = 'v4l2_mailbox',
                 device: str = '/dev/null'):
        self.healthy, self.closed = healthy, False
        self.source_kind, self._device = kind, device

    def info(self):
        return {'device': self._device}

    def is_healthy(self):
        return self.healthy

    def close(self):
        self.closed = True


class _Log:
    def __init__(self):
        self.lines = []

    def warning(self, m):
        self.lines.append(m)

    error = debug = warning


def _node(first: _Source, build):
    log = _Log()
    node = types.SimpleNamespace(_cam=first, _cam_name='downward',
                                 _build_camera=build, get_logger=lambda: log)
    return node, log


def _drive(node, seconds: float = 60.0) -> bool:
    watch, t = CameraWatch('v4l2_mailbox'), 0.0
    while t <= seconds:
        if not camera_node.CameraNode._watch_camera(node, watch, t):
            return False
        t += 0.5
    return True


def test_the_node_swaps_in_a_fresh_source_when_the_device_returns():
    lost, fresh = _Source(False), _Source(True)
    node, log = _node(lost, lambda: fresh)
    assert _drive(node) is True
    assert node._cam is fresh and lost.closed
    assert sum('reopening' in m for m in log.lines) == 1


def test_the_node_stops_loudly_when_the_device_never_returns():
    built = []

    def build():
        built.append(_Source(False))
        return built[-1]
    node, log = _node(_Source(False), build)
    assert _drive(node) is False            # the capture loop ends
    assert len(built) == 3
    assert any('LOST' in m and 'stays down' in m for m in log.lines)


def test_a_reopen_that_downgrades_the_source_is_refused():
    # The factory falls back to the OpenCV source when the mailbox will not
    # open; on the vehicle that is a 396 ms-stale camera for the whole run.
    made = []

    def build():
        made.append(_Source(True, kind='webcam'))
        return made[-1]
    first = _Source(False)
    node, log = _node(first, build)
    assert _drive(node) is False
    assert node._cam is first                  # never swapped in
    assert len(made) == 3 and all(m.closed for m in made)
    assert sum('refused' in m for m in log.lines) == 3


def test_a_missing_device_is_reported_once_not_opened():
    built = []
    node, log = _node(_Source(False, device='/dev/mongla_no_such_camera'),
                      lambda: built.append(1))
    assert _drive(node) is True                # waits, never gives up
    assert built == []
    assert sum('GONE' in m for m in log.lines) == 1


def test_a_failing_open_counts_as_an_attempt_and_never_raises():
    def build():
        raise OSError('No such device')
    node, log = _node(_Source(False), build)
    assert _drive(node) is False
    assert sum('reopen failed' in m for m in log.lines) == 3
