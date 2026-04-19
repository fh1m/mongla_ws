"""Tests for Heartbeat -- the always-warm RC override stream.

We don't talk to a real Pixhawk; a tiny stub records every
``send_neutral`` call so the tests can assert on cadence and on
pause/resume cooperation with imaginary other writers.
"""

import logging
import threading
import time

from duburi_control.heartbeat import Heartbeat


class ThrottleLogger:
    """stdlib logger that tolerates rclpy's ``throttle_duration_sec``."""

    def __init__(self):
        self._inner = logging.getLogger('test.heartbeat')

    def info(self, msg, *args, **kwargs):
        kwargs.pop('throttle_duration_sec', None)
        self._inner.info(msg, *args, **kwargs)

    def warn(self, msg, *args, **kwargs):
        self._inner.warning(msg, *args, **kwargs)

    def __getattr__(self, name):
        return getattr(self._inner, name)


class FakePixhawk:
    def __init__(self):
        self.neutrals = 0
        self._lock = threading.Lock()

    def send_neutral(self):
        with self._lock:
            self.neutrals += 1


def test_heartbeat_streams_neutral_at_cadence():
    pixhawk = FakePixhawk()
    log     = ThrottleLogger()

    hb = Heartbeat(pixhawk, log, hz=20.0)   # 50 ms period for a fast test
    hb.start()
    time.sleep(0.25)
    hb.stop()

    # 0.25 s @ 20 Hz -> roughly 5 packets, give it generous slack for
    # thread scheduling.
    assert pixhawk.neutrals >= 3, (
        f'expected at least 3 neutral writes in 0.25s @ 20Hz, '
        f'got {pixhawk.neutrals}')


def test_heartbeat_pause_stops_writes():
    pixhawk = FakePixhawk()
    log     = ThrottleLogger()

    hb = Heartbeat(pixhawk, log, hz=20.0)
    hb.start()
    time.sleep(0.15)
    hb.pause()
    time.sleep(0.05)            # let any in-flight tick land
    pre = pixhawk.neutrals
    time.sleep(0.25)
    post = pixhawk.neutrals
    hb.resume()
    time.sleep(0.15)
    final = pixhawk.neutrals
    hb.stop()

    assert post - pre <= 1, (
        f'paused heartbeat should not be writing -- got '
        f'{post - pre} extra packets while paused')
    assert final > post, (
        'resume must restart the heartbeat stream')


def test_heartbeat_pause_is_reentrant():
    """Two callers can stack ``pause`` / ``resume`` without one's
    resume turning the stream back on while the other still holds it."""
    pixhawk = FakePixhawk()
    log     = ThrottleLogger()

    hb = Heartbeat(pixhawk, log, hz=20.0)
    hb.start()
    time.sleep(0.1)

    hb.pause()                  # holder A
    hb.pause()                  # holder B
    time.sleep(0.05)
    pre = pixhawk.neutrals
    time.sleep(0.2)
    hb.resume()                 # holder A releases -- still held by B
    time.sleep(0.2)
    post_partial = pixhawk.neutrals

    hb.resume()                 # holder B releases -- stream resumes
    time.sleep(0.15)
    final = pixhawk.neutrals
    hb.stop()

    assert post_partial - pre <= 1, (
        'with one of two holders still active, the heartbeat must stay '
        'paused')
    assert final > post_partial, (
        'releasing the last hold must restart the heartbeat')


def test_heartbeat_hold_context_manager():
    pixhawk = FakePixhawk()
    log     = ThrottleLogger()

    hb = Heartbeat(pixhawk, log, hz=20.0)
    hb.start()
    time.sleep(0.1)

    with hb.hold():
        time.sleep(0.05)
        pre = pixhawk.neutrals
        time.sleep(0.25)
        post = pixhawk.neutrals
    time.sleep(0.15)
    final = pixhawk.neutrals
    hb.stop()

    assert post - pre <= 1, 'hold() must stop heartbeat writes'
    assert final > post, 'leaving hold() must resume the heartbeat'
