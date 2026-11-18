"""The tracker's clock is the camera's capture stamp, not the arrival time.

The Kalman's transition and process noise come from the interval between two
calls. At ~99 Hz that interval is 10 ms, so delivery jitter of a few
milliseconds was a 30 % error in every velocity the filter carried.
"""
from types import SimpleNamespace

from mongla_vision.tracker_node import capture_time


def _header(sec, nanosec):
    return SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=nanosec))


def test_the_interval_is_the_interval_between_exposures():
    a = capture_time(_header(1_700_000_000, 0))
    b = capture_time(_header(1_700_000_000, 10_100_000))
    assert abs((b - a) - 0.0101) < 1e-6


def test_an_unstamped_header_falls_back_to_arrival():
    import time
    t = capture_time(_header(0, 0))
    assert abs(t - time.monotonic()) < 1.0
