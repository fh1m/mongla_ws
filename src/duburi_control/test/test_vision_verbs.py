"""vision_verbs helpers: fire-channel parsing + non-blocking mid-hold fire.

These cover the pieces that don't need the full Duburi facade: the pure
_parse_channels parser and the threaded _fire_async (which fires each channel
via self._fire_payload on a daemon thread and skips firing after an abort).
"""

import time

from duburi_control.vision_verbs import _parse_channels, VisionVerbs


class _Log:
    def info(self, *a, **k): pass
    def warning(self, *a, **k): pass
    def error(self, *a, **k): pass
    def debug(self, *a, **k): pass


class _FireHarness(VisionVerbs):
    """Minimal stand-in exposing only what _fire_async touches."""
    def __init__(self, abort=False):
        self.log = _Log()
        self.fired = []
        self._abort = abort

    def _abort_fn(self):
        return self._abort

    def _fire_payload(self, ch):
        self.fired.append(ch)
        return True


def _wait_for(pred, timeout=2.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if pred():
            return True
        time.sleep(0.01)
    return False


# --------------------------------------------------------------------------- #
#  _parse_channels                                                             #
# --------------------------------------------------------------------------- #
def test_parse_channels_basic_and_order():
    assert _parse_channels('1,2') == [1, 2]
    assert _parse_channels('2,1') == [2, 1]      # order preserved (fire order)
    assert _parse_channels('3') == [3]


def test_parse_channels_tolerant_and_clamped():
    assert _parse_channels('') == []
    assert _parse_channels(None) == []
    assert _parse_channels(' 1 , 2 ') == [1, 2]  # whitespace
    assert _parse_channels('1,junk,2') == [1, 2]  # junk dropped
    assert _parse_channels('0,5,1') == [1]        # out-of-range 0/5 dropped


# --------------------------------------------------------------------------- #
#  _fire_async (threaded, non-blocking)                                        #
# --------------------------------------------------------------------------- #
def test_fire_async_fires_all_channels():
    h = _FireHarness()
    h._fire_async([1, 2])
    assert _wait_for(lambda: h.fired == [1, 2]), f'fired={h.fired}'


def test_fire_async_skips_when_aborted():
    h = _FireHarness(abort=True)
    h._fire_async([1, 2])
    # Give the daemon thread a moment; it must skip firing under abort.
    time.sleep(0.1)
    assert h.fired == []


def test_fire_async_is_non_blocking():
    # The call returns immediately even though the fire happens on a thread.
    h = _FireHarness()
    t0 = time.monotonic()
    h._fire_async([1, 2, 3])
    assert (time.monotonic() - t0) < 0.05        # returned without waiting on fires
    assert _wait_for(lambda: h.fired == [1, 2, 3])
