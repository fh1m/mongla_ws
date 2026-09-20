"""vision_verbs helpers: fire-channel parsing + non-blocking mid-hold fire.

These cover the pieces that don't need the full Mongla facade: the pure
_parse_channels parser and the threaded _fire_async (which fires each channel
via self._fire_payload on a daemon thread and skips firing after an abort).
"""

import time

from mongla_control.vision_verbs import _parse_channels, VisionVerbs


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
    """Channels are BOARD channels 1..16, not host-side indices.

    The range was 1-4 while a host-side fire map existed. Left at 1-4 after the map
    was removed, `fire=[9,10]` would parse to `[]` -- the mission would sail past
    the target having fired nothing, with no error anywhere.
    """
    assert _parse_channels('') == []
    assert _parse_channels(None) == []
    assert _parse_channels(' 1 , 2 ') == [1, 2]    # whitespace
    assert _parse_channels('1,junk,2') == [1, 2]   # junk dropped
    assert _parse_channels('9,10') == [9, 10]      # real switch channels survive
    assert _parse_channels('0,16,17') == [16]      # only 1..16 are addressable


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


# --------------------------------------------------------------------------- #
#  fire_gap -- space multi-channel shots (solenoid can't fire two together)    #
# --------------------------------------------------------------------------- #
def test_fire_async_gap_spaces_multiple_channels():
    # Two channels with a 0.3s gap: both fire, and the SECOND lands >= ~0.3s after
    # the first (spaced, not simultaneous).
    h = _FireHarness()
    stamps = {}
    orig = h._fire_payload
    def _timed(ch):
        stamps[ch] = time.monotonic()
        return orig(ch)
    h._fire_payload = _timed
    h._fire_async([1, 4], gap_s=0.3)
    assert _wait_for(lambda: h.fired == [1, 4]), f'fired={h.fired}'
    assert stamps[4] - stamps[1] >= 0.25, 'second shot must be spaced ~gap_s after the first'


def test_fire_async_no_gap_before_single_channel():
    # A single channel is unaffected by gap_s (no leading wait).
    h = _FireHarness()
    t0 = time.monotonic()
    h._fire_async([3], gap_s=1.0)
    assert _wait_for(lambda: h.fired == [3])
    assert (time.monotonic() - t0) < 0.3, 'single-channel fire must not wait a gap'


def test_fire_async_gap_aborts_mid_sequence():
    # An abort during the inter-shot gap cancels the remaining channels.
    h = _FireHarness()
    orig = h._fire_payload
    def _abort_after_first(ch):
        orig(ch)
        h._abort = True          # trip abort right after the first shot
        return True
    h._fire_payload = _abort_after_first
    h._fire_async([1, 2], gap_s=0.5)
    assert _wait_for(lambda: h.fired == [1])     # first fired
    time.sleep(0.7)
    assert h.fired == [1], 'abort during the gap must cancel the 2nd shot'
