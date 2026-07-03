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


# --------------------------------------------------------------------------- #
#  use_feature safety guard: fwd= must be dropped when use_feature is on       #
#  (the anchor fallback Sample has no bbox size -> fill=0 would drive blind)   #
# --------------------------------------------------------------------------- #
from contextlib import contextmanager
from types import SimpleNamespace

import duburi_control.vision_verbs as vv


class _AlignHarness(VisionVerbs):
    """Stubs the facade so vision_align runs and we can capture align_loop kwargs."""
    def __init__(self):
        self.log = _Log()
        self._captured = {}
        self.pixhawk = None   # passed into align_loop kwargs (call is mocked)
        # a fused state exists so the use_feature branch takes the wrap path
        self.feature_state_provider = lambda cam: SimpleNamespace(name='fused')

    # facade helpers vision_align touches -> no-ops / minimal stand-ins
    @contextmanager
    def _command_scope(self, _name): yield
    def _send_neutral_and_settle(self): pass
    def _resolve_vision_state(self, _cam): return SimpleNamespace(name='vs')
    def _ensure_alt_hold(self, _who): pass
    def _writers(self): return None
    def _abort_fn(self): return False
    def report_vision(self, *_a, **_k): pass
    def _retarget_heading_lock(self, _h): pass
    def _current_heading(self): return 0.0
    def _set_lock_hold(self, _b): pass
    @contextmanager
    def _suspend_heading_lock(self): yield
    def _make_result(self, *_a, **_k): return SimpleNamespace(**_k)


def _run_align(monkeypatch, **kw):
    h = _AlignHarness()
    captured = {}
    def _fake_align_loop(**loop_kw):
        captured.update(loop_kw)
        return SimpleNamespace(reason='ALIGNED', code=0, last_err_px=0.0,
                               end_x_px=0.0, end_y_px=0.0, elapsed_s=0.1)
    monkeypatch.setattr(vv, 'align_loop', _fake_align_loop)
    h.vision_align('forward', 'hole', 'lat,depth', **kw)
    return captured


def test_use_feature_drops_forward_axis(monkeypatch):
    # fwd_fill=30 + use_feature=True -> align_loop must receive fwd_fill 0.0
    cap = _run_align(monkeypatch, fwd_fill=30.0, use_feature=True)
    assert cap['fwd_fill'] == 0.0        # forward axis dropped (no blind drive)


def test_fwd_axis_preserved_without_use_feature(monkeypatch):
    # same fwd_fill WITHOUT use_feature -> passes through (30% -> 0.30 fraction)
    cap = _run_align(monkeypatch, fwd_fill=30.0, use_feature=False)
    assert cap['fwd_fill'] == 0.30       # unchanged path (regression)
