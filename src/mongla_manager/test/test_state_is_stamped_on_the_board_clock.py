"""`/mongla/state` carries board measurements; it must not carry host jitter.

Yaw and depth ORIGINATE ON THE BOARD and were stamped with the instant the host
got round to publishing. Measured on this vehicle at 50 Hz: the board's own
ATTITUDE interval is 20.00 ms with **sd 0.00**, and the host's arrival interval
is 20.00 ms with **sd 6.67, p2p 35.12** -- every bit of that is transport (UART
FIFO thresholding plus USB-serial scheduling). So the stamp described the
transport, not the measurement.

It matters because `flow_node` reads depth from this topic and DIFFERENCES it
over time (`_vz_down`), turning that jitter into a vertical speed that is not
happening. `ClockMap` was already fitted in this node for `/mongla/imu_rates`;
this is the same mapping applied to the other stream that needs it.

The verdict path is exercised directly -- a live graph would test rclpy.
"""
import sys
import time
import types
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from mongla_manager.auv_manager_node import AUVManagerNode   # noqa: E402


class _Stamp:
    def __init__(self): self.sec, self.nanosec = 0, 0
    def to_msg(self): return self


class _Clock:
    """Host clock, deliberately far from the board's, so the two are telling."""
    HOST_SEC = 7_000_000

    def now(self):
        s = _Stamp()
        s.sec, s.nanosec = self.HOST_SEC, 0
        return s


class _Fake:
    _BOARD_STAMP_MAX_AGE_S = AUVManagerNode._BOARD_STAMP_MAX_AGE_S

    def __init__(self, board_stamp=None):
        self._board_stamp = board_stamp
        self._clock = _Clock()

    def get_clock(self):
        return self._clock


def _stamp(fake):
    return AUVManagerNode._state_stamp(fake)


def test_a_fresh_board_sample_is_the_stamp():
    board_s = 1234.5
    st = _stamp(_Fake((board_s, time.monotonic())))
    assert st.sec == 1234
    assert abs(st.nanosec - 0.5e9) < 1e6
    assert st.sec != _Clock.HOST_SEC, 'fell back to the host clock'


def test_an_unmapped_clock_falls_back_to_host_time():
    """Before ClockMap converges there IS no board stamp. Falling back is
    correct; inventing one would be worse than the jitter."""
    assert _stamp(_Fake(None)).sec == _Clock.HOST_SEC


def test_a_STALE_board_sample_is_refused():
    """The board sample must belong to the state being published. Stamping a
    fresh message with a capture time from a second ago would be a confident
    lie -- worse than the arrival time it replaced, because it looks correct.
    """
    old = time.monotonic() - (AUVManagerNode._BOARD_STAMP_MAX_AGE_S + 0.05)
    assert _stamp(_Fake((1234.5, old))).sec == _Clock.HOST_SEC


def test_the_freshness_bound_covers_several_board_periods():
    """ATTITUDE is pinned at 50 Hz, so 0.2 s is 10 periods -- loose enough that
    one scheduling hiccup does not flip the stamp source back and forth."""
    assert 0.1 <= AUVManagerNode._BOARD_STAMP_MAX_AGE_S <= 0.5


def test_both_state_publishers_use_the_helper():
    """There are TWO publishers of MonglaState (the fast path and the telemetry
    path). One left on the host clock would make the topic's time base depend
    on which code path produced the message."""
    src = (Path(__file__).resolve().parents[1] / 'mongla_manager'
           / 'auv_manager_node.py').read_text()
    assert src.count('msg.header.stamp    = self._state_stamp()') == 2
    assert 'msg.header.stamp    = self.get_clock().now().to_msg()' not in src


def test_the_board_stamp_is_cached_ONLY_when_the_mapping_SUCCEEDED():
    """The helper is only as good as its source.

    `_imu_rates_tick` falls back to arrival time when `ClockMap` is not ready
    yet. Caching `stamp_s` after that fallback would store an ARRIVAL time
    under the name `_board_stamp`, and `/mongla/state` would then carry the
    jitter with a label claiming it had been removed -- the same lie in a
    better disguise, and strictly worse than the honest host stamp because
    nothing would look wrong.

    So the cache must sit INSIDE the `self._imu_clock.ready` branch, before
    the fallback.
    """
    src = (Path(__file__).resolve().parents[1] / 'mongla_manager'
           / 'auv_manager_node.py').read_text()
    assert 'self._board_stamp = (stamp_s, time.monotonic())' in src
    cache = src.index('self._board_stamp = (stamp_s, time.monotonic())')
    ready = src.index('if self._imu_clock.ready:')
    fallback = src.index('if stamp_s is None:')
    assert ready < cache < fallback, (
        'the board stamp is cached outside the mapped branch, so an arrival '
        'time can be stored and published as a board capture time')


# --------------------------------------------------------------------------
#  The consumer half. Kept here on purpose: a producer that stamps honestly
#  and a consumer that ignores the stamp is the same defect with an extra step,
#  and splitting the two halves across files is how one of them gets reverted
#  alone. Injection-checked: reverting `flow_node` to `time.monotonic()` was
#  caught by nothing at all before this.
# --------------------------------------------------------------------------

def test_flow_node_reads_the_depth_CAPTURE_stamp_not_arrival():
    """`stamps.py` records four prior cases of "the clock was read at the wrong
    place"; `flow_position` integrating on arrival time was the fifth. The
    depth series in `flow_node` was the sixth, inside the package that owns the
    fix -- and `_vz_down` differences those instants, so the transport jitter
    became a vertical speed that is not happening.
    """
    fn = (Path(__file__).resolve().parents[2] / 'mongla_vision'
          / 'mongla_vision' / 'flow' / 'flow_node.py').read_text()
    assert 'from mongla_vision.stamps import capture_monotonic' in fn, (
        'flow_node does not import capture_monotonic')

    i = fn.index('def _on_state')
    body = fn[i:i + 1800]
    assert 'capture_monotonic(msg.header)' in body, (
        '_on_state does not read the message stamp')
    assert 'self._depth_hist.append((t, self._depth_m))' in body, (
        'the depth history is not built from the capture stamp -- check for a '
        'time.monotonic() that crept back in')
    assert 'time.monotonic()' not in body.split('_depth_hist.append')[0], (
        'arrival time is still being taken in _on_state')


def test_the_fallback_to_arrival_time_is_LOUD():
    """An unstamped publisher is survivable; a SILENT fallback is not -- the
    velocity would simply be wrong with nothing to explain it."""
    fn = (Path(__file__).resolve().parents[2] / 'mongla_vision'
          / 'mongla_vision' / 'flow' / 'flow_node.py').read_text()
    i = fn.index('def _on_state')
    body = fn[i:i + 1800]
    assert '_state_stamp_warned' in body and 'warning(' in body, (
        'the arrival-time fallback in _on_state is silent')
