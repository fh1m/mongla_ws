"""The coast ladder's 4th rung, and the rate every rung is sized from.

`max_predict` was expressed in FRAMES for a window that only matters in
SECONDS. 30 frames is 1.5 s at the 20 Hz it was tuned at and 0.31 s at the
Hailo path's 98 Hz -- under a typical `vision.coast_s` of 0.8 s, so a coasted
target vanished off /tracks before the coast elapsed. Nothing raised anything,
because a tracker with a truncated coast still publishes and looks healthy.

The asymmetry that hid it: Roboflow's own buffer IS frame-rate aware
(`int(frame_rate/30 * lost_track_buffer)`, read out of the installed library),
so the config's frame_rate scaled one rung and not the other.
"""
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


# --------------------------------------------------------------------------- #
#  Seconds -> frames
# --------------------------------------------------------------------------- #
def _frames(max_predict_s, frame_rate):
    """The node's conversion, isolated. Kept in step by the test below that
    asserts the source still computes it this way."""
    return max(1, math.ceil(max_predict_s * frame_rate))


@pytest.mark.parametrize('hz,expect_s', [(20, 1.5), (55, 1.5), (98, 1.5)])
def test_the_coast_is_the_same_WALL_TIME_at_every_rate(hz, expect_s):
    """The property the old constant did not have."""
    assert _frames(1.5, hz) / hz == pytest.approx(expect_s, abs=0.02)


def test_the_old_constant_would_have_truncated_at_hailo_rates():
    """Not a test of our code -- a test of the arithmetic that motivates the
    change, so the reason fails loudly if someone reverts to frames."""
    typical_coast_s = 0.8
    assert 30 / 20.0 > typical_coast_s        # fine where it was tuned
    assert 30 / 98.0 < typical_coast_s        # broken where it now runs


def test_a_sub_frame_remainder_rounds_UP():
    """Rounding a coast window DOWN is the failure being fixed."""
    assert _frames(1.5, 21) == 32             # 31.5 -> 32, not 31


def test_a_tiny_window_still_gets_one_frame():
    """0 frames would suppress a track the instant a detection is missed --
    no coast at all, which is worse than a short one."""
    assert _frames(0.001, 1.0) == 1


def test_the_node_declares_the_window_in_SECONDS():
    """The parameter is the contract. Asserted on the declaration rather than
    on prose: my first version of this test grepped for the absence of the word
    `max_predict_frames`, which is IN the comment that explains the change --
    the same shape as the `_srot_drive` grep that stayed green through exactly
    the edit it guarded."""
    src = (Path(__file__).resolve().parents[1] / 'mongla_vision'
           / 'tracker_node.py').read_text()
    assert "declare_parameter('max_predict_s'" in src
    assert "declare_parameter('max_predict_frames'" not in src
    assert 'math.ceil(max_pred_s * frame_rate)' in src


# --------------------------------------------------------------------------- #
#  The measured-rate check
# --------------------------------------------------------------------------- #
def test_the_warmup_is_long_enough_to_clear_the_contended_start():
    """The first live run reported a 55 Hz camera arriving at 15 -- and it WAS,
    for the first two seconds, while both detectors still contended for the
    chip's activation. Measuring from t=0 described the startup, not the run.
    The warm-up must outlast that transient at the SLOWEST rate we run."""
    from mongla_vision.tracker_node import _RATE_WARMUP, _RATE_SAMPLES
    slowest_hz = 15.0
    transient_s = 2.0
    assert _RATE_WARMUP / slowest_hz > transient_s
    # ...and the measurement window after it must still be short enough to
    # reach the operator while they are watching the launch.
    assert (_RATE_WARMUP + _RATE_SAMPLES) / slowest_hz < 30.0


def test_the_rate_is_a_MEDIAN_gap_not_a_span_average():
    """One long gap -- a detector pause, a model switch, a scheduler hiccup --
    drags a span-based mean far more than it shifts the rate the coast windows
    actually see."""
    src = (Path(__file__).resolve().parents[1] / 'mongla_vision'
           / 'tracker_node.py').read_text()
    assert 'gaps[len(gaps) // 2]' in src
    assert '(_RATE_SAMPLES - 1) / span' not in src


# --------------------------------------------------------------------------- #
#  ...and the check has to RUN, not just have well-sized constants
# --------------------------------------------------------------------------- #
class _Stub:
    """The attributes `_check_rate` touches, and nothing else.

    The method is called unbound against this rather than constructing a real
    node, because building one needs rclpy, a live DDS graph and a camera --
    none of which this is about. What is exercised is the real method body.
    """

    def __init__(self, configured_hz, tol=1.5, kal_frames=82):
        self._rate_t = []
        self._rate_seen = 0
        self._rate_warned = False
        self._frame_rate = configured_hz
        self._rate_tol = tol
        self._kal_frames = kal_frames
        self.warnings = []

    def get_logger(self):
        return type('L', (), {
            'warn':    lambda _s, m: self.warnings.append(m),
            'warning': lambda _s, m: self.warnings.append(m)})()

    @property
    def mismatches(self):
        """Only the lines that say the config is WRONG.

        The node reports the measured rate either way, deliberately: silence is
        indistinguishable from the check not running, and on the vehicle I
        could not tell those apart. So a test that asserts `warnings == []`
        would now be asserting the check is broken -- these assert on the
        mismatch text instead.
        """
        return [w for w in self.warnings if 'but frame_rate is' in w]


def _feed(stub, hz, n, t0=0.0):
    """n detections arriving at a steady `hz`."""
    from mongla_vision.tracker_node import TrackerNode
    dt = 1.0 / hz
    for i in range(n):
        TrackerNode._check_rate(stub, t0 + i * dt)
    return t0 + n * dt


def test_a_correct_frame_rate_is_quiet():
    from mongla_vision.tracker_node import _RATE_WARMUP, _RATE_SAMPLES
    s = _Stub(55.0)
    _feed(s, 55.0, _RATE_WARMUP + _RATE_SAMPLES + 5)
    assert s.mismatches == []
    assert len(s.warnings) == 1   # ...but it DID report


def test_a_wrong_frame_rate_is_named_with_the_real_coast():
    from mongla_vision.tracker_node import _RATE_WARMUP, _RATE_SAMPLES
    s = _Stub(55.0, kal_frames=82)          # 1.5 s at the configured 55 Hz
    _feed(s, 15.0, _RATE_WARMUP + _RATE_SAMPLES + 5)
    assert len(s.mismatches) == 1
    w = s.mismatches[0]
    assert '15 Hz' in w and 'frame_rate:=15' in w
    assert '5.47s' in w or '5.4' in w       # 82 frames at a real 15 Hz


def test_the_startup_transient_does_not_decide_it():
    """THE LIVE DEFECT. The first run on the Pi warned that a 55 Hz camera was
    arriving at 15 -- and it was, for the first two seconds, while both
    detectors still contended for the chip. Without the warm-up, a burst of
    slow frames at t=0 is the whole measurement."""
    from mongla_vision.tracker_node import _RATE_WARMUP, _RATE_SAMPLES
    s = _Stub(55.0)
    t = _feed(s, 15.0, _RATE_WARMUP)        # the contended start
    _feed(s, 55.0, _RATE_SAMPLES + 5, t0=t)  # then the real rate
    assert s.mismatches == [], (
        'the startup transient was measured instead of the run: ' + str(s.warnings))


def test_it_warns_only_ONCE():
    """A rate mismatch is a configuration fact, not an event. Repeating it
    every window buries the log it is competing with."""
    from mongla_vision.tracker_node import _RATE_WARMUP, _RATE_SAMPLES
    s = _Stub(55.0)
    _feed(s, 15.0, (_RATE_WARMUP + _RATE_SAMPLES) * 4)
    assert len(s.warnings) == 1


def test_one_long_gap_does_not_move_the_verdict():
    """A detector pause or a model switch is not a rate change. A span-based
    mean would call a correct configuration wrong."""
    from mongla_vision.tracker_node import _RATE_WARMUP, _RATE_SAMPLES
    from mongla_vision.tracker_node import TrackerNode
    s = _Stub(55.0)
    t = _feed(s, 55.0, _RATE_WARMUP)
    TrackerNode._check_rate(s, t + 5.0)     # a five-second stall
    _feed(s, 55.0, _RATE_SAMPLES + 5, t0=t + 5.0)
    assert s.mismatches == []


def test_the_measured_rate_is_reported_even_when_it_is_RIGHT():
    """Silence is indistinguishable from the check not running. On the vehicle
    no warning appeared at a rate I had measured as out of band -- and the
    check was fine; `ros2 topic hz`'s own subscriber load was depressing my
    reading. One line either way settles that in the log instead of in a
    debugging session."""
    from mongla_vision.tracker_node import _RATE_WARMUP, _RATE_SAMPLES
    s = _Stub(55.0)
    _feed(s, 55.0, _RATE_WARMUP + _RATE_SAMPLES + 5)
    assert len(s.warnings) == 1
    assert s.mismatches == []
    assert '55 Hz' in s.warnings[0] and 'coast' in s.warnings[0]
