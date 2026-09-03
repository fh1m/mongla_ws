"""Two timing constants that were tuned at 20 Hz and stopped meaning what they
said when perception got fast.

The Hailo path runs the detector at 55-98 Hz and the srot control loop at
50 Hz, against the 20 Hz both of these were sized for. Neither raised anything
when the rate changed -- a constant expressed in TICKS or in FRAMES silently
becomes a different physical quantity when the rate underneath it moves, which
is the whole failure mode here.
"""
import pytest

from duburi_control.motion_vision import (            # noqa: E402
    _lock_gate, _ALIGN_STABLE_MIN_S,
    VISION_LOCK_GATE_NORM, VISION_LOCK_GATE_HZ, VISION_LOCK_GATE_MIN,
)


# --------------------------------------------------------------------------- #
#  The continuity lock is a VELOCITY limit -- keyed on the DETECTION interval
# --------------------------------------------------------------------------- #
_TUNED_DT = 1.0 / VISION_LOCK_GATE_HZ


def test_the_design_interval_is_unchanged():
    """The ArduSub path ran a 20 Hz detector and is the configuration that
    placed 8th. Any scaling that moves it is wrong whatever else it fixes."""
    assert _lock_gate(_TUNED_DT) == pytest.approx(VISION_LOCK_GATE_NORM)


def test_faster_DETECTIONS_get_a_tighter_gate():
    """0.30 of the frame between detections is 6.0 frame-widths/s at 20 Hz and
    29.4 at 98 Hz. Holding the number constant let a target move 5x faster
    before the lock let go."""
    assert _lock_gate(1 / 98.0) < _lock_gate(_TUNED_DT)


def test_the_gate_is_the_same_VELOCITY_at_every_interval():
    """The property that makes it a lock rather than a number: allowed jump
    divided by the gap it happened over is constant."""
    ref = _lock_gate(_TUNED_DT) / _TUNED_DT
    for hz in (25.0, 40.0, 50.0):
        assert _lock_gate(1 / hz) / (1 / hz) == pytest.approx(ref)


def test_the_CONTROL_rate_does_not_enter_it():
    """THE DEFECT I SHIPPED AND CORRECTED. The gate governs the jump between
    ACCEPTED CENTRES, and a centre only changes when a new detection lands.
    Keying it on the control period made it 1.7x TIGHTER than tuned at 50 Hz
    control over a 30 Hz camera -- dropping the lock on a real target, which is
    the failure the floor exists to prevent, reintroduced by the fix for the
    opposite one."""
    control_hz, detect_hz = 50.0, 30.0
    assert _lock_gate(1 / detect_hz) > _lock_gate(1 / control_hz)
    assert _lock_gate(1 / detect_hz) == pytest.approx(
        VISION_LOCK_GATE_NORM * (20.0 / 30.0))


def test_it_never_tightens_below_the_detector_s_own_jitter():
    """The mirror failure, and just as bad: a gate tighter than the per-frame
    centre noise drops the lock ON the noise."""
    assert _lock_gate(1 / 500.0) == pytest.approx(VISION_LOCK_GATE_MIN)


def test_a_slower_detector_is_not_LOOSENED_past_the_tuned_value():
    """Scaling up would admit a bigger jump than was ever tuned for. The
    constant is a ceiling, not a midpoint -- and at 3-4 Hz on a raw .pt the
    scaled value would otherwise be most of the frame."""
    assert _lock_gate(1 / 4.0) == pytest.approx(VISION_LOCK_GATE_NORM)


def test_a_nonsense_interval_falls_back_rather_than_dividing_by_zero():
    for bad in (0.0, -1.0, None):
        assert _lock_gate(bad) == pytest.approx(VISION_LOCK_GATE_NORM)


# --------------------------------------------------------------------------- #
#  The stable-frame gate needs a dwell
# --------------------------------------------------------------------------- #
def test_the_dwell_matches_what_three_frames_spanned_at_the_design_rate():
    """Three distinct frames span TWO inter-frame gaps, so 3 frames at 20 Hz is
    0.10 s -- not the 0.15 s the old comment claimed. The floor encodes the
    dwell that was actually being enforced, not the one that was written down.
    """
    frames, design_hz = 3, 20.0
    assert _ALIGN_STABLE_MIN_S == pytest.approx((frames - 1) / design_hz)


def test_the_frame_count_alone_stops_meaning_anything_at_hailo_rates():
    """Not a test of our code -- a test of the ARITHMETIC that motivates the
    floor, so the reason survives in a form that fails if someone edits the
    constant without re-deriving it. Three detections 20 ms apart are nearly one
    moment: the hull cannot settle in it and detector noise is correlated
    across it."""
    span_at_98hz = 2 / 98.0
    assert span_at_98hz < _ALIGN_STABLE_MIN_S / 4


# --------------------------------------------------------------------------- #
#  ...and it has to bite in the loop, not only in arithmetic
# --------------------------------------------------------------------------- #
import os                                                      # noqa: E402
import sys                                                     # noqa: E402
import time                                                    # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from duburi_control.motion_vision import align_loop, ALIGNED    # noqa: E402

# Reuse the engine's own fakes rather than growing a second set: this file is
# about two constants, and a private harness here would drift from the one the
# rest of the engine is tested against.
from types import SimpleNamespace                              # noqa: E402

from test_motion_vision import (                                # noqa: E402
    _FakePixhawk, _FakeVision, _FakeWriters, _Log, _sample)


class _Pix(_FakePixhawk):
    """`name` is what selects the loop rate -- the srot path ticks at 50 Hz,
    which is where three frames stop spanning enough time."""

    def __init__(self, name=''):
        super().__init__()
        self.name = name

    def manual(self, **kw):
        pass


def _run(name):
    """Perfectly on-target, every read a NEW frame (age_s=0) -- what a detector
    faster than the control loop looks like."""
    pix = _Pix(name)
    t0 = time.monotonic()
    res = align_loop(
        pixhawk=pix,
        vision_state=_FakeVision(_sample(ex=0.0, ey=0.0, w_frac=0.3, h_frac=0.3)),
        target_class='gate', axes={'lat'}, offsets={}, err_px=40.0,
        duration=3.0, gain=30.0, align_stable_frames=3, lost_grace_s=0.5,
        brake=False, writers=_FakeWriters(), log=_Log(), abort_fn=None)
    return res, time.monotonic() - t0


def test_a_perfect_target_still_aligns_on_both_backends():
    """The floor must not turn a good lock into a TIMEOUT."""
    for name in ('', 'srot'):
        res, _ = _run(name)
        assert res.code == ALIGNED, name


def test_the_50Hz_path_cannot_declare_aligned_in_three_ticks():
    """THE POINT. At 50 Hz three ticks span 0.04 s; the dwell three frames
    bought at 20 Hz is 0.10 s. Without the floor this exits in 60 ms on
    evidence spanning 40 ms -- and the same counter arms the mid-hold fire."""
    _, elapsed = _run('srot')
    assert elapsed >= _ALIGN_STABLE_MIN_S, (
        f'aligned in {elapsed*1000:.0f} ms, less than the '
        f'{_ALIGN_STABLE_MIN_S*1000:.0f} ms of evidence three frames bought at '
        f'the rate this was tuned at')


def test_the_20Hz_path_is_not_made_slower():
    """Three ticks at 20 Hz already span 0.10 s, so the floor is satisfied on
    the same tick the third frame lands. It must not cost an extra one."""
    _, elapsed = _run('')
    assert elapsed < _ALIGN_STABLE_MIN_S * 2.0, (
        f'{elapsed*1000:.0f} ms -- the floor should bind at 20 Hz, not add a tick')


class _GateSpy(_FakeVision):
    """Records the gate the loop actually asked for.

    Needed because the unit tests above prove `_lock_gate` computes the right
    number and say nothing about whether the loop USES it -- and reverting the
    call site to the bare constant left every existing lock test green, since
    they all run at 20 Hz where the two values coincide.
    """

    def bbox_error(self, *a, **kw):
        self.gates = getattr(self, 'gates', [])
        self.gates.append(kw.get('gate_norm'))
        return super().bbox_error(*a, **kw)


def _gate_used(name):
    v = _GateSpy(_sample(ex=0.0, ey=0.0, w_frac=0.3, h_frac=0.3))
    align_loop(pixhawk=_Pix(name), vision_state=v, target_class='gate',
               axes={'lat'}, offsets={}, err_px=40.0, duration=0.4, gain=30.0,
               align_stable_frames=3, lost_grace_s=0.5, lock_on=True,
               brake=False, writers=_FakeWriters(), log=_Log(), abort_fn=None)
    return v.gates[0]


def test_the_loop_STARTS_at_the_control_period_then_widens():
    """Before any interval has been observed there is nothing to key on, so it
    starts at the shortest gap possible -- tight. Erring tight pre-acquisition
    is safe: there is no lock to drop yet."""
    for name, hz in (('', 20.0), ('srot', 50.0)):
        assert _gate_used(name) == pytest.approx(_lock_gate(1.0 / hz))


class _SlowCamera(_GateSpy):
    """A detector SLOWER than the control loop, which is the whole case.

    An `age_s=0` fake makes every control tick a new frame, so the observed
    detection interval equals the control period and the two clocks are
    indistinguishable -- which is exactly why my first version of this test
    could not tell the corrected code from the bug. Here a frame lands every
    `every` ticks and `age_s` reports how long ago it did, the way the real
    `bbox_error` does.
    """

    def __init__(self, sample, every=2):
        super().__init__(sample)
        self.gates = []
        self.every = every
        self.n = -1

    def bbox_error(self, *a, **kw):
        self.gates.append(kw.get('gate_norm'))
        self.n += 1
        dt = 1.0 / 50.0                       # the srot control period
        age = (self.n % self.every) * dt      # 0 on a new frame, then ageing
        s = self._samples if not isinstance(self._samples, list) else self._samples[0]
        return SimpleNamespace(ex=s.ex, ey=s.ey, w_frac=s.w_frac, h_frac=s.h_frac,
                               age_s=age, track_id=1, coasted=False)


def test_the_gate_WIDENS_to_the_observed_detection_interval():
    """A 25 Hz camera under a 50 Hz loop must get the 25 Hz gate, not the 50 Hz
    one -- twice as wide. Keying it on the control period is what made it 1.7x
    too tight on the vehicle."""
    v = _SlowCamera(_sample(ex=0.0, ey=0.0, w_frac=0.3, h_frac=0.3), every=2)
    align_loop(pixhawk=_Pix('srot'), vision_state=v, target_class='gate',
               axes={'lat'}, offsets={}, err_px=40.0, duration=0.6, gain=30.0,
               align_stable_frames=3, lost_grace_s=0.5, lock_on=True,
               brake=False, writers=_FakeWriters(), log=_Log(), abort_fn=None)
    seen = [g for g in v.gates if g is not None]
    assert seen[-1] == pytest.approx(_lock_gate(2.0 / 50.0), rel=0.2), (
        f'ended at {seen[-1]:.4f}; the 25 Hz gate is '
        f'{_lock_gate(2.0/50.0):.4f} and the 50 Hz one '
        f'{_lock_gate(1.0/50.0):.4f}')
    assert seen[-1] > seen[0], 'the gate never widened off its seed'
