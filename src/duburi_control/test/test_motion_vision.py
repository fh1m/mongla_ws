"""Behaviour tests for the two-verb vision control engine.

`motion_vision.align_loop` / `move_loop` are the most competition-relevant
control code, so the pixel-error convergence, the `gain`-as-max-speed clamp,
the fill-ratio exit per `mode`, the `maintain` lateral hold, and the
loss -> grace -> LOST path are pinned here with lightweight fakes. No ROS /
MAVLink needed -- only the pure `Pixhawk.percent_to_pwm` static is used.
"""

import time
from types import SimpleNamespace

import pytest

from duburi_control.motion_vision import (
    align_loop, move_loop, _fill, _clamp, _present, _freshness, _range_gain,
    _coast_authority, _authority,
    VISION_FRESH_FULL_S, VISION_FRESH_ZERO_S, VISION_FRESH_ZERO_MAX_S,
    VISION_RANGE_GAIN_FILL_LO, VISION_RANGE_GAIN_FILL_HI, VISION_LOCK_GATE_NORM,
    FWD_BAND,
    ALIGNED, LOST, TIMEOUT, NO_CAMERA,
)


# --------------------------------------------------------------------------- #
#  Fakes                                                                       #
# --------------------------------------------------------------------------- #
def _sample(ex=0.0, ey=0.0, w_frac=0.0, h_frac=0.0, age_s=0.0,
            track_id=-1, coasted=False):
    return SimpleNamespace(ex=ex, ey=ey, w_frac=w_frac, h_frac=h_frac, age_s=age_s,
                           track_id=track_id, coasted=coasted)


class _FakeVision:
    """image_size() + info_seen() + list_classes() + bbox_error() stand-in.

    `samples` may be a single sample/None (returned every tick) or a list
    consumed one-per-call (last value repeats once exhausted).

    `info_seen` defaults to "a real size has arrived" so size=(0,0) reports
    the camera as not-up (matching the real VisionState); pass it explicitly
    to exercise the "size known but CameraInfo not yet seen" path.
    `classes` feeds the no-match diagnostic (boxes present, none our class).
    """
    def __init__(self, samples, size=(640, 480), info_seen=None, classes=None):
        self._size = size
        self._samples = samples
        self._i = 0
        self._info_seen = (
            (size[0] > 0 and size[1] > 0) if info_seen is None else info_seen)
        self._classes = list(classes or [])

    def image_size(self):
        return self._size

    def info_seen(self):
        return self._info_seen

    def list_classes(self):
        return list(self._classes)

    def bbox_error(self, _cls, **_kw):
        # accepts the continuity-lock kwargs (near/gate_norm/min_score) the loop
        # now passes; this double just replays its scripted samples.
        self.last_kw = _kw
        if isinstance(self._samples, list):
            s = self._samples[min(self._i, len(self._samples) - 1)]
            self._i += 1
            return s
        return self._samples


class _FakePixhawk:
    def __init__(self):
        self.rc = []          # recorded send_rc_override kwargs
        self.translations = []
        self.depths = []

    def get_attitude(self):
        return {'depth': -0.5}

    def send_rc_override(self, **kw):
        self.rc.append(kw)

    def send_rc_translation(self, **kw):
        self.translations.append(kw)

    def set_target_depth(self, d):
        self.depths.append(d)

    @staticmethod
    def percent_to_pwm(pct):
        return max(1100, min(1900, int(1500 + (pct / 100.0) * 400)))


class _FakeWriters:
    """Stand-in for the Writers bundle.

    The loops drive translation via `pixhawk` directly; only the arrival
    brake (`_brake_axis`) writes through `writers.forward`/`.lateral`, so
    these lists capture exactly the brake kicks (empty when braking is
    gated out / disabled).
    """
    def __init__(self):
        self.neutralised = 0
        self.forwards = []     # brake-kick PWMs on the forward axis
        self.laterals = []     # brake-kick PWMs on the lateral axis

    def forward(self, pwm):
        self.forwards.append(pwm)

    def lateral(self, pwm):
        self.laterals.append(pwm)

    def neutral(self):
        self.neutralised += 1


class _Log:
    def info(self, *_a, **_k): pass
    def warning(self, *_a, **_k): pass
    def error(self, *_a, **_k): pass
    def debug(self, *_a, **_k): pass


class _CapLog:
    """Capturing logger for assertions on the no-match diagnostic."""
    def __init__(self):
        self.warnings = []
        self.infos = []
        self.debugs = []

    def info(self, msg, *_a, **_k): self.infos.append(str(msg))
    def warning(self, msg, *_a, **_k): self.warnings.append(str(msg))
    def error(self, *_a, **_k): pass
    def debug(self, msg, *_a, **_k): self.debugs.append(str(msg))


def _align(vision, pix=None, **kw):
    pix = pix or _FakePixhawk()
    writers = _FakeWriters()
    defaults = dict(
        pixhawk=pix, vision_state=vision, target_class='gate',
        axes={'yaw', 'lat'}, offsets={}, err_px=40.0, duration=0.4, gain=30.0,
        align_stable_frames=3, lost_grace_s=0.1,
        writers=writers, log=_Log(), abort_fn=None)
    defaults.update(kw)
    return align_loop(**defaults), pix, writers


def _move(vision, pix=None, **kw):
    pix = pix or _FakePixhawk()
    writers = _FakeWriters()
    defaults = dict(
        pixhawk=pix, vision_state=vision, target_class='gate',
        fwd_fill=0.8, mode='area', duration=0.4, gain=30.0,
        lost_grace_s=0.1, writers=writers, log=_Log(), abort_fn=None)
    defaults.update(kw)
    return move_loop(**defaults), pix, writers


# --------------------------------------------------------------------------- #
#  Pure helpers                                                                #
# --------------------------------------------------------------------------- #
def test_fill_modes():
    s = _sample(w_frac=0.4, h_frac=0.9)
    assert _fill(s, 'width') == pytest.approx(0.4)
    assert _fill(s, 'height') == pytest.approx(0.9)
    assert _fill(s, 'area') == pytest.approx((0.4 * 0.9) ** 0.5)


def test_clamp_bounds():
    assert _clamp(5.0, -3.0, 3.0) == 3.0
    assert _clamp(-5.0, -3.0, 3.0) == -3.0
    assert _clamp(1.0, -3.0, 3.0) == 1.0


def test_present_rejects_stale_and_none():
    assert _present(_sample(age_s=0.2)) is True
    assert _present(_sample(age_s=5.0)) is False
    assert _present(None) is False


# --------------------------------------------------------------------------- #
#  align_loop                                                                  #
# --------------------------------------------------------------------------- #
def test_align_no_camera_when_size_zero():
    out, _, _ = _align(_FakeVision(_sample(), size=(0, 0)))
    assert out.code == NO_CAMERA


def test_align_no_camera_until_info_seen():
    # Size is known but CameraInfo has not arrived -> the pixel scale is not
    # trustworthy yet, so the verb reports NO_CAMERA instead of steering.
    out, _, _ = _align(_FakeVision(_sample(), size=(640, 480), info_seen=False))
    assert out.code == NO_CAMERA


def test_align_never_detected_message_on_timeout():
    # Target never appears + hold_through_loss keeps the loop alive to the
    # deadline -> TIMEOUT with the distinct "NEVER detected" reason (the pool
    # failure: wrong model/classes meant 'rescue' was never produced).
    out, _, _ = _align(_FakeVision(None), hold_through_loss=True)
    assert out.code == TIMEOUT
    assert 'NEVER detected' in out.reason


def test_align_never_detected_message_on_lost():
    # Same never-seen condition without hold -> LOST after grace, but the
    # reason says NEVER detected (not the misleading "lost", which implies
    # it was there and went away).
    out, _, _ = _align(_FakeVision(None))
    assert out.code == LOST
    assert 'NEVER detected' in out.reason


def test_align_seen_then_lost_says_lost_not_never():
    # Present once, then gone -> the honest "lost" wording, NOT "NEVER".
    samples = [_sample(ex=0.0, ey=0.0), None, None, None, None]
    out, _, _ = _align(_FakeVision(samples), align_stable_frames=99)
    assert out.code == LOST
    assert 'lost' in out.reason and 'NEVER' not in out.reason


def test_align_err_below_floor_still_completes():
    # An over-tight positive err must be clamped to MIN_ALIGN_ERR_PX so the loop
    # can converge on bbox-jitter-sized residual instead of perpetually TIMEOUTing.
    # ex=0.01 on a 640px frame -> ~3.2px residual: above err=1 (would never lock)
    # but inside the 5px floor -> ALIGNED.
    out, _, _ = _align(_FakeVision(_sample(ex=0.01)), axes={'lat'},
                       kp_lat=60.0, gain=30.0, err_px=1.0, duration=1.0)
    assert out.code == ALIGNED, (
        f'err below MIN_ALIGN_ERR_PX must be floored so a ~3px residual locks; '
        f'got {out.code} ({out.reason})')
    assert '/5px' in out.reason, (
        f'success line must state the effective deadband, got {out.reason!r}')


def test_align_err_above_floor_is_honored():
    # A tight-but-achievable err (8px) is NOT floored: a 3px residual locks and
    # the reason states the real 8px deadband.
    out, _, _ = _align(_FakeVision(_sample(ex=0.01)), axes={'lat'},
                       kp_lat=60.0, gain=30.0, err_px=8.0, duration=1.0)
    assert out.code == ALIGNED
    assert '/8px' in out.reason, f'expected the honored 8px deadband, got {out.reason!r}'


def test_align_release_yaw_writes_translation_not_ch4():
    # When the heading lock owns Ch4, a lat-only align must drive lateral via
    # send_rc_translation and never write Ch4 through send_rc_override.
    pix = _FakePixhawk()
    out, _, _ = _align(_FakeVision(_sample(ex=1.0)), pix=pix,
                       axes={'lat'}, kp_lat=60.0, gain=30.0,
                       release_yaw=True, duration=0.25)
    assert out.code == TIMEOUT
    assert pix.translations, 'expected send_rc_translation writes'
    assert not pix.rc, 'must not write Ch4 via send_rc_override under release_yaw'
    cap = _FakePixhawk.percent_to_pwm(30.0)
    lateral = [t['lateral'] for t in pix.translations
               if t.get('lateral', 1500) != 1500]
    assert lateral and max(lateral) <= cap


def test_align_warns_when_boxes_present_but_none_match():
    # The detector IS publishing boxes but none are our class -> the classic
    # "HUD shows a box, AUV won't move" trap. A throttled warning must fire.
    log = _CapLog()
    vis = _FakeVision(None, classes=['flare', 'rescue'])
    align_loop(pixhawk=_FakePixhawk(), vision_state=vis, target_class='gate',
               axes={'lat'}, offsets={}, err_px=40.0, duration=0.3, gain=30.0,
               lost_grace_s=2.0, writers=_FakeWriters(), log=log, abort_fn=None)
    # Emitted at debug so it doesn't pollute the operator view, but the
    # "why isn't it moving" hint still fires at --log-level debug.
    assert any('not among live detections' in d for d in log.debugs)


def test_align_converges_when_centered():
    # Centered target (ex=ey=0) is within err every tick -> ALIGNED after
    # align_stable_frames, well inside the duration budget.
    out, _, writers = _align(_FakeVision(_sample(ex=0.0, ey=0.0)))
    assert out.code == ALIGNED
    assert out.last_err_px <= 40.0
    assert writers.neutralised >= 1


def test_align_hold_zero_exits_on_first_stable():
    # hold_s=0 (default) keeps the original behaviour: exit the instant
    # alignment is confirmed (align_stable_frames ticks), well inside budget.
    out, _, _ = _align(_FakeVision(_sample(ex=0.1)), hold_s=0.0, duration=2.0)
    assert out.code == ALIGNED
    assert 'aligned' in out.reason
    assert out.elapsed_s < 0.5          # ~3 ticks @20Hz, nowhere near 2s


def test_align_hold_keeps_correcting_then_exits_aligned():
    # hold_s>0 is an ACTIVE station-keep: once centred it KEEPS issuing
    # corrections across the hold window (never goes neutral -- the whole
    # point), then exits ALIGNED in-band. ex=0.1 is inside err (32px<=40)
    # so it counts as stable, yet lat has no deadband so it still commands a
    # non-zero lateral correction every tick -> proof the loop stays active.
    hold_s = 0.3
    out, pix, _ = _align(_FakeVision(_sample(ex=0.1)), hold_s=hold_s, duration=2.0)
    assert out.code == ALIGNED
    assert 'held' in out.reason
    assert out.elapsed_s >= hold_s      # held the window, not exit-on-stable
    lat_cmds = [c['lateral'] for c in pix.rc if c.get('lateral', 1500) != 1500]
    assert lat_cmds, 'expected active lateral corrections DURING the hold'
    # Materially longer than the hold_s=0 exit-on-stable path on the same target.
    base, _, _ = _align(_FakeVision(_sample(ex=0.1)), hold_s=0.0, duration=2.0)
    assert out.elapsed_s > base.elapsed_s + hold_s * 0.5


def test_align_fires_on_locked_once_mid_hold():
    # on_locked fires EXACTLY once, mid-hold, while the loop keeps correcting.
    calls = []
    out, pix, _ = _align(_FakeVision(_sample(ex=0.1)), hold_s=0.3, duration=2.0,
                         on_locked=lambda: calls.append(1), fire_t=0.0)
    assert out.code == ALIGNED
    assert len(calls) == 1               # fired exactly once, not every tick
    # The loop kept issuing lateral corrections (fire didn't end the hold).
    lat_cmds = [c['lateral'] for c in pix.rc if c.get('lateral', 1500) != 1500]
    assert lat_cmds, 'expected corrections to continue after the fire'


def test_align_fire_t_delays_until_into_hold():
    # With fire_t close to the full hold, the fire still happens (before exit),
    # but not on the very first held tick.
    calls = []
    _align(_FakeVision(_sample(ex=0.1)), hold_s=0.4, duration=2.0,
           on_locked=lambda: calls.append(1), fire_t=0.2)
    assert calls == [1]                  # fired once, inside the hold window


def test_align_no_fire_without_on_locked():
    # No on_locked -> the loop behaves exactly as before (no fire path).
    out, _, _ = _align(_FakeVision(_sample(ex=0.1)), hold_s=0.2, duration=2.0)
    assert out.code == ALIGNED


def test_align_on_locked_exception_does_not_kill_loop():
    # A throwing on_locked must not abort the hold -- the verb still returns
    # ALIGNED (a payload glitch can't crash the control loop).
    def boom():
        raise RuntimeError('serial blew up')
    out, _, _ = _align(_FakeVision(_sample(ex=0.1)), hold_s=0.2, duration=2.0,
                       on_locked=boom, fire_t=0.0)
    assert out.code == ALIGNED


def test_align_does_not_fire_when_never_aligned():
    # The fire is GATED on alignment, not pure time: a target that is DETECTED
    # but never centred (out of band) never opens the hold/fire gate, so the
    # torpedo is NEVER fired off-target. ex=5.0 -> ~1600px, far outside err=40.
    calls = []
    out, _, _ = _align(_FakeVision(_sample(ex=5.0)), hold_s=0.3, duration=0.4,
                       on_locked=lambda: calls.append(1), fire_t=0.0)
    assert calls == []                   # never aligned -> never fired
    assert out.code != ALIGNED           # and it did not falsely report aligned


def test_align_does_not_fire_when_target_absent():
    # No detection at all -> never aligned -> never fired (LOST/TIMEOUT, not a shot).
    calls = []
    _align(_FakeVision(None), hold_s=0.3, duration=0.4,
           on_locked=lambda: calls.append(1), fire_t=0.0)
    assert calls == []


# --------------------------------------------------------------------------- #
#  rich end-state: signed (x,y) where the verb ended + live report_fn          #
# --------------------------------------------------------------------------- #
def test_align_returns_signed_end_position():
    # ex=0.2 -> 64px right (>err=40 -> TIMEOUT, but the END position is carried).
    # half_w=320, half_h=240 for the default 640x480.
    out, _, _ = _align(_FakeVision(_sample(ex=0.2, ey=-0.1)), duration=0.3)
    assert out.code == TIMEOUT
    assert out.end_x_px == pytest.approx(64.0, abs=1.0)    # signed +, target right
    assert out.end_y_px == pytest.approx(-24.0, abs=1.0)   # signed -, target above


def test_align_end_position_nan_when_never_seen():
    import math
    out, _, _ = _align(_FakeVision(None), duration=0.2)
    assert math.isnan(out.end_x_px) and math.isnan(out.end_y_px)


def test_align_report_fn_streams_signed_offsets():
    calls = []
    _align(_FakeVision(_sample(ex=0.2, ey=-0.1)), duration=0.2,
           report_fn=lambda x, y: calls.append((x, y)))
    assert calls, 'report_fn should fire every present tick'
    assert calls[-1][0] == pytest.approx(64.0, abs=1.0)
    assert calls[-1][1] == pytest.approx(-24.0, abs=1.0)


def test_move_populates_end_position():
    out, _, _ = _move(_FakeVision(_sample(ex=0.15, w_frac=0.1, h_frac=0.1)),
                      duration=0.3)
    assert out.end_x_px == pytest.approx(48.0, abs=1.0)    # 0.15*320
    assert out.end_y_px == pytest.approx(0.0, abs=1.0)


def test_align_gain_caps_speed():
    # Full-right target (ex=1.0) with kp=60 would command 60% but gain=30
    # must clamp it. Lateral PWM never exceeds percent_to_pwm(gain).
    out, pix, _ = _align(_FakeVision(_sample(ex=1.0)),
                         axes={'lat'}, kp_lat=60.0, gain=30.0, duration=0.25)
    assert out.code == TIMEOUT          # never centres -> times out
    cap = _FakePixhawk.percent_to_pwm(30.0)
    lateral = [c['lateral'] for c in pix.rc if c.get('lateral', 1500) != 1500]
    assert lateral, 'expected lateral thrust commands'
    assert max(lateral) <= cap
    assert max(lateral) == cap          # full error saturates exactly at gain


def test_align_yaw_same_polarity_as_lateral():
    # Sign regression pin: yaw must use the SAME polarity as the working lateral
    # axis on the same `ex` (no negation). ex>0 -> Ch4>1500, matching lateral's
    # ex>0 -> Ch6>1500. This guards against re-introducing the old `-ctrl`
    # negation. (Physical yaw DIRECTION is pool-verified, not asserted here --
    # this test only pins the numeric polarity / lateral-parity.)
    pix = _FakePixhawk()
    out, _, _ = _align(_FakeVision(_sample(ex=1.0)), pix=pix,
                       axes={'yaw'}, kp_yaw=60.0, gain=30.0, duration=0.25)
    assert out.code == TIMEOUT          # full-right never centres -> times out
    cap = _FakePixhawk.percent_to_pwm(30.0)   # 1620
    yaw = [c['yaw'] for c in pix.rc if c.get('yaw', 1500) != 1500]
    assert yaw, 'expected yaw thrust commands on Ch4'
    assert min(yaw) > 1500, 'ex>0 must give Ch4>1500 (same polarity as lateral)'
    assert max(yaw) == cap, 'full error saturates exactly at gain'


def test_align_yaw_polarity_mirrors_for_negative_ex():
    # Mirror pin: ex<0 -> Ch4<1500 (same sign relationship as lateral).
    pix = _FakePixhawk()
    _align(_FakeVision(_sample(ex=-1.0)), pix=pix,
           axes={'yaw'}, kp_yaw=60.0, gain=30.0, duration=0.25)
    yaw = [c['yaw'] for c in pix.rc if c.get('yaw', 1500) != 1500]
    assert yaw, 'expected yaw thrust commands on Ch4'
    assert max(yaw) < 1500, 'ex<0 must give Ch4<1500 (mirrors lateral polarity)'


def test_align_yaw_neutral_when_in_band():
    # No shot-jitter: when the target is inside err_px the yaw channel must
    # command exactly 1500 (still) so the mission can fire on a steady hull.
    pix = _FakePixhawk()
    # ex=0.05 on a 640px frame -> epx = 0.05*320 = 16px, inside err_px=40.
    out, _, _ = _align(_FakeVision(_sample(ex=0.05)), pix=pix,
                       axes={'yaw'}, kp_yaw=60.0, gain=30.0, err_px=40.0)
    assert out.code == ALIGNED                 # in-band -> stable -> aligned
    yaws = [c.get('yaw', 1500) for c in pix.rc]
    assert yaws, 'expected yaw RC frames'
    assert all(y == 1500 for y in yaws), (
        'in-band yaw must be neutral 1500 (no thrust when aligned)')


def test_align_yaw_min_floor_spins_thruster_when_close():
    # The hole-lock fix, CLOSE regime: a large bbox (fill >= VISION_YAW_FLOOR_FILL)
    # means the target is near, so a small OUT-of-band yaw error whose pure-
    # proportional output is below the stiction floor is bumped UP toward the floor
    # so micro-corrections don't die in the dead-zone. With the TAPER the lift is
    # proportional to how far past the deadband we are -- a real bump (> pure P) but
    # below the full hard floor (no relay slam). This is the anti-jitter fix.
    from duburi_control.motion_vision import (
        _vision_yaw_floor, VISION_YAW_MIN_PCT)
    pix = _FakePixhawk()
    # Big bbox -> close. err_px=10, ex=0.0625 -> epx=20px (out of band). Tiny kp_yaw
    # so pure proportional (0.0625*10 = 0.625%) sits below the tapered floor at 20px.
    ex, kp = 0.0625, 10.0
    _align(_FakeVision(_sample(ex=ex, w_frac=0.6, h_frac=0.6)), pix=pix,
           axes={'yaw'}, kp_yaw=kp, gain=30.0, err_px=10.0, duration=0.25)
    yaw = [c['yaw'] for c in pix.rc if c.get('yaw', 1500) != 1500]
    assert yaw, 'expected yaw thrust on a small out-of-band error'
    floor_pct = _vision_yaw_floor(ex * 320.0, 10.0, min(VISION_YAW_MIN_PCT, 30.0))
    prop_pwm  = _FakePixhawk.percent_to_pwm(ex * kp)               # pure proportional
    floor_pwm = _FakePixhawk.percent_to_pwm(floor_pct)            # tapered floor
    hard_pwm  = _FakePixhawk.percent_to_pwm(VISION_YAW_MIN_PCT)   # full hard floor (1520)
    assert min(yaw) >= floor_pwm, 'tapered floor must lift the tiny proportional'
    assert min(yaw) > prop_pwm, 'floor must actually add authority over pure P'
    assert max(yaw) < hard_pwm, 'taper must NOT slam the full hard floor (no relay)'


def test_align_yaw_no_floor_when_far_pure_proportional():
    # The wobble fix, FAR regime: a small bbox (fill < VISION_YAW_FLOOR_FILL) means
    # the target is distant. The floor MUST be suppressed so yaw stays pure
    # proportional and decays into the deadband -- a hard minimum on a rate channel
    # is a relay that limit-cycles (the left/right far-field wobble).
    from duburi_control.motion_vision import VISION_YAW_MIN_PCT
    pix = _FakePixhawk()
    # Small bbox -> far. Same small error as the close test, but proportional
    # 3.6% (pwm 1514) must be left as-is, BELOW the 5% floor pwm (1520).
    _align(_FakeVision(_sample(ex=0.06, w_frac=0.05, h_frac=0.05)), pix=pix,
           axes={'yaw'}, kp_yaw=60.0, gain=30.0, err_px=10.0, duration=0.25)
    floor_pwm = _FakePixhawk.percent_to_pwm(VISION_YAW_MIN_PCT)   # 1520
    yaw = [c['yaw'] for c in pix.rc if c.get('yaw', 1500) != 1500]
    assert yaw, 'expected proportional yaw thrust on a small far-field error'
    assert max(yaw) < floor_pwm, (
        'far-field yaw must stay pure-proportional below the floor (no relay)')


def test_align_yaw_floor_never_exceeds_gain_cap():
    # If the operator picks a gain below the floor (very slow fine-lock), the
    # gain cap must still win -- the floor must never push thrust above gain.
    # Close bbox so the floor path actually runs.
    pix = _FakePixhawk()
    _align(_FakeVision(_sample(ex=1.0, w_frac=0.6, h_frac=0.6)), pix=pix,
           axes={'yaw'}, kp_yaw=60.0, gain=3.0, err_px=10.0, duration=0.25)
    cap = _FakePixhawk.percent_to_pwm(3.0)     # gain=3% -> 1512
    yaw = [c['yaw'] for c in pix.rc if c.get('yaw', 1500) != 1500]
    assert yaw, 'expected yaw thrust commands'
    assert max(yaw) == cap, 'gain cap must bound the floor (floor <= gain)'


# --------------------------------------------------------------------------- #
#  align_loop -- per-axis gain caps                                            #
# --------------------------------------------------------------------------- #
def test_align_per_axis_gain_caps_yaw_independently_of_lat():
    # gain=40 globally, yaw_gain=10 just for yaw: full-right target saturates
    # both axes, so yaw clamps at 10% (slow micro-align) while lateral still
    # drives at the global 40% cap.
    pix = _FakePixhawk()
    _align(_FakeVision(_sample(ex=1.0)), pix=pix,
           axes={'yaw', 'lat'}, kp_yaw=60.0, kp_lat=60.0,
           gain=40.0, gain_yaw=10.0, err_px=10.0, duration=0.25)
    yaw_cap = _FakePixhawk.percent_to_pwm(10.0)   # 1540
    lat_cap = _FakePixhawk.percent_to_pwm(40.0)   # 1660
    yaw = [c['yaw'] for c in pix.rc if c.get('yaw', 1500) != 1500]
    lat = [c['lateral'] for c in pix.rc if c.get('lateral', 1500) != 1500]
    assert yaw and lat
    assert max(yaw) == yaw_cap, 'yaw must clamp at its own per-axis cap (10%)'
    assert max(lat) == lat_cap, 'lateral must still use the global gain (40%)'


def test_align_per_axis_gain_unset_inherits_global():
    # yaw_gain left None -> inherits gain=20; full-right saturates at 20%.
    pix = _FakePixhawk()
    _align(_FakeVision(_sample(ex=1.0)), pix=pix,
           axes={'yaw'}, kp_yaw=60.0, gain=20.0, err_px=10.0, duration=0.25)
    cap = _FakePixhawk.percent_to_pwm(20.0)   # 1580
    yaw = [c['yaw'] for c in pix.rc if c.get('yaw', 1500) != 1500]
    assert yaw and max(yaw) == cap, 'unset per-axis gain must inherit the global cap'


def test_align_depth_step_scales_nudge():
    # depth setpoint excursion scales with depth_step (the per-update resolution
    # knob), NOT gain_depth: a larger depth_step deepens more over the same run.
    # (gain_depth no longer drives depth -- depth_step is the sole depth-rate knob.)
    def _depth_excursion(step):
        pix = _FakePixhawk()
        _align(_FakeVision(_sample(ey=1.0, w_frac=0.3, h_frac=0.3)), pix=pix,
               axes={'depth'}, kp_depth=0.05, gain=40.0, depth_step=step,
               err_px=10.0, duration=0.6, align_stable_frames=99)
        return abs(pix.depths[-1] - (-0.5)) if pix.depths else 0.0
    small = _depth_excursion(0.02)
    large = _depth_excursion(0.08)
    assert large > small > 0.0, (
        'depth_step must scale the depth nudge (0.08 deepens more than 0.02)')


# --------------------------------------------------------------------------- #
#  Downward depth-hold (the "bin dives to the pool floor" fix)                 #
# --------------------------------------------------------------------------- #
def test_align_downward_surge_only_holds_depth_constant():
    # THE BUG: a downward SURGE-only align (lat + Ch5 surge, no descent -- the bin
    # task) released Ch3 while streaming NO depth setpoint, so nothing asserted
    # depth-hold and the hull sank to the floor ignoring set_depth. The fix streams
    # the CONSTANT captured depth (get_attitude=-0.5) every tick -- proving depth is
    # actively held. Pre-fix pix.depths would be EMPTY (the sink).
    pix = _FakePixhawk()
    _align(_FakeVision(_sample(ex=0.0, ey=0.0, w_frac=0.3, h_frac=0.3)), pix=pix,
           axes={'lat', 'depth'}, downward=True, release_yaw=True, surge_sign=-1,
           err_px=10.0, duration=0.4, align_stable_frames=99)
    assert pix.depths, 'downward surge-only align must STREAM depth (hold), not sink'
    assert all(d == pytest.approx(-0.5) for d in pix.depths), (
        'no descent -> the held setpoint must stay CONSTANT at the captured depth')


def test_align_downward_releases_ch3_while_streaming():
    # Consistent with the proven forward depth-axis path: when we stream the depth
    # setpoint we release Ch3 (65535) so ArduSub's depth PID is the sole Ch3 consumer.
    pix = _FakePixhawk()
    _align(_FakeVision(_sample(ex=0.0, ey=0.0, w_frac=0.3, h_frac=0.3)), pix=pix,
           axes={'lat', 'depth'}, downward=True, release_yaw=True, surge_sign=-1,
           err_px=10.0, duration=0.4, align_stable_frames=99)
    assert pix.translations, 'release_yaw downward align drives via send_rc_translation'
    assert all(t.get('throttle') == 65535 for t in pix.translations), (
        'Ch3 released (65535) on the streamed downward path')


def test_align_forward_lat_yaw_does_not_stream_depth():
    # REGRESSION GUARD: a FORWARD lat/yaw-only align must be byte-unchanged -- no
    # spurious depth streaming, Ch3 held neutral 1500 (ArduSub latches depth-hold).
    pix = _FakePixhawk()
    _align(_FakeVision(_sample(ex=0.0, ey=0.0, w_frac=0.3, h_frac=0.3)), pix=pix,
           axes={'lat', 'yaw'}, err_px=10.0, duration=0.4, align_stable_frames=99)
    assert pix.depths == [], 'forward lat/yaw-only must NOT stream a depth setpoint'
    assert pix.rc and all(r.get('throttle') == 1500 for r in pix.rc), (
        'forward non-depth align holds Ch3 neutral 1500')


def test_align_positive_depth_bound_warns():
    # SIGN GUARD: depths are negative metres; a POSITIVE max_depth_m / depth_ceiling is
    # a sign error that silently reads as OFF -- warn loudly instead of misleading.
    log = _CapLog()
    _align(_FakeVision(_sample(ex=0.0, ey=0.0, w_frac=0.3, h_frac=0.3)),
           axes={'lat', 'depth'}, downward=True, release_yaw=True, surge_sign=-1,
           max_depth_m=0.6, depth_ceiling_m=0.2,
           err_px=10.0, duration=0.2, align_stable_frames=99, log=log)
    assert any('POSITIVE' in w for w in log.warnings), (
        'a positive depth bound must emit a loud sign-error warning')


def test_align_yaw_and_lat_share_one_override_packet():
    # "Wired" when multiple axes are active: yaw + lat must land in the SAME
    # send_rc_override frame (not split / not clobbering each other).
    pix = _FakePixhawk()
    _align(_FakeVision(_sample(ex=1.0)), pix=pix,
           axes={'yaw', 'lat'}, kp_yaw=60.0, kp_lat=60.0, gain=30.0,
           err_px=10.0, duration=0.25)
    both = [c for c in pix.rc
            if c.get('lateral', 1500) != 1500 and c.get('yaw', 1500) != 1500]
    assert both, 'yaw+lat align must emit Ch6 and Ch4 in the same RC packet'


# --------------------------------------------------------------------------- #
#  Freshness-decay -- pace translational authority to detection FPS            #
# --------------------------------------------------------------------------- #
def test_freshness_pure_function():
    assert _freshness(0.0) == 1.0
    assert _freshness(VISION_FRESH_FULL_S) == 1.0          # boundary: still full
    assert _freshness(VISION_FRESH_ZERO_S) == 0.0          # boundary: zero
    assert _freshness(VISION_FRESH_ZERO_S + 1.0) == 0.0    # beyond: clamped
    mid = (VISION_FRESH_FULL_S + VISION_FRESH_ZERO_S) / 2  # halfway -> 0.5
    assert _freshness(mid) == pytest.approx(0.5, abs=1e-6)


def test_align_lateral_decays_when_sample_stale():
    # A STALE-but-present sample (age in the decay band) must command LESS lateral
    # than the identical FRESH sample -- the loop stops blind-driving on old data.
    fresh_pix = _FakePixhawk()
    _align(_FakeVision(_sample(ex=1.0, age_s=0.0)), pix=fresh_pix,
           axes={'lat'}, kp_lat=60.0, gain=30.0, duration=0.2)
    stale_pix = _FakePixhawk()
    _align(_FakeVision(_sample(ex=1.0, age_s=0.30)), pix=stale_pix,
           axes={'lat'}, kp_lat=60.0, gain=30.0, duration=0.2)
    fresh_lat = max(abs(c['lateral'] - 1500) for c in fresh_pix.rc
                    if c.get('lateral', 1500) != 1500)
    stale_lat = max(abs(c['lateral'] - 1500) for c in stale_pix.rc
                    if c.get('lateral', 1500) != 1500)
    assert stale_lat < fresh_lat, 'a stale frame must decay the lateral command'


def test_align_lateral_zero_when_blind():
    # Sample older than the zero threshold (but < _STALE_LIMIT_S so still "present")
    # -> lateral authority is fully decayed: never blind-drive on a dead frame.
    pix = _FakePixhawk()
    # Past the CEILING, not the floor: the zero threshold is derived from
    # the sensor now (see `_fresh_bounds`), so only an age beyond
    # VISION_FRESH_ZERO_MAX_S is blind for EVERY camera. The property under
    # test -- a blind sample must not drive -- is unchanged.
    _align(_FakeVision(_sample(ex=1.0, age_s=VISION_FRESH_ZERO_MAX_S + 0.05)), pix=pix,
           axes={'lat'}, kp_lat=60.0, gain=30.0, duration=0.2)
    laterals = [c.get('lateral', 1500) for c in pix.rc]
    assert laterals and all(l == 1500 for l in laterals), (
        'a frame past the freshness-zero age must command neutral lateral (no blind drive)')


def test_align_yaw_not_decayed_by_freshness():
    # Yaw is excluded from freshness-decay (Ch4 is a rate ArduSub bleeds). Even a
    # stale frame still drives full yaw -- so yaw authority is unchanged by age.
    pix = _FakePixhawk()
    _align(_FakeVision(_sample(ex=1.0, age_s=0.30)), pix=pix,
           axes={'yaw'}, kp_yaw=60.0, gain=30.0, duration=0.2)
    cap = _FakePixhawk.percent_to_pwm(30.0)
    yaw = [c['yaw'] for c in pix.rc if c.get('yaw', 1500) != 1500]
    assert yaw and max(yaw) == cap, 'yaw must NOT be freshness-decayed (full authority)'


def test_move_forward_decays_when_sample_stale():
    far_fresh = _FakePixhawk()
    _move(_FakeVision(_sample(w_frac=0.1, h_frac=0.1, age_s=0.0)), pix=far_fresh,
          fwd_fill=0.8, gain=30.0, duration=0.2)
    far_stale = _FakePixhawk()
    _move(_FakeVision(_sample(w_frac=0.1, h_frac=0.1, age_s=0.30)), pix=far_stale,
          fwd_fill=0.8, gain=30.0, duration=0.2)
    fresh_fwd = max(abs(c['forward'] - 1500) for c in far_fresh.rc
                    if c.get('forward', 1500) != 1500)
    stale_fwd = max(abs(c['forward'] - 1500) for c in far_stale.rc
                    if c.get('forward', 1500) != 1500)
    assert stale_fwd < fresh_fwd, 'a stale frame must decay the forward approach command'


def test_brake_reads_post_decay_command():
    # The brake EMA must see the POST-decay lateral command, so the two stacked
    # mechanisms don't double-count. A sustained strafe that arrives on STALE
    # frames (decayed) must brake LESS than the identical strafe on FRESH frames.
    drift_fresh = [_sample(ex=0.6, age_s=0.0)] * 4 + [_sample(ex=0.0, age_s=0.0)] * 4
    _, _, fresh_w = _align(_FakeVision(drift_fresh), axes={'lat'}, kp_lat=60.0,
                           gain=30.0, err_px=40.0, duration=2.0)
    drift_stale = [_sample(ex=0.6, age_s=0.30)] * 4 + [_sample(ex=0.0, age_s=0.30)] * 4
    _, _, stale_w = _align(_FakeVision(drift_stale), axes={'lat'}, kp_lat=60.0,
                           gain=30.0, err_px=40.0, duration=2.0)
    fresh_kick = max((1500 - p) for p in fresh_w.laterals) if fresh_w.laterals else 0
    stale_kick = max((1500 - p) for p in stale_w.laterals) if stale_w.laterals else 0
    assert fresh_kick > 0, 'fresh sustained strafe should brake'
    assert stale_kick < fresh_kick, (
        'brake must read the post-decay command -- a decayed approach brakes less')


def test_align_operator_line_format():
    # Output-level pin on the per-verb bearing line: signed lat px + class +
    # residual/deadband. Worded 'offset ... -> err N/Mpx' (NOT "aligned") so it
    # never reads as a verdict; the detector node owns the always-on copy.
    log = _CapLog()
    # ex=0.5 on a 640px frame -> +160px to the right of centre; never reaches
    # the err band, so the loop logs the offset line and times out.
    align_loop(pixhawk=_FakePixhawk(), vision_state=_FakeVision(_sample(ex=0.5)),
               target_class='gate', axes={'lat'}, offsets={}, err_px=10.0,
               duration=0.3, gain=30.0, kp_lat=60.0,
               writers=_FakeWriters(), log=log, abort_fn=None)
    line = next((m for m in log.debugs if m.startswith('[ offset')), None)
    assert line is not None, f'expected an operator offset line, got {log.debugs}'
    assert 'lat=+160' in line, f'expected signed lat px in {line!r}'
    assert "'gate'" in line and '/10px' in line, f'bad format: {line!r}'


# --------------------------------------------------------------------------- #
#  align_loop -- yaw floor is gated on the yaw axis being requested            #
# --------------------------------------------------------------------------- #
def test_yaw_never_driven_when_yaw_axis_absent():
    # Regression pin: omitting yaw= from align() must mean ZERO yaw command --
    # the VISION_YAW_MIN_PCT floor must NOT spin Ch4 just because the bbox is
    # large. A close (high-fill) target hard to one side (ex=0.8) is the exact
    # case that trips the floor when yaw IS active; with axes={'lat'} it must not.
    big_offset = _sample(ex=0.8, w_frac=0.5, h_frac=0.5)   # area-fill 0.5 >= 0.25
    _, pix, _ = _align(_FakeVision(big_offset), axes={'lat'})
    assert pix.rc, 'expected RC frames to be written'
    assert all(frame['yaw'] == 1500 for frame in pix.rc), \
        f'yaw driven without a yaw axis: {[f["yaw"] for f in pix.rc]}'


def test_yaw_floor_DOES_drive_when_yaw_axis_present():
    # Positive control: the SAME high-fill off-centre target WITH yaw requested
    # must drive Ch4 off-neutral (proves the test above would catch a regression
    # that re-introduced yaw drive, rather than passing vacuously).
    big_offset = _sample(ex=0.8, w_frac=0.5, h_frac=0.5)
    _, pix, _ = _align(_FakeVision(big_offset), axes={'yaw', 'lat'})
    assert any(frame['yaw'] != 1500 for frame in pix.rc), \
        'yaw axis requested but Ch4 never left neutral'


# --------------------------------------------------------------------------- #
#  align_loop -- inertial arrival brake                                        #
# --------------------------------------------------------------------------- #
def test_align_brake_fires_after_sustained_strafe():
    # A lat align that drives hard one way and then reaches the band must emit a
    # REVERSE-sign lateral brake kick (opposite the drive) before going neutral,
    # so the hull stops square instead of coasting sideways.
    from duburi_control.motion_vision import VISION_BRAKE_MIN_PCT
    # Off-centre for a while (drives +lat), then snaps into band -> ALIGNED.
    drift = [_sample(ex=0.6)] * 4 + [_sample(ex=0.0)] * 4
    pix = _FakePixhawk()
    out, _, writers = _align(_FakeVision(drift), pix=pix,
                             axes={'lat'}, kp_lat=60.0, gain=30.0,
                             err_px=40.0, duration=2.0)
    assert out.code == ALIGNED
    assert writers.laterals, 'expected a lateral brake kick on arrival'
    # EMA of +lat drive -> reverse kick is BELOW neutral (Ch6 < 1500).
    assert min(writers.laterals) < 1500, 'brake must reverse the drive direction'


def test_align_brake_gated_on_gentle_convergence():
    # THE fire-on-align safety pin: a target already in-band (EMA ~ 0, e.g. the
    # gently-converged hole-lock) must NOT be kicked -- a brake here would shove
    # the hull off the lock right before the mission fires.
    pix = _FakePixhawk()
    out, _, writers = _align(_FakeVision(_sample(ex=0.0)), pix=pix,
                             axes={'lat'}, kp_lat=60.0, gain=30.0, err_px=40.0)
    assert out.code == ALIGNED
    assert not writers.laterals, (
        'gentle convergence (EMA~0) must NOT brake -- the hole-lock shot '
        'must not be disturbed')


def test_align_brake_gated_on_realistic_rampdown_lock():
    # A realistic hole-lock ramps DOWN from off-centre into a tight err band -- the
    # P-output tapers, so the EMA decays below the gate and the lock is NOT kicked.
    # This is the real fire-on-align case (the ex=0-from-tick-1 test is degenerate).
    ramp = [_sample(ex=0.5), _sample(ex=0.4), _sample(ex=0.3), _sample(ex=0.2),
            _sample(ex=0.12), _sample(ex=0.06)] + [_sample(ex=0.03)] * 4
    out, _, writers = _align(_FakeVision(ramp),
                             axes={'lat'}, kp_lat=60.0, gain=12.0,
                             err_px=14.0, duration=5.0)
    assert out.code == ALIGNED
    assert not writers.laterals, (
        'a gently-ramped lock must stay below the brake gate (no pre-fire kick)')


def test_align_brake_fires_on_fast_snap_in():
    # The boundary case the docs warn about: sustained hard lateral drive then an
    # ABRUPT centre keeps the EMA above the gate, so a kick DOES fire on arrival.
    # This is exactly why the torpedo fire path passes brake=False -- pinned so the
    # doc claim stays honest if the gate / EMA constants are ever retuned.
    snap = [_sample(ex=1.0)] * 8 + [_sample(ex=0.03)] * 4
    out, _, writers = _align(_FakeVision(snap),
                             axes={'lat'}, kp_lat=60.0, gain=25.0,
                             err_px=14.0, duration=5.0)
    assert out.code == ALIGNED
    assert writers.laterals and min(writers.laterals) < 1500, (
        'a fast snap-in keeps EMA above the gate -> a reverse kick fires '
        '(documents why fire-from-lock uses brake=False)')


def test_align_brake_off_suppresses_kick():
    # brake=False must coast even after a hard strafe.
    drift = [_sample(ex=0.6)] * 4 + [_sample(ex=0.0)] * 4
    out, _, writers = _align(_FakeVision(drift),
                             axes={'lat'}, kp_lat=60.0, gain=30.0,
                             err_px=40.0, duration=2.0, brake=False)
    assert out.code == ALIGNED
    assert not writers.laterals, 'brake=False must emit no brake kick'


def test_align_lost_after_grace():
    out, _, _ = _align(_FakeVision(None), lost_grace_s=0.1, duration=2.0)
    assert out.code == LOST


def test_align_hold_through_loss_never_reports_lost():
    out, _, _ = _align(_FakeVision(None), hold_through_loss=True,
                       lost_grace_s=0.1, duration=0.3)
    assert out.code == TIMEOUT


def test_align_offset_shifts_target_band():
    # Target sits at ex such that it is exactly the requested offset to the
    # right; with that offset the loop should consider it centred.
    # offset 64px on a 640px frame -> ex = 64/320 = 0.2.
    out, _, _ = _align(_FakeVision(_sample(ex=0.2)),
                       axes={'lat'}, offsets={'lat': 64.0}, err_px=20.0)
    assert out.code == ALIGNED


# --------------------------------------------------------------------------- #
#  move_loop                                                                   #
# --------------------------------------------------------------------------- #
def test_move_no_camera_when_size_zero():
    out, _, _ = _move(_FakeVision(_sample(), size=(0, 0)))
    assert out.code == NO_CAMERA


def test_move_reaches_fill_area():
    out, _, writers = _move(_FakeVision(_sample(w_frac=1.0, h_frac=1.0)),
                            fwd_fill=0.8, mode='area')
    assert out.code == ALIGNED
    assert out.fill >= 0.8
    assert writers.neutralised >= 1


def test_move_mode_height_vs_width():
    # Tall, thin target: height fills the frame, width does not.
    tall = _sample(w_frac=0.2, h_frac=0.9)
    out_h, _, _ = _move(_FakeVision(tall), fwd_fill=0.8, mode='height')
    assert out_h.code == ALIGNED

    out_w, _, _ = _move(_FakeVision(tall), fwd_fill=0.8, mode='width',
                        duration=0.25)
    assert out_w.code == TIMEOUT       # width never reaches 0.8 -> drives in


def test_move_drives_forward_when_far():
    # Small bbox -> not reached -> positive forward thrust, clamped to gain.
    out, pix, _ = _move(_FakeVision(_sample(w_frac=0.1, h_frac=0.1)),
                        fwd_fill=0.8, gain=30.0, duration=0.25)
    assert out.code == TIMEOUT
    cap = _FakePixhawk.percent_to_pwm(30.0)
    fwd = [c['forward'] for c in pix.rc if c.get('forward', 1500) != 1500]
    assert fwd, 'expected forward thrust commands'
    assert max(fwd) <= cap


def test_move_maintain_lateral_clamped():
    # maintain a right offset while a far target drives forward; lateral
    # correction is clamped to gain just like align.
    out, pix, _ = _move(_FakeVision(_sample(ex=1.0, w_frac=0.1, h_frac=0.1)),
                        fwd_fill=0.8, maintain_on=True, maintain_px=0.0,
                        kp_lat=60.0, gain=30.0, duration=0.25)
    cap = _FakePixhawk.percent_to_pwm(30.0)
    lateral = [c['lateral'] for c in pix.rc if c.get('lateral', 1500) != 1500]
    assert lateral
    assert max(lateral) <= cap


def test_move_per_axis_gain_lat_caps_strafe_not_forward():
    # gain=50 forward, gain_lat=10 for the maintain strafe: a far, off-centre
    # target drives forward at the 50% cap while the lateral correction is held
    # to its own slow 10% cap.
    out, pix, _ = _move(_FakeVision(_sample(ex=1.0, w_frac=0.1, h_frac=0.1)),
                        fwd_fill=0.8, maintain_on=True, maintain_px=0.0,
                        kp_lat=60.0, gain=50.0, gain_lat=10.0, duration=0.25)
    assert out.code == TIMEOUT
    fwd = [c['forward'] for c in pix.rc if c.get('forward', 1500) != 1500]
    lat = [c['lateral'] for c in pix.rc if c.get('lateral', 1500) != 1500]
    assert fwd and lat
    assert max(fwd) <= _FakePixhawk.percent_to_pwm(50.0), 'forward keeps the global cap'
    assert max(lat) == _FakePixhawk.percent_to_pwm(10.0), 'strafe uses its own cap'


def test_move_brake_fires_forward_on_fill_stop():
    # Drive forward toward a target that fills the frame -> on the fill-stop
    # arrival the forward inertia is braked (reverse kick BELOW neutral) so the
    # hull halts in front instead of creeping in.
    far_then_big = [_sample(w_frac=0.2, h_frac=0.2)] * 3 + \
                   [_sample(w_frac=1.0, h_frac=1.0)] * 3
    out, _, writers = _move(_FakeVision(far_then_big), fwd_fill=0.8, mode='area',
                            gain=40.0, hold_s=0.0, duration=2.0)
    assert out.code == ALIGNED
    assert writers.forwards, 'expected a forward brake kick on fill-stop arrival'
    assert min(writers.forwards) < 1500, 'forward brake must reverse the drive'


def test_move_passthrough_never_brakes():
    # Pass-through is defined to COAST through the gate: seen then gone -> ALIGNED
    # with NO brake kick (a brake would defeat carrying the hull through).
    seen = _sample(w_frac=0.5, h_frac=0.5)
    vis = _FakeVision([seen, seen, None])
    out, _, writers = _move(vis, passthrough=True, hold_s=0.1,
                            lost_grace_s=5.0, duration=1.0)
    assert out.code == ALIGNED
    assert not writers.forwards, 'pass-through must never brake -- it must coast'


def test_move_timeout_does_not_brake():
    # A far target that never reaches fill -> TIMEOUT, no arrival, no brake.
    out, _, writers = _move(_FakeVision(_sample(w_frac=0.1, h_frac=0.1)),
                            fwd_fill=0.8, gain=30.0, duration=0.25)
    assert out.code == TIMEOUT
    assert not writers.forwards, 'a non-arrival (timeout) exit must not brake'


def test_move_brake_off_suppresses_forward_kick():
    far_then_big = [_sample(w_frac=0.2, h_frac=0.2)] * 3 + \
                   [_sample(w_frac=1.0, h_frac=1.0)] * 3
    out, _, writers = _move(_FakeVision(far_then_big), fwd_fill=0.8,
                            gain=40.0, hold_s=0.0, duration=2.0, brake=False)
    assert out.code == ALIGNED
    assert not writers.forwards, 'brake=False must emit no forward brake kick'


def test_move_lost_after_grace():
    out, _, _ = _move(_FakeVision(None), lost_grace_s=0.1, duration=2.0)
    assert out.code == LOST


def test_move_hold_through_loss_never_reports_lost():
    out, _, _ = _move(_FakeVision(None), hold_through_loss=True,
                      lost_grace_s=0.1, duration=0.3)
    assert out.code == TIMEOUT


# --------------------------------------------------------------------------- #
#  move_loop -- pass-through (fwd=None / fwd_fill<=0)                          #
# --------------------------------------------------------------------------- #
def test_move_passthrough_seen_then_lost_reaches():
    # Gate is seen for a couple of ticks, then leaves the frame -> after the
    # commit window the verb reports ALIGNED ("passed through").
    seen = _sample(w_frac=0.5, h_frac=0.5)
    vis = _FakeVision([seen, seen, None])   # present, present, then gone
    out, _, writers = _move(vis, passthrough=True, hold_s=0.1,
                            lost_grace_s=5.0, duration=1.0)
    assert out.code == ALIGNED
    assert writers.neutralised >= 1


def test_move_passthrough_never_seen_reports_lost():
    # Never detected -> must NOT blind-drive; falls through to LOST so a
    # mission fallback search can run.
    out, _, _ = _move(_FakeVision(None), passthrough=True,
                      lost_grace_s=0.1, duration=2.0)
    assert out.code == LOST


def test_move_passthrough_ignores_fill_stop():
    # A frame-filling bbox would normally trip the fill-stop; in pass-through
    # the AUV keeps driving forward at gain until the target leaves the frame.
    big = _sample(w_frac=1.0, h_frac=1.0)
    out, pix, _ = _move(_FakeVision(big), passthrough=True, fwd_fill=0.8,
                        gain=30.0, duration=0.25)
    assert out.code == TIMEOUT          # target never leaves -> drives the whole budget
    cap = _FakePixhawk.percent_to_pwm(30.0)
    fwd = [c['forward'] for c in pix.rc if c.get('forward', 1500) != 1500]
    assert fwd, 'pass-through must keep commanding forward thrust'
    assert max(fwd) <= cap


# --------------------------------------------------------------------------- #
#  Precision-alignment layers (range gain, lateral I-term, continuity lock)    #
# --------------------------------------------------------------------------- #
def test_range_gain_pure():
    # Far (low fill) -> full gain; close (high fill) -> floored; linear between.
    assert _range_gain(0.0, 0.3) == pytest.approx(1.0)
    assert _range_gain(VISION_RANGE_GAIN_FILL_LO, 0.3) == pytest.approx(1.0)
    assert _range_gain(VISION_RANGE_GAIN_FILL_HI, 0.3) == pytest.approx(0.3)
    assert _range_gain(1.0, 0.3) == pytest.approx(0.3)
    mid = 0.5 * (VISION_RANGE_GAIN_FILL_LO + VISION_RANGE_GAIN_FILL_HI)
    assert 0.3 < _range_gain(mid, 0.3) < 1.0
    # floor>=1.0 (or hi<=lo) is a no-op.
    assert _range_gain(0.9, 1.0) == pytest.approx(1.0)


def _max_lat_dev(pix):
    """Largest |lateral-1500| the loop commanded (0 if it never strafed)."""
    devs = [abs(c['lateral'] - 1500) for c in pix.rc if 'lateral' in c]
    return max(devs) if devs else 0


def test_range_gain_softens_lateral_when_close():
    # Same off-centre error, but a CLOSE target (high fill) must be driven more
    # gently than a FAR one (low fill) -- the 1/range damping fix.
    far  = _FakeVision(_sample(ex=0.5, w_frac=0.1, h_frac=0.1))   # fill 0.10 -> 1.0x
    near = _FakeVision(_sample(ex=0.5, w_frac=0.8, h_frac=0.8))   # fill 0.80 -> floor
    _, pix_far, _  = _align(far,  axes={'lat'}, gain=100.0, range_gain_floor=0.3)
    _, pix_near, _ = _align(near, axes={'lat'}, gain=100.0, range_gain_floor=0.3)
    assert _max_lat_dev(pix_near) < _max_lat_dev(pix_far)
    # floor=1.0 is OFF -> close target driven exactly like the far one's law.
    _, pix_off, _ = _align(near, axes={'lat'}, gain=100.0, range_gain_floor=1.0)
    assert _max_lat_dev(pix_off) > _max_lat_dev(pix_near)


def test_lateral_integral_grows_during_hold():
    # An in-band but non-zero lateral residual (a steady current) builds the
    # lateral integral during the hold, so the commanded strafe exceeds pure-P.
    s = _sample(ex=0.1, w_frac=0.3, h_frac=0.3)   # 32px residual <= 40 err -> in band
    _, pix_p,  _ = _align(_FakeVision(s), axes={'lat'}, gain=100.0,
                          hold_s=0.4, duration=0.7, ki_lat=0.0)
    _, pix_pi, _ = _align(_FakeVision(s), axes={'lat'}, gain=100.0,
                          hold_s=0.4, duration=0.7, ki_lat=200.0, i_lat_max=15.0)
    assert _max_lat_dev(pix_pi) > _max_lat_dev(pix_p)


def test_lateral_integral_clamped():
    # The integral is bounded: a huge ki can't drive past P + i_lat_max (then
    # the g_lat cap). With i_lat_max small the extra deflection stays small.
    s = _sample(ex=0.1, w_frac=0.3, h_frac=0.3)
    _, pix, _ = _align(_FakeVision(s), axes={'lat'}, gain=100.0,
                       hold_s=0.4, duration=0.7, ki_lat=999.0, i_lat_max=5.0)
    # p = 0.1*60 = 6%; + i_lat_max 5% = 11% -> PWM 1500 + 0.11*400 = 1544.
    assert _max_lat_dev(pix) <= _FakePixhawk.percent_to_pwm(11.0) - 1500 + 1


def test_lock_on_passes_near_and_gate():
    # With lock_on the loop hands bbox_error a `near` hint (the last accepted
    # centre) and a positive gate; ctrl_conf rides through as min_score.
    vis = _FakeVision(_sample(ex=0.2, ey=-0.1, w_frac=0.3, h_frac=0.3))
    _align(vis, axes={'lat'}, lock_on=True, ctrl_conf=0.55)
    kw = vis.last_kw
    assert kw['gate_norm'] == pytest.approx(VISION_LOCK_GATE_NORM)
    assert kw['min_score'] == pytest.approx(0.55)
    assert kw['near'] == pytest.approx((0.2, -0.1))   # last accepted centre


def test_lock_off_is_largest_box():
    # Default: no near hint, gate disabled -> bbox_error keeps largest-area.
    vis = _FakeVision(_sample(ex=0.2, w_frac=0.3, h_frac=0.3))
    _align(vis, axes={'lat'}, lock_on=False)
    assert vis.last_kw['near'] is None
    assert vis.last_kw['gate_norm'] == 0.0


def test_hold_against_current_does_not_brake_kick():
    # A hull holding STILL against a steady current carries a large lateral
    # integral but ~0 travel momentum. The arrival brake EMA tracks the
    # PROPORTIONAL command only, so an in-band converged exit is NOT reverse-
    # kicked even with a big integral (regression: EMA must exclude lat_i).
    s = _sample(ex=0.02, w_frac=0.3, h_frac=0.3)   # 6.4px residual, well in band
    _, _, writers = _align(_FakeVision(s), axes={'lat'}, gain=100.0,
                           hold_s=0.3, duration=0.6,
                           ki_lat=200.0, i_lat_max=15.0)   # brake on by default
    assert writers.laterals == []      # no reverse-kick on a steady hold


# --------------------------------------------------------------------------- #
#  Coast (gap-bridging) — _coast_authority + the live/coast authority split    #
#                                                                              #
#  Part B of the tracking integration. A coasted box must steer at DECLINING   #
#  authority over the coast window (never full-blast on a phantom), and must   #
#  NOT be freshness-decayed (it arrives fresh every tick — that would double-  #
#  decay and kill the coast inside ~0.4 s). Default coast_s=0 -> no coast.     #
# --------------------------------------------------------------------------- #
def test_coast_authority_shape():
    assert _coast_authority(0.0, 0.8) == 1.0          # full at the instant of loss
    assert _coast_authority(0.8, 0.8) == 0.0          # zero at the window edge
    assert _coast_authority(1.0, 0.8) == 0.0          # and beyond
    mid = _coast_authority(0.4, 0.8)
    assert 0.4 < mid < 0.6                            # ~halfway, monotonic linear
    assert _coast_authority(0.4, 0.0) == 0.0          # coast_s<=0 -> disabled


def test_authority_splits_live_vs_coasted():
    # A LIVE sample uses freshness (age = message staleness). A COASTED sample
    # uses the coast curve — proving freshness is NOT also applied (no double
    # decay). At age 0.4 s: freshness=0 (FRESH_ZERO_S) but coast(0.4,0.8)=0.5.
    live   = _sample(ex=0.5, age_s=0.4, coasted=False)
    coast  = _sample(ex=0.5, age_s=0.4, coasted=True)
    assert _authority(live, 0.8) == 0.0               # live, stale frame -> blind
    assert _authority(coast, 0.8) == pytest.approx(0.5, abs=1e-6)  # coast, mid-window


def _max_lat_kick(pix):
    lats = [c['lateral'] for c in pix.rc if c.get('lateral', 1500) != 1500]
    return max((p - 1500) for p in lats) if lats else 0


def test_coasted_sample_steers_at_reduced_authority():
    # A coasted box mid-window must drive lateral at LESS than a fresh live box
    # of the same error — the decaying-authority guarantee that stops a phantom
    # from being chased at full speed.
    live  = _FakeVision(_sample(ex=1.0, age_s=0.0, track_id=7, coasted=False))
    coast = _FakeVision(_sample(ex=1.0, age_s=0.4, track_id=7, coasted=True))
    _, pl, _ = _align(live,  axes={'lat'}, kp_lat=60.0, gain=30.0,
                      coast_s=0.8, duration=0.2)
    _, pc, _ = _align(coast, axes={'lat'}, kp_lat=60.0, gain=30.0,
                      coast_s=0.8, duration=0.2)
    live_kick, coast_kick = _max_lat_kick(pl), _max_lat_kick(pc)
    assert live_kick > 0
    assert 0 < coast_kick < live_kick, (
        f"coasted command {coast_kick} must be a fraction of live {live_kick}")


# --------------------------------------------------------------------------- #
#  align_loop forward range-hold axis (the unified torpedo standoff shot)      #
# --------------------------------------------------------------------------- #
def _fwd_cmds(pix):
    """Forward (Ch5) commands from send_rc_override that are not neutral."""
    return [c['forward'] for c in pix.rc if c.get('forward', 1500) != 1500]


def test_align_no_forward_axis_when_fwd_fill_zero():
    # fwd_fill=0 (default) -> align never commands forward (behaviour unchanged).
    # Target far on the fill metric but centred laterally; forward must stay neutral.
    out, pix, _ = _align(_FakeVision(_sample(ex=0.0, h_frac=0.1)),
                         axes={'lat'}, duration=0.3)
    assert all(c.get('forward', 1500) == 1500 for c in pix.rc), \
        "fwd_fill=0 must never write a forward command"


def test_align_forward_drives_when_far():
    # bbox smaller than the standoff -> drive forward (>1500), one-sided.
    out, pix, _ = _align(_FakeVision(_sample(ex=0.0, h_frac=0.1)),
                         axes={'lat'}, fwd_fill=0.5, fwd_mode='height',
                         duration=0.3)
    fwds = _fwd_cmds(pix)
    assert fwds and max(fwds) > 1500, "far target must drive forward"


def test_align_forward_one_sided_never_reverses_at_standoff():
    # bbox AT/PAST the standoff -> forward commanded EXACTLY neutral, never a
    # reverse PWM (<1500). Pins "no reverse-kick / no ramming the board".
    out, pix, _ = _align(_FakeVision(_sample(ex=0.0, h_frac=0.9)),
                         axes={'lat'}, fwd_fill=0.5, fwd_mode='height',
                         duration=0.3)
    assert all(c.get('forward', 1500) <= 1500 for c in pix.rc), \
        "forward must never reverse (one-sided)"
    # lat centred AND fill past standoff -> both in-band -> ALIGNED.
    assert out.code == ALIGNED


def test_align_forward_neutral_inside_band():
    # Within FWD_BAND of the standoff -> forward neutral (no twitch on fill noise).
    fill = 0.5
    out, pix, _ = _align(
        _FakeVision(_sample(ex=0.0, h_frac=fill - FWD_BAND * 0.5)),
        axes={'lat'}, fwd_fill=fill, fwd_mode='height', duration=0.3)
    assert all(c.get('forward', 1500) == 1500 for c in pix.rc), \
        "inside FWD_BAND the forward axis must be neutral"


def test_align_forward_gates_fire_until_standoff():
    # lat centred but bbox far from the standoff -> NOT in-band -> on_locked
    # (the mid-hold fire) must NEVER trip until the standoff range is reached.
    fired = []
    out, _, _ = _align(_FakeVision(_sample(ex=0.0, h_frac=0.1)),
                       axes={'lat'}, fwd_fill=0.5, fwd_mode='height',
                       hold_s=0.3, duration=0.5,
                       on_locked=lambda: fired.append(1))
    assert fired == [], "fire must be gated on reaching the standoff"
    assert out.code == TIMEOUT   # never reached the standoff -> no ALIGNED


def test_align_forward_fires_once_at_standoff():
    # lat centred AND bbox at the standoff -> in-band -> the mid-hold fire trips
    # exactly once while holding.
    fired = []
    out, _, _ = _align(_FakeVision(_sample(ex=0.0, h_frac=0.9)),
                       axes={'lat'}, fwd_fill=0.5, fwd_mode='height',
                       hold_s=0.3, duration=1.0,
                       on_locked=lambda: fired.append(1))
    assert fired == [1], "standoff lock must fire exactly once mid-hold"
    assert out.code == ALIGNED


def test_align_forward_decays_with_authority():
    # A STALE sample (age past the freshness-zero) decays the forward command to
    # ~neutral even though the bbox is far -- forward shares lat's freshness gate,
    # so a slow/blind frame can't blind-drive the standoff approach.
    out, pix, _ = _align(
        # Past the ceiling -- blind under any derived threshold. See above.
        _FakeVision(_sample(ex=0.0, h_frac=0.1, age_s=VISION_FRESH_ZERO_MAX_S + 0.05)),
        axes={'lat'}, fwd_fill=0.5, fwd_mode='height', duration=0.3)
    assert all(c.get('forward', 1500) == 1500 for c in pix.rc), \
        "a stale sample must not drive forward (freshness-decayed to neutral)"


# --------------------------------------------------------------------------- #
#  align_loop settle gate (opt-in; ports move's "settled when it stops")       #
# --------------------------------------------------------------------------- #
# Samples that stay IN-BAND on lat (epx < eff_err=40) but jump frame-to-frame:
# ex=0.09 -> epx≈28.8px, ex=0.0 -> 0px. Both in band, but |Δworst|≈28.8px > a
# small settle_px -> the hull is "passing through" the band, not settled.
_OSCILLATING = [_sample(ex=0.09), _sample(ex=0.0)] * 40


def test_settle_off_exits_on_oscillating_in_band():
    # settle_px=0 (default): position-in-band for align_stable_frames is enough,
    # so an in-band-but-moving hull still declares ALIGNED (today's behaviour).
    out, _, _ = _align(_FakeVision(list(_OSCILLATING)), axes={'lat'},
                       settle_px=0.0, duration=0.6)
    assert out.code == ALIGNED


def test_settle_gate_blocks_exit_while_moving():
    # settle_px>0: the same in-band-but-moving hull must NOT declare aligned --
    # |Δworst|≈28.8px exceeds settle_px=10, so `stable` keeps resetting -> TIMEOUT
    # rather than a premature mid-pass ALIGNED that would coast off target.
    out, _, _ = _align(_FakeVision(list(_OSCILLATING)), axes={'lat'},
                       settle_px=10.0, duration=0.6)
    assert out.code == TIMEOUT


def test_settle_gate_exits_when_settled():
    # settle_px>0 with a genuinely settled hull (centred every tick, |Δworst|=0)
    # -> declares ALIGNED. The gate adds a settle requirement, it doesn't block a
    # hull that has actually stopped on target.
    out, _, _ = _align(_FakeVision(_sample(ex=0.0)), axes={'lat'},
                       settle_px=10.0, duration=0.6)
    assert out.code == ALIGNED


def test_settle_gate_droop_safe_steady_offset_still_in_band():
    # A steady (non-moving) sample that is IN-BAND exits even with the gate on --
    # confirming the gate keys on error VELOCITY (|Δworst|), not absolute error or
    # command, so a steady-state hold (e.g. against a current, once ki_lat nulls
    # it into the band) is never blocked.
    out, _, _ = _align(_FakeVision(_sample(ex=0.05)), axes={'lat'},   # epx≈16px, in band, steady
                       settle_px=5.0, err_px=40.0, duration=0.6)
    assert out.code == ALIGNED


def test_settle_gate_does_not_block_fire_when_settled():
    # The mid-hold fire rides the SAME stable-frame counter the settle gate
    # gates. With a genuinely settled lock (centred every tick, |Δworst|=0) the
    # gate passes, so settle_px + hold + on_locked still fires -- the settle gate
    # doesn't break a settled fire-from-lock.
    fired = []
    out, _, _ = _align(_FakeVision(_sample(ex=0.0)), axes={'lat'},
                       settle_px=10.0, hold_s=0.3, duration=1.0,
                       on_locked=lambda: fired.append(1))
    assert out.code == ALIGNED
    assert fired == [1]


def test_settle_gate_below_jitter_can_suppress_fire():
    # Guard the documented footgun: a settle_px BELOW the bbox jitter (the hull
    # is in-band but the box wobbles |Δworst|≈28.8px each tick) keeps resetting
    # `stable`, so the mid-hold fire never trips and the verb TIMEOUTs. This is
    # exactly why `settle` must NOT be put on a terminal fire-lock.
    fired = []
    out, _, _ = _align(_FakeVision(list(_OSCILLATING)), axes={'lat'},
                       settle_px=4.0, hold_s=0.3, duration=0.6,
                       on_locked=lambda: fired.append(1))
    assert out.code == TIMEOUT
    assert fired == []


# --------------------------------------------------------------------------- #
#  V-FIRE: mid-hold fire only on a LIVE, FRESH detection (never a stale/coast) #
# --------------------------------------------------------------------------- #
def test_fire_withheld_on_coasted_sample():
    # A tracker-coasted (Kalman-predicted) box must NEVER fire a torpedo -- coast
    # holds the lock, it does not take the shot. Centred + coasted -> no fire.
    fired = []
    _align(_FakeVision(_sample(ex=0.0, coasted=True)), axes={'lat'},
           hold_s=0.3, duration=1.0, on_locked=lambda: fired.append(1))
    assert fired == [], 'must not fire on a coasted (predicted) box'


def test_fire_leaves_on_fresh_live_sample():
    # The control case: a fresh, live, centred box DOES fire (the guard only
    # blocks coasted / frozen-detector, never a genuine current sighting).
    fired = []
    out, _, _ = _align(_FakeVision(_sample(ex=0.0, age_s=0.0)), axes={'lat'},
                       hold_s=0.3, duration=1.0, on_locked=lambda: fired.append(1))
    assert out.code == ALIGNED
    assert fired == [1], 'a fresh live lock must fire'


class _FreezeAfterVision(_FakeVision):
    """Fresh, ADVANCING detections until ``freeze_after`` s, then a FROZEN frame
    (constant sampled_at, growing age). Models a detector that dies mid-hold AFTER
    the lock was already achieved on fresh frames -- the freeze-AFTER-alignment
    case the fire gate must survive (a held `stable` counter must not fire on a
    stale box). Distinct from _FrozenFrameVision (frozen from tick 0)."""
    def __init__(self, ex=0.0, freeze_after=0.25, **kw):
        super().__init__(_sample(ex=ex, age_s=0.0), **kw)
        self._ex = ex
        self._t0 = time.monotonic()
        self._freeze_after = freeze_after
        self._frozen_at = None

    def bbox_error(self, _cls, **_kw):
        now = time.monotonic()
        if now - self._t0 < self._freeze_after:
            return _sample(ex=self._ex, age_s=0.0)          # fresh, sampled_at advances
        if self._frozen_at is None:
            self._frozen_at = now                            # peg sampled_at here on
        age = min(now - self._frozen_at, 0.9)                # < _STALE_LIMIT_S (still present)
        return _sample(ex=self._ex, age_s=age)


def test_fire_withheld_when_detector_freezes_after_alignment():
    # The advisor's discriminating case: the hull locks on fresh frames (stable
    # reaches threshold), THEN the detector freezes. The re-read HOLDS stable at
    # threshold, but is_new_frame is False on the frozen frames -> no fire. fire_t
    # is set so the shot would only come DUE after the freeze, so a freshness gate
    # keyed on age (e.g. age <= _STALE_LIMIT_S) would WRONGLY fire here.
    fired = []
    _align(_FreezeAfterVision(ex=0.0, freeze_after=0.25), axes={'lat'},
           align_stable_frames=3, hold_s=0.6, duration=0.7, fire_t=0.35,
           on_locked=lambda: fired.append(1))
    assert fired == [], 'must not fire once the detector freezes mid-hold'


# --------------------------------------------------------------------------- #
#  V-STABLE: the stable-frame gate counts distinct DETECTIONS, not loop ticks  #
# --------------------------------------------------------------------------- #
class _FrozenFrameVision(_FakeVision):
    """Serves ONE detection with a CONSTANT arrival time (a frozen detector).

    sampled_at = now - age_s is pegged to a fixed monotonic instant, so as the
    loop's real clock advances the returned age_s grows in lockstep -- the same
    single frame re-read over and over. This is exactly the low-FPS degeneracy
    the distinct-frame gate must reject (sampled_at never advances -> never a 2nd
    distinct frame). age_s is capped below _STALE_LIMIT_S so the box stays
    "present" (the failure mode is a stuck frame, not a loss).
    """
    def __init__(self, ex=0.0, **kw):
        super().__init__(_sample(ex=ex, age_s=0.0), **kw)
        self._ex = ex
        self._t0 = time.monotonic()

    def bbox_error(self, _cls, **_kw):
        age = min(time.monotonic() - self._t0, 0.9)   # < _STALE_LIMIT_S
        return _sample(ex=self._ex, age_s=age)


def test_stable_gate_ignores_reread_of_one_frozen_frame():
    # A single frozen frame re-read many times must NOT declare ALIGNED --
    # sampled_at never advances, so the distinct-frame gate never counts a 2nd
    # frame. It ends TIMEOUT rather than firing on one lucky frame.
    out, _, _ = _align(_FrozenFrameVision(ex=0.0),
                       axes={'lat'}, align_stable_frames=3, duration=0.3)
    assert out.code == TIMEOUT, f'one frozen frame must not ALIGN (got {out.code})'


def test_frozen_frame_does_not_fire():
    # The safety consequence: a frozen detector must not fire a torpedo on its
    # one stuck frame (freshness guard AND the distinct-frame gate both block it).
    fired = []
    _align(_FrozenFrameVision(ex=0.0), axes={'lat'},
           align_stable_frames=3, hold_s=0.3, duration=0.3,
           on_locked=lambda: fired.append(1))
    assert fired == [], 'a frozen detector must never fire'


# --------------------------------------------------------------------------- #
#  fire_pass: guaranteed end-of-command shot (opt-in)                          #
# --------------------------------------------------------------------------- #
def test_fire_pass_fires_on_timeout_when_seen_recently():
    # Never fully aligned (target off-centre) but seen live+recently -> fire_pass
    # actuates the payload at command end (TIMEOUT) for a guaranteed partial shot.
    fired = []
    out, _, _ = _align(_FakeVision(_sample(ex=5.0, age_s=0.0)), axes={'lat'},
                       duration=0.3, fire_pass=True,
                       on_locked=lambda: fired.append(1))
    assert out.code == TIMEOUT
    assert fired == [1], 'fire_pass must fire at command end on a recent sighting'


def test_fire_pass_off_by_default_never_fires_unaligned():
    # Back-compat / safety: without fire_pass an unaligned run never fires.
    fired = []
    _align(_FakeVision(_sample(ex=5.0, age_s=0.0)), axes={'lat'},
           duration=0.3, on_locked=lambda: fired.append(1))
    assert fired == [], 'default (fire_pass off) must not fire when unaligned'


def test_fire_pass_withheld_when_never_seen():
    # fire_pass only fires on a recent LIVE sighting -- never into empty water.
    fired = []
    _align(_FakeVision(None), axes={'lat'}, duration=0.3, fire_pass=True,
           on_locked=lambda: fired.append(1))
    assert fired == [], 'fire_pass must NOT fire when the target was never seen'


def test_fire_pass_withheld_on_coasted_only():
    # A target only ever seen as a coasted (Kalman) box is not a live sighting;
    # fire_pass must still withhold (last_live_at never advances).
    fired = []
    _align(_FakeVision(_sample(ex=5.0, age_s=0.0, coasted=True)), axes={'lat'},
           duration=0.3, fire_pass=True, on_locked=lambda: fired.append(1))
    assert fired == [], 'fire_pass must NOT fire on a coasted-only target'


# --------------------------------------------------------------------------- #
#  Depth: deadband-freeze + stepped slew (the z-wobble fix)                    #
# --------------------------------------------------------------------------- #
def test_depth_setpoint_frozen_inside_deadband():
    # An in-band (centred) depth target FREEZES the ALT_HOLD setpoint -- ArduSub
    # holds instead of chasing bbox-y jitter. Every streamed setpoint == start.
    pix = _FakePixhawk()
    _align(_FakeVision(_sample(ey=0.0, age_s=0.0)), pix=pix, axes={'depth'},
           duration=0.3)
    assert pix.depths, 'depth axis must stream a setpoint'
    assert all(abs(d - (-0.5)) < 1e-9 for d in pix.depths), \
        'setpoint must stay frozen at start depth when in-band'


def test_depth_step_bounds_setpoint_slew_out_of_band():
    # Out of the deadband the setpoint steps toward target by <= depth_step per
    # 5 Hz update (the resolution knob) -- never a fast slew ArduSub can't track.
    pix = _FakePixhawk()
    step = 0.05
    _align(_FakeVision(_sample(ey=1.0, age_s=0.0)), pix=pix, axes={'depth'},
           depth_step=step, duration=0.6, align_stable_frames=99)
    assert len(pix.depths) >= 2, 'expected multiple depth updates'
    deltas = [abs(b - a) for a, b in zip(pix.depths, pix.depths[1:])]
    assert max(deltas) <= step + 1e-9, \
        f'each depth update must move <= depth_step ({max(deltas)} > {step})'


def test_depth_setpoint_never_shallower_than_floor():
    # The surface floor (_MIN_DEPTH_M) is preserved: an upward (shallower) target
    # can't drive the setpoint above the floor.
    from duburi_control.motion_vision import _MIN_DEPTH_M
    pix = _FakePixhawk()
    _align(_FakeVision(_sample(ey=-1.0, age_s=0.0)), pix=pix, axes={'depth'},
           depth_step=0.1, duration=0.6, align_stable_frames=99, depth_sign=+1)
    assert all(d <= _MIN_DEPTH_M + 1e-9 for d in pix.depths), \
        'setpoint must never rise above the surface floor'


# A shallow floor the descent actually REACHES within the test window, so the
# clamp genuinely engages (starts ~-0.5, steps deeper ~0.05/update at 5 Hz).
_FWD_FLOOR = -0.7


def test_forward_depth_deep_floor_clamps():
    # CTRL-11: the FORWARD depth axis is two-sided -- a target persistently BELOW
    # centre (ey=+1) walks the setpoint deeper. With max_depth_m set (<0) it must
    # never pass that deep floor (pool-floor guard), symmetric with the downward path.
    pix = _FakePixhawk()
    _align(_FakeVision(_sample(ey=+1.0, age_s=0.0)), pix=pix, axes={'depth'},
           depth_step=0.1, duration=2.0, align_stable_frames=99, depth_sign=+1,
           max_depth_m=_FWD_FLOOR)
    assert pix.depths, 'expected depth updates'
    assert all(d >= _FWD_FLOOR - 1e-9 for d in pix.depths), \
        f'forward depth must never pass max_depth_m ({min(pix.depths)} < {_FWD_FLOOR})'
    assert min(pix.depths) == pytest.approx(_FWD_FLOOR, abs=0.06), \
        'the descent must actually REACH the floor (else the clamp is untested)'


def test_forward_depth_unbounded_without_max_depth_regression():
    # max_depth_m unset (>=0) -> forward path byte-unchanged: the setpoint steps
    # deeper PAST where the floor would have clamped it, proving the fix is opt-in.
    pix = _FakePixhawk()
    _align(_FakeVision(_sample(ey=+1.0, age_s=0.0)), pix=pix, axes={'depth'},
           depth_step=0.1, duration=2.0, align_stable_frames=99, depth_sign=+1)
    assert min(pix.depths) < _FWD_FLOOR - 1e-3, \
        'without max_depth_m the forward depth axis is unclamped (regression guard)'


# --------------------------------------------------------------------------- #
#  Yaw floor taper (the close-in yaw-jitter fix)                               #
# --------------------------------------------------------------------------- #
def test_vision_yaw_floor_tapers_to_zero_at_deadband():
    from duburi_control.motion_vision import (
        _vision_yaw_floor, VISION_YAW_APPROACH_BAND_PX as BAND)
    full, eff = 5.0, 10.0
    assert _vision_yaw_floor(eff, eff, full) == 0.0            # deadband edge -> 0
    assert _vision_yaw_floor(eff + BAND, eff, full) == full    # band edge -> full
    assert _vision_yaw_floor(eff + 2 * BAND, eff, full) == full  # beyond -> full
    mid = _vision_yaw_floor(eff + 0.5 * BAND, eff, full)
    assert 0.0 < mid < full, 'floor must taper monotonically inside the band'


# --------------------------------------------------------------------------- #
#  DOWNWARD-camera frame remap (bin task): image-Y -> Ch5 surge, fill -> depth #
# --------------------------------------------------------------------------- #
def _fwd_pwms(pix):
    """Ch5 (forward/surge) PWMs commanded via send_rc_override, excluding neutral."""
    return [c['forward'] for c in pix.rc if c.get('forward', 1500) != 1500]


def test_downward_surge_drives_ch5_from_image_y():
    # DOWNWARD: the 'depth' axis (image-Y) drives Ch5 SURGE, two-sided. A target
    # BELOW centre (ey>0) drives forward (Ch5>1500); ABOVE (ey<0) drives back.
    fwd = _fwd_pwms(_align(_FakeVision(_sample(ey=1.0)), axes={'lat', 'depth'},
                           downward=True, err_px=10.0, duration=0.25)[1])
    assert fwd and max(fwd) > 1500, 'target below (ey>0) must surge FORWARD on Ch5'
    back = _fwd_pwms(_align(_FakeVision(_sample(ey=-1.0)), axes={'lat', 'depth'},
                            downward=True, err_px=10.0, duration=0.25)[1])
    assert back and min(back) < 1500, 'target above (ey<0) must surge BACK on Ch5'


def test_downward_surge_sign_flips_fore_aft():
    # surge_sign=-1 reverses the Ch5 polarity for a flipped physical mount.
    fwd = _fwd_pwms(_align(_FakeVision(_sample(ey=1.0)), axes={'lat', 'depth'},
                           downward=True, surge_sign=-1, err_px=10.0,
                           duration=0.25)[1])
    assert fwd and max(fwd) < 1500, 'surge_sign=-1 must reverse fore/aft'


def test_downward_surge_freshness_decays_with_sample_age():
    # Parity audit dim (a): the downward SURGE command (Ch5) must be freshness-
    # decayed like the forward lat/fwd axes -- a stale bbox drives the hull toward
    # neutral instead of blind-surging on an old sighting. Fresh sample surges hard;
    # a near-zero-authority-age sample surges much weaker.
    fresh = _fwd_pwms(_align(_FakeVision(_sample(ey=1.0, age_s=0.0)),
                             axes={'lat', 'depth'}, downward=True,
                             err_px=10.0, duration=0.25)[1])
    stale = _fwd_pwms(_align(_FakeVision(_sample(ey=1.0, age_s=0.35)),
                             axes={'lat', 'depth'}, downward=True,
                             err_px=10.0, duration=0.25)[1])
    fresh_mag = max((abs(p - 1500) for p in fresh), default=0)
    stale_mag = max((abs(p - 1500) for p in stale), default=0)
    assert fresh_mag > 0, 'fresh surge should drive Ch5 off neutral'
    assert stale_mag < fresh_mag * 0.5, \
        f'a stale sample must decay the surge (stale {stale_mag} vs fresh {fresh_mag})'


def test_downward_lat_still_drives_ch6():
    # image-X still drives Ch6 lateral on a downward camera (unchanged).
    _, pix, _ = _align(_FakeVision(_sample(ex=1.0)), axes={'lat', 'depth'},
                       downward=True, err_px=10.0, duration=0.25)
    lat = [c['lateral'] for c in pix.rc if c.get('lateral', 1500) != 1500]
    assert lat and max(lat) > 1500, 'target right (ex>0) must strafe RIGHT on Ch6'


def test_downward_depth_axis_does_not_move_setpoint():
    # On downward the 'depth' axis is SURGE, so it must NOT step the ALT_HOLD depth
    # setpoint (that would sink/surface the hull). Any streamed depth == start.
    _, pix, _ = _align(_FakeVision(_sample(ey=1.0)), axes={'lat', 'depth'},
                       downward=True, err_px=10.0, duration=0.3)
    assert all(abs(d - (-0.5)) < 1e-9 for d in pix.depths), \
        'downward depth axis must not drive the depth setpoint (it is surge)'


def test_forward_depth_axis_unchanged_regression():
    # FORWARD camera (downward=False, the default): the 'depth' axis still drives
    # the depth setpoint and NOT Ch5 -- the remap must not leak into forward.
    _, pix, _ = _align(_FakeVision(_sample(ey=1.0)), axes={'lat', 'depth'},
                       err_px=10.0, duration=0.3)
    assert not _fwd_pwms(pix), 'forward depth axis must never drive Ch5 surge'
    assert pix.depths and pix.depths[-1] < -0.5, \
        'forward depth axis must still deepen the setpoint on ey>0'


def test_downward_fill_to_depth_descends_bounded():
    # DOWNWARD + fwd_fill: descend (deeper) while the bbox is below the fill target,
    # bounded by max_depth_m so it can't drive into the pool floor.
    floor = -0.56
    _, pix, _ = _align(_FakeVision(_sample(ey=0.0, w_frac=0.1, h_frac=0.1)),
                       axes={'lat', 'depth'}, downward=True, fwd_fill=0.8,
                       fwd_mode='height', max_depth_m=floor, err_px=10.0, duration=1.0)
    assert pix.depths and pix.depths[-1] < -0.5, 'fill<target must DESCEND (deeper)'
    assert min(pix.depths) >= floor - 1e-9, \
        'descent must never pass max_depth_m (the deep floor)'


def test_downward_descent_step_bounded_by_depth_step():
    # The descent uses the depth_step logic: each 5 Hz update moves the setpoint by
    # AT MOST depth_step m (proportional to the fill deficit, so <= that cap).
    step = 0.03
    _, pix, _ = _align(_FakeVision(_sample(ey=0.0, w_frac=0.1, h_frac=0.1)),
                       axes={'lat', 'depth'}, downward=True, fwd_fill=0.9,
                       fwd_mode='height', depth_step=step, max_depth_m=-2.0,
                       err_px=10.0, duration=1.0)
    deltas = [abs(b - a) for a, b in zip(pix.depths, pix.depths[1:])]
    assert deltas and max(deltas) <= step + 1e-9, \
        f'each descent update must move <= depth_step ({max(deltas)} > {step})'


def test_downward_descent_frozen_at_fill_target():
    # Once the bbox fills to the target (deficit within FWD_BAND), the descent
    # FREEZES -- the setpoint holds instead of chasing bbox jitter (no z-wobble).
    _, pix, _ = _align(_FakeVision(_sample(ey=0.0, w_frac=0.95, h_frac=0.95)),
                       axes={'lat', 'depth'}, downward=True, fwd_fill=0.8,
                       fwd_mode='height', max_depth_m=-2.0, err_px=10.0, duration=0.5)
    assert all(abs(d - (-0.5)) < 1e-9 for d in pix.depths), \
        'at/past the fill target the depth setpoint must be frozen (no descent)'


def test_depth_ceiling_clamps_forward_ascent():
    # depth_ceiling_m is the shallowest allowed setpoint on ANY depth motion. A
    # FORWARD depth align whose target is well ABOVE centre (ey<0 -> ascend) must
    # never drive the setpoint shallower than the ceiling (surface guard).
    ceil = -0.4
    _, pix, _ = _align(_FakeVision(_sample(ey=-1.0)), axes={'lat', 'depth'},
                       depth_ceiling_m=ceil, err_px=10.0, duration=0.6,
                       align_stable_frames=99)
    assert pix.depths, 'depth axis must stream a setpoint'
    assert max(pix.depths) <= ceil + 1e-9, \
        'setpoint must never rise shallower than depth_ceiling_m (surface guard)'


def test_downward_surge_brakes_on_arrival():
    # Ch5 surge coasts (open-loop timed thrust) like Ch6 -- the arrival brake must
    # kick the forward axis so the hull stops square over the bin. A hard snap-in
    # (large ey then centred) crosses the brake gate -> writers.forward is written.
    seq = [_sample(ey=0.8)] * 4 + [_sample(ey=0.0)] * 4
    _, _, writers = _align(_FakeVision(seq), axes={'lat', 'depth'}, downward=True,
                           align_stable_frames=3, err_px=10.0, duration=0.6)
    assert writers.forwards, 'downward arrival must brake the Ch5 surge coast'


# --------------------------------------------------------------------------- #
#  The squareness gate -- 6-DoF pose in front of the trigger
# --------------------------------------------------------------------------- #
def test_the_squareness_gate_is_OFF_by_default():
    """Off must be byte-for-byte the previous behaviour. A gate that switched
    itself on would silently stop every existing torpedo mission firing."""
    calls = []
    out, _pix, _w = _align(_FakeVision(_sample(ex=0.1)), hold_s=0.3,
                           duration=2.0, on_locked=lambda: calls.append(1),
                           fire_t=0.0)
    assert out.code == ALIGNED and len(calls) == 1


def test_a_NOT_SQUARE_target_HOLDS_the_shot():
    """A round leaves along the hull's axis: centred is not aligned. Fired
    30 deg off-normal it misses an opening it was perfectly centred on."""
    calls = []
    out, _pix, _w = _align(_FakeVision(_sample(ex=0.1)), hold_s=0.3,
                           duration=2.0, on_locked=lambda: calls.append(1),
                           fire_t=0.0, require_square_deg=5.0,
                           square_fn=lambda tol: False)
    assert out.code == ALIGNED, 'the gate must hold the SHOT, not the align'
    assert calls == [], 'fired at a target that is not square'


def test_a_SQUARE_target_still_fires():
    calls = []
    _out, _pix, _w = _align(_FakeVision(_sample(ex=0.1)), hold_s=0.3,
                            duration=2.0, on_locked=lambda: calls.append(1),
                            fire_t=0.0, require_square_deg=5.0,
                            square_fn=lambda tol: True)
    assert len(calls) == 1


def test_NO_POSE_means_NO_FIRE():
    """An absent lock_node, an uncalibrated camera or an unset target width all
    arrive here as a falsey answer. For a firing gate the fail-safe direction
    is DO NOT FIRE -- the opposite choice fires on no information at all."""
    calls = []
    _align(_FakeVision(_sample(ex=0.1)), hold_s=0.3, duration=2.0,
           on_locked=lambda: calls.append(1), fire_t=0.0,
           require_square_deg=5.0, square_fn=None)
    assert calls == []


def test_a_RAISING_gate_refuses_rather_than_killing_the_loop():
    """The fire path must never take the control loop down with it, and a gate
    that threw must not be read as permission."""
    calls = []
    def boom(_tol):
        raise RuntimeError('pose node died')
    out, _pix, _w = _align(_FakeVision(_sample(ex=0.1)), hold_s=0.3,
                           duration=2.0, on_locked=lambda: calls.append(1),
                           fire_t=0.0, require_square_deg=5.0, square_fn=boom)
    assert out.code == ALIGNED and calls == []


def test_a_HELD_shot_still_fires_once_the_hull_squares_up():
    """A refusal is 'not yet', not 'never': the gate must not consume the shot.
    Otherwise a hull that squares up during the hold has already lost it."""
    calls = []
    state = {'n': 0}
    def gate(_tol):
        state['n'] += 1
        return state['n'] > 3          # square only after a few ticks
    _align(_FakeVision(_sample(ex=0.1)), hold_s=0.6, duration=2.0,
           on_locked=lambda: calls.append(1), fire_t=0.0,
           require_square_deg=5.0, square_fn=gate)
    assert len(calls) == 1, f'expected the delayed shot, got {len(calls)}'


def test_the_gate_is_asked_with_the_operators_tolerance():
    seen = []
    _align(_FakeVision(_sample(ex=0.1)), hold_s=0.3, duration=2.0,
           on_locked=lambda: None, fire_t=0.0, require_square_deg=7.0,
           square_fn=lambda tol: seen.append(tol) or True)
    assert seen and all(t == 7.0 for t in seen)
