"""Behaviour tests for the two-verb vision control engine.

`motion_vision.align_loop` / `move_loop` are the most competition-relevant
control code, so the pixel-error convergence, the `gain`-as-max-speed clamp,
the fill-ratio exit per `mode`, the `maintain` lateral hold, and the
loss -> grace -> LOST path are pinned here with lightweight fakes. No ROS /
MAVLink needed -- only the pure `Pixhawk.percent_to_pwm` static is used.
"""

from types import SimpleNamespace

import pytest

from duburi_control.motion_vision import (
    align_loop, move_loop, _fill, _clamp, _present, _freshness, _range_gain,
    _coast_authority, _authority,
    VISION_FRESH_FULL_S, VISION_FRESH_ZERO_S,
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
    # means the target is near, so a small OUT-of-band yaw error whose
    # pure-proportional output is below the T200 spin-up floor must be bumped UP
    # to the floor -- otherwise micro-corrections die in the dead-zone and the
    # tight lock stalls just outside err_px.
    from duburi_control.motion_vision import VISION_YAW_MIN_PCT
    pix = _FakePixhawk()
    # Big bbox -> close. err_px=10, ex=0.06 -> epx=19.2px (out of band).
    # Proportional 0.06*60 = 3.6% (pwm 1514) -- below the 5% floor; close so the
    # floor engages -> must floor to 5% (1520).
    _align(_FakeVision(_sample(ex=0.06, w_frac=0.6, h_frac=0.6)), pix=pix,
           axes={'yaw'}, kp_yaw=60.0, gain=30.0, err_px=10.0, duration=0.25)
    floor_pwm = _FakePixhawk.percent_to_pwm(VISION_YAW_MIN_PCT)   # 1520
    yaw = [c['yaw'] for c in pix.rc if c.get('yaw', 1500) != 1500]
    assert yaw, 'expected yaw thrust on a small out-of-band error'
    assert min(yaw) >= floor_pwm, (
        'close-up small out-of-band yaw must be floored to the spin-up minimum')


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


def test_align_per_axis_gain_depth_scales_nudge():
    # depth setpoint excursion must scale with gain_depth (it drives max_nudge),
    # independently of the global gain. Larger gain_depth -> larger depth move
    # over the same run. (Cadence-independent: compares total excursion.)
    def _depth_excursion(g_depth):
        pix = _FakePixhawk()
        _align(_FakeVision(_sample(ey=1.0, w_frac=0.3, h_frac=0.3)), pix=pix,
               axes={'depth'}, kp_depth=0.05, gain=40.0, gain_depth=g_depth,
               err_px=10.0, duration=0.4)
        return abs(pix.depths[-1] - (-0.5)) if pix.depths else 0.0
    small = _depth_excursion(10.0)
    large = _depth_excursion(50.0)
    assert large > small > 0.0, (
        'gain_depth must scale the depth nudge (50 deepens more than 10)')


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
    _align(_FakeVision(_sample(ex=1.0, age_s=VISION_FRESH_ZERO_S + 0.05)), pix=pix,
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
        _FakeVision(_sample(ex=0.0, h_frac=0.1, age_s=VISION_FRESH_ZERO_S)),
        axes={'lat'}, fwd_fill=0.5, fwd_mode='height', duration=0.3)
    assert all(c.get('forward', 1500) == 1500 for c in pix.rc), \
        "a stale sample must not drive forward (freshness-decayed to neutral)"
