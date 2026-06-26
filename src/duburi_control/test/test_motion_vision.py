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
    align_loop, move_loop, _fill, _clamp, _present,
    ALIGNED, LOST, TIMEOUT, NO_CAMERA,
)


# --------------------------------------------------------------------------- #
#  Fakes                                                                       #
# --------------------------------------------------------------------------- #
def _sample(ex=0.0, ey=0.0, w_frac=0.0, h_frac=0.0, age_s=0.0):
    return SimpleNamespace(ex=ex, ey=ey, w_frac=w_frac, h_frac=h_frac, age_s=age_s)


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

    def bbox_error(self, _cls):
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
    def __init__(self):
        self.neutralised = 0

    def neutral(self):
        self.neutralised += 1


class _Log:
    def info(self, *_a, **_k): pass
    def warning(self, *_a, **_k): pass
    def error(self, *_a, **_k): pass


class _CapLog:
    """Capturing logger for assertions on the no-match diagnostic."""
    def __init__(self):
        self.warnings = []
        self.infos = []

    def info(self, msg, *_a, **_k): self.infos.append(str(msg))
    def warning(self, msg, *_a, **_k): self.warnings.append(str(msg))
    def error(self, *_a, **_k): pass


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
    assert any('not among live detections' in w for w in log.warnings)


def test_align_converges_when_centered():
    # Centered target (ex=ey=0) is within err every tick -> ALIGNED after
    # align_stable_frames, well inside the duration budget.
    out, _, writers = _align(_FakeVision(_sample(ex=0.0, ey=0.0)))
    assert out.code == ALIGNED
    assert out.last_err_px <= 40.0
    assert writers.neutralised >= 1


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
