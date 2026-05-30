"""Pure-math sanity checks for the vision-control axis mapping.

`motion_vision.vision_track_axes` is the most competition-relevant control
code and historically had zero direct coverage (see robosub-2026-audit §4).
A sign error in the yaw mapping or a bad vis_range guard is exactly what
bites in the pool, so the two decision helpers are pulled out as pure
functions and pinned here. No ROS / MAVLink needed.
"""

from types import SimpleNamespace

import pytest

from duburi_control.motion_vision import (
    _yaw_pct, _lat_pct, _forward_decision,
    YAW_PCT_MAX, LAT_PCT_MAX, FWD_PCT_MAX,
)


def _sample(**kw):
    """Minimal stand-in for the per-tick vision Sample."""
    base = dict(ex=0.0, ey=0.0, h_frac=0.0, w_frac=0.0, vis_range=0.0)
    base.update(kw)
    return SimpleNamespace(**base)


# --- yaw sign (Ch4 is inverted: >1500 = yaw LEFT) -----------------------------

def test_yaw_target_right_yields_negative_pct():
    # ex > 0 = target RIGHT -> need Ch4 < 1500 to yaw right -> NEGATIVE pct.
    assert _yaw_pct(0.5, 60.0) < 0.0


def test_yaw_target_left_yields_positive_pct():
    assert _yaw_pct(-0.5, 60.0) > 0.0


def test_yaw_centered_is_zero():
    assert _yaw_pct(0.0, 60.0) == 0.0


def test_yaw_clamps_to_max():
    # ex=1.0 * kp=60 = 60 -> clamped to YAW_PCT_MAX magnitude.
    assert _yaw_pct(1.0, 60.0) == -YAW_PCT_MAX
    assert _yaw_pct(-1.0, 60.0) == YAW_PCT_MAX


# --- lateral sign (Ch6 NOT inverted: >1500 = strafe RIGHT) --------------------

def test_lat_target_right_yields_positive_pct():
    # ex > 0 = target RIGHT -> Ch6 > 1500 strafes right -> POSITIVE pct.
    assert _lat_pct(0.5, 60.0) > 0.0


def test_lat_target_left_yields_negative_pct():
    assert _lat_pct(-0.5, 60.0) < 0.0


def test_lat_and_yaw_have_opposite_polarity():
    # The 1801fe2 bug class: yaw negates (Ch4 inverted), lateral does not.
    # For the same target-right error they MUST point opposite ways.
    ex = 0.4
    assert _yaw_pct(ex, 60.0) < 0.0   # yaw right via Ch4 < 1500
    assert _lat_pct(ex, 60.0) > 0.0   # strafe right via Ch6 > 1500


def test_lat_clamps_to_max():
    assert _lat_pct(1.0, 60.0) == LAT_PCT_MAX
    assert _lat_pct(-1.0, 60.0) == -LAT_PCT_MAX


# --- forward / vis_range guard ------------------------------------------------

def test_vis_range_zero_suppresses_forward():
    # No depth signal (vis_range defaults 0.0 when node offline):
    # forward must be 0.0 and distance_error None (= suppressed, not settled).
    pct, derr = _forward_decision(_sample(vis_range=0.0), 'vis_range',
                                  target_h_frac=0.65, kp_forward=200.0,
                                  lock_mode='settle')
    assert pct == 0.0
    assert derr is None


def test_vis_range_present_drives_forward():
    # Real depth reading below target -> positive error -> forward thrust.
    pct, derr = _forward_decision(_sample(vis_range=0.4), 'vis_range',
                                  target_h_frac=0.65, kp_forward=200.0,
                                  lock_mode='settle')
    assert derr == pytest.approx(0.25)
    assert pct > 0.0


def test_pursue_never_reverses_when_too_close():
    # vis_range above target = too close = negative error; pursue clamps to 0
    # (never back off), unlike settle which would reverse.
    pct, _ = _forward_decision(_sample(vis_range=0.9), 'vis_range',
                               target_h_frac=0.65, kp_forward=200.0,
                               lock_mode='pursue')
    assert pct == 0.0


def test_settle_reverses_when_too_close():
    pct, derr = _forward_decision(_sample(vis_range=0.9), 'vis_range',
                                  target_h_frac=0.65, kp_forward=200.0,
                                  lock_mode='settle')
    assert derr < 0.0
    assert pct < 0.0


def test_non_vis_range_metric_unaffected_by_guard():
    # height metric with a real bbox still drives (guard is vis_range-only).
    pct, derr = _forward_decision(_sample(h_frac=0.3), 'height',
                                  target_h_frac=0.65, kp_forward=200.0,
                                  lock_mode='settle')
    assert derr == pytest.approx(0.35)
    assert pct == pytest.approx(FWD_PCT_MAX)   # 0.35*200 clamps to max
