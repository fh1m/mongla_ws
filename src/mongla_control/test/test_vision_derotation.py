"""A box is where the target WAS. The hull has kept turning since.

Scored against INJECTED TRUTH, not against the incumbent: a world-fixed target
seen through a camera that yaws at a known rate, with a known capture age. The
uncorrected error is the phantom offset; the corrected one must be near zero.
"""
import math

from mongla_control.motion_vision import _derotated_ex, DEROTATE_MAX_AGE_S

from test_motion_vision import _FakeVision, _FakePixhawk, _sample, _align


PX_PER_RAD = 741.0 * 1.333      # forward camera fx in air x n_water
HALF_W = 320.0


def _true_ex_now(ex_capture, yaw_rate, age_s):
    """A world-fixed target: turning RIGHT by dtheta moves it LEFT in the image."""
    theta_cap = math.atan(ex_capture * HALF_W / PX_PER_RAD)
    return PX_PER_RAD * math.tan(theta_cap - yaw_rate * age_s) / HALF_W


def test_corrected_bearing_matches_truth_and_raw_does_not():
    ex_cap, w, age = 0.10, 0.5, 0.030
    truth = _true_ex_now(ex_cap, w, age)
    raw_err_px = abs(ex_cap - truth) * HALF_W
    fixed_err_px = abs(_derotated_ex(ex_cap, age, w, PX_PER_RAD, HALF_W) - truth) * HALF_W
    assert raw_err_px > 10.0, raw_err_px          # the phantom this removes
    assert fixed_err_px < 1.0, fixed_err_px


def test_sign_turning_right_moves_the_target_left():
    assert _derotated_ex(0.0, 0.05, +0.5, PX_PER_RAD, HALF_W) < 0.0
    assert _derotated_ex(0.0, 0.05, -0.5, PX_PER_RAD, HALF_W) > 0.0


def test_absent_or_invalid_inputs_are_the_exact_identity():
    for ex, age, w in [(0.2, 0.0, 0.5), (0.2, 0.05, float('nan')), (0.2, -0.1, 0.5)]:
        assert _derotated_ex(ex, age, w, PX_PER_RAD, HALF_W) == ex
    assert _derotated_ex(0.2, 0.05, 0.5, 0.0, HALF_W) == 0.2       # no calibration
    assert _derotated_ex(0.2, 0.05, 0.5, PX_PER_RAD, 0.0) == 0.2


def test_age_is_capped_so_a_stale_box_is_not_extrapolated_forever():
    far = _derotated_ex(0.0, 5.0, 0.5, PX_PER_RAD, HALF_W)
    cap = _derotated_ex(0.0, DEROTATE_MAX_AGE_S, 0.5, PX_PER_RAD, HALF_W)
    assert far == cap


# --------------------------------------------------------------------------- #
#  Through the loop: a correction that never reaches the command is nothing.  #
# --------------------------------------------------------------------------- #
def _lat_cmds(pix):
    return [kw.get('lateral') for kw in pix.translations + pix.rc
            if kw.get('lateral') not in (None, 1500)]


def _run(derotate_fn, sample, **kw):
    pix = _FakePixhawk()
    _align(_FakeVision(sample), pix=pix, axes={'lat'}, err_px=5.0,
           duration=0.15, derotate_fn=derotate_fn, **kw)
    return _lat_cmds(pix)


def test_the_loop_steers_on_the_derotated_bearing():
    # Target captured 60 px right, 50 ms ago, hull yawing right at 2.5 rad/s:
    # it is now left of centre, so a derotated loop must strafe LEFT.
    s = _sample(ex=60.0 / HALF_W, age_s=0.05)
    raw = _run(None, s)
    fixed = _run(lambda: (2.5, PX_PER_RAD), s)
    assert raw and fixed
    assert raw[0] > 1500, raw[:3]            # uncorrected: strafe right
    assert fixed[0] < 1500, fixed[:3]        # corrected: strafe left


def test_a_coasted_box_is_not_derotated_twice():
    """The tracker already extrapolates a coasted box; correcting it again
    counts the same rotation twice."""
    s = _sample(ex=60.0 / HALF_W, age_s=0.05, coasted=True)
    raw = _run(None, s, coast_s=1.0)
    assert raw, 'the coasted box never reached the command -- test is vacuous'
    assert _run(lambda: (2.5, PX_PER_RAD), s, coast_s=1.0) == raw


def test_resolver_returning_none_or_raising_changes_nothing():
    s = _sample(ex=60.0 / HALF_W, age_s=0.05)
    raw = _run(None, s)

    def boom():
        raise RuntimeError('no attitude')
    assert _run(lambda: None, s) == raw
    assert _run(boom, s) == raw


# --------------------------------------------------------------------------- #
#  The inputs, and that the verb actually passes them.                         #
# --------------------------------------------------------------------------- #
from pathlib import Path                                         # noqa: E402

from mongla_control.vision_verbs import _derotation_inputs       # noqa: E402


class _FC:
    def __init__(self, rates): self._r = rates
    def get_angular_rates(self): return self._r


class _VS:
    def __init__(self, ppr): self._p = ppr
    def px_per_rad(self): return self._p


def test_inputs_pass_through_when_fresh():
    fc = _FC({'yaw_rate': 0.4, 'age_s': 0.01})
    assert _derotation_inputs(fc, _VS(988.0)) == (0.4, 988.0)


def test_unhealthy_stale_or_uncalibrated_means_no_correction():
    assert _derotation_inputs(_FC(None), _VS(988.0)) is None           # dead BNO
    assert _derotation_inputs(_FC({'yaw_rate': 0.4, 'age_s': 0.5}),
                              _VS(988.0)) is None                      # stale
    assert _derotation_inputs(_FC({'yaw_rate': 0.4, 'age_s': 0.01}),
                              _VS(None)) is None                       # no K


def test_the_verb_passes_derotation_into_the_loop():
    src = (Path(__file__).resolve().parents[1] / 'mongla_control'
           / 'vision_verbs.py').read_text()
    i = src.index('outcome = align_loop(')
    assert 'derotate_fn=' in src[i:i + 6000], 'align_loop is called without it'
