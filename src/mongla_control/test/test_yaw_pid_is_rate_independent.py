"""B13 -- YAW_KD and YAW_KI were per-TICK, silently coupled to YAW_RATE_HZ.

    d_term = YAW_KD * (error_deg - self._last_e)          # no dt
    self._i_acc += YAW_KI * error_deg                     # no dt either

Changing the loop rate therefore retuned the controller without anyone touching a
gain. The comment above YAW_KD even claimed "Uses 1/YAW_RATE_HZ as dt", which the
expression did not do.

THE GAINS ARE POOL-TUNED, so they are ANCHORED to the rate they were tuned at
rather than rewritten in per-second units: at `dt == _YAW_DT_REF` both expressions
reduce exactly to what shipped, while a rate change now preserves behaviour
instead of altering it. Same approach as the B09 Kalman-Q fix.

Also fixed here: `_last_e = 0.0` manufactured a first-tick derivative of the full
error (0.5 x 90 = 45 % for a 90 deg turn). The clamp masked it; it did not prevent
it -- which is why the first-tick OUTPUT is unchanged and only the internal term
differs.
"""

import math

import pytest

import mongla_control.motion_yaw as my


class _Shipped:
    """Faithful copy of the pre-fix code, to prove the tune is preserved."""
    def __init__(self):
        self._i_acc, self._last_e = 0.0, 0.0

    def update(self, e):
        if abs(e) <= my.YAW_TOL_DEG:
            self._i_acc = 0.0
            self._last_e = e
            return 0.0
        if math.copysign(1, e) != math.copysign(1, self._last_e):
            self._i_acc = 0.0
        d = my.YAW_KD * (e - self._last_e)
        self._i_acc = max(-my.YAW_KI_MAX,
                          min(my.YAW_KI_MAX, self._i_acc + my.YAW_KI * e))
        raw = my.YAW_KP * e + self._i_acc + d
        speed = max(my._yaw_floor(abs(e)), min(my.YAW_SPEED_MAX_PCT, abs(raw)))
        self._last_e = e
        return math.copysign(speed, raw)


_APPROACH = [90, 80, 60, 40, 25, 12, 6, 3, 1.5]


def test_the_pool_tune_is_preserved_exactly_at_the_shipped_rate():
    """A rate-independence fix that changes today's behaviour is a retune."""
    new, old = my._YawPID(), _Shipped()
    new.update(_APPROACH[0]); old.update(_APPROACH[0])     # past the first tick
    for e in _APPROACH[1:]:
        assert new.update(e) == pytest.approx(old.update(e), abs=1e-12), \
            f'behaviour changed at error {e} deg -- this is a retune, not a fix'


def test_the_first_tick_no_longer_manufactures_a_derivative():
    """`_last_e = 0.0` made the first D term the FULL error."""
    pid = my._YawPID()
    assert pid._last_e is None, 'the first tick must have no previous sample'
    pid.update(90.0)
    assert pid._last_e == 90.0


def test_the_derivative_is_scaled_by_dt_not_by_tick():
    """Halve the period and the D contribution must not silently double."""
    a, b = my._YawPID(), my._YawPID()
    a._last_e = b._last_e = 80.0
    assert a.update(60.0, dt=my._YAW_DT_REF) == pytest.approx(
        b.update(60.0, dt=my._YAW_DT_REF / 2.0), abs=1e-9)


def test_the_integral_is_scaled_by_dt_too():
    """Both terms had the defect; fixing only D would leave I rate-coupled."""
    slow, fast = my._YawPID(), my._YawPID()
    slow._last_e = fast._last_e = 30.0
    for _ in range(5):
        slow.update(30.0, dt=my._YAW_DT_REF)
    for _ in range(10):
        fast.update(30.0, dt=my._YAW_DT_REF / 2.0)
    assert slow._i_acc == pytest.approx(fast._i_acc, rel=1e-9), \
        'twice the ticks at half the period must integrate to the same place'


def test_a_non_positive_dt_falls_back_to_the_reference():
    """A zero dt from a stalled clock must not divide by zero."""
    pid = my._YawPID()
    pid._last_e = 80.0
    assert math.isfinite(pid.update(60.0, dt=0.0))
