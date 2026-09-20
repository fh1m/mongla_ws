"""The RPM -> body-acceleration model. All of it testable with no thrusters.

Every assertion here is arithmetic or a refusal, which is the point: this is
the half of the thruster work that CAN be verified on a bench, and it is
verified rather than deferred to a pool.
"""
import math

import pytest

from mongla_manager.estimator.thrust_model import (
    REVERSE_EFFICIENCY, body_accel_from_rpm)

K = 2.0e-6          # an arbitrary POSITIVE k -- these tests check shape, not scale
M = 20.0


def _a(rpm, k=K, mass=M):
    return body_accel_from_rpm(rpm, k_n_per_rpm2=k, mass_kg=mass)


# ── refusals: absence is never zero ──────────────────────────────────────── #

def test_an_UNSET_k_REFUSES_rather_than_guessing():
    """⛔ The load-bearing refusal. No absolute N-per-RPM^2 figure exists in our
    stack OR the firmware, so a default would be invented, and it multiplies
    every acceleration the estimator sees."""
    assert _a([1000] * 8, k=None) is None
    assert _a([1000] * 8, k=0.0) is None


def test_ABSENT_rpm_REFUSES():
    assert _a(None) is None


def test_a_PARTIAL_horizontal_ring_REFUSES():
    """A missing motor's contribution is UNKNOWN, not zero. Summing three of
    four thrusters yields a confident, wrong, and perfectly plausible force."""
    assert _a([1000, 1000, None, 1000]) is None
    assert _a([1000, 1000]) is None


def test_a_ZERO_ring_is_ZERO_acceleration_not_a_refusal():
    """Distinct from absence: four thrusters REPORTING zero really are still."""
    assert _a([0, 0, 0, 0, 0, 0, 0, 0]) == (0.0, 0.0)


# ── geometry ─────────────────────────────────────────────────────────────── #

def test_the_45_DEGREE_PROJECTION_is_applied_not_the_mixer_S_PLUS_MINUS_ONE():
    """⛔ The sqrt(2) trap. mixer.cpp's matrix is a normalised DEMAND mix whose
    entries are +/-1; the thrusters sit at 45 deg. A force sum that reuses the
    matrix entries overreads surge and sway by 41 % and looks fine.

    Pure forward: motors 3,4 ahead and 1,2 astern (fwd signs -1,-1,+1,+1).
    """
    n = 1000.0
    a_fwd, a_lat = _a([-n, -n, n, n])
    t = K * n * n
    # each of the four contributes +t*cos45 to forward; the two reversed ones
    # are weaker by REVERSE_EFFICIENCY.
    expect = (2 * t + 2 * t * REVERSE_EFFICIENCY) * math.cos(math.radians(45)) / M
    assert a_fwd == pytest.approx(expect)
    assert a_lat == pytest.approx(0.0, abs=1e-12)
    naive = (2 * t + 2 * t * REVERSE_EFFICIENCY) / M          # the +/-1 version
    assert a_fwd < naive, 'must not reuse the mixer demand entries as geometry'
    assert naive / a_fwd == pytest.approx(math.sqrt(2), rel=1e-9)


def test_VERTICAL_thrusters_contribute_NOTHING_to_surge_or_sway():
    """Motors 5-8 are the vertical block. Spinning them must not move surge."""
    quiet = _a([0, 0, 0, 0, 0, 0, 0, 0])
    loud = _a([0, 0, 0, 0, 3000, -3000, 3000, -3000])
    assert quiet == loud == (0.0, 0.0)


def test_surge_and_sway_are_INDEPENDENT_axes():
    n = 1000.0
    _, lat = _a([-n, -n, n, n])                 # pure forward mix
    fwd, _ = _a([n, -n, n, -n])                 # pure lateral mix
    assert lat == pytest.approx(0.0, abs=1e-12)
    assert fwd == pytest.approx(0.0, abs=1e-12)


# ── the signed reading, which is why ESC_STATUS is worth CRC-checking ────── #

def test_a_REVERSED_prop_SUBTRACTS_and_is_WEAKER():
    """Unsigned magnitude cannot express either fact. Both matter: a prop spun
    backwards by the wash and one driven backwards read identically without
    the sign, and reverse thrust is only ~77 % of forward."""
    n = 1000.0
    all_fwd, _ = _a([-n, -n, n, n])
    one_flipped, _ = _a([-n, -n, n, -n])        # motor 4 now turning backwards
    assert one_flipped < all_fwd, 'a reversed prop must subtract'

    # and the asymmetry itself: +n on a +1 motor vs -n on a -1 motor are NOT
    # equal in magnitude, they differ by REVERSE_EFFICIENCY.
    fwd_only, _ = _a([0, 0, n, 0])
    rev_only, _ = _a([0, 0, -n, 0])
    assert abs(rev_only) == pytest.approx(abs(fwd_only) * REVERSE_EFFICIENCY)


def test_thrust_is_QUADRATIC_in_rpm():
    n = 500.0
    a1, _ = _a([-n, -n, n, n])
    a2, _ = _a([-2 * n, -2 * n, 2 * n, 2 * n])
    assert a2 == pytest.approx(4.0 * a1), 'thrust ~ RPM^2'


def test_acceleration_scales_INVERSELY_with_mass():
    n = 1000.0
    light, _ = _a([-n, -n, n, n], mass=10.0)
    heavy, _ = _a([-n, -n, n, n], mass=20.0)
    assert light == pytest.approx(2.0 * heavy)


def test_nothing_in_the_stack_CALLS_this_yet():
    """Deliberate, and pinned so the wiring is a decision rather than a drift.

    No non-zero RPM has ever crossed this wire (958/958 recorded frames are
    exactly 0), so a caller today would feed the estimator a confident zero.
    When thrusters arrive and k is measured, delete this test in the same
    commit that wires it up.
    """
    import pathlib
    root = pathlib.Path(__file__).resolve().parents[3]
    hits = [p for p in root.rglob('src/**/*.py')
            if 'thrust_model' in p.read_text()
            and p.name not in ('thrust_model.py', 'test_thrust_model.py')]
    assert hits == [], f'thrust_model now has callers: {hits}'
