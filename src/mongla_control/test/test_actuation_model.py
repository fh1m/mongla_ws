"""The actuation model, against what the board actually produced.

TRUTH test, not an agreement test: the expected numbers below were read off the
RP2350 console on 2026-09-22 with the board armed, nothing attached to any ESC,
and eleven demand levels commanded one at a time. Checking the model against
itself would prove only that the arithmetic is self-consistent.

The capture is kept at `.claude/context/workbench/data/mixer_ladder.json`, so
these can be re-derived rather than trusted.
"""
import json
import math
from pathlib import Path

import pytest

from mongla_control.actuation_model import (
    CENTRE_EPS, DEF_MOT_SPIN_MIN, demand_to_fraction, dshot_signed,
    fraction_to_demand, inverse_thrust_expo, min_fraction, pilot_expo,
    thrust_expo)

LADDER = (Path(__file__).resolve().parents[3]
          / '.claude/context/workbench/data/mixer_ladder.json')

# demand -> mean |DShot magnitude| over M1..M4, measured on the board
MEASURED = {0.02: 181, 0.05: 223, 0.08: 259, 0.10: 282, 0.15: 333, 0.20: 380,
            0.30: 463, 0.45: 578, 0.60: 689, 0.80: 839}


@pytest.mark.parametrize('demand,counts', sorted(MEASURED.items()))
def test_the_model_reproduces_what_the_board_did(demand, counts):
    """Within one count of 999. Full scale is excluded: the band saturates
    there and the board returns 996 against a predicted 999."""
    predicted = demand_to_fraction(demand) * 999.0

    assert abs(predicted - counts) <= 1.5, (
        f'demand {demand}: model says {predicted:.1f}, board did {counts}')


def test_the_capture_backing_these_numbers_is_still_in_the_tree():
    """If the raw data goes missing, the table above becomes folklore."""
    rows = json.loads(LADDER.read_text())['ladder']

    levels = {r['level'] for r in rows if r['cmd_median']}
    assert MEASURED.keys() <= levels


# ── the floor, which is the finding ──────────────────────────────────────────

def test_the_smallest_commandable_output_is_the_spin_floor_not_zero():
    """The point of the whole module: a tiny demand does NOT give a tiny output.
    `VISION_YAW_MIN_PCT = 5.0` was invented to cross a deadband that is not
    there; the real floor is three times higher and cannot be crossed at all."""
    tiny = demand_to_fraction(0.01)

    assert tiny >= DEF_MOT_SPIN_MIN
    assert tiny > 3 * 0.05, 'a 5 % demand does not produce 5 % of anything'


def test_a_truly_centred_demand_still_stops():
    """mixer.cpp:102 -- below the centre epsilon the firmware returns neutral
    outright, so the floor must not apply there. If this broke, every thruster
    would creep at 15 % whenever the vehicle was asked to hold still."""
    assert demand_to_fraction(CENTRE_EPS / 2) == 0.0
    assert demand_to_fraction(0.0) == 0.0


def test_the_achievable_minimum_is_above_spin_min_not_equal_to_it():
    """`spin_min` is the bottom of the lifted range, but the smallest demand that
    survives the centre test still passes through both expo curves on the way, so
    the achievable minimum is strictly higher. Returning `spin_min` here is a
    lower bound, and `fraction_to_demand` gates on this value -- so the bound
    would accept a request it then over-delivers."""
    assert min_fraction() > DEF_MOT_SPIN_MIN
    assert min_fraction() == pytest.approx(0.1583, abs=5e-4)


def test_a_request_between_spin_min_and_the_achievable_minimum_is_refused():
    """The exact gap the lower bound would have let through: 0.155 sits above
    `spin_min` (0.150) and below what the vehicle can actually produce (0.158)."""
    between = 0.5 * (DEF_MOT_SPIN_MIN + min_fraction())

    assert DEF_MOT_SPIN_MIN < between < min_fraction()
    assert fraction_to_demand(between) == 0.0


def test_the_minimum_itself_round_trips_exactly():
    """The boundary must be reachable, or the floor is reported one step above
    where it is and the smallest usable correction is lost."""
    assert demand_to_fraction(fraction_to_demand(min_fraction())) == pytest.approx(
        min_fraction(), abs=1e-9)


def test_asking_for_less_than_the_floor_is_refused_not_rounded():
    """Returning the smallest non-zero demand would silently deliver 3x what
    the caller asked for -- a plausible number for an impossible request."""
    assert fraction_to_demand(0.05) == 0.0
    assert fraction_to_demand(min_fraction() * 0.99) == 0.0


# ── the inverse ──────────────────────────────────────────────────────────────

@pytest.mark.parametrize('frac', [0.16, 0.25, 0.40, 0.60, 0.85, 1.00])
def test_the_inverse_round_trips(frac):
    assert demand_to_fraction(fraction_to_demand(frac)) == pytest.approx(
        frac, abs=1e-6)


def test_the_inverse_keeps_the_sign():
    assert fraction_to_demand(-0.5) == -fraction_to_demand(0.5)
    assert demand_to_fraction(-0.3) == -demand_to_fraction(0.3)


def test_thrust_expo_is_inverted_by_the_quadratic_it_solves():
    for thr in (0.0, 0.1, 0.5, 0.9, 1.0):
        assert thrust_expo(inverse_thrust_expo(thr)) == pytest.approx(thr, abs=1e-9)


def test_expo_of_zero_is_the_identity():
    """e = 0 is documented as linear in both curves; a shaping function that
    still shapes at zero would be silently wrong for a board configured flat."""
    assert pilot_expo(0.37, 0.0) == pytest.approx(0.37)
    assert thrust_expo(0.37, 0.0) == pytest.approx(0.37)


# ── the DShot decode, which cost the first reading of the mixer ─────────────

def test_both_dshot_bands_have_their_own_zero():
    assert dshot_signed(1048) == 0
    assert dshot_signed(48) == 0


def test_equal_demands_in_opposite_directions_decode_to_equal_magnitude():
    """Read off the board: a 0.02 fwd demand put M1 at 229 and M2 at 1230."""
    assert abs(abs(dshot_signed(229)) - abs(dshot_signed(1230))) <= 1


def test_a_nonfinite_demand_is_neutral_rather_than_an_exception():
    """This sits under a 20 Hz vision loop; one NaN must degrade to 'hold',
    not raise."""
    assert demand_to_fraction(float('nan')) == 0.0
    assert demand_to_fraction(math.inf) == 0.0
    assert fraction_to_demand(float('nan')) == 0.0
