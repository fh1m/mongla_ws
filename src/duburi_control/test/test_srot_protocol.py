"""Pure-function tests for the SROT wire-constant module. No ROS/MAVLink needed."""

import pytest

from duburi_control.fc import srot_protocol as sp


# --------------------------------------------------------------------------- #
#  MANUAL_CONTROL scaling -- the sign/clamp math a units bug would break       #
# --------------------------------------------------------------------------- #
def test_unit_to_mc_endpoints_and_centre():
    assert sp.unit_to_mc(0.0) == 0
    assert sp.unit_to_mc(1.0) == 1000
    assert sp.unit_to_mc(-1.0) == -1000


def test_unit_to_mc_clamps_out_of_range():
    # The board silently truncates; we must clamp so an over-range value
    # under-drives (to the rail) rather than wrapping or bursting.
    assert sp.unit_to_mc(5.0) == 1000
    assert sp.unit_to_mc(-5.0) == -1000


def test_unit_to_mc_z_neutral_is_500_and_up_is_high():
    # z is 0..1000 with 500 neutral; +up (ascend) must map ABOVE 500, not below.
    assert sp.unit_to_mc_z(0.0) == 500
    assert sp.unit_to_mc_z(1.0) == 1000      # full ascend
    assert sp.unit_to_mc_z(-1.0) == 0        # full descend
    assert sp.unit_to_mc_z(0.5) == 750


def test_unit_to_mc_z_clamps():
    assert sp.unit_to_mc_z(9.0) == 1000
    assert sp.unit_to_mc_z(-9.0) == 0


def test_pct_helpers_match_unit_helpers():
    # The percent API (-100..100) and the unit API (-1..1) must agree at scale.
    assert sp.pct_to_mc(50.0) == sp.unit_to_mc(0.5) == 500
    assert sp.pct_to_mc_z(100.0) == sp.unit_to_mc_z(1.0) == 1000
    assert sp.pct_to_mc_z(0.0) == 500
    assert sp.pct_to_mc(-100.0) == -1000


# --------------------------------------------------------------------------- #
#  Mode name/int round-trips                                                    #
# --------------------------------------------------------------------------- #
def test_mode_name_known_and_unknown():
    assert sp.mode_name(sp.MODE_AUTO) == 'AUTO'
    assert sp.mode_name(sp.MODE_DEPTH_HOLD) == 'DEPTH_HOLD'
    assert sp.mode_name(99).startswith('UNKNOWN')
    assert sp.mode_name(None) == 'UNKNOWN'


def test_mode_int_round_trip_and_case_insensitive():
    for num, name in sp.MODE_NAMES.items():
        assert sp.mode_int(name) == num
    assert sp.mode_int('auto') == sp.MODE_AUTO       # case-insensitive
    assert sp.mode_int('nonsense') is None


# --------------------------------------------------------------------------- #
#  Move-type codes -- the wire contract                                         #
# --------------------------------------------------------------------------- #
def test_move_type_codes_are_the_wire_values():
    # Pinned to JETSON_COMMS.md §5 p1 table -- a drift here silently sends the
    # wrong primitive (e.g. dive instead of forward).
    assert (sp.MOVE_FORWARD, sp.MOVE_BACK, sp.MOVE_STRAFE_L, sp.MOVE_STRAFE_R,
            sp.MOVE_TURN, sp.MOVE_DIVE, sp.MOVE_STOP, sp.MOVE_HOLD,
            sp.MOVE_STYLE, sp.MOVE_ARC) == (0, 1, 2, 3, 4, 5, 6, 7, 8, 9)


def test_valid_move_types_is_zero_to_nine():
    assert sp.VALID_MOVE_TYPES == frozenset(range(10))
    assert 10 not in sp.VALID_MOVE_TYPES and -1 not in sp.VALID_MOVE_TYPES


def test_cmd_ids():
    assert sp.CMD_SROT_MOVE == 31000
    assert sp.CMD_USER_1 == 31010     # style_yaw -> yaw spin


def test_mv_state_names():
    assert sp.mv_state_name(sp.MV_CRUISE) == 'cruise'
    assert sp.mv_state_name(sp.MV_DONE) == 'done'


# --------------------------------------------------------------------------- #
#  Speed sanitisation                                                           #
# --------------------------------------------------------------------------- #
def test_sanitize_speed_clamps_to_cruise_max():
    assert sp.sanitize_speed(0.5) == 0.5
    assert sp.sanitize_speed(0.99) == sp.MOVE_CRUISE_MAX     # over cruise max -> capped
    assert sp.sanitize_speed(-0.2) == 0.0                    # negative -> 0
