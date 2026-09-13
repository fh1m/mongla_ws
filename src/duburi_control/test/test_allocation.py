"""The mixer's scale-down, computed host-side -- and pinned to the firmware.

⛔ WHY THIS MATTERS TO AUTONOMY, not just to telemetry. The board scales a
saturating demand and never reports it, so the host commands a demand the hull
does not receive, sees no response, and pushes harder. That is integrator
wind-up against an actuator limit, and it bites worst in a close-in vision
alignment where `ki_lat` is integrating a lateral error against an already
clipped horizontal group.

The worked example from the firmware's own comment is the test below:
forward = 1.0 with yaw = 0.5 drives motor 2 to -1.5, a scale of 0.667.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import pytest                                              # noqa: E402

from duburi_control.allocation import (                    # noqa: E402
    MIXER, Allocation, allocate, headroom, largest_axis_within_budget,
)

def _firmware_mixer() -> Path:
    """Find `mixer.cpp` beside the workspace, wherever the checkout sits.

    Walking up rather than counting `parents[N]`: the count was wrong on the
    first attempt and the test SKIPPED silently, which is the worst outcome
    for a drift guard -- it reports success while checking nothing.
    """
    here = Path(__file__).resolve()
    rel = Path('Mongla_others/srot-control-board/src/control/mixer.cpp')
    for base in here.parents:
        cand = base / rel
        if cand.exists():
            return cand
    return here / '__missing__'


FW = _firmware_mixer()


# --------------------------------------------------------------------------- #
#  one truth, two copies
# --------------------------------------------------------------------------- #
def test_the_matrix_matches_the_firmware_source():
    """⛔ TWO COPIES OF ONE FACT. The frame mix lives in the firmware and here.
    A silent divergence would make every prediction in this module confidently
    wrong, in the same way a drifted wire constant would -- which is why
    `test_srot_protocol_drift.py` exists for those. Same discipline."""
    if not FW.exists():
        pytest.skip('firmware source not checked out beside the workspace')
    text = FW.read_text()
    start = text.index('static const float M[NUM_THRUSTERS][6]')
    body = text[start:text.index('};', start)]
    rows = []
    for line in body.splitlines():
        line = line.strip()
        if not line.startswith('{'):
            continue
        nums = line[1:line.index('}')].split(',')
        rows.append(tuple(float(n.strip()) for n in nums))
    assert len(rows) == 8, f'parsed {len(rows)} rows'
    # tuple() BOTH SIDES. The first version compared a list against a tuple
    # of tuples, which is never equal in Python -- so the guard failed for a
    # type reason while the matrices were identical. A drift test that cries
    # wolf gets disabled, which is how a real drift then ships.
    assert tuple(rows) == tuple(MIXER), 'the host mixer matrix has drifted from the board'


def test_the_horizontal_group_is_the_first_four_motors():
    """Block-diagonal: motors 1-4 carry no roll, pitch or throttle."""
    for m in range(4):
        assert MIXER[m][0] == 0.0 and MIXER[m][1] == 0.0 and MIXER[m][3] == 0.0
    for m in range(4, 8):
        assert MIXER[m][2] == 0.0 and MIXER[m][4] == 0.0 and MIXER[m][5] == 0.0


# --------------------------------------------------------------------------- #
#  the arithmetic
# --------------------------------------------------------------------------- #
def test_a_small_demand_is_delivered_in_full():
    a = allocate(forward=0.3, lateral=0.2)
    assert a.horizontal_scale == 1.0
    assert a.vertical_scale == 1.0
    assert not a.saturated
    assert a.limiting_group() == ''


def test_the_firmwares_own_worked_example():
    """forward = 1.0 with yaw = 0.5 drives motor 2 to -1.5 -> scale 0.667."""
    a = allocate(forward=1.0, yaw=0.5)
    assert a.horizontal_scale == pytest.approx(1.0 / 1.5, abs=1e-9)
    assert a.saturated
    assert a.limiting_group() == 'horizontal'


def test_saturation_STEALS_FROM_THE_NEIGHBOURING_AXIS():
    """⛔ THE PROPERTY A CONTROLLER MUST KNOW. The scale is applied to the whole
    GROUP, so an over-large lateral demand does not merely clip itself -- it
    shrinks the yaw a heading lock is relying on, in the same frame."""
    a = allocate(yaw=0.5, lateral=1.0)
    assert a.saturated
    assert abs(a.delivered[2]) < 0.5, 'yaw survived a lateral saturation'


def test_the_two_groups_saturate_INDEPENDENTLY():
    """This was a real firmware bug: one global scale coupled two mechanically
    independent groups, so a hard forward burst silently cost a third of the
    vehicle's roll and pitch authority."""
    a = allocate(forward=1.0, yaw=0.5, roll=0.2)
    assert a.horizontal_scale < 1.0
    assert a.vertical_scale == 1.0
    assert a.delivered[0] == pytest.approx(0.2), 'roll was scaled by a horizontal limit'


def test_a_vertical_saturation_leaves_the_horizontals_alone():
    a = allocate(roll=1.0, pitch=1.0, throttle=1.0, forward=0.4)
    assert a.vertical_scale < 1.0
    assert a.horizontal_scale == 1.0
    assert a.delivered[4] == pytest.approx(0.4)


def test_zero_demand_is_not_reported_as_saturated():
    a = allocate()
    assert not a.saturated and a.worst_scale == 1.0


def test_both_groups_can_saturate_at_once():
    a = allocate(forward=1.0, yaw=1.0, roll=1.0, pitch=1.0, throttle=1.0)
    assert a.limiting_group() == 'both'


# --------------------------------------------------------------------------- #
#  headroom: what a controller should ask before integrating
# --------------------------------------------------------------------------- #
def test_headroom_is_one_exactly_at_the_limit():
    h, _ = headroom(forward=1.0)
    assert h == pytest.approx(1.0)


def test_headroom_halves_when_the_demand_doubles():
    h1, _ = headroom(forward=0.25)
    h2, _ = headroom(forward=0.5)
    assert h1 == pytest.approx(2.0 * h2)


def test_headroom_is_infinite_for_an_idle_group():
    h, v = headroom(forward=0.5)
    assert v == float('inf')


# --------------------------------------------------------------------------- #
#  the budget a vision loop should respect
# --------------------------------------------------------------------------- #
def test_the_largest_lateral_shrinks_as_yaw_spends_authority():
    """A vision loop correcting laterally while a heading lock holds yaw can
    only have what is left. Asking for more steals from the lock."""
    free = largest_axis_within_budget('lateral')
    with_yaw = largest_axis_within_budget('lateral', others={'yaw': 0.5})
    assert free == pytest.approx(1.0)
    assert with_yaw == pytest.approx(0.5)


def test_the_budget_never_goes_negative_or_past_full():
    for yaw in (0.0, 0.5, 1.0, 1.5):
        got = largest_axis_within_budget('lateral', others={'yaw': yaw})
        assert 0.0 <= got <= 1.0


def test_a_demand_at_the_budget_does_NOT_saturate():
    """The contract that makes it usable: take the budget, stay unclipped."""
    for yaw in (0.0, 0.2, 0.5, 0.8):
        lat = largest_axis_within_budget('lateral', others={'yaw': yaw})
        a = allocate(yaw=yaw, lateral=lat)
        assert a.horizontal_scale == pytest.approx(1.0, abs=1e-9), (yaw, lat)


def test_an_unknown_axis_raises_rather_than_returning_a_number():
    with pytest.raises(KeyError):
        largest_axis_within_budget('sideways')


# --------------------------------------------------------------------------- #
#  prioritised allocation: sacrifice the least important axis, not all of them
# --------------------------------------------------------------------------- #
def test_uniform_scaling_COSTS_MORE_THAN_HALF_THE_YAW():
    """⛔ THE MEASUREMENT THAT JUSTIFIES THE WHOLE IDEA.

    Ask for forward 0.9, lateral 0.9, yaw 0.3 -- a hard corner with a heading
    to hold. The board scales the group uniformly by 0.476, so yaw arrives at
    0.143: over half the heading authority gone, in the manoeuvre that needs it
    most. Prioritising keeps yaw whole and spends the shortfall on forward,
    which is the axis whose error is merely 'late' rather than 'pointing the
    tool at the wrong place'.
    """
    ask = {'forward': 0.9, 'lateral': 0.9, 'yaw': 0.3}
    uniform = allocate(**ask)
    assert uniform.delivered[2] < 0.15, 'the board should be clipping yaw hard'

    from duburi_control.allocation import prioritise
    kept = allocate(**prioritise(ask))
    assert kept.delivered[2] == pytest.approx(0.3), 'yaw was not protected'
    assert not kept.saturated, 'the prioritised demand should fit exactly'


def test_a_prioritised_demand_never_saturates():
    """The contract. If it still clipped, the board would scale it uniformly
    again and the prioritisation would be undone."""
    from duburi_control.allocation import prioritise

    for ask in ({'forward': 1.0, 'yaw': 0.5},
                {'lateral': 0.8, 'yaw': 0.6},
                {'forward': 1.0, 'lateral': 1.0, 'yaw': 1.0},
                {'roll': 1.0, 'pitch': 1.0, 'throttle': 1.0}):
        a = allocate(**prioritise(ask))
        assert a.horizontal_scale == pytest.approx(1.0, abs=1e-9), ask
        assert a.vertical_scale == pytest.approx(1.0, abs=1e-9), ask


def test_a_feasible_demand_is_returned_UNCHANGED():
    """Prioritising must cost nothing when nothing is saturated, or every
    ordinary manoeuvre pays for a case that is not happening."""
    from duburi_control.allocation import prioritise

    ask = {'forward': 0.3, 'yaw': 0.2, 'lateral': 0.1}
    got = prioritise(ask)
    for k, v in ask.items():
        assert got[k] == pytest.approx(v), k


def test_the_sign_of_every_axis_survives():
    """A sacrificed axis must be reduced, never reversed. Reversing thrust on
    an axis the mission asked to push is the worst possible failure."""
    from duburi_control.allocation import prioritise

    got = prioritise({'forward': -1.0, 'lateral': -0.9, 'yaw': 0.4})
    assert got['forward'] <= 0.0 and got['lateral'] <= 0.0
    assert got['yaw'] >= 0.0


def test_no_axis_is_ever_INCREASED():
    """Prioritising redistributes what was asked for; it must never invent
    authority the mission did not request."""
    from duburi_control.allocation import prioritise

    ask = {'forward': 0.9, 'lateral': 0.9, 'yaw': 0.3}
    got = prioritise(ask)
    for k, v in ask.items():
        assert abs(got[k]) <= abs(v) + 1e-9, k


def test_the_groups_are_prioritised_INDEPENDENTLY():
    """A vertical saturation must not cost the horizontals anything, and the
    reverse -- the same block-diagonal property the mixer has."""
    from duburi_control.allocation import prioritise

    got = prioritise({'roll': 1.0, 'pitch': 1.0, 'throttle': 1.0,
                      'forward': 0.4, 'yaw': 0.2})
    assert got['forward'] == pytest.approx(0.4)
    assert got['yaw'] == pytest.approx(0.2)


def test_the_input_dict_is_not_mutated():
    """A caller must be able to log what it asked for beside what it sent."""
    from duburi_control.allocation import prioritise

    ask = {'forward': 1.0, 'yaw': 0.5}
    prioritise(ask)
    assert ask == {'forward': 1.0, 'yaw': 0.5}
