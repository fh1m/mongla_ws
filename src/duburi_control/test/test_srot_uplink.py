"""The vision uplink's wire contract: the class map and the timeout ladder.

Both were declared and unasserted. The class map because our two senders
disagreed about what `target_num` means, and the ladder because it is a
cross-repo contract written in prose in `VISION_API.md` and checked nowhere --
one edit breaks "authority reaches zero before loss is declared", which is what
makes a dropout a glide rather than a lurch.
"""
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from duburi_control.fc import srot_protocol as sp     # noqa: E402
from duburi_control.fc.srot_fc import SrotFC          # noqa: E402


class _Mav:
    def __init__(self):
        self.sent = []

    def __getattr__(self, name):
        if name.startswith('_'):
            raise AttributeError(name)

        def _send(*a, **k):
            self.sent.append((name, a, k))
        return _send


class _Master:
    def __init__(self):
        self.mav = _Mav()
        self.messages = {}


class _Bearing:
    def __init__(self, ax=0.1, ay=0.05, sx=0.2, sy=0.1):
        self.angle_x, self.angle_y = ax, ay
        self.size_x, self.size_y = sx, sy


def _send(**kw):
    fc = SrotFC(_Master(), log=None)
    fc.send_landing_target(_Bearing(), **kw)
    sent = [a for n, a, _ in fc.master.mav.sent if n == 'landing_target_send']
    return sent[-1] if sent else None


# --------------------------------------------------------------------------- #
#  The class map
# --------------------------------------------------------------------------- #
def test_zero_is_reserved_for_unspecified():
    """An unconfigured sender must read as "unassigned", not as whichever class
    happens to be numbered first."""
    assert sp.UPLINK_CLASS_UNSPECIFIED == 0
    assert 0 not in sp.UPLINK_CLASSES.values()


def test_the_map_is_injective():
    """Two classes sharing a number is the same defect as inserting one: the
    board cannot tell them apart and nothing errors."""
    nums = list(sp.UPLINK_CLASSES.values())
    assert len(nums) == len(set(nums))


def test_ids_are_contiguous_from_one_so_an_append_is_obvious():
    assert sorted(sp.UPLINK_CLASSES.values()) == list(
        range(1, sp.UPLINK_CLASS_MAX + 1))


def test_lookup_is_case_insensitive():
    """Detector labels and mission strings have drifted in case before."""
    assert sp.uplink_class_num('GATE') == sp.uplink_class_num('gate')
    assert sp.uplink_class_num('  Red_Pipe ') == sp.UPLINK_CLASSES['red_pipe']


def test_an_unmapped_class_sends_UNSPECIFIED_rather_than_raising():
    """The bearing is still correct and still useful. Refusing to uplink a
    target because nobody has assigned it a number yet would be a worse failure
    than an unlabelled one."""
    assert sp.uplink_class_num('a_class_nobody_registered') == 0
    assert sp.uplink_class_num('') == 0
    assert sp.uplink_class_num(None) == 0


def test_the_classes_a_2026_mission_actually_names_are_present():
    """These are the strings the competition chunks pass to `detected()`. A
    missing one silently degrades to 0 -- correct, but unlabelled."""
    for name in ('gate', 'red_pipe', 'bin', 'torpedo', 'hole', 'flare'):
        assert sp.uplink_class_num(name) > 0, name


# --------------------------------------------------------------------------- #
#  The timeout ladder
# --------------------------------------------------------------------------- #
def test_the_ladder_is_strictly_increasing():
    """THE contract. Each rung must expire strictly before the next so
    authority reaches zero BEFORE loss is declared -- that ordering is what
    makes a detection dropout a glide instead of a lurch. Asserted as the
    ordering rather than as four constants, so a deliberate retune stays legal
    and an edit that inverts two rungs does not."""
    ladder = sp.UPLINK_LADDER
    assert len(ladder) == 4
    for a, b in zip(ladder, ladder[1:]):
        assert a < b, f'{a} must expire before {b}'


def test_the_shipped_values_are_the_ones_the_spec_names():
    assert sp.UPLINK_LADDER == (0.10, 0.40, 0.80, 1.00)


def test_coast_ends_before_loss_is_declared():
    """Stated separately because it is the rung pair that matters: steering on
    a prediction after the target is declared lost is steering at nothing."""
    assert sp.UPLINK_COAST_S < sp.UPLINK_DECLARE_LOST_S


# --------------------------------------------------------------------------- #
#  What actually goes on the wire
# --------------------------------------------------------------------------- #
def test_the_coasted_flag_and_gap_age_reach_the_message():
    """They ride x/y, which the spec leaves undefined and we were sending as
    literal 0.0. Without them the board can only age from ITS OWN receipt time,
    so a coasted target is double-decayed -- roughly 2x too fast."""
    args = _send(coasted=True, gap_age_s=0.35)
    assert args[8] == pytest.approx(1.0)     # x = coasted
    assert args[9] == pytest.approx(0.35)    # y = seconds since a real detection


def test_a_live_target_says_so():
    args = _send(coasted=False, gap_age_s=0.0)
    assert args[8] == pytest.approx(0.0)


def test_a_non_finite_gap_age_becomes_zero_not_NaN():
    """A NaN here reaches a 500 Hz control loop."""
    args = _send(coasted=True, gap_age_s=float('nan'))
    assert math.isfinite(args[9])


def test_a_negative_gap_age_is_clamped():
    """Time since a past event cannot be negative; a clock wobble must not
    become a target that is fresher than fresh."""
    assert _send(gap_age_s=-2.0)[9] == pytest.approx(0.0)


def test_the_class_number_reaches_the_message():
    assert _send(target_num=sp.uplink_class_num('torpedo'))[1] == \
        sp.UPLINK_CLASSES['torpedo']


def test_a_non_finite_bearing_is_dropped_entirely():
    """Unchanged behaviour, re-asserted because the new fields sit next to it."""
    fc = SrotFC(_Master(), log=None)
    b = _Bearing()
    b.angle_x = float('nan')
    fc.send_landing_target(b)
    assert not [n for n, _, _ in fc.master.mav.sent if n == 'landing_target_send']
