"""The two scoring tables, and the card that fills them in.

Plain pytest, no rclpy: Scorecard is deliberately a plain object so the award
rules can be tested without a running node, the same way AxisStyle is.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from duburi_sim_bridge import rulebook                       # noqa: E402
from duburi_sim_bridge.scoring import Scorecard              # noqa: E402


def test_both_books_load_and_every_item_is_unique():
    for name in ('sauvc', 'robosub'):
        book = rulebook.book(name)
        keys = [it['key'] for t in book['tasks'] for it in t['items']]
        assert len(keys) == len(set(keys)), f'{name} has a duplicate key'


def test_unknown_competition_falls_back_the_way_the_generator_does():
    assert rulebook.book('nonesuch') is rulebook.SAUVC


def test_reachable_maximum_is_below_the_rulebook_maximum():
    # If these ever match, something NOT_MODELLED was quietly marked scored --
    # which is exactly the "looks like a competition result" failure the split
    # exists to prevent.
    for name in ('sauvc', 'robosub'):
        full, reach = rulebook.maxima(rulebook.book(name))
        assert 0 < reach < full


def test_award_is_idempotent_up_to_the_rulebook_cap():
    card = Scorecard('robosub')
    assert card.award('gate_pass', 'port') is True
    assert card.award('gate_pass', 'port again') is False
    assert card.total == 100


def test_repeatable_items_count_up_to_their_cap():
    card = Scorecard('robosub')
    for _ in range(5):
        card.award('torp_any')
    # "max 2" in the table.
    assert card.snapshot()['tasks'][3]['items'][0]['count'] == 2
    assert card.total == 1200


def test_not_modelled_items_never_add_points():
    card = Scorecard('robosub')
    card.award('oct_basket_correct', 'pretend')
    assert card.total == 0


def test_penalties_subtract():
    card = Scorecard('sauvc')
    card.award('gate_pass')
    card.penalise('touch_pool', 'the floor')
    card.penalise('touch_pool', 'a wall')
    assert card.total == 15 - 10


def test_computed_bonus_is_reported_as_earned_not_as_the_table_value():
    # Timing is a formula, so its table entry is 0 and the earned value comes
    # from set_bonus. A snapshot that showed 0 would look like a missed bonus.
    card = Scorecard('sauvc')
    card.award('timing', '420 s run')
    card.set_bonus('timing', 14.4)
    bonus = [it for t in card.snapshot()['tasks'] for it in t['items']
             if it['key'] == 'timing'][0]
    assert bonus['earned'] == 14.4
    assert card.total == 14.4
