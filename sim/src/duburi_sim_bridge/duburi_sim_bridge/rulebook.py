"""Both competition scoring tables, as data.

A practice run is only useful preparation if you know what it was worth, and
"what it was worth" is a published table, not an opinion. Keeping the tables
here -- rather than as constants scattered through the scorer -- means the
scorer describes HOW a thing is detected and this file describes WHAT it is
worth, so a rules update is a one-file edit and every number carries the
citation it came from.

Three states, and the dashboard shows which is which, because a total that
quietly counts unreachable points is worse than no total:

    SCORED       the sim detects it and awards it
    NOT_MODELLED the rulebook scores it, the sim cannot judge it
    OURS         the rule exists but the published text does not give a number

The reachable maximum is therefore always lower than the rulebook maximum, and
both are reported. Sources:

  SAUVC   https://sauvc.org/rulebook/
  RoboSub https://robonation.gitbook.io/robosub-resources
          /section-4-scoring-and-awards/4.2-autonomy-challenge-scoring
"""

SCORED = 'scored'
NOT_MODELLED = 'not_modelled'


def item(key, label, points, state=SCORED, note='', repeat=1):
    """One scored line item, named as the rulebook names it."""
    return {'key': key, 'label': label, 'points': points, 'state': state,
            'note': note, 'repeat': repeat}


# ---------------------------------------------------------------------------
# SAUVC -- sauvc.org/rulebook. Real published numbers throughout.
# ---------------------------------------------------------------------------
SAUVC = {
    'name': 'SAUVC 2026',
    'run_seconds': 900,
    'tasks': [
        {'key': 'navigation', 'label': 'Task 1 - Navigation', 'items': [
            item('gate_pass', 'Pass through the gate', 15),
        ]},
        {'key': 'target', 'label': 'Task 2 - Target Acquisition', 'items': [
            item('drum_blue', 'Ball in the blue drum', 30),
            item('drum_pinger', 'Ball in the red drum with the pinger', 50),
            item('drum_red', 'Ball in another red drum', 10),
        ]},
        {'key': 'reacquire', 'label': 'Task 3 - Target Reacquisition', 'items': [
            item('reacquire', 'Retrieve and hold the ball', 60, NOT_MODELLED,
                 'no manipulator on the sim vehicle'),
        ]},
        {'key': 'flares', 'label': 'Task 4 - Communication & Localization',
         'items': [
            item('flare_bump', 'Bump a flare', 20, repeat=3),
            item('flare_order', 'All three in the commanded order', 60),
        ]},
        {'key': 'bonus', 'label': 'Bonuses', 'items': [
            item('surface', 'Surface after completing Task 1', 5),
            item('timing', 'Timing bonus, (900 - run) x 0.03', 0, SCORED,
                 'needs at least two tasks; computed, not a fixed value'),
            item('size', 'Under 70 x 50 x 50 cm', 10, NOT_MODELLED,
                 'a property of the hull, not of the run'),
            item('weight', 'Under 42 kg in air', 10, NOT_MODELLED,
                 'a property of the hull, not of the run'),
        ]},
    ],
    'penalties': [
        item('touch_gate', 'Touching the gate', -2),
        item('touch_pool', 'Touching the pool bottom or a wall', -5),
    ],
    # "Cumulative bottom/wall contact > 10 s, or more than 5 discrete touches"
    'abort_contact_s': 10.0,
    'abort_touches': 5,
}


# ---------------------------------------------------------------------------
# RoboSub -- the 4.2 Autonomy Challenge Scoring table.
# ---------------------------------------------------------------------------
ROBOSUB = {
    'name': 'RoboSub 2026',
    'run_seconds': 900,
    'tasks': [
        {'key': 'gate', 'label': 'Task 1 - Begin Assessment (Gate)', 'items': [
            item('gate_pass', 'Pass through the gate', 100),
            item('gate_control', 'Maintain control', 150, SCORED,
                 'the handbook defines this as actively holding a heading '
                 'rather than drifting through; scored here as heading held '
                 'within a band across the transit'),
            item('coin_flip', 'Coin flip', 300),
            item('random_role', 'Random role', 150),
            item('style_yaw', 'Style: yaw, per 90 deg', 100, SCORED,
                 'up to 8', repeat=8),
            item('style_rp', 'Style: roll/pitch, per 90 deg', 200, SCORED,
                 'up to 8', repeat=8),
        ]},
        {'key': 'slalom', 'label': 'Task 2 - Avoid Debris (Slalom)', 'items': [
            item('slalom_any', 'Navigate the debris, any side', 200),
            item('slalom_correct', 'Correct side', 400),
            item('slalom_depth', 'Correct depth, per slalom', 200, repeat=3),
        ]},
        {'key': 'bins', 'label': 'Task 3 - Recon (Bins)', 'items': [
            item('bin_any', 'Marker in any bin', 300, repeat=2),
            item('bin_correct', 'Marker in the correct bin', 800, repeat=2),
            item('bin_light', 'Turn off a light', 500, NOT_MODELLED,
                 'magnetically activated lights are not in the handbook text, '
                 'so there is no rule to implement against', repeat=2),
        ]},
        {'key': 'torpedo', 'label': 'Task 4 - Deploy (Torpedoes)', 'items': [
            item('torp_any', 'Torpedo through any opening', 600, repeat=2),
            item('torp_sequence', 'Correct sequence (large, then small)', 1400),
            item('torp_far', 'Fired from "far"', 200, repeat=2),
            item('torp_farther', 'Fired from "farther"', 400, repeat=2),
        ]},
        {'key': 'octagon', 'label': 'Task 5 - Restore (Octagon)', 'items': [
            item('oct_surface', 'Surface in the area', 800),
            item('oct_face_any', 'Surface facing any image', 200),
            item('oct_face_role', 'Surface facing your role', 400),
            item('oct_face_correct', 'Surface facing the correct role', 700),
            item('oct_with_object', 'Surface with an object', 400, NOT_MODELLED,
                 'no manipulator on the sim vehicle', repeat=2),
            item('oct_drop', 'Drop an object', 200, NOT_MODELLED,
                 'no manipulator', repeat=2),
            item('oct_basket_any', 'Object in any basket', 500, NOT_MODELLED,
                 'no manipulator', repeat=2),
            item('oct_basket_correct', 'Object in the correct basket', 700,
                 NOT_MODELLED, 'no manipulator', repeat=2),
            item('oct_count_close', 'Basket count within one', 500,
                 NOT_MODELLED, 'no manipulator'),
            item('oct_count', 'Basket count correct', 1000, NOT_MODELLED,
                 'no manipulator'),
        ]},
        {'key': 'home', 'label': 'Task 6 - Return Home', 'items': [
            item('return_home', 'Pass back through the start gate', 300),
        ]},
        {'key': 'extra', 'label': 'Additional', 'items': [
            item('pinger_1', 'Random pinger, first task', 500),
            item('pinger_2', 'Random pinger, second task', 1500),
            item('ivc', 'Inter-vehicle communication', 1000, NOT_MODELLED,
                 'Dubomini and IVC are committed phase-2 work with no code '
                 'today; there is one vehicle in the sim'),
            item('time', 'Time bonus, (minutes + sec/60) x 100', 0, SCORED,
                 'computed from the remaining run time, not a fixed value'),
        ]},
    ],
    'penalties': [
        item('spec_violation', 'Marker or torpedo out of spec', -500,
             NOT_MODELLED, 'a property of the payload, not of the run'),
    ],
    'abort_contact_s': None,
    'abort_touches': None,
}


BOOKS = {'sauvc': SAUVC, 'robosub': ROBOSUB}


def book(competition: str) -> dict:
    """The table for a competition, defaulting the way the generator does."""
    return BOOKS.get(competition, SAUVC)


def maxima(rules: dict) -> tuple:
    """(rulebook maximum, maximum this sim can actually award).

    They differ, and the gap is the point: a scorecard that shows only the
    total looks like a competition result. Showing both says how much of the
    table this practice run could ever have reached.
    """
    full = reach = 0
    for task in rules['tasks']:
        for it in task['items']:
            worth = it['points'] * it['repeat']
            full += worth
            if it['state'] == SCORED:
                reach += worth
    return full, reach
