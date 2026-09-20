"""B45 -- the deck default must MIRROR the spec default, and be pinned.

`vision_tunables` says of itself: "Mirrors the spec defaults in
mongla_control/commands.py exactly so nothing changes if the operator never
sets a param."  That claim is load-bearing and it was false.

`fields_for` substitutes in the order  goal -> runtime_defaults -> spec
defaults, so a `vision.*` param the operator never touched still OVERRIDES the
hardcoded spec default. A drift between the two tables is therefore not a
documentation nit: it silently changes what the hull does, on the pool-day
path, with `ros2 param list` showing exactly the value the docs promise you
would have to set yourself.

That is how `vision.coast_s` ran at 0.8 while the spec default, five documents
and its own adjacent comment all said 0.0 -- nobody reading the stack could
have told you what the hull actually did. 0.8 is the value the operator has
been flying and confirms was run in water; this file pins the tables to it so
the two halves can no longer disagree about what ships.
"""
import sys, os

_HERE = os.path.dirname(os.path.abspath(__file__))
_WS   = os.path.abspath(os.path.join(_HERE, '..', '..', '..'))
for _p in ('mongla_manager', 'mongla_control'):
    _s = os.path.join(_WS, 'src', _p)
    if _s not in sys.path:
        sys.path.insert(0, _s)

from mongla_manager.vision_tunables import (            # noqa: E402
    VISION_PARAM_DEFAULTS, _FIELDS_PER_COMMAND, runtime_defaults_for_command)
from mongla_control.commands import COMMANDS, fields_for  # noqa: E402


# surge_sign is the ONE deliberate exception, and it is documented as such in
# vision_tunables: the deck param IS the source of truth for the bottom-camera
# mount polarity, so no mission has to carry it. Anything else appearing here
# is a drift, not an exception -- do not extend this set to make a test pass.
_DELIBERATE = {('vision_align', 'surge_sign')}


def test_every_deck_default_mirrors_its_spec_default():
    drift = []
    for cmd, field_to_param in _FIELDS_PER_COMMAND.items():
        spec = COMMANDS[cmd]['defaults']
        for field, param in field_to_param.items():
            if (cmd, field) in _DELIBERATE:
                continue
            param_v = float(VISION_PARAM_DEFAULTS[param])
            spec_v  = float(spec.get(field, 0.0))   # absent spec default == rosidl zero
            if param_v != spec_v:
                drift.append(f'{cmd}.{field}: spec={spec_v} but {param}={param_v}')
    assert not drift, 'deck default overrides the spec default:\n  ' + '\n  '.join(drift)


def _unset_goal(cmd):
    class G: pass
    g = G()
    for f in COMMANDS[cmd]['fields']:
        d = COMMANDS[cmd]['defaults'].get(f)
        setattr(g, f, '' if isinstance(d, str) else 0.0)
    return g


def test_the_shipped_coast_is_what_a_plain_mission_gets():
    """The end-to-end assertion, not the table one.

    0.8 s is the value the vehicle has been flying, so it is the value a
    mission that says nothing must receive. This is pinned END TO END --
    through `runtime_defaults_for_command` and `fields_for`, the same path a
    real goal takes -- because the table agreeing with itself is not what
    matters; what reaches `align_loop` is.
    """
    for cmd in ('vision_align', 'vision_move'):
        rd = runtime_defaults_for_command(cmd, dict(VISION_PARAM_DEFAULTS))
        kw = fields_for(cmd, _unset_goal(cmd), runtime_defaults=rd)
        assert kw['coast_s'] == 0.8, (
            f'{cmd} got coast_s={kw["coast_s"]}, not the shipped 0.8')


def test_the_operator_can_still_turn_the_coast_OFF():
    """The knob must still reach zero.

    `coast_s=0` is the byte-identical raw-/detections path and it is the
    control arm for any future A/B in water. A default is not a lock.
    """
    snap = dict(VISION_PARAM_DEFAULTS)
    snap['vision.coast_s'] = 0.0
    rd = runtime_defaults_for_command('vision_align', snap)
    kw = fields_for('vision_align', _unset_goal('vision_align'), runtime_defaults=rd)
    assert kw['coast_s'] == 0.0


def test_the_coast_stays_inside_the_documented_timeout_ladder():
    """Rung 2 < rung 3, checked against the shipped values, not the prose.

    A coast that outlived `lost_grace_s` would steer on a prediction past the
    point the verb was supposed to declare LOST and hand over to the mission's
    fallback search -- the hull would keep driving at a box nobody has seen.
    """
    coast = float(VISION_PARAM_DEFAULTS['vision.coast_s'])
    grace = float(VISION_PARAM_DEFAULTS['vision.lost_grace_s'])
    assert coast < grace, f'coast_s={coast} must stay under lost_grace_s={grace}'


def test_the_tracker_outlives_the_coast():
    """Rung 4: the track must survive at least as long as we intend to coast.

    The 4th rung has bitten this stack once already -- the Kalman smoother
    dropped the track from /tracks before `coast_s` elapsed, so the coast
    truncated early and silently. It is a cross-package constraint, which is
    exactly the kind nothing checks.
    """
    import re, os
    y = os.path.join(_WS, 'src/mongla_vision/config/tracker.yaml')
    m = re.search(r'max_predict_s:\s*([0-9.]+)', open(y).read())
    assert m, 'max_predict_s not found in tracker.yaml'
    assert float(m.group(1)) >= float(VISION_PARAM_DEFAULTS['vision.coast_s'])
