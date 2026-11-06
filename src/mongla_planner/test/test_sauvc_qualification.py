"""Guards for the SAUVC qualification mission -- the run that gates the finals.

    "To qualify, an AUV must swim from the qualification starting line, pass
     completely through the qualification gate, perform a U-turn, and
     completely pass through the gate again."  -- https://sauvc.org/rulebook/

Arithmetic and shape are read from the mission's AST (its real arguments, not
its prose); behaviour is checked by EXECUTING `run()` against a recorder.
Nothing here proves the mission flies.
"""
import ast

import pytest
import yaml

import mission_ast as MA
from mongla_planner.missions import competition_config as CFG
from mongla_planner.missions import sauvc_qualification as Q


def _tree():
    return MA.tree('sauvc_qualification')


class _Recorder:
    """Plays the DSL: records every call; vision returns what it is told."""

    def __init__(self, vision_ok=True):
        self.sent = []
        rec = self

        class _Vision:
            def align(self, *a, **k):
                rec.sent.append(('vision_align', a, k))
                return vision_ok

            def move(self, *a, **k):
                rec.sent.append(('vision_move', a, k))
                return vision_ok

        self.vision = _Vision()

    def __getattr__(self, name):
        def _call(*a, **k):
            self.sent.append((name, a, k))
            return True
        return _call

    def names(self):
        return [n for n, _, _ in self.sent]


# ── 1. the class must be the hanging gate, from a model that ships ────────────

def test_it_steers_on_the_qualification_gate_of_the_shipped_model():
    tree = _tree()
    consts = MA.module_consts(tree)
    stems = {MA.resolve(c.args[0], consts) for c in MA.calls(tree, 'set_model')}
    assert stems == {'sauvc_sim'}
    targeted = set()
    for call in MA.calls(tree, 'set_classes'):
        targeted.update(str(MA.resolve(call.args[0], consts)).split(','))
    for attr in ('align', 'move'):
        for call in MA.calls(tree, attr):
            targeted.add(str(MA.resolve(call.args[0], consts)))
    assert targeted == {'qual_gate'}, \
        'qualification is the orange hanging gate, not the red/green final_gate'
    sidecar = MA.sidecar('sauvc_sim')
    if sidecar is None:
        pytest.skip('sauvc_sim.yaml sidecar is not on this host')
    assert targeted <= set(yaml.safe_load(sidecar.read_text())['names'].values())


# ── 2. rulebook arithmetic: the opening is the top ~1 m of water ──────────────

def test_the_pass_depth_fits_the_hanging_opening_and_the_lane_floor():
    """Surfacing FAILS the run and so does touching the bottom bar; the gate is
    "~100cm deep" (0.95 m with the ±5 % margin) and the hull 0.17 m tall."""
    opening = 1.00 * 0.95
    half_hull = 0.172 / 2
    d = -CFG.SAUVC_QUAL_PASS_DEPTH_M
    assert d - half_hull > 0.25, 'the hull top is within 0.25 m of a surfacing failure'
    assert d + half_hull < opening - 0.25, 'the hull bottom is within 0.25 m of the bar'
    assert CFG.SAUVC_QUAL_MAX_DEPTH_M >= -(1.28 * 0.95 - 0.2), \
        'the lane floor guard is deeper than the 1.28 m wall end allows'
    assert CFG.SAUVC_QUAL_PASS_DEPTH_M >= CFG.SAUVC_QUAL_MAX_DEPTH_M


def test_every_set_depth_in_the_mission_is_the_pass_depth():
    tree = _tree()
    consts = MA.module_consts(tree)
    depths = {MA.resolve(c.args[0], consts) for c in MA.calls(tree, 'set_depth')}
    assert depths == {CFG.SAUVC_QUAL_PASS_DEPTH_M}, depths


# ── 3. the shape the rulebook pays for ────────────────────────────────────────

def test_both_passes_drive_through_rather_than_stop_in_front():
    tree = _tree()
    consts = MA.module_consts(tree)
    moves = MA.calls(tree, 'move')
    assert moves
    for call in moves:
        node = MA.kw(call, 'fwd')
        assert node is not None and MA.resolve(node, consts) is None


def test_the_u_turn_is_180_degrees_clockwise_between_two_passes():
    """The video must show a CLOCKWISE 180; qualification needs a U-turn. One
    turn serves both, so it is pinned to yaw_right, between the two passes."""
    d = _Recorder()
    Q.run(d)
    names = d.names()
    assert 'yaw_left' not in names and 'turn' not in names
    turns = [(a, k) for n, a, k in d.sent if n == 'yaw_right']
    assert len(turns) == 1 and turns[0][0][0] == 180.0
    i = names.index('yaw_right')
    assert names[:i].count('vision_move') == 1 and names[i:].count('vision_move') == 1


def test_nothing_surfaces_before_both_passes():
    d = _Recorder()
    Q.run(d)
    names = d.names()
    last_pass = max(i for i, n in enumerate(names) if n == 'vision_move')
    for n, a, _ in d.sent[:last_pass]:
        assert n != 'surface', 'surfacing before the second pass fails the run'
        if n == 'set_depth':
            assert a[0] < 0, 'a depth setpoint at the surface fails the run'
    assert 'surface' in names[last_pass:]
    assert names[-1] == 'disarm'


def test_a_blind_leg_is_flown_but_never_counted():
    d = _Recorder(vision_ok=False)
    assert Q.run(d) == 0
    blind = [a[0] for n, a, _ in d.sent if n == 'move_forward'
             and a and a[0] in (CFG.SAUVC_QUAL_BLIND_OUT_S, CFG.SAUVC_QUAL_BLIND_BACK_S)]
    assert sorted(blind) == sorted([CFG.SAUVC_QUAL_BLIND_OUT_S,
                                    CFG.SAUVC_QUAL_BLIND_BACK_S])
    assert Q.run(_Recorder(vision_ok=True)) == 2


# ── 4. elapsed time ───────────────────────────────────────────────────────────

def test_every_blocking_budget_sums_under_the_run_ceiling():
    tree = _tree()
    consts = MA.module_consts(tree)
    total = 0.0
    for attr in ('align', 'move'):
        for call in MA.calls(tree, attr):
            node = MA.kw(call, 'duration')
            assert node is not None, f'{attr}() with no duration= is unbounded'
            total += float(MA.resolve(node, consts))
    for attr in ('set_depth', 'yaw_right'):
        for call in MA.calls(tree, attr):
            node = MA.kw(call, 'timeout')
            assert node is not None, f'{attr}() with no timeout= is unbounded'
            total += float(MA.resolve(node, consts))
    for call in MA.calls(tree, 'pause'):
        total += float(MA.resolve(call.args[0], consts))
    total += (CFG.SAUVC_QUAL_STANDOFF_S + CFG.SAUVC_QUAL_BLIND_OUT_S
              + CFG.SAUVC_QUAL_BLIND_BACK_S)
    assert total <= CFG.SAUVC_QUAL_BUDGET_S, f'{total:.0f}s over the ceiling'


def test_run_is_a_function_the_runner_can_call():
    tree = _tree()
    run = next(n for n in tree.body
               if isinstance(n, ast.FunctionDef) and n.name == 'run')
    assert [a.arg for a in run.args.args][:1] == ['mongla']
