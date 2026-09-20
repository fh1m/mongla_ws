"""Guards for the SAUVC Navigation mission -- the one task that gates all scoring.

    "The first task, Navigation, is mandatory and must be completed before
     attempting any other task."  -- https://sauvc.org/rulebook/

Everything asserted here is a rulebook arithmetic constraint or a name that must
match a file shipped on the vehicle. Nothing here proves the mission FLIES; no
thrusters are attached and the detector is sim-trained.

These read the mission's ACTUAL call arguments out of its AST, never its prose.
A previous round shipped a guard that passed against injected code because the
string it matched lived in an import line, so a docstring/comment match is not
evidence here: every assertion below resolves a real argument value.
"""

import ast

import pytest
import yaml

import mission_ast as MA
from mongla_planner.missions import competition_config as CFG

_MISSION = MA.MISSIONS / 'sauvc_navigation.py'


def _tree():
    return MA.tree('sauvc_navigation')


_module_consts = MA.module_consts
_resolve = MA.resolve
_sidecar = MA.sidecar


def _calls(tree, attr):
    return MA.calls(tree, attr)


def _kw(call, name):
    return MA.kw(call, name)


# ── 1. the model stem and the class name must be what ships on the vehicle ────

def test_the_model_stem_has_a_yaml_that_ships():
    """`sauvc_sim` resolves to a HEF on the Pi; `sim_sauvc_v1` does not.

    Naming a stem with no HEF is silent: `_resolve_model_path` falls back to the
    3-4 Hz PyTorch path with no error, which on a timed run is a lost slot.
    """
    tree = _tree()
    consts = _module_consts(tree)
    calls = _calls(tree, 'set_model')
    assert calls, 'the mission must select a model explicitly'
    stems = {_resolve(c.args[0], consts) for c in calls}
    assert stems == {'sauvc_sim'}, f'unexpected model stem(s): {stems}'
    for stem in stems:
        if _sidecar(stem) is None:
            pytest.skip(f'{stem}.yaml sidecar is not on this host '
                        '(weights and sidecars are gitignored)')


def test_every_targeted_class_is_in_the_shipped_model():
    """The class strings the mission steers on must exist in sauvc_sim.yaml.

    Asserted against the ARGUMENTS of set_classes / align / move, so replacing
    'final_gate' with a class the model does not emit fails here even though the
    module docstring still names the gate.
    """
    tree = _tree()
    consts = _module_consts(tree)
    sidecar = _sidecar('sauvc_sim')
    if sidecar is None:
        pytest.skip('sauvc_sim.yaml sidecar is not on this host')
    names = set(yaml.safe_load(sidecar.read_text())['names'].values())

    targeted = set()
    for call in _calls(tree, 'set_classes'):
        targeted.update(str(_resolve(call.args[0], consts)).split(','))
    for attr in ('align', 'move'):
        for call in _calls(tree, attr):
            if call.args:
                targeted.add(str(_resolve(call.args[0], consts)))

    assert targeted, 'the mission steers on no class at all'
    unknown = targeted - names
    assert not unknown, f'classes not in {sidecar}: {sorted(unknown)}'
    assert 'final_gate' in targeted, \
        "Navigation uses the red/green final_gate, not the full-height qual_gate"


# ── 2. rulebook arithmetic: the sloping floor ─────────────────────────────────

# "Touching the bottom of the pool or wall: 5" points -- per occurrence, the
# harshest penalty in the rulebook. The floor is 1.6 m at the pool centre and
# 1.2 m at both ends, so a setpoint is only safe if it clears the SHALLOW end.
_SHALLOW_FLOOR_M = 1.2
_MIN_CLEARANCE_M = 0.2


def test_no_sauvc_depth_setpoint_can_ground_the_hull():
    depths = {n: v for n, v in vars(CFG).items()
              if n.startswith('SAUVC_') and n.endswith('_DEPTH_M')}
    assert depths, 'the SAUVC config block declares no depths'
    floor_guard = -(_SHALLOW_FLOOR_M - _MIN_CLEARANCE_M)   # -1.0 m
    assert CFG.SAUVC_MAX_DEPTH_M >= floor_guard, (
        f'SAUVC_MAX_DEPTH_M {CFG.SAUVC_MAX_DEPTH_M} is deeper than the shallow '
        f'end allows ({floor_guard} m)')
    for name, value in depths.items():
        assert value < 0.0, f'{name} must be negative (below surface)'
        assert value >= CFG.SAUVC_MAX_DEPTH_M, (
            f'{name}={value} is deeper than SAUVC_MAX_DEPTH_M '
            f'{CFG.SAUVC_MAX_DEPTH_M} -- a -5 bottom touch at the pool ends')


def test_the_pass_depth_fits_inside_the_gate_opening():
    """The gate is 100 cm tall on a floor ~1.49 m down at the 16 m mark, so the
    opening runs from the floor up to ~0.49 m. Pass inside it, biased high:
    a gate touch is -2, a bottom touch is -5.
    """
    floor_at_gate = 1.49
    top_bar = floor_at_gate - 1.00          # 0.49 m below the surface
    depth = -CFG.SAUVC_GATE_PASS_DEPTH_M
    assert depth > top_bar, \
        'pass depth is ABOVE the gate top bar -- the hull would go over, not through'
    assert depth < floor_at_gate, 'pass depth is below the pool floor at the gate'
    assert (depth - top_bar) > (floor_at_gate - depth) * 0.4, \
        'pass depth is not biased away from the -5 floor'


# ── 3. elapsed time is a scored resource ──────────────────────────────────────

def test_the_mission_fits_its_share_of_the_run_budget():
    """`Bonus points = (900 - RUN_TIME) * 0.03` -- at 300 s that is 18 points,
    more than the 15-point task. Navigation may not eat the whole run.

    ⛔ The bonus is paid only "when completing at least 2 tasks", and this
    mission surfaces and disarms at the end -- so a navigation-only run earns
    NO bonus. The 300 s ceiling is what makes the bonus reachable once a
    combinator chains navigation into a second task on one dive.

    This sums EVERY blocking budget the mission declares, not just the vision
    verbs: the pre-arm tether pause, both `set_depth` timeouts, the vision
    durations and the blind-transit leg. Summing only the vision verbs gave
    195 s and read as headroom that is not there.
    """
    assert CFG.SAUVC_RUN_BUDGET_S == 900.0, 'the rulebook gives 15 minutes'
    assert CFG.SAUVC_NAV_BUDGET_S <= CFG.SAUVC_RUN_BUDGET_S / 3.0

    tree = _tree()
    consts = _module_consts(tree)
    total = 0.0
    for attr in ('align', 'move'):
        for call in _calls(tree, attr):
            node = _kw(call, 'duration')
            assert node is not None, f'{attr}() with no duration= is unbounded'
            total += float(_resolve(node, consts))
    for call in _calls(tree, 'set_depth'):
        node = _kw(call, 'timeout')
        assert node is not None, 'set_depth() with no timeout= is unbounded'
        total += float(_resolve(node, consts))
    for call in _calls(tree, 'pause'):
        if call.args:
            total += float(_resolve(call.args[0], consts))
    total += float(CFG.SAUVC_BLIND_TRANSIT_S)
    assert total <= CFG.SAUVC_NAV_BUDGET_S, (
        f'mission budgets sum to {total:.0f}s, over the '
        f'{CFG.SAUVC_NAV_BUDGET_S:.0f}s navigation budget')


# ── 4. the shape the rulebook pays for ────────────────────────────────────────

def test_the_gate_is_driven_through_not_stopped_in_front_of():
    """`vision.move(fwd=None)` is PASS-THROUGH; any number is a fill STOP.

    A fill stop parks the hull in front of the gate and scores nothing, and the
    difference between the two is one keyword.
    """
    tree = _tree()
    consts = _module_consts(tree)
    moves = _calls(tree, 'move')
    assert moves, 'the mission never drives at the gate'
    for call in moves:
        node = _kw(call, 'fwd')
        assert node is not None and _resolve(node, consts) is None, \
            'move() must pass fwd=None to transit the gate, not stop at a fill'


def test_the_run_surfaces_before_it_disarms():
    """Surfacing at the end of the run is +5 for free."""
    tree = _tree()
    run = next(n for n in tree.body
               if isinstance(n, ast.FunctionDef) and n.name == 'run')
    order = [n.func.attr for n in ast.walk(run)
             if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)]
    assert 'surface' in order, 'the mission never surfaces -- 5 points left behind'
    assert 'disarm' in order
    assert order.index('surface') < order.index('disarm')


def test_a_perception_miss_still_attempts_the_mandatory_task():
    """Selector[precise, always_act]: the blind transit must be a PARAMETER.

    Navigation gates every other point, so losing the gate must degrade to an
    attempt rather than to a stop.
    """
    assert CFG.SAUVC_BLIND_TRANSIT_ENABLED is True
    assert CFG.SAUVC_BLIND_TRANSIT_S > 0.0
    src = _MISSION.read_text()
    assert 'SAUVC_BLIND_TRANSIT_ENABLED' in src
    tree = _tree()
    navigate = next(n for n in tree.body
                    if isinstance(n, ast.FunctionDef) and n.name == 'navigate')
    guarded = [n for n in ast.walk(navigate) if isinstance(n, ast.If)]
    assert any(
        any(isinstance(x, ast.Name) and x.id == 'SAUVC_BLIND_TRANSIT_ENABLED'
            for x in ast.walk(n.test))
        and any(isinstance(c, ast.Call) and isinstance(c.func, ast.Attribute)
                and c.func.attr == 'move_forward' for c in ast.walk(n))
        for n in guarded), \
        'the blind transit is not actually driven under the config flag'


def test_the_mission_is_discoverable_by_the_runner():
    from mongla_planner.missions import discover
    assert 'sauvc_navigation' in discover()
