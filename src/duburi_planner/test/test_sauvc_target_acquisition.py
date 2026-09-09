"""Guards for SAUVC Target Acquisition -- and for the one class we must never use.

The point of this file is the first test. `drum_red_pinger` is worth 50 against
30 for blue and 10 for any other red, which makes it the biggest single lever on
the SAUVC card and the most expensive thing to get wrong. It is identified
ACOUSTICALLY -- "Drum Pinger: RJE International Pinger Model No. ULB-362B/45 kHz"
-- and the rulebook describes no visual marking whatsoever. The class is
separable in our training data only because `sauvc_drum_red_pinger/model.sdf`
paints a yellow emissive band on the simulator model. No hydrophone is fitted.

So a mission that steers on it does not score 50; it drops into a random 10-point
drum while the log says it found the pinger. That is a plausible number standing
in for an absent measurement, which is this repo's recurring defect, and it is
worth a guard that sweeps every SAUVC mission rather than a comment.

Nothing here proves the mission FLIES.
"""

import ast

import pytest
import yaml

import mission_ast as MA
from duburi_planner.missions import competition_config as CFG

_NAME = 'sauvc_target_acquisition'


def _steered_classes(mission):
    """Every class string a mission actually steers on, from call arguments."""
    node = MA.tree(mission)
    consts = MA.module_consts(node)
    out = set()
    for call in MA.calls(node, 'set_classes'):
        out.update(str(MA.resolve(call.args[0], consts)).split(','))
    for attr in ('align', 'move'):
        for call in MA.calls(node, attr):
            if call.args:
                out.add(str(MA.resolve(call.args[0], consts)))
    return out


# ── 1. the guard this file exists for ─────────────────────────────────────────

def test_no_sauvc_mission_steers_on_the_acoustic_pinger_drum():
    missions = MA.sauvc_mission_names()
    assert missions, 'the sweep found no SAUVC missions -- it would pass vacuously'
    offenders = {m for m in missions if 'drum_red_pinger' in _steered_classes(m)}
    assert not offenders, (
        f'{sorted(offenders)} steer on drum_red_pinger. The pinger is ACOUSTIC '
        '(45 kHz) and no hydrophone is fitted; the class is separable only '
        'because the simulator paints a yellow band on that drum. Steering on it '
        'drops into a 10-point drum while reporting 50.')


def test_the_geometry_table_still_records_why():
    """The finding must outlive this test file."""
    path = MA.SRC_ROOT / 'duburi_vision' / 'config' / 'target_geometry.yaml'
    text = path.read_text()
    assert 'ULB-362B' in text, \
        'the acoustic-pinger finding was deleted from the geometry table'
    assert 'pinger_band' in text, \
        'the simulator-artifact explanation was deleted from the geometry table'
    # And the entry itself must still exist and match plain red, since the two
    # drums are geometrically identical.
    data = yaml.safe_load(text)['sauvc']
    assert data['drum_red_pinger'] == data['drum_red']


# ── 2. the class we DO steer on has to exist ──────────────────────────────────

def test_the_targeted_drum_is_blue_and_is_in_the_shipped_model():
    steered = _steered_classes(_NAME)
    assert steered == {'drum_blue'}, \
        f'expected to steer only on drum_blue, got {sorted(steered)}'
    side = MA.sidecar('sauvc_sim')
    if side is None:
        pytest.skip('sauvc_sim.yaml sidecar is not on this host')
    names = set(yaml.safe_load(side.read_text())['names'].values())
    assert 'drum_blue' in names, f'drum_blue is not a class in {side}'


# ── 3. rulebook arithmetic ────────────────────────────────────────────────────

def test_the_hover_depth_cannot_ground_the_hull():
    """Bottom or wall contact is -5 per occurrence, the harshest penalty."""
    assert CFG.SAUVC_DRUM_HOVER_DEPTH_M < 0.0
    assert CFG.SAUVC_DRUM_HOVER_DEPTH_M >= CFG.SAUVC_MAX_DEPTH_M, (
        f'hover depth {CFG.SAUVC_DRUM_HOVER_DEPTH_M} is deeper than the SAUVC '
        f'floor guard {CFG.SAUVC_MAX_DEPTH_M}')
    # The drum is 0.30 m deep and stands on the floor, so its mouth is 0.30 m up.
    # Even at the shallow 1.2 m end the hull must stay above the mouth.
    mouth_depth = 1.2 - 0.30
    assert -CFG.SAUVC_DRUM_HOVER_DEPTH_M < mouth_depth, \
        'hover depth is below the drum mouth at the shallow end of the pool'


def test_the_descent_stays_off_while_the_downward_fov_is_unmeasured():
    """A fill-driven descent converts an unmeasured FOV into metres of dive.

    Turning it on is fine once the downward camera's in-water FOV is measured --
    but then this test must be updated deliberately, which is the point.
    """
    assert CFG.SAUVC_DRUM_DESCEND_FILL == 0, (
        'SAUVC_DRUM_DESCEND_FILL was enabled; measure the DOWNWARD in-water FOV '
        'and record it in measured-bars.md before trusting a fill-to-depth map')


def test_the_downward_align_is_bounded_by_the_sauvc_floor_guard():
    node = MA.tree(_NAME)
    consts = MA.module_consts(node)
    bounds = {}
    for call in MA.calls(node, 'set_vision_param'):
        if len(call.args) == 2 and isinstance(call.args[0], ast.Constant):
            bounds.setdefault(call.args[0].value, []).append(call.args[1])
    assert 'max_depth_m' in bounds, 'the descent floor is never set'
    values = {MA.resolve(v, consts) for v in bounds['max_depth_m']}
    assert CFG.SAUVC_MAX_DEPTH_M in values, \
        'the floor is not set from SAUVC_MAX_DEPTH_M'


# ── 4. the shape ──────────────────────────────────────────────────────────────

def test_the_ball_is_never_dropped_blind():
    """Unlike the gate, an off-target release scores nothing and spends the ball.

    `fire()` must sit under a branch on the align result, not at statement level.
    """
    node = MA.tree(_NAME)
    fires = MA.calls(node, 'fire')
    assert fires, 'the mission never releases the ball'
    guarded = set()
    for branch in ast.walk(node):
        if isinstance(branch, ast.If):
            for f in MA.calls(branch, 'fire'):
                guarded.add(id(f))
    assert all(id(f) in guarded for f in fires), \
        'fire() is not gated on the alignment result -- that is a blind drop'


def test_the_mission_restores_the_forward_camera():
    node = MA.tree(_NAME)
    fn = MA.func(node, 'acquire')
    tries = [n for n in ast.walk(fn) if isinstance(n, ast.Try) and n.finalbody]
    assert tries, 'the downward switch is not wrapped in try/finally'

    def _restores(block):
        return any(
            isinstance(c, ast.Call) and isinstance(c.func, ast.Attribute)
            and c.func.attr == 'use_camera'
            and c.args and getattr(c.args[0], 'value', None) == 'forward'
            for stmt in block for c in ast.walk(stmt))

    assert any(_restores(t.finalbody) for t in tries), (
        'the forward camera is not restored in finally -- a later forward task '
        'would run against a live downward detector')


def test_the_mission_is_discoverable_by_the_runner():
    from duburi_planner.missions import discover
    assert _NAME in discover()
