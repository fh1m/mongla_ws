"""A mission disarms even when surface raises, and the standalone SAUVC
missions run under their budget (beads 2s8, bx8)."""
import ast
import pathlib

MISSIONS = pathlib.Path(__file__).resolve().parents[1] / 'mongla_planner' / 'missions'


def _finally_blocks(tree):
    for node in ast.walk(tree):
        if isinstance(node, ast.Try) and node.finalbody:
            yield node


def test_no_mission_skips_disarm_when_surface_raises():
    bad = []
    for f in sorted(MISSIONS.glob('*.py')):
        src = f.read_text()
        for t in _finally_blocks(ast.parse(src)):
            body = [ast.unparse(s) for s in t.finalbody]
            if 'mongla.surface()' in body and 'mongla.disarm()' in body:
                bad.append(f.name)
    assert not bad, f'surface() then disarm() in one finally: {bad}'


def test_standalone_sauvc_missions_run_under_their_budget():
    for name, budget in (('sauvc_navigation', 'SAUVC_NAV_BUDGET_S'),
                         ('sauvc_qualification', 'SAUVC_QUAL_BUDGET_S')):
        src = (MISSIONS / f'{name}.py').read_text()
        run = src[src.index('def run('):]
        run = run[:run.index('\ndef ', 1)]
        assert f'deadline_s={budget}' in run, name
