"""The stillness gate — because "it's on a bench" is not evidence of stillness.

Every drift measurement in this project's history trusted that assumption, and
it has now been wrong three times: round 26's "+51.9 deg/min", this round's
"+38.4 deg/min", and — the one that matters — three captures reported as taken
on a "motionless board" that the operator then told us were not.

Measured difference once stillness was actually verified: peak-to-peak went from
**6.36 deg to 0.029 deg**, a factor of ~200. The entire "reproducible wander"
finding was the board being handled.

The board publishes its own body rates at 50 Hz inside ATTITUDE (fw
`mav_stream.cpp:226` packs `s.gx/gy/gz`), so stillness is MEASURABLE and never
has to be assumed again.
"""
import importlib.util
import sys
from pathlib import Path

_TOOL = Path(__file__).resolve().parents[3] / 'tools' / 'yaw_drift_check.py'


def _load():
    """Extract the pure decision from the tool WITHOUT importing it.

    The tool imports rclpy and duburi_interfaces, which are absent on a dev box.
    Stubbing them is brittle (the module defines `class W(Node)` at import time),
    so instead the two things under test -- `QUIET_RADS` and `stillness_verdict`
    -- are lifted out by AST and executed alone. That also means this test reads
    the SHIPPED source rather than a copy of it, which is the property that
    matters: a parallel implementation would pass while the tool was broken.
    """
    import ast, types
    tree = ast.parse(_TOOL.read_text(), str(_TOOL))
    wanted = []
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(
                getattr(t, 'id', None) == 'QUIET_RADS' for t in node.targets):
            wanted.append(node)
        elif isinstance(node, ast.FunctionDef) and node.name == 'stillness_verdict':
            wanted.append(node)
    assert len(wanted) == 2, f'expected QUIET_RADS + stillness_verdict, found {len(wanted)}'
    mod = types.ModuleType('yaw_drift_check_pure')
    exec(compile(ast.Module(body=wanted, type_ignores=[]), str(_TOOL), 'exec'), mod.__dict__)
    return mod


def test_tool_exists():
    assert _TOOL.is_file()


def test_a_quiescent_board_is_STILL():
    m = _load()
    v, txt = m.stillness_verdict(g_rms=0.0030, g_max=0.0253, quiet_frac=0.998)
    assert v == 'STILL', txt
    assert 'SENSOR' in txt


def test_a_HANDLED_board_is_NOT_STILL():
    """⛔ The case that invalidated a headline finding. A board being moved must
    never be graded as a clean measurement."""
    m = _load()
    v, txt = m.stillness_verdict(g_rms=0.35, g_max=1.20, quiet_frac=0.41)
    assert v == 'NOT STILL'
    assert 'contaminated' in txt


def test_a_MOSTLY_quiet_board_with_one_bump_is_NOT_STILL():
    """The dangerous middle: 99% quiet but a real excursion. The earlier
    contaminated captures looked calm on average and were not."""
    m = _load()
    v, _ = m.stillness_verdict(g_rms=0.02, g_max=0.85, quiet_frac=0.990)
    assert v == 'NOT STILL', 'a large max must veto, however good the average'


def test_no_gyro_is_UNKNOWN_and_never_STILL():
    """Absence of a stillness check is not evidence of stillness — the exact
    mistake this tool exists to stop."""
    m = _load()
    for args in ((None, 0.02, 0.99), (0.003, None, 0.99), (0.003, 0.02, None)):
        v, txt = m.stillness_verdict(*args)
        assert v == 'UNKNOWN', args
    assert 'not evidence of stillness' in m.stillness_verdict(None, None, None)[1]


def test_the_quiet_fraction_gate_actually_gates():
    m = _load()
    assert m.stillness_verdict(0.003, 0.02, 0.981)[0] == 'STILL'
    assert m.stillness_verdict(0.003, 0.02, 0.970)[0] == 'NOT STILL'
