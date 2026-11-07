"""Every file that imports pymavlink selects MAVLink 2 first, at module level.

pymavlink picks its dialect ONCE, when `mavutil` is first imported, from the
`MAVLINK20` environment variable. Import it before that is set and the whole
process runs the MAVLink 1 dialect: the board's ESC_TELEMETRY frames decode as
`UNKNOWN_11030` and read as zero RPM, and v2-only send signatures change
(`fc/_srot_base.py` has the measurement).

It was an import-order accident. `bringup_check` and seventeen bench tools
imported pymavlink without it and got MAVLink 2 only when something else had
already imported the board driver. In the test suite that accident made
`test_manager_end_to_end` fail whenever `test_bringup_check_fw_rev` ran first:
the fake board's `command_ack_send(...)` raised under the v1 dialect, the ack
never came, and the client got an empty result (bead iwk.5).

So each importer sets it itself, at module level -- inside a function is not
enough, because another function of the same file may import pymavlink first.
"""
import ast
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def _importers():
    for p in sorted([*ROOT.glob('src/*/**/*.py'), *ROOT.glob('tools/*.py'),
                     *ROOT.glob('scripts/*.py')]):
        if 'install' in p.parts or 'build' in p.parts:
            continue
        text = p.read_text(encoding='utf-8')
        if 'pymavlink' in text:
            yield p, text


def _imports_pymavlink(node):
    if isinstance(node, ast.ImportFrom):
        return (node.module or '').split('.')[0] == 'pymavlink'
    if isinstance(node, ast.Import):
        return any(a.name.split('.')[0] == 'pymavlink' for a in node.names)
    return False


def _selects_v2(node):
    return (isinstance(node, ast.Assign) and 'MAVLINK20' in ast.unparse(node)) or \
           (isinstance(node, ast.Expr) and "setdefault('MAVLINK20'" in ast.unparse(node))


def _imports_the_driver(node):
    """The board driver sets MAVLINK20 at its own module level, so importing it
    first is a selection too (the drift test does this on purpose: it checks the
    dialect the vehicle really decodes with)."""
    names = ([node.module or ''] + [a.name for a in node.names]) if isinstance(node, ast.ImportFrom) \
        else [a.name for a in node.names] if isinstance(node, ast.Import) else []
    return any(n.split('.')[-1] in ('srot_fc', '_srot_base') for n in names)


def offenders(files):
    bad = []
    for p, text in files:
        tree = ast.parse(text)
        first = min((n.lineno for n in ast.walk(tree) if _imports_pymavlink(n)), default=None)
        if first is None:
            continue
        if not any(_selects_v2(n) and n.lineno < first for n in tree.body) and \
                not any(_imports_the_driver(n) and n.lineno < first for n in ast.walk(tree)):
            bad.append(p.relative_to(ROOT).as_posix())
    return bad


def test_every_pymavlink_importer_selects_mavlink2_first():
    files = list(_importers())
    assert len(files) > 20, 'the scan found almost nothing -- it is looking in the wrong place'
    assert not offenders(files), offenders(files)


def test_the_check_catches_a_late_or_function_local_selection():
    late = "from pymavlink import mavutil\nimport os\nos.environ['MAVLINK20'] = '1'\n"
    local = ("import os\ndef f():\n    os.environ['MAVLINK20'] = '1'\n"
             "    from pymavlink import mavutil\n")
    good = "import os\nos.environ['MAVLINK20'] = '1'\nfrom pymavlink import mavutil\n"
    P = ROOT / 'x.py'
    assert offenders([(P, late)]) == ['x.py']
    assert offenders([(P, local)]) == ['x.py']
    assert offenders([(P, good)]) == []
