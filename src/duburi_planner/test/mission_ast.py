"""Shared AST helpers for the SAUVC mission guards.

These exist so a guard asserts on a mission's ACTUAL call arguments rather than
on its prose. Earlier rounds shipped guards that passed against injected code
because the string they matched survived in an import line or a comment, so
every helper here resolves a real value: a literal, a module-level constant in
the mission, or a name in `competition_config`.

One copy, two test files. A second copy of a lookup is how the geometry table's
host-specific path bug happened.
"""

import ast
import os
from pathlib import Path

from duburi_planner.missions import competition_config as CFG

MISSIONS = Path(__file__).resolve().parents[1] / 'duburi_planner' / 'missions'
SRC_ROOT = Path(__file__).resolve().parents[2]      # .../src


def tree(name):
    """Parse `missions/<name>.py`."""
    return ast.parse((MISSIONS / f'{name}.py').read_text())


def module_consts(node):
    """Top-level `NAME = <literal>` assignments."""
    out = {}
    for stmt in node.body:
        if isinstance(stmt, ast.Assign) and isinstance(stmt.value, ast.Constant):
            for tgt in stmt.targets:
                if isinstance(tgt, ast.Name):
                    out[tgt.id] = stmt.value.value
    return out


def resolve(node, consts):
    """Resolve an argument node to its value, or raise."""
    if isinstance(node, ast.Constant):
        return node.value
    if isinstance(node, ast.Name):
        if node.id in consts:
            return consts[node.id]
        if hasattr(CFG, node.id):
            return getattr(CFG, node.id)
    raise AssertionError(f'cannot resolve argument {ast.dump(node)}')


def calls(node, attr):
    """Every `<anything>.<attr>(...)` call under `node`."""
    return [n for n in ast.walk(node)
            if isinstance(n, ast.Call)
            and isinstance(n.func, ast.Attribute)
            and n.func.attr == attr]


def kw(call, name):
    for k in call.keywords:
        if k.arg == name:
            return k.value
    return None


def func(node, name):
    return next(n for n in node.body
                if isinstance(n, ast.FunctionDef) and n.name == name)


def sidecar(stem):
    """Find `<stem>.yaml` where the DETECTOR would find it, not where I looked.

    Class labels come from the sidecar beside the model file, and that lives in a
    different place per host: the dev box keeps it in the source models
    directory, the vehicle in `DUBURI_HEF_DIR` (~/hailo_models) beside the HEF.
    Pinning one path made a guard pass on the dev box and fail on the vehicle --
    the same defect as the geometry table's `parents[2]`. Weights and sidecars
    are gitignored, so a fresh clone has none and callers skip.
    """
    hefdir = os.environ.get('DUBURI_HEF_DIR') or str(Path.home() / 'hailo_models')
    for d in (Path(hefdir),
              SRC_ROOT / 'duburi_vision' / 'models',
              SRC_ROOT.parent / 'install' / 'duburi_vision' / 'share'
              / 'duburi_vision' / 'models'):
        cand = d / f'{stem}.yaml'
        if cand.is_file():
            return cand
    return None


def sauvc_mission_names():
    """Every SAUVC mission module stem, so a sweep cannot miss a new one."""
    return sorted(p.stem for p in MISSIONS.glob('sauvc_*.py'))
