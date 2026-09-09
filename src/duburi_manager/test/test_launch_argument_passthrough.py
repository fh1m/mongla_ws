"""Every key a launch passes to an included launch must be DECLARED there.

`IncludeLaunchDescription.execute()` raises only for a *missing required*
argument. A key the target never declared is dropped SILENTLY -- no exception,
no log line, not even at `--log-level debug`. So a renamed or mistyped key is
indistinguishable from a launch that simply ignores your setting. CLAUDE.md
records the season this cost: the sim's `vision: 'false'` leaked out of an
unscoped include and `vision:=true` started nothing, silently.

The guard walks every `IncludeLaunchDescription` in the repo's launch files and
asserts its key set is a subset of what the included file declares.

AST-only, deliberately: launch files import `ament_index_python` and the ROS
launch package, so importing them here would make the guard depend on the very
environment it exists to protect (and it would then skip on a dev box).

Resolution is bounded to the shapes the repo actually uses -- a dict literal, a
dict comprehension over a module-level tuple of names, and `d['k'] = ...`
follow-ups. Anything else lands in `_UNRESOLVED_OK`, which is FROZEN: a new
unresolvable include fails this file rather than quietly checking nothing.
"""
from __future__ import annotations

import ast
import pathlib

import pytest

_SRC = pathlib.Path(__file__).resolve().parents[2]

# Includes whose key set this parser cannot read. Empty on purpose: every
# include in the tree is currently resolvable, and a new one that is not
# must be made resolvable or added here WITH a reason.
_UNRESOLVED_OK: frozenset[str] = frozenset()


def _launch_files() -> list[pathlib.Path]:
    return sorted(_SRC.glob('*/launch/*.launch.py'))


def _declared(path: pathlib.Path) -> set[str]:
    return {n.args[0].value
            for n in ast.walk(ast.parse(path.read_text()))
            if isinstance(n, ast.Call)
            and getattr(n.func, 'id', '') == 'DeclareLaunchArgument'
            and n.args and isinstance(n.args[0], ast.Constant)}


def _str_seq_consts(tree: ast.AST) -> dict[str, list[str]]:
    """Module-level `NAME = ('a', 'b', ...)` string sequences."""
    out: dict[str, list[str]] = {}
    for node in ast.walk(tree):
        if not isinstance(node, ast.Assign) or len(node.targets) != 1:
            continue
        tgt = node.targets[0]
        if not isinstance(tgt, ast.Name) or not isinstance(node.value, (ast.Tuple, ast.List)):
            continue
        vals = [e.value for e in node.value.elts
                if isinstance(e, ast.Constant) and isinstance(e.value, str)]
        if vals and len(vals) == len(node.value.elts):
            out[tgt.id] = vals
    return out


def _launch_path_vars(tree: ast.AST) -> dict[str, str]:
    """`NAME = os.path.join(..., 'x.launch.py')` -> {'NAME': 'x.launch.py'}."""
    out: dict[str, str] = {}
    for node in ast.walk(tree):
        if not isinstance(node, ast.Assign) or len(node.targets) != 1:
            continue
        tgt = node.targets[0]
        if not isinstance(tgt, ast.Name):
            continue
        lits = [s.value for s in ast.walk(node.value)
                if isinstance(s, ast.Constant) and str(s.value).endswith('.launch.py')]
        if lits:
            out[tgt.id] = lits[-1]
    return out


def _dict_var_keys(tree: ast.AST, consts: dict[str, list[str]]) -> dict[str, set[str] | None]:
    """Keys of dict variables built by comprehension and/or subscript writes."""
    out: dict[str, set[str] | None] = {}
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign) and len(node.targets) == 1:
            tgt = node.targets[0]
            if isinstance(tgt, ast.Name) and isinstance(node.value, ast.DictComp):
                src = node.value.generators[0].iter
                out[tgt.id] = (set(consts[src.id])
                               if isinstance(src, ast.Name) and src.id in consts
                               else None)
            elif isinstance(tgt, ast.Name) and isinstance(node.value, ast.Dict):
                out[tgt.id] = {k.value for k in node.value.keys
                               if isinstance(k, ast.Constant)}
            elif (isinstance(tgt, ast.Subscript) and isinstance(tgt.value, ast.Name)
                  and isinstance(tgt.slice, ast.Constant)):
                known = out.get(tgt.value.id)
                if known is not None:
                    known.add(tgt.slice.value)
    return out


def _cases() -> list[tuple[pathlib.Path, str, set[str] | None]]:
    found = []
    for src in _launch_files():
        tree = ast.parse(src.read_text())
        consts = _str_seq_consts(tree)
        pathvars = _launch_path_vars(tree)
        dictvars = _dict_var_keys(tree, consts)
        for node in ast.walk(tree):
            if not (isinstance(node, ast.Call)
                    and getattr(node.func, 'id', '') == 'IncludeLaunchDescription'):
                continue

            target = ''
            for sub in ast.walk(node):
                if isinstance(sub, ast.Constant) and str(sub.value).endswith('.launch.py'):
                    target = sub.value
                elif isinstance(sub, ast.Name) and sub.id in pathvars:
                    target = pathvars[sub.id]

            keys: set[str] | None = set()
            for kw in node.keywords:
                if kw.arg != 'launch_arguments':
                    continue
                d = kw.value.func.value if isinstance(kw.value, ast.Call) else kw.value
                if isinstance(d, ast.Dict):
                    keys = {k.value for k in d.keys if isinstance(k, ast.Constant)}
                elif isinstance(d, ast.Name):
                    keys = dictvars.get(d.id)
                else:
                    keys = None
            found.append((src, target, keys))
    return found


_CASES = _cases()


def test_the_parser_found_every_include_and_could_read_it():
    """A guard that silently checks nothing is worse than no guard.

    Counts the includes by plain text search and compares, so a parser that
    stops recognising the call shape fails here instead of passing vacuously.
    """
    literal = sum(p.read_text().count('IncludeLaunchDescription(')
                  for p in _launch_files())
    assert len(_CASES) == literal, (
        f'parsed {len(_CASES)} includes but the text shows {literal}')

    unreadable = {f'{s.name}->{t}' for s, t, k in _CASES if k is None}
    assert unreadable <= _UNRESOLVED_OK, (
        f'cannot read launch_arguments for {sorted(unreadable - _UNRESOLVED_OK)}. '
        f'Make it resolvable or add it to _UNRESOLVED_OK with a reason -- '
        f'an unreadable include is an UNCHECKED include.')

    unresolved = {s.name for s, t, _ in _CASES if not t}
    assert not unresolved, f'could not name the included launch file in {unresolved}'


@pytest.mark.parametrize('src,target,keys', _CASES,
                         ids=[f'{s.name}->{t or "?"}' for s, t, _ in _CASES])
def test_every_passed_launch_argument_is_declared_by_the_target(src, target, keys):
    if keys is None:
        pytest.skip('key set unreadable -- covered by the parser self-check above')
    hits = [p for p in _launch_files() if p.name == target]
    assert len(hits) == 1, f'{target!r} matched {len(hits)} files in-tree'

    undeclared = keys - _declared(hits[0])
    assert not undeclared, (
        f'{src.name} passes {sorted(undeclared)} to {target}, which does not '
        f'declare them. Launch drops unknown keys SILENTLY, so the setting '
        f'never takes effect and nothing reports it.')


def _assigned_includes(tree: ast.AST) -> set[str]:
    """`NAME = IncludeLaunchDescription(...)` -> {'NAME'}."""
    return {n.targets[0].id
            for n in ast.walk(tree)
            if isinstance(n, ast.Assign) and len(n.targets) == 1
            and isinstance(n.targets[0], ast.Name)
            and isinstance(n.value, ast.Call)
            and getattr(n.value.func, 'id', '') == 'IncludeLaunchDescription'}


@pytest.mark.parametrize('src', _launch_files(), ids=lambda p: p.name)
def test_every_include_built_is_actually_returned(src):
    """An include that is constructed but never added to the LaunchDescription
    starts nothing, and launch reports no error -- it is simply absent.

    This is the same failure as an undeclared key: the file *looks* like it
    wires the thing up. Caught here on the first write of the vehicle vision
    include, which was built and then left out of the returned list.
    """
    tree = ast.parse(src.read_text())
    built = _assigned_includes(tree)
    if not built:
        pytest.skip('no include assigned to a name')

    # Any later mention of the name counts as use: returned in a list, appended,
    # or passed to a GroupAction.
    used = {n.id for n in ast.walk(tree)
            if isinstance(n, ast.Name) and isinstance(n.ctx, ast.Load)}
    orphans = built - used
    assert not orphans, (
        f'{src.name} builds {sorted(orphans)} and never uses it -- the include '
        f'is constructed and silently never launched.')
