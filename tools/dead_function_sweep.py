#!/usr/bin/env python3
"""Functions and methods nothing calls -- the defect class inside a LIVE file.

⛔ THE FOURTH FACE OF THE SAME DEFECT, and the one the other three cannot see.
`orphan_sweep.py` finds MODULES nothing imports. `topic_wiring_sweep.py` finds
TOPICS nothing reads. `launch_param_sweep.py` finds PARAMETERS no launch file
passes. All three work at the file or graph level, so a dead FUNCTION inside a
module that is imported, tested and running is invisible to every one of them.

⭐ FOUND ON ITS FIRST RUN: `pool_lines._dominant_angle` -- 50 lines of correct
axial statistics, with the wraparound trap documented, born in the commit that
added the file and called by nothing ever. It was residue of a first design the
module's own docstring records as REJECTED ("a line detector that prefers the
background to the foreground is not a line detector"), left behind when the
implementation moved to second moments. Two module constants existed only to
serve it, and `MIN_SEGMENTS` not even that.

⚠ WHAT IS *NOT* A DEFECT, so the output is not over-read:

  * ROS and framework callbacks, `main`, and anything a `setup.py` entry point
    or a launch file names. Called by name from outside Python.
  * dunders, properties and dataclass hooks -- called by the language.
  * a method that overrides or implements an interface, called through the base
    class rather than by its own name.
  * a public helper a package exports for a consumer outside this repo.

So: a question list, like the other three. For each row, name the caller. If
there is none, it either gets one now or it goes -- that is exactly the choice
section 9 of CLAUDE.md says to make at WRITE time, and this tool is how the
ones written before the rule are found.

    python3 tools/dead_function_sweep.py
    python3 tools/dead_function_sweep.py --package mongla_vision
    python3 tools/dead_function_sweep.py --private   # only _underscore names
"""
from __future__ import annotations

import argparse
import ast
import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
# Every tree that could hold a caller: the packages, the tools, the simulator's
# own workspace, and the mission scripts.
SEARCH = ('src', 'tools', 'sim', 'missions', 'scripts')

# Called by the language, a framework, or the ROS graph -- never by name here.
_EXEMPT_EXACT = {'main', 'setup', 'teardown', 'generate_launch_description'}
_EXEMPT_PREFIX = ('test_', '__')
# ⛔ `on_` AND `_on_` WERE EXEMPT HERE AND MUST NOT BE. A ROS callback is PASSED
# as a reference to `create_subscription`, `create_timer` or `add_on_set_
# parameters_callback`, and the AST scan below sees that reference -- so a
# subscribed callback is found by the ordinary path. Exempting the prefix hid
# the most common shape of this defect instead: a handler that was written and
# never subscribed, or whose subscription was later removed. Framework
# lifecycle hooks that really are called by name from outside go in
# `_EXEMPT_EXACT`, one name at a time.
# ⛔ CLASSES WHOSE PUBLIC METHODS ARE AN API FOR CODE OUTSIDE THIS REPO.
# `MonglaMission` is the mission DSL: operators write mission scripts under
# `~/missions`, which `build_mongla.sh` mirrors in and which are deliberately
# NOT in git (a course re-measured at a venue beats one committed months ago).
# So a DSL verb with no in-repo caller is the NORMAL state, not a defect, and
# `tools/gen_reference.py` already fails if one is undocumented. Private
# methods of the same class are still checked -- nothing outside can reach
# those.
_EXEMPT_PUBLIC_METHODS_OF = {'MonglaMission'}
# A ROS node's own plumbing: a timer or subscription callback is passed as a
# reference, which the AST scan DOES see -- so these are not exempted. Only
# names invoked purely by a framework's own convention are.


def _py_files():
    for top in SEARCH:
        d = ROOT / top
        if not d.is_dir():
            continue
        for p in d.rglob('*.py'):
            if 'install/' in str(p) or 'build/' in str(p):
                continue
            yield p


def _defs_and_refs():
    """Definitions in shipped package code; references from EVERYWHERE."""
    defs: dict[str, list] = {}
    refs: set[str] = set()
    for p in _py_files():
        try:
            tree = ast.parse(p.read_text())
        except (OSError, SyntaxError, UnicodeDecodeError):
            continue
        rel = p.relative_to(ROOT)
        # A definition counts only if it ships: not a test, not a tool, not a
        # launch file. Those are consumers, not capabilities.
        shipped = (rel.parts[0] == 'src' and '/test' not in str(rel)
                   and 'launch' not in rel.parts and rel.name != 'setup.py')
        api, methods = set(), set()
        for cls in (n for n in ast.walk(tree) if isinstance(n, ast.ClassDef)):
            methods |= {f for f in cls.body
                        if isinstance(f, (ast.FunctionDef, ast.AsyncFunctionDef))}
            if cls.name in _EXEMPT_PUBLIC_METHODS_OF:
                api |= {f.name for f in cls.body
                        if isinstance(f, (ast.FunctionDef, ast.AsyncFunctionDef))
                        and not f.name.startswith('_')}
        for node in ast.walk(tree):
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
                if shipped and node.name not in api:
                    defs.setdefault(node.name, []).append(
                        (str(rel), node.lineno, node in methods))
                # ⛔ A DECORATED FUNCTION IS REFERENCED BY ITS DECORATOR, and a
                # `@property` or `@x.setter` is called through attribute access
                # that never spells the name as a call.
                for dec in node.decorator_list:
                    for n in ast.walk(dec):
                        if isinstance(n, ast.Name):
                            refs.add(n.id)
                        elif isinstance(n, ast.Attribute):
                            refs.add(n.attr)
            elif isinstance(node, ast.Name):
                refs.add(node.id)
            elif isinstance(node, ast.Attribute):
                refs.add(node.attr)
            elif isinstance(node, ast.Call):
                # ⛔ A NAME REACHED BY STRING IS STILL A CALLER, and the first
                # version of this sweep called `srot_fc.leak_state` dead when
                # `auv_manager_node` reaches it as
                # `getattr(fc, 'leak_state', lambda: (None, None))()` -- the
                # late-bound form the cross-repo boundary uses on purpose, so a
                # board build without the method degrades instead of crashing.
                # Missing this would have deleted a wired health reporter.
                fn = node.func
                name = (fn.id if isinstance(fn, ast.Name)
                        else fn.attr if isinstance(fn, ast.Attribute) else '')
                if name in ('getattr', 'hasattr', 'setattr', 'delattr'):
                    for arg in node.args[1:2]:
                        if isinstance(arg, ast.Constant) and isinstance(
                                arg.value, str):
                            refs.add(arg.value)
    return defs, refs


def _verb_refs():
    """Every verb in the command registry, which is dispatched BY NAME.

    ⛔ `commands.py` is the repo's one dynamic-dispatch table:
    `getattr(mongla, cmd)(**kwargs)`, where `cmd` is a key of `COMMANDS`. So a
    facade method's caller is a dict key, and nothing in the source ever spells
    it as a call. Adding a verb is documented as "a row in `commands.py` and a
    method of the same name on the facade" -- the row IS the reference.
    """
    p = (ROOT / 'src' / 'mongla_control' / 'mongla_control' / 'commands.py')
    try:
        tree = ast.parse(p.read_text())
    except (OSError, SyntaxError):
        return set()
    for node in ast.walk(tree):
        if (isinstance(node, ast.Assign)
                and any(getattr(t_, 'id', '') == 'COMMANDS'
                        for t_ in node.targets)
                and isinstance(node.value, ast.Dict)):
            return {k.value for k in node.value.keys
                    if isinstance(k, ast.Constant) and isinstance(k.value, str)}
    return set()


def _text_refs():
    """Names spelled in NON-Python files: entry points, launch YAML, configs.

    ⚠ NOT the context docs. A name in a design document is a discussion of a
    capability, not a caller, and counting it would hide exactly the defect
    this looks for.
    """
    out: set[str] = set()
    pat = re.compile(r'[A-Za-z_][A-Za-z0-9_]*')
    for top in SEARCH:
        d = ROOT / top
        if not d.is_dir():
            continue
        for ext in ('*.yaml', '*.yml', '*.xml', '*.cfg', '*.txt'):
            for p in d.rglob(ext):
                if 'install/' in str(p) or 'build/' in str(p):
                    continue
                try:
                    out |= set(pat.findall(p.read_text()))
                except (OSError, UnicodeDecodeError):
                    continue
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--package', default='')
    ap.add_argument('--private', action='store_true',
                    help='only _underscore names -- these can have no caller '
                         'outside their own package, so a dead one is certain')
    a = ap.parse_args()

    defs, refs = _defs_and_refs()
    refs |= _text_refs() | _verb_refs()

    rows = []
    for name, sites in sorted(defs.items()):
        if name in _EXEMPT_EXACT or name.startswith(_EXEMPT_PREFIX):
            continue
        if a.private and not name.startswith('_'):
            continue
        # A METHOD defined more than once is an override or a duck-typed
        # interface: the call site names one of them and cannot say which, so
        # neither can be judged. A module-level FUNCTION defined twice is just
        # two functions -- if one is dead the tool must still say so.
        if len(sites) > 1 and all(m for _f, _l, m in sites):
            continue
        if name in refs:
            continue
        f, ln, _m = sites[0]
        if a.package and a.package not in f:
            continue
        rows.append((f, ln, name))

    for f, ln, name in sorted(rows):
        print(f'{f}:{ln}: {name}()')
    if not rows:
        print('every shipped function is referenced somewhere')
        return 0
    print(f'\n{len(rows)} function(s) with no reference anywhere in '
          f'{", ".join(SEARCH)}.\n'
          '⚠ Not a delete list. Callbacks named by a framework, entry points '
          'and interface methods land here legitimately. For each, NAME THE '
          'CALLER out loud -- if there is none it gets one now or it goes. '
          '`pool_lines._dominant_angle` was found exactly that way.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
