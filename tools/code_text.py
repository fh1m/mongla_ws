"""A module's CODE, for the tests that must assert on source text (issue #22).

WHY. Dozens of guards here read a `.py` file and assert that a string is,
or is not, `in` it. Read raw, that passes on a COMMENT or a docstring that
mentions the line, fails on a harmless reformat, and a `not in` check is
dodged by any change of spacing. This returns the code with comments and
docstrings gone and the layout normalised by `ast.unparse`, so:

  * a commented-out call is ABSENT,
  * `f(x = 1)` and `f(x=1)` read the same,
  * an explanation that quotes the forbidden line no longer trips the guard.

Strings that are not docstrings stay -- they are code, and several guards are
about exactly which literal is passed.

`ast.unparse` is not stable across Python versions: 3.10 writes a tuple
target as `(a, b) = f()` and 3.12 -- the vehicle's Jazzy -- as `a, b = f()`.
Pin the part of a line that cannot vary (the call, the literal), not its
punctuation; the ROS CI runs both versions and catches the difference.

A substring check is still weaker than driving the behaviour; where a test
CAN run the code, it should. This makes the ones that cannot honest.
"""
from __future__ import annotations

import ast
import inspect
import pathlib
import textwrap


class _DropDocstrings(ast.NodeTransformer):
    def _strip(self, node):
        self.generic_visit(node)
        body = getattr(node, 'body', None)
        if (body and isinstance(body[0], ast.Expr)
                and isinstance(body[0].value, ast.Constant)
                and isinstance(body[0].value.value, str)):
            node.body = body[1:] or [ast.Pass()]
        return node

    visit_Module = _strip
    visit_ClassDef = _strip
    visit_FunctionDef = _strip
    visit_AsyncFunctionDef = _strip


def code_only(source: str) -> str:
    """`source` with comments and docstrings removed, layout normalised."""
    tree = ast.parse(textwrap.dedent(source))
    return ast.unparse(_DropDocstrings().visit(tree))


def code_of(obj) -> str:
    """`code_only` of a function, class or module object."""
    return code_only(inspect.getsource(obj))


def mixin_files(path: pathlib.Path) -> list:
    """Sibling modules that hold MIXINS of a class defined in `path`.

    A node too long for one file is one class in several: `class LockNode(
    LockSetupMixin, ..., Node)` with each mixin imported `from .lock_setup
    import LockSetupMixin`. Those files are part of the class, so a test reading
    the class reads them too.
    """
    tree = ast.parse(path.read_text(encoding='utf-8'))
    bases = {b.id for n in tree.body if isinstance(n, ast.ClassDef)
             for b in n.bases if isinstance(b, ast.Name)}
    out = []
    for n in tree.body:
        if isinstance(n, ast.ImportFrom) and n.level == 1 and n.module:
            if any(a.name in bases for a in n.names):
                f = path.parent / (n.module.replace('.', '/') + '.py')
                if f.is_file():
                    out.append(f)
    return out


def source_of(path) -> str:
    """Raw text of a `.py` file and its mixin modules, or of a package directory.

    A module that grows into a package (motion_vision.py -> motion_vision/), or a
    class that grows into mixins (lock_node.py + lock_setup.py ...), keeps
    working for every test that reads it by path.
    """
    p = pathlib.Path(path)
    if p.is_dir():
        return '\n'.join(f.read_text(encoding='utf-8') for f in sorted(p.glob('*.py')))
    return '\n'.join(f.read_text(encoding='utf-8') for f in [p] + mixin_files(p))


def code_of_file(path) -> str:
    """`code_only` of a `.py` file or a package directory."""
    return code_only(source_of(path))
