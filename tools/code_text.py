"""A module's CODE, for the tests that must assert on source text (issue #22).

⛔ WHY. Dozens of guards here read a `.py` file and assert that a string is,
or is not, `in` it. Read raw, that passes on a COMMENT or a docstring that
mentions the line, fails on a harmless reformat, and a `not in` check is
dodged by any change of spacing. This returns the code with comments and
docstrings gone and the layout normalised by `ast.unparse`, so:

  * a commented-out call is ABSENT,
  * `f(x = 1)` and `f(x=1)` read the same,
  * an explanation that quotes the forbidden line no longer trips the guard.

Strings that are not docstrings stay -- they are code, and several guards are
about exactly which literal is passed.

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


def source_of(path) -> str:
    """Raw text of a `.py` file, or of every `.py` file in a package directory.

    A module that grows into a package (motion_vision.py -> motion_vision/)
    keeps working for every test that reads it by path.
    """
    p = pathlib.Path(path)
    if p.is_dir():
        return '\n'.join(f.read_text(encoding='utf-8') for f in sorted(p.glob('*.py')))
    return p.read_text(encoding='utf-8')


def code_of_file(path) -> str:
    """`code_only` of a `.py` file or a package directory."""
    return code_only(source_of(path))
