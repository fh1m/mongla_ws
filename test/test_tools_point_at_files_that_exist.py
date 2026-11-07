"""Every path and package module a tool names still exists.

Tools are not imported by the suite: most need a camera, a board or a sourced
ROS environment, so a test cannot simply run them. That left them blind to
renames. When `mongla_vision/distance/` became `mongla_vision/flow/`, four
tools kept loading the old path -- among them `flow_console.py`, the instrument
behind the 1.09 cm velocity result -- and each failed on start, for weeks,
with every test green.

So this reads each tool's source instead of running it, and checks the two
ways a tool reaches our code: a repository path written as a string
(`'src/mongla_vision/.../flow_math.py'`), and an import of one of our packages
(`from mongla_vision.flow.flow_math import ...`).
"""
import ast
import re
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
TOOLS = sorted((ROOT / 'tools').glob('*.py')) + sorted((ROOT / 'scripts').glob('*.py'))
OURS = ('mongla_control', 'mongla_manager', 'mongla_vision',
        'mongla_planner', 'mongla_localization', 'mongla_interfaces')
_PATH = re.compile(r"""['"](src/[\w./-]+\.(?:py|yaml|json))['"]""")


def _module_exists(dotted: str) -> bool:
    """`mongla_vision.flow.flow_math` -> the file or package under src/."""
    pkg = dotted.split('.')[0]
    base = ROOT / 'src' / pkg / Path(*dotted.split('.'))
    return base.with_suffix('.py').exists() or (base / '__init__.py').exists()


def _problems(tool: Path):
    text = tool.read_text()
    for m in _PATH.finditer(text):
        if not (ROOT / m.group(1)).exists():
            yield f'names {m.group(1)}, which does not exist'
    for node in ast.walk(ast.parse(text)):
        if isinstance(node, ast.ImportFrom) and node.module and node.level == 0:
            names = [node.module]
        elif isinstance(node, ast.Import):
            names = [a.name for a in node.names]
        else:
            continue
        for dotted in names:
            if dotted.startswith('mongla_interfaces.'):
                continue                      # generated from the IDL at build
            if dotted.split('.')[0] in OURS and '.' in dotted \
                    and not _module_exists(dotted):
                yield f'imports {dotted}, which does not exist'


def test_there_are_tools_to_check():
    assert len(TOOLS) >= 20, [t.name for t in TOOLS]


@pytest.mark.parametrize('tool', TOOLS, ids=lambda t: t.name)
def test_a_tool_points_at_files_that_exist(tool):
    found = list(_problems(tool))
    assert not found, f'{tool.relative_to(ROOT)}: ' + '; '.join(found)
