"""What a launch file declares, read without ROS -- including what it passes through.

Three tools read launch arguments statically (command_lint, gen_reference, and
the bringup settings test), because importing a launch file needs the ROS
environment they exist to check. This is the one reader they share.

A launch may re-declare another launch's arguments from that file, so their
defaults stay written once. It says so with a module-level tuple named
`<STEM>_PASSTHROUGH`, and the arguments come from `<stem>.launch.py`:

    VISION_PI_PASSTHROUGH = ('tracking', 'replay', ...)   # in bringup.launch.py
    -> declared as vision_pi.launch.py declares them

So `tracking:=false` on bringup is a real bringup argument to every tool here,
with vision_pi's default and description, exactly as it is at runtime.
"""
from __future__ import annotations

import ast
import re
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


@dataclass(frozen=True)
class Arg:
    name: str
    default: str
    description: str
    choices: tuple | None
    source: str                  # the file that writes the default


def _own(path: Path) -> list[Arg]:
    out = []
    for node in ast.walk(ast.parse(path.read_text())):
        if not (isinstance(node, ast.Call)
                and getattr(node.func, 'id', '') == 'DeclareLaunchArgument'
                and node.args and isinstance(node.args[0], ast.Constant)):
            continue
        default, desc, choices = '', '', None
        for kw in node.keywords:
            if kw.arg == 'default_value' and isinstance(kw.value, ast.Constant):
                default = str(kw.value.value)
            elif kw.arg == 'description' and isinstance(kw.value, ast.Constant):
                desc = re.sub(r'\s+', ' ', str(kw.value.value)).strip()
            elif kw.arg == 'choices' and isinstance(kw.value, (ast.List, ast.Tuple)):
                choices = tuple(str(e.value) for e in kw.value.elts
                                if isinstance(e, ast.Constant))
        out.append(Arg(str(node.args[0].value), default, desc, choices, path.name))
    return out


def passthrough(path: Path) -> dict[str, tuple[str, ...]]:
    """{'vision_pi.launch.py': (names...)} from `<STEM>_PASSTHROUGH` tuples."""
    out = {}
    for node in ast.parse(path.read_text()).body:
        if (isinstance(node, ast.Assign) and len(node.targets) == 1
                and isinstance(node.targets[0], ast.Name)
                and node.targets[0].id.endswith('_PASSTHROUGH')
                and isinstance(node.value, (ast.Tuple, ast.List))):
            stem = node.targets[0].id[:-len('_PASSTHROUGH')].lower()
            out[f'{stem}.launch.py'] = tuple(e.value for e in node.value.elts
                                             if isinstance(e, ast.Constant))
    return out


def find_launch(name: str) -> Path:
    hits = sorted((ROOT / 'src').glob(f'*/launch/{name}'))
    if len(hits) != 1:
        raise FileNotFoundError(f'{name}: {len(hits)} launch files in src/')
    return hits[0]


def declarations(path: Path) -> list[Arg]:
    """Every argument `path` declares, its own first, then what it passes through."""
    own = _own(path)
    seen = {a.name for a in own}
    for target, names in passthrough(path).items():
        theirs = {a.name: a for a in _own(find_launch(target))}
        for n in names:
            if n in theirs and n not in seen:
                own.append(theirs[n])
                seen.add(n)
    return own
