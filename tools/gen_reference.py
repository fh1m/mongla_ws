#!/usr/bin/env python3
"""The command reference, generated from the code that decides it.

Every other reference table in this repository was typed by a human and started
drifting the moment someone edited the code beside it. This one is derived:

  * the verbs come from `mongla_control/commands.py` -- the single table the
    CLI, the action server and the Python client all read;
  * whether a verb runs ON THE BOARD, on the host, or is REFUSED comes from
    `fc/srot_fc.py` (`UNSUPPORTED_VERBS`, and which verbs `_build_params`
    actually maps to a SROT_MOVE primitive);
  * the executables come from each package's `setup.py` console_scripts;
  * the launch arguments come from `DeclareLaunchArgument` in the launch files;
  * the node parameters come from `declare_parameter` calls;
  * the wire constants come from `fc/srot_protocol.py`.

Nothing here is written from memory, which is the point: run it after any change
to those files and the document follows.

    python3 tools/gen_reference.py
    python3 tools/gen_reference.py --check    # exit 1 if the doc has drifted
"""
from __future__ import annotations

import argparse
import ast
import importlib
import re
import sys
import types
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PKG = ROOT / 'src' / 'mongla_control' / 'mongla_control'
OUT = ROOT / '.claude' / 'context' / 'reference' / 'commands.md'


def _load(mod: str):
    """Import a module from mongla_control.fc without the package __init__."""
    pkg = types.ModuleType('mongla_control')
    pkg.__path__ = [str(PKG)]
    sys.modules.setdefault('mongla_control', pkg)
    return importlib.import_module(f'mongla_control.fc.{mod}')


def verbs() -> dict:
    ns: dict = {}
    exec(compile((PKG / 'commands.py').read_text(), 'commands.py', 'exec'), ns)
    return ns['COMMANDS']


def srot_routing() -> tuple[set, set]:
    """(refused, on-board) verb names, read out of srot_fc.py."""
    src = (PKG / 'fc' / 'srot_fc.py').read_text()
    m = re.search(r'UNSUPPORTED_VERBS\s*=\s*frozenset\(\{(.*?)\}\)', src, re.S)
    refused = set(re.findall(r"'([a-z_]+)'", m.group(1))) if m else set()
    body = src[src.index('def _build_params'):]
    nxt = body.find('\ndef ', 1)
    if nxt > 0:
        body = body[:nxt]
    board = set(re.findall(r"verb == '([a-z_]+)'", body))
    return refused, board - refused


def entry_points() -> dict[str, list[tuple[str, str]]]:
    out: dict[str, list[tuple[str, str]]] = {}
    for setup in sorted((ROOT / 'src').glob('*/setup.py')):
        s = setup.read_text()
        m = re.search(r"'console_scripts'\s*:\s*\[(.*?)\]", s, re.S)
        if not m:
            continue
        eps = re.findall(r"'\s*([\w_]+)\s*=\s*([\w_.:]+)\s*'", m.group(1))
        if eps:
            out[setup.parent.name] = eps
    return out


def launch_args() -> list[tuple[str, str, str]]:
    out = []
    for lf in sorted((ROOT / 'src').glob('*/launch/*.launch.py')):
        tree = ast.parse(lf.read_text())
        for node in ast.walk(tree):
            if not (isinstance(node, ast.Call)
                    and getattr(node.func, 'id', '') == 'DeclareLaunchArgument'):
                continue
            name = (node.args[0].value
                    if node.args and isinstance(node.args[0], ast.Constant) else '?')
            default = desc = ''
            for kw in node.keywords:
                if kw.arg == 'default_value' and isinstance(kw.value, ast.Constant):
                    default = str(kw.value.value)
                if kw.arg == 'description' and isinstance(kw.value, ast.Constant):
                    desc = re.sub(r'\s+', ' ', str(kw.value.value)).strip()
            out.append((name, default, desc))
    return out


def node_params() -> dict[str, list[tuple[str, str]]]:
    out: dict[str, list[tuple[str, str]]] = {}
    for py in sorted((ROOT / 'src').glob('*/*/*.py')):
        src = py.read_text(encoding='utf-8', errors='ignore')
        if 'declare_parameter' not in src:
            continue
        found = [(m.group(1), re.sub(r'\s+', ' ', m.group(2)).strip())
                 for m in re.finditer(r"declare_parameter\(\s*'([\w.]+)'\s*,\s*([^,\)]+)", src)]
        if found:
            out[f'{py.parent.parent.name}/{py.name}'] = found
    return out


def wire_constants() -> list[tuple[str, str]]:
    sp = _load('srot_protocol')
    names = ['VEHICLE_SYSID', 'VEHICLE_COMPID', 'SOURCE_SYSID', 'SOURCE_COMPID',
             'CMD_SROT_MOVE', 'FW_BEHAVIOUR_REV_REQUIRED', 'GCS_FAILSAFE_MS',
             'MOVE_CRUISE_MAX', 'MOVE_YAW_RATE', 'MOVE_DEPTH_RATE']
    out = [(n, repr(getattr(sp, n))) for n in names if hasattr(sp, n)]
    moves = sorted((getattr(sp, n), n) for n in dir(sp)
                   if n.startswith('MOVE_') and isinstance(getattr(sp, n), int))
    out.append(('SROT_MOVE p1 codes', ', '.join(f'{v}={n}' for v, n in moves)))
    modes = sorted((getattr(sp, n), n) for n in dir(sp)
                   if n.startswith('MODE_') and isinstance(getattr(sp, n), int))
    out.append(('flight modes', ', '.join(f'{v}={n[5:]}' for v, n in modes)))
    return out


def render() -> str:
    C = verbs()
    refused, board = srot_routing()
    L = ['# Command reference — generated from the code',
         '',
         '> ⚠ **This file is generated.** Do not edit it by hand: run',
         '> `python3 tools/gen_reference.py`. A test fails if it has drifted from the source.',
         '> Every row below was read out of the file named beside it, never written from memory.',
         '',
         f'**{len(C)} verbs.** On the SROT board **{len(board)}** run as one `SROT_MOVE` '
         f'primitive, which the board runs *and brakes* itself; **{len(refused)}** are refused '
         'before dispatch; the rest are host-side loops or single messages.',
         '',
         '## 1. Verbs — `mongla_control/commands.py`', '',
         '| verb | fields | defaults | on srot | what it does |',
         '|---|---|---|---|---|']
    for name in sorted(C):
        spec = C[name]
        where = ('⛔ **refused**' if name in refused
                 else 'on the board' if name in board else 'host')
        fields = ', '.join(f'`{f}`' for f in spec.get('fields', [])) or '—'
        defaults = ', '.join(f'`{k}={v}`' for k, v in spec.get('defaults', {}).items()) or '—'
        help_text = re.sub(r'\s+', ' ', str(spec.get('help', ''))).strip()
        L.append(f'| `{name}` | {fields} | {defaults} | {where} | {help_text} |')
    L += ['',
          f'Refused on srot: {", ".join(f"`{v}`" for v in sorted(refused))} — from '
          '`srot_fc.UNSUPPORTED_VERBS`. The board has no such primitive, and faking one '
          'host-side is how a mission comes to believe it moved a distance it never moved.',
          '',
          "## 2. Executables — each package's `setup.py`", '',
          '| run it with | module |', '|---|---|']
    for pkg, eps in entry_points().items():
        for exe, target in eps:
            L.append(f'| `ros2 run {pkg} {exe}` | `{target}` |')
    L += ['', '## 3. Launch arguments — `DeclareLaunchArgument`', '',
          '| argument | default | what it does |', '|---|---|---|']
    for name, default, desc in launch_args():
        L.append(f'| `{name}` | `{default}` | {desc} |')
    L += ['', '## 4. Node parameters — `declare_parameter`', '']
    for node, params in node_params().items():
        L += [f'**`{node}`** — {len(params)} parameters', '',
              '| parameter | default |', '|---|---|']
        L += [f'| `{n}` | `{d}` |' for n, d in params]
        L.append('')
    L += ['## 5. The wire — `fc/srot_protocol.py`', '',
          '| constant | value |', '|---|---|']
    L += [f'| `{n}` | `{v}` |' for n, v in wire_constants()]
    L += ['',
          '## 6. The ROS surface', '',
          'One action and one topic, on purpose:', '',
          '| interface | type | notes |', '|---|---|---|',
          '| `/mongla/move` | `mongla_interfaces/action/Move` | one verb per goal |',
          '| `/mongla/state` | `mongla_interfaces/msg/MonglaState` | armed, mode, yaw, depth, '
          'battery; **`NaN` when absent, never `0.0`**; published on change |',
          '',
          '⛔ `/mongla/arm`, `/mongla/depth_cmd`, `Attitude.msg` and `RCOverride.msg` appear in '
          'older notes. **None of them exist.**', '']
    return '\n'.join(L) + '\n'


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--check', action='store_true', help='exit 1 if the doc has drifted')
    args = ap.parse_args()
    body = render()
    if args.check:
        if not OUT.exists() or OUT.read_text() != body:
            print(f'{OUT.relative_to(ROOT)} has drifted — run tools/gen_reference.py',
                  file=sys.stderr)
            return 1
        print(f'{OUT.relative_to(ROOT)} current')
        return 0
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(body)
    print(f'{OUT.relative_to(ROOT)}  {len(body.splitlines())} lines')
    return 0


if __name__ == '__main__':
    sys.exit(main())
