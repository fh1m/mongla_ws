#!/usr/bin/env python3
"""The command reference, generated from the code that decides it.

Every other reference table in this repository was typed by a human and started
drifting the moment someone edited the code beside it. This one is derived:

  * the verbs come from `mongla_control/commands.py` -- the single table the
    CLI, the action server and the Python client all read;
  * whether a verb runs ON THE BOARD or on the host comes from `fc/srot_moves.py`
    (which verbs `_build_params` actually maps to a SROT_MOVE primitive);
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


def board_verbs() -> set:
    """Verb names the board runs as one SROT_MOVE, read out of srot_moves.py."""
    src = (PKG / 'fc' / 'srot_moves.py').read_text()
    body = src[src.index('def _build_params'):]
    nxt = body.find('\ndef ', 1)
    if nxt > 0:
        body = body[:nxt]
    return set(re.findall(r"verb == '([a-z_]+)'", body))


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


def launch_args(only: Path | None = None) -> list[tuple[str, str, str, str]]:
    """(launch file, argument, default, description) in declaration order.

    A passed-through argument (launch_ast) is listed under the file that
    accepts it, with the description saying where its default lives.
    """
    import importlib.util
    la = sys.modules.get('launch_ast')
    if la is None:              # registered first: a dataclass needs its module
        spec = importlib.util.spec_from_file_location('launch_ast', ROOT / 'tools' / 'launch_ast.py')
        la = sys.modules['launch_ast'] = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(la)
    out = []
    files = [only] if only else sorted((ROOT / 'src').glob('*/launch/*.launch.py'))
    for lf in files:
        rel = f'{lf.parent.parent.name}/{lf.name}'
        for a in la.declarations(lf):
            desc = a.description
            if a.source != lf.name:
                desc = (f'(passed to {a.source}, which sets the default) ' + desc).strip()
            out.append((rel, a.name, a.default, desc))
    return out


def missions_table() -> list[str]:
    """One row per mission: what it is, which models it loads and whether they
    ship, and which board moves it calls -- each enters AUTO, which closes the
    depth loop, so each is gated on the two armed bench checks."""
    board = board_verbs()
    shipped = {p.stem for p in (ROOT / 'src/mongla_vision/models').glob('*.yaml')}
    rows = ['| mission | what it does | models it loads | board moves (depth-gated) |',
            '|---|---|---|---|']
    d = ROOT / 'src/mongla_planner/mongla_planner/missions'
    for f in sorted(d.glob('*.py')):
        if f.name.startswith('_'):
            continue
        text = f.read_text()
        tree = ast.parse(text)
        if not any(isinstance(n, ast.FunctionDef) and n.name == 'run' for n in tree.body):
            continue
        doc = (ast.get_docstring(tree) or '').strip().splitlines()
        first = doc[0] if doc else ''
        first = re.sub(r'^[a-z_0-9]+\s*(--|—|-)\s*', '', first).replace('|', '/')
        consts = dict(re.findall(r"^(_?[A-Z_]+)\s*=\s*'([a-z0-9_]+)'", text, re.M))
        stems = set(re.findall(r"set_model\(\s*'([a-z0-9_]+)'", text))
        stems |= {consts[c] for c in re.findall(r"set_model\(\s*([A-Z_]+)\b", text)
                  if c in consts}
        stems = sorted(stems)
        models = ', '.join(f'`{m}`' if m in shipped else f'`{m}` (not shipped)'
                           for m in stems) or '—'
        calls = sorted({n.func.attr for n in ast.walk(tree)
                        if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
                        and n.func.attr in board})
        ref = ', '.join(f'`{c}`' for c in calls) or '—'
        rows.append(f'| `{f.stem}` | {first} | {models} | {ref} |')
    return rows


BRINGUP = ROOT / 'src' / 'mongla_manager' / 'launch' / 'bringup.launch.py'
EXAMPLE = ROOT / 'src' / 'mongla_manager' / 'config' / 'bringup.example.yaml'


def render_example() -> str:
    """Every bringup argument as a commented YAML line: copy to
    ~/.mongla/bringup.yaml and uncomment what the day needs."""
    import textwrap
    L = ['# Bringup settings -- GENERATED by tools/gen_reference.py, do not edit.',
         '# This copy is NOT LOADED AT RUNTIME: it is the template. The launch',
         '# reads ~/.mongla/bringup.yaml (or $MONGLA_BRINGUP).',
         '#',
         '# Copy to ~/.mongla/bringup.yaml on the vehicle and UNCOMMENT only the',
         '# lines you want to change. Then a plain',
         '#     ros2 launch mongla_manager bringup.launch.py',
         '# uses them. `name:=value` on the command line still wins over this file.',
         '# A key that is not listed here stops the launch (a typo cannot vanish).',
         '# How it works: src/mongla_localization/mongla_localization/launch_defaults.py',
         '#',
         '# Each line shows the repository default.', '']
    for _, name, default, desc in launch_args(BRINGUP):
        # Descriptions are indented so that only setting lines read `# key:`.
        for line in textwrap.wrap(desc, 74) or ['(no description)']:
            L.append(f'#   {line}')
        shown = default if default and not re.search(r'[:#{}\[\],&*!|>%@`\'"]|^\s|\s$', default) \
            else f"'{default}'"
        L += [f'# {name}: {shown}', '']
    L += _render_example_sections()
    L += _render_example_nodes()
    return '\n'.join(L)


def _render_example_sections() -> list[str]:
    """`launch:` -- one section per launch file other than bringup, with its
    arguments and defaults. Each applies only when that file is typed."""
    L = ['# ---- other launch files ----------------------------------------------',
         '# `ros2 launch mongla_vision <name>.launch.py` reads `launch: <name>:`.',
         '# A section applies ONLY to the file typed on the command line: when',
         '# bringup includes vision_pi, set those arguments at the top level above.',
         '# What each argument does: .claude/context/reference/commands.md.',
         '#',
         '# launch:']
    for lf in sorted((ROOT / 'src').glob('*/launch/*.launch.py')):
        if lf == BRINGUP:
            continue
        L.append(f"#   {lf.name.removesuffix('.launch.py')}:")
        for _, name, default, _desc in launch_args(lf):
            shown = default if default and not re.search(r'[:#{}\[\],&*!|>%@`\'"]|^\s|\s$', default) \
                else f"'{default}'"
            L.append(f'#     {name}: {shown}')
    return L + ['']


def _render_example_nodes() -> list[str]:
    """The `nodes:` section: the deck-tunable vision gains, with their defaults.

    Read from `vision_tunables.VISION_PARAM_DEFAULTS`, the table the manager
    declares, so this cannot list a gain the node does not have.
    """
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        'vision_tunables', ROOT / 'src/mongla_manager/mongla_manager/vision_tunables.py')
    tun = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(tun)      # stdlib only: no ROS needed to generate
    L = ['# ---- node parameters -------------------------------------------------',
         '# Every knob that is not a launch argument is a node parameter, and it',
         '# goes here, under `nodes:`, by node name. Bringup hands it to every node',
         '# it starts; a node REFUSES a key it does not declare, and a key a launch',
         '# argument already sets (use the argument above instead). Every node',
         '# parameter, by node: .claude/context/reference/commands.md section 4.',
         '#',
         '# The vision gains below are read per goal, so `ros2 param set',
         '# /mongla_manager vision.kp_yaw 80.0` also works live, on the next goal.',
         '#',
         '# nodes:',
         '#   mongla_manager:']
    for name, default in tun.VISION_PARAM_DEFAULTS.items():
        if name == 'vision.mixer_aware':
            continue                      # the `mixer_aware` argument sets it
        shown = 'true' if default is True else 'false' if default is False else default
        L.append(f'#     {name}: {shown}')
    return L + ['']


def node_params() -> dict[str, list[tuple[str, str]]]:
    out: dict[str, list[tuple[str, str]]] = {}
    # Every module of every package, nested ones included (flow/, depth/, web/
    # hold nodes too), and never a test: a probe node there is not the vehicle's.
    # A node split into mixins is listed under the node's own file: that is
    # the file an operator goes looking for.
    sys.path.insert(0, str(ROOT / 'tools'))
    from code_text import mixin_files
    files = sorted((ROOT / 'src').glob('mongla_*/mongla_*/**/*.py'))
    mixins = {m: py for py in files for m in mixin_files(py)}
    for py in files:
        if py in mixins:
            continue
        src = ''.join(f.read_text(encoding='utf-8', errors='ignore')
                      for f in [py] + mixin_files(py))
        if 'declare_parameter' not in src:
            continue
        found = [(m.group(1), re.sub(r'\s+', ' ', m.group(2)).strip())
                 for m in re.finditer(r"declare_parameter\(\s*'([\w.]+)'\s*,\s*([^,\)]+)", src)]
        if found:
            pkg_dir = next(q for q in py.parents if q.parent.name.startswith('mongla_')
                           and q.name == q.parent.name)
            out[f'{pkg_dir.name}/{py.relative_to(pkg_dir).as_posix()}'] = found
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
    board = board_verbs()
    L = ['# Command reference — generated from the code',
         '',
         '> WARNING: **This file is generated.** Do not edit it by hand: run',
         '> `python3 tools/gen_reference.py`. A test fails if it has drifted from the source.',
         '> Every row below was read out of the file named beside it, never written from memory.',
         '',
         f'**{len(C)} verbs.** On the SROT board **{len(board)}** run as one `SROT_MOVE` '
         f'primitive, which the board runs *and brakes* itself; the rest are host-side '
         'loops or single messages.',
         '',
         '## 1. Verbs — `mongla_control/commands.py`', '',
         '| verb | fields | defaults | on srot | what it does |',
         '|---|---|---|---|---|']
    for name in sorted(C):
        spec = C[name]
        where = 'on the board' if name in board else 'host'
        fields = ', '.join(f'`{f}`' for f in spec.get('fields', [])) or '—'
        defaults = ', '.join(f'`{k}={v}`' for k, v in spec.get('defaults', {}).items()) or '—'
        help_text = re.sub(r'\s+', ' ', str(spec.get('help', ''))).strip()
        L.append(f'| `{name}` | {fields} | {defaults} | {where} | {help_text} |')
    L += ['',
          "## 2. Executables — each package's `setup.py`", '',
          '| run it with | module |', '|---|---|']
    for pkg, eps in entry_points().items():
        for exe, target in eps:
            L.append(f'| `ros2 run {pkg} {exe}` | `{target}` |')
    L += ['', '## 3. Launch arguments — `DeclareLaunchArgument`', '',
          'WARNING: `ros2 launch` **silently ignores** a `name:=value` the file does not '
          'declare. Check the name against the table for THAT file, or put it in '
          '`~/.mongla/bringup.yaml`, which refuses unknown names '
          '(example: `src/mongla_manager/config/bringup.example.yaml`).']
    last = None
    for rel, name, default, desc in launch_args():
        if rel != last:
            L += ['', f'### `{rel}`', '',
                  '| argument | default | what it does |', '|---|---|---|']
            last = rel
        L.append(f'| `{name}` | `{default}` | {desc} |')
    L += ['', '## 3b. Missions — `mongla_planner/missions/`', '',
          'Run with `ros2 run mongla_planner mission <name>`. A model marked *not shipped* is '
          'not in `src/mongla_vision/models/`: it arrives on a water day, and until '
          'then the mission refuses at `set_model`. A board move enters AUTO, which '
          'closes the depth loop: every one is gated on the two armed bench checks '
          '(CLAUDE.md §2).', '']
    L += missions_table()
    L += ['', '## 4. Node parameters — `declare_parameter`', '',
          'Set any of these on the vehicle under `nodes:` in `~/.mongla/bringup.yaml`,',
          'keyed by the RUNNING node name -- `ros2 node list` prints them, e.g.',
          '`mongla_manager`, `mongla_tracker_forward`, `mongla_lock_downward`. A misspelt',
          'key, or one a launch argument already sets, stops that node at launch.',
          '',
          'Listed by source file, literal `declare_parameter` calls only. The',
          'manager\'s `vision.*` gains are declared from a table and are listed in',
          '`bringup.example.yaml` instead, with their defaults.', '']
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
          'ERROR: `/mongla/arm`, `/mongla/depth_cmd`, `Attitude.msg` and `RCOverride.msg` appear in '
          'older notes. **None of them exist.**', '']
    return '\n'.join(L) + '\n'


PKGS = ('mongla_control', 'mongla_manager', 'mongla_vision', 'mongla_localization',
        'mongla_planner')
MAP_BEGIN, MAP_END = '<!-- modules:begin -->', '<!-- modules:end -->'


def _summary(py: Path) -> str:
    """A module's docstring summary line, minus a leading `name -- `."""
    doc = ast.get_docstring(ast.parse(py.read_text(encoding='utf-8'))) or ''
    line = doc.strip().split('\n')[0].strip() if doc.strip() else ''
    return re.sub(r'^[\w.]+\s+(--|—)\s+', '', line)


def module_map(pkg: str) -> str:
    """Every module of a package with its own one-line summary, by folder."""
    root = ROOT / 'src' / pkg / pkg
    rows = ['Generated from each module\'s docstring by `tools/gen_reference.py`; a module',
            'with no summary line shows as *(no summary)* until it gets one.', '',
            '| module | what it is |', '|---|---|']
    for py in sorted(root.rglob('*.py'), key=lambda q: (len(q.relative_to(root).parts), str(q))):
        if py.name == '__init__.py' or '__pycache__' in py.parts:
            continue
        what = _summary(py).replace('|', '\\|') or '*(no summary)*'
        rows.append(f'| `{py.relative_to(root).as_posix()}` | {what} |')
    return '\n'.join(rows)


def package_pages() -> dict:
    """Each package page with its module map block brought up to date."""
    out = {}
    for pkg in PKGS:
        page = ROOT / '.claude' / 'context' / 'packages' / pkg / 'README.md'
        text = page.read_text(encoding='utf-8')
        if text.count(MAP_BEGIN) != 1 or text.count(MAP_END) != 1:
            raise SystemExit(f'{page.relative_to(ROOT)}: needs one {MAP_BEGIN} / {MAP_END} pair')
        head, rest = text.split(MAP_BEGIN, 1)
        out[page] = f'{head}{MAP_BEGIN}\n{module_map(pkg)}\n{MAP_END}{rest.split(MAP_END, 1)[1]}'
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--check', action='store_true', help='exit 1 if the doc has drifted')
    args = ap.parse_args()
    outputs = {OUT: render(), EXAMPLE: render_example(), **package_pages()}
    if args.check:
        stale = [p for p, body in outputs.items()
                 if not p.exists() or p.read_text() != body]
        for p in stale:
            print(f'{p.relative_to(ROOT)} has drifted — run tools/gen_reference.py',
                  file=sys.stderr)
        if not stale:
            print('generated references current')
        return 1 if stale else 0
    for p, body in outputs.items():
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(body)
        print(f'{p.relative_to(ROOT)}  {len(body.splitlines())} lines')
    return 0


if __name__ == '__main__':
    sys.exit(main())
