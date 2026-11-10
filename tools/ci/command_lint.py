#!/usr/bin/env python3
"""Check every `ros2 run` / `ros2 launch` line written anywhere in the repo.

Why this exists: `ros2 launch` accepts ANY `name:=value` and silently ignores a
name the launch file does not declare. A typo'd switch on pool day does not
fail -- it just does not happen, and the run looks normal. The same goes for a
doc that still names a model, a mission or a flag that was renamed months ago.
Under time pressure people copy commands from docs, so a wrong command in a doc
is a wrong command on the vehicle.

For each command found in a doc, a docstring or a script, this checks it
against the code that will actually receive it:

  ros2 launch PKG FILE a:=b    the launch file exists; every `a` is declared in
                               it; `b` is one of its `choices` (if it has any);
                               a model stem has a `<stem>.yaml` sidecar
  ros2 run PKG EXE             EXE is a console_script of PKG
  ... mongla VERB --flag x     parsed by the real `mongla` CLI parser
  ... mission NAME             NAME is a mission module with a run()

Nothing is executed: no node starts, no board is touched.

    python3 tools/ci/command_lint.py            # every finding, grouped by file
    python3 tools/ci/command_lint.py --strict   # exit 1 if an OPERATOR doc is wrong

The operator docs are the ones a person copies from on the day (README, the
site, CLAUDE.md, the runbooks). History documents -- the ledger, the defect
register, scouting notes -- quote old commands on purpose and are reported but
never fail.
"""
from __future__ import annotations

import argparse
import ast
import re
import shlex
import sys
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SRC_DIRS = [ROOT / 'src']

# Documents that record what WAS true. They may quote a retired command as
# evidence; they are listed, never failed.
HISTORY = ('measured-bars.md', 'BUGS.md', '/sota/', '/scouting/', '/upstream/',
           '/future/', '/archive/', 'CHANGELOG', '/plans/', 'DECISIONS.md',
           '/research/', 'bumblebee-study')

# Args whose value names a model stem in mongla_vision/models/.
MODEL_ARGS = {'fwd_model', 'dwn_model', 'model', 'model_name'}
# Args whose value is a CSV registry of stems, `stem` or `alias=stem`.
MODEL_CSV_ARGS = {'models', 'fwd_models', 'dwn_models'}

# A value that is a placeholder, not a literal: <x>, ..., $VAR, {x}, or a
# substitution. Checking it would be checking the prose.
_PLACEHOLDER = re.compile(r'[<>{}$*…]|\.\.\.|\bX\b|\|')


@dataclass
class Finding:
    path: str
    line: int
    command: str
    problem: str

    @property
    def operator(self) -> bool:
        return not any(h in self.path for h in HISTORY)


# ── what the code declares ────────────────────────────────────────────────────

def packages() -> dict[str, Path]:
    out = {}
    for base in SRC_DIRS:
        for pkg in base.glob('*/package.xml'):
            out[pkg.parent.name] = pkg.parent
    return out


def console_scripts(pkg_dir: Path) -> set[str]:
    """Every name `ros2 run <pkg> NAME` can start: setup.py console_scripts
    (read through the AST -- a `[...]` in a comment there ends a regex early)
    plus anything a CMakeLists installs with install(PROGRAMS ...)."""
    out: set[str] = set()
    setup = pkg_dir / 'setup.py'
    if setup.exists():
        for node in ast.walk(ast.parse(setup.read_text())):
            if not isinstance(node, ast.Dict):
                continue
            for k, v in zip(node.keys, node.values):
                if (isinstance(k, ast.Constant) and k.value == 'console_scripts'
                        and isinstance(v, ast.List)):
                    out |= {str(e.value).split('=')[0].strip() for e in v.elts
                            if isinstance(e, ast.Constant)}
    cmake = pkg_dir / 'CMakeLists.txt'
    if cmake.exists():
        for body in re.findall(r'install\(\s*PROGRAMS(.*?)DESTINATION',
                               cmake.read_text(), re.S):
            out |= {Path(p).name for p in body.split()}
    return out


def launch_decl(path: Path) -> dict[str, list[str] | None]:
    """{arg name: choices or None} for every argument a launch file accepts,
    including the ones it passes through from another launch (launch_ast)."""
    return {a.name: (list(a.choices) if a.choices else None)
            for a in _launch_ast().declarations(path)}


def _launch_ast():
    """tools/launch_ast.py, the one static reader of launch arguments."""
    import importlib.util
    import sys
    if 'launch_ast' not in sys.modules:
        spec = importlib.util.spec_from_file_location(
            'launch_ast', Path(__file__).resolve().parents[1] / 'launch_ast.py')
        mod = importlib.util.module_from_spec(spec)
        sys.modules['launch_ast'] = mod
        spec.loader.exec_module(mod)
    return sys.modules['launch_ast']


def missions() -> set[str]:
    d = ROOT / 'src/mongla_planner/mongla_planner/missions'
    out = set()
    for f in d.glob('*.py'):
        if f.name.startswith('_'):
            continue
        tree = ast.parse(f.read_text())
        if any(isinstance(n, ast.FunctionDef) and n.name == 'run'
               for n in tree.body):
            out.add(f.stem)
    return out


def model_stems() -> set[str]:
    return {p.stem for p in (ROOT / 'src/mongla_vision/models').glob('*.yaml')}


_CAMERAS = ('forward', 'downward', 'laptop', 'dual')


def node_names() -> set[str]:
    """Every node name the code can create: literal names, plus each
    `mongla_<x>_{camera}` pattern expanded over the camera roles."""
    out: set[str] = set()
    lit = re.compile(r"(?:super\(\)\.__init__\(|Node\(|name=)\s*'([a-z_]+)'")
    pat = re.compile(r"f'(mongla_[a-z_]*)\{[a-z_]+\}'|\['(mongla_[a-z_]*)',\s*cam\]")
    for base in SRC_DIRS:
        for py in base.rglob('*.py'):
            text = py.read_text(errors='replace')
            out |= set(lit.findall(text))
            for a, b in pat.findall(text):
                out |= {f'{a or b}{c}' for c in _CAMERAS}
    return out


def mongla_parser():
    """The real CLI parser, or None when the ROS packages are not importable."""
    for p in ('src/mongla_control', 'src/mongla_planner', 'src/mongla_interfaces'):
        sys.path.insert(0, str(ROOT / p))
    try:
        from mongla_planner.cli import _build_parser
    except Exception:                               # noqa: BLE001 -- no ROS here
        return None
    return _build_parser()


# ── finding commands in text ──────────────────────────────────────────────────

_CMD = re.compile(r'ros2\s+(?:run|launch|param\s+(?:get|set|list|dump))\s+[^\n`|;&#]*')


def _logical_lines(text: str):
    """(line number, text) with backslash continuations joined."""
    lines = text.split('\n')
    i = 0
    while i < len(lines):
        start, buf = i, lines[i]
        while re.search(r'\\\\?\s*$', buf) and i + 1 < len(lines):
            buf = re.sub(r'\\\\?\s*$', ' ', buf) + lines[i + 1].lstrip(' #>')
            i += 1
        yield start + 1, buf
        i += 1


def commands_in(text: str):
    for n, line in _logical_lines(text):
        for m in _CMD.finditer(line):
            cmd = m.group(0).strip().rstrip('.,:)')
            # Prose like "ros2 run mongla_planner mission <name>" is fine;
            # a command with no package after the verb is prose, not a command.
            if len(cmd.split()) >= 4 or cmd.split()[1] == 'param':
                yield n, cmd


def sources():
    pats = ['README.md', 'CLAUDE.md', 'AGENTS.md', 'docs/**/*.md', 'docs/*.html',
            '.claude/context/**/*.md', 'src/**/*.py', 'src/**/*.md',
            'tools/**/*.sh']
    seen = set()
    for pat in pats:
        for p in ROOT.glob(pat):
            if p in seen or '/install/' in str(p) or '/build/' in str(p):
                continue
            # A test holds wrong commands on purpose (injections); it is
            # not something an operator reads.
            if p == Path(__file__).resolve() or '/test/' in str(p):
                continue
            seen.add(p)
            yield p


# ── checking one command ──────────────────────────────────────────────────────

class Checker:
    def __init__(self):
        self.pkgs = packages()
        self.scripts = {k: console_scripts(v) for k, v in self.pkgs.items()}
        self.missions = missions()
        self.models = model_stems()
        self.parser = mongla_parser()
        self.nodes = node_names()
        self._launch: dict[Path, dict] = {}

    def launch_args(self, path: Path) -> dict:
        if path not in self._launch:
            self._launch[path] = launch_decl(path)
        return self._launch[path]

    def check(self, cmd: str, context: str = '', text: str = '') -> list[str]:
        try:
            tok = shlex.split(cmd, comments=False)
        except ValueError:
            tok = cmd.split()
        # A command quoted inside Python source keeps its closing quote/paren.
        tok = [t.rstrip('\'")') for t in tok]
        tok = [t for t in tok if t]
        if tok[1] == 'param':
            return self._check_param(tok)
        verb, pkg = tok[1], tok[2]
        if pkg not in self.pkgs:
            return []                       # someone else's package (foxglove...)
        rest = tok[3:]
        if verb == 'launch':
            return self._check_launch(pkg, rest, text)
        return self._check_run(pkg, rest, context, text)

    def _check_param(self, tok):
        node = next((t for t in tok[3:] if t.startswith('/')), None)
        if node is None or _PLACEHOLDER.search(node):
            return []
        name = node.strip('/').split('/')[-1]
        if name.startswith('mongla') and name not in self.nodes:
            return [f'no node named `{node}` -- the code creates '
                    f'{sorted(n for n in self.nodes if n.startswith(name.split("_")[0] + "_" + (name.split("_") + [""])[1]))[:4] or "none like it"}']
        return []

    def _check_launch(self, pkg, rest, text=''):
        # A mission's own docstring may name a model that arrives on a water
        # day: it loads it with set_model() in the same file, and the generated
        # mission table says it is not shipped. That is not a typo.
        self._future = set(re.findall(r"set_model\(\s*'([a-z0-9_]+)'", text))
        if not rest:
            return []
        fname = rest[0]
        if _PLACEHOLDER.search(fname):
            return []
        path = self.pkgs[pkg] / 'launch' / fname
        if not path.exists():
            return [f'no launch file {pkg}/launch/{fname}']
        decl = self.launch_args(path)
        out = []
        for t in rest[1:]:
            if t.startswith('--'):
                continue
            if ':=' not in t:
                continue
            name, value = t.split(':=', 1)
            value = value.strip('"\'')
            if _PLACEHOLDER.search(name):
                continue
            if name not in decl:
                out.append(f'`{name}:=` is not an argument of {fname} '
                           '(ros2 launch ignores it SILENTLY)')
                continue
            if _PLACEHOLDER.search(value) or not value:
                continue
            choices = decl[name]
            if choices and value not in choices:
                out.append(f'`{name}:={value}` is not one of {choices}')
            if name in MODEL_ARGS and value not in self.models | self._future:
                out.append(f'`{name}:={value}`: no model sidecar '
                           f'models/{value}.yaml')
            if name in MODEL_CSV_ARGS:
                for item in value.split(','):
                    stem = item.split('=')[-1].strip()
                    if stem and stem not in self.models | self._future:
                        out.append(f'`{name}:=...{stem}...`: no model sidecar '
                                   f'models/{stem}.yaml')
        return out

    def _check_run(self, pkg, rest, context, text=''):
        if not rest:
            return []
        exe = rest[0]
        if _PLACEHOLDER.search(exe):
            return []
        if exe not in self.scripts[pkg]:
            return [f'`{exe}` is not an executable of {pkg}']
        args = [a for a in rest[1:]]
        if '--ros-args' in args:
            args = args[:args.index('--ros-args')]
        if pkg == 'mongla_planner' and exe == 'mission':
            names = [a for a in args if not a.startswith('-')]
            # A document that says "drop this in missions/<name>.py" is
            # teaching the reader to CREATE that mission; it is not a typo.
            if names and not _PLACEHOLDER.search(names[0]) \
                    and names[0] not in self.missions \
                    and f'missions/{names[0]}.py' not in text:
                return [f'no mission named `{names[0]}`']
        if pkg == 'mongla_planner' and exe == 'mongla' and args \
                and self.parser is not None:
            return self._check_cli(args, context)
        return []

    def _check_cli(self, args, context):
        if any(_PLACEHOLDER.search(a) for a in args):
            # Still check the verb itself when the values are placeholders.
            args = args[:1]
            if _PLACEHOLDER.search(args[0]):
                return []
            sub = self.parser._subparsers._group_actions[0].choices
            if args[0] not in sub:
                return [f'`mongla {args[0]}`: no such verb']
            return []
        import contextlib
        import io
        err = io.StringIO()
        try:
            with contextlib.redirect_stderr(err), \
                    contextlib.redirect_stdout(io.StringIO()):
                self.parser.parse_args(args)
        except SystemExit as e:
            if e.code:
                msg = err.getvalue().strip().splitlines()
                return [f'`mongla {" ".join(args)}`: '
                        f'{msg[-1] if msg else "rejected by the CLI"}']
        return []


def lint() -> list[Finding]:
    ck = Checker()
    out = []
    for path in sources():
        try:
            text = path.read_text()
        except (UnicodeDecodeError, OSError):
            continue
        lines = text.split('\n')
        for n, cmd in commands_in(text):
            ctx = (str(path.relative_to(ROOT)) + ' '
                   + ' '.join(lines[max(0, n - 3):n + 1]))
            for problem in ck.check(cmd, ctx, text):
                out.append(Finding(str(path.relative_to(ROOT)), n, cmd, problem))
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--strict', action='store_true',
                    help='exit 1 if any operator-facing doc has a wrong command')
    ap.add_argument('--operator-only', action='store_true')
    a = ap.parse_args()
    found = lint()
    shown = [f for f in found if f.operator or not a.operator_only]
    last = None
    for f in sorted(shown, key=lambda f: (not f.operator, f.path, f.line)):
        if f.path != last:
            print(f'\n{f.path}' + ('' if f.operator else '   [history: reported only]'))
            last = f.path
        print(f'  {f.line:>5}: {f.problem}\n         {f.command[:110]}')
    ops = sum(f.operator for f in found)
    print(f'\n{ops} wrong command(s) in operator docs, '
          f'{len(found) - ops} in history docs')
    return 1 if (a.strict and ops) else 0


if __name__ == '__main__':
    sys.exit(main())
