"""B49 -- a config file must be loaded, or say that it is not.

YAMLs live under `config/`. NONE of them is passed as a `--params-file` by
any launch in this repo -- checked by searching every launch file for a yaml
reference and finding none. So every value in them is inert, and each one
disagrees with the `declare_parameter` default that actually runs.

That is not a documentation nit, it is a measured operator failure. The
`cameras.yaml` docstring records it happening:

    "switching the Pi cameras to the low-latency source by editing the YAML
     changed NOTHING. The launch came up on the old source, published frames,
     ran detections, and logged no error -- the only symptom was a latency
     measurement that did not move."

`tracker.yaml` was the sharpest edge: written in exact `--params-file` shape,
keyed to a node name the vehicle does not use, and quoted by the ledger while
the node shipped a different value (B-83). It is deleted; node parameters go
under `nodes:` in bringup.yaml, which every node checks.

ONE RULE, deliberately: a reader must be able to tell whether editing the file
does anything. Three ways to satisfy that, all honest:

  * a launch loads it;
  * it carries the NOT LOADED marker and says where to set the value instead;
  * it documents that the OPERATOR loads it explicitly with `--params-file`
    (an operator-loaded file would be the legitimate case; none ships now --
    node parameters go under `nodes:` in bringup.yaml).

Deliberately NOT "mirror every value in both places": a second copy of a number
is the disease, not the cure. This checks the one property that matters.
"""
import ast
import os
import re
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[3] / 'src'   # the package root
_MARK = 'NOT LOADED AT RUNTIME'


def _config_yamls():
    return sorted(_ROOT.glob('mongla_*/config/*.yaml'))


def _launch_sources():
    return sorted(_ROOT.glob('mongla_*/launch/*.py'))


def _package_sources():
    """Python that could read a config directly, not via a launch parameter."""
    return sorted(_ROOT.glob('mongla_*/mongla_*/**/*.py'))


def _source_string_literals():
    """Non-docstring string constants in package source, as a SET of whole values.

    The original premise here was "loaded == named in a launch file". That is
    true of every ROS-parameter config and FALSE of a config a module opens
    itself: `target_geometry.py` resolves `target_geometry.yaml` out of the
    installed share directory at call time, so the file is loaded and would
    have been forced to carry a NOT-LOADED marker that is a lie.

    Two wider premises were tried and BOTH were too loose, each verified by
    injection rather than by reading:

    * "the name appears anywhere in package source" -- satisfied by a COMMENT in
      an unrelated file that merely mentions the filename.
    * "the name appears in any non-docstring string literal" -- satisfied by a
      LOG MESSAGE in `lock_node.py` that names the file in a sentence.

    What actually loads a file is a literal that IS the filename, passed to
    open()/os.path.join(). So collect whole-literal matches only. A prose string
    containing the name is not equal to it.
    """
    out = set()
    for path in _package_sources():
        try:
            tree = ast.parse(path.read_text())
        except SyntaxError:                       # pragma: no cover
            continue
        docstrings = set()
        for node in ast.walk(tree):
            if isinstance(node, (ast.Module, ast.ClassDef,
                                 ast.FunctionDef, ast.AsyncFunctionDef)):
                body = getattr(node, 'body', None)
                if (body and isinstance(body[0], ast.Expr)
                        and isinstance(body[0].value, ast.Constant)
                        and isinstance(body[0].value.value, str)):
                    docstrings.add(id(body[0].value))
        for node in ast.walk(tree):
            if (isinstance(node, ast.Constant) and isinstance(node.value, str)
                    and id(node) not in docstrings):
                out.add(node.value)
    return out


def test_there_are_config_yamls_to_check():
    """Guard the guard: a glob that matches nothing passes every test below."""
    assert _config_yamls(), 'no config yamls found -- this file has gone stale'


def test_no_launch_silently_relies_on_one_of_these(): 
    """The premise, re-derived each run rather than trusted.

    If a launch DOES start loading one, this test fails and the marker on that
    file has to come off -- which is the correct outcome, not a nuisance.
    """
    loaded = []
    for lp in _launch_sources():
        src = lp.read_text()
        for y in _config_yamls():
            if y.name in src and 'parameters' in src:
                loaded.append((lp.name, y.name))
    # Record, don't forbid: a loaded file simply must not carry the marker.
    for launch, yname in loaded:
        y = next(p for p in _config_yamls() if p.name == yname)
        assert _MARK not in y.read_text(), (
            f'{yname} is loaded by {launch} but still says it is not')


def test_every_unloaded_config_says_so_in_its_first_lines():
    """The rule. An inert file must announce that it is inert."""
    unloaded = []
    launches = ' '.join(p.read_text() for p in _launch_sources())
    sources = _source_string_literals()
    for y in _config_yamls():
        if y.name in launches:
            continue                              # a launch loads it
        if y.name in sources:
            continue                              # a module opens it by name
        head = '\n'.join(y.read_text().splitlines()[:30])
        if _MARK in head:
            continue                              # says it is inert
        if '--params-file' in head:
            continue                              # operator loads it on purpose
        unloaded.append(str(y.relative_to(_ROOT)))
    assert not unloaded, (
        'config files nothing loads, and which do not say so -- editing one is a '
        'silent no-op on the deck:\n  ' + '\n  '.join(unloaded))


def test_the_marker_points_at_the_real_place_to_set_the_value():
    """A warning that does not say what to do instead just moves the problem."""
    for y in _config_yamls():
        text = y.read_text()
        if _MARK not in text:
            continue
        head = text[:text.index(_MARK) + 2500]
        assert ('ros2 param set' in head or 'ros2 launch' in head
                or '--ros-args' in head or '--params-file' in head), (
            f'{y.name} says it is not loaded but never says how to set the value')


def test_the_retired_tunables_file_stays_gone():
    """`vision_tunables.yaml` was a fourth copy of the vision gains, loaded only
    by a hand-typed `--params-file`. By 2026-11-06 it listed 7 of the 15, so the
    file an operator would have reached for was wrong about half the knobs. The
    gains are set in ONE place now: `nodes:` in bringup.yaml, whose example is
    generated from the table the manager declares.
    """
    assert not (_ROOT / 'mongla_manager' / 'config' / 'vision_tunables.yaml').exists()
