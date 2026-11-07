"""Every launch switch, and every pair of them, renders into a real node list.

`command_lint` proves a doc's `name:=value` is a DECLARED argument. That is not
the same as the launch working: a condition can raise on an unexpected string,
an OpaqueFunction can index a missing key, and two switches can each work alone
and fail together. None of that shows until someone types the line on the day.

So this walks the launch description the way `ros2 launch` does -- arguments,
conditions, groups, includes, opaque functions -- under every value of every
boolean switch and every choice, singly and in pairs, and stops at each Node:
it resolves the node's substitutions (name, executable, parameters) and records
it, but never starts a process. A combination that raises, or that produces no
`auv_manager_node`, fails here instead of on the deck.
"""
from __future__ import annotations

import itertools
import pathlib

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import get_launch_description_from_python_launch_file

ROOT = pathlib.Path(__file__).resolve().parents[3]
BRINGUP = ROOT / 'src' / 'mongla_manager' / 'launch' / 'bringup.launch.py'


@pytest.fixture(autouse=True)
def _source_tree(monkeypatch):
    """Resolve packages to src/, not install/.

    Launch files are COPIED into install/ by colcon, so without this the test
    renders the last build and an edit to a launch file is invisible until the
    next one. Injection-verified: deleting the flow node's condition in
    vision_pi.launch.py passed against install/ and fails here.
    """
    import ament_index_python.packages as aip
    real = aip.get_package_share_directory

    def share(pkg, *a, **k):
        src = ROOT / 'src' / pkg
        return str(src) if (src / 'launch').is_dir() else real(pkg, *a, **k)
    monkeypatch.setattr(aip, 'get_package_share_directory', share)


def _declared():
    ld = get_launch_description_from_python_launch_file(str(BRINGUP))
    return {a.name: a for a in ld.entities if isinstance(a, DeclareLaunchArgument)}


def render(args: dict) -> list:
    """Node names `ros2 launch bringup.launch.py <args>` would start."""
    ctx = LaunchContext()
    ctx.launch_configurations.update({k: str(v) for k, v in args.items()})
    started = []

    def walk(entities):
        for e in entities or ():
            cond = getattr(e, 'condition', None)
            if cond is not None and not cond.evaluate(ctx):
                continue
            if isinstance(e, ExecuteProcess):
                # Node._perform_substitutions resolves name, executable and the
                # parameter files exactly as execute() would, minus the spawn.
                if hasattr(e, '_perform_substitutions'):
                    e._perform_substitutions(ctx)
                started.append(f"{getattr(e, 'node_package', '?')}/"
                               f"{getattr(e, 'node_executable', e.__class__.__name__)}")
                continue
            walk(e.visit(ctx))

    walk([get_launch_description_from_python_launch_file(str(BRINGUP))])
    return started


def _switches():
    """Every boolean switch and every choice argument, with its values."""
    out = {}
    for name, a in _declared().items():
        if a.choices:
            out[name] = list(a.choices)
        else:
            d = ''.join(getattr(s, 'text', '') for s in (a.default_value or []))
            if d in ('true', 'false'):
                out[name] = ['true', 'false']
    return out


SWITCHES = _switches()
SINGLES = [{k: v} for k, vals in SWITCHES.items() for v in vals]
PAIRS = [{a: va, b: vb}
         for a, b in itertools.combinations(sorted(SWITCHES), 2)
         for va in SWITCHES[a] for vb in SWITCHES[b]]


def test_the_switch_table_is_not_empty():
    # If argument parsing changed shape, everything below would pass vacuously.
    assert len(SWITCHES) >= 15, sorted(SWITCHES)


@pytest.mark.parametrize('args', [{}] + SINGLES,
                         ids=lambda a: ','.join(f'{k}={v}' for k, v in a.items()) or 'defaults')
def test_each_switch_alone_renders_and_starts_the_manager(args):
    nodes = render(args)
    assert 'mongla_manager/start' in nodes, nodes


def test_every_pair_of_switches_renders():
    broken = []
    for args in PAIRS:
        try:
            render(args)
        except Exception as exc:  # noqa: BLE001 -- collect them all, report once
            broken.append(f'{args}: {type(exc).__name__}: {exc}')
    assert not broken, f'{len(broken)} of {len(PAIRS)} pairs fail:\n' + '\n'.join(broken[:20])


@pytest.mark.parametrize('args, executable', [
    ({'vision': 'true'}, 'mongla_vision/detector_dual_node'),
    ({'vision': 'true', 'flow': 'true'}, 'mongla_vision/flow_node'),
    ({'vision': 'true', 'viewer': 'true'}, 'mongla_vision/vision_display'),
    ({'localization': 'true'}, 'mongla_localization/localization_node'),
])
def test_a_switch_starts_what_it_names(args, executable):
    """A switch that renders but starts nothing is the failure this repo keeps
    finding: a capability in no launch path."""
    assert executable in render(args)


@pytest.mark.parametrize('args, executable', [
    ({'vision': 'false'}, 'mongla_vision/detector_dual_node'),
    ({'vision': 'true', 'flow': 'false'}, 'mongla_vision/flow_node'),
    ({'localization': 'false'}, 'mongla_localization/localization_node'),
])
def test_a_switch_turned_off_starts_nothing(args, executable):
    assert executable not in render(args)


def _loaded_bringup():
    import importlib.util
    spec = importlib.util.spec_from_file_location('bringup_launch', BRINGUP)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


@pytest.mark.parametrize('cfg, named', [
    ({'vision': 'false', 'flow': 'true'}, 'flow:=true'),
    ({'vision': 'false', 'viewer': 'true'}, 'viewer:=true'),
    ({'vision': 'false', 'lock_class': 'gate'}, 'lock_class:=gate'),
])
def test_a_vision_only_switch_without_vision_is_named(cfg, named):
    """`flow:=true` alone starts nothing; the launch now says so."""
    assert named in _loaded_bringup().vision_only_ignored(cfg)


def test_with_vision_on_nothing_is_flagged():
    assert _loaded_bringup().vision_only_ignored(
        {'vision': 'true', 'flow': 'true', 'lock_class': 'gate'}) == []


def test_the_warning_is_in_the_rendered_launch():
    """And it is wired: rendering `flow:=true` alone emits the LogInfo."""
    from launch.actions import LogInfo
    ctx = LaunchContext()
    ctx.launch_configurations.update({'flow': 'true'})
    seen = []

    def walk(entities):
        for e in entities or ():
            cond = getattr(e, 'condition', None)
            if cond is not None and not cond.evaluate(ctx):
                continue
            if isinstance(e, ExecuteProcess):
                continue
            if isinstance(e, LogInfo):
                seen.append(''.join(getattr(m, 'text', str(m)) for m in e.msg))
                continue
            walk(e.visit(ctx))
    walk([get_launch_description_from_python_launch_file(str(BRINGUP))])
    assert any('flow:=true has no effect' in m for m in seen), seen
