"""Render a launch file into the nodes it would start, without starting any.

Walks the launch description the way `ros2 launch` does -- arguments,
conditions, groups, includes, opaque functions -- and stops at each Node: it
resolves the node's substitutions and records `package/executable`, but never
spawns a process. Used by `test_every_switch_combination_renders.py` and by
`fig_launch_switches.py`, so the test and the figure read one rendering.

Packages resolve to `src/`, not `install/`: colcon COPIES launch files into
install/, so rendering install/ would show the last build, not the source.
"""
from __future__ import annotations

import contextlib
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BRINGUP = ROOT / 'src' / 'mongla_manager' / 'launch' / 'bringup.launch.py'


@contextlib.contextmanager
def source_tree():
    """Point `get_package_share_directory` at src/<pkg> while rendering."""
    import ament_index_python.packages as aip
    real = aip.get_package_share_directory

    def share(pkg, *a, **k):
        src = ROOT / 'src' / pkg
        return str(src) if (src / 'launch').is_dir() else real(pkg, *a, **k)
    aip.get_package_share_directory = share
    try:
        yield
    finally:
        aip.get_package_share_directory = real


def declared(launch: Path = BRINGUP) -> dict:
    from launch.actions import DeclareLaunchArgument
    from launch.launch_description_sources import get_launch_description_from_python_launch_file
    ld = get_launch_description_from_python_launch_file(str(launch))
    return {a.name: a for a in ld.entities if isinstance(a, DeclareLaunchArgument)}


def switches(launch: Path = BRINGUP) -> dict:
    """Every boolean switch and every choice argument, with its values."""
    out = {}
    for name, a in declared(launch).items():
        if a.choices:
            out[name] = list(a.choices)
        else:
            d = ''.join(getattr(s, 'text', '') for s in (a.default_value or []))
            if d in ('true', 'false'):
                out[name] = ['true', 'false']
    return out


def default_of(name: str, launch: Path = BRINGUP) -> str:
    a = declared(launch)[name]
    return ''.join(getattr(s, 'text', '') for s in (a.default_value or []))


def render(args: dict, launch: Path = BRINGUP) -> list:
    """`package/executable` of every node `ros2 launch <launch> <args>` would start."""
    from launch import LaunchContext
    from launch.actions import ExecuteProcess
    from launch.launch_description_sources import get_launch_description_from_python_launch_file
    ctx = LaunchContext()
    ctx.launch_configurations.update({k: str(v) for k, v in args.items()})
    started = []

    def walk(entities):
        for e in entities or ():
            cond = getattr(e, 'condition', None)
            if cond is not None and not cond.evaluate(ctx):
                continue
            if isinstance(e, ExecuteProcess):
                if hasattr(e, '_perform_substitutions'):
                    e._perform_substitutions(ctx)
                started.append(f"{getattr(e, 'node_package', '?')}/"
                               f"{getattr(e, 'node_executable', e.__class__.__name__)}")
                continue
            walk(e.visit(ctx))

    walk([get_launch_description_from_python_launch_file(str(launch))])
    return started
