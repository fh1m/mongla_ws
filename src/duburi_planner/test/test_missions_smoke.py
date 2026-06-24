"""Smoke test for the mission registry.

The registry is auto-discovery based: `missions/__init__.py::discover()`
walks the missions folder (source tree when present, installed package
otherwise) and returns `{mission_name: run_callable}`. There is no static
`NAMES` table to maintain -- a file's stem IS its mission name.

Every discovered mission must:
  1. Import cleanly -- `discover()` exec/imports each module, so a
     top-level rclpy/ROS-only dependency that fails without a running
     node would surface right here.
  2. Expose a `run(duburi, log)` callable with exactly two required
     positional parameters (that is how `mission.py` invokes it).

These checks catch the most common breakage from adding a new mission
file: a missing `run`, or naming the entry point differently.
"""

import inspect

from duburi_planner.missions import discover


def test_discover_finds_missions():
    reg = discover()
    assert reg, 'discover() returned an empty registry'
    for name, fn in reg.items():
        assert callable(fn), f'mission {name!r} is not callable'


def test_every_mission_accepts_runner_call():
    """The runner calls `fn(duburi, log)`. `log` may be optional
    (`run(duburi, log=None)`), but the two-positional call must bind and
    `duburi` must be required."""
    for name, fn in discover().items():
        sig = inspect.signature(fn)
        try:
            sig.bind(object(), object())
        except TypeError as exc:
            raise AssertionError(
                f'mission {name!r} cannot be called as fn(duburi, log): '
                f'{sig} ({exc})')
        required = [
            p for p in sig.parameters.values()
            if p.kind in (
                inspect.Parameter.POSITIONAL_ONLY,
                inspect.Parameter.POSITIONAL_OR_KEYWORD,
            )
            and p.default is inspect.Parameter.empty
        ]
        assert 1 <= len(required) <= 2, (
            f'mission {name!r} signature must be `(duburi, log[=None])`, '
            f'got {sig}')


def test_known_missions_are_present():
    """Hard-code the missions we ship today so a deletion is loud."""
    reg = discover()
    for name in ('demo_square', 'demo_arc', 'demo_heading_lock'):
        assert name in reg, f'expected mission {name!r} to be registered'
