"""Smoke test for the mission registry.

Every mission registered in `missions/__init__.py::NAMES` must:
  1. Import cleanly (no top-level rclpy/ROS-only deps that would
     fail without a running node).
  2. Expose a `run(client, log)` callable.

These two checks would catch the most common breakage from copy-pasting
a new mission file -- forgetting to register it in NAMES, or naming
the entry point differently.
"""

import inspect

from duburi_planner.missions import NAMES


def test_every_mission_is_callable():
    assert NAMES, 'NAMES registry is empty'
    for name, fn in NAMES.items():
        assert callable(fn), f'mission {name!r} is not callable'


def test_every_mission_has_two_arg_signature():
    """The runner calls `fn(client, log)`. Reject any other signature."""
    for name, fn in NAMES.items():
        sig = inspect.signature(fn)
        params = list(sig.parameters.values())
        # Require exactly two positional-or-keyword params (client, log).
        positionals = [
            p for p in params
            if p.kind in (
                inspect.Parameter.POSITIONAL_ONLY,
                inspect.Parameter.POSITIONAL_OR_KEYWORD,
            )
            and p.default is inspect.Parameter.empty
        ]
        assert len(positionals) == 2, (
            f'mission {name!r} signature must be `(client, log)`, '
            f'got {sig}')


def test_known_missions_are_present():
    """Hard-code the missions we ship today so a deletion is loud."""
    for name in ('square_pattern', 'arc_demo', 'heading_lock_demo'):
        assert name in NAMES, f'expected mission {name!r} to be registered'
