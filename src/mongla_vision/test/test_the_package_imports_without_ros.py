"""`import mongla_vision` must not need rclpy.

⛔ WHY THIS IS A GUARD AND NOT A STYLE RULE. The calibration tool runs on a dev
box with no ROS install, and it loads `calibration/solver.py` BY PATH for that
reason. Twice, a comment in this package records the failure that forced it:
`mongla_vision/__init__.py` imported `preflight`, which imported `rclpy.node`
for one type annotation, so `import mongla_vision` died with
`ModuleNotFoundError: rclpy` on the machine where calibration actually happens.

The chain is gone -- `preflight` needed rclpy only for `assert_vision_ready`,
which was exported for its whole life and called by nothing. This test keeps it
gone, because the annotation that brings it back is one line and looks harmless.

⚠ NOT A BAN ON rclpy IN THE PACKAGE. Every node in it imports rclpy and must.
The rule is only about what `__init__.py` pulls in at import time.
"""
from __future__ import annotations

import importlib
import sys

import pytest


class _NoRclpy:
    """A meta-path finder that refuses rclpy, standing in for a box without it."""

    def find_module(self, name, path=None):          # py2-style API, still honoured
        if name == 'rclpy' or name.startswith('rclpy.'):
            raise ImportError('rclpy is not installed on this machine')
        return None

    def find_spec(self, name, path=None, target=None):
        if name == 'rclpy' or name.startswith('rclpy.'):
            raise ImportError('rclpy is not installed on this machine')
        return None


def test_the_package_imports_with_rclpy_unavailable():
    blocker = _NoRclpy()
    dropped = {k: v for k, v in sys.modules.items()
               if k == 'mongla_vision' or k.startswith('mongla_vision.')}
    for k in dropped:
        del sys.modules[k]
    sys.meta_path.insert(0, blocker)
    try:
        mod = importlib.import_module('mongla_vision')
        # And the public surface is really there, not a degraded stub.
        for name in ('make_camera', 'CAMERA_PROFILES', 'Detection',
                     'wait_vision_state_ready'):
            assert hasattr(mod, name), name
    finally:
        sys.meta_path.remove(blocker)
        for k in [k for k in sys.modules
                  if k == 'mongla_vision' or k.startswith('mongla_vision.')]:
            del sys.modules[k]
        sys.modules.update(dropped)


def test_preflight_itself_does_not_import_rclpy():
    """The specific line that used to do it: `from rclpy.node import Node`, for
    one annotation on a function nobody called."""
    import ast
    from pathlib import Path

    # AST, NOT A SUBSTRING. The docstring EXPLAINS the rclpy executor and the
    # imports that were removed, so a text search matches the explanation and
    # would fail forever -- the same mistake that made the loop-closure guard
    # reject its own comment.
    tree = ast.parse((Path(__file__).resolve().parents[1] / 'mongla_vision'
                      / 'preflight.py').read_text())
    got = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            got |= {a.name.split('.')[0] for a in node.names}
        elif isinstance(node, ast.ImportFrom) and node.module:
            got.add(node.module.split('.')[0])
    assert 'rclpy' not in got, f'preflight imports rclpy again: {sorted(got)}'
    for msgs in ('sensor_msgs', 'vision_msgs', 'qos'):
        assert msgs not in got, (
            'preflight subscribes to nothing -- it polls an existing '
            'VisionState')


def test_the_live_preflight_is_the_state_one():
    """⛔ The node-based `assert_vision_ready` must not come back without a
    caller. `utils/check_pipeline.py` already measures those three topics, and
    it is the right tool: it reports a fixed window, where a preflight exits on
    the first good window and caches."""
    from mongla_vision import preflight

    assert hasattr(preflight, 'wait_vision_state_ready')
    assert not hasattr(preflight, 'assert_vision_ready')
