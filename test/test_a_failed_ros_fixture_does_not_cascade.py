"""A ROS fixture whose setup fails is shut down, so later tests can init.

The suites run in one process. On 2026-11-18 one discovery timeout in a
batighor fixture left its rclpy context up, and every later `rclpy.init()`
on the Jazzy runner raised "Context.init() must only be called once" --
dozens of failures in three packages from one. The root conftest's setup hook
shuts down a context the failing setup itself created, and only that one.
"""
import types

import pytest

rclpy = pytest.importorskip('rclpy')

import importlib.util  # noqa: E402
import pathlib  # noqa: E402

# The repository root's conftest, by PATH. `import conftest` returns whichever
# conftest the one-process run imported first -- `mongla_control`'s on the
# Jazzy runner, which has no such hook.
_spec = importlib.util.spec_from_file_location(
    '_root_conftest', pathlib.Path(__file__).resolve().parents[1] / 'conftest.py')
conftest = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(conftest)


def _drive(setup_inits: bool, setup_fails: bool):
    gen = conftest.pytest_runtest_setup(None)
    next(gen)
    if setup_inits:
        rclpy.init()
    out = types.SimpleNamespace(excinfo=(AssertionError,) if setup_fails else None)
    with pytest.raises(StopIteration):
        gen.send(out)


def test_a_failed_setup_that_started_rclpy_is_shut_down():
    _drive(setup_inits=True, setup_fails=True)
    assert not rclpy.ok()


def test_a_passing_setup_keeps_its_context():
    _drive(setup_inits=True, setup_fails=False)
    try:
        assert rclpy.ok()
    finally:
        rclpy.shutdown()


def test_a_context_that_was_already_up_is_left_alone():
    rclpy.init()
    try:
        _drive(setup_inits=False, setup_fails=True)
        assert rclpy.ok()
    finally:
        rclpy.shutdown()
