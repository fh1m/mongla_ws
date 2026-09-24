"""Every method `lock_node` calls on itself must exist.

⛔ WHY THIS EXISTS. `_build_anchor()` called `self._load_bank()`, and the
method was never defined -- a patch matched on the wrong signature and silently
inserted nothing. Nothing caught it: the vision suite never constructs
`LockNode` (it needs rclpy and a ROS graph), so the first thing to notice was
the VEHICLE, where the anchor rung disabled itself with

    [WARN] anchor DISABLED: AttributeError: 'LockNode' object has no
           attribute '_load_bank' -- the follower rung still runs

and degraded exactly as designed, which is why it produced a warning rather
than a crash and could have run a whole pool day like that.

This reads the source with `ast` instead of importing it, so it needs no ROS
and cannot be defeated by an import guard.
"""
import ast
from pathlib import Path

SRC = (Path(__file__).resolve().parents[1] / 'mongla_vision' / 'lock_node.py')


def _lock_node_class():
    tree = ast.parse(SRC.read_text())
    for node in tree.body:
        if isinstance(node, ast.ClassDef) and node.name == 'LockNode':
            return node
    raise AssertionError('LockNode class not found in lock_node.py')


def test_every_self_method_called_is_defined():
    cls = _lock_node_class()
    defined = {n.name for n in ast.walk(cls) if isinstance(n, ast.FunctionDef)}
    # Attributes assigned on self that may be callables handed in elsewhere.
    assigned = {t.attr for n in ast.walk(cls) if isinstance(n, ast.Assign)
                for t in n.targets
                if isinstance(t, ast.Attribute) and isinstance(t.value, ast.Name)
                and t.value.id == 'self'}
    called = set()
    for n in ast.walk(cls):
        if (isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
                and isinstance(n.func.value, ast.Name)
                and n.func.value.id == 'self'):
            called.add(n.func.attr)
    missing = sorted(called - defined - assigned - set(dir(object)))
    # `Node`'s own API is inherited, not defined here; allow anything rclpy
    # supplies by name.
    rclpy_api = {'declare_parameter', 'get_parameter', 'get_logger',
                 'create_subscription', 'create_publisher', 'create_timer',
                 'destroy_node', 'get_clock', 'declare_parameters',
                 'add_on_set_parameters_callback', 'get_name',
                 'create_service', 'create_client', 'set_parameters'}
    missing = [m for m in missing if m not in rclpy_api]
    assert not missing, (
        f'lock_node calls self.{missing} but never defines them. This is the '
        f'_load_bank defect: the anchor rung disables itself on the vehicle '
        f'and logs a WARN rather than failing, so it can run a whole pool day '
        f'broken.')


def test_load_bank_specifically_is_defined():
    """Named on its own because it is the one that shipped broken."""
    cls = _lock_node_class()
    defined = {n.name for n in ast.walk(cls) if isinstance(n, ast.FunctionDef)}
    assert '_load_bank' in defined
