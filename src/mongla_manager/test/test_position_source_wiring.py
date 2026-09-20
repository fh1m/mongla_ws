"""`position_source:=flow` -- the wiring that makes three dead verbs runnable.

`drive_forward_dist` and `drive_lateral_dist` duck-type on `get_position` /
`reset_position`. Only the Nortek sources provide those, and the Nortek is not
fitted, so on this hull the verbs have always refused. `FlowPositionSource`
supplies exactly that contract from the bottom camera -- and until this round
it was imported by nothing but its own test.

These tests walk CALLS, not imports. A class that is imported and never
constructed is the defect being fixed here, so a guard that checks an import
would pass against the very state it exists to catch.
"""
import ast
from pathlib import Path

import pytest

pytest.importorskip('rclpy')

_SRC = Path(__file__).resolve().parents[1] / 'mongla_manager'
_NODE = _SRC / 'auv_manager_node.py'


def _tree():
    return ast.parse(_NODE.read_text())


def _func(name, cls=None):
    tree = _tree()
    scope = tree
    if cls:
        scope = next(n for n in tree.body
                     if isinstance(n, ast.ClassDef) and n.name == cls)
    return next(n for n in ast.walk(scope)
                if isinstance(n, ast.FunctionDef) and n.name == name)


# ── the consumer's side of the contract ───────────────────────────────────────

def test_the_motion_layer_still_gates_on_exactly_these_two_methods():
    """Anchor the contract to its CONSUMER.

    If the motion layer ever stops checking these names, the wrapper below
    becomes decorative while every other test here still passes.
    """
    src = (Path(__file__).resolve().parents[2] / 'mongla_control'
           / 'mongla_control' / 'motion_forward.py').read_text()
    assert "hasattr(yaw_source, 'get_position')" in src
    assert 'reset_position' in src


# ── the wiring ────────────────────────────────────────────────────────────────

def test_the_wrapper_is_actually_called_during_setup():
    """Reachability, not presence. The bug being fixed is a class that exists
    and is never constructed."""
    setup = _func('_setup_yaw_source', cls='AUVManagerNode')
    called = [n.func.attr for n in ast.walk(setup)
              if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)]
    assert '_wrap_position_source' in called, \
        '_wrap_position_source is defined but never called from setup'


def test_position_source_defaults_to_none():
    """An unvalidated position source must not arrive by default.

    In-water accuracy is unvalidated (dry bench 103.4 % of truth, +-7 % height
    uncertainty). Off-by-default with a working refusal path IS the capability.
    """
    tree = _tree()
    decls = {}
    for call in ast.walk(tree):
        if (isinstance(call, ast.Call) and isinstance(call.func, ast.Attribute)
                and call.func.attr == 'declare_parameter'
                and len(call.args) == 2
                and isinstance(call.args[0], ast.Constant)):
            if isinstance(call.args[1], ast.Constant):
                decls[call.args[0].value] = call.args[1].value
    assert 'position_source' in decls, 'position_source is not a parameter'
    assert decls['position_source'] == 'none'


# ── behaviour, against the real method ────────────────────────────────────────

class _Log:
    def __init__(self):
        self.msgs = []

    def _rec(self, m):
        self.msgs.append(str(m))

    info = warning = warn = error = fatal = debug = _rec

    def text(self):
        return ' '.join(self.msgs)


class _Inner:
    """A yaw source with NO position -- the premise of the whole fold."""
    name = 'bno085'

    def __init__(self):
        self.closed = False

    def read_yaw(self):
        return 0.0

    def close(self):
        self.closed = True


class _FakeNode:
    """Enough of the manager for `_wrap_position_source` to run for real."""

    def __init__(self, pos='flow', yaw='bno085', srot=False):
        self._is_srot = srot
        self._pos_src_name = pos
        self._yaw_src_name = yaw
        self.yaw_source = _Inner()
        self._log = _Log()
        self.subs = {}

    def get_logger(self):
        return self._log

    def create_subscription(self, msg_type, topic, cb, qos):
        self.subs[topic] = cb
        return object()

    def destroy_subscription(self, sub):
        pass


def _wrap(node):
    from mongla_manager.auv_manager_node import AUVManagerNode
    # Take the trusted-heading list FROM THE REAL CLASS rather than copying it
    # here: a second copy of that list is how it silently drifts out of step
    # with the code it is meant to describe.
    node._POSITION_TRUSTED_YAW = AUVManagerNode._POSITION_TRUSTED_YAW
    AUVManagerNode._wrap_position_source(node)
    return node


def test_flow_gives_the_yaw_source_the_position_contract():
    node = _wrap(_FakeNode(pos='flow'))
    assert hasattr(node.yaw_source, 'get_position')
    assert hasattr(node.yaw_source, 'reset_position')
    assert node.yaw_source.read_yaw() == 0.0, 'delegation broke'


def test_none_leaves_the_source_untouched():
    node = _wrap(_FakeNode(pos='none'))
    assert not hasattr(node.yaw_source, 'get_position')


def test_an_unknown_value_refuses_instead_of_guessing():
    node = _wrap(_FakeNode(pos='flwo'))
    assert not hasattr(node.yaw_source, 'get_position')
    assert 'unknown' in node._log.text()


def test_an_untrusted_heading_warns_because_it_becomes_cross_track_error():
    """Flow measures BODY velocity; this rotates it by the heading. On the
    aluminium hull `mavlink_ahrs` is the untrusted compass, and 5 deg over
    1 m is 8.7 cm of cross-track error."""
    node = _wrap(_FakeNode(pos='flow', yaw='mavlink_ahrs'))
    assert hasattr(node.yaw_source, 'get_position'), 'it should still wrap'
    assert 'cross-track' in node._log.text()


def test_on_srot_mavlink_ahrs_is_the_board_bno_and_does_not_warn():
    """On srot the RIEKF attitude that rotates the displacement comes from
    the board's fused BNO085, not a hull compass."""
    node = _wrap(_FakeNode(pos='flow', yaw='mavlink_ahrs', srot=True))
    assert hasattr(node.yaw_source, 'get_position')
    assert 'cross-track' not in node._log.text()


def test_a_trusted_heading_does_not_warn():
    node = _wrap(_FakeNode(pos='flow', yaw='bno085'))
    assert 'cross-track' not in node._log.text()


def test_closing_the_wrapper_also_closes_the_source_it_wraps():
    """`close` is the ONE method that exists on both, so `__getattr__` never
    fires for it and delegation does not happen by default.

    The manager shuts down with `node.yaw_source.close()`, which after wrapping
    lands on the wrapper. Without an explicit inner call the BNO085's serial
    port (or a DVL's TCP session) is left open, and nothing reports it.
    """
    node = _wrap(_FakeNode(pos='flow'))
    inner = node.yaw_source._inner
    assert not inner.closed
    node.yaw_source.close()
    assert inner.closed, 'the wrapper closed itself and orphaned the real source'


def test_the_wrapped_name_still_says_which_heading_sensor_is_live():
    """The manager wraps BEFORE it prints the startup banner, and the banner
    is the operator's one statement of which heading source is running.

    `name` is defined on the wrapper, so `__getattr__` never forwards it --
    the same reason `close` needed an explicit call. A bare 'flow' would make
    the banner report the position source as if it were the heading source,
    and `_yaw_src_name`'s branches would then append the BNO port or the DVL
    host to it.
    """
    node = _wrap(_FakeNode(pos='flow', yaw='bno085'))
    assert node.yaw_source.name == 'bno085+flow'
