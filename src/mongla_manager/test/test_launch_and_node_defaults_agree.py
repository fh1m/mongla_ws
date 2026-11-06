"""B46 -- the launch default and the node default must agree, or be listed.

`bringup.launch.py` is what an operator actually types on the pool deck;
`auv_manager_node.declare_parameter` is what a bare `ros2 run mongla_manager
start` gets. Where the two disagree, the vehicle behaves differently depending
on how it was started -- which is fine, but ONLY if the difference is
deliberate and written down. Where they agree, a doc claiming they differ sends
the operator looking for a knob that is already set.

Both failures had happened. CLAUDE.md and ros2-conventions.md both said the
launch brought the vehicle up on a DVL heading while the node defaulted to the
board's AHRS -- on a hull with no DVL fitted.

This is a text-level check on purpose: it needs no ROS runtime, so it runs in
the same suite as everything else, and it reads the two files an operator
reads.
"""
import os
import re

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG  = os.path.abspath(os.path.join(_HERE, '..'))
_LAUNCH = os.path.join(_PKG, 'launch', 'bringup.launch.py')
_NODE   = os.path.join(_PKG, 'mongla_manager', 'auv_manager_node.py')


# Deliberate differences, each with its reason. Adding an entry here is a
# decision to be made in review, not a way to make this test pass: it means the
# vehicle starts differently under `ros2 launch` than under `ros2 run`.
# There are none today.
_DELIBERATE: dict = {}


def _launch_defaults():
    src = open(_LAUNCH).read()
    return dict(re.findall(
        r"DeclareLaunchArgument\(\s*'([A-Za-z0-9_]+)'\s*,\s*"
        r"default_value=\s*'([^']*)'", src))


def _node_defaults():
    src = open(_NODE).read()
    raw = dict(re.findall(
        r"declare_parameter\(\s*'([A-Za-z0-9_.]+)'\s*,\s*([^),\n]+)", src))
    # A default may legitimately be a NAMED CONSTANT rather than a literal --
    # that is the single-source-of-truth fix B47 made. Resolve those by IMPORTING
    # the constant, never by re-typing its value here: a second copy of the number
    # is the very thing this file exists to catch.
    import importlib
    cc = importlib.import_module('mongla_manager.connection_config')
    out = {}
    for k, v in raw.items():
        v = v.strip().strip("'\"")
        if v.isupper() and hasattr(cc, v):
            v = str(getattr(cc, v))
        out[k] = v
    return out


def test_the_two_default_tables_agree_except_where_listed():
    launch, node = _launch_defaults(), _node_defaults()
    shared = sorted(set(launch) & set(node))
    assert shared, 'parsed no shared params -- the regexes have gone stale'

    unexpected = []
    for k in shared:
        lv, nv = launch[k].strip(), node[k].strip()
        if lv.lower() == nv.lower():
            continue
        if _DELIBERATE.get(k) == (lv, nv):
            continue
        unexpected.append(f'{k}: launch={lv!r} node={nv!r}')
    assert not unexpected, (
        'launch and node defaults disagree and it is not recorded as '
        'deliberate:\n  ' + '\n  '.join(unexpected))


def test_a_listed_difference_has_not_quietly_gone_away():
    """The other direction, which is the one that bit us.

    A doc that says two values differ, when they no longer do, sends the
    operator hunting for a knob that is already set the way they want it. When
    a deliberate difference is resolved, the entry must be removed here -- and
    that forces someone to go and fix the prose that describes it.
    """
    launch, node = _launch_defaults(), _node_defaults()
    stale = [k for k, (lv, nv) in _DELIBERATE.items()
             if launch.get(k, '').strip() == node.get(k, '').strip()]
    assert not stale, (
        f'{stale} listed as a deliberate launch/node difference but the two '
        f'now agree -- drop the entry and correct CLAUDE.md and '
        f'ros2-conventions.md, which describe the split')


def test_no_backend_selector_survives_on_either_path():
    """One board, one heading source, one link. `flight_controller`,
    `yaw_source` and the `mode` connection profile chose between backends that
    no longer exist on this vehicle; a selector with one legal value is a typo
    waiting to be made on the pool deck.
    """
    launch, node = _launch_defaults(), _node_defaults()
    for knob in ('flight_controller', 'yaw_source', 'mode', 'position_source'):
        assert knob not in launch, f'bringup.launch.py declares {knob!r} again'
        assert knob not in node, f'the node declares {knob!r} again'
