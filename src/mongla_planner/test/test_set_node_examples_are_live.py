"""Every `set_node` example the DSL documents names a parameter its node
applies while running.

rclpy reports success for any declared parameter. Until 2026-11-18 five of
the eight nodes `set_node` reaches read their parameters once at startup and
had no refusal, so four of the six documented examples (`camera
exposure_us`, `lock full_authority_s`, `pnp max_reproj_px`, `flow
max_baseline_s`) succeeded and changed nothing -- or, for the camera, named a
parameter that did not exist. Each node now names what it applies
(`LIVE_PARAMS`, or the keys of its `apply_live` call); this test reads those
from the source and holds the docstring to them.
"""
import ast
import re
from pathlib import Path

SRC = Path(__file__).resolve().parents[2]

# set_node kind -> the files that hold that node's live parameters.
NODE_FILES = {
    'detector': ['mongla_vision/mongla_vision/detector_setup.py'],
    'camera':   ['mongla_vision/mongla_vision/camera_node.py'],
    'tracker':  ['mongla_vision/mongla_vision/tracker_node.py'],
    'lock':     ['mongla_vision/mongla_vision/lock_setup.py'],
    'pnp':      ['mongla_vision/mongla_vision/pnp_node.py'],
    'posefuse': ['mongla_vision/mongla_vision/pose_fuse_node.py'],
    'flow':     ['mongla_vision/mongla_vision/flow/flow_node.py'],
    'manager':  ['mongla_manager/mongla_manager/node_setup.py'],
}


def live_params(kind):
    names = set()
    for rel in NODE_FILES[kind]:
        tree = ast.parse((SRC / rel).read_text())
        for n in ast.walk(tree):
            if (isinstance(n, ast.Assign)
                    and any(getattr(t, 'id', '') == 'LIVE_PARAMS' for t in n.targets)):
                if not isinstance(n.value, ast.Name):    # `LIVE_PARAMS = LIVE_PARAMS`
                    names |= set(ast.literal_eval(n.value))
            if (isinstance(n, ast.Call) and getattr(n.func, 'id', '') == 'apply_live'
                    and len(n.args) > 1 and isinstance(n.args[1], ast.Dict)):
                names |= {k.value for k in n.args[1].keys}
    return names


def documented_examples():
    text = (SRC / 'mongla_planner/mongla_planner/dsl_detector.py').read_text()
    for kind, args in re.findall(r"mongla\.set_node\('(\w+)',\s*(.+)\)[ \t]*(?:#.*)?$", text, re.M):
        if args.startswith('**'):
            yield kind, list(ast.literal_eval(args[2:]))
        else:
            yield kind, [a.split('=')[0].strip() for a in args.split(',')]


def test_the_set_node_table_and_this_test_name_the_same_nodes():
    tree = ast.parse((SRC / 'mongla_planner/mongla_planner/mongla_dsl.py').read_text())
    for n in ast.walk(tree):
        if isinstance(n, ast.Assign) and any(getattr(t, 'id', '') == '_NODE_SUFFIX' for t in n.targets):
            assert set(ast.literal_eval(n.value)) == set(NODE_FILES)
            return
    raise AssertionError('_NODE_SUFFIX not found')


def test_every_node_set_node_reaches_names_its_live_parameters():
    for kind in NODE_FILES:
        assert live_params(kind), f'{kind}: no LIVE_PARAMS and no apply_live'


def test_every_documented_example_is_live():
    seen = 0
    for kind, names in documented_examples():
        live = live_params(kind)
        for name in names:
            ok = name in live or (name.startswith('vision.') and 'vision.*' in live)
            assert ok, f"set_node('{kind}', {name}=...) is documented but not live: {sorted(live)}"
            seen += 1
    assert seen >= 6
