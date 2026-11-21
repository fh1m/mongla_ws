"""A parameter's first declaration is not a live change (BUGS B-145).

rclpy runs a node's on-set callbacks when a parameter is declared. `spin`
declares `tether_bound` on every node, and every callback that refused the
names it cannot apply live refused that declaration too: on the vehicle the
trackers, PnP and pose fusion died at start and respawned forever, with
every unit test passing, because no test started a node through `spin`.
"""
import pathlib
import re

import pytest

rclpy = pytest.importorskip('rclpy')

from mongla_runtime import tether  # noqa: E402

SRC = pathlib.Path(__file__).resolve().parents[2]


@pytest.fixture
def node():
    from rclpy.node import Node
    rclpy.init()
    n = Node('declaring_is_not_changing')
    yield n
    n.destroy_node()
    rclpy.shutdown()


def _refusing_callback(n, live=('conf',)):
    from rcl_interfaces.msg import SetParametersResult

    def on_set(params):
        fixed = [p.name for p in tether.changes(n, params) if p.name not in live]
        return SetParametersResult(successful=not fixed, reason='read once at startup')
    n.add_on_set_parameters_callback(on_set)


def test_a_node_that_refuses_fixed_names_still_survives_spin_declaring_on_it(node):
    node.declare_parameter('fixed_at_start', 30)
    _refusing_callback(node)
    tether.declare(node)                         # what spin() does to every node
    assert node.has_parameter(tether.PARAM)


def test_a_real_change_to_a_fixed_name_is_still_refused(node):
    from rclpy.parameter import Parameter
    node.declare_parameter('track_buffer', 30)
    _refusing_callback(node)
    r = node.set_parameters([Parameter('track_buffer', value=150)])[0]
    assert not r.successful
    assert node.get_parameter('track_buffer').value == 30


def test_apply_live_survives_spin_declaring_on_it(node):
    """The live table holds only its own names: a declaration reaching it raised KeyError."""
    from mongla_vision.live_params import apply_live
    node.declare_parameter('max_reproj_px', 4.0)
    got = []
    apply_live(node, {'max_reproj_px': got.append})
    tether.declare(node)
    assert node.has_parameter(tether.PARAM) and got == []


def _body(text, name):
    i = text.find(f'def {name}(')
    if i < 0:
        return None
    j = text.find('\n    def ', i + 1)
    return text[i:j if j > 0 else len(text)]


def test_every_parameter_callback_skips_declarations():
    """Every registered on-set callback drops declarations before it refuses or applies."""
    files = {f: f.read_text(errors='replace') for f in sorted(SRC.glob('*/*/**/*.py'))
             if '/test/' not in str(f)}
    names = sorted({m.group(1).split('.')[-1] for t in files.values()
                    for m in re.finditer(r'add_on_set_parameters_callback\(([\w.]+)\)', t)})
    assert len(names) >= 5, names
    bad = []
    for n in names:
        bodies = [b for t in files.values() if (b := _body(t, n))]
        if not bodies or not all('params = changes(' in b for b in bodies):
            bad.append(n)
    assert not bad, f'a declaration reaches these callbacks as a change: {bad}'
