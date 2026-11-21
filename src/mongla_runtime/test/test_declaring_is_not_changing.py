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


def _callbacks_that_refuse():
    for f in sorted(SRC.glob('*/*/**/*.py')):
        if '/test/' in str(f):
            continue
        text = f.read_text(errors='replace')
        for m in re.finditer(r'read once at startup', text):
            start = text.rfind('\n    def ', 0, m.start())
            start = max(start, text.rfind('\ndef ', 0, m.start()))
            yield f, text[start:m.start()]


def test_every_callback_that_refuses_a_fixed_name_skips_declarations():
    found = list(_callbacks_that_refuse())
    assert len(found) >= 5, [str(f) for f, _ in found]
    bad = [str(f.relative_to(SRC)) for f, body in found
           if 'fixed' in body and 'changes(' not in body]
    assert not bad, f'refuses a declaration as if it were a change: {bad}'
