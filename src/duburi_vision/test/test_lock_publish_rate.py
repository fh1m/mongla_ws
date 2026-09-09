"""The ladder tracks on every frame; it must not PUBLISH on every frame.

MEASURED on the vehicle 2026-09-10: `/duburi/vision/downward/lock` ran at
**132.4 Hz** while `/detections` ran at 30.5 Hz, and the ladder costs 16 points
of CPU idle (53.4 % -> 37.4 %).

⛔ THAT RATE IS INTENDED, and the limiter therefore DEFAULTS TO OFF. The
control loop is **500 Hz on the srot board**; the host's 20-50 Hz is a
limitation of the un-ported vision uplink, not the requirement. Capping this
publisher would throttle the stream the uplink exists to feed. The knob exists
for a CPU-bound bench only.

⛔ The one thing a rate limit here must NOT do is delay a RUNG CHANGE.
detection -> follow is the information; a limiter that swallowed it for 25 ms
would be hiding the transition the whole ladder exists to expose.
"""
import ast
import pathlib
import re

_NODE = (pathlib.Path(__file__).resolve().parents[1] / 'duburi_vision'
         / 'lock_node.py')
_SRC = _NODE.read_text()


def test_publish_hz_is_declared_AND_read():
    """A parameter that is declared and never read is decoration -- the defect
    this package has produced four times, by its own launch file's count."""
    assert "declare_parameter('publish_hz'" in _SRC
    assert "self.get_parameter('publish_hz').value" in _SRC, (
        'publish_hz is declared but never read, so the limiter is inert and '
        '`ros2 param set` on it does nothing')
    tree = ast.parse(_SRC)
    names = [n.args[0].value for n in ast.walk(tree)
             if isinstance(n, ast.Call)
             and getattr(n.func, 'attr', '') == 'declare_parameter'
             and n.args and isinstance(n.args[0], ast.Constant)]
    assert names.count('publish_hz') == 1


def test_the_real_value_is_assigned_after_the_placeholder():
    """`_pub_min_dt` is initialised to 0.0 (limiter off) early in __init__ and
    set from the parameter later. If those two ever swap order the limiter is
    silently disabled and the only symptom is 132 Hz again."""
    i = _SRC.index('self._pub_min_dt = 0.0')
    j = _SRC.index('self._pub_min_dt = (1.0 / _phz)')
    assert j > i, 'the placeholder overwrites the configured value'


def test_a_rung_change_is_never_rate_limited():
    """The transition is the information. Assert the guard is conditioned on
    the rung being UNCHANGED, not on time alone."""
    i = _SRC.index('def _publish(self, st, header)')
    body = _SRC[i:i + 1400]
    m = re.search(r'if self\._pub_min_dt > 0\.0 and ([^:]+):', body)
    assert m, 'no rate-limit condition found in _publish'
    assert 'rung == self._last_pub_rung' in m.group(1), (
        f'the limiter fires on {m.group(1)!r} -- it must only throttle while '
        f'the rung is UNCHANGED, or a detection->follow transition is delayed')
    assert body.index('self._last_pub_rung = rung') < body.index('dets = []'), (
        'the remembered rung must be updated before the message is built')


def test_the_limiter_defaults_to_OFF():
    """The board closes the visual loop at 500 Hz, so the default must not cap
    the freshest stream we can produce. A limit is opt-in, for a CPU-bound
    bench."""
    m = re.search(r"declare_parameter\('publish_hz',\s*([0-9.]+)\)", _SRC)
    assert m, 'publish_hz default not found'
    assert float(m.group(1)) == 0.0, (
        f'publish_hz defaults to {m.group(1)}, which throttles /lock. The srot '
        f'control loop is 500 Hz; the host-side 20-50 Hz figure is a present '
        f'limitation of the un-ported vision uplink, not the requirement.')
