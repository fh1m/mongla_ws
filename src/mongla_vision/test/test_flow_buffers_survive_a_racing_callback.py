"""The flow worker reads gyro buffers that an IMU callback appends to.

Measured on Kutum 2026-11-23: `[FLOW ] worker: RuntimeError('deque mutated
during iteration')`. The callback (`flow_setup._on_rates`) appends on one
thread while the worker walks the same deque in a Python-level loop on
another; Python may switch threads between any two steps of that loop.
`list(deque)` copies inside one C call under the GIL, so every reader takes
one snapshot and walks that.

Two guards: one runs `TimeOffset.estimate` against a thread appending with
the switch interval forced to a microsecond; the other reads the flow
mixins' source and fails any use of a shared buffer other than appending,
testing, measuring or snapshotting it.
"""
import ast
import math
import pathlib
import sys
import threading

from mongla_vision.flow.flow_timing import TimeOffset

FLOW = pathlib.Path(__file__).resolve().parents[1] / 'mongla_vision' / 'flow'
SHARED = {'_rate_buf', '_yaw_rate_buf'}
# A shared buffer may be appended to, tested for emptiness, measured, or
# copied whole; anything else walks it while the callback may append.
ALLOWED_CALLS = {'list', 'len'}


def test_estimate_survives_a_callback_appending_on_another_thread():
    est = TimeOffset()
    for k in range(4000):
        t = k * 0.01
        est.add_image_yaw(t, math.sin(t * 3.0))
        est.add_gyro_yaw(t, math.sin(t * 3.0))
    stop = threading.Event()

    def callback():
        k = 4000
        while not stop.is_set():
            t = k * 0.01
            est.add_gyro_yaw(t, math.sin(t * 3.0))
            est.add_image_yaw(t, math.sin(t * 3.0))
            k += 1

    old = sys.getswitchinterval()
    sys.setswitchinterval(1e-6)
    th = threading.Thread(target=callback, daemon=True)
    th.start()
    try:
        for _ in range(30):
            est.estimate()
    finally:
        stop.set()
        th.join()
        sys.setswitchinterval(old)


def _uses(tree):
    parents = {}
    for node in ast.walk(tree):
        for child in ast.iter_child_nodes(node):
            parents[child] = node
    for node in ast.walk(tree):
        if (isinstance(node, ast.Attribute) and node.attr in SHARED
                and isinstance(node.value, ast.Name) and node.value.id == 'self'):
            yield node, parents.get(node)


def _safe(node, parent):
    if isinstance(parent, ast.Attribute) and parent.attr == 'append':
        return True
    if isinstance(parent, ast.Call) and node in parent.args:
        return isinstance(parent.func, ast.Name) and parent.func.id in ALLOWED_CALLS
    if isinstance(parent, (ast.If, ast.BoolOp, ast.UnaryOp)):
        return True
    if isinstance(parent, (ast.Assign, ast.AnnAssign)) and node is not getattr(parent, 'value', None):
        return True
    return False


def test_no_flow_reader_walks_a_shared_buffer_in_place():
    bad = []
    for path in sorted(FLOW.glob('*.py')):
        tree = ast.parse(path.read_text())
        for node, parent in _uses(tree):
            if not _safe(node, parent):
                bad.append(f'{path.name}:{node.lineno} self.{node.attr} '
                           f'under {type(parent).__name__}')
    assert not bad, ('walk a snapshot, list(self.<buf>), not the deque the '
                     'IMU callback appends to:\n' + '\n'.join(bad))
