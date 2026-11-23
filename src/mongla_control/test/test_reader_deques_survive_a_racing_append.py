"""The board's reader thread appends to deques other threads walk.

Measured on Kutum 2026-11-23 (bead mongla_ws-43l): `MAVLink reader fault #1:
RuntimeError: deque mutated during iteration`. `battery_trusted` walked the
battery rings with Python generators (`sum(1 for s in ring ...)`) from the
timer while the reader appended, and `statustext_log` walked its ring in a
comprehension. Python may switch threads between any two steps of such a
loop; `list(deque)` copies inside one C call, so every walker snapshots first.

Two guards: one runs `battery_trusted` against an appending thread with the
switch interval at a microsecond; the other reads fc/ and the manager and
fails any Python-level walk (a for loop or a comprehension) of a deque that
is not a snapshot.
"""
import ast
import pathlib
import sys
import threading

from mongla_control.fc import srot_protocol as sp
from mongla_control.fc.srot_telemetry import SrotTelemetry

ROOT = pathlib.Path(__file__).resolve().parents[2]
FILES = sorted((ROOT / 'mongla_control' / 'mongla_control' / 'fc').glob('*.py')) + \
    sorted((ROOT / 'mongla_manager' / 'mongla_manager').glob('*.py'))


def test_battery_trusted_survives_the_reader_appending():
    fc = SrotTelemetry.__new__(SrotTelemetry)
    fc._battery_last, fc._battery_steps = {}, {}
    for k in range(200):
        fc._note_battery_step(0, 14.7 + (k % 3) * 0.01)
    stop = threading.Event()
    errors = []

    def reader():
        k = 0
        while not stop.is_set():
            fc._note_battery_step(0, 14.7 + (k % 5) * 0.4)
            k += 1

    old = sys.getswitchinterval()
    sys.setswitchinterval(1e-6)
    th = threading.Thread(target=reader, daemon=True)
    th.start()
    try:
        for _ in range(3000):
            try:
                fc.battery_trusted(0)
            except RuntimeError as exc:      # the defect, caught to report it
                errors.append(exc)
                break
    finally:
        stop.set()
        th.join()
        sys.setswitchinterval(old)
    assert not errors, errors
    assert sp.BATTERY_STEP_WINDOW > 0


def _is_deque_call(n):
    return isinstance(n, ast.Call) and getattr(n.func, 'attr', getattr(n.func, 'id', '')) == 'deque'


def _deque_attrs(trees):
    """Attributes holding a deque, or a dict of them, across every file: a
    deque made in one mixin is walked in another (`_statustext_log`)."""
    names = set()
    for tree in trees:
        for node in ast.walk(tree):
            if isinstance(node, ast.Assign) and any(_is_deque_call(n) for n in ast.walk(node.value)):
                names |= {t.attr for t in node.targets if isinstance(t, ast.Attribute)}
            # self._x.setdefault(k, deque(...)) / self.__dict__.setdefault('_x', {})...
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute) \
                    and node.func.attr in ('setdefault', 'get') \
                    and any(_is_deque_call(a) for a in node.args):
                base = node.func.value
                if isinstance(base, ast.Attribute):
                    names.add(base.attr)
                if isinstance(base, ast.Call) and base.args and isinstance(base.args[0], ast.Constant):
                    names.add(str(base.args[0].value))
    return names


def _walked(it, deques, locals_from):
    """True when this iterable is a deque walked in Python, not a snapshot."""
    if isinstance(it, ast.Call) and getattr(it.func, 'id', '') == 'list':
        return False                                 # list(d): one C call
    if isinstance(it, ast.Attribute):
        return it.attr in deques
    if isinstance(it, ast.Name):
        return it.id in locals_from
    return False


def _offences(path, deques=None):
    tree = ast.parse(path.read_text())
    deques = _deque_attrs([tree]) if deques is None else deques
    out = []
    for fn in ast.walk(tree):
        if not isinstance(fn, (ast.FunctionDef, ast.AsyncFunctionDef)):
            continue
        # Locals bound straight from a deque-holding attribute: `ring = self._x.get(b)`.
        locals_from = set()
        for n in ast.walk(fn):
            if isinstance(n, ast.Assign) and len(n.targets) == 1 and isinstance(n.targets[0], ast.Name):
                v = n.value
                base = v.func.value if isinstance(v, ast.Call) and isinstance(v.func, ast.Attribute) else v
                if (isinstance(base, ast.Attribute) and base.attr in deques) or _is_deque_call(v):
                    locals_from.add(n.targets[0].id)
        for n in ast.walk(fn):
            iters = []
            if isinstance(n, (ast.For, ast.AsyncFor)):
                iters.append(n.iter)
            elif isinstance(n, (ast.ListComp, ast.SetComp, ast.DictComp, ast.GeneratorExp)):
                iters += [g.iter for g in n.generators]
            for it in iters:
                if _walked(it, deques, locals_from):
                    out.append(f'{path.name}:{it.lineno} {fn.name}')
    return out


def test_no_deque_is_walked_in_python_where_a_thread_may_append():
    deques = _deque_attrs([ast.parse(f.read_text()) for f in FILES])
    assert {'_battery_steps', '_statustext_log'} <= deques, deques
    bad = [o for f in FILES for o in _offences(f, deques)]
    assert not bad, ('snapshot with list(...) before walking: ' + ', '.join(bad))


def test_the_guard_catches_the_walk_that_faulted(tmp_path):
    """Injection: the shape that raised on Kutum is refused, its fix is not."""
    src = (
        'import collections\n'
        'class T:\n'
        '    def note(self, b, v):\n'
        '        ring = self._steps.setdefault(b, collections.deque(maxlen=8))\n'
        '        ring.append(v)\n'
        '    def judge(self, b):\n'
        '        ring = self._steps.get(b)\n'
        '        return sum(1 for s in ring if s > 2)\n')
    f = tmp_path / 'bad.py'
    f.write_text(src)
    assert _offences(f), 'the faulting walk must be caught'
    f.write_text(src.replace('ring = self._steps.get(b)\n', 'ring = list(self._steps.get(b) or ())\n'))
    assert not _offences(f), 'a snapshot must pass'
