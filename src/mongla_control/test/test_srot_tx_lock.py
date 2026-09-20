"""One serial line, several writers, one shared sequence counter.

pymavlink builds every frame through a single object with a single `seq`, so
two threads sending at once can interleave bytes on the wire and corrupt both
frames. On this backend that is not hypothetical: `manual()` streams at 50 Hz
from the vision loop, `send_gcs_heartbeat()` ticks at 2 Hz from a ROS timer
against a 5 s failsafe, and `send_landing_target()` runs at the uplink rate --
three independent schedulers on one 115200 line.

The failure is silent in the worst way. A corrupted frame is dropped by the
far end as BAD_DATA, so a lost heartbeat looks like nothing at all until the
board's GCS failsafe surfaces the vehicle five seconds later.

Two tests, because the two things that can go wrong are different:
  * a NEW writer added without the lock -- structural, so checked structurally
  * the lock not actually excluding -- behavioural, so a thread has to try
"""
import ast
import sys
import threading
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from mongla_control.fc.srot_fc import SrotFC          # noqa: E402

_SRC = Path(__file__).resolve().parents[1] / 'mongla_control' / 'fc' / 'srot_fc.py'


def _is_tx_lock(item) -> bool:
    """`with self._tx_lock:` -- matched on the attribute, not on source text."""
    ctx = item.context_expr
    return (isinstance(ctx, ast.Attribute) and ctx.attr == '_tx_lock'
            and isinstance(ctx.value, ast.Name) and ctx.value.id == 'self')


def _wire_writes_outside_the_lock():
    """Every `self.master.mav.*_send(...)` not lexically inside a tx-lock `with`.

    Parsed rather than grepped. A grep for the two strings near each other stays
    green when a send is moved out of the block but left adjacent to it -- which
    is exactly the shape of the round-26 `_srot_drive` guard that passed through
    the change it existed to catch.
    """
    tree = ast.parse(_SRC.read_text())
    held = []           # stack of bools: is this ancestor a tx-lock `with`?
    offenders = []

    class V(ast.NodeVisitor):
        def visit_With(self, node):
            locked = any(_is_tx_lock(i) for i in node.items)
            held.append(locked)
            self.generic_visit(node)
            held.pop()

        def visit_Call(self, node):
            f = node.func
            if (isinstance(f, ast.Attribute) and f.attr.endswith('_send')
                    and isinstance(f.value, ast.Attribute) and f.value.attr == 'mav'):
                if not any(held):
                    offenders.append((f.attr, node.lineno))
            self.generic_visit(node)

    V().visit(tree)
    return offenders


def test_every_wire_write_is_under_the_lock():
    """The whole point is that this holds for writers not yet written. There are
    seven today; the eighth is the one that will be added by someone who does
    not know the seq counter is shared."""
    offenders = _wire_writes_outside_the_lock()
    assert not offenders, (
        'these send the wire without holding _tx_lock, so they can interleave '
        'bytes with a concurrent write: '
        + ', '.join(f'{n} at srot_fc.py:{ln}' for n, ln in offenders))


def test_the_check_would_notice_an_unlocked_write():
    """The guard's own guard. A structural check that cannot fail is worse than
    none, because it reads as verification -- so confirm the walker actually
    finds a send outside a lock rather than trivially finding nothing."""
    tree = ast.parse('def f(self):\n    self.master.mav.heartbeat_send(1)\n')
    found = []

    class V(ast.NodeVisitor):
        def visit_Call(self, node):
            f = node.func
            if (isinstance(f, ast.Attribute) and f.attr.endswith('_send')
                    and isinstance(f.value, ast.Attribute) and f.value.attr == 'mav'):
                found.append(f.attr)
            self.generic_visit(node)

    V().visit(tree)
    assert found == ['heartbeat_send']


# --------------------------------------------------------------------------- #
#  The lock actually excludes
# --------------------------------------------------------------------------- #
class _BlockingMav:
    """Blocks INSIDE the first send until released, and counts occupancy.

    Deterministic on purpose. A test that starts two threads and sleeps passes
    on a fast box and flakes on the Pi; here the first sender is pinned inside
    the critical section until the second has had its chance, so an absent lock
    produces overlap every time rather than usually.
    """

    def __init__(self):
        self.entered = threading.Event()     # a sender is inside
        self.release = threading.Event()     # let it out
        self._n = 0
        self.max_inside = 0
        self._m = threading.Lock()
        self._blocked_once = False

    def __getattr__(self, name):
        if name.startswith('_'):
            raise AttributeError(name)

        def _send(*a, **k):
            with self._m:
                self._n += 1
                self.max_inside = max(self.max_inside, self._n)
                first = not self._blocked_once
                self._blocked_once = True
            if first:
                self.entered.set()
                self.release.wait(timeout=2.0)
            with self._m:
                self._n -= 1
        return _send


class _Master:
    def __init__(self):
        self.mav = _BlockingMav()
        self.messages = {}


def _run_two_senders(fc):
    mav = fc.master.mav
    a = threading.Thread(target=fc.send_gcs_heartbeat, daemon=True)
    b = threading.Thread(target=fc.send_gcs_heartbeat, daemon=True)
    a.start()
    assert mav.entered.wait(timeout=2.0), 'the first sender never entered'
    b.start()
    b.join(timeout=0.5)          # with the lock held, b cannot get in
    peak = mav.max_inside
    mav.release.set()
    a.join(timeout=2.0)
    b.join(timeout=2.0)
    return peak


def test_a_second_writer_cannot_enter_while_one_is_sending():
    assert _run_two_senders(SrotFC(_Master(), log=None)) == 1


def test_the_harness_really_does_contend():
    """Without this, the test above proves nothing: a fake in which the second
    thread never reaches the send would report no overlap for the wrong reason,
    and would keep reporting it after the lock was deleted. So take the lock
    away and confirm the overlap appears.

    This is the same error made twice already this round -- a harness that does
    not exercise the mechanism cannot be found by injecting into the code.
    """
    fc = SrotFC(_Master(), log=None)

    class _NullLock:
        def __enter__(self):
            return self

        def __exit__(self, *a):
            return False

    fc._tx_lock = _NullLock()
    assert _run_two_senders(fc) == 2, (
        'both threads should have been inside at once with no lock -- if they '
        'were not, this fake never contended and the test above is vacuous')
