"""B40 -- the MAVLink reader thread could die silently and look like a dead cable.

`AUVManagerNode.reader_loop` is a bare `threading.Thread` target and the ONLY
thing draining the link ("only place recv_match() is called"). Its body had no
exception handling at all: a recorder write, a demux callback
(`note_named_value` / `note_battery` / `note_statustext`) or a malformed frame
could kill the thread outright.

Nothing logged that. Nothing noticed the thread was gone. What the operator saw
was every reading ageing out and `board_link` reporting "no heartbeat within the
stale window" ~3 s later -- i.e. **a host-side software fault presenting as a dead
cable**, at the pool, with the hull in the water. Someone would pull the USB-C and
re-seat the board while the real cause sat in this process.

Two properties are pinned here:
  1. a raising demux callback does NOT kill the loop, and
  2. a dead reader is reported SEPARATELY from the link, saying which side is at
     fault.

This is B23's shape (an unguarded bare Thread body) at the most critical place in
the stack, which is why it gets its own entry rather than a line in that one.
"""

import threading
import time

import pytest

from duburi_manager import health_reporters as hr
from duburi_manager.health import State


# --------------------------------------------------------------------------- #
#  1. the reporter tells the two failures apart                                 #
# --------------------------------------------------------------------------- #

def test_a_dead_reader_is_FAILED_and_blames_the_HOST():
    h = hr.mavlink_reader(lambda: False, lambda: 0, lambda: '')
    assert h.state is State.FAILED
    assert 'HOST' in h.evidence, 'must say which side is at fault'
    assert 'cable' in h.evidence, 'must actively steer the operator off the cable'


def test_survived_faults_are_DEGRADED_not_silent():
    h = hr.mavlink_reader(lambda: True, lambda: 3, lambda: 'ValueError: bad frame')
    assert h.state is State.DEGRADED
    assert '3' in h.evidence and 'ValueError' in h.evidence, \
        'a survived fault still interrupted real telemetry -- say how many and what'


def test_a_healthy_reader_is_OK():
    assert hr.mavlink_reader(lambda: True, lambda: 0, lambda: '').state is State.OK


def test_a_probe_that_raises_is_UNKNOWN_not_OK():
    """UNKNOWN is worse than DEGRADED here: nothing is watching the watcher."""
    def boom(): raise RuntimeError('probe broke')
    assert hr.mavlink_reader(boom, lambda: 0, lambda: '').state is State.UNKNOWN


def test_it_is_a_SEPARATE_reporter_from_board_link():
    """Same name would collapse the two stories back into one."""
    dead = hr.mavlink_reader(lambda: False, lambda: 0, lambda: '')
    assert dead.name == 'mavlink_reader' and dead.name != 'board_link'


# --------------------------------------------------------------------------- #
#  2. the loop body actually survives a raising callback                        #
# --------------------------------------------------------------------------- #

def test_a_raising_demux_callback_does_not_kill_the_loop():
    """The realistic failure: one malformed frame blows up a demux handler.

    Modelled on the real structure -- an inner drain loop whose per-message
    callback raises -- rather than by constructing a whole ROS node.
    """
    faults = {'n': 0}
    drained = {'n': 0}
    stop = threading.Event()

    def demux(msg):
        if msg % 3 == 0:
            raise ValueError('malformed frame')
        drained['n'] += 1

    def loop():
        i = 0
        while not stop.is_set():
            try:
                for _ in range(4):
                    i += 1
                    demux(i)
            except Exception:                    # the guard under test
                faults['n'] += 1
            time.sleep(0.001)

    t = threading.Thread(target=loop, daemon=True)
    t.start()
    time.sleep(0.15)
    alive_during = t.is_alive()
    stop.set()
    t.join(timeout=1.0)

    assert alive_during, 'the loop must survive a raising callback'
    assert faults['n'] > 1, 'the test must actually have exercised the fault path'
    assert drained['n'] > 0, 'and must still have drained real messages'


def test_the_real_reader_loop_body_is_guarded():
    """Structural: catch a future refactor that removes the try.

    Asserted on the source because constructing a live AUVManagerNode needs a
    MAVLink connection; the behavioural half is covered above.
    """
    import ast
    import inspect
    import pathlib

    src = pathlib.Path(
        inspect.getfile(hr)).parent.joinpath('auv_manager_node.py').read_text()
    tree = ast.parse(src)
    fn = next(n for n in ast.walk(tree)
              if isinstance(n, ast.FunctionDef) and n.name == 'reader_loop')
    handlers = [n for n in ast.walk(fn) if isinstance(n, ast.ExceptHandler)]
    assert handlers, (
        'reader_loop has no exception handler -- it is a bare Thread target and '
        'the only thing draining the link; an unguarded raise kills it silently '
        'and presents as a dead cable (B40)')
    seg = ast.get_source_segment(src, fn) or ''
    assert '_reader_faults' in seg, 'faults must be COUNTED, or the guard hides them'
