"""B41 -- the camera health line under-counted, because read-then-zero races.

`CameraNode._sent` / `_dropped` are incremented by the CAPTURE THREAD.
`_log_health` runs on a ROS timer thread and used to do:

    hz = self._sent / elapsed
    ... is_healthy(), format, get_logger().info(...) ...
    self._sent = 0                     # <-- everything published in between is LOST

The window is not theoretical and it is not narrow: between the read and the
reset, `_log_health` probes the camera, builds a string and LOGS -- and logging is
I/O, which releases the GIL. Every frame published during that window vanished
from the count.

Measured with a yield in the window (which is what the real logging call is):

    read-then-zero   published 83719   counted    114   LOST 99.86%
    delta            published 137667  counted 137667   LOST  0.00%

That readout is the line an operator uses to decide the pipeline is healthy, so
under-reporting DROPS is precisely the wrong direction to be wrong in.

FIXED BY STRUCTURE, NOT BY A LOCK. The counters are cumulative with exactly ONE
writer (the capture thread); the reader keeps its own high-water marks. Single
writer + private reader state needs no mutual exclusion, costs nothing in the
publish path, and stays correct on a free-threaded build (PEP 703), where `+=` is
not atomic either.
"""

import sys
import threading
import time

import pytest


def _run(reporter, iterations=1500):
    """Drive a counter from one thread while another reports, with a real window."""
    state = {'sent': 0, 'last': 0}
    published = {'n': 0}
    stop = threading.Event()

    def publisher():
        while not stop.is_set():
            state['sent'] += 1
            published['n'] += 1

    old_interval = sys.getswitchinterval()
    sys.setswitchinterval(1e-6)          # make the window reachable in a short test
    t = threading.Thread(target=publisher, daemon=True)
    t.start()
    try:
        counted = 0
        for _ in range(iterations):
            counted += reporter(state)
        stop.set()
        t.join(timeout=1.0)
        counted += reporter(state)
    finally:
        sys.setswitchinterval(old_interval)
    return published['n'], counted


def _read_then_zero(state):
    v = state['sent']
    time.sleep(0)                 # stands in for is_healthy() + format + log I/O
    state['sent'] = 0
    return v


def _delta(state):
    tot = state['sent']
    time.sleep(0)                 # same window, same work
    v = tot - state['last']
    state['last'] = tot
    return v


def test_the_old_read_then_zero_really_does_lose_counts():
    """If this ever stops losing, the test below proves nothing -- so assert it."""
    published, counted = _run(_read_then_zero)
    assert published > 0
    assert counted < published, (
        'read-then-zero did not lose a single count -- the window has closed and '
        'this test no longer demonstrates the defect it exists for')


def test_the_delta_scheme_loses_nothing():
    published, counted = _run(_delta)
    assert counted == published, f'delta lost {published - counted} of {published}'


def test_camera_node_uses_the_delta_scheme_and_never_zeroes():
    """Structural: catch a refactor that reintroduces the reset."""
    import ast
    import pathlib

    src = (pathlib.Path(__file__).resolve().parents[1]
           / 'duburi_vision' / 'camera_node.py').read_text(encoding='utf-8')
    tree = ast.parse(src)
    fn = next(n for n in ast.walk(tree)
              if isinstance(n, ast.FunctionDef) and n.name == '_log_health')
    body = ast.get_source_segment(src, fn) or ''

    assert '_last_sent' in body and '_last_dropped' in body, \
        'the reader must keep its own high-water marks'
    for zeroed in ('self._sent = 0', 'self._dropped = 0'):
        assert zeroed not in body, (
            f'{zeroed!r} is back in _log_health -- that is the read-then-zero race '
            f'(B41); the counters are cumulative and single-writer now')
