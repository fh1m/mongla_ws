"""`debug:=true` must tag frames from the threads that actually send them.

⛔ THE DEFECT. `set_enabled` wrote a **contextvar**, and a new thread in CPython
starts with an EMPTY context -- it does not inherit the one that set the flag.
The manager runs a `MultiThreadedExecutor`, so every action callback, and
therefore every `command_scope()` in the facade and in `_run_srot_move`, runs on
a worker thread where `_enabled.get()` returned the default `False`. The scope
took its no-op branch, `current_tag()` stayed empty, and `debug:=true` produced
**no `cmd=` tag on any frame, ever**.

It looked like it worked. The parameter was read, `set_enabled(True)` ran, the
logger level went to DEBUG and `[MAV ...]` lines appeared -- just without the
one thing the module exists for. The module's own docstring anticipated half of
it ("background daemons ... will simply lack the `cmd=` tag") and called that an
accepted trade; the executor's workers are not daemons this file spawned, they
are where the entire command path runs.

⭐ THE FIX IS THE DISTINCTION THE TYPES SHOULD HAVE MADE. Two different things
were both contextvars:
  * `_cmd_id` -- WHICH verb is in flight. Genuinely per-scope, per-thread, and
    nested-scope-safe. Stays a contextvar.
  * `_enabled` -- whether tracing is on AT ALL. A process-wide switch, set once
    at startup. A plain module global, which every thread sees.
"""
from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor

import pytest

from mongla_control import tracing


@pytest.fixture(autouse=True)
def _restore():
    was = tracing.is_enabled()
    yield
    tracing.set_enabled(was)


def _tag_from_a_scope(verb='yaw_right'):
    with tracing.command_scope(verb):
        return tracing.current_tag()


def test_the_flag_reaches_a_worker_thread():
    """⛔ THE FAILING CASE. This is the manager's real shape: the flag is set on
    the main thread at startup, and every verb runs on an executor worker."""
    tracing.set_enabled(True)
    with ThreadPoolExecutor(max_workers=2) as ex:
        assert ex.submit(tracing.is_enabled).result() is True, \
            'the tracing switch did not cross a thread boundary'


def test_a_verb_on_a_worker_thread_is_tagged():
    """The consequence, end to end: `command_scope` must not take its no-op
    branch just because it is running off the main thread."""
    tracing.set_enabled(True)
    with ThreadPoolExecutor(max_workers=2) as ex:
        assert ex.submit(_tag_from_a_scope).result() == 'cmd=yaw_right'


def test_two_workers_keep_their_own_verbs():
    """⚠ AND THE PART THAT MUST NOT REGRESS. Making the switch global must not
    make the VERB global: two commands in flight on two threads are exactly
    what a MultiThreadedExecutor produces, and one must not wear the other's
    tag."""
    tracing.set_enabled(True)
    with ThreadPoolExecutor(max_workers=2) as ex:
        a = ex.submit(_tag_from_a_scope, 'surface')
        b = ex.submit(_tag_from_a_scope, 'fire')
        assert {a.result(), b.result()} == {'cmd=surface', 'cmd=fire'}


def test_off_is_still_off_everywhere():
    tracing.set_enabled(False)
    assert _tag_from_a_scope() == ''
    with ThreadPoolExecutor(max_workers=1) as ex:
        assert ex.submit(_tag_from_a_scope).result() == ''


def test_the_switch_is_not_a_contextvar():
    """⛔ THE SPECIFIC MECHANISM, named so it cannot come back quietly. A
    contextvar here is not a style choice -- it silently disables the whole
    module in any threaded host."""
    import contextvars

    assert not isinstance(getattr(tracing, '_enabled', None),
                          contextvars.ContextVar), \
        'the tracing switch is a contextvar again -- a new thread starts with ' \
        'an empty context and will never see it'
    # the VERB, by contrast, must stay per-context
    assert isinstance(tracing._cmd_id, contextvars.ContextVar)
