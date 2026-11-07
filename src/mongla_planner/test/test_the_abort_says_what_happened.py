"""The mission abort reports what was done, not what it hopes happened.

Found by the cloud audit, 2026-11-07. The abort printed "Mongla will surface on
positive buoyancy" and a three-second "Surfacing in 3 ... 2 ... 1 ... done"
countdown that waited on nothing. The hull's net buoyancy has never been
measured, and nothing on this path commands SURFACE. And `_try_step` printed
[OK] for any call that did not raise, so a disarm the board REFUSED -- returned
as `success=False`, not raised -- was reported to the operator as taken.

INJECTION-VERIFIED: restore the `getattr(res, 'success', True) is False` check
to always-True and the refusal test fails.
"""
from types import SimpleNamespace

from mongla_planner import mission


def test_a_refused_disarm_is_not_reported_as_done(capsys):
    refused = SimpleNamespace(success=False, message='board refused: still moving')
    assert mission._try_step('disarm', lambda: refused) is False
    err = capsys.readouterr().err
    assert '[OK]' not in err and 'board refused' in err


def test_a_taken_step_and_a_void_step_are_ok(capsys):
    assert mission._try_step('stop', lambda: SimpleNamespace(success=True)) is True
    assert mission._try_step('cancel', lambda: None) is True
    assert capsys.readouterr().err.count('[OK]') == 2


def test_the_abort_promises_no_surfacing(capsys):
    calls = []

    class _M:
        client = SimpleNamespace(cancel_active=lambda: calls.append('cancel'))

        def stop(self):
            calls.append('stop')
            return SimpleNamespace(success=True)

        def disarm(self):
            calls.append('disarm')
            return SimpleNamespace(success=True)

    log = SimpleNamespace(warn=lambda *a: None)
    mission._abort_sequence(_M(), log, 'demo')
    err = capsys.readouterr().err
    assert calls == ['cancel', 'stop', 'disarm']
    assert 'Surfacing in' not in err and 'will surface' not in err
    assert 'not been measured' in err
