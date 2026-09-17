"""The autotune CLI: briefing without the token, the tune with it, Ctrl-C aborts."""
import threading
from unittest.mock import MagicMock

from duburi_control.fc import srot_protocol as sp
from duburi_manager.srot_autotune import run


def test_without_the_token_only_the_briefing_is_shown():
    fc = MagicMock()
    fc.autotune_briefing.return_value = 'briefing'
    lines = []
    assert run(fc, None, 150.0, threading.Event(), out=lines.append) == 1
    assert lines == ['briefing']
    fc.autotune.assert_not_called()
    assert run(fc, 'run autotune in water', 150.0, threading.Event(), out=lines.append) == 1
    fc.autotune.assert_not_called()                     # near-miss token is not the token


def test_with_the_token_the_tune_runs_and_ctrl_c_is_its_abort():
    fc = MagicMock()
    fc.autotune.return_value = (True, 'autotune finished')
    stop = threading.Event()
    assert run(fc, sp.AUTOTUNE_TOKEN, 120.0, stop, out=lambda s: None) == 0
    args, kw = fc.autotune.call_args
    assert args == (sp.AUTOTUNE_TOKEN,) and kw['timeout'] == 120.0
    assert kw['abort_fn']() is False
    stop.set()                                          # what the SIGINT handler does
    assert kw['abort_fn']() is True


def test_a_refused_tune_exits_nonzero():
    fc = MagicMock()
    fc.autotune.return_value = (False, 'autotune requires ARMED')
    assert run(fc, sp.AUTOTUNE_TOKEN, 150.0, threading.Event(), out=lambda s: None) == 1
