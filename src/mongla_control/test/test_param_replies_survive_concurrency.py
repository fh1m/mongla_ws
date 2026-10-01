"""Two threads asking for parameters at once both get their answers (issue #17).

pymavlink keeps ONE PARAM_VALUE slot. get_param / set_param / set_default_gain
each popped it and polled it, so the telemetry timer's set_default_gain could
erase an action thread's channel_role reply -- the loser timed out reading
"unreadable", and `fire()` turns an unreadable role into a refusal. The reader
now files every reply under its own name.
"""
import threading
import time
from types import SimpleNamespace

import mongla_control.fc.srot_fc as mod
from test_srot_fc import _fc


def _reply(name, value):
    return SimpleNamespace(param_id=name.encode(), param_value=float(value),
                           _timestamp=time.time())


def _board(fc, replies):
    """Answer each request through the READER path, back to back -- the burst
    that overwrote one slot."""
    def run():
        time.sleep(0.05)
        for name, value in replies:
            msg = _reply(name, value)
            fc.master.messages['PARAM_VALUE'] = msg     # pymavlink's one slot
            fc.note_param_value(msg)                     # the reader's demux
    threading.Thread(target=run, daemon=True).start()


def test_two_concurrent_reads_both_get_their_own_value():
    """Falsified by a None: that is the reply the other thread erased."""
    fc = _fc()
    fc.note_param_value(_reply('WARMUP', 0))            # the reader is running
    out = {}
    _board(fc, [('SERVO9_ROLE', 2.0), ('JS_GAIN_DEFAULT', 1.0)])
    t1 = threading.Thread(target=lambda: out.__setitem__(
        'role', fc.get_param('SERVO9_ROLE', timeout=1.0)))
    t2 = threading.Thread(target=lambda: out.__setitem__(
        'gain', fc.get_param('JS_GAIN_DEFAULT', timeout=1.0)))
    t1.start(); t2.start(); t1.join(); t2.join()
    assert out == {'role': 2.0, 'gain': 1.0}


def test_a_reply_older_than_the_request_is_not_the_answer():
    """A value filed BEFORE this request was sent is last time's, not this one's."""
    fc = _fc()
    fc.note_param_value(_reply('SERVO9_ROLE', 1.0))     # stale
    assert fc.get_param('SERVO9_ROLE', timeout=0.2) is None


def test_set_param_confirms_on_its_own_echo():
    fc = _fc()
    fc.note_param_value(_reply('WARMUP', 0))
    _board(fc, [('JS_GAIN_DEFAULT', 1.0)])
    assert fc.set_default_gain(1.0, timeout=1.0) is True


def test_without_a_reader_the_single_slot_path_still_works():
    fc = _fc()
    def run():
        time.sleep(0.05)
        fc.master.messages['PARAM_VALUE'] = _reply('SERVO9_ROLE', 2.0)
    threading.Thread(target=run, daemon=True).start()
    assert fc.get_param('SERVO9_ROLE', timeout=1.0) == 2.0


def test_the_reader_routes_every_param_value_to_the_table():
    import pathlib
    src = (pathlib.Path(__file__).resolve().parents[2] / 'mongla_manager'
           / 'mongla_manager' / 'auv_manager_node.py').read_text(encoding='utf-8')
    assert "'PARAM_VALUE': note_param" in src
