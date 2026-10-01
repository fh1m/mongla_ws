"""A STOP's in-flight ACKs are consumed by the move that caused them (issue #10).

The firmware answers a displaced sequence -- CANCELLED, or ACCEPTED if it had
just completed -- on its stream task's NEXT tick, and then the STOP's own
terminal ACK. Both arrive after the following command's send, and the move ACK
carries no sequence id, so the next move read them as its own: `yaw_right`
reported SUCCEEDED in ~20 ms off the STOP's ACCEPTED while the board had only
just begun the turn. Now the aborted move waits for them.

The scene is the issue's own failure scenario, replayed against the clock.
"""
import threading
import time

import mongla_control.fc.srot_fc as srot_mod
import mongla_control.fc.srot_protocol as sp
from test_srot_fc import _ack, _fc


def _late_acks_after_stop(fc, delay=0.1):
    """The board's answer to the STOP: CANCELLED for the displaced sequence,
    then ACCEPTED for the STOP, each a stream tick apart."""
    def run():
        time.sleep(delay)
        fc.master.messages['COMMAND_ACK'] = _ack(sp.ACK_CANCELLED)
        time.sleep(0.08)
        fc.master.messages['COMMAND_ACK'] = _ack(sp.ACK_ACCEPTED)
    threading.Thread(target=run, daemon=True).start()


def test_an_aborted_move_consumes_the_acks_its_stop_set_in_flight():
    fc = _fc()
    stop_sent = threading.Event()
    real_stop = fc.stop_motion

    def stop_and_answer():
        real_stop()
        _late_acks_after_stop(fc)
        stop_sent.set()
    fc.stop_motion = stop_and_answer
    abort = threading.Event()
    threading.Timer(0.2, abort.set).start()
    res = fc.move('move_forward', duration=5.0, gain=50, abort_fn=abort.is_set)
    assert res.code == srot_mod.ABORTED
    assert stop_sent.is_set()
    # Wait PAST the late ACKs (0.1 s and 0.18 s after the STOP): checking at
    # once would pass before they had even arrived, drain or no drain.
    time.sleep(0.4)
    left = fc.master.messages.get('COMMAND_ACK')
    assert left is None, f'a stale {left.result} was left for the next move'


def test_the_NEXT_move_is_not_fooled_by_a_stop_s_accepted():
    """Falsified by SUCCEEDED in under the turn's own time -- the issue's bug."""
    fc = _fc()
    real_stop = fc.stop_motion

    def stop_and_answer():
        real_stop()
        _late_acks_after_stop(fc)
    fc.stop_motion = stop_and_answer
    abort = threading.Event()
    threading.Timer(0.2, abort.set).start()
    fc.move('move_forward', duration=5.0, gain=50, abort_fn=abort.is_set)

    # The genuine turn: progress, then done after ~0.6 s.
    def real_turn():
        for p in (20, 60):
            time.sleep(0.25)
            fc.master.messages['COMMAND_ACK'] = _ack(sp.ACK_IN_PROGRESS, progress=p)
        time.sleep(0.1)
        fc.master.messages['COMMAND_ACK'] = _ack(sp.ACK_ACCEPTED)
    threading.Thread(target=real_turn, daemon=True).start()
    t0 = time.monotonic()
    res = fc.move('yaw_right', target=90.0)
    took = time.monotonic() - t0
    assert res.code == srot_mod.SUCCEEDED
    assert took > 0.5, f'SUCCEEDED in {took * 1000:.0f} ms -- that was the STOP\'s ACK'


def test_a_board_that_never_answers_the_stop_does_not_hang_the_abort():
    fc = _fc()
    abort = threading.Event()
    threading.Timer(0.1, abort.set).start()
    t0 = time.monotonic()
    res = fc.move('move_forward', duration=5.0, gain=50, abort_fn=abort.is_set)
    assert res.code == srot_mod.ABORTED
    assert time.monotonic() - t0 < 0.1 + srot_mod.MOVE_ACK_DRAIN_S + 0.5
