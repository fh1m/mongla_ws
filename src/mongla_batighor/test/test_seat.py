"""The pilot token keeps the owner's rules: one pilot, safety for everyone, a lost pilot loses it.

Each test is one rule from bead mongla_ws-102.25.3, driven by a clock the test
owns, so a timeout is a number and not a sleep.
"""
from mongla_batighor.seat import ANSWER_S, LOST_S, Seat


def test_the_first_to_ask_takes_a_free_helm_and_others_only_watch():
    s = Seat()
    assert s.request('ana', 0.0).ok and s.pilot == 'ana'
    assert not s.may_send('raf', 'move_forward').ok
    assert s.may_send('ana', 'move_forward').ok
    assert not s.request('', 0.0).ok                      # a name is required


def test_disarm_stop_and_surface_are_open_to_everyone_even_with_no_pilot():
    for seat in (Seat(), Seat(pilot='ana')):
        for cmd in ('disarm', 'stop', 'surface'):
            assert seat.may_send('raf', cmd).ok, (seat.pilot, cmd)
            assert seat.may_send('', cmd).ok


def test_arming_needs_both_confirmations_from_the_pilot():
    s = Seat()
    s.request('ana', 0.0)
    assert not s.may_send('ana', 'arm').ok
    assert not s.may_send('ana', 'arm', ['propellers_clear']).ok
    assert s.may_send('ana', 'arm', ['propellers_clear', 'kill_switch_manned']).ok
    assert not s.may_send('raf', 'arm', ['propellers_clear', 'kill_switch_manned']).ok


def test_a_silent_pilot_loses_the_helm_after_the_timeout_and_not_before():
    s = Seat()
    s.request('ana', 0.0)
    s.heartbeat('ana', 1.0)
    assert s.tick(1.0 + LOST_S) is False and s.pilot == 'ana'
    assert s.tick(1.0 + LOST_S + 0.01) is True and s.pilot == ''
    assert s.tick(10.0) is False                          # lost once, reported once


def test_only_the_pilots_own_beat_keeps_the_helm():
    s = Seat()
    s.request('ana', 0.0)
    for t in range(1, 10):
        s.heartbeat('raf', float(t))                       # a watcher's beat
    assert s.tick(9.0) is True and s.pilot == ''


def test_a_request_passes_when_the_pilot_gives_it_or_says_nothing():
    s = Seat()
    s.request('ana', 0.0)
    assert s.request('raf', 1.0).ok and s.pilot == 'ana'
    assert not s.request('lee', 2.0).ok                   # one asker at a time
    assert s.give('ana', 3.0).ok and s.pilot == 'raf' and not s.requester
    s.request('ana', 4.0)
    s.heartbeat('raf', 4.0 + ANSWER_S - 1)
    s.tick(4.0 + ANSWER_S - 0.5)
    assert s.pilot == 'raf'
    s.tick(4.0 + ANSWER_S)
    assert s.pilot == 'ana'


def test_only_the_pilot_gives_or_releases():
    s = Seat()
    s.request('ana', 0.0)
    s.request('raf', 1.0)
    assert not s.give('raf', 2.0).ok and not s.release('raf', 2.0).ok
    assert s.release('ana', 2.0).ok and s.pilot == 'raf'
    assert s.release('raf', 3.0).ok and s.pilot == ''


def test_a_lost_pilot_hands_over_to_whoever_was_waiting():
    s = Seat()
    s.request('ana', 0.0)
    s.request('raf', 0.5)
    assert s.tick(LOST_S + 0.1) is True and s.pilot == 'raf'


def test_every_change_of_pilot_is_written_down():
    s = Seat()
    s.request('ana', 0.0)
    s.request('raf', 1.0)
    s.give('ana', 2.0)
    s.tick(2.0 + LOST_S + 1)
    text = '\n'.join(s.events)
    for name in ('ana is the pilot', 'raf asked ana', 'raf is the pilot (given by ana)',
                 'raf went silent'):
        assert name in text, (name, text)
