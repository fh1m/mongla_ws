"""`TriggerPairer` against streams whose truth is chosen.

The bar, written before the module ran: on a synthetic camera with frame loss
and trigger-message loss, it never names the wrong pulse, it stamps at least
99 % of the frames whose pulse can be known, and a filled-in pulse is timed
to within 100 microseconds. And in each case built to be unanswerable, it
refuses and says which case it was.
"""
import random

import pytest

from mongla_vision.trigger_stamps import TriggerPairer


def _run(hz, *, lo, hi, latency=0.012, latency_jitter=0.0015, n=3000,
         frame_loss=0.05, msg_loss=0.10, report_every=1, trig_jitter=20e-6,
         seed=1):
    """Feed one synthetic run. Returns (right, wrong, refused, worst_fill_s)."""
    rng = random.Random(seed)
    period = 1.0 / hz
    pairer = TriggerPairer(lo, hi)
    events = []
    truth = {}
    for seq in range(n):
        t = 100.0 + seq * period + rng.uniform(-trig_jitter, trig_jitter)
        truth[seq] = t
        if seq % report_every == 0 and (seq < 2 or rng.random() >= msg_loss):
            # The message itself arrives a little after the pulse.
            events.append((t + 0.001, 'trig', seq, t))
        if rng.random() >= frame_loss:
            arrive = t + latency + rng.uniform(-latency_jitter, latency_jitter)
            events.append((arrive, 'frame', seq, arrive))
    right = wrong = refused = 0
    worst_fill = 0.0
    for _, kind, seq, t in sorted(events):
        if kind == 'trig':
            pairer.note_trigger(seq, t)
            continue
        got = pairer.stamp(t)
        if got is None:
            refused += 1
        elif got.seq == seq:
            right += 1
            if got.interpolated:
                worst_fill = max(worst_fill, abs(got.t_s - truth[seq]))
        else:
            wrong += 1
    return right, wrong, refused, worst_fill, pairer


@pytest.mark.parametrize('hz, lo, hi', [(50.0, 0.008, 0.016),
                                        (60.0, 0.008, 0.016),
                                        (210.0, 0.0100, 0.0140)])
def test_frames_get_their_own_pulse_through_loss_on_both_sides(hz, lo, hi):
    right, wrong, refused, worst_fill, _ = _run(hz, lo=lo, hi=hi)
    assert wrong == 0
    assert right / (right + refused) >= 0.99, (right, refused)
    assert worst_fill <= 100e-6, worst_fill


def test_a_board_that_reports_every_fourth_pulse_is_enough():
    """The upstream ask offers this to save link budget: the timer is
    crystal-timed, so the pulses between reports are known."""
    right, wrong, refused, worst_fill, _ = _run(
        60.0, lo=0.008, hi=0.016, report_every=4, msg_loss=0.0)
    assert wrong == 0 and right / (right + refused) >= 0.99
    assert worst_fill <= 100e-6, worst_fill


def test_a_latency_window_as_wide_as_the_period_refuses_every_frame():
    """Times alone cannot tell pulse s from s-1 then. Picking one would be
    right most of the time and wrong by a whole frame the rest."""
    right, wrong, refused, _, pairer = _run(210.0, lo=0.008, hi=0.016)
    assert right == 0 and wrong == 0 and refused > 0
    assert pairer.refused['window_not_narrower_than_period'] == refused


def test_a_latency_outside_its_stated_bounds_is_refused_not_mispaired():
    """The bounds are a measurement. If the camera is slower than they say,
    the answer is no stamp, never the neighbouring pulse."""
    right, wrong, refused, _, pairer = _run(
        50.0, lo=0.002, hi=0.006, latency=0.012, latency_jitter=0.0005)
    assert wrong == 0 and right == 0
    # The first frame can arrive before two pulses were heard.
    assert refused > 0
    assert pairer.refused['none_in_window'] >= refused - 2


def test_a_silent_link_stops_the_stamps():
    pairer = TriggerPairer(0.008, 0.016, max_gap_s=0.5)
    for seq in range(100):
        pairer.note_trigger(seq, 10.0 + seq * 0.02)
    last = 10.0 + 99 * 0.02
    assert pairer.stamp(last + 0.2 + 0.012) is not None      # within the gap
    assert pairer.stamp(last + 2.0 + 0.012) is None
    assert pairer.refused['link_silent'] == 1


def test_no_stamp_before_two_pulses_are_heard():
    pairer = TriggerPairer(0.008, 0.016)
    assert pairer.stamp(1.0) is None
    pairer.note_trigger(0, 1.0)
    assert pairer.stamp(1.012) is None
    assert pairer.refused['no_triggers'] == 2


def test_a_board_reboot_does_not_pair_across_the_old_numbering():
    pairer = TriggerPairer(0.008, 0.016)
    for seq in range(500, 520):
        pairer.note_trigger(seq, 10.0 + (seq - 500) * 0.02)
    pairer.note_trigger(0, 30.0)                 # the count went back
    assert pairer.resets == 1
    assert pairer.stamp(30.012) is None          # one pulse: no period yet
    pairer.note_trigger(1, 30.02)
    got = pairer.stamp(30.02 + 0.012)
    assert got is not None and got.seq == 1 and not got.interpolated


def test_a_pulse_before_the_first_one_heard_is_never_invented():
    pairer = TriggerPairer(0.008, 0.016)
    pairer.note_trigger(10, 5.0)
    pairer.note_trigger(11, 5.02)
    assert pairer.stamp(4.98 + 0.012) is None    # that would be pulse 9


@pytest.mark.parametrize('lo, hi', [(0.01, 0.01), (0.02, 0.01), (-0.001, 0.01)])
def test_bounds_that_are_not_a_window_are_refused_at_construction(lo, hi):
    with pytest.raises(ValueError):
        TriggerPairer(lo, hi)
