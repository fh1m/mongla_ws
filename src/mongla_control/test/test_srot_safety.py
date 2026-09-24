"""Safety paths that had ZERO assertions before this file.

Each of these is a mechanism whose failure is silent by construction, which is
why none of them was noticed as untested:

  * the GCS heartbeat -- if it stops, the board surfaces the vehicle 5 s later
    and nothing on the host reports anything at all
  * `set_servo` bounds -- an out-of-range pulse is clamped by the far end, so
    the host cannot tell a sane command from a nonsensical one
  * the params-saved match string -- matching the wrong substring reproduces a
    bug that says "saved" while nothing was written

The rule they share: a check that cannot fail is worse than no check, because
the operator reads a line that looks like verification.
"""
import sys
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from mongla_control.fc import srot_protocol as sp     # noqa: E402
from mongla_control.fc.srot_fc import SrotFC          # noqa: E402


class _Mav:
    """Records every message the driver emits, with a timestamp."""

    def __init__(self):
        self.sent = []

    def __getattr__(self, name):
        if name.startswith('_'):
            raise AttributeError(name)

        def _send(*a, **k):
            self.sent.append((time.monotonic(), name, a, k))
        return _send


class _Master:
    def __init__(self):
        self.mav = _Mav()
        self.messages = {}


def _fc():
    return SrotFC(_Master(), log=None)


def _names(fc):
    return [n for _, n, _, _ in fc.master.mav.sent]


# --------------------------------------------------------------------------- #
#  The GCS heartbeat vs GCS_FAILSAFE_MS
# --------------------------------------------------------------------------- #
def test_the_failsafe_window_is_what_we_think_it_is():
    """Pinned because every cadence below is justified against it, and it is a
    firmware constant that can move under us."""
    assert sp.GCS_FAILSAFE_MS == 5000


def test_the_managers_cadence_has_real_margin_on_the_failsafe():
    """`auv_manager_node._HeartbeatThread` beats at 0.5 s (2 Hz) against a 5 s
    failsafe. That is 10x, and the margin is the point: the board SURFACES the
    vehicle mid-mission on 5 s of silence, so the cadence must survive a
    scheduler hiccup, a param-download blackout, and a slow ACK -- not merely
    beat the deadline on a good day.
    """
    tick_s = 0.5                      # auv_manager_node.HEARTBEAT_PERIOD_S
    window_s = sp.GCS_FAILSAFE_MS / 1000.0
    missed = window_s / tick_s
    assert missed >= 5, (
        f'only {missed:.0f} ticks fit in the failsafe window; a single stall '
        f'would surface the vehicle')


def test_send_gcs_heartbeat_actually_emits_one():
    """It had no test at all. A no-op here surfaces the vehicle 5 s later and
    reports nothing -- there is no ACK and no error path to notice."""
    fc = _fc()
    fc.send_gcs_heartbeat()
    assert 'heartbeat_send' in _names(fc)


def test_it_identifies_as_a_companion_not_a_gcs():
    """We are compid 191 (ONBOARD_COMPUTER). Claiming MAV_TYPE_GCS on the same
    frame contradicts that, which is the ambiguity the compid change removed."""
    import pymavlink.mavutil as mavutil
    fc = _fc()
    fc.send_gcs_heartbeat()
    _, _, args, _ = fc.master.mav.sent[-1]
    assert args[0] == mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER
    # AUTOPILOT_INVALID is load-bearing: our own `_vehicle_hb` filter keys on it
    # to ignore our loopback frames. Change it and the driver starts reading its
    # own heartbeats as the vehicle's.
    assert args[1] == mavutil.mavlink.MAV_AUTOPILOT_INVALID


def test_repeated_calls_each_emit():
    """A cadence is only a cadence if every tick produces a frame. A driver that
    coalesced or rate-limited internally would silently halve it."""
    fc = _fc()
    for _ in range(5):
        fc.send_gcs_heartbeat()
    assert _names(fc).count('heartbeat_send') == 5


# --------------------------------------------------------------------------- #
#  set_servo bounds
# --------------------------------------------------------------------------- #
def test_the_servo_pulse_range_is_pinned():
    assert sp.SERVO_MIN_US == 1000 and sp.SERVO_MAX_US == 2000


def test_an_out_of_range_pulse_is_clamped_before_it_is_sent():
    """`SERVO_MIN_US`/`SERVO_MAX_US` were defined and unused, so the host sent
    whatever it was given and relied on the far end to cope. That is the same
    shape as relying on the board to clamp `gain`: a limit enforced only at the
    far end is not a limit the host can reason about."""
    fc = _fc()
    fc.set_servo(3, 9999)
    fc.set_servo(3, -500)
    pulses = [a[5] for _, n, a, _ in fc.master.mav.sent if n == "command_long_send"]
    assert pulses, 'no servo command was sent'
    for p in pulses:
        assert sp.SERVO_MIN_US <= p <= sp.SERVO_MAX_US, f'{p} escaped the clamp'


def test_an_in_range_pulse_is_untouched():
    fc = _fc()
    fc.set_servo(3, 1500)
    pulses = [a[5] for _, n, a, _ in fc.master.mav.sent if n == "command_long_send"]
    assert pulses[-1] == 1500


# --------------------------------------------------------------------------- #
#  The params-saved match string
# --------------------------------------------------------------------------- #
def test_the_saved_confirmation_matches_the_right_line():
    """The record states TWICE that matching `"saved"` instead of
    `"Params saved"` reproduced the bug: the board emits its CALIBRATION save
    line first, so the looser match confirms a write that has not happened.

    Pinned as a property of the strings, not of any one implementation, so it
    holds for whichever consumer does the matching.
    """
    calibration_line = 'Calibration saved + verified on flash'
    params_line = 'Params saved to flash'
    fail_line = 'SAVE FAILED - params NOT written (NVS full?)'

    assert 'saved' in calibration_line, (
        'the loose match would fire on the calibration line -- which is exactly '
        'the bug, and if this ever stops being true the hazard is gone')
    assert 'Params saved' not in calibration_line
    assert 'Params saved' in params_line
    assert 'Params saved' not in fail_line


# --------------------------------------------------------------------------- #
#  The parameter interlock: DEPTH_P, GAIN, and the one-slot cache
# --------------------------------------------------------------------------- #
class _ParamMaster(_Master):
    """A master that answers a param request the way the link does.

    Two properties are modelled because both matter:

    * the reply arrives AFTER the request. The driver pops the PARAM_VALUE slot
      before it sends, so a fake that plants beforehand tests nothing -- the
      driver erases it, which is the stale-value guard doing its job.
    * pymavlink keeps ONE message per msgid, so PARAM_VALUE has exactly the
      demux hazard NAMED_VALUE_FLOAT has: whatever param answered last sits in
      the slot, and a reader that does not check the name reads it as its own
      answer.
    """

    def __init__(self, reply=None):
        super().__init__()
        # name -> value the board answers with, whatever was asked for
        self.reply = dict(reply or {})

    def plant(self, name, value):
        self.messages['PARAM_VALUE'] = type(
            'PV', (), {'param_id': name.encode(), 'param_value': float(value)})()

    def answer(self):
        for name, value in self.reply.items():
            self.plant(name, value)


class _AnsweringMav(_Mav):
    """Plants the scripted reply when the driver sends a param request."""

    def __init__(self, master):
        super().__init__()
        self.master = master

    def __getattr__(self, name):
        send = super().__getattr__(name)

        def _wrapped(*a, **k):
            send(*a, **k)
            if name in ('param_request_read_send', 'param_set_send'):
                self.master.answer()
        return _wrapped


def _pfc(**reply):
    m = _ParamMaster(reply)
    m.mav = _AnsweringMav(m)
    return SrotFC(m, log=None)


def test_a_read_never_answers_with_a_different_parameter():
    """The one-slot hazard. If DEPTH_P is asked for while GAIN's reply is in the
    slot, an unchecked read returns 0.5 for a param that is 3.0 -- and 0.5 is a
    plausible DEPTH_P, so nothing about the number looks wrong. This is the same
    shape as the round-26 NAMED_VALUE_FLOAT harness error."""
    fc = _pfc(JS_GAIN_DEFAULT=0.5)
    assert fc.get_param('DEPTH_P', timeout=0.12) is None


def test_a_stale_reply_from_before_the_request_is_not_the_answer():
    """`get_param` pops the slot before it sends. Without that, a value cached
    from an EARLIER read of the same name returns instantly -- so a param that
    has since changed reads as its old value, with no delay to hint at it."""
    fc = SrotFC(_ParamMaster(), log=None)    # a link that never answers
    fc.master.plant('DEPTH_P', 3.0)          # the stale, pre-request value
    assert fc.get_param('DEPTH_P', timeout=0.12) is None


def test_a_read_that_times_out_is_None_and_not_zero():
    """Absence is not zero -- the rule that already caught BARO_HEALTH. A 0.0
    DEPTH_P would divide the arming guard's depth estimate by zero, or with a
    guard, read as an infinitely stiff loop."""
    got = _pfc().get_param('DEPTH_P', timeout=0.12)
    assert got is None and got is not False


def test_the_read_is_matched_by_name_and_returns_it():
    fc = _pfc(DEPTH_P=0.5)
    assert fc.get_param('DEPTH_P', timeout=0.12) == pytest.approx(0.5)


def test_a_failed_depth_p_read_leaves_it_absent_rather_than_guessed():
    """`read_depth_p` is best-effort, and honest about it: on a failed read
    `depth_p` stays None so the guard KNOWS it is using a default and can name
    it. Filling in a guess here would erase that distinction one layer too
    early."""
    fc = _pfc()                               # a link that never answers
    assert fc.read_depth_p(timeout=0.12) is None
    assert fc.depth_p is None


def test_a_successful_read_is_cached_for_the_guard():
    fc = _pfc(DEPTH_P=0.5)
    assert fc.read_depth_p(timeout=0.12) == pytest.approx(0.5)
    assert fc.depth_p == pytest.approx(0.5)


def _with_depth_cmd(fc, cmd):
    fc.note_named_value(type('N', (), {'name': b'DEPTH_CMD', 'value': cmd})())
    return fc


def test_the_gain_is_what_decides_the_verdict_not_a_margin():
    """The round-26 defect, as arithmetic rather than as a constant.

    The guard recovers the board's depth as `DEPTH_CMD / DEPTH_P + 0.10`, so
    DEPTH_P is not a refinement of the answer -- it IS the answer's scale. With
    the board running 0.5 and our fallback stuck at 3.0, every reading was
    divided by 6x too much, which put a real 0.90 m surface error (three times
    the limit) at an apparent 0.20 m.

    Asserted as the bracket rather than as "the stale value arms". A future
    sanity bound on `depth_p` would be an IMPROVEMENT, and an assertion that
    the old behaviour survives cannot tell an improvement from a regression --
    it would fail on the fix. The bracket says why 6x mattered without pinning
    the bug in place.
    """
    cmd = 0.40
    true_depth = cmd / 0.5 + sp.DEPTH_PREVIEW_TARGET_M      # what the board sees
    apparent = cmd / 3.0 + sp.DEPTH_PREVIEW_TARGET_M        # what a stale gain shows
    assert true_depth > sp.DEPTH_ERR_ARM_LIMIT_M, 'this reading must be refused'
    assert apparent < sp.DEPTH_ERR_ARM_LIMIT_M, (
        'and the stale gain must make the SAME reading look acceptable -- '
        'otherwise this no longer demonstrates the failure mode')


def test_the_guard_refuses_that_reading_on_the_boards_own_gain():
    """The half that must hold whatever else changes: with DEPTH_P read from
    the board, a barometer 0.90 m off at the surface does not arm."""
    fc = _with_depth_cmd(_pfc(), 0.40)
    fc.depth_p = 0.5
    ok, why = fc.check_depth_loop_settled()
    assert ok is False, f'a 0.9 m surface reading must refuse: {why}'
    assert 'IMPLAUSIBLE' in why, 'and must say what it saw, not just refuse'


def test_the_default_used_when_the_read_fails_is_the_boards_own_value():
    """No read -> the guard falls back. Pinned against hardware: the live board
    answered DEPTH_P = 0.5, so the fallback and the board now agree and the
    fallback path is no longer a silent 6x."""
    assert sp.DEPTH_P_DEFAULT == 0.5
    fc = _with_depth_cmd(_pfc(), 0.40)        # depth_p never read
    assert fc.depth_p is None
    assert fc.check_depth_loop_settled()[0] is False


def test_a_write_is_only_confirmed_by_an_echo_of_the_value_written():
    """A board that clamps or rejects echoes a DIFFERENT value. Confirming on the
    name alone reports success for a write that did not take -- and for GAIN that
    silently halves every axis, which looks like a weak vehicle, not a bug."""
    fc = _pfc(JS_GAIN_DEFAULT=0.5)                   # board clamped our 1.0
    assert fc.set_param('JS_GAIN_DEFAULT', 1.0, timeout=0.12) is False
    assert fc.set_default_gain(1.0, timeout=0.12) is False


def test_a_write_confirmed_by_a_matching_echo_succeeds():
    fc = _pfc(JS_GAIN_DEFAULT=1.0)
    assert fc.set_default_gain(1.0, timeout=0.12) is True


def test_full_authority_is_what_autonomy_asks_for():
    """GAIN boots at 0.5 and MANUAL_CONTROL is scaled by it, so every vision
    command runs at half thrust until this is set. Pinned because a wrong value
    here is invisible: the vehicle moves, just not as far as commanded."""
    assert sp.GAIN_FOR_AUTONOMY == 1.0


# --------------------------------------------------------------------------- #
#  Two opposite instructions, one wire code
# --------------------------------------------------------------------------- #
def _reason(result, statustext=None, verb='move_forward'):
    fc = _fc()
    if statustext is not None:
        fc.master.messages['STATUSTEXT'] = type(
            'ST', (), {'text': statustext.encode()})()
    from mongla_control.fc.srot_fc import DENIED
    return fc._terminal_reason(verb, DENIED, result)


def test_the_two_meanings_of_one_ack_code_are_told_apart():
    """fw rev 13 gave ACK_TEMPORARILY_REJECTED a SECOND meaning: SROT_MOVE now
    requires ARMED and refuses with the same result a state-lock miss uses. The
    two need OPPOSITE actions -- wait and retry, versus arm first -- so a reason
    string that covers both is worse than no advice."""
    busy = _reason(sp.ACK_TEMPORARILY_REJECTED, 'state lock held')
    disarmed = _reason(sp.ACK_TEMPORARILY_REJECTED,
                       'SROT_MOVE refused: arm first')
    assert 'safe to retry' in busy
    assert 'DISARMED' in disarmed and 'Arm' in disarmed
    assert busy != disarmed


def test_unsupported_says_the_opposite_of_retry():
    """The third fork, and the one a retry loop gets wrong forever: this
    firmware does not implement the verb and will not until it is flashed."""
    out = _reason(sp.ACK_UNSUPPORTED)
    assert 'NOT SUPPORTED' in out and 'do not retry' in out


def test_retryable_and_unsupported_are_distinct_on_the_wire():
    """They must not share a value -- the reason strings are opposite advice."""
    assert sp.ACK_TEMPORARILY_REJECTED != sp.ACK_UNSUPPORTED
    assert {sp.ACK_TEMPORARILY_REJECTED, sp.ACK_UNSUPPORTED} <= sp.TERMINAL_ACKS


def test_a_previous_commands_statustext_is_not_read_as_this_ones():
    """STATUSTEXT is a single pymavlink slot, and here it is load-bearing: it is
    the ONLY thing separating the two meanings above. An 'arm first' left from
    an earlier disarmed attempt would relabel the next genuine busy rejection as
    'the board is DISARMED' -- on an armed board, telling the operator to fix
    something that is not wrong while the real state lock persists."""
    fc = _fc()
    fc.master.messages['STATUSTEXT'] = type(
        'ST', (), {'text': b'SROT_MOVE refused: arm first'})()
    fc.master.messages['COMMAND_ACK'] = type('A', (), {'result': 0})()
    fc._clear_ack()
    assert 'STATUSTEXT' not in fc.master.messages
    assert 'COMMAND_ACK' not in fc.master.messages


def test_a_reason_with_no_statustext_still_reads_sensibly():
    """The board does not always send text. The advice must survive its absence
    rather than degrade into a dangling parenthesis."""
    out = _reason(sp.ACK_TEMPORARILY_REJECTED)
    assert 'safe to retry' in out and out.endswith('retry')
