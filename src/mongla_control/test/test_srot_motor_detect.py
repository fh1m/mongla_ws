"""`MOTOR_DETECT` (mode 20) -- GATE 0, and the one routine that rewrites the
signs every controller downstream depends on.

Three things make it different from `motor_test`, and each has a test here:

  * IT PULSES ALL EIGHT THRUSTERS, unattended, and then writes. So the
    confirmation is not decoration -- a bool `force=True` default would put an
    accidental full-thruster run one keystroke from a typo.
  * THE ANSWER IT PRODUCES IS SPLIT ACROSS THREE PLACES. `CAL_MDIRn` multiplies
    with `MOT_n_DIRECTION`, and `FRAME_REVERSE` negates all six axes after the
    mix. No display anywhere shows the product, which is precisely how the
    2026-08-07 confusion happened (fw `mav_stream.cpp:457`).
  * ITS SUCCESS STATE LOOKS LIKE A FAULT. The board finishes MANUAL and
    DISARMED on purpose (`task_control_loop.cpp:383-406`): handing an armed
    hull to a closed-loop controller straight after an unverified sign change
    flipped the vehicle in water.
"""
import sys
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from mongla_control.fc import srot_protocol as sp     # noqa: E402
from mongla_control.fc.srot_fc import SrotFC          # noqa: E402

TOKEN = sp.MOTOR_DETECT_TOKEN


class _Mav:
    """Answers ON SEND. A fake that plants a reply before the request has it
    popped by the driver's own staleness guard and never answers."""

    def __init__(self, master):
        self.sent = []
        self._m = master

    def __getattr__(self, name):
        if name.startswith('_'):
            raise AttributeError(name)

        def _send(*a, **k):
            self.sent.append((name, a, k))
            m = self._m
            if name == 'param_request_read_send':
                pid = a[2].decode() if isinstance(a[2], bytes) else str(a[2])
                if pid in m.params:
                    m.messages['PARAM_VALUE'] = type('P', (), {
                        'param_id': pid,
                        'param_value': float(m.params[pid])})()
            elif name == 'command_long_send':
                m.messages['COMMAND_ACK'] = type(
                    'A', (), {'command': a[2], 'result': sp.ACK_ACCEPTED})()
                if a[2] == 176:                      # MAV_CMD_DO_SET_MODE
                    m.mode = int(a[5])               # p2 is index 5
        return _send


class _Hb:
    """The heartbeat the driver polls. Reading it is what advances the board.

    A fake that jumps straight to the end state ON SEND is the mirror of the
    plant-before-request bug: it skips the intermediate state the code is
    polling for, so `set_mode` never observes MOTOR_DETECT and reports "could
    not enter" for a run that did happen. Modelling time means modelling the
    states in order, not just the last one.
    """

    autopilot = 0

    def __init__(self, master):
        self._m = master

    @property
    def custom_mode(self):
        self._m.tick()
        return self._m.mode


class _Master:
    """Scripted board. Entering MOTOR_DETECT is what STARTS the routine, so the
    completion is a consequence of the mode change and of time passing, never
    something planted alongside it."""

    def __init__(self, params=None, mode=sp.MODE_STABILIZE, armed=True,
                 finish_after=None, new_cal=None):
        self.params = dict(params or {})
        self.mode = mode
        self.armed = armed
        self.finish_after = finish_after      # None = the routine never ends
        self.new_cal = new_cal or {}
        self.on_finish = None                 # the board's STATUSTEXT burst
        self.reads = 0
        self.mav = _Mav(self)
        self.messages = {'HEARTBEAT': _Hb(self)}

    def tick(self):
        self.reads += 1
        if (self.finish_after is not None
                and self.mode == sp.MODE_MOTOR_DETECT
                and self.reads >= self.finish_after):
            self.finish_detect()

    def finish_detect(self):
        """What the board does when the routine clears itself: MANUAL and
        DISARMED, deliberately (fw task_control_loop.cpp:383-406)."""
        self.mode = sp.MODE_MANUAL
        self.armed = False
        self.params.update(self.new_cal)
        if self.on_finish is not None:
            self.on_finish()


_HULL = dict(
    {f'CAL_MDIR{i}': 1.0 for i in range(1, 9)},
    **{f'MOT_{i}_DIRECTION': 1.0 for i in range(1, 9)})
_HULL['MOT_1_DIRECTION'] = -1.0
_HULL['MOT_8_DIRECTION'] = -1.0
_HULL['FRAME_REVERSE'] = 1.0


def _fc(master=None, **kw):
    m = master or _Master(params=dict(_HULL), **kw)
    fc = SrotFC(m, log=None)
    fc.is_armed = lambda: m.armed
    return fc


def _mode_sets(fc):
    return [a[5] for n, a, _ in fc.master.mav.sent
            if n == 'command_long_send' and a[2] == 176]


# --------------------------------------------------------------------------- #
#  The confirmation
# --------------------------------------------------------------------------- #
def test_no_token_means_no_thrusters_move():
    """The default path must be inert. Nothing is sent that could start it."""
    fc = _fc()
    # A short timeout so a BROKEN guard fails here instead of hanging for 60 s
    # -- the guard returns long before it is consulted.
    ok, why = fc.motor_detect(timeout=1.0)
    assert ok is False
    assert sp.MODE_MOTOR_DETECT not in _mode_sets(fc)


def test_the_refusal_is_the_briefing_so_the_token_cannot_be_reached_blind():
    """A confirmation you can satisfy without seeing what it changes is not a
    confirmation. The refusal text IS the live reading."""
    fc = _fc()
    _, why = fc.motor_detect(timeout=1.0)
    assert 'CAL_MDIR1..8' in why
    assert 'MOT_n_DIRECTION' in why
    assert 'FRAME_REVERSE' in why
    assert TOKEN in why


def test_the_briefing_reads_the_board_rather_than_assuming_defaults():
    """A params-reset boot silently restores defaults; assuming them would
    report a configuration this hull does not have."""
    m = _Master(params=dict(_HULL, **{'CAL_MDIR3': -1.0, 'FRAME_REVERSE': 0.0}))
    text = _fc(m).motor_detect_briefing()
    assert '+ + - + + + + +' in text
    # FRAME_REVERSE 0 must not carry the warning that only applies when it is 1
    assert 'negates ALL six axes' not in text


def test_frame_reverse_is_flagged_when_set_because_detect_cannot_fix_it():
    text = _fc().motor_detect_briefing()
    assert 'negates ALL six axes' in text


def test_the_effective_mix_is_shown_because_neither_stored_value_is_it():
    """CAL_MDIR x MOT_n_DIRECTION is what reaches the mixer, and it is the one
    number no display shows -- the 2026-08-07 confusion in one line."""
    text = _fc().motor_detect_briefing()
    eff = [l for l in text.splitlines() if 'effective mix' in l][0]
    assert eff.split()[-8:] == ['-', '+', '+', '+', '+', '+', '+', '-']


def test_frame_reverse_is_not_folded_into_the_per_motor_product():
    """It is an axis-level negation. Folding it in would suggest a per-motor
    fix for a whole-vehicle inversion."""
    text = _fc().motor_detect_briefing()
    eff = [l for l in text.splitlines() if 'effective mix' in l][0]
    assert eff.split()[-8:].count('-') == 2      # M1 and M8, not all eight


def test_a_wrong_token_does_not_run_it():
    fc = _fc()
    for bad in (True, 1, 'yes', TOKEN.lower(), TOKEN + ' '):
        ok, _ = fc.motor_detect(bad, timeout=1.0)
        assert ok is False, bad
    assert sp.MODE_MOTOR_DETECT not in _mode_sets(fc)


def test_an_unreadable_param_is_a_question_mark_not_a_plus():
    """Absence is not a value. Printing '+' for a param that did not answer
    would report a configuration nobody read."""
    m = _Master(params={})
    # short timeout: 16 params that never answer would otherwise be 32 s of
    # this suite's runtime spent proving nothing about the guard.
    text = _fc(m).motor_detect_briefing(timeout=0.05)
    assert '?' in text and '+' not in text.split('FRAME_REVERSE')[0]


# --------------------------------------------------------------------------- #
#  Preconditions
# --------------------------------------------------------------------------- #
def test_a_disarmed_board_is_refused_before_the_mode_change():
    """The board bounces MOTOR_DETECT back to STABILIZE when disarmed, so
    without this the caller watches a mode that silently reverted."""
    fc = _fc(armed=False)
    ok, why = fc.motor_detect(TOKEN)
    assert ok is False and 'ARMED' in why
    assert sp.MODE_MOTOR_DETECT not in _mode_sets(fc)


def test_water_is_stated_not_checked():
    """There is no wet sensor. A guard that implied one was verified would be
    worse than no guard -- an in-air run reports FAIL and writes nothing, but
    only on fw rev 6 and later."""
    text = _fc().motor_detect_briefing()
    assert 'WATER' in text and 'in air' in text


# --------------------------------------------------------------------------- #
#  Running it
# --------------------------------------------------------------------------- #
def test_the_token_enters_the_mode():
    m = _Master(params=dict(_HULL), finish_after=3)
    fc = _fc(m)
    ok, why = fc.motor_detect(TOKEN, timeout=5.0)
    assert ok is True, why
    assert sp.MODE_MOTOR_DETECT in _mode_sets(fc)


def test_disarmed_in_manual_is_reported_as_the_designed_end_state():
    """It looks exactly like a failure. Saying so is the difference between an
    operator re-arming to verify and an operator debugging a non-fault."""
    m = _Master(params=dict(_HULL), finish_after=3)
    ok, why = _fc(m).motor_detect(TOKEN, timeout=5.0)
    assert ok is True
    assert 'DISARMED' in why and 'designed' in why and 're-arm' in why.lower()


def test_a_changed_sign_is_named_by_motor():
    m = _Master(params=dict(_HULL), finish_after=3,
                new_cal={'CAL_MDIR2': -1.0, 'CAL_MDIR5': -1.0})
    ok, why = _fc(m).motor_detect(TOKEN, timeout=5.0)
    assert ok is True
    assert 'M2' in why and 'M5' in why
    assert 'M3' not in why


def test_no_change_is_reported_as_AMBIGUOUS_not_as_success():
    """Nothing changed means either idempotent success (already correct) or an
    INCONCLUSIVE run that correctly declined to write. Collapsing those two
    into 'detect passed' is how an in-air run gets believed."""
    m = _Master(params=dict(_HULL), finish_after=3)
    _, why = _fc(m).motor_detect(TOKEN, timeout=5.0)
    assert 'INCONCLUSIVE' in why and 'already correct' in why


def test_the_boards_own_lines_are_relayed_verbatim():
    """Ours is derived, theirs is the source. It prints DETECTED and EFFECTIVE
    on two lines because both together exceed the 50-char budget."""
    m = _Master(params=dict(_HULL), finish_after=3)
    fc = _fc(m)
    # DURING the run, not before it -- planted up front they are older than the
    # run's own mark and correctly filtered out, which is the next test.
    m.on_finish = lambda: [
        fc._statustext_log.append((time.time(), 6, t)) for t in
        ('MotorDetect: + + - + + - + +', 'MotorDetect eff: - + - + + - + -')]
    _, why = fc.motor_detect(TOKEN, timeout=5.0)
    assert 'MotorDetect: + + - + + - + +' in why
    assert 'MotorDetect eff: - + - + + - + -' in why


def test_older_lines_are_not_relayed_as_this_runs_result():
    """A stale MotorDetect line from a previous run would read as this run's
    answer -- the same one-slot hazard as PARAM_VALUE and STATUSTEXT."""
    m = _Master(params=dict(_HULL), finish_after=3)
    fc = _fc(m)
    fc._statustext_log.append((time.time() - 600, 6, 'MotorDetect: STALE'))
    _, why = fc.motor_detect(TOKEN, timeout=5.0)
    assert 'STALE' not in why


def test_a_run_that_never_finishes_fails_rather_than_reporting_success():
    m = _Master(params=dict(_HULL))       # on_mode does nothing: it hangs
    ok, why = _fc(m).motor_detect(TOKEN, timeout=0.4)
    assert ok is False and 'did not finish' in why


def test_abort_leaves_the_mode_AND_disarms():
    """Leaving the mode is what stops it driving motors, but a mode change
    alone leaves thrusters live."""
    m = _Master(params=dict(_HULL))
    fc = _fc(m)
    fc.disarm = lambda *a, **k: setattr(m, 'armed', False) or (True, 'disarmed')
    ok, why = fc.motor_detect(TOKEN, timeout=5.0, abort_fn=lambda: True)
    assert ok is False and 'aborted' in why
    assert sp.MODE_MANUAL in _mode_sets(fc)
    assert m.armed is False


def test_a_refused_mode_change_is_not_treated_as_a_run():
    fc = _fc()
    fc.set_mode = lambda *a, **k: (False, 'mode stayed STABILIZE')
    ok, why = fc.motor_detect(TOKEN, timeout=5.0)
    assert ok is False and 'could not enter' in why
