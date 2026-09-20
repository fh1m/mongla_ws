"""`AUTOTUNE` (21) and `MOTOR_TUNE` (22) -- the two in-water tuners.

They look like `MOTOR_DETECT` and differ from it in four ways that each produce
a WRONG ANSWER rather than an error, so each has a test here:

  * THEY FINISH IN STABILIZE, NOT MANUAL (fw task_control_loop.cpp:830-833 and
    :846-853). Code modelled on detect watches for MANUAL, waits out the whole
    timeout on a run that SUCCEEDED, and reports failure.
  * MOTOR_TUNE IS GATED ON A PARAM THAT DEFAULTS TO ZERO (`MTUNE_EN`,
    fw config.h:588). SET_MODE alone is accepted and the tuner never starts --
    the silent-no-op shape that already cost us a round in SURFACE.
  * AUTOTUNE HAS THREE TRIGGERS AND TWO OF THEM LATCH. `ATUNE` and
    `MAV_CMD_USER_5` set `autotune_active` with no reference to arming; the
    firmware's own comment records a disarmed `ATUNE` starting a full-authority
    tune on the NEXT arm. We enter by mode only.
  * MOTOR_TUNE FITS ON PER-MOTOR RPM, which on this vehicle has never been
    non-zero. It must warn on UNKNOWN and refuse on KNOWN-BAD -- never the
    reverse, or the method is unreachable forever.
"""
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from mongla_control.fc import srot_protocol as sp     # noqa: E402
from mongla_control.fc.srot_fc import SrotFC          # noqa: E402


class _Mav:
    """Answers ON SEND -- a reply planted before the request is popped by the
    driver's own staleness guard and never seen."""

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
                        'param_id': pid, 'param_value': float(m.params[pid])})()
            elif name == 'command_long_send':
                m.messages['COMMAND_ACK'] = type(
                    'A', (), {'command': a[2], 'result': sp.ACK_ACCEPTED})()
                if a[2] == 176:                       # MAV_CMD_DO_SET_MODE
                    m.mode = int(a[5])
        return _send


class _Hb:
    autopilot = 0

    def __init__(self, master):
        self._m = master

    @property
    def custom_mode(self):
        self._m.tick()
        return self._m.mode


class _Master:
    """Entering the mode is what STARTS the tuner, so completion is a
    consequence of the mode change plus time -- never planted alongside it."""

    def __init__(self, params=None, mode=sp.MODE_STABILIZE, armed=True,
                 finish_after=None, writes=None, run_mode=None):
        self.params = dict(params or {})
        self.mode = mode
        self.armed = armed
        self.finish_after = finish_after
        self.writes = writes or {}
        self.run_mode = run_mode
        self.reads = 0
        self.mav = _Mav(self)
        self.messages = {'HEARTBEAT': _Hb(self)}

    def tick(self):
        self.reads += 1
        if (self.finish_after is not None and self.mode == self.run_mode
                and self.reads >= self.finish_after):
            # BOTH tuners end DISARMED in STABILIZE. This is the designed end
            # state, and it is the one detect does NOT use.
            self.mode = sp.MODE_STABILIZE
            self.armed = False
            self.params.update(self.writes)


_PIDS = {n: 1.0 for n in sp.AUTOTUNE_PID_PARAMS}
_MT = dict({n: 1.0 for n in sp.MOTOR_TUNE_PARAMS}, MTUNE_EN=1.0)


def _fc(m):
    fc = SrotFC(m, log=None)
    fc.is_armed = lambda: m.armed
    fc.statustext_log = lambda since=0: []
    return fc


def _mode_sets(fc):
    return [int(a[5]) for n, a, _ in fc.master.mav.sent
            if n == 'command_long_send' and a[2] == 176]


def _param_sets(fc):
    return [n for n, a, _ in fc.master.mav.sent if n == 'param_set_send']


# --------------------------------------------------------------------------- #
#  The confirmation -- the default path must be inert
# --------------------------------------------------------------------------- #
def test_no_token_starts_NOTHING():
    for meth, mode in (('autotune', sp.MODE_AUTOTUNE),
                       ('motor_tune', sp.MODE_MOTOR_TUNE)):
        fc = _fc(_Master(params=dict(_PIDS, **_MT)))
        ok, why = getattr(fc, meth)()
        assert ok is False
        assert mode not in _mode_sets(fc), f'{meth} entered its mode with no token'
        assert 'confirmation required' in why


def test_the_briefings_quote_LIVE_values_not_defaults():
    fc = _fc(_Master(params=dict(_PIDS, **dict(_MT, MTUNE_EN=0.0))))
    assert 'RATE_RLL_P' in fc.autotune_briefing()
    mt = fc.motor_tune_briefing()
    assert 'DISABLED' in mt, 'the briefing must show MTUNE_EN is off'


# --------------------------------------------------------------------------- #
#  AUTOTUNE
# --------------------------------------------------------------------------- #
def test_autotune_refuses_DISARMED():
    """The firmware accepts the mode and never starts the tuner. Silent no-op."""
    fc = _fc(_Master(params=dict(_PIDS), armed=False))
    ok, why = fc.autotune(sp.AUTOTUNE_TOKEN, timeout=5.0)
    assert ok is False and 'ARMED' in why
    assert sp.MODE_AUTOTUNE not in _mode_sets(fc)


def test_autotune_NEVER_writes_the_ATUNE_param():
    """⛔ The latch. `ATUNE` and MAV_CMD_USER_5 set autotune_active with no
    reference to arming, and the firmware records a disarmed ATUNE starting a
    full-authority tune on the NEXT arm. We enter by mode only."""
    m = _Master(params=dict(_PIDS), finish_after=3,
                run_mode=sp.MODE_AUTOTUNE, writes={'RATE_RLL_P': 2.0})
    fc = _fc(m)
    ok, _ = fc.autotune(sp.AUTOTUNE_TOKEN, timeout=5.0)
    assert ok is True
    assert _param_sets(fc) == [], 'autotune must not PARAM_SET anything'
    sent = [a for n, a, _ in fc.master.mav.sent if n == 'command_long_send']
    assert all(int(a[2]) != 180 for a in sent), 'must not use MAV_CMD_USER_5'


def test_autotune_completes_on_STABILIZE_not_MANUAL():
    """⛔ The end state that would break a detect-shaped implementation."""
    m = _Master(params=dict(_PIDS), finish_after=3,
                run_mode=sp.MODE_AUTOTUNE, writes={'RATE_YAW_P': 5.0})
    fc = _fc(m)
    ok, why = fc.autotune(sp.AUTOTUNE_TOKEN, timeout=5.0)
    assert ok is True, why
    assert m.mode == sp.MODE_STABILIZE and not m.armed
    assert 'RATE_YAW_P' in why, 'must name what actually moved'


def test_autotune_that_moved_NO_PID_is_not_reported_as_success():
    """The depth phase is SKIPPED without a healthy baro, and a relay that never
    established a limit cycle writes nothing. An empty list is an ambiguity."""
    m = _Master(params=dict(_PIDS), finish_after=3,
                run_mode=sp.MODE_AUTOTUNE, writes={})
    ok, why = _fc(m).autotune(sp.AUTOTUNE_TOKEN, timeout=5.0)
    assert ok is True
    assert 'NO PID CHANGED' in why


def test_autotune_timeout_reports_WHAT_IT_SAW():
    m = _Master(params=dict(_PIDS), finish_after=None, run_mode=sp.MODE_AUTOTUNE)
    ok, why = _fc(m).autotune(sp.AUTOTUNE_TOKEN, timeout=1.0)
    assert ok is False
    assert 'did not finish' in why and 'mode' in why


def test_autotune_abort_LEAVES_THE_MODE_AND_DISARMS():
    """A mode change alone leaves thrusters live."""
    m = _Master(params=dict(_PIDS), finish_after=None, run_mode=sp.MODE_AUTOTUNE)
    fc = _fc(m)
    calls = {'n': 0}

    def _abort():
        calls['n'] += 1
        return calls['n'] > 1

    disarmed = {'v': False}
    fc.disarm = lambda: disarmed.__setitem__('v', True)
    ok, why = fc.autotune(sp.AUTOTUNE_TOKEN, timeout=5.0, abort_fn=_abort)
    assert ok is False and 'aborted' in why
    assert disarmed['v'], 'abort must DISARM, not only change mode'
    assert sp.MODE_STABILIZE in _mode_sets(fc)


# --------------------------------------------------------------------------- #
#  MOTOR_TUNE
# --------------------------------------------------------------------------- #
def test_motor_tune_refuses_when_MTUNE_EN_is_ZERO_and_does_not_enter_the_mode():
    """⛔ The silent no-op. DEF_MTUNE_EN = 0.0f, so the mode is accepted and the
    tuner never starts -- the vehicle sits in MOTOR_TUNE doing nothing."""
    m = _Master(params=dict(_MT, MTUNE_EN=0.0))
    fc = _fc(m)
    ok, why = fc.motor_tune(sp.MOTOR_TUNE_TOKEN, timeout=5.0)
    assert ok is False and 'DISABLED' in why
    assert sp.MODE_MOTOR_TUNE not in _mode_sets(fc)


def test_motor_tune_refuses_when_MTUNE_EN_is_UNREADABLE():
    """Absence is not zero, and it is not one either."""
    m = _Master(params={n: 1.0 for n in sp.MOTOR_TUNE_PARAMS})   # no MTUNE_EN
    fc = _fc(m)
    ok, why = fc.motor_tune(sp.MOTOR_TUNE_TOKEN, timeout=5.0)
    assert ok is False and 'could not read' in why
    assert sp.MODE_MOTOR_TUNE not in _mode_sets(fc)


def test_motor_tune_REFUSES_a_known_bad_thruster():
    m = _Master(params=dict(_MT))
    fc = _fc(m)
    fc.thruster_health = lambda: (False, 'thruster 3 LOST telemetry')
    ok, why = fc.motor_tune(sp.MOTOR_TUNE_TOKEN, timeout=5.0)
    assert ok is False and 'thruster 3' in why
    assert sp.MODE_MOTOR_TUNE not in _mode_sets(fc)


def test_motor_tune_RUNS_on_UNKNOWN_health_but_says_so():
    """⛔ The same asymmetry as the pre-fire gate. `None` is this hull's
    permanent state; refusing on it makes the method unreachable forever."""
    m = _Master(params=dict(_MT), finish_after=3,
                run_mode=sp.MODE_MOTOR_TUNE, writes={'FF_A': 3.0})
    fc = _fc(m)
    fc.thruster_health = lambda: (None, 'presence announced at first arm only')
    ok, why = fc.motor_tune(sp.MOTOR_TUNE_TOKEN, timeout=5.0)
    assert ok is True, why
    assert 'UNKNOWN' in why, 'a run on unknown telemetry must be flagged'
    assert 'FF_A' in why


def test_motor_tune_completes_on_STABILIZE_not_MANUAL():
    m = _Master(params=dict(_MT), finish_after=3,
                run_mode=sp.MODE_MOTOR_TUNE, writes={'RPM_KP': 9.0})
    fc = _fc(m)
    fc.thruster_health = lambda: (True, 'all 8 reporting')
    ok, why = fc.motor_tune(sp.MOTOR_TUNE_TOKEN, timeout=5.0)
    assert ok is True, why
    assert m.mode == sp.MODE_STABILIZE and not m.armed
    assert 'RPM_KP' in why


def test_motor_tune_that_changed_nothing_is_not_success():
    """With zero RPM telemetry this is the EXPECTED outcome, not a pass."""
    m = _Master(params=dict(_MT), finish_after=3,
                run_mode=sp.MODE_MOTOR_TUNE, writes={})
    fc = _fc(m)
    fc.thruster_health = lambda: (True, 'all 8 reporting')
    ok, why = fc.motor_tune(sp.MOTOR_TUNE_TOKEN, timeout=5.0)
    assert ok is True and 'NOTHING CHANGED' in why


def test_the_tune_REPORTS_PROGRESS_instead_of_going_silent():
    """⛔ A gap in our own first version, found by reading the firmware.

    The board publishes STUNT_PRG and the live limit-cycle measurement
    AT_N / AT_AMP / AT_TU / AT_OKPCT, added for a stated reason: "so a run can
    be watched from the GCS instead of only explained after it aborts"
    (fw control/autotune.h:34). Our first `autotune()` read NONE of it and sat
    silent for up to 150 s while all eight thrusters ran at full authority.
    """
    class _Log:
        def __init__(self):
            self.lines = []

        def info(self, m):
            self.lines.append(m)

        def warning(self, m):
            self.lines.append(m)

    m = _Master(params=dict(_PIDS), finish_after=200,      # long enough to report
                run_mode=sp.MODE_AUTOTUNE, writes={'RATE_RLL_P': 2.0})
    fc = SrotFC(m, log=_Log())
    fc.is_armed = lambda: m.armed
    fc.statustext_log = lambda since=0: []
    m.params.update({'STUNT_PRG': 42.0, 'AT_N': 7.0, 'AT_TU': 1.25,
                     'AT_OKPCT': 80.0, 'AT_AMP': 0.031})
    fc._named_cache = {k: (v, 9e18) for k, v in
                       (('STUNT_PRG', 42.0), ('AT_N', 7.0), ('AT_TU', 1.25),
                        ('AT_OKPCT', 80.0), ('AT_AMP', 0.031))}

    fc.autotune(sp.AUTOTUNE_TOKEN, timeout=7.0)
    tune_lines = [l for l in fc._log.lines if '[TUNE ]' in l]
    assert tune_lines, 'a multi-minute full-authority tune must not run silent'
    assert '42%' in tune_lines[0], tune_lines[0]
    assert 'Tu 1.25s' in tune_lines[0] and 'consensus 80%' in tune_lines[0], \
        'the limit-cycle detail is what separates "gathering" from "stuck"'


def test_no_run_here_relies_on_a_PRODUCTION_timeout():
    """A regression in the completion check must FAIL FAST, not hang.

    Written after an injected defect (modelling the end state on MOTOR_DETECT's
    MANUAL) ran for over TEN MINUTES instead of failing, because every
    completion test used the 150 s / 300 s production defaults. The timeout
    VALUE is irrelevant to what those tests assert, and a suite that takes
    minutes to report is a suite somebody disables.
    """
    src = Path(__file__).read_text()
    calls = re.findall(r'\.(?:autotune|motor_tune)\(sp\.\w+_TOKEN([^)]*)\)', src)
    bare = [c for c in calls if 'timeout=' not in c]
    assert bare == [], f'runs that would use the production timeout: {bare}'
