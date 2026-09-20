"""B38 -- a failure must be VISIBLE in the log, not only in a returned value.

The commonest real failure on this link is a move stall: the board accepts a
SROT_MOVE and never reports a terminal result. Before this, it was reported
*only* as `MoveResult(TIMEOUT, ...)`. Nothing logged it. Whether the operator saw
anything depended entirely on who consumed the result -- and the DSL logged every
outcome, success and failure alike, through the same formatter at INFO.

So during a pool run the moment the vehicle stopped responding produced a line
that looked exactly like the nineteen successful lines above it.

Two changes, pinned here:
  * `SrotFC` WARNs at the instant of the stall, so it is visible regardless of
    consumer, and names the two causes that are indistinguishable from the host
    side (dead link vs. a board refusing AUTO on an unhealthy Bar30).
  * `MonglaMission._send` levels the log line by outcome -- INFO on success,
    WARNING with a `!!` marker on failure.
"""

import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).parent))


class _Log:
    def __init__(self):
        self.info_lines, self.warn_lines = [], []
    def info(self, m): self.info_lines.append(m)
    def warning(self, m): self.warn_lines.append(m)
    def warn(self, m): self.warn_lines.append(m)
    def error(self, m): self.warn_lines.append(m)
    def debug(self, m): pass


def test_a_stall_warns_at_the_moment_it_happens():
    from test_srot_fc import _fc
    log = _Log()
    fc = _fc()
    fc._log = log
    res = fc.move('move_forward', duration=1.0, gain=50)

    assert res.code == 4, 'expected TIMEOUT'
    stall = [m for m in log.warn_lines if 'STALL' in m]
    assert stall, 'a stall must WARN, not only return a MoveResult'
    msg = stall[0]
    assert 'connect' in msg, 'name the tool that distinguishes the two causes'
    assert 'Bar30' in msg, 'the unhealthy-baro refusal looks identical -- say so'
    assert 'braked' in msg, 'say the hull was braked, so nobody wonders'


def test_a_stall_still_brakes_the_hull():
    """The warning must not have displaced the braking."""
    import mongla_control.fc.srot_protocol as sp
    from test_srot_fc import _fc
    fc = _fc()
    fc.move('move_forward', duration=1.0, gain=50)
    stops = [s for s in fc.master.mav.sent
             if s[0] == 'cmd' and s[1] == sp.CMD_SROT_MOVE and s[2][0] == sp.MOVE_STOP]
    assert stops, 'a stall must brake the vehicle'


def _mission(results):
    from mongla_planner.mongla_dsl import MonglaMission

    class _C:
        def send(self, cmd, **kw):
            ok = results.get(cmd, True)
            return SimpleNamespace(success=ok, message='ok' if ok else 'stall',
                                   final_value=0.0, error_value=0.0)
    d = MonglaMission.__new__(MonglaMission)
    d.client, d.log, d._scoreboard = _C(), _Log(), []
    d.camera, d.target = 'forward', ''
    return d


def test_a_failed_verb_logs_at_WARNING_not_INFO():
    d = _mission({'set_depth': False})
    d._send('set_depth')
    assert d.log.warn_lines, 'a failed verb must not be logged at INFO'
    assert not d.log.info_lines


def test_a_successful_verb_still_logs_at_INFO():
    d = _mission({})
    d._send('move_forward')
    assert d.log.info_lines and not d.log.warn_lines


def test_the_failure_line_is_visually_distinct():
    """Same level is not enough -- a scrollback is read by eye, fast."""
    d = _mission({'set_depth': False})
    d._send('set_depth')
    assert d.log.warn_lines[0].startswith('!!'), \
        'mark the failure so it is findable in a wall of green'
