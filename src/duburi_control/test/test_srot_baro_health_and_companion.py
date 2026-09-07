"""Two NAMED_VALUE_FLOATs the board has always emitted and we never read.

A field-level inventory of all 31 names the firmware can emit (`mav_stream.cpp`
`sendNamed` call sites) against everything `duburi_ws/src` consumes found
exactly two gaps. Both are diagnostics whose entire purpose is to explain a
withdrawal or a misconfiguration, i.e. precisely the things that are invisible
until they matter:

  BARO_HEALTH  -- WHY depth withdrew (jitter / read failures / not initialised),
                  not merely that it did. The board sends it UNGATED on purpose:
                  "its whole job is to explain a withdrawal, so gating it on the
                  health it reports would hide it exactly when it matters"
                  (fw mav_stream.cpp:876-879).

  COMP_SEEN    -- has the board heard the companion its GCS failsafe is SCOPED
                  to (FS_GCS_SYSID/FS_GCS_COMPID, default 255/191 = us)?
                  (fw mav_stream.cpp:894, mav_commands.cpp:91-99.)

⛔ THE TRAP IN THE FIRST ONE. `NAMED_VALUE_FLOAT.name` is TEN CHARACTERS and the
firmware does not shorten its own strings, so "BARO_HEALTH" arrives as
"BARO_HEALT". Looking up the full name returns None -- which is
indistinguishable from "the board never sent it", so the bug is silent. This is
the same truncation that produced our worst misreading of this signal: read as a
health SCORE, `3` looked excellent when it means NOT INITIALISED.
"""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from duburi_control.fc import srot_protocol as sp          # noqa: E402
from duburi_control.fc.base import Telemetry               # noqa: E402
from duburi_control.fc.srot_fc import SrotFC               # noqa: E402


class _FC(SrotFC):
    """Drives the REAL `telemetry()`; only the inputs it reads are stubbed."""

    def __init__(self, named):
        self._named = named

    def _vehicle_hb(self):
        return None

    def _cache(self, key):
        return None

    def _baro_healthy(self):
        return False

    def sys_status_leak(self):
        return None

    def get_batteries(self, max_age_s: float = 5.0):
        return {}

    def _named_value(self, name):
        return self._named.get(name)


def _tel(**named):
    return _FC(named).telemetry()


# --------------------------------------------------------------------------- #
#  BARO_HEALTH — and the 10-char truncation
# --------------------------------------------------------------------------- #
def test_the_wire_name_is_the_TRUNCATED_one():
    """⛔ Reading 'BARO_HEALTH' finds nothing, for ever, and says nothing."""
    assert sp.NAME_BARO_HEALTH == 'BARO_HEALT'
    assert len(sp.NAME_BARO_HEALTH) <= 10, \
        'NAMED_VALUE_FLOAT.name is 10 chars; a longer key can never match'


def test_baro_health_is_read_from_the_truncated_name():
    assert _tel(BARO_HEALT=0.0).baro_health == 0.0
    assert _tel(BARO_HEALT=3.0).baro_health == 3.0


def test_the_untruncated_name_alone_is_NOT_enough():
    """The injection that proves the guard: send only the full name and the
    value must NOT arrive -- which is exactly the silent failure being fixed."""
    t = _tel(BARO_HEALTH=1.0)
    assert math.isnan(t.baro_health)


def test_absent_baro_health_stays_NaN_not_zero():
    """0 is HEALTHY. Defaulting an absent reading to 0 would claim a sensor is
    fine when the board has said nothing at all -- absence is not zero, and here
    the zero is the specifically reassuring value."""
    assert math.isnan(_tel().baro_health)
    assert math.isnan(Telemetry().baro_health)


def test_health_codes_are_faults_not_scores():
    """3 is the WORST state, not the best. This is the misreading that cost us a
    round: `BARO_HEALT = 3` was read as a health score on a bare board."""
    assert sp.baro_health_text(0) == 'healthy'
    assert 'not initialised' in sp.baro_health_text(3)
    assert 'jitter' in sp.baro_health_text(1)
    assert 'read failures' in sp.baro_health_text(2)


def test_unknown_and_missing_health_codes_are_not_invented():
    assert sp.baro_health_text(None) == 'not reported'
    assert sp.baro_health_text(float('nan')) == 'not reported'
    assert 'unknown code 9' in sp.baro_health_text(9)


# --------------------------------------------------------------------------- #
#  COMP_SEEN — tri-state, because "not said" is not "never seen"
# --------------------------------------------------------------------------- #
def test_companion_seen_is_TRI_STATE():
    assert _tel(COMP_SEEN=1.0).companion_seen is True
    assert _tel(COMP_SEEN=0.0).companion_seen is False
    assert _tel().companion_seen is None, \
        'no COMP_SEEN yet must stay None -- older firmware does not send it, and ' \
        'reporting that as "companion never seen" is a false alarm on every connect'


def test_default_telemetry_companion_is_unknown():
    assert Telemetry().companion_seen is None


def test_false_is_distinguishable_from_unknown():
    """The whole point. False is actionable (our heartbeat is not matching
    FS_GCS_SYSID/COMPID, so the failsafe is not watching us); None is not."""
    assert _tel(COMP_SEEN=0.0).companion_seen is not None
    assert _tel().companion_seen is not True
