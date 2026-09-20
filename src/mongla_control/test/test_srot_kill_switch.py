"""The 2nd-board thruster kill switch, and why `KILL = 0` is not "clear".

The switch is a rotary knob on the SECOND board: outside its two ON windows the
propulsion battery is physically disconnected and the hull cannot move at all.
Its state crosses to the control board over ESP-NOW, and on link loss the
firmware reports kill=false ON PURPOSE:

    kill = f ? s_kill : false;   // link lost -> don't assert kill (display-only)
                                 //   (fw src/drivers/espnow_link.cpp:49)

That is correct for a display and wrong for anything that gates, because it is
NOT display-only -- it is packed into NAMED_VALUE_FLOAT "KILL" and read by us.
So `KILL = 0` means "power is live" OR "nobody is telling us", and the two must
not collapse.

BATTERY_STATUS instance 1 is the disambiguator, free: the board SUPPRESSES it
rather than zeroing it when the ESP-NOW link is stale, so its presence IS the
link liveness.

⚠ OBSERVED LIVE before this fix: a session logged `thruster --` (no instance 1,
so no link) and `KILL clear` in the same status row.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from mongla_control.fc import srot_protocol as sp          # noqa: E402
from mongla_control.fc.srot_fc import SrotFC               # noqa: E402


class _FC(SrotFC):
    """Drives the REAL `telemetry()`. Only its six inputs are stubbed.

    ⚠ AN EARLIER VERSION OF THIS FILE REIMPLEMENTED the tri-state logic in the
    test helper. It passed, and it would have kept passing through any change to
    `telemetry()` -- the "compare what SHIPS, not a parallel implementation"
    trap, which this project has now hit three times (`_srot_drive` grepped for
    a string its own rename preserved; a tracker A/B that measured the bare
    library instead of our wrapper). The production method is called here.
    """

    def __init__(self, *, kill, link_up):
        self._kill = kill
        self._link = link_up

    # -- the six inputs telemetry() actually reads ------------------------- #
    def _vehicle_hb(self):
        return None

    def _cache(self, key):
        return None

    def _baro_healthy(self):
        return False

    def sys_status_leak(self):
        return None

    def get_batteries(self, max_age_s: float = 5.0):
        # Instance 1 present == the ESP-NOW link is alive. The board SUPPRESSES
        # it when stale, which is the whole mechanism under test.
        return {sp.BATTERY_ID_THRUSTER: {'voltage': 15.4, 'current': None}} \
            if self._link else {}

    def _named_value(self, name):
        if name != 'KILL':
            return None
        return None if self._kill is None else (1.0 if self._kill else 0.0)


def _kill_of(*, kill, link_up):
    return _FC(kill=kill, link_up=link_up).telemetry().kill_switch


# --------------------------------------------------------------------------- #
#  The three states
# --------------------------------------------------------------------------- #
def test_NO_LINK_is_UNKNOWN_not_CLEAR():
    """⛔ The defect, in one line. The board reports kill=false on link loss, so
    without the BATTERY_STATUS cross-check this reads as a live, safe vehicle."""
    assert _kill_of(kill=False, link_up=False) is None
    assert _kill_of(kill=True, link_up=False) is None, \
        'even a stale TRUE must not be asserted -- the board zeroed it already'


def test_LINK_UP_reports_the_real_state():
    assert _kill_of(kill=False, link_up=True) is False
    assert _kill_of(kill=True, link_up=True) is True


def test_link_up_but_KILL_never_seen_is_UNKNOWN():
    """A single-slot NAMED_VALUE_FLOAT that has not arrived is absence, not zero."""
    assert _kill_of(kill=None, link_up=True) is None


def test_the_default_Telemetry_is_UNKNOWN_not_CLEAR():
    """A Telemetry nobody filled in must not claim the hull is powered."""
    from mongla_control.fc.base import Telemetry
    assert Telemetry().kill_switch is None


# --------------------------------------------------------------------------- #
#  The arm gate
# --------------------------------------------------------------------------- #
class _ArmFC(SrotFC):
    def __init__(self, kill):
        self._kill = kill

    def telemetry(self):
        from mongla_control.fc.base import Telemetry
        t = Telemetry()
        t.kill_switch = self._kill
        return t


def test_arm_REFUSES_when_power_is_KNOWN_cut():
    ok, why = _ArmFC(True).check_thruster_power()
    assert ok is False
    assert 'CUT' in why and '2nd-board' in why


def test_arm_ALLOWS_on_UNKNOWN():
    """⛔ The asymmetry. A vehicle with no second board is an ordinary bench
    configuration; refusing on unknown would make arm() unreachable there --
    the same trap as refusing a torpedo on UNKNOWN thruster health."""
    ok, why = _ArmFC(None).check_thruster_power()
    assert ok is True, why


def test_arm_ALLOWS_when_power_is_live():
    assert _ArmFC(False).check_thruster_power()[0] is True


def test_a_RAISING_probe_does_not_ground_the_vehicle():
    class _Boom(SrotFC):
        def __init__(self):
            pass

        def telemetry(self):
            raise RuntimeError('link down')

    ok, why = _Boom().check_thruster_power()
    assert ok is True and 'unreadable' in why


def test_the_gate_is_WIRED_INTO_arm():
    """A gate nothing calls is not a gate -- the defect `check_for_reboot()` had."""
    import inspect
    src = inspect.getsource(SrotFC.arm)
    assert 'check_thruster_power' in src
