"""Unit tests for the mode/profile resolver.

`resolve_mode` is in the live `mode:=auto` bring-up path — a wrong auto-detect
picks the wrong connection profile and the manager never sees MAVLink at the pool.
The two environment probes (`_udp_port_in_use`, `_pixhawk_serial_present`) touch
sockets / `/dev`, so they are monkeypatched here; the decision logic is pure.
"""

import pytest

from duburi_manager import connection_config as cc
from duburi_manager.connection_config import (
    resolve_mode, resolve_profile, PROFILES, _SERIAL_AUTO,
)


@pytest.fixture
def probes(monkeypatch):
    """Control both environment probes. Returns a setter (udp, serial)."""
    def set_state(*, udp: bool, serial: bool):
        monkeypatch.setattr(cc, '_udp_port_in_use', lambda *a, **k: udp)
        monkeypatch.setattr(cc, '_pixhawk_serial_present', lambda *a, **k: serial)
    return set_state


# --- resolve_mode: explicit profiles bypass probing ---------------------------

@pytest.mark.parametrize('mode', ['sim', 'pool', 'laptop', 'desk'])
def test_explicit_mode_returned_verbatim(mode, monkeypatch):
    # If an explicit known profile is requested, the probes must NOT run.
    def boom(*a, **k):
        raise AssertionError('probe ran for an explicit mode')
    monkeypatch.setattr(cc, '_udp_port_in_use', boom)
    monkeypatch.setattr(cc, '_pixhawk_serial_present', boom)
    assert resolve_mode(mode) == mode


def test_mode_is_normalized(monkeypatch):
    monkeypatch.setattr(cc, '_udp_port_in_use', lambda *a, **k: False)
    monkeypatch.setattr(cc, '_pixhawk_serial_present', lambda *a, **k: False)
    assert resolve_mode('  POOL ') == 'pool'


# --- resolve_mode: auto-detect decision table (CLAUDE.md §3) ------------------

def test_auto_picks_pool_when_udp_listening(probes):
    probes(udp=True, serial=False)
    assert resolve_mode('auto') == 'pool'


def test_auto_picks_desk_when_serial_present(probes):
    probes(udp=False, serial=True)
    assert resolve_mode('auto') == 'desk'


def test_auto_picks_sim_when_nothing(probes):
    probes(udp=False, serial=False)
    assert resolve_mode('auto') == 'sim'


def test_udp_wins_over_serial(probes):
    # Probe order: UDP checked before serial.
    probes(udp=True, serial=True)
    assert resolve_mode('auto') == 'pool'


def test_unknown_mode_falls_back_to_autodetect(probes):
    probes(udp=False, serial=False)
    assert resolve_mode('bogus') == 'sim'   # warns, then auto-detects


# --- resolve_profile ----------------------------------------------------------

def test_profile_sim_returns_sim_conn():
    prof = resolve_profile('sim')
    assert prof['conn'] == PROFILES['sim']['conn']


def test_profile_unknown_mode_falls_back_to_sim():
    assert resolve_profile('nonsense')['conn'] == PROFILES['sim']['conn']


def test_mav_device_serial_override_sets_baud():
    prof = resolve_profile('sim', mav_device='/dev/ttyACM0')
    assert prof['conn'] == '/dev/ttyACM0'
    assert prof['baud'] == 115200


def test_mav_device_udp_override_does_not_force_baud():
    prof = resolve_profile('sim', mav_device='udpin:0.0.0.0:14560')
    assert prof['conn'] == 'udpin:0.0.0.0:14560'
    assert prof['baud'] != 115200   # not a /dev/ path -> baud untouched


def test_desk_with_no_serial_falls_back_to_pool(monkeypatch):
    assert PROFILES['desk']['conn'] == _SERIAL_AUTO   # guard the precondition
    monkeypatch.setattr(cc, '_find_pixhawk_serial', lambda *a, **k: None)
    prof = resolve_profile('desk')
    assert prof['conn'] == PROFILES['pool']['conn']


def test_desk_with_serial_uses_found_path(monkeypatch):
    monkeypatch.setattr(cc, '_find_pixhawk_serial',
                        lambda *a, **k: '/dev/serial/by-id/usb-ArduPilot')
    prof = resolve_profile('desk')
    assert prof['conn'] == '/dev/serial/by-id/usb-ArduPilot'


# --- resolve_srot_profile: direct USB serial (no BlueOS) ----------------------

def test_srot_mav_device_override_serial():
    p = cc.resolve_srot_profile('/dev/ttyUSB0')
    assert p['conn'] == '/dev/ttyUSB0' and p['baud'] == cc.SROT_BAUD


def test_srot_mav_device_override_udp_no_baud():
    # A conn string (e.g. re-introduced BlueOS router) keeps baud=None.
    p = cc.resolve_srot_profile('udpout:192.168.2.2:14550')
    assert p['conn'] == 'udpout:192.168.2.2:14550' and p['baud'] is None


def test_srot_autodetect_uses_found_serial(monkeypatch):
    monkeypatch.setattr(cc, 'find_srot_serial',
                        lambda: '/dev/serial/by-id/usb-Silicon_Labs_CP2102')
    p = cc.resolve_srot_profile('')
    assert 'CP2102' in p['conn'] and p['baud'] == cc.SROT_BAUD


# --- resolve_srot_profile: auto-detect serial OR UDP --------------------------
#
# srot is the default backend and the board can now arrive two ways -- direct
# Type-C (the designed transport) or UDP via a BlueOS bridge while the hull is
# still wired Pi-first. Auto-detect must handle both with no arguments, and must
# say which it picked: a node that silently listens on the wrong transport just
# blocks at wait_heartbeat with a healthy-looking banner.

class _Log:
    def __init__(self): self.info_, self.warn_, self.err_ = [], [], []
    def info(self, m): self.info_.append(m)
    def warn(self, m): self.warn_.append(m)
    def warning(self, m): self.warn_.append(m)
    def error(self, m): self.err_.append(m)


def test_srot_autodetect_prefers_serial_over_udp(monkeypatch):
    """The Type-C cable always wins: it is the designed transport and was MEASURED
    at zero BAD_DATA against ~8-9% over the bridge."""
    monkeypatch.setattr(cc, 'find_srot_serial', lambda: '/dev/ttyUSB7')
    monkeypatch.setattr(cc, 'probe_udp_mavlink', lambda *a, **k: 'mavlink')
    p = cc.resolve_srot_profile('')
    assert p['conn'] == '/dev/ttyUSB7' and p['baud'] == cc.SROT_BAUD


def test_srot_autodetect_uses_udp_when_mavlink_is_arriving(monkeypatch):
    monkeypatch.setattr(cc, 'find_srot_serial', lambda: None)
    monkeypatch.setattr(cc, 'probe_udp_mavlink', lambda *a, **k: 'mavlink')
    log = _Log()
    p = cc.resolve_srot_profile('', logger=log)
    assert p['conn'] == cc.SROT_UDP_CONN and p['baud'] is None
    assert log.info_ and not log.err_


def test_a_held_port_is_reported_as_busy_not_as_no_board(monkeypatch):
    """MEASURED 2026-08-03: a leftover `duburi_manager start` still held 14550, the
    probe could not bind, reported 'no board', and then pymavlink bound the same
    port a second later and streamed fine. 'I could not look' must never collapse
    into 'nothing is there'."""
    monkeypatch.setattr(cc, 'find_srot_serial', lambda: None)
    monkeypatch.setattr(cc, 'probe_udp_mavlink', lambda *a, **k: 'busy')
    log = _Log()
    p = cc.resolve_srot_profile('', logger=log)
    assert p['conn'] == cc.SROT_UDP_CONN
    assert log.warn_ and not log.err_, 'a busy port must WARN, not ERROR'
    assert 'ss -lunp' in log.warn_[0], 'the warning must name how to find the holder'


def test_nothing_anywhere_is_a_loud_error_naming_both_transports(monkeypatch):
    monkeypatch.setattr(cc, 'find_srot_serial', lambda: None)
    monkeypatch.setattr(cc, 'probe_udp_mavlink', lambda *a, **k: 'silent')
    log = _Log()
    p = cc.resolve_srot_profile('', logger=log)
    assert p['conn'] == cc.SROT_UDP_CONN and p['baud'] is None
    assert log.err_, 'no board found must be an ERROR'
    assert 'mav_device' in log.err_[0], 'must name the explicit override'


def test_udp_probe_can_be_skipped(monkeypatch):
    """udp_probe_s=0 must not stall a caller that only wants the serial answer."""
    monkeypatch.setattr(cc, 'find_srot_serial', lambda: None)
    def _boom(*a, **k):
        raise AssertionError('probe ran despite udp_probe_s=0')
    monkeypatch.setattr(cc, 'probe_udp_mavlink', _boom)
    assert cc.resolve_srot_profile('', udp_probe_s=0)['conn'] == cc.SROT_UDP_CONN


def test_probe_returns_busy_when_the_port_is_already_bound():
    """Exercises the real socket path, not a stub."""
    import socket as _s
    holder = _s.socket(_s.AF_INET, _s.SOCK_DGRAM)
    holder.bind(('0.0.0.0', 0))
    port = holder.getsockname()[1]
    try:
        assert cc.probe_udp_mavlink(port, timeout=0.2) == 'busy'
    finally:
        holder.close()


def test_probe_returns_silent_on_a_free_quiet_port():
    assert cc.probe_udp_mavlink(0, timeout=0.2) in ('silent', 'busy')
