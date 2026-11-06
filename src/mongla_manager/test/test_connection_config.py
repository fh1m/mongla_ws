"""Unit tests for how the manager finds the SROT board.

`resolve_srot_profile` is in the live bring-up path -- a wrong auto-detect
opens the wrong device (which resets it) or blocks at wait_heartbeat. The
environment probes touch sockets / `/dev`, so they are monkeypatched here.
"""

import time
import pytest

from mongla_manager import connection_config as cc


# --- resolve_srot_profile: direct USB serial ---------------------------------

def test_srot_mav_device_override_serial():
    p = cc.resolve_srot_profile('/dev/ttyUSB0')
    assert p['conn'] == '/dev/ttyUSB0' and p['baud'] == cc.SROT_BAUD


def test_srot_mav_device_override_udp_no_baud():
    # A conn string (a serial->UDP bridge) keeps baud=None.
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
# Type-C (the designed transport) or UDP via a serial->UDP bridge while the hull is
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
    """MEASURED 2026-08-03: a leftover `mongla_manager start` still held 14550, the
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


# --- the probe must not judge on one datagram's first byte --------------------

def test_probe_finds_mavlink_when_the_datagram_starts_MID_MESSAGE():
    """A serial->UDP bridge chunks the serial stream at arbitrary offsets -- MEASURED,
    only ~77% of its datagrams begin on a message boundary.

    Judging on `data[0]` alone therefore returned 'silent' on roughly a quarter of
    attempts against a board streaming ~90 datagrams/s, in 0.02 s. bringup_check
    printed "no MAVLink on UDP" three lines above a healthy heartbeat read over that
    same link, and back-to-back runs flip-flopped -- the shape of intermittent that
    gets mistaken for a hardware fault.
    """
    import socket as _s
    import threading

    sender = _s.socket(_s.AF_INET, _s.SOCK_DGRAM)
    # A frame that STARTS mid-message: tail of one packet, then the 0xFD magic.
    payload = bytes([0x11, 0x22, 0x33, 0x44]) + bytes([0xFD, 0x09, 0x00, 0x00])
    stop = threading.Event()

    def blast(port):
        while not stop.is_set():
            try:
                sender.sendto(payload, ('127.0.0.1', port))
            except OSError:
                pass
            time.sleep(0.01)

    probe_port = 14993
    t = threading.Thread(target=blast, args=(probe_port,), daemon=True)
    t.start()
    try:
        assert cc.probe_udp_mavlink(probe_port, 1.5) == 'mavlink'
    finally:
        stop.set()
        t.join(timeout=1)
        sender.close()


def test_probe_reports_silent_only_after_waiting_the_whole_timeout():
    """A fast 'silent' is the bug signature: it means we bailed on a transient
    instead of waiting. Must consume the budget before concluding nothing is there."""
    t0 = time.monotonic()
    assert cc.probe_udp_mavlink(14994, 0.6) == 'silent'
    assert time.monotonic() - t0 >= 0.5, 'returned "silent" without waiting'
