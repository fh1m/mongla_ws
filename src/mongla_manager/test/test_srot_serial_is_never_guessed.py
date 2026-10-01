"""The SROT serial port is found, or refused -- never guessed (issue #20).

The board's CH340 has no serial number, so its by-id name is shared by every
CH340 attached. `find_srot_serial()` returned the FIRST match; opening the wrong
one RESETS it (DTR) and then waited forever on a heartbeat that never comes.
"""
import pytest

from mongla_manager import connection_config as cc


def _fs(monkeypatch, *, links=(), raw=(), real=None, udev=False):
    real = real or {}
    links = list(links)

    def fake_glob(pat):
        import fnmatch
        return [p for p in links if fnmatch.fnmatch(p, pat)]

    exists = set(raw) | ({cc.SROT_UDEV_LINK} if udev else set())
    monkeypatch.setattr(cc, 'glob', fake_glob)
    monkeypatch.setattr(cc.os.path, 'exists', lambda p: p in exists)
    monkeypatch.setattr(cc.os.path, 'realpath', lambda p: real.get(p, p))


BOARD = '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'
OTHER = '/dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0'


def test_one_ch340_is_used(monkeypatch):
    _fs(monkeypatch, links=[BOARD], raw=['/dev/ttyUSB0'],
        real={BOARD: '/dev/ttyUSB0'})
    assert cc.find_srot_serial() == BOARD


def test_two_ch340s_are_REFUSED_with_both_named(monkeypatch):
    """Falsified by a path being returned: that is the guess that resets a
    flasher or a LoRa dongle."""
    _fs(monkeypatch, links=[BOARD, OTHER],
        real={BOARD: '/dev/ttyUSB0', OTHER: '/dev/ttyUSB1'})
    with pytest.raises(cc.AmbiguousSrotSerial) as e:
        cc.find_srot_serial()
    assert BOARD in str(e.value) and OTHER in str(e.value)
    assert 'make_srot_rule.sh' in str(e.value)


def test_the_udev_link_wins_over_any_ambiguity(monkeypatch):
    _fs(monkeypatch, links=[BOARD, OTHER], udev=True,
        real={BOARD: '/dev/ttyUSB0', OTHER: '/dev/ttyUSB1'})
    assert cc.find_srot_serial() == cc.SROT_UDEV_LINK


def test_one_device_matched_by_two_patterns_is_still_one(monkeypatch):
    """`*1a86*` and `*USB_Serial*` both match the same board -- not ambiguous."""
    _fs(monkeypatch, links=[BOARD], real={BOARD: '/dev/ttyUSB0'})
    assert cc.find_srot_serial() == BOARD


def test_two_raw_nodes_and_no_by_id_are_refused(monkeypatch):
    _fs(monkeypatch, raw=['/dev/ttyUSB0', '/dev/ttyACM0'])
    with pytest.raises(cc.AmbiguousSrotSerial):
        cc.find_srot_serial()


def test_nothing_attached_is_None_not_an_error(monkeypatch):
    _fs(monkeypatch)
    assert cc.find_srot_serial() is None


def test_bringup_check_FAILS_the_section_instead_of_crashing(monkeypatch):
    from mongla_manager import bringup_check as bc
    _fs(monkeypatch, links=[BOARD, OTHER],
        real={BOARD: '/dev/ttyUSB0', OTHER: '/dev/ttyUSB1'})
    conn, state = bc._resolve_srot_for_check('')
    assert conn == '' and state.startswith('ambiguous')
