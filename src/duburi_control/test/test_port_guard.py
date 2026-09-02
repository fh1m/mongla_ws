"""The port guard, and the reboot detector it exists to make unnecessary.

Both defend against one measured fact: opening the srot serial device REBOOTS
the flight controller, and it does so even while another process holds the
port. See `fc/port_guard.py` for the measurements.
"""
import os
import subprocess
import sys
import tempfile
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from duburi_control.fc.port_guard import (      # noqa: E402
    PortBusy, PortGuard, is_serial, lock_path,
)


# --------------------------------------------------------------------------- #
#  What counts as a port worth guarding
# --------------------------------------------------------------------------- #
def test_only_serial_devices_are_guarded():
    """A UDP endpoint has no auto-reset circuit and several processes may share
    one legitimately -- guarding it would refuse a workflow that is fine."""
    assert is_serial('/dev/ttyUSB0')
    assert is_serial('/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0')
    assert not is_serial('udpin:0.0.0.0:14550')
    assert not is_serial('tcp:127.0.0.1:5763')


def test_acquire_is_a_noop_on_a_non_serial_path():
    """So every call site can wrap its connection uniformly, without each one
    re-deciding whether the guard applies."""
    g = PortGuard('udpin:0.0.0.0:14550')
    assert g.acquire() is False
    g.release()                       # must not raise


# --------------------------------------------------------------------------- #
#  Exclusion
# --------------------------------------------------------------------------- #
def _fake_device(tmp_path) -> str:
    dev = tmp_path / 'ttyFAKE0'
    dev.write_text('')
    return str(dev)


def test_a_second_acquire_is_refused(tmp_path, monkeypatch):
    monkeypatch.setattr('duburi_control.fc.port_guard.is_serial', lambda p: True)
    dev = _fake_device(tmp_path)
    first = PortGuard(dev)
    assert first.acquire() is True
    try:
        with pytest.raises(PortBusy):
            PortGuard(dev).acquire()
    finally:
        first.release()


def test_release_hands_the_claim_back(tmp_path, monkeypatch):
    monkeypatch.setattr('duburi_control.fc.port_guard.is_serial', lambda p: True)
    dev = _fake_device(tmp_path)
    a = PortGuard(dev)
    a.acquire()
    a.release()
    b = PortGuard(dev)
    assert b.acquire() is True        # not permanently poisoned
    b.release()


def test_the_refusal_names_the_holder(tmp_path, monkeypatch):
    """'Port busy' with no name sends the operator hunting. The whole point is
    to say which process to stop."""
    monkeypatch.setattr('duburi_control.fc.port_guard.is_serial', lambda p: True)
    dev = _fake_device(tmp_path)
    first = PortGuard(dev)
    first.acquire()
    try:
        with pytest.raises(PortBusy) as exc:
            PortGuard(dev).acquire()
        assert str(os.getpid()) in str(exc.value)
        assert 'REBOOT' in str(exc.value)     # says WHY, not just that it refused
    finally:
        first.release()


def test_two_names_for_one_device_collide(tmp_path, monkeypatch):
    """The guard is trivially defeated if /dev/serial/by-id/... and /dev/ttyUSB0
    take different locks -- and our own tooling does exactly that, since
    bringup_check prefers by-id while the manager takes a literal path. The
    lock is keyed on the RESOLVED device."""
    monkeypatch.setattr('duburi_control.fc.port_guard.is_serial', lambda p: True)
    real = tmp_path / 'ttyFAKE0'
    real.write_text('')
    link = tmp_path / 'by-id-alias'
    link.symlink_to(real)

    assert lock_path(os.path.realpath(str(link))) == lock_path(str(real))

    first = PortGuard(str(real))
    first.acquire()
    try:
        with pytest.raises(PortBusy):
            PortGuard(str(link)).acquire()      # the alias must be refused too
    finally:
        first.release()


def test_the_lock_survives_the_holder_dying(tmp_path, monkeypatch):
    """flock is released by the kernel when the fd closes, so a crashed holder
    must NOT leave the port permanently claimed -- a guard that strands the
    vehicle after a crash is worse than the reboot it prevents."""
    monkeypatch.setattr('duburi_control.fc.port_guard.is_serial', lambda p: True)
    dev = _fake_device(tmp_path)
    child = subprocess.run(
        [sys.executable, '-c',
         'import sys; sys.path.insert(0, %r)\n'
         'from duburi_control.fc.port_guard import PortGuard\n'
         'import duburi_control.fc.port_guard as pg\n'
         'pg.is_serial = lambda p: True\n'
         'g = PortGuard(%r); g.acquire()\n'
         % (str(Path(__file__).resolve().parents[1]), dev)],
        capture_output=True, text=True, timeout=30,
        env={**os.environ, 'XDG_RUNTIME_DIR': os.environ.get(
            'XDG_RUNTIME_DIR', tempfile.gettempdir())})
    assert child.returncode == 0, child.stderr
    after = PortGuard(dev)
    assert after.acquire() is True
    after.release()


# --------------------------------------------------------------------------- #
#  The reboot detector
# --------------------------------------------------------------------------- #
class _Att:
    def __init__(self, ms):
        self.time_boot_ms = ms

    def get_type(self):
        return 'ATTITUDE'


class _FC:
    """Minimal stand-in exposing just what check_for_reboot touches."""
    from duburi_control.fc.srot_fc import SrotFC
    check_for_reboot = SrotFC.check_for_reboot
    _REBOOT_DROP_MS = SrotFC._REBOOT_DROP_MS

    def __init__(self):
        self._peak_boot_ms = None
        self._reboots = 0
        self._log = None
        self._att = None

    def _cache(self, msgtype):
        return self._att if msgtype == 'ATTITUDE' else None


def test_a_reboot_is_detected_once():
    fc = _FC()
    fc._att = _Att(20000)
    assert fc.check_for_reboot() is False        # first sample only primes it
    fc._att = _Att(21000)
    assert fc.check_for_reboot() is False
    fc._att = _Att(545)                          # the measured post-reboot value
    assert fc.check_for_reboot() is True
    fc._att = _Att(1545)                         # climbing again -- not a NEW reboot
    assert fc.check_for_reboot() is False
    assert fc._reboots == 1


def test_a_small_backwards_step_is_not_a_reboot():
    """Messages are stamped when built and the tx queue can reorder them
    slightly. Treating every backwards millisecond as a restart would cry wolf
    until the warning is ignored -- which is how a real reboot gets missed."""
    fc = _FC()
    fc._att = _Att(20000)
    fc.check_for_reboot()
    fc._att = _Att(19940)                        # 60 ms of reordering
    assert fc.check_for_reboot() is False
    assert fc._reboots == 0


def test_no_attitude_yet_is_not_a_reboot():
    fc = _FC()
    assert fc.check_for_reboot() is False
