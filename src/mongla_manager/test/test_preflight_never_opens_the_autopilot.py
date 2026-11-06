"""B47 -- the preflight must not reboot the flight controller.

The SROT board is a CH340 (1a86:7523), and opening that serial port REBOOTS
it (`fc/port_guard.py` carries the measurements). `bringup_check` once had a
payload section that auto-detected a USB payload board by that same VID/PID and
OPENED what it found -- on this hull, the autopilot -- and reported PASS.

The payload is now the board's own channels over MAVLink, and that section is
gone. This pins that it stays gone: no USB payload probe, no second backend
whose branch could reach one.
"""
import os

_HERE = os.path.dirname(os.path.abspath(__file__))
_CHECK = os.path.join(_HERE, '..', 'mongla_manager', 'bringup_check.py')


def test_the_preflight_has_no_usb_payload_probe():
    src = open(_CHECK).read()
    for needle in ('PayloadDriver', 'auto_detect_port', 'mongla_control.payload'):
        assert needle not in src, (
            f'bringup_check references {needle!r} again -- a USB payload scan on '
            f'this hull returns the SROT board, and verifying it opens (reboots) it')


def test_the_preflight_has_one_backend():
    """A second backend is how the bare command once took the wrong branch."""
    src = open(_CHECK).read()
    assert "'--pixhawk'" not in src
    assert 'DEFAULT_FLIGHT_CONTROLLER' not in src
