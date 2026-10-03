"""Not checked is not passed (found on the vehicle, 2026-10-03).

With a manager holding the board's port the gate skipped the whole board
section as a WARN and exited 0 -- passing over a Bar30 it FAILS with the port
free. Skipping is right (opening the port reboots the board); passing is not.
"""
import pytest

pytest.importorskip('pymavlink')


def test_a_held_port_fails_the_board_section(monkeypatch):
    import mongla_manager.bringup_check as bc
    from mongla_control.fc import port_guard

    class _Held:
        def __init__(self, port, **k):
            self.port = port

        def acquire(self):
            raise port_guard.PortBusy(f'{self.port} is already held by pid 4242 (start)')

    monkeypatch.setattr(port_guard, 'PortGuard', _Held)
    rows = bc._check_srot(skip_mav=False, device='/dev/ttyUSB9')
    statuses = [r[0] for r in rows]
    assert bc.FAIL in statuses, rows
    assert any('NOT' in r[2] and 'checked' in r[2] for r in rows if r[0] == bc.FAIL)
