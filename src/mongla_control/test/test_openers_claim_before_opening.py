"""B48 -- every operator-supplied device path is claimed before it is opened.

B47 fixed the path by which `bringup_check` came to open the flight controller.
This is the class behind it: an **explicit device path bypasses every VID/PID
filter**, so the auto-detect safeguards do not apply to it at all. On this hull
one stale `sensors.yaml` line or one typo --

    yaw_source:=bno085 -p bno085_port:=/dev/ttyUSB0

-- opens the SROT board, and opening it REBOOTS it. `dtr=False`/`rts=False`
prevent the esptool reset circuit; they do not prevent the reboot that an open
itself causes. If the manager is up, its board disarms and reinitialises and
nothing in the graph says why.

`PortGuard` already existed and already had the measurements in its docstring.
It simply was not on these two paths.

WHY THIS ASSERTS ORDER, NOT PRESENCE: a claim taken *after* the port is open is
not a claim -- the reboot has already happened. A test that greps for the word
`PortGuard` passes on that broken arrangement, which is the mistake B42 made
(matching a comment's text position instead of the call order).
"""
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_WS   = os.path.abspath(os.path.join(_HERE, '..', '..', '..'))
for _p in ('mongla_control', 'mongla_sensors'):
    _s = os.path.join(_WS, 'src', _p)
    if _s not in sys.path:
        sys.path.insert(0, _s)


def _order_of(path, marks):
    """Return the marks that appear, in source order. Cheap and honest.

    Line numbers, not the presence of a string: `acquire` below `open()` is the
    defect this file exists to catch.
    """
    src = open(path).read().splitlines()
    hits = []
    for i, line in enumerate(src):
        for name, needle in marks:
            if needle in line:
                hits.append((i, name))
    return [n for _, n in sorted(hits)]


def test_the_bno_claims_the_port_before_opening_it():
    p = os.path.join(_WS, 'src', 'mongla_sensors', 'mongla_sensors',
                     'sources', 'bno085.py')
    order = _order_of(p, [('acquire', 'self._guard.acquire()'),
                          ('open',    '_ser.open()')])
    assert order[:2] == ['acquire', 'open'], (
        f'BNO opens the device before claiming it (order={order}) -- the reboot '
        f'has already happened by the time the guard is taken')


def test_the_payload_claims_the_port_before_opening_it():
    p = os.path.join(_WS, 'src', 'mongla_control', 'mongla_control', 'payload.py')
    order = _order_of(p, [('acquire', 'self._guard.acquire()'),
                          ('open',    '_p.open()')])
    assert order[:2] == ['acquire', 'open'], (
        f'payload opens the device before claiming it (order={order})')


def test_both_release_the_claim_when_they_close():
    """A guard never released is a device nothing can reopen -- including us."""
    bno = open(os.path.join(_WS, 'src', 'mongla_sensors', 'mongla_sensors',
                            'sources', 'bno085.py')).read()
    pay = open(os.path.join(_WS, 'src', 'mongla_control', 'mongla_control',
                            'payload.py')).read()
    assert 'self._guard.release()' in bno, 'BNO close() leaks the port claim'
    assert 'self._guard.release()' in pay, 'payload disconnect() leaks the port claim'


def test_the_guard_is_inert_for_the_sim_pty():
    """The sim drives the UNMODIFIED driver through a PTY under /tmp.

    `is_serial` matches `^/dev/` only, so the guard must not engage there. If it
    ever did, every sim run would start taking lock files for pseudo-terminals
    and the BNO sim path would break for a hazard that does not exist there.
    """
    from mongla_control.fc.port_guard import PortGuard, is_serial
    pty = f'/tmp/mongla-{os.environ.get("USER", "x")}/bno085'
    assert not is_serial(pty)
    assert PortGuard(pty).acquire() is False


def test_a_second_opener_is_refused_rather_than_taking_the_device(tmp_path, monkeypatch):
    """The property that matters, exercised through the real PortGuard.

    Two processes wanting one device must end with ONE of them refused and told
    who holds it -- never both believing they have it, which is the state that
    reboots the board under a running mission.
    """
    import mongla_control.fc.port_guard as pg
    monkeypatch.setattr(pg, 'is_serial', lambda p: True)
    monkeypatch.setattr(pg, '_lock_dir', lambda: tmp_path)

    dev = str(tmp_path / 'ttyFAKE')
    open(dev, 'w').close()

    first = pg.PortGuard(dev)
    assert first.acquire() is True
    try:
        second = pg.PortGuard(dev)
        try:
            second.acquire()
        except pg.PortBusy as exc:
            assert 'REBOOTS' in str(exc) or 'already held' in str(exc)
        else:
            raise AssertionError('a second opener was allowed to take the device')
    finally:
        first.release()

    # and the device is claimable again once released
    third = pg.PortGuard(dev)
    assert third.acquire() is True
    third.release()
