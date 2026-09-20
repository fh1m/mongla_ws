"""One process owns the srot serial port. Everything else is refused, loudly.

WHY THIS EXISTS -- measured on the bench, 2026-09-03
----------------------------------------------------
Opening `/dev/ttyUSB0` REBOOTS the flight controller. The ESP32 DevKit's
USB-UART bridge wires DTR/RTS to the EN/IO0 auto-reset circuit, so the tty
layer asserting DTR at open pulls the board into reset. Measured: three
consecutive opens each saw `ATTITUDE.time_boot_ms` restart at **545**, and the
board re-emitted its whole boot banner every time. Held open, uptime advances
normally (545 -> 7540 over 8 s).

Two things make this worse than a nuisance:

  * **Clearing HUPCL does not help.** That is the documented fix for the
    related "resets when the host disconnects" problem, and it was tried and
    measured: still 545 on every open.
  * **A SECOND opener reboots the board while a first process holds the
    port.** Linux does not lock a tty. Measured: with a holder connected and
    reading, a second open restarted the board (its first `time_boot_ms` was
    545) and the holder's own stream restarted with it.

So the failure mode is not theoretical. `mongla_ws` has FOUR places that open
a MAVLink connection -- `auv_manager_node`, `srot_connect`, and two in
`bringup_check` -- and `bringup_check` is a PREFLIGHT tool an operator runs by
hand. Running it against a live manager reboots the flight controller
mid-mission, and the only symptom is that the vehicle silently disarms and
re-initialises.

The kernel will not stop the second open, so we must. This is an advisory
`flock` on a lock FILE, not on the device: taking a lock on the device itself
would require opening it, which is the very thing that causes the reset.

WHAT THIS IS NOT
----------------
It cannot stop a process that does not use it (`cat /dev/ttyUSB0`, `pio device
monitor`, Bondor pointed at the same port). It is a guard against our own
tooling, which is where the realistic risk lives. The real fix is upstream --
a firmware/board option to defeat the auto-reset circuit -- and is filed as
such.
"""
from __future__ import annotations

import errno
import fcntl
import os
import re
from pathlib import Path

# Serial device paths only. A UDP/TCP endpoint has no auto-reset circuit and
# several processes may legitimately share one, so guarding it would refuse a
# workflow that is actually fine.
_SERIAL_RE = re.compile(r'^(/dev/|COM\d)')


def is_serial(path: str) -> bool:
    return bool(_SERIAL_RE.match(str(path)))


def _lock_dir() -> Path:
    """A per-user run directory, created on demand.

    Not /var/lock: that needs group membership we cannot assume on a fresh Pi,
    and a guard that fails to start because it cannot write its own lock file
    would be worse than no guard.
    """
    base = os.environ.get('XDG_RUNTIME_DIR') or f'/tmp/mongla-{os.environ.get("USER", os.getuid())}'
    d = Path(base) / 'port-locks'
    d.mkdir(parents=True, exist_ok=True)
    return d


def lock_path(device: str) -> Path:
    # Flatten the device path so /dev/serial/by-id/... and /dev/ttyUSB0 do not
    # collide in a flat directory. NOTE: they are still DIFFERENT names for the
    # same device, so a by-id holder and a by-path opener will not see each
    # other -- resolve to the real device first (see acquire()).
    return _lock_dir() / ('LCK..' + str(device).strip('/').replace('/', '_'))


def _holder(path: Path) -> str:
    """Best-effort description of whoever holds the lock. Never raises."""
    try:
        pid = int(path.read_text().split()[0])
    except Exception:
        return 'another process'
    try:
        cmd = Path(f'/proc/{pid}/cmdline').read_bytes().replace(b'\0', b' ').decode().strip()
        return f'pid {pid} ({cmd[:90]})' if cmd else f'pid {pid}'
    except Exception:
        return f'pid {pid} (no longer running?)'


class PortBusy(RuntimeError):
    """Raised instead of opening a port another mongla process already owns."""


class PortGuard:
    """Advisory exclusive claim on a serial device.

    Usage::

        guard = PortGuard('/dev/ttyUSB0')
        guard.acquire()          # raises PortBusy if someone else holds it
        try:
            ...
        finally:
            guard.release()

    Also works as a context manager. `acquire()` on a non-serial path is a
    no-op that returns False, so callers can wrap every connection uniformly.
    """

    def __init__(self, device: str, log=None):
        self.device = str(device)
        self._log = log
        self._fd: int | None = None
        self._path: Path | None = None

    # -- the resolved identity ------------------------------------------- #
    def _resolved(self) -> str:
        """Follow symlinks so /dev/serial/by-id/... and /dev/ttyUSB0 lock the
        SAME file. Without this the guard is trivially defeated by naming the
        same device two different ways -- which is exactly what our own config
        does, since `bringup_check` prefers by-id and the manager takes a
        literal path."""
        try:
            return os.path.realpath(self.device)
        except OSError:
            return self.device

    # -- lifecycle -------------------------------------------------------- #
    def acquire(self) -> bool:
        if not is_serial(self.device):
            return False
        path = lock_path(self._resolved())
        fd = os.open(path, os.O_RDWR | os.O_CREAT, 0o644)
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError as exc:
            if exc.errno not in (errno.EACCES, errno.EAGAIN):
                os.close(fd)
                raise
            who = _holder(path)
            os.close(fd)
            raise PortBusy(
                f'{self.device} is already held by {who}.\n'
                f'  Opening it a second time REBOOTS the srot board -- measured, and it '
                f'happens even while the first process is connected.\n'
                f'  Stop the other process, or point this one at a different device.'
            ) from None
        os.ftruncate(fd, 0)
        os.write(fd, f'{os.getpid()} {self.device}\n'.encode())
        self._fd, self._path = fd, path
        if self._log is not None:
            self._log.info(f'[PORT ] claimed {self.device} (lock {path.name})')
        return True

    def release(self) -> None:
        if self._fd is None:
            return
        try:
            fcntl.flock(self._fd, fcntl.LOCK_UN)
        except OSError:
            pass
        try:
            os.close(self._fd)
        except OSError:
            pass
        self._fd = None
        # The lock FILE is deliberately left behind. Unlinking it races: another
        # process can open the same path, then we unlink the inode it is holding
        # and a third process creates a fresh file and locks that instead --
        # two owners, both believing they are exclusive. A stale empty file
        # costs nothing.

    def __enter__(self):
        self.acquire()
        return self

    def __exit__(self, *exc):
        self.release()
        return False
