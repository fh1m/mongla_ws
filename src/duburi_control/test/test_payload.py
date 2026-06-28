"""PayloadDriver.fire concurrency test.

The mid-hold vision fire runs on a background thread; it must not interleave
on the serial port with a later standalone fire(). PayloadDriver._fire_lock
serialises the writes. A fake port (no real serial) records enter/exit windows
so an overlap would be visible.
"""

import threading
import time

from duburi_control.payload import PayloadDriver


class _SlowPort:
    """Fake serial port whose write() takes time, recording overlap."""
    is_open = True

    def __init__(self):
        self.active = 0
        self.max_active = 0
        self.writes = []
        self._guard = threading.Lock()

    def write(self, data):
        # Track how many writers are inside write() at once. With the driver's
        # _fire_lock this must never exceed 1.
        with self._guard:
            self.active += 1
            self.max_active = max(self.max_active, self.active)
        time.sleep(0.05)
        self.writes.append(bytes(data))
        with self._guard:
            self.active -= 1

    def flush(self):
        pass


def _driver_with_port(port):
    d = PayloadDriver()
    d._port = port           # inject open fake port -> is_ready True
    return d


def test_fire_serialises_concurrent_writes():
    port = _SlowPort()
    d = _driver_with_port(port)

    threads = [threading.Thread(target=d.fire, args=(ch,)) for ch in (1, 2, 3)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()

    assert port.max_active == 1, 'concurrent fire() writes interleaved on the port'
    assert len(port.writes) == 3                 # all three fired
    assert set(port.writes) == {bytes([0x31]), bytes([0x32]), bytes([0x33])}


def test_fire_rejects_out_of_range_channel():
    d = _driver_with_port(_SlowPort())
    assert d.fire(9) is False                    # not 1-4 -> rejected, no write
