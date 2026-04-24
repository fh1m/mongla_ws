"""NucleusDVLSource -- Nortek Nucleus 1000 DVL over TCP.

YawSource + DVL position source in one. Connects directly over TCP to the
Nucleus 1000 and reads AHRS heading + bottom-track velocity packets. No
ROS dependency -- same pattern as bno085.py.

Hardware: Nortek Nucleus 1000 at 192.168.2.201:9000 on the AUV ethernet switch.

Lazy connect
------------
The source starts DISCONNECTED. Call `connect()` (typically via the
`dvl_connect` Move.action verb at pool-side) to open TCP, authenticate,
and start streaming. This matches pool workflow: deploy the AUV, arm
the DVL connection separately after confirming network, then run the
mission.

YawSource contract
------------------
read_yaw()   -- returns latest AHRS heading in degrees [0, 360), or None
is_healthy() -- True when connected and packets arriving within 3 s
close()      -- stops the reader thread and closes the socket

DVL extensions (not part of YawSource ABC)
-------------------------------------------
get_position()   -- returns (x_m, y_m) integrated body-frame position
reset_position() -- zero the integrator; call at start of each distance move
connect()        -- open TCP, authenticate, send START; raises on failure

AHRS heading convention
-----------------------
The Nucleus 1000 AHRS outputs heading in degrees [0, 360), NED convention
(0=North, +CW), independent of the Pixhawk magnetometer. No calibration
offset is applied here -- the same single-source design as bno085.
"""
from __future__ import annotations

import socket
import threading
import time
from datetime import datetime, timezone

from .base import YawSource
from .nucleus_parser import PacketAccumulator, ID_BOTTOMTRACK, ID_AHRS

_HOST    = '192.168.2.201'
_PORT    = 9000
_PASSWD  = 'nortek'
_TIMEOUT = 5.0          # TCP connect + read timeout (seconds)
_STALE   = 3.0          # declare unhealthy after this many seconds without packets
_DT_MAX  = 0.5          # discard integration step if dt exceeds this (stale packet)


class NucleusDVLSource(YawSource):
    """Nortek Nucleus 1000 over TCP. Lazy-connect; call connect() before use."""

    name: str = 'nucleus_dvl'

    def __init__(self, host: str = _HOST, port: int = _PORT,
                 password: str = _PASSWD, logger=None):
        self._host     = host
        self._port     = port
        self._password = password
        self._log      = logger  # anything with .info() / .warn(), or None

        self._sock: socket.socket | None = None
        self._thread: threading.Thread | None = None
        self._running = False

        self._lock = threading.Lock()

        # AHRS heading cache
        self._heading: float | None = None
        self._last_pkt_time: float = 0.0

        # Position integrator (body frame, metres)
        self._pos_x: float = 0.0
        self._pos_y: float = 0.0
        self._last_bt_time: float | None = None

    # ------------------------------------------------------------------
    #  YawSource ABC
    # ------------------------------------------------------------------

    def read_yaw(self) -> float | None:
        with self._lock:
            return self._heading

    def is_healthy(self) -> bool:
        if not self._running or self._sock is None:
            return False
        return (time.monotonic() - self._last_pkt_time) < _STALE

    def close(self) -> None:
        self._running = False
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=3.0)
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
        self._sock = None

    # ------------------------------------------------------------------
    #  DVL extensions
    # ------------------------------------------------------------------

    def get_position(self) -> tuple[float, float]:
        """Return (x_m, y_m) integrated body-frame position since last reset."""
        with self._lock:
            return self._pos_x, self._pos_y

    def reset_position(self) -> None:
        """Zero the body-frame position integrator."""
        with self._lock:
            self._pos_x = 0.0
            self._pos_y = 0.0
            self._last_bt_time = None

    def connect(self) -> None:
        """Open TCP, authenticate, send GETALL + START.

        Raises
        ------
        ConnectionError
            If TCP connect, authentication, or START fails.
        RuntimeError
            If already connected.
        """
        if self._running and self._sock is not None:
            raise RuntimeError('NucleusDVLSource: already connected')

        self._log_info(f'[DVL  ] connecting to {self._host}:{self._port}...')

        try:
            sock = socket.create_connection((self._host, self._port),
                                            timeout=_TIMEOUT)
        except OSError as e:
            raise ConnectionError(
                f'DVL TCP connect failed ({self._host}:{self._port}): {e}') from e

        try:
            self._authenticate(sock, self._password)
            self._startup(sock)
        except Exception as e:
            sock.close()
            raise ConnectionError(f'DVL startup sequence failed: {e}') from e

        self._sock    = sock
        self._running = True
        self._last_pkt_time = time.monotonic()

        self._thread = threading.Thread(
            target=self._reader_loop, daemon=True, name='nucleus_dvl_reader')
        self._thread.start()
        self._log_info('[DVL  ] connected and streaming')

    # ------------------------------------------------------------------
    #  Internal
    # ------------------------------------------------------------------

    def _log_info(self, msg: str) -> None:
        if self._log is not None:
            self._log.info(msg)

    @staticmethod
    def _send(sock: socket.socket, cmd: str) -> None:
        sock.sendall(cmd.encode())

    @staticmethod
    def _authenticate(sock: socket.socket, password: str) -> None:
        """Wait for the password prompt and reply."""
        deadline = time.monotonic() + _TIMEOUT
        buf = b''
        while time.monotonic() < deadline:
            chunk = sock.recv(256)
            if not chunk:
                raise ConnectionError('DVL closed connection during auth')
            buf += chunk
            if b'Please enter password:' in buf:
                sock.sendall(f'{password}\r\n'.encode())
                return
        raise ConnectionError('DVL auth: password prompt not received')

    @staticmethod
    def _startup(sock: socket.socket) -> None:
        """Send SETCLOCKSTR, GETALL, START in sequence."""
        now = datetime.now(timezone.utc)
        clock_str = now.strftime('%Y/%m/%d,%H:%M:%S.000')
        NucleusDVLSource._send(sock, f'SETCLOCKSTR,TIME="{clock_str}"\r\n')
        time.sleep(0.1)
        NucleusDVLSource._send(sock, 'GETALL\r\n')
        time.sleep(0.2)
        NucleusDVLSource._send(sock, 'START\r\n')
        time.sleep(0.1)

    def _reader_loop(self) -> None:
        sock = self._sock
        assert sock is not None
        acc = PacketAccumulator()
        while self._running:
            try:
                sock.settimeout(1.0)
                chunk = sock.recv(4096)
            except socket.timeout:
                continue
            except OSError:
                self._log_info('[DVL  ] socket error -- reader stopped')
                break

            if not chunk:
                self._log_info('[DVL  ] remote closed connection')
                break

            for pkt in acc.feed(chunk):
                self._last_pkt_time = time.monotonic()
                self._handle_packet(pkt)

        self._running = False

    def _handle_packet(self, pkt: dict) -> None:
        pkt_id = pkt.get('id')

        if pkt_id == ID_AHRS:
            with self._lock:
                self._heading = pkt['heading']

        elif pkt_id == ID_BOTTOMTRACK:
            if not (pkt['beam1_fom_valid'] and pkt['beam2_fom_valid']
                    and pkt['x_velocity_valid'] and pkt['y_velocity_valid']):
                return

            now = time.monotonic()
            with self._lock:
                if self._last_bt_time is not None:
                    dt = now - self._last_bt_time
                    if 0 < dt < _DT_MAX:
                        self._pos_x += pkt['velocity_x'] * dt
                        self._pos_y += pkt['velocity_y'] * dt
                self._last_bt_time = now
