"""nucleus_parser.py -- lean binary packet decoder for Nortek Nucleus 1000.

Packet framing and field extraction derived from the Nortek Nucleus 1000
reference driver. All queue, thread, logger, and connection infrastructure
is stripped -- this module is pure computation, no I/O.

Packet structure (binary, little-endian):
  [0]       0xa5  sync byte
  [1]       sizeHeader (uint8)  -- total header length in bytes
  [2]       id (uint8)          -- packet type
  [3]       family (uint8)      -- 0x20 = NUCLEUS
  [4:6]     sizeData (uint16)
  [6:8]     dataCheckSum (uint16)
  [8:10]    headerCheckSum (uint16)
  [sizeHeader : sizeHeader+sizeData]  data payload

Both checksums use the same XOR-sum: init=0xb58c, pairs of bytes summed as
uint16 LE (odd-length data zero-pads the last pair on the right).

Bottom Track (ID 0xb4) -- `data` = payload after header:
  data[12:16]    status word (uint32) -- validity bit flags
  data[96:100]   velocityX (float32 m/s, body-forward)
  data[100:104]  velocityY (float32 m/s, body-lateral)
  data[104:108]  velocityZ (float32 m/s, body-vertical)
  Status bits: beam1FomValid=6, beam2FomValid=7, xVelocityValid=9, yVelocityValid=10

AHRS (ID 0xd2) -- `data` = payload after header:
  data[1]        offsetOfData (uint8)
  data[offsetOfData+0:+4]   roll    (float32 deg)
  data[offsetOfData+4:+8]   pitch   (float32 deg)
  data[offsetOfData+8:+12]  heading (float32 deg, 0-360)
"""
from __future__ import annotations
from itertools import zip_longest
from struct import unpack, error as StructError

ID_BOTTOMTRACK: int = 0xb4
ID_AHRS: int        = 0xd2
_FAMILY_NUCLEUS: int = 0x20


def _checksum(data: bytes | bytearray) -> int:
    cs = 0xb58c
    for u, v in zip_longest(data[::2], data[1::2], fillvalue=0):
        cs += int(u) | (int(v) << 8)
        cs &= 0xFFFF
    return cs


def parse_packet(buf: bytearray) -> dict | None:
    """Decode one complete packet starting at buf[0] == 0xa5.

    Returns a dict with 'id' plus type-specific fields, or None on any
    framing / checksum error. Caller is responsible for providing exactly
    sizeHeader + sizeData bytes.
    """
    if len(buf) < 10 or buf[0] != 0xa5:
        return None

    try:
        size_header = buf[1]
        pkt_id      = buf[2]
        family      = buf[3]
        size_data   = unpack('<H', buf[4:6])[0]
        data_cs     = unpack('<H', buf[6:8])[0]
        header_cs   = unpack('<H', buf[8:10])[0]
    except (StructError, IndexError):
        return None

    if len(buf) < size_header + size_data:
        return None

    if _checksum(buf[:size_header - 2]) != header_cs:
        return None

    raw = buf[size_header:size_header + size_data]

    if _checksum(raw) != data_cs:
        return None

    if family != _FAMILY_NUCLEUS:
        return None

    if pkt_id not in (ID_BOTTOMTRACK, ID_AHRS):
        return {'id': pkt_id}

    try:
        if pkt_id == ID_BOTTOMTRACK:
            status = unpack('<I', raw[12:16])[0]
            return {
                'id': ID_BOTTOMTRACK,
                'beam1_fom_valid':  bool((status >> 6)  & 1),
                'beam2_fom_valid':  bool((status >> 7)  & 1),
                'x_velocity_valid': bool((status >> 9)  & 1),
                'y_velocity_valid': bool((status >> 10) & 1),
                'velocity_x': unpack('<f', raw[96:100])[0],
                'velocity_y': unpack('<f', raw[100:104])[0],
                'velocity_z': unpack('<f', raw[104:108])[0],
            }

        if pkt_id == ID_AHRS:
            offset = raw[1]
            return {
                'id': ID_AHRS,
                'roll':    unpack('<f', raw[offset:     offset + 4])[0],
                'pitch':   unpack('<f', raw[offset + 4: offset + 8])[0],
                'heading': unpack('<f', raw[offset + 8: offset + 12])[0],
            }

    except (StructError, IndexError):
        return None

    return None  # unreachable but satisfies type checkers


class PacketAccumulator:
    """Feed raw TCP chunks; get decoded packet dicts back.

    Usage:
        acc = PacketAccumulator()
        while True:
            chunk = sock.recv(4096)
            for pkt in acc.feed(chunk):
                if pkt['id'] == ID_BOTTOMTRACK:
                    ...
    """

    _MAX_PACKET = 2048  # sanity cap on a single Nucleus packet

    def __init__(self) -> None:
        self._buf: bytearray = bytearray()

    def feed(self, chunk: bytes) -> list[dict]:
        self._buf.extend(chunk)
        packets: list[dict] = []

        while True:
            # Scan for sync byte
            idx = self._buf.find(0xa5)
            if idx == -1:
                self._buf.clear()
                break
            if idx > 0:
                del self._buf[:idx]

            if len(self._buf) < 10:
                break

            size_header = self._buf[1]
            size_data   = (self._buf[4] & 0xFF) | ((self._buf[5] & 0xFF) << 8)
            total       = size_header + size_data

            if total > self._MAX_PACKET:
                # Bad sync byte; skip it and hunt for the next one
                del self._buf[:1]
                continue

            if len(self._buf) < total:
                break  # partial packet; wait for more data

            pkt_bytes = bytearray(self._buf[:total])
            del self._buf[:total]

            p = parse_packet(pkt_bytes)
            if p is not None:
                packets.append(p)

        return packets
