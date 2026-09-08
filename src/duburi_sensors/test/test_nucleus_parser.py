"""Unit tests for the Nortek Nucleus 1000 binary packet decoder.

Pure byte logic (no socket/thread), so fully unit-testable. Packets are framed
with the module's own `_checksum` so valid packets are valid by construction;
corruption tests then flip exactly one thing and assert rejection.
"""

import struct

import pytest

from duburi_sensors.sources.nucleus_parser import (
    _checksum, parse_packet, PacketAccumulator,
    ID_BOTTOMTRACK, ID_AHRS, _FAMILY_NUCLEUS,
)

_SYNC = 0xA5
_HDR = 10  # size_header used throughout these tests


def _frame(pkt_id: int, data: bytes, *, family: int = _FAMILY_NUCLEUS) -> bytearray:
    """Build a full, checksum-valid packet: 10-byte header + data."""
    size_data = len(data)
    data_cs = _checksum(data)
    head = bytearray([_SYNC, _HDR, pkt_id, family])
    head += struct.pack('<H', size_data)
    head += struct.pack('<H', data_cs)
    header_cs = _checksum(head[:8])      # checksum covers header[:size_header-2]
    head += struct.pack('<H', header_cs)
    return head + bytearray(data)


def _ahrs_data(roll: float, pitch: float, heading: float, offset: int = 8) -> bytes:
    d = bytearray(offset + 12)
    d[1] = offset                        # offsetOfData
    struct.pack_into('<fff', d, offset, roll, pitch, heading)
    return bytes(d)


def _bt_data(status: int, vx: float, vy: float, vz: float) -> bytes:
    d = bytearray(108)
    struct.pack_into('<I', d, 12, status)
    struct.pack_into('<fff', d, 96, vx, vy, vz)
    return bytes(d)


# --- _checksum ----------------------------------------------------------------

def test_checksum_init_value_on_empty():
    assert _checksum(b'') == 0xB58C


def test_checksum_odd_length_zero_pads():
    # Last byte pairs with an implicit 0x00 high byte.
    assert _checksum(b'\x01') == (0xB58C + 0x01) & 0xFFFF


def test_checksum_wraps_uint16():
    assert _checksum(b'\xff\xff' * 10) <= 0xFFFF


# --- parse_packet: valid packets ----------------------------------------------

def test_parse_ahrs():
    pkt = _frame(ID_AHRS, _ahrs_data(1.5, -2.5, 270.0))
    out = parse_packet(pkt)
    assert out['id'] == ID_AHRS
    assert out['roll'] == pytest.approx(1.5)
    assert out['pitch'] == pytest.approx(-2.5)
    assert out['heading'] == pytest.approx(270.0)


def test_parse_bottom_track_velocities_and_flags():
    # bits 6 (beam1) and 9 (x_velocity) set; 7 and 10 clear.
    status = (1 << 6) | (1 << 9)
    pkt = _frame(ID_BOTTOMTRACK, _bt_data(status, 0.3, -0.1, 0.05))
    out = parse_packet(pkt)
    assert out['id'] == ID_BOTTOMTRACK
    assert out['beam1_fom_valid'] is True
    assert out['beam2_fom_valid'] is False
    assert out['x_velocity_valid'] is True
    assert out['y_velocity_valid'] is False
    assert out['velocity_x'] == pytest.approx(0.3)
    assert out['velocity_y'] == pytest.approx(-0.1)
    assert out['velocity_z'] == pytest.approx(0.05)


def test_parse_unknown_id_returns_id_only():
    pkt = _frame(0x99, b'\x00\x00\x00\x00')
    assert parse_packet(pkt) == {'id': 0x99}


# --- parse_packet: rejection paths --------------------------------------------

def test_reject_bad_sync_byte():
    pkt = _frame(ID_AHRS, _ahrs_data(0, 0, 0))
    pkt[0] = 0x00
    assert parse_packet(pkt) is None


def test_reject_too_short():
    assert parse_packet(bytearray(b'\xa5\x0a')) is None


def test_reject_wrong_family():
    pkt = _frame(ID_AHRS, _ahrs_data(0, 0, 0), family=0x10)
    assert parse_packet(pkt) is None


def test_reject_corrupt_data_checksum():
    pkt = _frame(ID_AHRS, _ahrs_data(1.0, 2.0, 3.0))
    pkt[-1] ^= 0xFF          # flip a payload byte -> data checksum mismatch
    assert parse_packet(pkt) is None


def test_reject_truncated_payload():
    pkt = _frame(ID_BOTTOMTRACK, _bt_data(0, 0, 0, 0))
    assert parse_packet(pkt[:-4]) is None   # fewer than size_header+size_data


# --- PacketAccumulator --------------------------------------------------------

def test_accumulator_two_packets_one_chunk():
    a = _frame(ID_AHRS, _ahrs_data(10.0, 0.0, 90.0))
    b = _frame(ID_BOTTOMTRACK, _bt_data(0, 1.0, 0.0, 0.0))
    pkts = PacketAccumulator().feed(bytes(a + b))
    assert [p['id'] for p in pkts] == [ID_AHRS, ID_BOTTOMTRACK]


def test_accumulator_reassembles_split_packet():
    a = _frame(ID_AHRS, _ahrs_data(5.0, 5.0, 5.0))
    acc = PacketAccumulator()
    assert acc.feed(bytes(a[:6])) == []      # partial -> nothing yet
    out = acc.feed(bytes(a[6:]))
    assert len(out) == 1 and out[0]['id'] == ID_AHRS


def test_accumulator_skips_leading_garbage():
    a = _frame(ID_AHRS, _ahrs_data(1.0, 1.0, 1.0))
    out = PacketAccumulator().feed(b'\x00\x11\x22' + bytes(a))
    assert len(out) == 1 and out[0]['id'] == ID_AHRS


# --- BUGS.md B02 / B03: resync on a FALSE sync byte ---------------------------
# The three accumulator tests above all feed garbage containing no 0xA5, so they
# exercise only the find(0xA5) skip. Both real defects need a 0xA5 *inside* the
# garbage -- the only case where the wire's length field is trusted. These two
# pin the reproductions from the 2026-09-08 audit. They are xfail because this
# was a find-only pass: the tests document the bugs, they do not fix them.

def test_b02_zero_length_packet_does_not_hang():
    """A 0xA5 with size_header=0 and size_data=0 makes total=0, so feed()
    deletes nothing and loops on a byte-identical buffer, forever.

    Before the fix this HUNG rather than failing (verified: `timeout 10` -> exit
    124), which is why it was carried as xfail(run=False). It now returns.
    """
    evil = bytes([0xA5, 0x00, ID_BOTTOMTRACK, _FAMILY_NUCLEUS,
                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11, 0x22])
    assert PacketAccumulator().feed(evil) == []


def test_b03_false_sync_does_not_eat_the_following_packet():
    """On a checksum failure the accumulator has already consumed `total`
    bytes, so a spurious 0xA5 with a plausible length field discards whatever
    real packets began inside that span.

    Measured: 2 valid packets fed, 1 recovered.
    """
    good = bytes(_frame(ID_AHRS, _ahrs_data(1.0, 2.0, 123.0)))
    # size_header=10, size_data=20 -- plausible, checksums junk. The claimed
    # length must be <= what the buffer holds, or the accumulator simply STALLS
    # waiting for more bytes and the packet is never eaten. Measured against the
    # pre-fix feed(): this input recovered 1 of 2; the fix recovers 2 of 2.
    false_sync = bytes([_SYNC, _HDR, 0x99, _FAMILY_NUCLEUS, 20, 0x00]) + bytes(4)

    out = PacketAccumulator().feed(false_sync + good + good)

    assert len(out) == 2, f'false sync ate a real packet: recovered {len(out)}/2'
