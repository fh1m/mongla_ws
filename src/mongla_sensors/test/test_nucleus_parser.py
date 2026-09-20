"""Unit tests for the Nortek Nucleus 1000 binary packet decoder.

Pure byte logic (no socket/thread), so fully unit-testable. Packets are framed
with the module's own `_checksum` so valid packets are valid by construction;
corruption tests then flip exactly one thing and assert rejection.
"""

import struct

import pytest

from mongla_sensors.sources.nucleus_parser import (
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


# =========================================================================== #
#  B14 -- the accumulator tests covered the ONE garbage case that cannot fail  #
# =========================================================================== #
# `test_accumulator_skips_leading_garbage` feeds b'\x00\x11\x22': no 0xa5 in it,
# so it exercises only the find(0xa5) skip -- the easy path. Both real defects
# (B02 spin-forever, B03 swallow-the-next-packet) need a 0xa5 INSIDE the garbage,
# which is the only case where the unvalidated length field gets trusted. Three
# green accumulator tests, none touching the rule they exist to protect.
#
# B20 additions here too: a header shorter than the 10-byte minimum used to make
# `buf[:size_header - 2]` a NEGATIVE slice, silently checksumming a different
# span; and a truncated payload was rejected only incidentally, by StructError.

import struct as _struct
from itertools import zip_longest as _zl

from mongla_sensors.sources.nucleus_parser import (
    ID_AHRS, ID_BOTTOMTRACK, PacketAccumulator, parse_packet,
)


def _b14_cs(data):
    c = 0xb58c
    for u, v in _zl(data[::2], data[1::2], fillvalue=0):
        c = (c + (int(u) | (int(v) << 8))) & 0xFFFF
    return c


def _b14_frame(pkt_id, raw):
    h = bytearray(10)
    h[0], h[1], h[2], h[3] = 0xa5, 10, pkt_id, 0x20
    h[4:6] = _struct.pack('<H', len(raw))
    h[6:8] = _struct.pack('<H', _b14_cs(raw))
    h[8:10] = _struct.pack('<H', _b14_cs(h[:8]))
    return bytes(h + bytearray(raw))


def _b14_ahrs(heading):
    a = bytearray(20)
    a[1] = 2
    a[2:6] = _struct.pack('<f', 1.0)
    a[6:10] = _struct.pack('<f', 2.0)
    a[10:14] = _struct.pack('<f', heading)
    return _b14_frame(ID_AHRS, bytes(a))


def test_a_FALSE_sync_byte_does_not_eat_the_next_real_packet():
    """B03's exact shape: 0xa5 inside garbage, with a plausible length field."""
    acc = PacketAccumulator()
    false_sync = bytes([0xa5, 60, 0x99, 0x20]) + bytes(6)     # claims 60+ bytes
    got = acc.feed(false_sync + _b14_ahrs(123.0) + _b14_ahrs(45.0))
    headings = [p['heading'] for p in got if p.get('id') == ID_AHRS]
    assert headings == [pytest.approx(123.0), pytest.approx(45.0)], (
        f'a false sync byte swallowed a real packet: got {headings}')


def test_a_zero_length_false_sync_does_not_spin_forever():
    """B02: size_header=0, size_data=0 -> total=0 -> `del buf[:0]` removed nothing."""
    acc = PacketAccumulator()
    got = acc.feed(bytes([0xa5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]) + _b14_ahrs(77.0))
    assert [p['heading'] for p in got if p.get('id') == ID_AHRS] == [pytest.approx(77.0)]


def test_a_header_shorter_than_the_minimum_is_rejected():
    """B20: size_header < 2 made `buf[:size_header - 2]` a negative slice."""
    for size_header in (0, 1, 2, 9):
        buf = bytearray(b'\xa5' + bytes([size_header]) + b'\xb4\x20' + bytes(30))
        assert parse_packet(buf) is None, f'size_header={size_header} was accepted'


def test_a_truncated_payload_is_rejected_by_a_length_check():
    """B20: slicing never raises IndexError, so this was caught only by accident."""
    assert parse_packet(bytearray(_frame(ID_BOTTOMTRACK, bytes(20)))) is None
    assert parse_packet(bytearray(_frame(ID_AHRS, bytes([0, 250] + [0] * 18)))) is None


def test_a_well_formed_packet_still_decodes():
    """The guards must not have made the parser reject real data."""
    raw = bytearray(108)
    raw[96:100] = _struct.pack('<f', 1.25)
    got = parse_packet(bytearray(_b14_frame(ID_BOTTOMTRACK, bytes(raw))))
    assert got is not None and got['velocity_x'] == pytest.approx(1.25)
    assert parse_packet(bytearray(_b14_ahrs(30.0)))['heading'] == pytest.approx(30.0)
