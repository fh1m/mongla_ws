"""Leak from the SYS_STATUS extended health bits, which pymavlink cannot parse.

We asked for this mechanism (TASKS_FROM_DUBURI_WS.md §3) because LEAK rides the
multiplexed NAMED_VALUE_FLOAT and pymavlink keeps one message per msgid, making
the observation of a flooding hull a lottery on arrival order. The firmware
delivered it in rev 3 and we never adopted it, because pymavlink 2.4.49's
SYS_STATUS schema has 13 fields and no extensions.

It parses them away; the bytes are still there. These tests pin the decode
against the layout in the firmware's own header, and in particular against
MAVLink v2's trailing-zero truncation -- measured on the live board, the payload
arrives at 40 bytes rather than its declared 43, with `health_extended` cut to a
single byte.
"""
import struct
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from duburi_control.fc import srot_protocol as sp     # noqa: E402
from duburi_control.fc.srot_fc import SrotFC          # noqa: E402

LEAK = sp.SYS_STATUS_SENSOR_LEAK


class _Msg:
    """A SYS_STATUS frame carrying a real v2 header and a chosen payload."""

    def __init__(self, payload: bytes):
        # v2 header: magic, len, incompat, compat, seq, sysid, compid, msgid(3)
        self._buf = bytes([0xFD, len(payload), 0, 0, 0, 1, 1, 1, 0, 0]) + payload

    def get_msgbuf(self):
        return self._buf


class _Master:
    def __init__(self, msg=None):
        self.mav = type('M', (), {'__getattr__': lambda s, n: (lambda *a, **k: None)})()
        self.messages = {} if msg is None else {'SYS_STATUS': msg}


def _payload(present, enabled, health, truncate_to=None):
    """A SYS_STATUS payload: 31 base bytes then three uint32 extension fields.

    `truncate_to` reproduces MAVLink v2 dropping trailing zero bytes, which is
    what the real board does and what a naive unpack trips over.
    """
    body = bytes(30) + b'\xff'                       # battery_remaining = -1 at 30
    body += struct.pack('<III', present, enabled, health)
    return body[:truncate_to] if truncate_to else body


def _fc(payload=None):
    return SrotFC(_Master(None if payload is None else _Msg(payload)), log=None)


def test_a_dry_hull_reads_false():
    """Health bit SET means healthy, i.e. dry. Inverting this reports a leak on
    every healthy vehicle, which trains the operator to ignore it."""
    assert _fc(_payload(LEAK, LEAK, LEAK)).sys_status_leak() is False


def test_a_leak_reads_true():
    """Health bit CLEAR is the leak. This is the whole point of the mechanism."""
    assert _fc(_payload(LEAK, LEAK, 0)).sys_status_leak() is True


def test_the_truncated_health_field_still_decodes():
    """THE CASE THAT ACTUALLY OCCURS. Measured on the live board: the payload
    arrives at 40 bytes, not its declared 43, because MAVLink v2 drops trailing
    zeros -- `health_extended` is cut to one byte. Without zero-padding, the
    unpack reads past the end and the leak read fails exactly when the payload
    is at its most ordinary."""
    p = _payload(LEAK, LEAK, LEAK, truncate_to=40)
    assert len(p) == 40
    assert _fc(p).sys_status_leak() is False        # health byte 0x02 survived

    # And a leak, where health is zero and therefore truncated away entirely.
    p2 = _payload(LEAK, LEAK, 0, truncate_to=39)
    assert _fc(p2).sys_status_leak() is True


def test_a_disabled_leak_sensor_is_absence_not_dryness():
    """LEAK_EN = 0 -- which is how the live board is configured -- disables the
    board's own leak failsafe AND its pre-arm refusal. Reporting False there
    would say "no leak" about a vehicle that is not looking for one, which is
    the confusion this mechanism exists to remove."""
    assert _fc(_payload(LEAK, 0, LEAK)).sys_status_leak() is None


def test_a_board_with_no_leak_sensor_reports_absence():
    assert _fc(_payload(0, 0, 0)).sys_status_leak() is None


def test_a_message_with_no_extension_bytes_reports_absence():
    """An older build, or one whose extension is entirely zero and truncated to
    the base length. Nothing to read is not the same as nothing wrong."""
    assert _fc(bytes(30) + b'\xff').sys_status_leak() is None


def test_no_sys_status_at_all_is_absence():
    assert _fc(None).sys_status_leak() is None


def test_the_offsets_match_the_firmware_header():
    """Pinned against `lib/mavlink/common/mavlink_msg_sys_status.h`: MIN_LEN 31
    is the base message and the accessors read the extended fields at 31 / 35 /
    39, giving LEN 43. A drift here silently decodes the wrong bytes."""
    from duburi_control.fc.srot_fc import (_SYS_STATUS_BASE_LEN,
                                           _SYS_STATUS_EXT_END)
    assert _SYS_STATUS_BASE_LEN == 31
    assert _SYS_STATUS_EXT_END == 43
    assert sp.SYS_STATUS_SENSOR_LEAK == 2
