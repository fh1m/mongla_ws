"""Per-thruster health from two signals the board already sends.

Both were sitting on the wire unread:

  * ESC_STATUS (291) carries SIGNED rpm. Both repos record that pymavlink
    "silently discards" the message; it does not -- it returns a
    MAVLink_unknown carrying the whole frame. Signed matters because
    ESC_TELEMETRY_* is uint16 magnitude, which cannot tell a thruster turning
    the wrong way from one turning correctly.
  * Per-thruster PRESENCE exists in board state and reaches the wire only as
    English, once, at the first arm.

The rule both tests enforce: UNKNOWN is a distinct answer from OK. An ESC
without Bluejay reports nothing while its motor spins perfectly, so a gate that
graded silence as health would be worse than no gate.
"""
import struct
import sys
from pathlib import Path

import pytest
from pymavlink import mavutil

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from duburi_control.fc.srot_fc import SrotFC, _decode_esc_status   # noqa: E402


def esc_frame(index, rpm, corrupt=False, truncate=None):
    """A real ESC_STATUS frame: v2 header, payload, correct x25 CRC."""
    payload = struct.pack('<Q4i4f4fB', 1234, *rpm, *([0.0] * 8), index)
    if truncate:
        payload = payload[:truncate]
    hdr = struct.pack('<BBBBBBB', 0xFD, len(payload), 0, 0, 7, 1, 1) \
        + struct.pack('<I', 291)[:3]
    crc = mavutil.x25crc(hdr[1:] + payload)
    crc.accumulate_str(chr(10))
    frame = hdr + payload + struct.pack('<H', crc.crc ^ (0xFFFF if corrupt else 0))
    return frame


class _Msg:
    def __init__(self, buf):
        self._buf = buf

    def get_msgbuf(self):
        return self._buf


class _Master:
    def __init__(self, **msgs):
        self.mav = type('M', (), {'__getattr__': lambda s, n: (lambda *a, **k: None)})()
        self.messages = dict(msgs)


def _fc(**msgs):
    return SrotFC(_Master(**msgs), log=None)


def _text(t, sev=6):
    return type('S', (), {'text': t.encode(), 'severity': sev})()


# --------------------------------------------------------------------------- #
#  Signed RPM
# --------------------------------------------------------------------------- #
def test_signed_rpm_decodes_from_the_message_pymavlink_has_no_entry_for():
    """The whole premise. Negative values are the point: ESC_TELEMETRY_* is
    uint16 and cannot express a reversing thruster at all."""
    fc = _fc(UNKNOWN_291=_Msg(esc_frame(0, [1500, -1500, 900, -900])))
    assert fc.esc_status_rpm()[:4] == (1500, -1500, 900, -900)


def test_both_halves_of_the_hull_are_assembled():
    """The board sends two frames, index 0 and 4. pymavlink keeps one message
    per msgid, so only the last is in the slot -- which is why the reader has to
    be the thing that sees them, not a poller."""
    fc = _fc(UNKNOWN_291=_Msg(esc_frame(4, [10, 20, 30, 40])))
    got = fc.esc_status_rpm()
    assert got[4:] == (10, 20, 30, 40)
    assert got[:4] == (None, None, None, None), 'unseen thrusters must be None'


def test_a_corrupted_frame_is_refused_not_decoded():
    """THE test. pymavlink returns an UNKNOWN message BEFORE it validates the
    checksum, so a corrupt frame arrives looking exactly like a good one and
    would decode into plausible RPM. We check x25 with crc_extra=10 ourselves."""
    assert _decode_esc_status(esc_frame(0, [1500, 0, 0, 0], corrupt=True)) is None
    fc = _fc(UNKNOWN_291=_Msg(esc_frame(0, [1500, 0, 0, 0], corrupt=True)))
    assert fc.esc_status_rpm() is None


def test_a_v2_TRUNCATED_frame_still_decodes():
    """MAVLink v2 drops TRAILING ZERO bytes, so an idle hull's frame arrives
    short -- here 20 bytes instead of 57, because every rpm and both float
    blocks are zero. Padding back to full length is what recovers it, and it is
    mandatory: unpacking 57 bytes from a 20-byte buffer throws.

    Same trap as SYS_STATUS's extended health, and I got this test backwards
    first -- I asserted the frame should be REFUSED. It should not: this is the
    ordinary shape of a frame from a stationary vehicle, and refusing it would
    have thrown away every reading the moment the thrusters stopped. The guard
    against a genuinely bad frame is the CRC above, which covers the truncated
    length and therefore validates exactly what arrived.
    """
    got = _decode_esc_status(esc_frame(0, [0, 0, 0, 0], truncate=20))
    assert got == (0, [0, 0, 0, 0])


def test_no_message_is_absence_not_zeros():
    """A stopped thruster and a missing message are different facts."""
    assert _fc().esc_status_rpm() is None


# --------------------------------------------------------------------------- #
#  Presence, parsed from the board's own English
# --------------------------------------------------------------------------- #
def test_the_all_wired_summary_is_understood():
    fc = _fc()
    fc.note_statustext(_text('Thrusters wired: all 8'))
    present, lost = fc.esc_presence()
    assert present == set(range(1, 9)) and not lost


def test_a_partial_frame_names_the_missing_thrusters():
    fc = _fc()
    fc.note_statustext(_text('Thrusters wired: 1,2,4,5,6,7,8 (absent: 3)', 4))
    ok, why = fc.thruster_health()
    assert ok is False and '3' in why


def test_a_lost_thruster_is_caught_after_a_clean_start():
    """The LOST line is edge-triggered and sent once, so it has to be captured
    as it goes past -- there is no way to ask the board again."""
    fc = _fc()
    fc.note_statustext(_text('Thrusters wired: all 8'))
    fc.note_statustext(_text('Thruster 6 LOST telemetry', 3))
    ok, why = fc.thruster_health()
    assert ok is False and '6' in why


def test_silence_is_UNKNOWN_and_never_OK():
    """The load-bearing case. The summary is sent at the FIRST ARM, so a
    disarmed bench session legitimately has nothing -- and an ESC without
    Bluejay never reports at all while its motor spins perfectly. Grading that
    as healthy is worse than not checking."""
    ok, why = _fc().thruster_health()
    assert ok is None
    assert 'UNKNOWN' in why


def test_a_healthy_hull_reports_ok():
    fc = _fc()
    fc.note_statustext(_text('Thrusters wired: all 8'))
    ok, _ = fc.thruster_health()
    assert ok is True


# --------------------------------------------------------------------------- #
#  Stall / reversal, which needs the SIGN
# --------------------------------------------------------------------------- #
def test_a_thruster_turning_the_wrong_way_is_caught():
    """Unsigned magnitude cannot see this, and it is the fault that made every
    axis respond backwards on this hull in August."""
    fc = _fc(UNKNOWN_291=_Msg(esc_frame(0, [1500, -1500, 1500, 1500])))
    bad = fc.thruster_stalled(commanded={1: +1, 2: +1, 3: +1, 4: +1})
    assert [b[0] for b in bad] == [2]
    assert 'WRONG WAY' in bad[0][2]


def test_a_commanded_thruster_that_is_not_turning_is_caught():
    fc = _fc(UNKNOWN_291=_Msg(esc_frame(0, [1500, 0, 1500, 1500])))
    bad = fc.thruster_stalled(commanded={1: +1, 2: +1, 3: +1, 4: +1})
    assert [b[0] for b in bad] == [2] and 'not turning' in bad[0][2]


def test_an_uncommanded_thruster_at_rest_is_not_a_fault():
    fc = _fc(UNKNOWN_291=_Msg(esc_frame(0, [1500, 0, 0, 0])))
    assert fc.thruster_stalled(commanded={1: +1, 2: 0, 3: 0, 4: 0}) == []


def test_without_a_commanded_map_it_answers_None():
    """On an idle hull every thruster reads zero, so a bare list would flag all
    eight. None is the honest answer."""
    fc = _fc(UNKNOWN_291=_Msg(esc_frame(0, [0, 0, 0, 0])))
    assert fc.thruster_stalled() is None


# --------------------------------------------------------------------------- #
#  The boot burst
# --------------------------------------------------------------------------- #
def test_the_ring_keeps_the_whole_burst_not_just_the_last_line():
    """13 announcements arrive back to back at boot and pymavlink keeps one."""
    fc = _fc()
    for i in range(13):
        fc.note_statustext(_text(f'line {i}'))
    assert len(fc.statustext_log()) == 13


def test_a_defaults_reset_is_surfaced():
    """It silently returns JS_GAIN_DEFAULT to 0.5 (half authority on every
    translation) and LEAK_EN to 0 (leak failsafe AND pre-arm refusal off). The
    operator's only clue is one line inside that burst."""
    fc = _fc()
    fc.note_statustext(_text('RST: POWERON', 4))
    fc.note_statustext(_text('Params reset to build defaults', 4))
    fc.note_statustext(_text('CFG r14 FR=1 DIR=-1,1,1,1,1,1,1,-1'))
    warns = [t for _, t in fc.boot_warnings()]
    assert warns == ['Params reset to build defaults']


def test_an_nvs_reformat_is_surfaced():
    fc = _fc()
    fc.note_statustext(_text('NVS reformatted - params AND calibration lost', 2))
    assert fc.boot_warnings()


def test_an_ordinary_boot_raises_nothing():
    fc = _fc()
    for t in ('Hengla v0.2.0 ready', 'RST: POWERON', 'CAL loaded from NVS'):
        fc.note_statustext(_text(t))
    assert fc.boot_warnings() == []
