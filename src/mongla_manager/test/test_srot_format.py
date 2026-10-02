#!/usr/bin/env python3
"""srot_format + srot_changes -- the rendering contract, pinned.

These guard two rules the hardware taught us:
  * MATCH THE BOARD -- a heading is 0..360 everywhere, because the SROT OLED,
    VFR_HUD.heading and /mongla/state all use 0..360.
  * ABSENCE IS NOT ZERO -- the board suppresses what it cannot stand behind.
"""
import math

import pytest

from mongla_manager import srot_format as sfmt
from mongla_manager import srot_changes
import sys as _sys_ct
import pathlib as _pl_ct
_sys_ct.path.insert(0, str(_pl_ct.Path(__file__).resolve().parents[3] / 'tools'))
from code_text import code_of, code_of_file  # noqa: E402  (issue #22)


# --------------------------------------------------------------------------- #
#  Heading -- THE display bug                                                  #
# --------------------------------------------------------------------------- #

def test_heading_matches_the_board_through_the_negative_half():
    """THE regression. The board's OLED and VFR_HUD.heading showed 197 while `connect`
    printed -162.23 -- the same angle, disagreeing by exactly 360, on adjacent lines.
    Anything negative must wrap."""
    assert sfmt.heading_deg(math.radians(-162.23)) == pytest.approx(197.77, abs=0.01)


def test_heading_matches_the_board_on_the_measured_value():
    """Measured on the bench 2026-08-03: ATTITUDE.yaw = 2.7953 rad, board OLED and
    VFR_HUD.heading both read 160."""
    assert sfmt.heading_deg(2.7953) == pytest.approx(160.16, abs=0.01)


@pytest.mark.parametrize('rad,expect', [
    (0.0, 0.0), (math.pi, 180.0), (-math.pi / 2, 270.0), (math.pi / 2, 90.0),
])
def test_heading_is_always_in_0_360(rad, expect):
    assert sfmt.heading_deg(rad) == pytest.approx(expect, abs=0.01)


def test_roll_and_pitch_stay_signed():
    """Roll/pitch are genuinely +/- -- a 3 deg list to port is not a 357 deg list, and
    CAL_LVL_R/P are signed radians. Only the heading wraps."""
    assert sfmt.signed_deg(math.radians(-3.0)) == pytest.approx(-3.0)


def test_absent_attitude_is_absent_not_zero_degrees():
    assert sfmt.heading_deg(None) is None
    assert sfmt.heading_deg(float('nan')) is None
    assert sfmt.signed_deg(None) is None


# --------------------------------------------------------------------------- #
#  Absence is not zero                                                        #
# --------------------------------------------------------------------------- #

def test_fmt_renders_absence_as_a_dash():
    assert sfmt.fmt(None) == '--'
    assert sfmt.fmt(float('nan')) == '--'
    assert sfmt.fmt(1.5, '{:.1f}', ' V') == '1.5 V'


def test_the_mavlink_no_data_sentinels_are_not_readings():
    """UINT16_MAX volts and -1 amps are MAVLink's "not provided", not measurements.
    Rendering them literally reports a 65 V pack and a -0.01 A draw."""
    assert sfmt.mv_to_volts(0xFFFF) is None
    assert sfmt.mv_to_volts(0) is None
    assert sfmt.mv_to_volts(13951) == pytest.approx(13.951)
    assert sfmt.centiamps_to_amps(-1) is None
    assert sfmt.centiamps_to_amps(250) == pytest.approx(2.5)


def test_imu_temperature_zero_means_not_provided():
    """SCALED_IMU2.temperature sends 0 as MAVLink's "not provided" sentinel, and the
    board sends exactly that when the baro is unhealthy. Reading it as a temperature
    puts a freezing hull on the screen of a vehicle in a warm room."""
    assert sfmt.imu_temp_c(0) is None
    assert sfmt.imu_temp_c(2863) == pytest.approx(28.63)


def test_an_all_zero_esc_row_is_absent_not_eight_idle_escs():
    """BARE-BOARD BENCH, 2026-08-03: no Pico, no ESCs. `0 0 0 0 0 0 0 0` is what an
    attached-but-idle ESC sends AND what arrives with nothing connected; an ESC
    reporting 0 C is implausible, so an all-zero temp row means nothing is there."""
    assert sfmt.esc_temp_row([0] * 8) == '--'
    assert sfmt.esc_temp_row([]) == '--'
    assert '31' in sfmt.esc_temp_row([31, 32, 33, 34])


def test_rpm_row_keeps_real_zeros_but_reports_absence():
    """Unlike temperature, 0 RPM IS a legitimate reading for a stopped ESC -- so an
    all-zero RPM row is rendered, and only a MISSING frame is `--`."""
    assert sfmt.rpm_row([]) == '--'
    assert sfmt.rpm_row([0] * 8).split() == ['0'] * 8


def test_leak_never_reported_is_not_the_same_as_dry():
    """"No leak reported" and "we never heard from the leak sensor" are different
    facts and only one of them means the hull is dry."""
    assert sfmt.bool_word(None, 'WET', 'dry') == '--'
    assert sfmt.bool_word(False, 'WET', 'dry') == 'dry'
    assert sfmt.bool_word(True, 'WET', 'dry') == 'WET'


# --------------------------------------------------------------------------- #
#  Change detection                                                           #
# --------------------------------------------------------------------------- #

def test_the_first_snapshot_reports_nothing():
    """Otherwise every field "changes" at startup and the first screen is pure noise."""
    assert srot_changes.diff(None, {'mode': 'MANUAL'}) == []


def test_a_mode_flip_is_reported():
    out = srot_changes.diff({'mode': 'MANUAL'}, {'mode': 'AUTO'})
    assert len(out) == 1 and 'MANUAL' in out[0][2] and 'AUTO' in out[0][2]


def test_a_leak_is_critical():
    lvl, _, _ = srot_changes.diff({'leak': False}, {'leak': True})[0]
    assert lvl == 'CRIT'


def test_a_field_going_absent_is_reported():
    """The board SUPPRESSES values it cannot stand behind, so a barometer that stops
    being reported is the board talking. A detector that only watches numbers move
    would never mention it -- the value just stops updating, which looks like steady."""
    out = srot_changes.diff({'water_temp_c': 21.0}, {'water_temp_c': None})
    assert len(out) == 1 and 'STOPPED' in out[0][2]


def test_a_field_coming_back_is_reported():
    out = srot_changes.diff({'depth_m': None}, {'depth_m': -0.4})
    assert len(out) == 1 and 'started' in out[0][2]


def test_absent_on_both_sides_is_silent():
    assert srot_changes.diff({'water_temp_c': None}, {'water_temp_c': None}) == []


def test_noise_below_the_threshold_is_not_logged():
    """A log that fires on sensor noise is a log nobody reads."""
    assert srot_changes.diff({'heading_deg': 160.0}, {'heading_deg': 160.5}) == []


def test_heading_wrap_uses_the_shortest_arc():
    """359 -> 1 is two degrees of rotation, not 358. Without this a slow turn past
    north spams the log with a false 358-degree jump every tick."""
    # 359 -> 0.5 is half a degree of rotation. Naively it is 358.5 and would log.
    assert srot_changes.diff({'heading_deg': 359.0}, {'heading_deg': 0.5}) == []
    # 359 -> 20 is 21 degrees the short way, not 339.
    out = srot_changes.diff({'heading_deg': 359.0}, {'heading_deg': 20.0})
    assert len(out) == 1
    assert '21.00' in out[0][2] and 'shortest arc' in out[0][2]


# ── the version line, rendered through the formatter ─────────────────────────


def _render_board_line(av):
    """The first two lines of `connect`'s report, with one AUTOPILOT_VERSION."""
    from mongla_manager import srot_connect

    snap = srot_connect.Snapshot()
    if av is not None:
        snap.msgs['AUTOPILOT_VERSION'] = av
    return '\n'.join(srot_connect.render(snap, conn=None))


class _AV:
    def __init__(self, flight_sw_version, middleware_sw_version=14):
        self.flight_sw_version = flight_sw_version
        self.middleware_sw_version = middleware_sw_version


def test_an_unanswered_version_request_renders_dashes_not_v0_0_0():
    """⛔ THE DEFECT THIS CLOSES. AUTOPILOT_VERSION is one-shot and must be
    ASKED for, so an unanswered request is the ordinary case for the first
    second of every session. `render` unpacked the bit-fields inline, so a
    missing message left the packed word at 0 and the shifts turned it into
    `Hengla v0.0.0` -- a version number that reads as a real board running
    very old firmware, which is exactly the reading that matters most here
    because the behaviour rev is a hull-safety interlock.

    `srot_format.fw_version` was written for this line and returns `--` for 0.
    It was never called from it."""
    out = _render_board_line(None)
    assert 'v0.0.0' not in out
    assert 'Hengla --' in out


def test_a_real_version_still_renders(capsys):
    packed = (0 << 24) | (2 << 16) | (11 << 8)          # v0.2.11
    out = _render_board_line(_AV(packed))
    assert 'Hengla v0.2.11' in out
    assert 'behaviour rev 14' in out


def test_the_formatter_is_the_one_copy():
    """⛔ ONE TRUTH, TWO COPIES IS THE BUG. If the bit-shifts come back into
    the renderer the two will drift, and the suppression above is lost again."""
    from pathlib import Path

    src = code_of_file(Path(__file__).resolve().parents[1] / 'mongla_manager'
           / 'srot_connect.py')
    assert 'sfmt.fw_version(' in src
    assert '>> 24' not in src, 'the renderer unpacks the version itself again'
