"""The preflight firmware-behaviour-rev gate.

This grades the one number that decides whether `stop` decelerates 20 kg of hull.
Below `FW_BEHAVIOUR_REV_REQUIRED` the board's MOVE_STOP applies zero braking thrust
and this host no longer carries the reverse-leg brake that used to cover it -- so a
misgrade here is a silent failure in the water, not an error anyone would see.

`_behaviour_rev_verdict` is pure precisely so this can be tested without a board.
"""

import pytest

from duburi_manager.bringup_check import _behaviour_rev_verdict, PASS, WARN, FAIL
from duburi_control.fc import srot_protocol as sp


def _status(rev, required=2):
    return _behaviour_rev_verdict(rev, required)[0]


def test_current_firmware_passes():
    assert _status(2) == PASS
    assert _status(3) == PASS          # a newer board is fine, not "unexpected"


def test_old_firmware_fails_closed():
    """A board that ANSWERS with a too-old rev is a definite statement: stop coasts."""
    assert _status(1) == FAIL


def test_zero_is_too_old_not_unknown():
    """0 is what pre-2026-08-01 firmware reports -- that build never populated the
    field. Reading it as "unknown" and warning through would be exactly backwards."""
    assert _status(0) == FAIL


def test_silent_board_warns_but_does_not_fail():
    """Asymmetric on purpose: no answer is far more likely a dropped frame than a
    genuine old board, and failing a whole preflight on a comms hiccup is its own
    hazard. It must still be loud."""
    status, label, detail = _behaviour_rev_verdict(None, 2)
    assert status == WARN
    assert 'COAST' in detail.upper()   # the operator is told what they are risking


def test_failure_detail_names_the_consequence_and_the_fix():
    """A preflight line nobody understands gets overridden. Say what breaks and how
    to fix it, in the line itself -- not in a doc the operator is not reading."""
    _, _, detail = _behaviour_rev_verdict(1, 2)
    assert 'decelerate' in detail.lower()
    assert 'flash' in detail.lower()


def test_grades_against_the_shipped_requirement():
    """Guards the wiring, not the helper: the real call site must pass the pinned
    constant, so bumping FW_BEHAVIOUR_REV_REQUIRED tightens the preflight too."""
    required = sp.FW_BEHAVIOUR_REV_REQUIRED
    assert _status(required, required) == PASS
    assert _status(required - 1, required) == FAIL


@pytest.mark.parametrize('rev', [0, 1])
def test_no_too_old_revision_is_ever_a_mere_warning(rev):
    """The whole point of the gate: a known-old board must not be waved through."""
    assert _behaviour_rev_verdict(rev, 2)[0] == FAIL


# --------------------------------------------------------------------------- #
#  Bar30 health -- since fw rev 3 this decides whether ANY move verb runs      #
# --------------------------------------------------------------------------- #

from duburi_manager.bringup_check import _baro_health_verdict          # noqa: E402
from pymavlink import mavutil                                          # noqa: E402

_BARO = mavutil.mavlink.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE
_OTHER = mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_GYRO


def test_healthy_baro_passes():
    status, label, _ = _baro_health_verdict(_BARO | _OTHER, _BARO | _OTHER)
    assert status == PASS and 'Bar30' in label


def test_unhealthy_baro_fails_because_every_move_verb_is_denied():
    """Present but unhealthy is a definite statement: the board refuses AUTO, and
    SROT_MOVE enters AUTO, so move_forward is denied too. FAIL, not WARN -- finding
    this on the bench is the entire point of the line."""
    status, _, detail = _baro_health_verdict(_OTHER, _BARO | _OTHER)
    assert status == FAIL
    assert 'DENIED' in detail or 'denied' in detail


def test_absent_sys_status_warns_rather_than_failing():
    """Same asymmetry as the behaviour-rev check: silence is far more likely a dropped
    frame than a dead sensor, and failing a preflight on a comms hiccup is its own hazard."""
    assert _baro_health_verdict(None, None)[0] == WARN


def test_baro_not_present_warns():
    assert _baro_health_verdict(_OTHER, _OTHER)[0] == WARN


# --------------------------------------------------------------------------- #
#  Barometer NOISE + a saturated depth loop -- both measured on the vehicle    #
# --------------------------------------------------------------------------- #

from duburi_manager.bringup_check import (                            # noqa: E402
    _baro_noise_verdict, _depth_loop_verdict)


def test_a_noisy_baro_fails_even_though_every_sample_is_plausible():
    """THE finding, 2026-08-02. On the bench, still, the Bar30 produced samples
    spanning 321..740 mbar. Each one sits inside the firmware's wide per-sample
    plausibility band, so the board reports the barometer HEALTHY and keeps streaming
    SCALED_PRESSURE2 -- while depth wanders metres. Only variance sees this."""
    real = [321.4, 740.2, 519.3, 413.8, 623.5, 896.1]
    status, label, detail = _baro_noise_verdict(real)
    assert status == FAIL
    assert 'NOISE' in label or 'NOISE' in detail
    assert 'HEALTHY' in detail, 'must warn that the board disagrees'


def test_a_still_baro_passes():
    assert _baro_noise_verdict([1013.1, 1013.2, 1013.0, 1013.15])[0] == PASS


def test_a_stable_but_wrong_baro_still_fails():
    """A rock-steady 465 mbar is not noise, but depth derived from it is still
    wrong by metres -- the variance check alone would wave it through."""
    assert _baro_noise_verdict([465.0, 465.1, 465.0, 465.05])[0] == FAIL


def test_absent_pressure_warns_and_says_depth_is_untrustworthy():
    """Since rev 3 absence means the board WITHHELD it (unhealthy/stale baro), which
    is information, not a comms hiccup -- but a dropped frame looks identical, so
    WARN rather than FAIL and say what it implies."""
    status, _, detail = _baro_noise_verdict([])
    assert status == WARN and 'trustworthy' in detail


def _cmd(depth_m, gain=None):
    """The DEPTH_CMD a board at `depth_m` would emit: clamp(DEPTH_P * (depth - 0.10)).

    Fixtures are written in METRES and converted here, deliberately. Hand-typed
    wire values silently encode whatever DEPTH_P was current when they were
    written, and when the firmware retuned the gain (f533bd2, 3.0 -> 0.5 after
    the 2026-08-07 water test) every one of them started describing a different
    physical depth -- a "healthy surface reading" fixture became a 0.44 m error.
    Three test files carried that rot at once. Stating the metres makes the next
    retune a no-op here.
    """
    g = sp.DEPTH_P_DEFAULT if gain is None else gain
    return max(-1.0, min(1.0, g * (depth_m - 0.10)))


def test_an_implausible_barometer_fails_and_names_the_vertical_thrusters():
    """Observed disarmed 2026-08-02: a phantom baro reading -3.1 m at the surface.
    DEPTH_CMD is clamp(DEPTH_P * (depth - 0.10)) so that pins at -1.00. The mixer
    throttle column is -1 on all four verticals and 0 on all four horizontals, so
    arming turns it into full vertical thrust -- the reported arming blocker."""
    status, label, detail = _depth_loop_verdict(-1.0)
    assert status == FAIL
    assert 'IMPLAUSIBLE' in label and 'vertical' in detail
    assert 'SATURATED' in detail, 'a clamped reading must be named as clamped'


def test_the_preflight_reads_depth_cmd_because_depth_out_is_gone_at_rev_8():
    """REGRESSION. fw rev 8 suppresses DEPTH_OUT while the loop is not running, and
    this probe only ever runs disarmed -- so reading DEPTH_OUT hit the 'not reported'
    WARN branch on a perfectly healthy board and the check silently stopped working.
    A healthy surface reading must PASS, and it must do so from DEPTH_CMD alone."""
    status, _, _ = _depth_loop_verdict(_cmd(0.00))     # a board reading exactly 0 m
    assert status == PASS


def test_the_preflight_threshold_follows_depth_p():
    """DEPTH_CMD is a clamped OUTPUT, so a fixed limit means a different physical
    depth once the gain is retuned. Same reading, two gains, opposite verdicts."""
    assert _depth_loop_verdict(0.5, 1.0)[0] == FAIL     # 0.50 + 0.10 = 0.60 m
    assert _depth_loop_verdict(0.5, 10.0)[0] == PASS    # 0.05 + 0.10 = 0.15 m


def test_absent_depth_cmd_warns_that_the_board_will_refuse_auto():
    """DEPTH_CMD is gated on depth_ok, so absence is the board declaring the baro
    unhealthy -- which makes it refuse DEPTH_HOLD/AUTO, and since SROT_MOVE enters
    AUTO, every move verb with it. Say that, rather than 'check unavailable'."""
    status, _, detail = _depth_loop_verdict(None)
    assert status == WARN and 'AUTO' in detail


def test_only_a_locked_yaw_reference_passes():
    """Only LOCKED means ATTITUDE.yaw is a magnetic heading. Anything else and an
    absolute turn aims at a boot-relative number -- silently, since the move
    completes normally on the wrong bearing."""
    from duburi_manager.bringup_check import _yaw_ref_verdict
    import duburi_control.fc.srot_protocol as sp
    assert _yaw_ref_verdict(float(sp.YAW_REF_LOCKED))[0] == PASS
    for bad in (sp.YAW_REF_IDLE, sp.YAW_REF_SAMPLING, sp.YAW_REF_REFUSED_CAL,
                sp.YAW_REF_REFUSED_FIELD, sp.YAW_REF_REFUSED_NOISE):
        status, _, detail = _yaw_ref_verdict(float(bad))
        assert status == WARN, f'state {bad} must not pass'
        assert 'relative' in detail
    # fw < 9 has no YAW_REF: unknown, not assumed either way.
    assert _yaw_ref_verdict(None)[0] == WARN


def test_a_board_that_never_reports_depth_cmd_only_warns():
    """Firmware older than rev 3 has no DEPTH_OUT. Silence must not become a hard
    preflight failure for a value the board cannot produce."""
    assert _depth_loop_verdict(None, None)[0] == WARN


def test_the_reader_thread_starts_before_the_payload_role_read():
    """preflight_roles() reads params out of the pymavlink cache and never calls
    recv_match() itself. With no reader running, every role comes back None and the
    payload reports UNREADABLE -- indistinguishable from a mis-roled board. Pin the
    ordering so a future refactor cannot silently disable the payload."""
    import inspect
    from duburi_manager import auv_manager_node as amn
    src = inspect.getsource(amn.AUVManagerNode.__init__)
    assert (src.index('_setup_reader_and_warmup')
            < src.index('_preflight_payload')), \
        'the reader thread must start before the payload roles are read'


# --------------------------------------------------------------------------- #
# GCS-failsafe scope (bench mode saved to flash)
# --------------------------------------------------------------------------- #
def test_wildcarded_gcs_failsafe_is_a_hard_fail():
    """MEASURED on the vehicle 2026-08-07: FS_GCS_ENABLE=1, FS_GCS_COMPID=0, and
    `bringup_check --srot` reported 0 FAIL. Bench mode had been saved to flash.

    0 is the wildcard, so any station -- Bondor on the bench -- keeps the failsafe fed.
    In water a dead Jetson then looks like a live GCS and the hull station-keeps
    instead of surfacing. That is the failure this line exists to catch."""
    from duburi_manager.bringup_check import _gcs_failsafe_verdict
    status, label, detail = _gcs_failsafe_verdict(1.0, 0.0)
    assert status == FAIL
    assert 'bench' in label.lower() or 'wildcard' in label.lower()
    assert '191' in detail, 'the fix (set it to our compid) must be in the message'


def test_failsafe_scoped_to_our_compid_passes():
    from duburi_manager.bringup_check import _gcs_failsafe_verdict
    import duburi_control.fc.srot_protocol as sp
    assert _gcs_failsafe_verdict(1.0, float(sp.SOURCE_COMPID))[0] == PASS


def test_a_disabled_failsafe_is_not_silently_a_pass():
    """FS_GCS_ENABLE=0 makes the compid moot, so the scope check would pass
    vacuously -- but nothing surfaces the vehicle either. Report it separately."""
    from duburi_manager.bringup_check import _gcs_failsafe_verdict
    assert _gcs_failsafe_verdict(0.0, 191.0)[0] == WARN


def test_unreadable_params_warn_rather_than_pass():
    """A bridge that drops the reply must never read as 'scoped correctly'."""
    from duburi_manager.bringup_check import _gcs_failsafe_verdict
    assert _gcs_failsafe_verdict(None, None)[0] == WARN
    assert _gcs_failsafe_verdict(1.0, None)[0] == WARN


def test_a_healthy_in_air_board_does_not_cry_wolf():
    """MEASURED 2026-08-07: a healthy board with ~0.15 m of baro offset read
    DEPTH_CMD = -0.74. Judged as raw error that is -0.25 m, five centimetres from a
    FAIL it does not deserve -- because the preview's fixed 0.10 m target is baked in
    and spends a third of the budget before the barometer says anything.

    Subtracting the target recovers the board's actual depth (-0.15 m), which is a
    WARN worth a calibrate_depth and nothing more. A guard that fires on a healthy
    vehicle is a guard that gets overridden by habit."""
    status, _, detail = _depth_loop_verdict(_cmd(-0.15))
    assert status == PASS, 'a 0.15 m offset in air is normal, not a warning'
    assert '-0.15' in detail


def test_a_real_offset_warns_and_names_the_fix():
    """Between the healthy band and the refusal there is a real offset worth acting
    on. It must name calibrate_depth -- a WARN nobody knows how to clear is noise."""
    status, _, detail = _depth_loop_verdict(_cmd(0.25))   # +0.25 m of offset
    assert status == WARN and 'calibrate_depth' in detail


def test_saturation_points_at_the_offset_before_the_sensor():
    """Saturation is refused whatever its cause -- saturation IS the hazard. But a
    large zero offset is far more common than a dead Bar30, and the deck fix differs,
    so the refusal must name calibrate_depth rather than sending someone hunting a
    hardware fault."""
    _, _, detail = _depth_loop_verdict(-1.0)
    assert 'calibrate_depth' in detail and 'VARIANCE' in detail


def test_a_perfectly_zeroed_board_passes_cleanly():
    """DEPTH_CMD = -0.30 at DEPTH_P = 3.0 is exactly 0.00 m. If this ever WARNs, the
    target-offset correction has been lost again."""
    assert _depth_loop_verdict(_cmd(0.00))[0] == PASS
