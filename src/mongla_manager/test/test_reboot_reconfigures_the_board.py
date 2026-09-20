"""B43 -- a board reboot aborted the command but left the board misconfigured.

`check_for_reboot()` is wired and aborts the active command. Its own message says
the board is "disarmed and no longer configured" -- and nothing reconfigured it.
The NEXT verb then ran on a board holding its COMPILED DEFAULTS, and both losses
are silent:

  * `JS_GAIN_DEFAULT` reverts to 0.5, so every MANUAL_CONTROL -- every vision
    align, every arrival brake -- runs at HALF authority. The hull simply
    corrects more weakly; nothing reports it.
  * stream rates revert to defaults, so ATTITUDE drops ~55 Hz -> ~11 Hz and the
    vision loop's freshness decay bleeds translational authority on a link that
    looks perfectly healthy.

A reboot is not exotic here: opening the serial port reboots this board, so any
second process touching the device causes one -- and a brownout on a thruster
current spike is the in-water version.
"""

import ast
import inspect

import pytest

import mongla_manager.auv_manager_node as amn


def _reboot_branch_source():
    src = inspect.getsource(amn.AUVManagerNode._publish_srot_telemetry)
    return src


def test_the_reboot_branch_reconfigures_and_not_only_aborts():
    src = _reboot_branch_source()
    assert 'check_for_reboot' in src
    assert '_reapply_srot_config' in src, (
        'a detected reboot must RE-PUSH the config it cleared -- aborting the '
        'running command leaves the next one on compiled defaults (B43)')


def test_the_reconfigure_restores_both_things_a_reboot_loses():
    src = inspect.getsource(amn.AUVManagerNode._reapply_srot_config)
    assert 'set_message_rate' in src, 'stream rates revert on reboot'
    assert 'set_default_gain' in src, \
        'JS_GAIN_DEFAULT reverts to 0.5 -- MANUAL_CONTROL at half authority'


def test_a_failed_reconfigure_is_reported_as_an_ERROR_not_swallowed():
    """Half authority that nobody knows about is the whole defect."""
    src = inspect.getsource(amn.AUVManagerNode._reapply_srot_config)
    tree = ast.parse(src.lstrip())
    handlers = [n for n in ast.walk(tree) if isinstance(n, ast.ExceptHandler)]
    assert handlers, 'a reconfigure that raises must not kill the telemetry tick'
    for h in handlers:
        seg = ast.get_source_segment(src.lstrip(), h) or ''
        assert 'error' in seg.lower(), 'a swallowed reconfigure failure is silent again'
    assert 'HALF authority' in src or 'half authority' in src, \
        'say what the operator loses, not just that a call failed'


def test_it_does_not_rerun_the_diagnostic_round_trips():
    """Behaviour-rev and yaw-reference answers cannot change across a reboot of
    the same firmware; re-running them from a 2 Hz tick would add link traffic
    for no information."""
    src = inspect.getsource(amn.AUVManagerNode._reapply_srot_config)
    assert 'check_behaviour_rev' not in src
    assert 'check_yaw_reference' not in src
