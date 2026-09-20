"""The SROT bring-up reads must happen AFTER the MAVLink reader thread starts.

WHY THIS TEST EXISTS. `check_behaviour_rev()` and `set_default_gain()` do not call
`recv_*` -- they send, then poll `master.messages`, which pymavlink only fills while
something else drains the link. They used to run inside `_setup_mavlink()`, where
nothing between `wait_heartbeat()` and `_setup_reader_and_warmup()` drains anything.
So both replies were parsed by no one and the interlock reported
`FW_BEHAVIOUR_REV_UNKNOWN` on EVERY startup.

Measured on the vehicle 2026-08-03, same link, same call: with no drain running it
returned UNKNOWN after 6.0 s of retries; with a drain thread running it returned
rev 4 in 0.1 s. Transport-independent -- it failed on direct USB serial too.

That failure mode is worse than a missing read: its warning claims the firmware may
predate rev 2, i.e. that MOVE_STOP coasts with no host-side brake. The one bring-up
check meant to catch an un-brakeable 20 kg hull cried wolf every time, which trains
an operator to ignore it -- and it is the same warning a genuinely too-old board
would produce.

This is an ordering invariant, so it is asserted against the source: a behavioural
test would need a live board or a fake that reproduces pymavlink's cache semantics,
and either would be a weaker guard than "the call is not back in the un-pumped
function".
"""

import inspect

from mongla_manager import auv_manager_node as amn


PUMPED_READS = ('check_behaviour_rev', 'set_default_gain')


def _src(fn):
    return inspect.getsource(fn)


def test_pumped_reads_are_not_in_setup_mavlink():
    """_setup_mavlink runs before any reader exists -- nothing that needs a reply."""
    src = _src(amn.AUVManagerNode._setup_mavlink)
    for name in PUMPED_READS:
        assert f'{name}(' not in src, (
            f'{name}() is back in _setup_mavlink(), where no thread drains the link, '
            f'so its reply lands in master.messages after the poll has given up. '
            f'Call it from _srot_preflight_reads() instead.')


def test_pumped_reads_live_in_the_preflight_helper():
    src = _src(amn.AUVManagerNode._srot_preflight_reads)
    for name in PUMPED_READS:
        assert f'{name}(' in src, f'{name}() should be called from _srot_preflight_reads()'


def test_preflight_runs_after_the_reader_thread_starts():
    """Ordering is the whole point: before .start() the helper is just as blind."""
    src = _src(amn.AUVManagerNode._setup_reader_and_warmup)
    assert '_srot_preflight_reads()' in src, (
        '_setup_reader_and_warmup() must invoke _srot_preflight_reads()')
    start_at = src.index('reader_thread.start()')
    preflight_at = src.index('_srot_preflight_reads()')
    assert start_at < preflight_at, (
        '_srot_preflight_reads() runs BEFORE reader_thread.start() -- nothing is '
        'draining the link yet, which is the exact bug this ordering fixes.')


def test_rate_pinning_may_stay_unpumped():
    """set_message_rate is fire-and-forget (reads no reply), so it needs no reader.

    Pinned here so a future 'tidy-up' that moves rate pinning into the preflight
    helper has to notice it is deliberately allowed to run early -- rates should be
    set as soon as possible, not after the warmup that depends on them.
    """
    src = _src(amn.AUVManagerNode._setup_mavlink)
    assert 'set_message_rate(' in src
