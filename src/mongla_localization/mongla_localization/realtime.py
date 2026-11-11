"""Give a process that streams commands a realtime priority and a short lock.

Measured on Kutum with `tools/bench/wake_latency.py` (measured-bars 76): a
50 Hz Python loop beside a thread parsing a full-rate MAVLink stream, with
every core busy, wakes at worst 5.17 ms late under the default scheduler. Two
things cause that and neither is the kernel:

  * the scheduler lets a busy process on the same core run first (2.9 ms at
    p99.9); a realtime priority removes it;
  * the interpreter lock is handed over every 5 ms by default, so a thread
    that wakes while another holds it waits up to 5 ms whatever its priority;
    a 0.5 ms switch interval bounds that.

Together the worst wake-up is 0.56 ms on the stock kernel, so a PREEMPT_RT
kernel has nothing left to remove at a 20 ms period.

`MONGLA_REALTIME` chooses: `on` (the default) or `off`. A machine that does
not permit a realtime priority (a dev box, or a Kutum whose limits file is not
installed) keeps the short switch interval and says so once.

It lives in `mongla_localization` for the reason `params_file` does: every node
package already depends on it.
"""
from __future__ import annotations

import os
import sys

ENV = 'MONGLA_REALTIME'
CHOICES = ('on', 'off')
#: Below the kernel's own interrupt threads (50), above every ordinary process.
PRIORITY = 10
SWITCH_INTERVAL_S = 0.0005


def choice(env=None) -> str:
    """What was asked for; a value that is not a choice is refused."""
    value = (os.environ if env is None else env).get(ENV, 'on').strip().lower()
    if value not in CHOICES:
        raise ValueError(f'{ENV}={value!r}: expected one of {", ".join(CHOICES)}')
    return value


def apply(logger=None, env=None, setscheduler=None) -> str:
    """Apply it to this process; returns what took effect, as one line.

    Call it before the process starts its worker threads: a thread inherits
    the scheduling class of the thread that creates it.
    """
    if choice(env) == 'off':
        return 'off (MONGLA_REALTIME=off): default scheduler, 5 ms switch interval'
    sys.setswitchinterval(SWITCH_INTERVAL_S)
    setscheduler = setscheduler or getattr(os, 'sched_setscheduler', None)
    try:
        setscheduler(0, os.SCHED_FIFO, os.sched_param(PRIORITY))
    except (PermissionError, OSError, TypeError, AttributeError) as exc:
        if logger is not None:
            logger.warning(
                f'[RT   ] no realtime priority ({exc!r}): this user may not ask '
                f'for one. On Kutum, `setup.sh --system` installs '
                f'/etc/security/limits.d/99-mongla-realtime.conf; log in again '
                f'after it. Worst wake-up stays about 3 ms instead of 0.6 ms.')
        return (f'switch interval {SWITCH_INTERVAL_S * 1e3:.1f} ms, default '
                f'scheduler (realtime priority not permitted)')
    return (f'SCHED_FIFO {PRIORITY}, switch interval '
            f'{SWITCH_INTERVAL_S * 1e3:.1f} ms')
