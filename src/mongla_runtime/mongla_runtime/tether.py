"""Does this process die when the tether is pulled?

A run is tetherless: the operator starts the stack and the mission over the
tether, counts down, and pulls it. A process started from an ssh session,
outside tmux or screen, with that session's terminal as its controlling
terminal, gets SIGHUP when the tether goes and stops. The manager stopping
means the board's link-loss failsafe surfaces the hull: safe, and the run is
lost. Vision stopping means a mission steering blind.

`spin()` declares the answer on every node as the read-only parameter
`tether_bound` ('' = survives; otherwise why not) and warns at start-up;
`countdown()` asks every Mongla node and refuses while any one is bound.
"""
from __future__ import annotations

import os
import signal

PARAM = 'tether_bound'


def _has_controlling_tty() -> bool:
    try:
        fd = os.open('/dev/tty', os.O_RDONLY | os.O_NOCTTY)
    except OSError:
        return False
    os.close(fd)
    return True


def tether_bound(env=None, *, has_tty=None, sighup_ignored=None) -> str:
    """Why this process would stop when the tether is pulled, or ''."""
    env = os.environ if env is None else env
    if not env.get('SSH_CONNECTION'):
        return ''                                 # local console, or systemd
    if env.get('TMUX') or env.get('STY'):
        return ''                                 # tmux / screen keep it
    if sighup_ignored is None:
        sighup_ignored = signal.getsignal(signal.SIGHUP) is signal.SIG_IGN
    if sighup_ignored:
        return ''                                 # nohup
    if has_tty is None:
        has_tty = _has_controlling_tty()
    if not has_tty:
        return ''                                 # setsid: no terminal to hang up
    return ('started from an ssh session outside tmux or screen: pulling the '
            'tether hangs it up. Start the stack in tmux (tmux new -s run)')


def declare(node) -> str:
    """Declare `tether_bound` on `node` (read-only) and warn when bound."""
    from rcl_interfaces.msg import ParameterDescriptor
    why = tether_bound()
    if not hasattr(node, 'declare_parameter'):     # a stand-in node in a test
        return why
    if not node.has_parameter(PARAM):
        node.declare_parameter(PARAM, why, ParameterDescriptor(
            read_only=True, description='why this process stops when the tether is '
                                        "pulled; '' = it survives"))
    if why:
        node.get_logger().warning(f'[TETH ] {node.get_name()}: {why}')
    return why
