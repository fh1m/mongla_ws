"""`tether_bound`: which processes stop when the tether is pulled."""
import pytest

from mongla_runtime.tether import tether_bound

SSH = {'SSH_CONNECTION': '10.0.0.2 5000 10.0.0.3 22'}


@pytest.mark.parametrize('env,tty,ign,bound', [
    (SSH, True, False, True),                          # bare interactive ssh
    ({}, True, False, False),                          # local console / systemd
    ({**SSH, 'TMUX': '/tmp/tmux-1/default,1,0'}, True, False, False),
    ({**SSH, 'STY': '123.pts-0.kutum'}, True, False, False),
    (SSH, False, False, False),                        # setsid: no terminal
    (SSH, True, True, False),                          # nohup
])
def test_tether_bound(env, tty, ign, bound):
    why = tether_bound(env, has_tty=tty, sighup_ignored=ign)
    assert bool(why) is bound
    if bound:
        assert 'tmux' in why


def test_every_node_declares_it_when_it_spins():
    import pathlib
    src = (pathlib.Path(__file__).resolve().parents[1] / 'mongla_runtime' / 'spin.py').read_text()
    assert 'declare(n)' in src
