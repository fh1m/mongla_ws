"""`flare_order` operator tool: repeat until the VEHICLE acks, and say which exit it was."""
from types import SimpleNamespace

from mongla_control.fc import srot_protocol as sp
from mongla_manager.flare_order_send import send_until_acked, main


class _Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


class _Conn:
    """Answers with `replies[i]` on the i-th recv; advances the clock 0.2 s per recv."""

    def __init__(self, clock, replies):
        self.clock, self.replies, self.sent = clock, list(replies), []
        self.mav = SimpleNamespace(
            heartbeat_send=lambda *a: None,
            command_long_send=lambda *a: self.sent.append(a))

    def recv_match(self, **_):
        self.clock.t += 0.2
        return self.replies.pop(0) if self.replies else None


def _ack(result, command=sp.CMD_SROT_FLARE_ORDER):
    return SimpleNamespace(get_type=lambda: 'COMMAND_ACK', command=command, result=result)


def _run(replies, timeout=10.0):
    clock = _Clock()
    conn = _Conn(clock, replies)
    lines = []
    rc = send_until_acked(conn, ('red', 'blue', 'yellow'), 42, timeout,
                          clock=clock, out=lines.append)
    return rc, conn, lines


def test_accepted_exits_zero_and_sent_the_right_params():
    rc, conn, _ = _run([None] * 7 + [_ack(sp.ACK_ACCEPTED)])
    assert rc == 0
    _sys, _comp, cmd, _conf, *params = conn.sent[0]
    assert cmd == sp.CMD_SROT_FLARE_ORDER
    assert params == sp.flare_order_params(('red', 'blue', 'yellow'), 42)
    assert len(conn.sent) >= 2, 'it must repeat while no ACK has come back'


def test_an_ack_for_another_command_is_not_ours():
    rc, _, _ = _run([_ack(sp.ACK_ACCEPTED, command=sp.CMD_SROT_MOVE)], timeout=2.0)
    assert rc == 1


def test_denied_exits_two():
    assert _run([_ack(sp.ACK_DENIED)])[0] == 2


def test_silence_times_out_with_one():
    rc, conn, lines = _run([], timeout=3.0)
    assert rc == 1 and 'NO ACK' in lines[-1] and len(conn.sent) == 3


def test_bad_operator_text_never_opens_the_port(capsys):
    assert main(['R-R-B', '--port', '/dev/does-not-exist']) == 2
    assert 'bad order' in capsys.readouterr().out
