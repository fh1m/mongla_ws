"""The link meter says what is arriving now, and keeps absence absent."""
import math

from mongla_manager.link_meter import LINE_BYTES_PER_S, LinkMeter


class Frame:
    def __init__(self, kind, nbytes, **fields):
        self.kind, self.nbytes = kind, nbytes
        self.__dict__.update(fields)

    def get_type(self):
        return self.kind

    def get_msgbuf(self):
        if self.nbytes is None:
            raise AttributeError('a bad frame has no buffer')
        return b'x' * self.nbytes


def test_rates_and_bytes_over_the_window():
    m = LinkMeter(window_s=2.0)
    for i in range(100):                       # 50 Hz of 40-byte frames for 2 s
        m.note(Frame('ATTITUDE', 40), i * 0.02)
    m.note(Frame('HEARTBEAT', 21), 1.5)
    bps, rates = m.read(1.99)
    assert rates == {'ATTITUDE': 50.0, 'HEARTBEAT': 0.5}
    assert bps == (100 * 40 + 21) / 2.0


def test_a_stream_that_stopped_reads_zero_not_its_average():
    m = LinkMeter(window_s=2.0)
    for i in range(100):
        m.note(Frame('ATTITUDE', 40), i * 0.02)
    assert m.read(10.0) == (0.0, {})


def test_the_boards_own_counters_are_absent_until_sys_status_arrives():
    m = LinkMeter()
    assert math.isnan(m.drop_pct) and math.isnan(m.errors)
    m.note(Frame('SYS_STATUS', 43, drop_rate_comm=250, errors_comm=3), 0.0)
    assert m.drop_pct == 2.5 and m.errors == 3.0


def test_a_frame_with_no_buffer_counts_and_never_raises():
    m = LinkMeter(window_s=1.0)
    m.note(Frame('BAD_DATA', None), 0.0)       # the reader thread must not die here
    assert m.read(0.5) == (0.0, {'BAD_DATA': 1.0})


def test_the_line_is_115200_baud_8n1():
    assert LINE_BYTES_PER_S == 11520
