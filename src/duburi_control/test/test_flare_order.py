"""The flare order: host encoder == firmware validator, and absence reads as None.

The order crosses three boundaries (operator text -> COMMAND_LONG params -> board RAM ->
NAMED_VALUE_FLOAT digits -> colour names). Each test below pins one of them. The
firmware cross-check COMPILES fw src/comms/flare_order.h and runs its pack() on the
same inputs, so the two validators cannot silently disagree about what is an order.
"""
import shutil
import subprocess
import time
from pathlib import Path

import pytest

from duburi_control.fc import srot_protocol as sp
from duburi_control.fc.srot_fc import SrotFC

from test_srot_protocol_drift import _FW   # the one firmware locator


# --------------------------------------------------------------------------- #
#  Host codec                                                                  #
# --------------------------------------------------------------------------- #

def test_params_and_digits_round_trip():
    p = sp.flare_order_params(('red', 'blue', 'yellow'), 7)
    assert p == [1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 7.0]
    assert sp.flare_order_from_digits(123.0) == ('red', 'blue', 'yellow')


@pytest.mark.parametrize('text', ['R-B-Y', 'RBY', 'red,blue,yellow', 'r b y',
                                  'Red > Blue > Yellow'])
def test_operator_text_forms(text):
    assert sp.parse_flare_order(text) == ('red', 'blue', 'yellow')


@pytest.mark.parametrize('colours,nonce', [
    ((), 1), (('red', 'red'), 1), (('mauve',), 1),
    (('red', 'blue', 'yellow', 'green', 'white'), 1),
    (('red',), -1), (('red',), 65536), (('red',), 1.5)])
def test_the_host_refuses_what_the_board_refuses(colours, nonce):
    with pytest.raises(ValueError):
        sp.flare_order_params(colours, nonce)


@pytest.mark.parametrize('value', [None, float('nan'), 0.0, -12.0, 12.5, 112.0,
                                   10.0, 12345.0])
def test_digits_that_are_not_an_order_decode_to_None(value):
    assert sp.flare_order_from_digits(value) is None


# --------------------------------------------------------------------------- #
#  Firmware agreement                                                          #
# --------------------------------------------------------------------------- #

_HDR = None if _FW is None else _FW / 'src' / 'comms' / 'flare_order.h'
needs_fw = pytest.mark.skipif(
    _HDR is None or not _HDR.is_file(),
    reason='fw flare_order.h not checked out (PR not merged / other branch)')

_CASES = [  # p1..p4, nonce
    (1, 2, 3, 0, 7), (3, 1, 2, 0, 0), (1, 2, 3, 4, 65535), (9, 0, 0, 0, 1),
    (0, 0, 0, 0, 1), (1, 0, 2, 0, 1), (1, 1, 2, 0, 1), (1.5, 2, 3, 0, 1),
    (10, 2, 3, 0, 1), (-1, 2, 3, 0, 1), (1, 2, 3, 0, -1), (1, 2, 3, 0, 65536),
    (1, 2, 3, 0, 2.5)]


def _host_digits(codes, nonce) -> int:
    """What the host would send for raw codes: 0 if flare_order_params refuses."""
    by_code = {v: k for k, v in sp.FLARE_COLOURS.items()}
    used = []
    for x in codes:
        if x == 0:
            break
        if x not in by_code:
            return 0
        used.append(by_code[x])
    if any(x != 0 for x in codes[len(used):]):
        return 0                                       # a code after the end marker
    try:
        params = sp.flare_order_params(used, nonce)
    except ValueError:
        return 0
    return int(''.join(str(int(v)) for v in params[:sp.FLARE_MAX_SLOTS] if v))


@needs_fw
def test_command_id_matches_the_firmware():
    text = _HDR.read_text()
    assert f'#define MAV_CMD_SROT_FLARE_ORDER {sp.CMD_SROT_FLARE_ORDER}' in text
    assert f'MAX_SLOTS = {sp.FLARE_MAX_SLOTS}' in text


@needs_fw
def test_the_named_values_are_emitted():
    src = (_FW / 'src' / 'comms' / 'mav_stream.cpp').read_text(errors='ignore')
    for name in ('FLARE_ORD', 'FLARE_NON', 'FLARE_AGE'):
        assert f'"{name}"' in src


@needs_fw
@pytest.mark.skipif(shutil.which('g++') is None, reason='no g++ to compile pack()')
def test_host_encoder_and_firmware_pack_agree(tmp_path: Path):
    fl = lambda v: f'{float(v)}f'                     # noqa: E731 -- `1f` is not C++
    calls = ''.join(f'{{float p[7]={{{fl(a)},{fl(b)},{fl(c)},{fl(d)},0,0,{fl(n)}}};'
                    f'printf("%lu\\n",(unsigned long)flare_order::pack(p));}}'
                    for a, b, c, d, n in _CASES)
    src = tmp_path / 'm.cpp'
    src.write_text('#include "flare_order.h"\n#include <cstdio>\nint main(){'
                   + calls + 'return 0;}\n')
    exe = tmp_path / 'm'
    subprocess.run(['g++', '-std=c++17', '-I', str(_HDR.parent), str(src), '-o', str(exe)],
                   check=True, capture_output=True, timeout=60)
    fw = [int(x) for x in subprocess.run([str(exe)], capture_output=True, text=True,
                                         check=True).stdout.split()]
    assert len(fw) == len(_CASES)
    for (a, b, c, d, n), got in zip(_CASES, fw):
        assert _host_digits((a, b, c, d), n) == got, ((a, b, c, d, n), got)
    assert any(fw) and not all(fw), 'the case table must exercise both verdicts'


# --------------------------------------------------------------------------- #
#  SrotFC readout                                                              #
# --------------------------------------------------------------------------- #

class _Master:
    def __init__(self):
        self.messages = {}


def _fc_with(**named):
    fc = SrotFC(_Master())
    now = time.time()
    for k, v in named.items():
        fc._named_cache[k] = v if isinstance(v, tuple) else (v, now)
    return fc


def test_no_order_published_reads_None():
    assert _fc_with().flare_order() is None


def test_a_latched_order_is_read():
    fc = _fc_with(FLARE_ORD=312.0, FLARE_NON=9.0, FLARE_AGE=4.5)
    assert fc.flare_order() == {'colours': ('yellow', 'red', 'blue'),
                                'nonce': 9, 'age_s': 4.5}


def test_an_order_that_stopped_arriving_reads_None():
    old = time.time() - 60.0
    fc = _fc_with(FLARE_ORD=(123.0, old), FLARE_NON=(1.0, old), FLARE_AGE=(2.0, old))
    assert fc.flare_order() is None
