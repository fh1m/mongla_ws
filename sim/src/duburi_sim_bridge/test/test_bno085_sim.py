"""The virtual BNO085 board.

The full chain (board -> PTY -> real BNO085Source -> /duburi/state vs Gazebo
ground truth) is measured against a running sim and recorded in BNO085.md.
These pin the parts that would otherwise drift silently.
"""

import math
import os
import random
import re
import statistics

import pytest

from duburi_sim_bridge.bno085_sim import (
    DATASHEET_GYRO_ACCURACY_DEG_PER_S,
    DATASHEET_HEADING_DRIFT_DEG_PER_MIN,
    FIRMWARE_RATE_HZ,
)

HERE = os.path.dirname(os.path.abspath(__file__))
FIRMWARE_DOC = os.path.join(
    HERE, '..', '..', '..', '..', 'src', 'duburi_sensors', 'firmware',
    'esp32c3_bno085.md')


def test_datasheet_constants_are_the_published_ones():
    """BNO08X datasheet rev 1.17, Figure 6-14.

    These are quoted numbers, not tuning knobs. If someone "improves" the sim
    by softening the drift, the sim stops representing the hardware and every
    heading-hold result taken from it becomes optimistic.
    """
    assert DATASHEET_HEADING_DRIFT_DEG_PER_MIN == 0.5
    assert DATASHEET_GYRO_ACCURACY_DEG_PER_S == 3.1


def test_stream_rate_matches_the_firmware_contract():
    """The board must not out-run or under-run the real one.

    BNO085Source._STALE_S is 0.08 s -- four frames at 50 Hz. A board streaming
    much slower would look permanently stale; much faster would hide a
    staleness bug that the vehicle has.
    """
    assert FIRMWARE_RATE_HZ == 50.0
    if not os.path.exists(FIRMWARE_DOC):
        pytest.skip('firmware contract doc not present')
    with open(FIRMWARE_DOC) as f:
        doc = f.read()
    assert re.search(r'\*\*~?50 Hz\*\*|~50 Hz', doc), (
        'firmware contract no longer says 50 Hz')

    from duburi_sensors.sources.bno085 import _STALE_S
    assert _STALE_S > 2.0 / FIRMWARE_RATE_HZ, (
        'the stale window is under two frames at the board rate -- the source '
        'would flap between fresh and stale')


def test_drift_is_linear_and_the_datasheet_magnitude():
    """Drift comes from a fixed per-run ZRO, so heading error grows with t.

    The datasheet explains its own figure causally -- "removal of gyroscope
    ZRO is critical to reduce heading drift" (section 3.3) -- and ZRO is
    essentially constant across a power-up.

    This started as a random walk and measurement rejected it twice: a walk
    grows as sqrt(t) rather than t, AND its coefficient had been divided by 60
    as though it were a rate, together making the drift 7.7x too weak at one
    minute. A sim that holds heading better than the vehicle hides the very
    problem the operator has to plan around.
    """
    spec = DATASHEET_HEADING_DRIFT_DEG_PER_MIN

    def run(minutes, seed):
        bias = random.Random(seed).gauss(0.0, spec)
        return abs(bias * minutes)

    for minutes in (1, 5, 10):
        sample = [run(minutes, s) for s in range(400)]
        rms = math.sqrt(statistics.fmean(x * x for x in sample))
        assert 0.85 < rms / (spec * minutes) < 1.15, (
            f'at {minutes} min the rms drift is {rms:.2f} deg, but the '
            f'datasheet rate predicts {spec * minutes:.2f} deg')

    # Linear, not sqrt: ten minutes must be about ten times one minute.
    one = math.sqrt(statistics.fmean(run(1, s) ** 2 for s in range(400)))
    ten = math.sqrt(statistics.fmean(run(10, s) ** 2 for s in range(400)))
    assert 9.0 < ten / one < 11.0, (
        f'drift grows as t^{math.log(ten / one) / math.log(10):.2f}, not '
        f'linearly -- a random walk would give ~3.2x over this span')


def test_per_sample_noise_is_derived_from_the_rate_spec():
    """The datasheet quotes RATE accuracy; a frame carries that times dt."""
    expected = DATASHEET_GYRO_ACCURACY_DEG_PER_S / FIRMWARE_RATE_HZ
    assert expected == pytest.approx(0.062, abs=1e-3)
    # It must be small next to the deadband the yaw loop closes on (2 deg),
    # or the sim would be testing the loop against noise rather than control.
    assert expected < 0.2


def test_the_board_emits_sensor_frame_ccw():
    """Two conventions, both silent if wrong.

    BNO085Source._reader_loop negates the raw value once, because the firmware
    is ENU/+CCW while the stack is compass/NED. And the board's zero must be
    boot-relative, or the Pixhawk calibration handshake becomes a no-op and is
    never tested.
    """
    import inspect

    from duburi_sim_bridge import bno085_sim

    src = inspect.getsource(bno085_sim.Bno085Sim._sensor_yaw)
    assert 'truth - boot' in src.replace('  ', ' '), (
        'the board must subtract its boot offset -- publishing true heading '
        'makes the calibration handshake untested')
    assert 'boot_offset_deg' in inspect.getsource(bno085_sim.Bno085Sim.__init__)

    from duburi_sensors.sources.bno085 import BNO085Source
    reader = inspect.getsource(BNO085Source._reader_loop)
    assert "-float(msg['yaw'])" in reader, (
        'the driver no longer negates; the board would need to flip too')
