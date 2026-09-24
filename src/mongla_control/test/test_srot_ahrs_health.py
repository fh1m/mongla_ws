"""The attitude half of the sensor-health gate, which was missing.

⛔ MEASURED ON THE VEHICLE, 2026-09-11. After a bad boot the SROT board
reported:

    SYS_STATUS present=0x80203c0b health=0x2408
    NVF MAGACC=0.0  COMP_SEEN=0.0  YAW_REF=0.0
    ATTITUDE rpy 0.0 0.0 0.0  rates 0.0 0.0 0.0        (10 Hz, for minutes)
    SCALED_IMU2 acc 0 0 0 mG   gyro 0 0 0 mrad/s       (50 Hz)

0x2408 has neither 3D_GYRO (0x01) nor 3D_ACCEL (0x02) set, while `present`
has both -- so the board KNEW and said so on a field nothing read. Meanwhile
`get_attitude()` returned yaw 0.0 as a number, `_effective_yaw_deg` published
it to `/mongla/state`, and `heading_lock` would have closed a 50 Hz Ch4 loop
on a heading the vehicle does not have.

After the operator power-cycled the board the same probe returned accel
(-27, -179, 978) mG -- magnitude 0.995 g, gravity included, matching the
measured -15.8 degree pitch. So the zeros were the sensor, not the encoding,
and a real reading is never all-zero: the firmware packs accel as
`(s.lx + s.grx)` (mav_stream.cpp:230), so a level hull at rest owes ~1000 mG
on one axis.

This is the BAROMETER gate's twin. That half was written after a failing Bar30
saturated the depth PID into full vertical thrust on arming. The attitude half
sat open in the same file, against the same bitfield, until a bad boot produced
the fixture.
"""
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from pymavlink import mavutil                              # noqa: E402

from mongla_control.fc.srot_fc import SrotFC               # noqa: E402

GYRO = mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_GYRO
ACCEL = mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_ACCEL

# The two bitfields exactly as the board sent them.
SICK_HEALTH = 0x2408
WELL_HEALTH = 0x2408 | GYRO | ACCEL


class _Msg:
    def __init__(self, **kw):
        self.__dict__.update(kw)


class _FC(SrotFC):
    """Drives the REAL accessors; only the message cache is stubbed."""

    def __init__(self, health=None, *, attitude=True, imu=True):
        self._health = health
        self._attitude = attitude
        self._imu = imu

    def _cache(self, key):
        if key == 'SYS_STATUS':
            if self._health is None:
                return None
            return _Msg(onboard_control_sensors_health=self._health,
                        onboard_control_sensors_present=0x80203c0b)
        if key == 'ATTITUDE' and self._attitude:
            return _Msg(roll=0.0, pitch=0.0, yaw=0.0, rollspeed=0.0,
                        pitchspeed=0.0, yawspeed=0.0, time_boot_ms=1000,
                        _timestamp=time.time())   # fresh: get_attitude gates on age
        if key == 'SCALED_IMU2' and self._imu:
            return _Msg(xacc=0, yacc=0, zacc=0, xgyro=0, ygyro=0, zgyro=0,
                        time_boot_ms=1000, _timestamp=1.0)
        if key == 'VFR_HUD':
            return _Msg(alt=-1.18)
        return None

    def _baro_healthy(self):
        return True


# --------------------------------------------------------------------------- #
#  the gate itself
# --------------------------------------------------------------------------- #
def test_an_unhealthy_ahrs_makes_yaw_NaN_not_zero():
    att = _FC(SICK_HEALTH).get_attitude()
    assert math.isnan(att['yaw']), 'a dead BNO must not report heading 0.0'
    assert math.isnan(att['roll'])
    assert math.isnan(att['pitch'])


def test_depth_SURVIVES_an_unhealthy_ahrs():
    """Depth comes off the Bar30 and is routinely fine while the BNO is not.

    Blanking the whole reading would trade one silent failure for another --
    every depth guard in this stack compares against a negative constant, so a
    NaN depth simply stops all of them firing.
    """
    att = _FC(SICK_HEALTH).get_attitude()
    assert att['depth'] == -1.18


def test_a_healthy_ahrs_is_untouched():
    att = _FC(WELL_HEALTH).get_attitude()
    assert att['yaw'] == 0.0 and not math.isnan(att['yaw'])


def test_no_SYS_STATUS_yet_is_NOT_unhealthy():
    """Tri-state, same as the baro gate. Failing closed on a missing bit would
    blank attitude for the first half-second of every connection, and would
    strand a firmware revision that does not populate the field at all."""
    att = _FC(None).get_attitude()
    assert att['yaw'] == 0.0


def test_either_bit_alone_is_not_enough():
    """The attitude solution fuses gyro and accel, so one failing is enough to
    make the quaternion untrustworthy."""
    assert math.isnan(_FC(SICK_HEALTH | GYRO).get_attitude()['yaw'])
    assert math.isnan(_FC(SICK_HEALTH | ACCEL).get_attitude()['yaw'])


# --------------------------------------------------------------------------- #
#  rates and the raw IMU take the same gate
# --------------------------------------------------------------------------- #
def test_rates_go_ABSENT_not_zero():
    """The flow node de-rotates by `f*omega*dt`. A zero rate is not a neutral
    value there -- it is a positive claim that the hull did not turn."""
    assert _FC(SICK_HEALTH).get_angular_rates() is None
    assert _FC(WELL_HEALTH).get_angular_rates() is not None


def test_get_imu_goes_ABSENT_not_zero():
    assert _FC(SICK_HEALTH).get_imu() is None
    assert _FC(WELL_HEALTH).get_imu() is not None


def test_get_imu_converts_units_from_the_firmware_packing():
    """mg -> m/s^2 and mrad/s -> rad/s, matching mav_stream.cpp:230-240."""
    class _Live(_FC):
        def _cache(self, key):
            if key == 'SCALED_IMU2':
                return _Msg(xacc=-27, yacc=-179, zacc=978,
                            xgyro=9, ygyro=-11, zgyro=-3,
                            time_boot_ms=52758, _timestamp=1.0)
            return super()._cache(key)

    fc = _Live(WELL_HEALTH)
    for _ in range(SrotFC._ACCEL_FRAME_VOTES):
        imu = fc.get_imu()
    # The real sample from the vehicle: |a| must come out near one g. (The
    # stub's ATTITUDE is level, and this sample was taken at -15.8 deg, so
    # the frame can never be proven here -- units are read off the raw path.)
    raw = (fc._cache('SCALED_IMU2'))
    mag = math.sqrt(sum((c * 9.80665e-3) ** 2 for c in (raw.xacc, raw.yacc, raw.zacc)))
    assert imu['accel'] is None, 'an unproven frame must not be published'
    assert 9.5 < mag < 10.1, f'{mag} is not one gravity'
    assert abs(imu['gyro'][0] - 0.009) < 1e-9
    assert imu['board_ms'] == 52758
