"""SCALED_IMU2 accel arrives in the BNO's sensor axes; ATTITUDE and gyro in FRD.

`bno085.cpp` remaps attitude and gyro (x, y, z) -> (y, x, -z) under
`BNO_SWAP_ROLL_PITCH` and leaves gravity + linear acceleration unmapped.
Measured on the vehicle 2026-09-15: ATTITUDE roll +1.57, pitch -15.10 deg and
accel (-0.41, -2.56, +9.38) m/s^2. Unmapped into the filter, 50 s at rest ran
to 583 m. These drive the REAL `get_imu` with that sample.
"""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from pymavlink import mavutil                              # noqa: E402

from duburi_control.fc.srot_fc import SrotFC               # noqa: E402

HEALTH = 0x2408 | mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_GYRO | \
    mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_ACCEL
MG = 1000.0 / 9.80665


class _Msg:
    def __init__(self, **kw):
        self.__dict__.update(kw)


class _FC(SrotFC):
    """The real accessors; only the MAVLink cache is stubbed."""

    def __init__(self, roll_deg, pitch_deg, acc):
        self.roll, self.pitch, self.acc = roll_deg, pitch_deg, acc

    def _cache(self, key):
        if key == 'SYS_STATUS':
            return _Msg(onboard_control_sensors_health=HEALTH,
                        onboard_control_sensors_present=0x80203c0b)
        if key == 'ATTITUDE':
            return _Msg(roll=math.radians(self.roll), pitch=math.radians(self.pitch),
                        yaw=1.64, rollspeed=0.0, pitchspeed=0.0, yawspeed=0.0,
                        time_boot_ms=1000, _timestamp=1.0)
        if key == 'SCALED_IMU2':
            ax, ay, az = (int(round(c * MG)) for c in self.acc)
            return _Msg(xacc=ax, yacc=ay, zacc=az, xgyro=0, ygyro=0, zgyro=0,
                        time_boot_ms=1000, _timestamp=1.0)
        return None

    def _baro_healthy(self):
        return True


def _settle(fc, n=None):
    imu = None
    for _ in range(n or SrotFC._ACCEL_FRAME_VOTES):
        imu = fc.get_imu()
    return imu


def _frd_rest(roll_deg, pitch_deg):
    r, p, g = math.radians(roll_deg), math.radians(pitch_deg), 9.80665
    return (g * math.sin(p), -g * math.sin(r) * math.cos(p),
            -g * math.cos(r) * math.cos(p))


def test_the_measured_board_is_proven_sensor_frame_and_mapped_to_frd():
    fc = _FC(1.57, -15.10, (-0.41, -2.56, 9.38))
    imu = _settle(fc)
    assert imu['accel_frame'] == 'sensor'
    exp = _frd_rest(1.57, -15.10)
    for got, want in zip(imu['accel'], exp):
        assert abs(got - want) < 0.3, (imu['accel'], exp)


def test_nothing_is_published_before_the_frame_is_proven():
    fc = _FC(1.57, -15.10, (-0.41, -2.56, 9.38))
    assert _settle(fc, SrotFC._ACCEL_FRAME_VOTES - 1)['accel'] is None


def test_a_firmware_that_fixes_the_accel_is_not_mapped_twice():
    """If the board ever sends FRD accel, the swap would put gravity UP."""
    exp = _frd_rest(1.57, -15.10)
    imu = _settle(_FC(1.57, -15.10, exp))
    assert imu['accel_frame'] == 'vehicle'
    assert imu['accel'][2] < -9.0


def test_level_is_separable_by_the_sign_of_z():
    assert _settle(_FC(0.0, 0.0, (0.0, 0.0, 9.80665)))['accel_frame'] == 'sensor'
    assert _settle(_FC(0.0, 0.0, (0.0, 0.0, -9.80665)))['accel_frame'] == 'vehicle'


def test_a_manoeuvre_does_not_vote():
    """|a| far from g is not a gravity reading."""
    assert _settle(_FC(0.0, 0.0, (8.0, 0.0, 9.80665)), 200)['accel'] is None


def test_an_ambiguous_board_never_latches():
    """Neither hypothesis near the attitude's gravity: refuse, do not pick."""
    assert _settle(_FC(0.0, 0.0, (9.80665, 0.0, 0.0)), 200)['accel'] is None
