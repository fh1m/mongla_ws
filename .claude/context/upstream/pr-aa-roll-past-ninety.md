# Ask AA: past 90 degrees, ATTITUDE roll disagrees in sign with the board's own gravity

**Status:** sent 2026-11-24 as [srot-control-board issue 83](https://github.com/Compasless/srot-control-board/issues/83). Firmware behaviour revision 14.

## What we measured

The board on the bench, on USB, read through `/mongla/imu` (the manager builds its
orientation from `ATTITUDE` without changing the angles; the round trip is exact).
Accelerometer in body FRD, so at rest `f = (g sin p, -g sin r cos p, -g cos r cos p)`.

| pose (held still) | ATTITUDE roll / pitch | accelerometer (x y z, m/s^2) | roll gravity says |
|---|---|---|---|
| flat | +0.1 / -0.7 | -0.29 -0.03 -9.80 | 0 |
| rolling, passing +50 | +50.1 / +2.2 | +0.14 -7.55 -6.22 | +50 |
| rolling, passing +74 | +74.4 / +8.7 | +0.38 -9.52 -2.30 | +76 |
| held 35 s | -172.1 / -3.1 | -0.34 -1.34 +9.62 | **+172** |
| held 9 s | -114.2 / +8.7 | +0.68 -8.93 +3.95 | **+114** |
| face down on the table, 15 s | +172.6 / -2.2 | -0.20 +1.26 +9.66 | **-172.6** |

Below 90 degrees the attitude and gravity agree to about 2 degrees. Past 90 degrees they
disagree in sign, held still, for as long as it is held. A filter that has converged
agrees with gravity when the board is still, and a frame error would show at every
angle, so the fault appears where roll passes 90.

## Where we think it is

`src/drivers/bno085.cpp`, under `BNO_SWAP_ROLL_PITCH` (set to 1 in `config.h`): the
quaternion is converted to Euler angles first, and then roll and pitch are exchanged and
yaw negated. That exchange equals the sensor's mounting rotation only near level; at
large angles, Euler angles of a rotated frame are not the exchanged Euler angles of the
original. Rotating the quaternion by the mount first and then converting is exact at
every attitude:

    q_body = q_sensor * q_mount     (q_mount: the 180 deg turn about x = y)
    then quatToEuler(q_body)

We have not reproduced the exact numbers above in a host model of the code, so this is
the likely place, not a proven one.

## Pass / fail bench test

Board still for 10 s in each pose; read `ATTITUDE` and `SCALED_IMU`:

1. flat: roll within 2 deg of 0;
2. right side down, about 45 deg: roll about +45, and `atan2(-ay, -az)` within 3 deg of it;
3. face down: roll within 3 deg of `atan2(-ay, -az)` (about +/-180), **same sign**;
4. left side down past vertical, about 120 deg: roll about -120, same check.

Today 1 and 2 pass and 3 and 4 fail.

## Why it matters

The hull never rolls past 90 degrees in normal running, but `style_roll` turns it through
a full roll, and every loop that reads roll is wrong for that half of the turn.
