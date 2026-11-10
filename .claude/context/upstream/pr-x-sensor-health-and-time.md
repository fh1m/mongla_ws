# PR request: three changes to what the board says about its own sensors

From: `mongla_ws` (companion, MAVLink compid 191; then named duburi_ws)
Date: 2026-09-11
Status: opened 2026-11-10 as [issue #76](https://github.com/RakibulIslam1/srot-control-board/issues/76), after two months unsent.
Firmware read: behaviour rev 14 (`config.h:805`), `src/comms/mav_stream.cpp`

All three items are measurements taken on the vehicle tonight, on the hull, over
USB-C at 115200. Item 1 is a safety item and is the one we would merge first.

---

## 1. Suppress `ATTITUDE` when the AHRS is unhealthy, instead of streaming zeros

### What we measured

The board came up from a bad boot and, for as long as we watched it, sent:

```
SYS_STATUS  present=0x80203c0b  health=0x2408
ATTITUDE    roll 0.0  pitch 0.0  yaw 0.0   rates 0.0 0.0 0.0     (10 Hz)
SCALED_IMU2 acc 0 0 0 mG        gyro 0 0 0 mrad/s                (50 Hz)
NAMED_VALUE_FLOAT  MAGACC 0.0   COMP_SEEN 0.0   YAW_REF 0.0
```

`health = 0x2408` clears both `MAV_SYS_STATUS_SENSOR_3D_GYRO` (0x01) and
`MAV_SYS_STATUS_SENSOR_3D_ACCEL` (0x02), while `present` sets both. **The board
knew.** It reported the fault correctly on `SYS_STATUS` and then kept sending a
full-rate attitude solution of exact zeros on the message every consumer
actually reads.

After a power cycle the same probe returned `acc (-27, -179, 978) mG` —
magnitude 0.995 g, gravity included, consistent with the reported −15.8° pitch.
So the zeros were the sensor, and the encoding is fine.

### Why it matters to us

`ATTITUDE.yaw = 0.0` is not a missing value, it is **due north**. On our side it
reached the heading publisher, and `heading_lock` closes a 50 Hz yaw-rate loop
on that number. A hull that believes it is already on heading holds still while
being wrong, and nothing logs a fault, because nothing is faulty — every layer
is doing exactly what it was told.

### What we ask for

`sendAttitude` (`mav_stream.cpp:225`) currently packs unconditionally. Apply the
same rule `sendScaledPressure` and `WTEMP` already follow for the barometer:
**do not send `ATTITUDE` (or `SCALED_IMU2`) while the IMU health bits are
clear.** Absence is unambiguous; zero is not.

This is rev 3's own stated principle — *"the board refuses to report data it
cannot stand behind"* — applied to the one message that escaped it. We think
you will consider this a bug rather than a feature request.

### What we have already done on our side

We are not waiting on this. `SrotFC._ahrs_healthy()` gates `get_attitude()` to
NaN, and `get_angular_rates()` / `get_imu()` to `None`, whenever health says the
IMU is bad — tri-state, so a revision that never sets the bits still passes.
That guard should be redundant once this lands, and we intend to keep it
anyway.

---

## 2. `SCALED_IMU2` is right, and we would like to state what we now depend on

No change requested. Recorded so it does not get optimised away.

`mav_stream.cpp:230-240` packs accel as `(s.lx + s.grx) / 9.80665 * 1000` mg
and gyro as `s.gx * 1000` mrad/s. We had been decoding this message for exactly
one field — `temperature` — and discarding six axes of live inertial data every
frame at 50 Hz. We now consume all six: it is the propagation input for an
invariant filter (`duburi_localization`) running on the Pi.

Two properties we now rely on:

* **accel includes gravity.** A level hull at rest owes ~1000 mg on one axis,
  which is also what makes all-zero a detectable fault rather than a plausible
  reading.
* **`time_boot_ms` is the board's own capture time.** Measured: the board's
  ATTITUDE interval has sd 0.00 ms; host arrival has sd 6.67 ms and p2p
  35.12 ms. All the jitter is transport.

If `SCALED_IMU2` is ever gated, rate-limited or repacked, it is now load-bearing
for position, not only for a temperature readout.

---

## 3. `TIMESYNC` (111)

Still absent from the handler switch. We measured 0 of 12 requests answered.

We work around it with `flow_timing.ClockMap`, which maps `time_boot_ms` onto
host time by fitting the **lower envelope** of one-way pairs (transport delay is
strictly non-negative, so a fit through the middle of the cloud measures mean
delay rather than offset). It works, and it is an estimator standing in for a
measurement.

A standard MAVLink ping-pong would replace it with two timestamps. Lowest
priority of the three; we have a working path.

---

## What we are not asking for

Nothing in items 1–3 changes control behaviour, and none of it touches the
mixer, the depth loop, or arming. We are asking the board to be as careful
about what it *reports* as rev 3 already made it about what it *believes*.
