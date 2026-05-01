# Future TODO -- feed BNO085 yaw into ArduSub's EKF

> **Status:** parked. Do NOT implement before the next pool test.
> Reasoning, design sketch, and abort-criteria are below so future work
> can resume without re-deriving the trade-offs.

---

## What this would actually mean

Today, `yaw_source:=bno085` only changes which sensor our **Python**
loops read. ArduSub's onboard 400 Hz attitude controller (the one
that runs in STABILIZE / ALT_HOLD-stick-release / yaw-hold-sub-loop)
still uses its compass-fed EKF and has no idea the BNO exists. See
[`ardusub-canon.md`](./ardusub-canon.md) §4A for the full table.

This future work would push BNO yaw into the EKF so that:

1. The yaw-hold sub-loop in ALT_HOLD/STABILIZE corrects to BNO,
   not to compass.
2. STABILIZE finally becomes useful underwater (today it tracks the
   compass which is being wrecked by ESC magnetic noise).
3. We get to delete `HeadingLock` -- ArduSub's own hold would do it.

---

## How ArduSub accepts external yaw (3 paths, ranked)

The relevant ArduPilot params and messages:

| EK3 source value | Means | Wire-side input we'd have to send |
| --- | --- | --- |
| `EK3_SRC1_YAW=1` | compass (today's default) | -- |
| `EK3_SRC1_YAW=2` | GPS yaw / GSF | `GPS_INPUT.yaw` field |
| `EK3_SRC1_YAW=6` | external nav | `ATT_POS_MOCAP` quaternion or `ODOMETRY` |
| `EK3_SRC1_YAW=8` | GPS-with-fallback | requires GPS |

Plus three EKF tunables we'd touch: `EK3_YAW_M_NSE` (m noise),
`EK3_GPS_CHECK` (GPS gate, must allow yaw-only when no XY), and the
arming-check bypass for compass.

**Ranked options:**

1. **`ATT_POS_MOCAP` with yaw-only quaternion** -- best fit. Streams
   a fake "external nav" pose at e.g. 25 Hz with an identity X/Y/Z
   and a yaw-bearing quaternion. ArduSub already supports it
   (added in 4.1). Requires `EK3_SRC1_YAW=6` and the right typemask.
2. **Synthetic `GPS_INPUT` with `yaw` field, no XY** -- works in
   theory but requires faking a GPS fix; ArduSub's pre-arm GPS
   checks are touchy and would need bypassing, which is a regression
   in our safety posture.
3. **`SET_ATTITUDE_TARGET` in GUIDED with yaw quaternion + ignore
   the rest** -- not a true EKF feed; just a setpoint. Would still
   need our HeadingLock to keep streaming. Not worth it.

Pick (1).

---

## Why we are NOT doing it before the next pool test

Adding any of the above silently changes how the entire 400 Hz
inner loop is fed. The blast radius is wider than HeadingLock:

* **Sign-error magnification.** A wrong-handed quaternion looks fine
  for a few frames then catastrophically inverts the yaw-hold
  command. The compass-fed loop at least matches Earth -- a wrong
  external feed can spin the sub at full Ch4.
* **Circular calibration.** Today we capture the BNO->Earth offset
  ONCE from the compass at startup. If we then feed BNO back into
  the EKF and the compass is degraded, the offset itself drifts and
  the next cold-start calibration is corrupted by what was a
  corrupted EKF the previous run. The fix is per-session magnetic
  reference (Earth-frame from a known marker) but we don't have
  that infrastructure.
* **Pre-arm refusal.** `EK3_SRC1_YAW=6` makes ArduSub refuse to arm
  until it sees a valid `ATT_POS_MOCAP` stream. A late
  manager start = no arm = mission scrubbed. We'd want a guard that
  sends a blank/identity stream the moment MAVLink comes up so
  pre-arm always sees a feed; that's another ~80 LOC + tests.
* **Failsafe degradation.** `FS_EKF_ACTION` triggers on EKF
  innovations above `FS_EKF_THRESH`. A flaky BNO USB cable
  (read: real 2026 hardware) would now trip the EKF failsafe
  instead of silently degrading our Python loop.
* **Validating means we have to fly it twice.** Once with the old
  Python-side BNO loop to bank a baseline, once with the EKF feed,
  in calm water, with a VICON / dive plumb-bob ground truth. That
  is a half-day of pool time we don't have before tomorrow.

---

## Design sketch (when we DO pick this up)

A new module would own the EKF feed:

```
src/duburi_sensors/duburi_sensors/sources/ekf_yaw_feeder.py
```

Behaviour:

1. Spawn at manager startup if `ek3_yaw_feeder:=true` and
   `yaw_source` is a real BNO source.
2. Stream `ATT_POS_MOCAP` at 25 Hz with:
   * `q = quaternion(0, 0, yaw=BNO_yaw_earth)`
   * `x=y=z=0`, `covariance` with yaw set very low and XYZ set very
     high so EKF treats only yaw as informative.
3. Stop streaming the moment BNO `is_healthy()` flips False --
   ArduSub falls back to compass automatically.
4. Log one `[EKF ]` line per second so the operator sees the feed
   alive.

Manager parameter glue:

```yaml
ek3_yaw_feeder: true        # default false
ek3_yaw_feed_hz: 25
ek3_yaw_arm_grace_s: 5      # send identity stream this long before allowing arm
```

ArduSub-side params (set ONCE via QGC, NOT in our code; written down
here so the operator has the recipe):

```
EK3_SRC1_YAW = 6        (was 1)
EK3_GPS_CHECK = 0        (skip GPS gate; we have no GPS underwater)
GPS_TYPE = 0            (no GPS attached)
COMPASS_USE = 0          (don't waste EKF cycles on a compass we're ignoring)
```

`HeadingLock` becomes optional after this -- the autopilot's onboard
yaw-hold sub-loop now closes on the same BNO data and is faster
(400 Hz vs our 20 Hz). Keep `HeadingLock` for one more pool day
post-implementation as a fallback, then delete.

---

## Pre-implementation checklist

Before opening the PR, the following must be true:

* [ ] Pool team has scheduled at least 4 hours of pool time with a
      dive plumb-bob / VICON / external yaw ground truth.
* [ ] BNO firmware has been audited for chirality (ENU vs NED is
      already converted in `bno085.py`; verify the EKF feed uses the
      SAME convention -- a sign error here is what wrecks day 1).
* [ ] An operator-visible "abort the EKF feed" command exists
      (`duburi ekf_yaw_off`) that sets `EK3_SRC1_YAW` back to 1 via
      `param_set` AND restarts the Python `HeadingLock` so the sub
      stays controllable mid-mission.
* [ ] At least one whole `find_person_demo` mission has been run
      back-to-back: once with the EKF feed, once without, with the
      same start state, comparing trajectories.
* [ ] Failsafe behaviour has been tested: yank the BNO USB during
      a `lock_heading` and verify ArduSub falls back to compass
      within `FS_EKF_THRESH * EK3_GLITCH_RNG_GATE` (default ~1 s).

---

## Abort criteria (when to revert before the next mission)

Any of these means: revert `EK3_SRC1_YAW=1`, restart manager, log a
post-mortem, do not retry until the issue is understood:

* Compass-vs-BNO disagreement > 15 deg sustained for > 10 s on
  the surface.
* EKF innovations above 2x `FS_EKF_THRESH` for > 1 s during any
  yaw command.
* `STATUSTEXT` stream contains `EKF compass error` or
  `EKF yaw rejected` more than once in 30 s.
* Any pre-arm refusal that didn't exist with `EK3_SRC1_YAW=1`.

---

## Cross-references

* ArduPilot EKF3 sources documentation:
  https://ardupilot.org/copter/docs/common-ek3-sources.html
* `ATT_POS_MOCAP` MAVLink message:
  https://mavlink.io/en/messages/common.html#ATT_POS_MOCAP
* Today's BNO offset / ENU-to-NED handling:
  `src/duburi_sensors/duburi_sensors/sources/bno085.py`
* Today's Python-side yaw consumers (the things this future work
  would relieve): [`ardusub-canon.md`](./ardusub-canon.md) §4A
