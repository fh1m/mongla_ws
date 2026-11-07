# Firmware revisions — what each Hengla behaviour revision changed for this host

The board reports `SROT_FW_BEHAVIOUR_REV` and bumps it whenever it changes
behaviour this host has written a workaround for. The host reads it at connect
and inside `arm()` (`SrotFC.check_behaviour_rev()`), and refuses a board below
`FW_BEHAVIOUR_REV_REQUIRED` in `src/mongla_control/mongla_control/fc/srot_protocol.py`,
which is where the current values live. This page is the history behind them,
moved out of that file so the code says what is true now and this says how it
got there.

**Why the host checks a number and not the firmware's code.** `test_stop_still_does_not_apply_reverse_thrust`
detected the MOVE_STOP fix by grepping their `Type::STOP` case for `abort()`.
They fixed it a different way -- restoring the outgoing leg's axis and speed,
which `start()` had zeroed before the switch -- so our tripwire stayed GREEN
while the behaviour changed underneath it. Had we trusted it we would have kept
the host brake and kicked the hull twice.

## The revisions

```text
  1  MOVE_STOP coasts. A move can be stranded on IN_PROGRESS for ever when a
     failsafe or mode change displaces AUTO. No SET_MESSAGE_INTERVAL. ESC_STATUS
     only (undecodable). DIVE target not clamped. TURN/DIVE progress constant.
  2  fw 2026-08-01 (AUDIT.md R35-R44): MOVE_STOP brakes -- HOST BRAKE REMOVED.
     Every move reaches exactly one terminal ACK, sent once. 511/510 implemented.
     ESC_TELEMETRY_1_TO_4 / _5_TO_8 emitted. DIVE clamped >= 0. Real TURN/DIVE
     progress. SURFACE zeroes pilot translation/yaw; MANUAL_CONTROL ages out.
  3  fw 2026-08-02 (AUDIT.md R46-R50): the Bar30's calibration PROM is validated (CRC-4)
     and no longer read in a race, so DEPTH and TEMPERATURE can now be ABSENT rather than
     silently wrong. WTEMP and SCALED_PRESSURE2 are SUPPRESSED when the baro is unhealthy
     or stale; SCALED_IMU2.temperature sends 0 (MAVLink's "not provided"); a board whose
     PROM fails CRC REFUSES DEPTH_HOLD/AUTO/PATTERN. SYS_STATUS gains the extended health
     bitfield with LEAK on MAV_SYS_STATUS_SENSOR_LEAK. FS_GCS_SYSID/COMPID (255/191) scope
     the GCS failsafe to a named companion. ESPNOW_EN is tri-state, so Battery 2 populates.
  4  fw 2026-08-02 (AUDIT.md R51-R54): YAW IS NOW ABSOLUTE. MAG_YAW_REF defaults on and
     is migrated on for boards storing the old 0, so ATTITUDE.yaw / VFR_HUD.heading are a
     magnetic compass heading instead of relative to wherever the BNO booted. A heading
     compared across a vehicle RESET will differ from rev <= 3 -- that is the fix, not a
     regression, and absolute MOVE_TURN (p4=1) finally turns to the heading it is given.
     The BNO's own calibration is persisted to sensor flash (mag accuracy survives a power
     cycle), and the mag report stops once the one-shot reference locks.
  5  fw 2026-08-03: BARO JITTER GATE -- the barometer is judged on the VARIANCE of the
     stream, not only per-sample plausibility. This is the fix for the bench finding we
     reported: a failing Bar30 produced 317-874 mbar sample-to-sample while EVERY reading
     sat inside the wide plausibility band, so depth stayed "healthy" and its phantom value
     saturated the depth PID into full vertical thrust on arming. Peak-to-peak > 15 mbar
     over an 8-sample window now marks the baro unhealthy (suppressing WTEMP /
     SCALED_PRESSURE2 and refusing DEPTH_HOLD/AUTO), and the measure itself is published
     as NAMED_VALUE_FLOAT "BARO_P2P" -- ungated, deliberately, so it can explain WHY depth
     withdrew. Observed live at 5.90 mbar, matching our own independently-computed spread.
  6  fw 2026-08-06: MOTOR_DETECT converges, and can no longer wipe a good calibration.
     Its pulse is driven THROUGH the direction flag, so it measures an AGREEMENT rather
     than an absolute direction; storing that as the new absolute cal made the routine
     converge only when CAL_MDIRn was already +1, so an inverted thruster stayed inverted
     through any number of runs. It now composes. Separately, an inconclusive detect used
     to default to +1 and store it, so a run in air silently reset all eight thrusters and
     reported SUCCESS; it now leaves the value alone and finishes FAIL (which also keeps
     it out of flash). Nothing for this host to adapt to -- mongla_ws never runs
     MOTOR_DETECT -- but it changes what an operator should expect from Bondor.
  7  fw 2026-08-07 (PR #5): PREFLIGHT_STORAGE reports SUCCESS, not only failure. The
     command has always been ACKed ACCEPTED the instant it parses, but the NVS write is
     deferred to the Core-0 update() -- so the ACK has never meant "written", only
     "request received", and the only signal was a CRITICAL statustext on FAILURE with
     silence on success. Silence is indistinguishable from a dropped request or a wedged
     board. Rev 7 emits STATUSTEXT "Params saved to flash" (MAV_SEVERITY_INFO) on the
     success path: if you ever automate a save, wait for the statustext, NOT the ACK.

     The rev number under-counts this release. Five other commits shipped inside rev 7
     without a bump of their own, and two change behaviour we care about:
       * 83ef62e FRAME_REVERSE -- a new param that negates all six axis demands after the
         controllers and before the mixer. DEFAULT 0, so a stock board is unchanged and
         the gate is honest; but it is SET TO 1 on our hull, where it inverts the meaning
         of every MANUAL_CONTROL axis and every SROT_MOVE primitive relative to rev <= 6.
       * 502eb23 MOT_1/MOT_8_DIRECTION now DEFAULT to -1 (as-flown), the rest +1. Our
         documented restore was a uniform [-1] x 8, which is no longer the intended
         configuration and interacts with FRAME_REVERSE. See srot-integration.md.
     The other three are not ours: 4aaa755 adds a COMP_SEEN NAMED_VALUE_FLOAT (a Bondor
     signal -- inert here, our reader table is per-name), b447fe5 unswaps the PM1
     volt/curr pins (a battery reading we only display), 2213c9a is their AGENTS.md.
     Neither of the two that ARE ours is a host-code change -- the wire is identical, and the drift suite is green
     against d6f1da5 -- but do not read "rev 7" as "one small additive change".
  rev 10 (2026-08-07) YAW SENSE IS CORRECTED, AND THIS ONE IS NOT ADDITIVE.
     BNO_SWAP_ROLL_PITCH was an improper transform (swap x/y, keep z -- determinant -1),
     leaving the frame left-handed and SILENTLY INVERTING YAW while roll and pitch read
     correctly. Measured in water 2026-08-07: turning right made yaw DECREASE. MANUAL was
     fine (open loop); STABILIZE and DEPTH_HOLD span the hull.
     *** any heading recorded against rev <= 9 has the opposite sense ***, absolute
     MOVE_TURN targets included. Every heading constant, pool-day note and recorded
     competition_config heading taken before this is negated. The mag alignment must be
     re-run after flashing. This is why the constant below is not merely "bumped".
     Also: MOTOR_DETECT now decides on the gyro projection onto the expected angular
     direction (the old "dominant axis" was degenerate for M5-M8 and gave a wrong vertical
     result) and ends DISARMED in MANUAL rather than armed in STABILIZE.
  rev 11 autotune limit-cycle validity is median + consensus, not min-vs-max.
  rev 12 autotune holds depth through the rate/angle phases.
  rev 13 SROT_MOVE REQUIRES ARMED. The movement state machine never consulted the arm
     state, so a move sent while disarmed ran its whole profile with the thrusters silent
     and finished ACCEPTED at 100 %. Measured: a 3 m FORWARD "completed" in 3.5 s having
     moved nothing -- and the doc names US as the consumer that would advance an entire
     mission on a dead hull. It is now TEMPORARILY_REJECTED + "SROT_MOVE refused: arm
     first". See ACK_TEMPORARILY_REJECTED below: our retry-on-that-code path must not
     spin forever on a disarmed board.
     Also adds per-axis ATC_RAT_*_FLTD and the previously absent ATC_RAT_*_FLTT.
  rev 14 the depth refusal names WHY instead of one identical "No depth sensor" for three
     faults, publishes BARO_HEALTH, and makes the jitter threshold the BARO_JIT_MAX param.
```

Related: [`srot-integration.md`](srot-integration.md) ·
[`cross-repo-contract.md`](cross-repo-contract.md) ·
[`upstream/`](../upstream/README.md)
