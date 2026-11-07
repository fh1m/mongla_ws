---
name: pool-day
description: Interactive pool-day preflight checklist runner for the Mongla vehicle (SROT board + Pi 5 / Hailo-8). Use at the start of any in-water session to verify the board link, sensors, cameras, models and disarm safety before touching thrusters.
---

# Pool-Day Preflight

Run the preflight sequence for the Mongla vehicle. Report pass/fail at each step. Stop and
surface any failure with the specific fix. **Never arm thrusters from this skill.**

## Sequence

1. **The gate** — on the Pi (`fh1m@mongla.local`)
   ```bash
   ros2 run mongla_manager bringup_check --strict
   ```
   Grades compute, vision deps, the CH340 driver (and brltty absent), the SROT board link,
   HEARTBEAT, firmware behaviour revision, barometer, failsafe, the board's yaw source,
   payload channels, both cameras, the Hailo models and their sidecars, and Pi power.
   Exit 0 = no FAIL. Report each section. To *look* rather than grade:
   `ros2 run mongla_manager connect --watch`.

2. **Telemetry**
   ```bash
   timeout 5 ros2 topic hz /mongla/state
   ```
   Published on change. No messages → the manager is not up or the board link is dead.

3. **State sanity**
   ```bash
   timeout 3 ros2 topic echo /mongla/state --once
   ```
   Confirm `armed: false`, a plausible `mode`, finite yaw and depth, battery present. A
   missing number is `NaN`, never `0.0` — and a depth value does **not** prove the barometer
   is healthy (`VFR_HUD` is not gated on it; section D–F of the gate is).

4. **Vision topics** (if running vision)
   ```bash
   ros2 topic list | grep mongla/vision
   timeout 5 ros2 topic hz /mongla/vision/forward/detections
   ```
   Detections flowing at the camera rate. An empty list every frame with a healthy pipeline
   is the missing-sidecar failure (CLAUDE.md §4).

5. **Physical checklist** — walk `.claude/context/platform/pool-day.md` and
   `first-pool-verification.md`, and ask the operator to confirm each item (props clear,
   tether on, the Pi reachable over mosh, a human on the kill switch).

## Rules

- If any step fails, halt and report the fix from `pool-day.md` / `BUGS.md`.
- Confirm `armed: false` before anyone is near the thrusters.
- This skill is read-only on the vehicle — it never arms, never sends movement.
