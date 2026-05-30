---
name: pool-day
description: Interactive pool-day preflight checklist runner for Duburi 4.2. Use at the start of any in-water session to verify network, MAVLink, sensors, cameras, and disarm safety before touching thrusters.
---

# Pool-Day Preflight

Run the preflight sequence for the Duburi 4.2 AUV. Report pass/fail at each step. Stop and
surface any failure with the specific fix. **Never arm thrusters from this skill.**

## Sequence

1. **Network + serial + sensors**
   ```bash
   ros2 run duburi_manager bringup_check
   ```
   Pings Pi/Jetson IPs, sniffs UDP 14550 for an active MAVLink stream, lists Pixhawk USB
   devices, tests BNO085 auto-detection. Exit 0 = all clear. Report each sub-check.

2. **Heartbeat / telemetry**
   ```bash
   timeout 5 ros2 topic hz /duburi/state
   ```
   Expect ~1 Hz (or faster on change). No messages → manager not up or MAVLink dead.

3. **State sanity**
   ```bash
   timeout 3 ros2 topic echo /duburi/state --once
   ```
   Confirm `armed: false`, a plausible `mode`, finite `yaw_deg` / `depth_m`, battery > 0.

4. **Vision topics** (if running vision)
   ```bash
   ros2 topic list | grep duburi/vision
   timeout 5 ros2 topic hz /duburi/vision/forward/image_raw
   ```
   Expect `pub≈30Hz`. Detector + tracking topics present.

5. **Physical checklist** — walk `.claude/context/pool-day.md` items and ask the operator
   to confirm each (props clear, tether on, topside can ping Jetson, kill-switch reachable).

## Rules

- If any step fails, halt and report the fix from `pool-day.md` / `known-issues.md`.
- Confirm `armed: false` before anyone is near the thrusters.
- This skill is read-only on the vehicle — it never arms, never sends movement.
