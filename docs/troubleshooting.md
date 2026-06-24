# Troubleshooting

> Run `colcon build --packages-select duburi_manager && source install/setup.bash`
> first — stale generated files cause ~80% of weird failures.

---

## Connection & startup

| Symptom | Fix |
|---------|-----|
| No `[STATE]` line after startup | UDP 14550 not reaching the Jetson. Verify BlueOS `inspector` endpoint IP matches Jetson static IP (`192.168.2.69`). Run `ss -lun \| grep 14550`. |
| `/dev/ttyACM0: Permission denied` (desk mode) | `sudo usermod -aG dialout "$USER"` then log out and back in. |
| `BNO085 calibration timed out` at startup | Pixhawk yaw or BNO yaw unavailable for 5 s. Test each separately: `yaw_source=mavlink_ahrs` first, then `sensors_node` with BNO only. Fix whichever is silent. |
| BNO Earth-ref offset looks wrong | Calibration was done with the AUV not level or near a magnetic field. Re-run at the surface, away from metal. Restart the manager to re-zero. |
| Startup banner missing BlueOS hint | Hint only prints for `mode:=pool` or `mode:=laptop`. |

---

## Arming

| Symptom | Fix |
|---------|-----|
| `arm -> FAIL: DENIED` | ArduSub pre-arm check failed. Read `[ARDUB]` lines for the reason (compass cal, GPS, battery voltage, ...). |
| `arm -> FAIL: NO_ACK` | Heartbeat present but no ACK. Pre-arm stall. Restart ArduSub or BlueOS if persistent. |

---

## Depth

| Symptom | Fix |
|---------|-----|
| Depth command times out at ~-0.5 m | ArduSub didn't enter `ALT_HOLD`. Check `[CMD  ] set_depth` is followed by a mode confirmation, and `[STATE]` shows `ALT_HOLD`. On real hardware verify Bar30 calibration. |
| First `set_depth` after arming lurches downward | The 0.5 s `prime_alt_hold` phase should prevent this. If it returns, raise the prime duration in `motion_depth.py`. |
| `set_mode -> FAIL: DENIED` | Trying to enter a mode that requires conditions (e.g. ALT_HOLD needs a healthy Bar30). |

---

## Motion

| Symptom | Fix |
|---------|-----|
| Yaw overshoots target | Enable `-p smooth_yaw:=true`. If still overshooting, reduce `ATC_ANG_YAW_P` on the ArduSub side via QGC. |
| Small backward drift after `move_forward` | Enable `-p smooth_translate:=true` (ramp braking) or pass `--settle 1.0` per command. |
| `arc` curves the wrong way | `yaw_rate_pct` positive = clockwise from above. Flip the sign. |
| `lock_heading` active but yaw drifts during `move_forward` | Build is stale — translations must use lock-aware `Writers`. Rebuild: `colcon build --packages-select duburi_control`. |

---

## Vision

| Symptom | Fix |
|---------|-----|
| `vision_align` / `vision_move` returns `NO_CAMERA` (code 3) | Camera pipeline isn't up — `camera_info` was never seen, so there's no trustworthy pixel scale. Start `ros2 launch duburi_vision cameras_.launch.py`, confirm the `--camera`/`camera=` name matches a running detector, then retry. |
| Camera clearly sees the target but the AUV doesn't move | The detector is publishing boxes, but **none match `target_class`**. Watch for `[VIS  ] align: 'gate' not among live detections [...] -- check classes filter / model`. Fix the class allowlist (`ros2 param set /duburi_detector classes "gate,flare"`) or switch to the model that actually has that class. Class matching is case-insensitive. |
| Target oscillates horizontally | `vision.kp_yaw` or `vision.kp_lat` too high. Lower to 40–50, or loosen the per-call `err` (e.g. `err=60`). |
| AUV overshoots / never settles the forward approach | `vision.kp_forward` too high, or the per-call `gain` cap too high for a clean stop. Reduce `vision.kp_forward` to 150 and/or lower `gain`. |
| Verb reports `LOST` and gives up immediately | No `fallback` was supplied, so the loop only coasts `vision.lost_grace_s` then returns `LOST`. Pass a mission-authored `fallback=` search (the verb re-enters after it) or raise `vision.lost_grace_s` to ride out transient drops. |
| Wrong object being chased | Model detecting background noise. Narrow the YOLO class allowlist in `config/detector.yaml` (or the `classes` param) or use a more specific `target_class`. |

---

## Hardware & EKF

| Symptom | Fix |
|---------|-----|
| EKF3 switches compass rapidly in logs | Expected on freshly powered Pixhawk. If persistent underwater, recalibrate compass on land. |
| TDR says "VectorNav VN200" but there's no code for it | Intentional deviation — BNO085 + ESP32-C3 is used instead. See [`vehicle-spec.md`](../../.claude/context/vehicle-spec.md). |
