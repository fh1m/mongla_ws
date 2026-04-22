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
| Vision verb times out immediately with "vision state not ready" | Vision pipeline isn't up. Start `ros2 launch duburi_vision webcam_demo.launch.py` first, then retry. |
| Depth axis stalls / barely moves when tracking a tall person | `depth_anchor_frac` is 0.5 (bbox centre). Set `ros2 param set /duburi_manager vision.depth_anchor_frac 0.2` to align near the top of the bbox instead. |
| Target oscillates horizontally | `kp_yaw` or `kp_lat` too high. Lower to 40–50. Or increase `deadband` to 0.12. |
| AUV overshoots distance target | `kp_forward` too high, or `deadband` too loose for the `forward` axis. Reduce `kp_forward` to 150. |
| Detection drops frequently ("stale" in logs) | Lighting or model confidence. Lower `vision.stale_after` only after fixing the root cause. Use `on_lost='hold'` to ride out transient drops. |
| Wrong object being chased | Model detecting background noise. Narrow YOLO class allowlist in `config/detector.yaml` or use a more specific `target_class`. |

---

## Hardware & EKF

| Symptom | Fix |
|---------|-----|
| EKF3 switches compass rapidly in logs | Expected on freshly powered Pixhawk. If persistent underwater, recalibrate compass on land. |
| TDR says "VectorNav VN200" but there's no code for it | Intentional deviation — BNO085 + ESP32-C3 is used instead. See [`vehicle-spec.md`](../../.claude/context/vehicle-spec.md). |
