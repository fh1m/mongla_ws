# Command reference — generated from the code

> ⚠ **This file is generated.** Do not edit it by hand: run
> `python3 tools/gen_reference.py`. A test fails if it has drifted from the source.
> Every row below was read out of the file named beside it, never written from memory.

**31 verbs.** On the SROT board **11** run as one `SROT_MOVE` primitive, which the board runs *and brakes* itself; **6** are refused before dispatch; the rest are host-side loops or single messages.

## 1. Verbs — `mongla_control/commands.py`

| verb | fields | defaults | on srot | what it does |
|---|---|---|---|---|
| `arc` | `duration`, `gain`, `target_yaw`, `settle` | `gain=50.0`, `target_yaw=0.0`, `settle=0.0` | ⛔ **refused** | Curved motion to an ABSOLUTE heading: drives forward at `gain` pct for `duration` s while a PID turns the hull to `target_yaw` (deg) and holds it. Turn direction auto-computed. |
| `arm` | `timeout` | `timeout=15.0` | host | Arm the vehicle (motors hot). |
| `calc_distance` | `phase` | `phase=start` | host | Downward optical-flow distance bracket: phase='start' latches the axis + resets, 'stop' returns accumulated metres in final_value. DVL-free; runs with detectors paused. |
| `calibrate_depth` | — | — | host | Re-zero the barometer at the surface (QGC "Calibrate Pressure") so depth reads 0 before a dive. DISARMED + surface only; verifies the re-zero took. Auto-run by mission_reset; also standalone. |
| `disarm` | `timeout` | `timeout=20.0` | host | Disarm safely (mode -> MANUAL, neutral, then disarm). |
| `dvl_connect` | — | — | host | Connect to the Nortek Nucleus 1000 DVL over TCP and begin streaming. Must be called before any move_*_dist command when yaw_source is dvl/nucleus_dvl. |
| `fire` | `fire_channel` | `fire_channel=0.0` | host | Activate payload BOARD channel N (1..16) -- the same n as SERVO{n}_ROLE, no host-side map. The board decides: a SWITCH channel fires, a PWM/SERVO channel is REFUSED (it is the on-board arm). `mongla_manager connect` lists which is which. |
| `head` | — | — | host | Read current heading (degrees) at execution time. Result is in final_value. Also works as a magic value in other CLI commands: --target head resolves to the live heading the moment the command runs. |
| `lock_heading` | `target`, `timeout` | `target=0.0`, `timeout=300.0` | ⛔ **refused** | Stream Ch4 rate-overrides driven by yaw_source until unlock_heading. target=0 means lock current heading. |
| `mission_reset` | — | — | host | Stop heading lock, clear abort event, send RC neutral. Call at start of every mission run() to clear state carried forward from any previous mission. |
| `motor_test` | `target`, `gain`, `duration` | `target=1.0`, `gain=20.0`, `duration=2.0` | host | Spin ONE thruster (1-8) at `gain` percent for `duration` s. SROT only; requires ARMED. The board expires the test and DISARMS within 3 s of the last keep-alive, which is the safety mechanism -- there is no stop command. PROPS OFF OR VEHICLE RESTRAINED. |
| `move_back` | `duration`, `gain`, `settle` | `gain=80.0`, `settle=0.0` | on the board | Drive backward for `duration` s at `gain` percent thrust. |
| `move_back_dist` | `distance_m`, `gain`, `dvl_tolerance`, `settle` | `gain=60.0`, `dvl_tolerance=0.1`, `settle=0.0` | ⛔ **refused** | Drive backward `distance_m` metres using DVL position feedback. Falls back to open-loop timed drive if no DVL position available. |
| `move_forward` | `duration`, `gain`, `settle` | `gain=80.0`, `settle=0.0` | on the board | Drive forward for `duration` s at `gain` percent thrust. |
| `move_forward_dist` | `distance_m`, `gain`, `dvl_tolerance`, `settle` | `gain=60.0`, `dvl_tolerance=0.1`, `settle=0.0` | ⛔ **refused** | Drive forward `distance_m` metres using DVL position feedback. Falls back to open-loop timed drive if no DVL position available. |
| `move_lateral_dist` | `distance_m`, `gain`, `dvl_tolerance`, `settle` | `gain=36.0`, `dvl_tolerance=0.1`, `settle=0.0` | ⛔ **refused** | Strafe `distance_m` metres (positive=right, negative=left) using DVL position feedback. |
| `move_left` | `duration`, `gain`, `settle` | `gain=80.0`, `settle=0.0` | on the board | Strafe left for `duration` s at `gain` percent thrust. |
| `move_right` | `duration`, `gain`, `settle` | `gain=80.0`, `settle=0.0` | on the board | Strafe right for `duration` s at `gain` percent thrust. |
| `pause` | `duration` | `duration=2.0` | on the board | Release RC override for N seconds (autopilot takes over). |
| `set_depth` | `target`, `timeout`, `settle` | `timeout=30.0`, `settle=0.0` | on the board | Hold absolute depth (`target` metres, negative below surface). |
| `set_mode` | `target_name`, `timeout` | `timeout=8.0` | host | Switch flight mode. srot: STABILIZE|DEPTH_HOLD|SURFACE|MANUAL|ACRO (ALT_HOLD aliases to DEPTH_HOLD). pixhawk: MANUAL|ALT_HOLD|STABILIZE|... |
| `stop` | — | — | on the board | Active hold: send neutral 1500 PWM to all six channels. |
| `style_roll` | `gain`, `timeout`, `flips`, `headroom` | `gain=60.0`, `timeout=20.0`, `flips=1`, `headroom=1.0` | on the board | Style: N×360° roll on Ch2 in ACRO mode, one flip per loop iteration; `timeout` is per flip, not total. BNO085-confirmed (AHRS2 fallback) with direction-locked unwrap. Per flip: optional `headroom` m pre-dive, ACRO roll guarded by a hard surface-depth abort and cos(roll)-modulated Ch3 depth correction, then ALT_HOLD recovery to the origin depth before the next flip. ACRO_BAL_ROLL + ACRO_TRAINER zeroed before and restored after. Cancel mid-flip restores ALT_HOLD then disarms. |
| `style_yaw` | `flips`, `deg_per_step`, `settle` | `flips=1`, `deg_per_step=90.0`, `settle=1.0` | ⛔ **refused** | Style: N×360° yaw spin in ALT_HOLD. flips full rotations, each as (360/deg_per_step) steps with settle between. BNO heading tracking active. No mode change — safest style verb. |
| `surface` | `timeout` | `timeout=60.0` | host | Emergency surface: set depth to 0 m and hold until reached. Bypasses command_active gate so it works during a running mission. |
| `turn` | `target`, `timeout`, `settle` | `timeout=30.0`, `settle=0.0` | on the board | Rotate to absolute heading `target` degrees (0-360) via shortest arc. Direction (left/right) is chosen automatically. |
| `unlock_heading` | — | — | host | Stop the heading-lock streamer, send neutral. |
| `vision_align` | `camera`, `target_class`, `axes`, `offset_lat`, `offset_yaw`, `offset_depth`, `err_px`, `duration`, `gain`, `gain_lat`, `gain_yaw`, `brake_off`, `brake_gain`, `hold_s`, `hold_through_loss`, `fire_channels`, `fire_t`, `kp_lat`, `kp_yaw`, `kp_depth`, `lost_grace_s`, `align_stable_frames`, `lock_target`, `ctrl_conf`, `range_gain_floor`, `ki_lat`, `coast_s`, `lock_s`, `fwd_fill`, `mode`, `kp_forward`, `settle_px`, `depth_step`, `fire_pass_enabled`, `hold_heading`, `surge_sign`, `max_depth_m`, `depth_ceiling_m`, `fire_gap`, `standoff_max_tilt_deg`, `tool` | `camera=forward`, `target_class=`, `axes=`, `offset_lat=0.0`, `offset_yaw=0.0`, `offset_depth=0.0`, `err_px=40.0`, `duration=20.0`, `gain=30.0`, `gain_lat=0.0`, `gain_yaw=0.0`, `brake_off=False`, `brake_gain=0.0`, `hold_s=0.0`, `hold_through_loss=False`, `fire_channels=`, `fire_t=0.0`, `kp_lat=60.0`, `kp_yaw=60.0`, `kp_depth=0.05`, `lost_grace_s=1.0`, `align_stable_frames=3.0`, `lock_target=False`, `ctrl_conf=0.0`, `range_gain_floor=1.0`, `ki_lat=0.0`, `coast_s=0.8`, `lock_s=1.0`, `fwd_fill=0.0`, `mode=area`, `kp_forward=200.0`, `settle_px=0.0`, `depth_step=0.0`, `fire_pass_enabled=False`, `hold_heading=False`, `surge_sign=0.0`, `max_depth_m=0.0`, `depth_ceiling_m=0.0`, `fire_gap=0.0`, `standoff_max_tilt_deg=0.0`, `tool=` | host | Centre target_class on the active axes (CSV of lat,yaw,depth) each at its signed pixel offset (offset_lat/yaw/depth; 0=centre). Aligned when every active axis is within err_px. gain caps speed; gain_lat/gain_yaw override the cap on the lat/yaw axis (0 = inherit gain) so e.g. yaw can micro-align slowly while lateral stays brisk (depth rate is set by depth_step, not a %% cap). On arrival the lateral inertia is braked (reverse-kick) so the hull stops square; brake_off=true coasts, brake_gain scales the kick. A gently-converged lock exits with ~0 momentum and is not kicked. hold_s>0 turns it into an ACTIVE station-keep: once centred it keeps correcting for hold_s seconds (fighting water inertia, e.g. to hold a torpedo-hole lock steady for the shot) before exiting -- budget duration >= approach + hold_s. fire_channels (CSV e.g. "1,2") fires those payload channels ONCE mid-hold, fire_t seconds into the hold (0 = at hold start), on a background thread so the loop keeps correcting while the shot leaves -- so a torpedo launches while still glued to the hole (no align-then-fire drift). Requires fire_t < hold_s (else clamped to 0). On duration expiry it logs NOT-aligned and the mission continues. hold_through_loss=true coasts on target loss (set by the DSL when no fallback search is supplied). lock_target=true locks onto the acquired target (steers to the box nearest the last-accepted centre, not the largest) so a second hole / spurious box cannot steal the aim. ctrl_conf is a control-side min score to accept a box. range_gain_floor softens lat/depth gain when the bbox fills the frame (close) to stop overshoot. ki_lat adds a lateral integral during the hold to null a steady current. fwd_fill>0 adds a forward range-hold axis (mode=area/width/height, kp_forward gain): align ALSO drives forward to that fill %% and holds the standoff -- one verb does forward-standoff + lat/depth + hold + mid-hold fire (the torpedo standoff shot). The fire is gated on the standoff too. fwd_fill=0 (default) = no forward axis. settle_px>0 = SETTLE GATE: only declare aligned once the hull is in-band AND barely moving (|Δerr|<=settle_px) so it ends settled on target like vision_move (not mid-pass through the band); 0=off. depth_step = per-update depth-setpoint resolution (m, 0.02..0.10; 0=default 0.02): depth moves SLOWLY in these steps + freezes in the deadband so ArduSub settles (no z-wobble). fire_pass_enabled = fire the payload at command end even if never fully aligned, as long as the target was seen live+recently (a guaranteed partial-points shot). hold_heading = widen the heading-lock deadband during the hold so the launcher heading holds steady (no terminal yaw jitter) when yaw is released to the lock. DOWNWARD CAMERA (camera=downward/sim_bottom): the frame rotates -- image-X still drives Ch6 lateral, but the depth axis (image-Y) drives Ch5 SURGE fore/aft (two-sided, braked), and fwd_fill DESCENDS the ALT_HOLD depth to that bbox fill using the depth_step logic (proportional, deadband-frozen, one-sided). Bounded by max_depth_m (deepest) and depth_ceiling_m (shallowest -- surface guard so alignment cannot lift the hull out of the water). surge_sign (+1/-1) flips fore/aft for the mount (verify DISARMED). fire drops droppers (3/4). fire_gap spaces multi-channel shots (fire=[1,4]) apart in seconds (solenoid needs the gap). See the bin task. |
| `vision_move` | `camera`, `target_class`, `fwd_fill`, `mode`, `maintain_px`, `maintain_on`, `hold_s`, `err_px`, `duration`, `gain`, `gain_lat`, `brake_off`, `brake_gain`, `hold_through_loss`, `kp_forward`, `kp_lat`, `lost_grace_s`, `range_gain_floor`, `coast_s`, `lock_s` | `camera=forward`, `target_class=`, `fwd_fill=95.0`, `mode=area`, `maintain_px=0.0`, `maintain_on=False`, `hold_s=0.0`, `err_px=40.0`, `duration=20.0`, `gain=30.0`, `gain_lat=0.0`, `brake_off=False`, `brake_gain=0.0`, `hold_through_loss=False`, `kp_forward=200.0`, `kp_lat=60.0`, `lost_grace_s=1.0`, `range_gain_floor=1.0`, `coast_s=0.8`, `lock_s=1.0` | host | Drive forward toward target_class. fwd_fill > 0 stops once the bbox fills that %% of the frame (mode = area/width/height; height for tall slalom). fwd_fill <= 0 (e.g. --fwd_fill -1, or DSL move(fwd=None)) is PASS-THROUGH: drive until the target is seen and then leaves the frame, plus a commit overshoot (hold_s, else ~2s) to carry the hull through a gate. maintain_on holds a maintain_px lateral offset while moving; depth and yaw are left to ArduSub / heading lock. gain caps forward speed; gain_lat overrides the cap on the maintain strafe (0 = inherit gain). On a fill-stop arrival the forward (and maintain) inertia is braked so the hull halts in front of the target instead of creeping in; PASS-THROUGH never brakes (it must coast through the gate). brake_off=true coasts, brake_gain scales the kick. Does NOT re-centre. |
| `yaw_left` | `target`, `timeout`, `settle` | `timeout=30.0`, `settle=0.0` | on the board | Sharp pivot left by `target` degrees within `timeout` s. |
| `yaw_right` | `target`, `timeout`, `settle` | `timeout=30.0`, `settle=0.0` | on the board | Sharp pivot right by `target` degrees within `timeout` s. |

Refused on srot: `arc`, `lock_heading`, `move_back_dist`, `move_forward_dist`, `move_lateral_dist`, `style_yaw` — from `srot_fc.UNSUPPORTED_VERBS`. The board has no such primitive, and faking one host-side is how a mission comes to believe it moved a distance it never moved.

## 2. Executables — each package's `setup.py`

| run it with | module |
|---|---|
| `ros2 run mongla_localization localization_node` | `mongla_localization.localization_node:main` |
| `ros2 run mongla_localization pnp_node` | `mongla_localization.pnp_node:main` |
| `ros2 run mongla_localization pose_fuse_node` | `mongla_localization.pose_fuse_node:main` |
| `ros2 run mongla_localization course_survey` | `mongla_localization.course_survey:main` |
| `ros2 run mongla_manager start` | `mongla_manager.auv_manager_node:main` |
| `ros2 run mongla_manager auv_manager` | `mongla_manager.auv_manager_node:main` |
| `ros2 run mongla_manager auv_manager_node` | `mongla_manager.auv_manager_node:main` |
| `ros2 run mongla_manager bringup_check` | `mongla_manager.bringup_check:main` |
| `ros2 run mongla_manager connect` | `mongla_manager.srot_connect:main` |
| `ros2 run mongla_manager flare_order` | `mongla_manager.flare_order_send:main` |
| `ros2 run mongla_manager autotune` | `mongla_manager.srot_autotune:main` |
| `ros2 run mongla_sensors sensors_node` | `mongla_sensors.sensors_node:main` |
| `ros2 run mongla_vision camera_node` | `mongla_vision.camera_node:main` |
| `ros2 run mongla_vision detector_node` | `mongla_vision.detector_node:main` |
| `ros2 run mongla_vision detector_dual_node` | `mongla_vision.detector_dual_node:main` |
| `ros2 run mongla_vision tracker_node` | `mongla_vision.tracker_node:main` |
| `ros2 run mongla_vision lock_node` | `mongla_vision.lock_node:main` |
| `ros2 run mongla_vision vision_node` | `mongla_vision.vision_node:main` |
| `ros2 run mongla_vision vision_check` | `mongla_vision.utils.check_pipeline:main` |
| `ros2 run mongla_vision water_check` | `mongla_vision.utils.water_check:main` |
| `ros2 run mongla_vision vision_thrust_check` | `mongla_vision.utils.check_thrust:main` |
| `ros2 run mongla_vision tracker_check` | `mongla_vision.utils.check_tracker:main` |
| `ros2 run mongla_vision vision_display` | `mongla_vision.utils.display_node:main` |
| `ros2 run mongla_vision export_engine` | `mongla_vision.utils.export_engine:main` |
| `ros2 run mongla_vision switch_camera` | `mongla_vision.utils.switch_camera:main` |
| `ros2 run mongla_vision calibrate` | `mongla_vision.calibration.guide:main` |
| `ros2 run mongla_vision calibrate_solve` | `mongla_vision.calibration.solver:main` |
| `ros2 run mongla_vision depth_estimation_node` | `mongla_vision.depth.depth_estimation_node:main` |
| `ros2 run mongla_vision distance_estimation_node` | `mongla_vision.flow.distance_estimation_node:main` |
| `ros2 run mongla_vision flow_node` | `mongla_vision.flow.flow_node:main` |
| `ros2 run mongla_vision mission_web` | `mongla_vision.web.mission_web_node:main` |

## 3. Launch arguments — `DeclareLaunchArgument`

| argument | default | what it does |
|---|---|---|
| `mode` | `pool` | Connection mode: pool|sim|auto|desk|laptop |
| `yaw_source` | `mavlink_ahrs` | Yaw source: mavlink_ahrs|dvl|bno085|bno085_dvl |
| `flight_controller` | `srot` | Autopilot backend: srot|pixhawk (srot = the SROT/Hengla board over USB serial; pixhawk = the ArduSub/BlueOS path) |
| `mav_device` | `` | SROT serial device, '' = autodetect. A path (/dev/serial/by-id/...) or any pymavlink connection string |
| `payload_channels` | `` | OPTIONAL labels for the log, "<board_channel>:<name>", e.g. "9:torpedo_1, 11:dropper_1". Labels ONLY -- fire(N) always addresses board channel N; nothing here routes a shot |
| `allow_fw_behaviour_mismatch` | `false` | Arm against firmware older than FW_BEHAVIOUR_REV_REQUIRED. Accepts an un-braked stop -- leave false |
| `dvl_host` | `192.168.2.201` |  |
| `dvl_port` | `9000` |  |
| `bno085_port` | `auto` | BNO085 device, or "auto" to scan USB VID/PID. |
| `baro_calibration` | `true` | Re-zero the barometer at the surface. TRUE on the pool hull. The simulator passes false: SITL ACKs the calibration and then stops tracking depth. |
| `payload_port` | `auto` | Payload board device, or "auto" to scan USB VID/PID. |
| `dvl_auto_connect` | `true` | Auto-connect DVL at startup (true|false) |
| `vision` | `false` | Start camera + detector alongside manager |
| `vision_stack` | `pi` | pi = both cameras + calibration (vision_pi.launch.py, the vehicle); generic = single camera, no calibration (vision.launch.py, dev/CUDA). |
| `vision_profile` | `fast` | vision_stack:=pi -- vision_pi's own profile argument (its `vision:=`). Renamed here because `vision` is this file's boolean on/off switch. |
| `fwd_model` | `gate_rescue_repair` | vision_stack:=pi -- forward-camera model stem. |
| `dwn_model` | `bin_fire_blood` | vision_stack:=pi -- downward-camera model stem. |
| `fwd_classes` | `` | vision_stack:=pi -- forward class allowlist (empty = the model sidecar's full set). |
| `dwn_classes` | `` | vision_stack:=pi -- downward class allowlist. |
| `localization` | `true` | Run the invariant filter (mongla_localization). Read-only: it publishes /mongla/odom and commands nothing, so it is on by default. Set false to take it off the graph. |
| `flow` | `false` | vision_stack:=pi -- start flow_node (downward-camera velocity). Pairs with the manager's position_source:=flow. |
| `pool_depth_m` | `nan` | flow:=true -- metres from the DOWNWARD camera to the floor. Required: without it flow_node publishes quality 0 and refuses, by design. |
| `velocity_uplink` | `false` | manager: RIEKF body velocity -> board (VISION_SPEED_ESTIMATE). Needs fw PR #23 to act. |
| `position_uplink` | `false` | manager: RIEKF pose -> board (VISION_POSITION_ESTIMATE). |
| `mixer_aware` | `true` | manager: srot vision frames fitted to the mixer yaw-first + saturation-aware anti-windup. |
| `zupt` | `true` | localization: zero-velocity updates when still. |
| `demand_aid` | `true` | localization: velocity from commanded demand when the floor goes blank. |
| `retrodict` | `false` | localization: apply flow/depth/fixes at the instant they describe, replaying later events. |
| `use_yaw` | `false` | localization: fuse the landmark heading anchor. |
| `caustics` | `true` | flow: sun-caustic erosion + bare-floor refusal. |
| `lane_lines` | `false` | flow: lane-line heading (mod 180) yaw bound. |
| `tile_m` | `0.0` | flow: pool tile size in metres; 0 = tile grating (height + yaw bound) OFF. |
| `medium` | `water` | The medium the VEHICLE is in. Read by flow_node, lock_node AND pnp_node -- one value, one argument. `air` for a bench run, or the scale is off by ~1.33. |
| `lock_class` | `` | vision_stack:=pi, lock:=true -- the class the ladder follows. Empty = any class, and no 6-DoF pose. |
| `lock` | `true` | vision_stack:=pi -- start lock_node (follower + XFeat anchor continuity ladder). |
| `paused` | `true` | vision_stack:=pi -- start both detectors paused; the mission resumes the one it needs. false = both infer and compete. |
| `camera` | `forward` | Camera ROLE -- names the topics and nodes (forward|downward). Change `camera_profile`, not this, to point a role at other hardware. |
| `camera_profile` | `pi_forward` | Camera HARDWARE profile (CAMERA_PROFILES in mongla_vision/config.py -- the loaded copy). Default pi_forward matches flight_controller:=srot (Pi box). On the Jetson pass forward. |
| `model` | `gate_flare_medium_100ep` | Single-model: gate_flare_medium_100ep|gate_nano_100ep|gate_medium_100ep|flare_medium_100ep|yolov11n (ROBOSUB-tested pretrained, sim/bench)|yolo26_nano_pretrained |
| `models` | `` | Multi-model registry (CSV name=stem): "gate=gate_nano_100ep,combined=gate_flare_medium_100ep" |
| `active_model` | `` | Registry key to start with (requires models:="..." to be set) |
| `classes` | `gate` | CSV class names for detector |
| `conf` | `0.15` | Detector confidence floor. 0.15 = Hailo INT8 operating point; pass 0.35 for the generic CUDA stack. |
| `imgsz` | `640` | Inference square size. NOTE: a TensorRT .engine bakes imgsz at export -- this only re-scales the .pt fallback. For TRT, re-export to match (export_engine --all --imgsz <N>), then imgsz:=<N>. |
| `max_det` | `100` | Post-NMS detection cap (runtime; lower toward ~10 if NMS is the FPS bottleneck on busy frames). |
| `viewer` | `false` | Open vision_display (OpenCV viewer) alongside vision pipeline |
| `foxglove` | `false` | Start foxglove_bridge (WebSocket telemetry on foxglove_port). Off the mission path -- pure viz. Needs ros-humble-foxglove-bridge installed. Connect the Foxglove desktop app to ws://<jetson-ip>:<foxglove_port>. |
| `foxglove_port` | `8765` | foxglove_bridge WebSocket port |
| `cameras` | `both` | rig: both (vision_dual) | forward | downward (single camera via vision.launch.py) |
| `web_port` | `8090` | HTTP port for the mission console + SSE |
| `video_port` | `8080` | web_video_server MJPEG port |
| `no_browser` | `false` | true = do not auto-open the browser |
| `paused` | `false` |  |
| `debug_image_hz` | `15.0` |  |
| `viewer` | `false` | also open the OpenCV HUD window (web console is primary) |
| `fwd_device` | `0` |  |
| `dwn_device` | `4` |  |
| `fwd_device_path` | `` |  |
| `dwn_device_path` | `` |  |
| `fwd_model` | `gate_rescue_repair` |  |
| `fwd_models` | `` |  |
| `fwd_classes` | `gate,rescue,repair` |  |
| `fwd_conf` | `0.35` |  |
| `fwd_model_conf` | `` |  |
| `dwn_model` | `bin_fire_blood` |  |
| `dwn_models` | `` |  |
| `dwn_classes` | `fire,blood` |  |
| `dwn_conf` | `0.35` |  |
| `dwn_model_conf` | `` |  |
| `fwd_video` | `` |  |
| `dwn_video` | `` |  |
| `imgsz` | `640` |  |
| `max_det` | `100` |  |
| `tracking` | `true` |  |
| `fwd_video` | `` | Forward-camera video (e.g. a gate dataset clip). At least one of fwd_video / dwn_video is required. |
| `dwn_video` | `` | Downward-camera video (e.g. a bin dataset clip). |
| `fwd_model` | `gate_rescue_repair` | YOLO model stem for the forward detector. |
| `fwd_classes` | `gate,rescue,repair` | Class filter for the forward detector. |
| `dwn_model` | `bin_fire_blood` | YOLO model stem for the downward detector. |
| `dwn_classes` | `fire,blood` | Class filter for the downward detector. |
| `conf` | `0.35` | Detection confidence threshold (both detectors). |
| `loop` | `true` | Loop both videos at EOF. true = detections keep flowing for verb/gain tuning; false = single pass for a clean mission-sequence run (a loop re-shows the target at EOF, which can re-trigger detected() acquisition mid-run). |
| `viewer` | `true` | Open the HUD (viewer:=false = headless autonomous run). |
| `tracking` | `true` |  |
| `imgsz` | `640` |  |
| `max_det` | `100` |  |
| `profile` | `` | Camera HARDWARE profile (CAMERA_PROFILES) (pi_forward|pi_downward|forward|...). Empty = same as `camera`. Set this to keep the role name while changing which unit it opens. |
| `lock` | `false` | Run the lock ladder (follower + XFeat anchor), publishing <ns>/lock. Costs ~23 % of the detection rate, measured. Control ignores it until vision.lock_s > 0 -- turn both on together. |
| `lock_class` | `` |  |
| `camera` | `forward` | Camera profile (forward|downward|sim_front|laptop|...). Drives node names: mongla_detector_<camera>. |
| `device` | `-1` | /dev/videoN index; -1 = use profile default |
| `device_path` | `` | override device (empty = auto /dev/mongla_cam_<camera> symlink if present, else int `device`) |
| `width` | `640` |  |
| `height` | `480` |  |
| `fps` | `0` | 0 = use the camera profile fps |
| `video_file` | `` | Path to a video file; when set, replaces the live webcam |
| `topic` | `` | ROS image topic to consume (Gazebo / re-published stream). Ignored when video_file is set. |
| `loop` | `true` | Loop the video file at EOF (video_file only) |
| `model` | `yolov11n` | Model stem (models/) or .pt path (single-model mode). Pool: gate_rescue_repair / gate_flare_medium_100ep. |
| `models` | `` | CSV name=stem registry for hot model switching: "gate=gate_nano_100ep,combined=gate_flare_medium_100ep" |
| `active_model` | `` | Registry key to start with (requires models:="...") |
| `classes` | `` | CSV class filter; empty = all model classes |
| `conf` | `0.35` |  |
| `iou` | `0.5` |  |
| `device_cls` | `cuda:0` | Inference device for YOLO (cuda:0 | cpu) |
| `imgsz` | `640` | Inference square size. NOTE: a TensorRT .engine bakes imgsz at export -- this arg only re-scales the .pt fallback. To bench a size on TRT, re-export to match: export_engine --all --imgsz <N>, then imgsz:=<N> here. |
| `max_det` | `100` | Post-NMS detection cap (runtime; lower toward ~10 if NMS is the FPS bottleneck on busy frames). |
| `paused` | `false` | Start the detector paused (resume_detector(camera) per task) |
| `debug_image_hz` | `10.0` | Annotated image_debug publish rate (raise toward inference FPS for a smoother web/mission_web stream) |
| `viewer` | `true` | Open the OpenCV vision_display HUD (viewer:=false = headless) |
| `tracking` | `true` | Start tracker_node (Roboflow OC-SORT/ByteTrack + Kalman) |
| `tracker_type` | `ocsort` | Tracker engine: ocsort (default) | bytetrack | legacy_bytetrack |
| `track_buffer` | `30` |  |
| `min_hits` | `1` |  |
| `max_predict_s` | `1.5` |  |
| `depth` | `false` | Start depth_estimation_node (monocular vis_range) |
| `depth_model` | `` | DA V2-Small ONNX path; empty = bbox-area fallback |
| `distance` | `false` | Start the optical-flow distance node (calc_distance). |
| `pool_depth_m` | `4.0` | Water column surface->floor (m); metric scale for flow. |
| `camera_focal_px` | `500.0` | Camera f_px (intrinsics calibration; distance scale rides on it). |
| `hud_distance` | `true` | HUD pre-arms the distance panel. |
| `fwd_device` | `0` | /dev/videoN index for the forward camera — LAST-RESORT fallback, only used on a box with no /dev/mongla_cam_* symlink (dev box). On the Jetson the port-stable symlink wins automatically (see device_path below). |
| `dwn_device` | `4` | /dev/videoN index for the downward camera (dev-box fallback; the Jetson symlink wins automatically) |
| `fwd_device_path` | `` | override device for forward (empty = auto /dev/mongla_cam_forward) |
| `dwn_device_path` | `` | override device for downward (empty = auto /dev/mongla_cam_downward) |
| `fwd_model` | `gate_rescue_repair` | Single YOLO model stem (used only when fwd_models is empty) |
| `fwd_models` | `` | CSV of model stems for runtime switching (e.g. gate_rescue_repair,slalom_red_pipe,torpedo_blood_hole). Empty = single fwd_model. |
| `fwd_classes` | `gate,rescue,repair` | Class filter for the forward detector |
| `dwn_model` | `bin_fire_blood` | Single YOLO model stem (used only when dwn_models is empty) |
| `dwn_models` | `` | CSV of model stems for runtime switching on the downward detector. Empty = single dwn_model. |
| `dwn_classes` | `fire,blood` | Class filter for the downward detector |
| `fwd_conf` | `0.35` | Uniform conf for the forward detector (every model unless overridden below) |
| `dwn_conf` | `0.35` | Uniform conf for the downward detector |
| `fwd_model_conf` | `` | Per-model conf overrides on forward, CSV name=conf (e.g. torpedo_blood_hole=0.55) |
| `dwn_model_conf` | `` | Per-model conf overrides on downward, CSV name=conf |
| `device_cls` | `cuda:0` | Inference device for YOLO (cuda:0 | cpu) |
| `imgsz` | `640` | Inference square size (both detectors). NOTE: a TensorRT .engine bakes imgsz at export -- this only re-scales the .pt fallback. For TRT, re-export to match: export_engine --all --imgsz <N>, then imgsz:=<N> here. |
| `max_det` | `100` | Post-NMS detection cap, both detectors (runtime; lower toward ~10 if NMS is the FPS bottleneck on busy frames). |
| `paused` | `true` | Start both detectors paused (resume_detector per task) |
| `debug_image_hz` | `10.0` | Annotated image_debug publish rate (the browser-stream smoothness knob; raise toward inference FPS for a web viewer) |
| `viewer` | `true` |  |
| `tracking` | `true` |  |
| `tracker_type` | `ocsort` | Tracker engine: ocsort (default) | bytetrack | legacy_bytetrack |
| `fwd_topic` | `` | Subscribe this ROS image topic instead of a webcam. Set by the simulator to /mongla/sim/front_camera/image_raw. |
| `dwn_topic` | `` | Subscribe this ROS image topic instead of a webcam. Set by the simulator to /mongla/sim/bottom_camera/image_raw. |
| `fwd_video` | `` | Video file for the FORWARD camera (e.g. a gate clip). Set => forward runs off the file; empty => live webcam. |
| `dwn_video` | `` | Video file for the DOWNWARD camera (e.g. a bin clip). Set => downward runs off the file; empty => live webcam. |
| `fwd_loop` | `true` | Loop the forward video at EOF (video source only). |
| `dwn_loop` | `true` | Loop the downward video at EOF (video source only). |
| `distance` | `false` | Start the downward optical-flow distance node (enables calc_distance start/stop). |
| `pool_depth_m` | `4.0` | Water column surface->floor (m); metric scale for flow. |
| `camera_focal_px` | `500.0` | Downward camera f_px (intrinsics calibration; scale rides on it). |
| `hud_distance` | `true` | HUD pre-arms the distance panel (shows on calc_distance). |
| `fwd_profile` | `pi_forward` |  |
| `dwn_profile` | `pi_downward` |  |
| `fwd_model` | `gate_rescue_repair` |  |
| `dwn_model` | `bin_fire_blood` |  |
| `fwd_models` | `` |  |
| `dwn_models` | `` |  |
| `fwd_active` | `` |  |
| `dwn_active` | `` |  |
| `contours` | `true` |  |
| `fwd_classes` | `` |  |
| `dwn_classes` | `` |  |
| `conf` | `0.15` | INT8 operating point. NOT the CUDA path's 0.35-0.45 -- see the module docstring. |
| `max_det` | `100` |  |
| `vision` | `fast` |  |
| `preprocess` | `auto` |  |
| `range_crop` | `-1` |  |
| `preprocess_clip` | `0.0` |  |
| `imgsz` | `640` |  |
| `paused` | `true` | Start both detectors paused. The mission resumes the one it needs; leaving BOTH live makes them compete for the chip (~35 Hz each instead of ~98). paused:=false to watch both streams with no mission. |
| `viewer` | `false` |  |
| `tracking` | `true` |  |
| `tracker_type` | `ocsort` |  |
| `kalman_adaptive_noise` | `true` |  |
| `fwd_frame_rate` | `30.0` |  |
| `dwn_frame_rate` | `32.0` |  |
| `dwn_calibration` | `` |  |
| `fwd_calibration` | `` |  |
| `flow` | `false` |  |
| `lock` | `false` | Run the lock ladder (follower + XFeat anchor) on the forward camera, publishing <ns>/lock. Control ignores it until vision.lock_s > 0. |
| `lock_class` | `` | Class the ladder locks onto. Empty = whatever the detector is publishing. |
| `dwn_lock_class` | `` | Class the DOWNWARD ladder follows. Empty = any class, and no 6-DoF pose (the geometry table is keyed by class). |
| `pool_depth_m` | `nan` | Water depth in metres. REQUIRED with flow:=true -- the node refuses to publish velocity without it. |
| `tile_m` | `0.0` | Floor tile pitch in metres, measured on deck. 0 = OFF. Sets height from the floor and publishes the grid angle that bounds yaw drift. A wrong value rescales every height silently -- measure it. |
| `caustics` | `true` | flow: erode sun caustics and refuse a bare floor under them (false = always track the raw floor; A/B switch) |
| `bank_forward` | `` | checkpoint bank (.npz) preloaded into the FORWARD lock node, from tools/build_practice_bank.py. Empty learns live. A preloaded reference is trusted exactly like a live one -- it clears MIN_INLIERS or it does not answer. |
| `bank_downward` | `` | checkpoint bank (.npz) preloaded into the DOWNWARD lock node -- places rather than props. |
| `lane_lines` | `false` | Read the lane line (heading mod 180) for the yaw drift bound. OFF: a path marker is also a dark band. Enable on a floor with lanes and no markers. |
| `medium` | `water` | The medium the VEHICLE is in. 'water' engages the flat-port rectification in flow_node, lock_node AND pnp_node; 'air' for a dry bench run. Was 'flow_medium' when only the flow node read it -- one value, so one argument, or the three drift apart and each reports a plausible number. |
| `fwd_fps` | `0` |  |
| `dwn_fps` | `0` |  |
| `fwd_publish_hz` | `40` |  |
| `dwn_publish_hz` | `0` |  |
| `replay` | `false` | Consume image_raw from a RECORDED BAG instead of a camera. Build no camera, keep every detector, and let `ros2 bag play` drive the graph. This is how a recorded session becomes a regression fixture: the detector, tracker, lock ladder and checkpoint bank all run for real, on input that never changes. |
| `fwd_device_path` | `` |  |
| `dwn_device_path` | `` |  |

## 4. Node parameters — `declare_parameter`

**`mongla_localization/localization_node.py`** — 15 parameters

| parameter | default |
|---|---|
| `flow_camera` | `'downward'` |
| `flow_sigma` | `0.05` |
| `depth_sigma` | `0.02` |
| `yaw_sigma_deg` | `2.0` |
| `fix_sigma` | `0.5` |
| `attitude_sigma_deg` | `0.5` |
| `zupt_sigma` | `0.01` |
| `zupt` | `True` |
| `grid_sigma_deg` | `1.0` |
| `grid_max_correction_deg` | `20.0` |
| `use_yaw` | `False` |
| `demand_tau_s` | `1.0` |
| `demand_aid` | `True` |
| `retrodict` | `False` |
| `retro_horizon_s` | `2.0` |

**`mongla_localization/pnp_node.py`** — 5 parameters

| parameter | default |
|---|---|
| `camera` | `'forward'` |
| `medium` | `'water'` |
| `ambiguity_max` | `AMBIGUITY_MAX` |
| `max_reproj_px` | `MAX_REPROJ_PX` |
| `variant` | `''` |

**`mongla_localization/pose_fuse_node.py`** — 6 parameters

| parameter | default |
|---|---|
| `camera` | `'forward'` |
| `window_s` | `WINDOW_S` |
| `tol_deg` | `CLUSTER_TOL_DEG` |
| `min_poses` | `MIN_POSES` |
| `max_ambiguity` | `0.9` |
| `max_reproj_px` | `10.0` |

**`mongla_manager/auv_manager_node.py`** — 29 parameters

| parameter | default |
|---|---|
| `mode` | `DEFAULT_MODE` |
| `mav_device` | `''` |
| `smooth_yaw` | `False` |
| `smooth_translate` | `False` |
| `yaw_source` | `'mavlink_ahrs'` |
| `position_source` | `'none'` |
| `bno085_port` | `'auto'` |
| `bno085_baud` | `115200` |
| `payload_port` | `'auto'` |
| `baro_calibration` | `True` |
| `record` | `''` |
| `vision_uplink_camera` | `''` |
| `vision_uplink_class` | `''` |
| `vision_uplink_hz` | `25.0` |
| `vision_uplink_medium` | `'water'` |
| `velocity_uplink` | `False` |
| `position_uplink` | `False` |
| `payload_channels` | `''` |
| `payload_fire_map` | `''` |
| `nucleus_dvl_host` | `'192.168.2.201'` |
| `nucleus_dvl_port` | `9000` |
| `nucleus_dvl_password` | `'nortek'` |
| `dvl_auto_connect` | `True` |
| `dvl_retry_s` | `5.0` |
| `debug` | `False` |
| `flight_controller` | `DEFAULT_FLIGHT_CONTROLLER` |
| `allow_fw_behaviour_mismatch` | `False` |
| `srot_telemetry_period_s` | `2.0` |
| `allow_saturated_depth_arm` | `False` |

**`mongla_manager/test_preflight_never_opens_the_autopilot.py`** — 1 parameters

| parameter | default |
|---|---|
| `flight_controller` | `DEFAULT_FLIGHT_CONTROLLER` |

**`mongla_sensors/sensors_node.py`** — 6 parameters

| parameter | default |
|---|---|
| `yaw_source` | `'mavlink_ahrs'` |
| `bno085_port` | `'/dev/ttyACM0'` |
| `bno085_baud` | `115200` |
| `calibrate` | `False` |
| `mavlink_url` | `'udpin:0.0.0.0:14550'` |
| `print_period_s` | `0.5` |

**`mongla_vision/camera_node.py`** — 15 parameters

| parameter | default |
|---|---|
| `profile` | `''` |
| `source` | `''` |
| `name` | `''` |
| `topic` | `''` |
| `device` | `-1` |
| `device_path` | `''` |
| `width` | `640` |
| `height` | `480` |
| `fps` | `0` |
| `frame_id` | `''` |
| `calibration` | `''` |
| `publish_rate_hz` | `0` |
| `path` | `''` |
| `loop` | `True` |
| `discover_on_start` | `False` |

**`mongla_vision/detector_dual_node.py`** — 2 parameters

| parameter | default |
|---|---|
| `device` | `-1` |
| `replay` | `False` |

**`mongla_vision/detector_node.py`** — 25 parameters

| parameter | default |
|---|---|
| `camera` | `'laptop'` |
| `image_topic` | `''` |
| `model_path` | `'yolov11n'` |
| `models` | `''` |
| `active_model` | `''` |
| `device` | `'cuda:0'` |
| `half` | `True` |
| `conf` | `0.35` |
| `assoc_conf` | `0.0` |
| `model_conf` | `''` |
| `iou` | `0.5` |
| `imgsz` | `640` |
| `max_det` | `100` |
| `classes` | `'person'` |
| `publish_contours` | `True` |
| `masks` | `True` |
| `publish_debug_image` | `True` |
| `debug_image_hz` | `5.0` |
| `alignment_deadband` | `0.05` |
| `paused` | `False` |
| `direct_feed` | `True` |
| `preprocess` | `'auto'` |
| `preprocess_clip` | `0.0` |
| `range_crop` | `-1` |
| `vision_profile` | `''` |

**`mongla_vision/lock_node.py`** — 16 parameters

| parameter | default |
|---|---|
| `camera` | `'forward'` |
| `target_class` | `''` |
| `follow` | `True` |
| `anchor` | `False` |
| `anchor_model` | `''` |
| `anchor_bank` | `''` |
| `loop_closure` | `_lc.defaults(` |
| `place_period_s` | `_lc.defaults(` |
| `place_travel_m` | `_lc.defaults(` |
| `pool_depth_m` | `_lc.defaults(` |
| `publish_hz` | `0.0` |
| `anchor_hz` | `3.0` |
| `full_authority_s` | `FULL_AUTHORITY_S` |
| `zero_authority_s` | `ZERO_AUTHORITY_S` |
| `target_width_m` | `0.0` |
| `medium` | `'water'` |

**`mongla_vision/tracker_node.py`** — 16 parameters

| parameter | default |
|---|---|
| `camera` | `'laptop'` |
| `tracker_type` | `'ocsort'` |
| `frame_rate` | `20.0` |
| `track_buffer` | `60` |
| `min_hits` | `1` |
| `iou_threshold` | `0.2` |
| `track_activation_threshold` | `0.40` |
| `high_conf_det_threshold` | `0.6` |
| `detector_conf` | `0.0` |
| `classes` | `''` |
| `enable_kalman` | `True` |
| `kalman_process_noise` | `0.1` |
| `kalman_measurement_noise` | `1.0` |
| `kalman_adaptive_noise` | `True` |
| `max_predict_s` | `1.5` |
| `frame_rate_warn_ratio` | `1.5` |

**`mongla_vision/vision_node.py`** — 14 parameters

| parameter | default |
|---|---|
| `profile` | `'laptop'` |
| `source` | `''` |
| `topic` | `''` |
| `device` | `'cuda:0'` |
| `model_path` | `'yolov11n'` |
| `classes` | `'person'` |
| `conf` | `0.35` |
| `iou` | `0.5` |
| `imgsz` | `640` |
| `half` | `False` |
| `publish_debug_image` | `True` |
| `debug_image_hz` | `10.0` |
| `tick_hz` | `30.0` |
| `alignment_deadband` | `0.05` |

**`mongla_vision/test_assoc_conf.py`** — 1 parameters

| parameter | default |
|---|---|
| `assoc_conf` | `0.0` |

**`mongla_vision/test_camera_fps_override.py`** — 2 parameters

| parameter | default |
|---|---|
| `fps` | `30` |
| `fps` | `0` |

**`mongla_vision/test_composed_vision.py`** — 1 parameters

| parameter | default |
|---|---|
| `direct_feed` | `True` |

**`mongla_vision/test_flow_reads_the_floor.py`** — 1 parameters

| parameter | default |
|---|---|
| `tile_m` | `0.0` |

**`mongla_vision/test_preprocess.py`** — 1 parameters

| parameter | default |
|---|---|
| `preprocess` | `'auto'` |

**`mongla_vision/test_profiles.py`** — 1 parameters

| parameter | default |
|---|---|
| `range_crop` | `-1` |

**`mongla_vision/test_rangecrop.py`** — 1 parameters

| parameter | default |
|---|---|
| `range_crop` | `-1` |

**`mongla_vision/test_refraction_in_the_pose_path.py`** — 1 parameters

| parameter | default |
|---|---|
| `medium` | `'water'` |

## 5. The wire — `fc/srot_protocol.py`

| constant | value |
|---|---|
| `VEHICLE_SYSID` | `1` |
| `VEHICLE_COMPID` | `1` |
| `SOURCE_SYSID` | `255` |
| `SOURCE_COMPID` | `191` |
| `CMD_SROT_MOVE` | `31000` |
| `FW_BEHAVIOUR_REV_REQUIRED` | `10` |
| `GCS_FAILSAFE_MS` | `5000` |
| `MOVE_CRUISE_MAX` | `0.8` |
| `MOVE_YAW_RATE` | `45.0` |
| `MOVE_DEPTH_RATE` | `0.2` |
| `SROT_MOVE p1 codes` | `0=MOVE_FORWARD, 1=MOVE_BACK, 2=MOVE_STRAFE_L, 3=MOVE_STRAFE_R, 4=MOVE_TURN, 5=MOVE_DIVE, 6=MOVE_STOP, 7=MOVE_HOLD, 8=MOVE_STYLE, 9=MOVE_ARC` |
| `flight modes` | `0=STABILIZE, 1=ACRO, 2=DEPTH_HOLD, 9=SURFACE, 19=MANUAL, 20=MOTOR_DETECT, 21=AUTOTUNE, 22=MOTOR_TUNE, 23=AUTO, 100=STUNT, 101=PATTERN` |

## 6. The ROS surface

One action and one topic, on purpose:

| interface | type | notes |
|---|---|---|
| `/mongla/move` | `mongla_interfaces/action/Move` | one verb per goal |
| `/mongla/state` | `mongla_interfaces/msg/MonglaState` | armed, mode, yaw, depth, battery; **`NaN` when absent, never `0.0`**; published on change |

⛔ `/mongla/arm`, `/mongla/depth_cmd`, `Attitude.msg` and `RCOverride.msg` appear in older notes. **None of them exist.**

