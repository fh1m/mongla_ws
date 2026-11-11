# Command reference — generated from the code

> WARNING: **This file is generated.** Do not edit it by hand: run
> `python3 tools/ci/gen_reference.py`. A test fails if it has drifted from the source.
> Every row below was read out of the file named beside it, never written from memory.

**23 verbs.** On the SROT board **11** run as one `SROT_MOVE` primitive, which the board runs *and brakes* itself; the rest are host-side loops or single messages.

## 1. Verbs — `mongla_control/commands.py`

| verb | fields | defaults | on srot | what it does |
|---|---|---|---|---|
| `arm` | `timeout` | `timeout=15.0` | host | Arm the vehicle (motors hot). |
| `calc_distance` | `phase` | `phase=start` | host | Downward optical-flow distance bracket: phase='start' latches the axis + resets, 'stop' returns accumulated metres in final_value. |
| `calibrate_depth` | — | — | host | Re-zero the barometer at the surface so depth reads 0 before a dive. DISARMED + surface only; verifies the re-zero took. Auto-run by mission_reset; also standalone. |
| `disarm` | `timeout` | `timeout=20.0` | host | Disarm and confirm it took on the board's HEARTBEAT. Signals abort first, so a running verb stops. |
| `fire` | `fire_channel` | `fire_channel=0.0` | host | Activate payload BOARD channel N (1..16) -- the same n as SERVO{n}_ROLE, no host-side map. The board decides: a SWITCH channel fires, a PWM/SERVO channel is REFUSED (it is the on-board arm). `mongla_manager connect` lists which is which. |
| `head` | — | — | host | Read current heading (degrees) at execution time. Result is in final_value. Also works as a magic value in other CLI commands: --target head resolves to the live heading the moment the command runs. |
| `mission_reset` | — | — | host | Clear the abort flag, zero the demand, and re-zero depth if disarmed at the surface. First line of every mission. |
| `motor_test` | `target`, `gain`, `duration` | `target=1.0`, `gain=20.0`, `duration=2.0` | host | Spin ONE thruster (1-8) at `gain` percent for `duration` s. Requires ARMED. The board expires the test and DISARMS within 3 s of the last keep-alive, which is the safety mechanism -- there is no stop command. PROPS OFF OR VEHICLE RESTRAINED. |
| `move_back` | `duration`, `gain`, `settle` | `gain=80.0`, `settle=0.0` | on the board | Drive backward for `duration` s at `gain` percent thrust. |
| `move_forward` | `duration`, `gain`, `settle` | `gain=80.0`, `settle=0.0` | on the board | Drive forward for `duration` s at `gain` percent thrust. |
| `move_left` | `duration`, `gain`, `settle` | `gain=80.0`, `settle=0.0` | on the board | Strafe left for `duration` s at `gain` percent thrust. |
| `move_right` | `duration`, `gain`, `settle` | `gain=80.0`, `settle=0.0` | on the board | Strafe right for `duration` s at `gain` percent thrust. |
| `pause` | `duration` | `duration=2.0` | on the board | Hold station for `duration` s (the board's MOVE_HOLD). |
| `set_depth` | `target`, `timeout`, `settle` | `timeout=30.0`, `settle=0.0` | on the board | Hold absolute depth (`target` metres, negative below surface). |
| `set_mode` | `target_name`, `timeout` | `timeout=8.0` | host | Switch board mode: STABILIZE|DEPTH_HOLD|SURFACE|MANUAL|ACRO, and verify it took (a silent refusal looks like success). |
| `stop` | — | — | on the board | Halt now: the board's MOVE_STOP brake. |
| `style_roll` | `flips`, `timeout` | `flips=1`, `timeout=20.0` | on the board | Style: `flips` x 360 deg roll, run by the board (MOVE_STYLE). `timeout` bounds the whole manoeuvre. |
| `surface` | `timeout` | `timeout=60.0` | host | Emergency surface: brake, then the board's SURFACE mode (ascend and hold). Bypasses the busy gate, so it works during a running mission. |
| `turn` | `target`, `timeout`, `settle` | `timeout=30.0`, `settle=0.0` | on the board | Rotate to absolute heading `target` degrees (0-360) via shortest arc. Direction (left/right) is chosen automatically. |
| `vision_align` | `camera`, `target_class`, `axes`, `offset_lat`, `offset_yaw`, `offset_depth`, `err_px`, `duration`, `gain`, `gain_lat`, `gain_yaw`, `brake_off`, `brake_gain`, `hold_s`, `hold_through_loss`, `fire_channels`, `fire_t`, `kp_lat`, `kp_yaw`, `lost_grace_s`, `align_stable_frames`, `lock_target`, `ctrl_conf`, `range_gain_floor`, `ki_lat`, `coast_s`, `lock_s`, `fwd_fill`, `mode`, `kp_forward`, `settle_px`, `verify_off`, `fire_pass_enabled`, `surge_sign`, `fire_gap`, `standoff_max_tilt_deg`, `fire_max_tilt_deg`, `tool` | `camera=forward`, `target_class=`, `axes=`, `offset_lat=0.0`, `offset_yaw=0.0`, `offset_depth=0.0`, `err_px=40.0`, `duration=20.0`, `gain=30.0`, `gain_lat=0.0`, `gain_yaw=0.0`, `brake_off=False`, `brake_gain=0.0`, `hold_s=0.0`, `hold_through_loss=False`, `fire_channels=`, `fire_t=0.0`, `kp_lat=60.0`, `kp_yaw=60.0`, `lost_grace_s=1.0`, `align_stable_frames=3.0`, `lock_target=False`, `ctrl_conf=0.0`, `range_gain_floor=1.0`, `ki_lat=0.0`, `coast_s=0.8`, `lock_s=1.0`, `fwd_fill=0.0`, `mode=area`, `kp_forward=200.0`, `settle_px=0.0`, `verify_off=False`, `fire_pass_enabled=False`, `surge_sign=0.0`, `fire_gap=0.0`, `standoff_max_tilt_deg=0.0`, `fire_max_tilt_deg=0.0`, `tool=` | host | Centre target_class on the active axes (CSV of lat,yaw; on the downward camera also depth, which drives SURGE) each at its signed pixel offset (offset_lat/yaw/depth; 0=centre). Aligned when every active axis is within err_px. gain caps speed; gain_lat/gain_yaw override the cap per axis (0 = inherit gain). On arrival the lateral inertia is braked (reverse-kick) so the hull stops square; brake_off=true coasts, brake_gain scales the kick. hold_s>0 keeps correcting for hold_s seconds once centred (station-keep for a shot) -- budget duration >= approach + hold_s. fire_channels (CSV e.g. "9,10") fires those board channels ONCE, fire_t seconds into the hold, on a background thread so the loop keeps correcting while the shot leaves. fire_gap spaces multi-channel shots apart in seconds. fire_pass_enabled fires at command end even if never fully aligned, as long as the target was seen recently. hold_through_loss=true coasts on target loss. lock_target=true steers to the box nearest the last-accepted centre, not the largest. ctrl_conf is a control-side min score. range_gain_floor softens gain when the bbox fills the frame. ki_lat adds a lateral integral during the hold. fwd_fill>0 (forward camera) adds a forward range-hold axis (mode=area/width/height, kp_forward gain) to that fill %% and gates the shot on the standoff. settle_px>0 declares aligned only once the hull is in-band AND barely moving. VERIFY (verify_off=true skips it) re-measures on align_stable_frames new frames after braking: DRIFTED if out of band, LOST if not re-seen. surge_sign (+1/-1) flips the downward surge for the mount. The board holds depth and heading itself; a forward depth axis and a downward fwd_fill are REFUSED. |
| `vision_move` | `camera`, `target_class`, `fwd_fill`, `mode`, `maintain_px`, `maintain_on`, `hold_s`, `err_px`, `duration`, `gain`, `gain_lat`, `brake_off`, `brake_gain`, `hold_through_loss`, `kp_forward`, `kp_lat`, `lost_grace_s`, `range_gain_floor`, `coast_s`, `lock_s` | `camera=forward`, `target_class=`, `fwd_fill=95.0`, `mode=area`, `maintain_px=0.0`, `maintain_on=False`, `hold_s=0.0`, `err_px=40.0`, `duration=20.0`, `gain=30.0`, `gain_lat=0.0`, `brake_off=False`, `brake_gain=0.0`, `hold_through_loss=False`, `kp_forward=200.0`, `kp_lat=60.0`, `lost_grace_s=1.0`, `range_gain_floor=1.0`, `coast_s=0.8`, `lock_s=1.0` | host | Drive forward toward target_class. fwd_fill > 0 stops once the bbox fills that %% of the frame (mode = area/width/height; height for tall slalom). fwd_fill <= 0 (e.g. --fwd_fill -1, or DSL move(fwd=None)) is PASS-THROUGH: drive until the target is seen and then leaves the frame, plus a commit overshoot (hold_s, else ~2s) to carry the hull through a gate. maintain_on holds a maintain_px lateral offset while moving; depth and heading are held by the board. gain caps forward speed; gain_lat overrides the cap on the maintain strafe (0 = inherit gain). On a fill-stop arrival the forward (and maintain) inertia is braked so the hull halts in front of the target instead of creeping in; PASS-THROUGH never brakes (it must coast through the gate). brake_off=true coasts, brake_gain scales the kick. Does NOT re-centre. |
| `yaw_left` | `target`, `timeout`, `settle` | `timeout=30.0`, `settle=0.0` | on the board | Sharp pivot left by `target` degrees within `timeout` s. |
| `yaw_right` | `target`, `timeout`, `settle` | `timeout=30.0`, `settle=0.0` | on the board | Sharp pivot right by `target` degrees within `timeout` s. |

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
| `ros2 run mongla_vision switch_camera` | `mongla_vision.utils.switch_camera:main` |
| `ros2 run mongla_vision calibrate` | `mongla_vision.calibration.guide:main` |
| `ros2 run mongla_vision calibrate_solve` | `mongla_vision.calibration.solver:main` |
| `ros2 run mongla_vision depth_estimation_node` | `mongla_vision.depth.depth_estimation_node:main` |
| `ros2 run mongla_vision flow_node` | `mongla_vision.flow.flow_node:main` |
| `ros2 run mongla_vision mission_web` | `mongla_vision.web.mission_web_node:main` |

## 3. Launch arguments — `DeclareLaunchArgument`

WARNING: `ros2 launch` **silently ignores** a `name:=value` the file does not declare. Check the name against the table for THAT file, or put it in `~/.mongla/bringup.yaml`, which refuses unknown names (example: `src/mongla_manager/config/bringup.example.yaml`).

### `mongla_manager/bringup.launch.py`

| argument | default | what it does |
|---|---|---|
| `mav_device` | `` | SROT serial device, '' = autodetect. A path (/dev/serial/by-id/...) or any pymavlink connection string |
| `payload_channels` | `` | OPTIONAL labels for the log, "<board_channel>:<name>", e.g. "9:torpedo_1, 11:dropper_1". Labels ONLY -- fire(N) always addresses board channel N; nothing here routes a shot |
| `allow_fw_behaviour_mismatch` | `false` | Arm against firmware older than FW_BEHAVIOUR_REV_REQUIRED. Accepts an un-braked stop -- leave false |
| `vision` | `false` | Start camera + detector alongside manager |
| `vision_stack` | `pi` | pi = both cameras + calibration (vision_pi.launch.py, the vehicle); generic = single camera, no calibration (vision.launch.py, dev box). |
| `vision_profile` | `fast` | vision_stack:=pi -- vision_pi's own profile argument (its `vision:=`). Renamed here because `vision` is this file's boolean on/off switch. |
| `fwd_model` | `gate_sharks` | vision_stack:=pi -- forward-camera model stem. |
| `dwn_model` | `bin_fire_blood` | vision_stack:=pi -- downward-camera model stem. |
| `fwd_classes` | `` | vision_stack:=pi -- forward class allowlist (empty = the model sidecar's full set). |
| `dwn_classes` | `` | vision_stack:=pi -- downward class allowlist. |
| `localization` | `true` | Run the invariant filter (mongla_localization). Read-only: it publishes /mongla/odom and commands nothing, so it is on by default. Set false to take it off the graph. |
| `flow` | `false` | vision_stack:=pi -- start flow_node (downward-camera velocity). Pairs with the manager's position_source:=flow. |
| `pool_depth_min_m` | `nan` | flow:=true -- the venue's shallowest floor, metres (#35). |
| `pool_depth_max_m` | `nan` | flow:=true -- the venue's deepest floor, metres (#35). |
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
| `camera_profile` | `pi_forward` | Camera HARDWARE profile (CAMERA_PROFILES in mongla_vision/config.py -- the loaded copy). Default pi_forward is the vehicle's Fantech. |
| `model` | `gate_sharks` | vision_stack:=generic -- model stem in mongla_vision/models/ (a <stem>.yaml sidecar must sit beside it). |
| `models` | `` | vision_stack:=generic -- multi-model registry, CSV name=stem: "gate=gate_sharks,bins=bin_fire_blood" |
| `active_model` | `` | Registry key to start with (requires models:="..." to be set) |
| `classes` | `gate` | CSV class names for detector |
| `conf` | `0.45` | Detector confidence floor. 0.45 is gate_sharks' measured bar; another model's bar is in its <stem>.yaml. |
| `imgsz` | `640` | Inference square size for a .pt. A .hef bakes its input size at compile time and ignores this. |
| `max_det` | `100` | Post-NMS detection cap (runtime; lower toward ~10 if NMS is the FPS bottleneck on busy frames). |
| `viewer` | `false` | Open vision_display (OpenCV viewer) alongside vision pipeline |
| `foxglove` | `false` | Start foxglove_bridge (WebSocket telemetry on foxglove_port). Off the mission path -- pure viz. Needs ros-humble-foxglove-bridge installed. Connect the Foxglove desktop app to ws://<pi>:<foxglove_port>. |
| `foxglove_port` | `8765` | foxglove_bridge WebSocket port |
| `fwd_profile` | `pi_forward` | (passed to vision_pi.launch.py, which sets the default) |
| `dwn_profile` | `pi_downward` | (passed to vision_pi.launch.py, which sets the default) |
| `fwd_models` | `` | (passed to vision_pi.launch.py, which sets the default) |
| `dwn_models` | `` | (passed to vision_pi.launch.py, which sets the default) |
| `fwd_active` | `` | (passed to vision_pi.launch.py, which sets the default) |
| `dwn_active` | `` | (passed to vision_pi.launch.py, which sets the default) |
| `contours` | `true` | (passed to vision_pi.launch.py, which sets the default) |
| `assoc_conf` | `0.10` | (passed to vision_pi.launch.py, which sets the default) |
| `preprocess` | `auto` | (passed to vision_pi.launch.py, which sets the default) |
| `range_crop` | `-1` | (passed to vision_pi.launch.py, which sets the default) |
| `preprocess_clip` | `0.0` | (passed to vision_pi.launch.py, which sets the default) |
| `tracking` | `true` | (passed to vision_pi.launch.py, which sets the default) |
| `tracker_type` | `ocsort` | (passed to vision_pi.launch.py, which sets the default) |
| `kalman_adaptive_noise` | `true` | (passed to vision_pi.launch.py, which sets the default) |
| `fwd_frame_rate` | `30.0` | (passed to vision_pi.launch.py, which sets the default) |
| `dwn_frame_rate` | `32.0` | (passed to vision_pi.launch.py, which sets the default) |
| `dwn_calibration` | `` | (passed to vision_pi.launch.py, which sets the default) |
| `fwd_calibration` | `` | (passed to vision_pi.launch.py, which sets the default) |
| `act_conf` | `0.45` | (passed to vision_pi.launch.py, which sets the default) ladder ACTING bar: below this a detection may be associated by the tracker but is not acted on (B-59; measured knee 0.60, shipped 0.45) |
| `anchor_xfeat_hef` | `true` | (passed to vision_pi.launch.py, which sets the default) run XFeat on the Hailo-8 rather than the Pi CPU (B-62); safe because the lock is composed with the detector |
| `anchor_semi_dense` | `false` | (passed to vision_pi.launch.py, which sets the default) semi-dense matching: 2.1-2.6x inliers for +37 % match cost. OFF until frame-to-reference is measured |
| `target_width_m_forward` | `0.0` | (passed to vision_pi.launch.py, which sets the default) measured TRUE width of the forward target in metres; 0 = use the rulebook nominal. A measured prop beats a nominal, which is what SAUVC +/-5 % allows for |
| `target_width_m_downward` | `0.0` | (passed to vision_pi.launch.py, which sets the default) measured true width of the downward target, metres |
| `dwn_lock_class` | `` | (passed to vision_pi.launch.py, which sets the default) Class the DOWNWARD ladder follows. Empty = any class, and no 6-DoF pose (the geometry table is keyed by class). |
| `bank_forward` | `` | (passed to vision_pi.launch.py, which sets the default) checkpoint bank (.npz) preloaded into the FORWARD lock node, from tools/pool/build_practice_bank.py. Empty learns live. A preloaded reference is trusted exactly like a live one -- it clears MIN_INLIERS or it does not answer. |
| `bank_downward` | `` | (passed to vision_pi.launch.py, which sets the default) checkpoint bank (.npz) preloaded into the DOWNWARD lock node -- places rather than props. |
| `fwd_fps` | `0` | (passed to vision_pi.launch.py, which sets the default) |
| `dwn_fps` | `0` | (passed to vision_pi.launch.py, which sets the default) |
| `fwd_publish_hz` | `40` | (passed to vision_pi.launch.py, which sets the default) |
| `dwn_publish_hz` | `0` | (passed to vision_pi.launch.py, which sets the default) |
| `replay` | `false` | (passed to vision_pi.launch.py, which sets the default) Consume image_raw from a RECORDED BAG instead of a camera. Build no camera, keep every detector, and let `ros2 bag play` drive the graph. This is how a recorded session becomes a regression fixture: the detector, tracker, lock ladder and checkpoint bank all run for real, on input that never changes. |
| `fwd_device_path` | `` | (passed to vision_pi.launch.py, which sets the default) |
| `dwn_device_path` | `` | (passed to vision_pi.launch.py, which sets the default) |

### `mongla_vision/mission_web.launch.py`

| argument | default | what it does |
|---|---|---|
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
| `fwd_model` | `gate_sharks` |  |
| `fwd_models` | `` |  |
| `fwd_classes` | `gate,shark,shaw_fish` |  |
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

### `mongla_vision/video.launch.py`

| argument | default | what it does |
|---|---|---|
| `fwd_video` | `` | Forward-camera video (e.g. a gate dataset clip). At least one of fwd_video / dwn_video is required. |
| `dwn_video` | `` | Downward-camera video (e.g. a bin dataset clip). |
| `fwd_model` | `gate_sharks` | YOLO model stem for the forward detector. |
| `fwd_classes` | `gate,shark,shaw_fish` | Class filter for the forward detector. |
| `dwn_model` | `bin_fire_blood` | YOLO model stem for the downward detector. |
| `dwn_classes` | `fire,blood` | Class filter for the downward detector. |
| `conf` | `0.35` | Detection confidence threshold (both detectors). |
| `loop` | `true` | Loop both videos at EOF. true = detections keep flowing for verb/gain tuning; false = single pass for a clean mission-sequence run (a loop re-shows the target at EOF, which can re-trigger detected() acquisition mid-run). |
| `viewer` | `true` | Open the HUD (viewer:=false = headless autonomous run). |
| `tracking` | `true` |  |
| `imgsz` | `640` |  |
| `max_det` | `100` |  |

### `mongla_vision/vision.launch.py`

| argument | default | what it does |
|---|---|---|
| `profile` | `` | Camera HARDWARE profile (CAMERA_PROFILES) (pi_forward|pi_downward|forward|...). Empty = same as `camera`. Set this to keep the role name while changing which unit it opens. |
| `lock` | `false` | Run the lock ladder (follower + XFeat anchor), publishing <ns>/lock. Costs ~23 % of the detection rate, measured. Control ignores it until vision.lock_s > 0 -- turn both on together. |
| `lock_class` | `` |  |
| `camera` | `forward` | Camera profile (forward|downward|laptop|auto). Drives node names: mongla_detector_<camera>. |
| `device` | `-1` | /dev/videoN index; -1 = use profile default |
| `device_path` | `` | override device (empty = auto /dev/mongla_cam_<camera> symlink if present, else int `device`) |
| `width` | `640` |  |
| `height` | `480` |  |
| `fps` | `0` | 0 = use the camera profile fps |
| `video_file` | `` | Path to a video file; when set, replaces the live webcam |
| `topic` | `` | ROS image topic to consume (bag replay / re-published stream). Ignored when video_file is set. |
| `loop` | `true` | Loop the video file at EOF (video_file only) |
| `model` | `yolov11n` | Model stem (models/) or .pt path (single-model mode). Pool: gate_sharks / gate_flare_medium_100ep. |
| `models` | `` | CSV name=stem registry for hot model switching: "gate=gate_nano_100ep,combined=gate_flare_medium_100ep" |
| `active_model` | `` | Registry key to start with (requires models:="...") |
| `classes` | `` | CSV class filter; empty = all model classes |
| `conf` | `0.35` |  |
| `iou` | `0.5` |  |
| `device_cls` | `auto` | Inference device for a .pt (auto | cuda:0 | cpu); a .hef ignores it |
| `imgsz` | `640` | Inference square size for a .pt. A .hef bakes its input size at compile time and ignores this. |
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
| `flow` | `false` | Start flow_node on the downward camera (velocity, and calc_distance start/stop). |
| `pool_depth_m` | `nan` | Water depth in metres. REQUIRED with flow:=true -- the node refuses to publish without it. |
| `hud_distance` | `true` | HUD pre-arms the distance panel. |

### `mongla_vision/vision_dual.launch.py`

| argument | default | what it does |
|---|---|---|
| `fwd_device` | `0` | /dev/videoN index for the forward camera — LAST-RESORT fallback, only used on a box with no /dev/mongla_cam_* symlink (dev box). On the vehicle the port-stable symlink wins automatically (see device_path below). |
| `dwn_device` | `4` | /dev/videoN index for the downward camera (dev-box fallback; the vehicle symlink wins automatically) |
| `fwd_device_path` | `` | override device for forward (empty = auto /dev/mongla_cam_forward) |
| `dwn_device_path` | `` | override device for downward (empty = auto /dev/mongla_cam_downward) |
| `fwd_model` | `gate_sharks` | Single YOLO model stem (used only when fwd_models is empty) |
| `fwd_models` | `` | CSV of model stems for runtime switching (e.g. gate_sharks,slalom_red_pipe,torpedo_blood_hole). Empty = single fwd_model. |
| `fwd_classes` | `gate,shark,shaw_fish` | Class filter for the forward detector |
| `dwn_model` | `bin_fire_blood` | Single YOLO model stem (used only when dwn_models is empty) |
| `dwn_models` | `` | CSV of model stems for runtime switching on the downward detector. Empty = single dwn_model. |
| `dwn_classes` | `fire,blood` | Class filter for the downward detector |
| `fwd_conf` | `0.35` | Uniform conf for the forward detector (every model unless overridden below) |
| `dwn_conf` | `0.35` | Uniform conf for the downward detector |
| `fwd_model_conf` | `` | Per-model conf overrides on forward, CSV name=conf (e.g. torpedo_blood_hole=0.55) |
| `dwn_model_conf` | `` | Per-model conf overrides on downward, CSV name=conf |
| `device_cls` | `auto` | Inference device for a .pt (auto | cuda:0 | cpu); a .hef ignores it |
| `imgsz` | `640` | Inference square size for a .pt (both detectors). A .hef bakes its input size and ignores this. |
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
| `flow` | `false` | Start flow_node on the downward camera (velocity, and calc_distance start/stop). |
| `pool_depth_m` | `nan` | Water depth in metres. REQUIRED with flow:=true -- the node refuses to publish without it. |
| `hud_distance` | `true` | HUD pre-arms the distance panel (shows on calc_distance). |

### `mongla_vision/vision_pi.launch.py`

| argument | default | what it does |
|---|---|---|
| `fwd_profile` | `pi_forward` |  |
| `dwn_profile` | `pi_downward` |  |
| `fwd_model` | `gate_sharks` |  |
| `dwn_model` | `bin_fire_blood` |  |
| `fwd_models` | `` |  |
| `dwn_models` | `` |  |
| `fwd_active` | `` |  |
| `dwn_active` | `` |  |
| `contours` | `true` |  |
| `fwd_classes` | `` |  |
| `dwn_classes` | `` |  |
| `conf` | `0.45` | The acting bar: gate_sharks' measured bar. Another model's bar is in its <stem>.yaml; see the module docstring. |
| `assoc_conf` | `0.10` |  |
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
| `act_conf` | `0.45` | ladder ACTING bar: below this a detection may be associated by the tracker but is not acted on (B-59; measured knee 0.60, shipped 0.45) |
| `anchor_xfeat_hef` | `true` | run XFeat on the Hailo-8 rather than the Pi CPU (B-62); safe because the lock is composed with the detector |
| `anchor_semi_dense` | `false` | semi-dense matching: 2.1-2.6x inliers for +37 % match cost. OFF until frame-to-reference is measured |
| `target_width_m_forward` | `0.0` | measured TRUE width of the forward target in metres; 0 = use the rulebook nominal. A measured prop beats a nominal, which is what SAUVC +/-5 % allows for |
| `target_width_m_downward` | `0.0` | measured true width of the downward target, metres |
| `lock_class` | `` | Class the ladder locks onto. Empty = whatever the detector is publishing. |
| `dwn_lock_class` | `` | Class the DOWNWARD ladder follows. Empty = any class, and no 6-DoF pose (the geometry table is keyed by class). |
| `pool_depth_m` | `nan` | Water depth in metres. REQUIRED with flow:=true -- the node refuses to publish velocity without it. |
| `pool_depth_min_m` | `nan` | Shallowest floor in the venue, metres (#35). With pool_depth_max_m, flow's sigma carries the scale error a sloping floor makes. nan = declare it flat. |
| `pool_depth_max_m` | `nan` | Deepest floor in the venue, metres (#35). |
| `tile_m` | `0.0` | Floor tile pitch in metres, measured on deck. 0 = OFF. Sets height from the floor and publishes the grid angle that bounds yaw drift. A wrong value rescales every height silently -- measure it. |
| `caustics` | `true` | flow: erode sun caustics and refuse a bare floor under them (false = always track the raw floor; A/B switch) |
| `bank_forward` | `` | checkpoint bank (.npz) preloaded into the FORWARD lock node, from tools/pool/build_practice_bank.py. Empty learns live. A preloaded reference is trusted exactly like a live one -- it clears MIN_INLIERS or it does not answer. |
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

## 3b. Missions — `mongla_planner/missions/`

Run with `ros2 run mongla_planner mission <name>`. A model marked *not shipped* is not in `src/mongla_vision/models/`: it arrives on a water day, and until then the mission refuses at `set_model`. A board move enters AUTO, which closes the depth loop: every one is gated on the two armed bench checks (CLAUDE.md §2).

| mission | what it does | models it loads | board moves (depth-gated) |
|---|---|---|---|
| `demo_dual_camera` | gate → bin → torpedo across TWO cameras, ONE detector at a time. | `gate_sharks`, `torpedo_blood_hole` (not shipped) | `pause`, `set_depth` |
| `demo_find_person` | canonical vision-driven mission for desk/thruster testing. | — | `move_forward`, `set_depth`, `stop`, `yaw_left`, `yaw_right` |
| `demo_move_see` | minimal open-loop + vision demo (two-verb). | — | `move_back`, `move_forward`, `set_depth`, `yaw_left` |
| `demo_pursue` | torpedo-style approach using align() then move(). | — | `move_back`, `move_forward`, `set_depth`, `stop`, `yaw_right` |
| `demo_square` | the classic shakedown mission. | — | `move_forward`, `pause`, `set_depth`, `yaw_right` |
| `out_and_back` | dive, swim straight, turn clockwise, swim back. Control only. | — | `move_forward`, `set_depth`, `yaw_right` |
| `pool_day_practice` | Pool day practice run: Gate → Slalom → Torpedo → Bin. | `bin_fire_blood`, `gate_sharks`, `slalom_red_pipe` (not shipped), `torpedo_blood_hole` (not shipped) | `move_forward`, `pause`, `set_depth`, `stop`, `turn` |
| `pool_day_torpedo` | Pool day torpedo practice: align → approach → fine-lock → fire. | `torpedo_blood_hole` (not shipped) | `move_forward`, `set_depth`, `stop`, `turn` |
| `sauvc_full` | SAUVC 2026 full run: Navigation, Target Acquisition, then (opt-in) the flares. | `sauvc_sim` | `move_back`, `move_forward`, `set_depth` |
| `sauvc_navigation` | SAUVC 2026 Navigation — swim through the red/green gate. THE mandatory task. | `sauvc_sim` | `move_forward`, `set_depth` |
| `sauvc_qualification` | SAUVC Qualification — through the hanging gate, U-turn, back through it. | `sauvc_sim` | `move_forward`, `set_depth`, `yaw_right` |
| `sauvc_target_acquisition` | SAUVC 2026 Target Acquisition — drop the ball into the BLUE drum. 30 points. | `sauvc_sim` | `move_forward`, `pause`, `set_depth` |
| `survey_course` | put the props where the vehicle actually sees them. | — | `set_depth` |
| `task_bin` | Bin drop task — DOWNWARD camera, hover over the bin, drop a marker. | `bin_fire_blood` | `move_forward`, `pause`, `set_depth`, `turn` |
| `task_full_2026` | Full RoboSub 2026 competition run: Gate → Slalom → Bin → Torpedo → Return+Roll. | — | `set_depth` |
| `task_gate` | Gate task — find the gate and drive through it. | `gate_sharks` | `move_forward`, `pause`, `set_depth` |
| `task_return` | Return task — find gate again, pass back through, style roll. | `gate_sharks` | `move_forward`, `pause`, `set_depth`, `style_roll`, `turn` |
| `task_slalom` | Slalom task — weave through red pipes. | `slalom_red_pipe` (not shipped) | `move_forward`, `pause`, `turn` |
| `task_torpedo` | Torpedo task — align on board, approach via blood, standoff-lock hole, fire. | `torpedo_blood_hole` (not shipped) | `move_forward`, `pause`, `set_depth`, `turn` |
| `test` | a scratch bench mission, retargeted 2026-09-28. | `gate_sharks` | `move_forward`, `move_left`, `pause`, `set_depth`, `stop` |

## 4. Node parameters — `declare_parameter`

Set any of these on the vehicle under `nodes:` in `~/.mongla/bringup.yaml`,
keyed by the RUNNING node name -- `ros2 node list` prints them, e.g.
`mongla_manager`, `mongla_tracker_forward`, `mongla_lock_downward`. A misspelt
key, or one a launch argument already sets, stops that node at launch.

Listed by source file, literal `declare_parameter` calls only. The
manager's `vision.*` gains are declared from a table and are listed in
`bringup.example.yaml` instead, with their defaults.

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

**`mongla_manager/auv_manager_node.py`** — 16 parameters

| parameter | default |
|---|---|
| `mav_device` | `''` |
| `record` | `''` |
| `vision_uplink_camera` | `''` |
| `vision_uplink_class` | `''` |
| `vision_uplink_hz` | `25.0` |
| `vision_uplink_medium` | `'water'` |
| `velocity_uplink` | `False` |
| `position_uplink` | `False` |
| `payload_channels` | `''` |
| `payload_fire_map` | `''` |
| `debug` | `False` |
| `allow_fw_behaviour_mismatch` | `False` |
| `srot_telemetry_period_s` | `2.0` |
| `allow_saturated_depth_arm` | `False` |
| `allow_unknown_kill_switch` | `False` |
| `shield_max_depth_m` | `0.0` |

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

**`mongla_vision/depth/depth_estimation_node.py`** — 6 parameters

| parameter | default |
|---|---|
| `camera` | `'forward'` |
| `model_path` | `''` |
| `run_every_n_frames` | `3` |
| `publish_depth_map` | `False` |
| `invert_depth` | `True` |
| `use_tracks` | `False` |

**`mongla_vision/detector_dual_node.py`** — 3 parameters

| parameter | default |
|---|---|
| `device` | `-1` |
| `replay` | `False` |
| `lock` | `False` |

**`mongla_vision/detector_node.py`** — 27 parameters

| parameter | default |
|---|---|
| `camera` | `'laptop'` |
| `image_topic` | `''` |
| `model_path` | `'yolov11n'` |
| `models` | `''` |
| `active_model` | `''` |
| `device` | `'auto'` |
| `half` | `True` |
| `conf` | `0.35` |
| `assoc_conf` | `0.0` |
| `model_conf` | `''` |
| `iou` | `0.5` |
| `nms_floor` | `0.05` |
| `nms_iou` | `0.0` |
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

**`mongla_vision/flow/flow_node.py`** — 40 parameters

| parameter | default |
|---|---|
| `camera` | `'downward'` |
| `pool_depth_m` | `float('nan'` |
| `pool_depth_min_m` | `float('nan'` |
| `pool_depth_max_m` | `float('nan'` |
| `medium` | `'water'` |
| `focal_air_px` | `_F_AIR_PX` |
| `focal_water_px` | `_F_WATER_PX` |
| `gyro_gain_x` | `_GYRO_GAIN_X_DEFAULT` |
| `gyro_gain_y` | `_GYRO_GAIN_Y_DEFAULT` |
| `rot_fraction_max` | `0.80` |
| `refractive_rectify` | `True` |
| `min_net_flow_px` | `0.5` |
| `max_dispersion_ratio` | `5.0` |
| `grid_buckets` | `4` |
| `want_points` | `80` |
| `target_px` | `8.0` |
| `max_px` | `25.0` |
| `max_baseline_s` | `0.75` |
| `max_rotation_deg` | `6.0` |
| `use_planar_fit` | `True` |
| `ransac_px` | `2.0` |
| `fb_reject_px` | `2.0` |
| `yaw_image_sign` | `-1.0` |
| `yaw_cross_check_tol` | `0.25` |
| `calibration` | `''` |
| `profile` | `''` |
| `allow_uncalibrated` | `False` |
| `undistort` | `True` |
| `exposure_us` | `0.0` |
| `stamp_source` | `'soe'` |
| `stamp_at_midpoint` | `True` |
| `estimate_time_offset` | `True` |
| `time_offset_s` | `FLOW_TD_MEASURED_S` |
| `time_offset_max_s` | `0.15` |
| `time_offset_min_quality` | `0.5` |
| `tile_m` | `0.0` |
| `tile_period_s` | `0.5` |
| `tile_decimate` | `3` |
| `lane_lines` | `False` |
| `caustic_suppression` | `True` |

**`mongla_vision/lock_node.py`** — 20 parameters

| parameter | default |
|---|---|
| `camera` | `'forward'` |
| `target_class` | `''` |
| `follow` | `True` |
| `anchor` | `False` |
| `anchor_model` | `''` |
| `anchor_xfeat_hef` | `False` |
| `anchor_semi_dense` | `False` |
| `anchor_bank` | `''` |
| `loop_closure` | `_lc.defaults(` |
| `place_period_s` | `_lc.defaults(` |
| `place_travel_m` | `_lc.defaults(` |
| `pool_depth_m` | `_lc.defaults(` |
| `act_conf` | `0.45` |
| `publish_hz` | `0.0` |
| `anchor_hz` | `4.5` |
| `full_authority_s` | `FULL_AUTHORITY_S` |
| `zero_authority_s` | `ZERO_AUTHORITY_S` |
| `target_width_m` | `0.0` |
| `medium` | `'water'` |
| `direct_feed` | `False` |

**`mongla_vision/tracker_node.py`** — 16 parameters

| parameter | default |
|---|---|
| `camera` | `'laptop'` |
| `tracker_type` | `'ocsort'` |
| `frame_rate` | `20.0` |
| `track_buffer` | `150` |
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

**`mongla_vision/utils/display_node.py`** — 10 parameters

| parameter | default |
|---|---|
| `camera` | `'forward'` |
| `cameras` | `'forward` |
| `video_file_mode` | `False` |
| `launch_pipeline` | `False` |
| `model` | `'yolov11n'` |
| `classes` | `'person'` |
| `conf` | `0.35` |
| `max_display_hz` | `30.0` |
| `yaw_source` | `'mavlink_ahrs'` |
| `hud_distance` | `False` |

**`mongla_vision/vision_node.py`** — 14 parameters

| parameter | default |
|---|---|
| `profile` | `'laptop'` |
| `source` | `''` |
| `topic` | `''` |
| `device` | `'auto'` |
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

**`mongla_vision/web/mission_web_node.py`** — 3 parameters

| parameter | default |
|---|---|
| `cameras` | `'` |
| `web_port` | `8090` |
| `video_port` | `8080` |

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

ERROR: `/mongla/arm`, `/mongla/depth_cmd`, `Attitude.msg` and `RCOverride.msg` appear in older notes. **None of them exist.**

