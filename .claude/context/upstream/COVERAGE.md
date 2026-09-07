# Read coverage of the SROT / Hengla / Bondor repos

What we have actually read, file by file, so "we know their stack" is an
auditable claim rather than an impression. Written 2026-09-03 against
`srot-control-board` `f1d3ba9`, `srot-ground-station` `1adc14c`,
`srot-esc-flasher` `c30b843`.

**Scope.** Every tracked file EXCEPT the vendored MAVLink headers
(`lib/mavlink/**` in two of the repos — ~450 generated files). Those are
upstream-generated code we consult by lookup, not something to read; we do read
the specific message definitions we depend on (`mavlink_msg_sys_status.h`,
`mavlink_msg_esc_status.h`, `common.h`'s `MAV_RESULT` and
`MAV_SYS_STATUS_SENSOR_*` enums), and `test_srot_protocol_drift.py` parses them
mechanically, which is stronger than reading them.

| repo | their own files | lines |
|---|---|---|
| `srot-control-board` | 116 | 22,771 |
| `srot-ground-station` | 43 | 6,909 |
| `srot-esc-flasher` | 20 | 3,415 |
| **total** | **179** | **33,095** |

## `srot-control-board` — complete

**Control path**, read line by line by me: `task_control_loop.cpp`,
`mixer.cpp`, `movement.cpp`, `attitude_control.cpp`, `depth_control.cpp`,
`arming.cpp`, `safety_monitor.cpp`, `yaw_ref.cpp`, `thrust_trim.cpp`,
`mav_commands.cpp`, `mav_stream.cpp`, `params.cpp` (including all 189 rows of
the table and the five one-shot NVS migrations), `config.h` (all 846 lines
including the rev 1-14 ladder), `state_types.h`, `pico/main.cpp` (the ESC
telemetry path).

**Drivers, tasks, tuning**: `autotune.cpp`, `calibration.cpp`, `motor_tune.cpp`,
`pattern.cpp`, `stunt.cpp`, `feedforward.cpp`, `pid.h`, `bno085.cpp`,
`bar30.cpp`, `analog_mon.cpp`, `oled.cpp`, `lora_mission.cpp`,
`espnow_link.cpp`, `sd_log.cpp`, `pca9685_aux.cpp`, `dshot_out.cpp`,
`thruster_link.cpp`, `buzzer.cpp`, `neopixel_rgb.cpp`, all seven `task_*.cpp`,
`mission.cpp`, `mavlink_bridge.cpp`, `ui_log.cpp`, `second_board/main.cpp`,
`main.cpp`, all three `shared/*.h`, and every matching header.

**Docs**: `AUDIT.md`, `ROADMAP.md`, `JETSON_COMMS.md`, `VISION_API.md`,
`TASKS_FROM_DUBURI_WS.md`, `JETSON_FEEDBACK.md`, `FIRMWARE_CHANGELOG_FOR_DUBURI.md`,
`AGENTS.md`, `DOC_DRIFT_AND_DESIGN_REVIEW`, `WATER_TEST_CONTROL_ONLY.md`, all six
`docs/*.md`, plus `ALGORITHMS.md`, `ARCHITECTURE.md`, `HARDWARE.md`,
`PARAMETERS.md`, `DUBURI_WS_INTEGRATION.md`, both HANDOFF docs, `BENCH_FINDINGS`,
`README.md`, `platformio.ini`, `partitions_hengla.csv`, and the vendored
`lib/MS5837` driver.

## `srot-ground-station` (Bondor) — complete

All 43 files: the Electron main process and its four MAVLink transports, the
preload bridge, all 11 views, all 7 components, the telemetry store, the
parameter metadata (the board's full parameter surface as an operator sees it),
the gamepad poller, the `.params` file format, the theme, the build config, and
**`src/groundstation/main.cpp`** — the ESP32-C3 LoRa bridge firmware, which is a
third piece of embedded code in this system.

## `srot-esc-flasher` — complete

All 20 files including the five protocol-layer sources (`4Way.cpp`, `MSP.cpp`,
`BitBangSerial.cpp`, `ESC_Serial.cpp`, `serial_comm.cpp`) that an earlier pass
had covered only by grep.

## Why this file exists

Round 27 found that they had shipped the `SYS_STATUS` leak bit **at our
request** in rev 3, and we did not adopt it for four rounds because we had
written down that we could not read it. The read was the fix, and the same
question — *what is already there that we are not using?* — has since produced
signed thruster RPM, per-thruster presence, the boot-burst warnings and the
in-session gain path, none of which needed a firmware change.

Keeping the coverage explicit is what stops that from being rediscovered. When
their repos move, `git fetch` on `Mongla_others` is step 0 of every srot round,
and this table says what a diff has to be read against.


---

## The vehicle can now check its own work

Three things were true of the Pi and are no longer:

**It had no `.git`.** It was file-copied, not cloned, so it could not `git pull`
and was kept current by hand — which is how a stale `test_srot_fc.py` sat there
reporting 578 collected against the dev box's 579. It is a real clone now, and
`git status` is the check that it matches the branch.

**It had no internet.** `enp59s0u1c2` (10.42.0.1) is a NetworkManager shared
link, which gives the Pi an address and DHCP but **no masquerade rule for
10.42.0.0/24** — Docker's rules were present and this subnet's was not. The
confusing part is the split it produces: **DNS resolved fine** (the dev box's
`systemd-resolved` answered) while every packet to an external address was
dropped, so it looked like a routing problem at the far end rather than a
missing NAT rule at the near one. `tools/pi_share_internet.sh` adds the
masquerade plus the two FORWARD rules and is idempotent.

**The drift suite skipped all 25 of its tests there.** It walks up looking for
`Mongla_others/srot-control-board`, which did not exist on the Pi, so on the one
machine that flies the vehicle the cross-repo referee was silent. All three of
their repos are cloned beside the workspace now.

That last one matters more this round than last, because this round added
several constants hand-mirrored from their headers — `MOTOR_TEST_*`,
`SYS_STATUS_SENSOR_LEAK`, `_ESC_STATUS_CRC_EXTRA`, the `UPLINK_*` block. Those
are exactly what the drift test referees, and they were being refereed only on
a laptop.

**Vehicle parity, measured:** 579 collected / 579 passed / 0 skipped, matching
the dev box exactly. It was 553 passed + 25 skipped when this round started.

⚠ **`colcon build` on the Pi COPIES rather than symlinks** — `--symlink-install`
notwithstanding, `build/<pkg>/<pkg>/*.py` are regular files. So editing `src/`
does not change what runs until a rebuild, and a test run can pass against
source while `ros2 run` serves the previous version. Rebuild after any sync.
