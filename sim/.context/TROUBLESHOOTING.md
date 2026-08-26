# Troubleshooting

> ℹ **Absorbed 2026-08-27.** This workspace is no longer the sibling tree
> `Ros_workspaces/duburi-sim_ws`; it lives inside the `duburi_ws` repo at
> `duburi_ws/sim/` and is under version control. Paths below have been
> updated; any remaining "sibling" phrasing is historical.

## Lost manual control / arm NO_ACK

**Cause:** Two managers (or anything else) competing on UDP **14550**, or ArduSub
failsafe on pilot input.

**Fix:**

```zsh
ros2 run duburi_sim_bringup duburi_sim stop
ros2 run duburi_sim_bringup duburi_sim sim
ros2 run duburi_sim_bringup duburi_sim stack --no-vision
# ensure only one /duburi_manager
ros2 node list | grep duburi_manager
```

Sim params set `FS_PILOT_INPUT` appropriately in `duburi_sub.parm`.

## No AUV in Gazebo / empty world

**Cause:** Second headless sim already owns world / FDM port **9002**.

**Fix:** `duburi_sim stop`, then one `sim` with GUI. Look at **x ≈ −11.8**.

## Black / missing lab cameras

1. Confirm `contract_check` and `/api/cameras/front/jpeg` returns image/jpeg.  
2. Status `link.cams` / `cameras.front`.  
3. Rebuild frontend static if UI is stale.  
4. MJPEG URL: `/api/cameras/front/mjpeg` — some clients show first frame slowly.

## Lab connection refused / wrong port

Lab auto-binds if 28765 busy (Electron often steals 8088 historically).

```zsh
cat /tmp/duburi_lab_port.txt   # if your launcher wrote it
echo $DUBURI_LAB_PORT
ss -tlnp | grep 2876
```

Open `http://localhost:<actual_port>/`.

## Teleop connected false / no motion

- Arm first (`● arm` or planner `arm`).
- Hold D-pad (not tap); teleop only streams while axes non-zero.
- Endpoint must be `tcp:127.0.0.1:5763` (ArduSub serial1-style TCP).
- Do not point teleop at 14550.

## Record stop fails / empty zip / no meta.json

- Wait until recording is actually live (lab waits for `.ready`).
- Very short clips can finalize with `counts: 0` (warning in stdout).
- Orphan `record_cameras`: `pkill -f record_cameras` then retry.
- Dataset API hides dirs without `meta.json`.

## Props spawn errors

- Ensure `prop_manager` running with matching `world:={course}`.
- Lab World restart restarts prop_manager after course change.
- `props list` shows **catalog**, not spawned names — use your instance `name` for remove.

## X11 / Gazebo GUI fails

- Set `DISPLAY` and readable `XAUTHORITY` (helper tries mutter cookie).
- Or use `duburi_sim sim --headless` (physics still runs).

## gz-transport flaky / commands dropped

```zsh
export GZ_IP=127.0.0.1
```

Required on hosts with many NICs; launch also sets this.

## Depth bouncing / GPS spam

Historical: duplicate sims; fake GPS. Current `duburi_sub.parm` disables GPS
noise paths — keep one sim only.

## Stack cannot find duburi_ws

```zsh
export DUBURI_WS=/path/to/duburi_ws
# must contain install/setup.bash
```

## srot branch hunts USB board

Always use `duburi_sim stack` (forces `flight_controller:=pixhawk`) or pass that
arg yourself. See [INTEGRATION_DUBURI_WS.md](INTEGRATION_DUBURI_WS.md).
