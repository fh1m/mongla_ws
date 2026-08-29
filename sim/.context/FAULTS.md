# Fault injection — making the recovery paths run

The stack handles a DVL that stops reporting, a camera that goes away, a
MAVLink link that dies, a sagging battery and a dead thruster. Every one of
those paths exists in the code and **none had ever executed in simulation**,
because there was no way to cause the fault. They were first exercised in the
pool, on the day, once.

Each fault is armed by setting a duration and clears itself when the time is
up; `0` clears it early. A duration rather than a toggle, because a fault you
have to remember to clear is one you will leave on and then debug for an hour.

```bash
ros2 param set /faults dvl_dropout_s 6.0        # 6 s with no bottom lock
ros2 param set /faults camera_loss_s 4.0        # both cameras stop
ros2 param set /faults mavlink_loss_s 3.0       # the autopilot link drops
ros2 param set /faults battery_sag_v 13.2       # pack sags; 0 restores nominal
ros2 param set /t200_curve dead_thrusters "[3]" # thruster 3 fails; [0] = none
ros2 topic echo /duburi/sim/faults              # what is currently armed
```

A dead thruster lives on `/t200_curve` because that node is already the only
thing between ArduSub and the propellers; routing it through `/faults` would be
a facade over a one-line parameter.

## What each one is measured to do

| Fault | Measured effect |
|---|---|
| `dvl_dropout_s` | `move_forward_dist 1.0` **refuses**: *"no progress — DVL still reads 0.000 m after 15 s at 40 % thrust … refusing to keep driving blind"*. Healthy, the same command returns 1.011 m. After the fault clears: 1.018 m. |
| `camera_loss_s` | `image_raw` 10.50 Hz → **0.00 Hz** → 10.38 Hz on its own. `vision_align` returns **NO_CAMERA (3)**, *"camera pipeline not up (no camera_info)"* — distinct from the **LOST (1)** it returns when cameras work and the target simply is not there. |
| `mavlink_loss_s` | `arm` → **`NO_ACK`**. The same command succeeds before the cut and after it. |
| `battery_sag_v` | 16.0 V → 13.0 V widens the deadband to (1466, 1534) and costs **14.2 %** of the distance travelled by a fixed 6 s / 30 % command (0.662 m → 0.568 m). |
| `dead_thrusters` | Thruster 3 goes 12.72 N → **0.00 N** while thruster 1 is untouched at 10.11 N, and ArduSub flies on the remaining seven. |

Those numbers are the point of the table: a fault that does not change a
measurement is not injected, whatever the log says.

## Three things worth knowing

**The DVL is INTERPOSED, so `/faults` is now load-bearing.** The stack
subscribes to Gazebo directly (`SimDvlSource`), so nothing on the ROS side can
take the sensor away from it. The sensor therefore publishes
`dvl/velocity_raw` and this node republishes `dvl/velocity` — the same shape as
the T200 curve sitting on `cmd_thrust_linear`. **Without `/faults` running
there is no `dvl/velocity` at all and every `*_dist` verb refuses.** A test
asserts the model still publishes the raw name, because leaving it at
`dvl/velocity` would make dropouts quietly do nothing.

**Cameras are stopped with SIGSTOP, not by dropping frames.** Gating a
republish would mean moving two ~1 MB image streams to do nothing, and the
image bridge is a separate process anyway. The node clears every SIGSTOP on
shutdown — a frozen image bridge outlives the node and looks exactly like a
broken camera for the rest of the session.

**The MAVLink relay is opt-in.** ArduSub is a `udpclient` and the manager binds
the far end, so cutting the link means sitting between them.
`mavlink_relay:=true` points the SITL at 14559 and the injector forwards both
directions; with it off (the default) nothing in the fault path touches the
link the whole session depends on.

```bash
ros2 launch duburi_sim_bringup sim.launch.py mavlink_relay:=true
```

SIGSTOPping ArduSub would be simpler and is wrong: the FDM link is lock-stepped
to Gazebo, so freezing the autopilot freezes the whole simulation instead of
just the telemetry.

## A probe that lied

`/duburi/state` is **not** a MAVLink liveness probe. It kept updating at 22 Hz
through a fully cut link, because with `yaw_source=sim_dvl` its yaw comes from
Gazebo, not from the autopilot. Probe the link with something that must
traverse it — `arm` returning `NO_ACK` is the honest test.
