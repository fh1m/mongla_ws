# duburi_interfaces — the whole cross-package surface

**Messages only · [`src/duburi_interfaces`](../../../../src/duburi_interfaces)**

One action and one state topic. That is the entire contract between the mission layer and the
vehicle; everything else in this codebase is an implementation detail behind it.

---

## `Move.action` — one action, thirty verbs

Every command is the same message with a different `cmd` string. Adding a verb does not add a
topic, a service or a message type.

```
# goal
string cmd            # 'arm', 'move_forward', 'vision_align', …
float32 duration      # seconds, for timed moves
float32 gain          # a SPEED CAP in % thrust, not a target speed
float32 target        # depth, heading — whatever the verb means by it
…plus the vision fields: camera, target class, axes, pixel offsets, fill, fire channels

# result
bool    success
string  message       # human-readable outcome
float32 final_value   # the verb's typed outcome code
```

Three decisions inside this file are worth knowing:

**An action, not a topic.** A goal can be cancelled, it reports when it finishes, and only one
runs at a time. A mission interrupted mid-move still lands somewhere defined — a topic would
give neither the cancellation nor the answer.

**`gain` is a ceiling, never a target.** `gain=40` means "never exceed 40 % thrust". A control
loop is free to use less, and usually does.

**The vision verbs never fail.** They return `success=True` with an outcome code — aligned,
lost, timed out, no camera, aborted — because "I could not see it" is a normal outcome for a
mission to branch on, not an exception to crash into.

## `DuburiState.msg` — what the vehicle is doing

```
std_msgs/Header header
bool    armed
string  mode
float32 yaw_deg
float32 depth_m
float32 battery_voltage
```

Published when something changes, plus a slow heartbeat.

**A missing number is `NaN`, never `0.0`.** That is a safety rule, not a style choice: zero
depth is a *reading* — it means the surface — so a consumer cannot tell an absent sensor from a
surfaced vehicle if absence is written as zero. The same rule holds on the board, which
suppresses values it cannot stand behind rather than sending a plausible number.

## The rest

| File | Purpose |
|---|---|
| `msg/TargetPose.msg` | the pose of a recognised prop |
| `msg/TargetContours.msg` | outlines from segmentation |
| `msg/TargetCorrespondences.msg` | image points matched to model points, for pose solving |

---

Related: [`duburi_manager`](../duburi_manager/README.md) (serves the action) ·
[`duburi_planner`](../duburi_planner/README.md) (sends the goals)
