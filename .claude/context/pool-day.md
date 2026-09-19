# Pool day

The order of operations for an in-water session, and the reasons behind it. Nothing here is a
ritual: every step exists because skipping it cost a run.

---

## Before the vehicle gets wet

```bash
source scripts/pool_session.sh <label>     # SOURCE it in every terminal — one folder per run
ros2 run duburi_manager bringup_check --srot
```

`bringup_check` grades each subsystem and exits non-zero on a real fault. Read the three
lines that matter:

- **Firmware revision** — the host refuses to arm below its floor, because an older revision
  inverts yaw.
- **Barometer** — variance, not just presence. A per-sample plausibility band cannot see a
  noisy sensor, and an unhealthy barometer makes the board refuse **every** move verb while
  still arming.
- **Depth loop settled** — refuses to arm while the depth controller is saturated.

⛔ **The board's depth loop has never run closed in water.** Two armed bench checks gate every
on-board move, including `move_forward`. Until they pass, expect the board to deny moves. This
is the single most likely reason a session goes nowhere.

## Watch what the board is actually saying

```bash
ros2 run duburi_manager connect --watch
```

Both battery packs, per-ESC RPM and temperature, the depth controller's own command, error and
output, leak, kill, water temperature, heap and stacks. A value the board cannot stand behind
renders `--`, never `0.0`.

**Kill switch:** confirm it is live before anything is armed. Until the open firmware ask
lands, "power is live" and "I cannot hear the power board" can look the same from here.

## Bringing the stack up

```bash
ros2 launch duburi_manager bringup.launch.py vision:=true
```

Bring it up **bare first** if anything is unfamiliar — control only, then vision, then
localization. Every capability is a switch, and that is exactly so a fault can be isolated
instead of argued about.

Check before trusting vision:

```bash
ros2 run duburi_vision vision_check --camera forward --require-class gate
```

Require the model stem, a non-empty class allowlist, **and** a live alignment line. A missing
`<stem>.yaml` sidecar yields an empty allowlist and a silent `[]` every frame, with every node
looking healthy.

## Arming

1. **Propellers clear.** Hands, tether, tools, cable ties.
2. **A human on the kill switch**, watching the vehicle, not the laptop.
3. `ros2 run duburi_planner duburi arm` — and expect the interlocks to refuse if something is
   wrong. A refusal here is the system working.
4. First motion is a small one, on the surface, with someone able to reach it.

## During a run

- **`stop`, `surface` and `disarm` always work** — they bypass the busy gate, so they work
  mid-mission and mid-verb.
- Ctrl-C on the manager stops and disarms.
- The board surfaces on its own if the link goes quiet for 5 s, if it leaks, or if the
  thruster battery sags — whether or not the Pi is alive.
- Record the run:

```bash
scripts/pool_record.sh record <label>
```

## After each run

- A scorecard lands in `DUBURI_RUN_DIR`: every verb, its outcome, the goal id, what the camera
  saw. Read it before deciding what the next run changes.
- Replay the bag rather than re-flying the same experiment:

```bash
scripts/pool_record.sh replay <bag-dir>
```

- **Write the number down.** A measurement that stays in a terminal scrollback becomes an
  opinion by the next session. [`measured-bars.md`](measured-bars.md) is where it goes, with
  its conditions.

## Change one thing at a time

The recurring failure in this project is not a broken subsystem; it is two changes in one run
and no way to tell which one moved the result. If the day has three questions, it has three
runs.

Related: [`foxglove-and-bags.md`](foxglove-and-bags.md) ·
[`srot-integration.md`](srot-integration.md) ·
[`capability-map.md`](capability-map.md) · [`ROADMAP.md`](ROADMAP.md)
