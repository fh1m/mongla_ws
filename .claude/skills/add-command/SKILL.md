---
name: add-command
description: Scaffold a new /mongla/move verb — COMMANDS registry row, Mongla facade method, and pytest stub. Use when adding a new movement or payload command to the AUV.
disable-model-invocation: true
---

# Add a /mongla/move verb

**Usage:** `/add-command <verb_name> "<one-line description>"`

Adding a verb requires editing exactly **two** files (plus a test). The action server, the
`mongla` CLI, and the `MonglaClient` all pick the verb up automatically from the registry —
no other file needs editing.

## Steps

1. **Registry** — `src/mongla_control/mongla_control/commands.py`
   Add a `CommandSpec` row to the `COMMANDS` dict. Match the shape of an existing verb
   (`move_forward`, `yaw_left`, `fire`). Set `verb`, `timeout`, and any
   goal-field defaults the verb reads.

2. **Facade method** — `src/mongla_control/mongla_control/mongla.py`
   Add a same-named method on `Mongla`. Follow the lock-and-dispatch pattern of a
   neighboring verb:
   - acquire `self._lock`,
   - check `_abort_event` once per loop tick,
   - bounded `while` with a `time.monotonic()` timeout,
   - single return path,
   - on exception → `self.stop()`.
   A board motion is one `SROT_MOVE` through `SrotFC` (see `move_forward`); a host loop writes
   `fc.manual(...)` through `motion_writers.py` rather than inlining MANUAL_CONTROL math.

3. **Test** — `src/mongla_control/test/test_commands.py`
   Add a stub asserting the verb is in `COMMANDS` and maps to a `Mongla` attribute.
   Mirror existing parametrized checks.

## After

```bash
./build_mongla.sh
ros2 run mongla_planner mongla <verb_name> --help   # CLI auto-generated from COMMANDS
colcon test --packages-select mongla_control --pytest-args -k commands
```

If the verb closes a control axis, ask the `srot-reviewer` agent to check mode
preconditions and RC channel direction before pool use.
