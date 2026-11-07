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
   Add a row to the `COMMANDS` dict: `'help'` (the CLI's one-liner), `'fields'` (the
   `Move.Goal` fields the verb reads, by name) and `'defaults'` (every field that may be
   omitted; a field without a default is required). Copy the shape of a neighbour
   (`move_forward`, `yaw_left`, `fire`).

2. **Facade method** — `src/mongla_control/mongla_control/mongla.py`
   Add a method on `Mongla` with the same name, taking the row's fields as parameters
   (plus `on_progress=None`). It returns a `Move.Result`.
   - A board motion is one line: `return self._board('<verb>', on_progress, ...)`, as
     `move_forward` does — one `SROT_MOVE`, braked on the board.
   - A host loop runs under `self.lock`, checks the abort flag once per tick, is bounded
     by a `time.monotonic()` deadline, and writes thrust through `motion_writers.py`
     rather than inlining MANUAL_CONTROL arithmetic.

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
preconditions and the MANUAL_CONTROL axis signs before pool use.
