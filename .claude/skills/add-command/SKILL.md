---
name: add-command
description: Scaffold a new /duburi/move verb — COMMANDS registry row, Duburi facade method, and pytest stub. Use when adding a new movement or payload command to the AUV.
disable-model-invocation: true
---

# Add a /duburi/move verb

**Usage:** `/add-command <verb_name> "<one-line description>"`

Adding a verb requires editing exactly **two** files (plus a test). The action server, the
`duburi` CLI, and the `DuburiClient` all pick the verb up automatically from the registry —
no other file needs editing.

## Steps

1. **Registry** — `src/duburi_control/duburi_control/commands.py`
   Add a `CommandSpec` row to the `COMMANDS` dict. Match the shape of an existing verb
   (`move_forward`, `yaw_left`, `drop_marker`). Set `verb`, `timeout`, and any
   goal-field defaults the verb reads.

2. **Facade method** — `src/duburi_control/duburi_control/duburi.py`
   Add a same-named method on `Duburi`. Follow the lock-and-dispatch pattern of a
   neighboring verb:
   - acquire `self._lock` (or respect heading_lock for Ch4 verbs),
   - check `_abort_event` once per loop tick,
   - bounded `while` with a `time.monotonic()` timeout,
   - single return path,
   - on exception → `self.stop()`.
   Delegate motion shaping to the right `motion_*.py` helper rather than inlining RC math.

3. **Test** — `src/duburi_control/test/test_commands.py`
   Add a stub asserting the verb is in `COMMANDS` and maps to a `Duburi` attribute.
   Mirror existing parametrized checks.

## After

```bash
./build_dubomini.sh
ros2 run duburi_planner duburi <verb_name> --help   # CLI auto-generated from COMMANDS
colcon test --packages-select duburi_control --pytest-args -k commands
```

If the verb closes a control axis, ask the `mavlink-reviewer` agent to check mode
preconditions and RC channel direction before pool use.
