# Future TODO -- shrink `commands.py` registry

> **Status:** parked. Do NOT implement before the next pool test.
> The current `COMMANDS = { ... }` dict is verbose but human-traceable;
> any compaction needs to keep `rg "'cmd_name'" src/` working as the
> primary debugging entry point.

---

## Why we want to shrink it

Look at `src/duburi_control/duburi_control/commands.py` today:

* 24 verbs, ~190 lines.
* Every vision verb repeats `'camera': 'laptop', 'target_class':
  'person', 'on_lost': 'fail', 'stale_after': 1.5` -- 6 verbs * 4
  fields = 24 lines of identical default boilerplate.
* All 4 `move_*` verbs have IDENTICAL `fields` and `defaults`.
* All 2 `yaw_*` verbs have IDENTICAL `fields` and `defaults`.

So roughly 60-80 of 190 lines are pure copy-paste. Removing it
would cut about 1/3 of the file.

---

## Why we want to keep it loud

The user wrote (verbatim, paraphrased): "I dont want one source of
commands, THATS WHAT I WANT TO AVOID -- when each command runs and
something goes wrong I want to grep ONE name and find ONE row."

Concretely, when a pool mission misbehaves we today do:

```
rg "'move_forward'" src/             # finds the spec row
rg "def move_forward"  src/          # finds the implementation
```

Any shrinkage MUST preserve both greps. That rules out:

* DSL macros that build verb names by string concatenation
  (`f'move_{direction}'`) -- breaks `rg`.
* `__getattr__` / metaclass tricks that synthesize methods at
  import time -- breaks `rg "def "`.
* Auto-generated `COMMANDS` from class introspection -- breaks
  `rg "'verb_name'"`.

---

## Two viable shapes (pick when it's time)

### Option A -- "shared default profiles" (smallest diff, recommended)

Pull repeated default sets into named constants at the top of
`commands.py`:

```python
_VISION_BASE = {
    'camera': 'laptop',
    'target_class': 'person',
    'on_lost': 'fail',
    'stale_after': 1.5,
}
_TRANSLATE_BASE = {
    'gain': 80.0,
    'settle': 0.0,
}
_YAW_BASE = {
    'timeout': 30.0,
    'settle': 0.0,
}

COMMANDS = {
    'move_forward': {
        'help':     'Drive forward...',
        'fields':   ['duration', 'gain', 'settle'],
        'defaults': {**_TRANSLATE_BASE},
    },
    ...
    'vision_align_yaw': {
        'help':     '...',
        'fields':   ['camera', 'target_class', 'duration', 'deadband',
                     'kp_yaw', 'on_lost', 'stale_after'],
        'defaults': {**_VISION_BASE, 'duration': 15.0,
                     'deadband': 0.18, 'kp_yaw': 60.0},
    },
    ...
}
```

Pros:

* Verb names still appear literally as dict keys -- `rg` works.
* Each verb's `defaults` is still ONE line readable at a glance.
* Zero new abstractions; `**_VISION_BASE` is plain Python.
* Saves ~40-60 lines.

Cons:

* Operator now has to look up the constant to know what
  `_VISION_BASE` resolves to. Mitigation: keep the constants in
  the SAME file, no imports.

### Option B -- "fields tables" (denser, less recommended)

Group fields by family at the top:

```python
_TRANSLATE_FIELDS = ['duration', 'gain', 'settle']
_YAW_FIELDS       = ['target', 'timeout', 'settle']
_VISION_AXIS_FIELDS = ['camera', 'target_class', 'duration', 'deadband',
                       'on_lost', 'stale_after']

COMMANDS = {
    'move_forward':  {'help': 'Drive forward...',
                      'fields': _TRANSLATE_FIELDS,
                      'defaults': {**_TRANSLATE_BASE}},
    ...
}
```

Pros:

* Saves more lines.

Cons:

* Field LIST is no longer literal in the verb's row, so reading
  `commands.py` no longer tells you what the verb takes -- you
  have to chase the constant.
* Direct conflict with the user's "human, traceable, lean but
  loud" preference.

Recommended verdict: **Option A only.** Defer Option B unless we
ever exceed ~30 verbs.

---

## Pre-implementation checklist

Before opening the PR:

* [ ] `rg "'move_forward'"` (and one verb from every family)
      still returns exactly one hit in `commands.py`.
* [ ] `rg "def move_forward"` still returns exactly one hit per
      family in `duburi.py` / `vision_verbs.py`.
* [ ] `command-reference.md` is regenerated and the `impl path`
      column still resolves.
* [ ] `colcon test` passes.
* [ ] `duburi --help` for every verb still prints the right
      defaults.

---

## Cross-references

* Today's registry: `src/duburi_control/duburi_control/commands.py`
* Field-resolution algorithm: `commands.fields_for()` (top of same
  file)
* Per-verb impl pointers: [`command-reference.md`](./command-reference.md)
* Why we keep verbs distinct in the first place:
  [`mission-cookbook.md`](./mission-cookbook.md) "one verb, one
  closed loop" section.
