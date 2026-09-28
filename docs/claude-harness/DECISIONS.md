# Claude harness — decision ledger

One entry per change to the Claude Code environment. Problem, evidence, what
changed, measured result, how to undo it. The environment should be
**understandable rather than magical**.

Backups of every file touched: `~/.claude/backups/harness-<timestamp>/`.

---

## D1 — ECC fired 24 hooks on a plugin that was already disabled

**Problem.** GateGuard blocked the first `Bash` call of every session with a
"present these facts" prompt, and the operator reported ECC warnings and wasted
tokens. `enabledPlugins["ecc@ecc"]` was already `false`.

**Evidence.** ECC is installed as a **marketplace**, and
`~/.claude/plugins/marketplaces/ecc/hooks/hooks.json` registers **24 hooks** —
9 `PreToolUse`, 7 `Stop`, 2 `SessionStart`, 2 `PostToolUse`, 2
`PostToolUseFailure`, 1 `PreCompact`, 1 `SessionEnd`. Marketplace hooks load
from the *registration*, not from `enabledPlugins`, so disabling the plugin
disabled its skills and agents and left every hook running.

⭐ **The same defect, twice more.** Auditing the other marketplaces the same
way: `cco` (9 hooks) and `last30days-skill` (1 hook) also had all their plugins
disabled and all their hooks live. Only `ros2-engineering-skills` (2 hooks) was
legitimately enabled.

**Change.** Unregistered `ecc`, `cco` and `last30days-skill` from
`settings.json → extraKnownMarketplaces` and from
`plugins/known_marketplaces.json`, and dropped their now-meaningless
`enabledPlugins: false` entries. Belt and braces, each marketplace's
`hooks/hooks.json` renamed to `hooks.json.disabled`.

**Measured result.** Registered hooks **40 → 6**: 4 from this project
(`block_install`, the graphify hint, `py_check`, `pkg_test`) and 2 from
`ros2-engineering-skills`. Every remaining hook belongs to an enabled plugin or
to this repo.

**Rollback.** `mv hooks.json.disabled hooks.json` in each marketplace, then
`/plugin marketplace add` for the three names — or restore
`~/.claude/backups/harness-20260928-173827/{settings.json,known_marketplaces.json}`.
Nothing was deleted: ECC's 94 MB cache and all three marketplace trees are
untouched on disk.

⚠ **Not yet verified across a restart.** The hook count above is what is
registered on disk; that hooks stop firing needs one new session to confirm.

---

## D2 — `bun add` failed with EACCES, and the error named the wrong cause

**Problem.** `error: EACCES accessing temporary directory. Please set
$BUN_TMPDIR or $BUN_INSTALL` — while **both** were already set.

**Evidence.** Bisected in a bare `env -i` shell, which is what a hook actually
gets:

| environment | result |
|---|---|
| neither var | **installs fine** |
| `BUN_TMPDIR` only | **installs fine** |
| `BUN_INSTALL` only | EACCES |
| both | EACCES |

So the poison is `BUN_INSTALL`, not the temp directory the message blames. The
reason: `$BUN_INSTALL/install` and `$BUN_INSTALL/install/cache` are
**root-owned** (`drwxr-xr-x root root`) — someone ran bun under sudo once — and
`chown` needs root to undo. 42 MB, 5 entries, all disposable npm cache.

⚠ **The first fix was wrong and the measurement caught it.** The obvious read
was "the hook environment lacks `BUN_TMPDIR` because `.zshrc` is not sourced by
hooks", and setting both vars in `settings.json → env` *reproduced the failure
rather than fixing it*. The bisect above is what found the real cause.

**Change.** Point the cache at a writable path — no sudo, nothing root-owned
touched — in both places that matter:

* `~/.claude/settings.json → env`, which Claude Code applies to hooks and Bash
  alike (absolute paths: that block is **not** shell-expanded);
* `~/.zshrc`, for interactive shells, with the reason written beside it.

```
BUN_INSTALL            $HOME/.bun
BUN_TMPDIR             $HOME/.cache/bun/tmp
BUN_INSTALL_CACHE_DIR  $HOME/.cache/bun/install     <- the fix
```

**Measured result.** `bun add left-pad` in a bare `env -i` shell: 2 of 2 runs
installed, against 3 of 3 failing before.

**Rollback.** Drop `BUN_INSTALL_CACHE_DIR` from both files; `.zshrc` backup is
in the same backup directory. The proper repair, if root is ever available, is
`sudo chown -R "$USER" ~/.bun/install`.
