# Ecosystem review 2026-09-24: the handoff to the SROT repos

**Who this is for:** the Claude Code session, or the person, that has write access to `RakibulIslam1/srot-control-board`, `srot-ground-station` and `srot-esc-flasher`.

**Why it is a folder here and not PRs and issues there:** the review ran in a session whose GitHub App has no access to the RakibulIslam1 repos. Both `git push` and "create issue" returned **403**. Everything is written out here instead, ready to file, following the same convention as the `pr-*.md` asks one level up.

**Rules carried over** (see `../README.md` and `platform/cross-repo-contract.md`):
- mongla_ws never commits to their repos. **Pull requests and issues only.**
- **Written here is not filed.** When you file something, update the "Filed as" table at the bottom.

---

## 1. `srot-control-board`: one PR (5 fix commits)

```bash
cd srot-control-board
git fetch origin && git checkout -b review-2026-09-24/safety origin/main   # base was f1d3ba9
git am ../mongla_ws/.claude/context/upstream/review-2026-09-24/srot-control-board/patches/*.patch
pio run          # ⚠ NOT compiled in the review container (registry blocked) -- every env must build
git push -u origin review-2026-09-24/safety
```
Open the PR with the title and body in [`srot-control-board/PR_BODY.md`](srot-control-board/PR_BODY.md). That body lists the bench checks, and the **one behaviour change to verify in Bondor**: MOTOR_TUNE is now "arm, then select".

If `git am` conflicts because `main` has moved, resolve against the commit messages. Each one states the defect and the intent in full.

## 2. `srot-control-board`: issues

File one issue per file in [`srot-control-board/issues/`](srot-control-board/issues/). **The title is the first line** (without the `# `) and the rest is the body. They are in severity order; each cites evidence at `f1d3ba9` and has a "Related" line pointing at existing PRs and issues.

| file | severity |
|---|---|
| 00 tunnel_5dof: roll demand becomes heave; the roll trim-learner fights depth; the 8 sites a frame change must touch | High |
| 01 NVS write on core 0 stalls core 1 (control + DShot) | High |
| 02 A defaults-version bump silently resets FRAME_REVERSE and the motor directions, and arming is still allowed | High |
| 03 PID integrator limit is in error·s, so I authority is 0.4–10 % | High |
| 04 Depth D-term differentiates a 20 Hz sample-and-hold barometer | High |
| 05 IMU self-heal starves the barometer and leak reads | High |
| 06–17 VFR_HUD ungated, telemetry blackout during param download, MANUAL_CONTROL multi-sender, payload §7, LEAK_EN=0, kill not consumed, MAVLink conformance, STATUSTEXT drops, TURN wrap, Bar30 blocking, ESP-NOW NaN/spoofing, Pico PI windup | Medium |
| 18 Low-severity control-loop hygiene (checklist) | Low |
| 19–20 Proposals: IMU off the shared I2C bus, and an on-board vertical state estimator | Proposal |

## 3. `srot-ground-station`: issues

File each file in [`srot-ground-station/issues/`](srot-ground-station/issues/) the same way; evidence is at `1adc14c`. The top three are **High**:
- the LoRa uplink queue has no expiry or priority, so a stale ARM or MOVE is sent on reconnect and DISARM is silently dropped;
- Bondor accepts any sender's HEARTBEAT, so the Arm/Disarm toggle can send ARM;
- after a window reload, Disarm is disabled while "Link OK" is shown.

## 4. `srot-esc-flasher`: issue

[`srot-esc-flasher/issues/01-flasher-testalive-always-ok.md`](srot-esc-flasher/issues/). Evidence is at `c30b843`.

---

## What is NOT here, and where it is
- **Host-side (mongla_ws) fixes** from this review are on branch `claude/zealous-pascal-sdgidr`, with their PR on fh1m/mongla_ws.
- **Host-side issues** are filed directly on fh1m/mongla_ws; the tracking issue links them all.
- **Duplicates we deliberately did not refile:** the joystick arm bypass (#6), mixer scale-down / PIDs wind up (#20), REQUEST_MESSAGE (#5), the flasher's SET_ADDRESS abort (flasher #1), and the frame selector (#25; issue 00 builds on it).

## Filed as (update when filed)

| item | repo | URL |
|---|---|---|
| safety PR (5 commits) | srot-control-board | — |
| issues 00–20 | srot-control-board | — |
| issues 01–09 | srot-ground-station | — |
| issue 01 | srot-esc-flasher | — |
