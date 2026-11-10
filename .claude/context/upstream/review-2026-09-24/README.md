# Ecosystem review 2026-09-24: the handoff to the SROT repos

**Tracking issue:** fh1m/mongla_ws#53, which links every PR, issue and draft from this review. **Host fixes:** fh1m/mongla_ws#43.

**Who this is for:** the Claude Code session, or the person, that has write access to `RakibulIslam1/srot-control-board`, `srot-ground-station` and `srot-esc-flasher`.

**Why it was a folder here first:** the review ran in a session whose GitHub App has no access to the RakibulIslam1 repos. Both `git push` and "create issue" returned **403**. Everything is written out here instead, ready to file, following the same convention as the `pr-*.md` asks one level up.

**Rules carried over** (see `../README.md` and `platform/cross-repo-contract.md`):
- mongla_ws never commits to their repos. **Pull requests and issues only.**
- **Written here is not filed.** Everything below is now filed; the table at the bottom has each URL.

---

## 1. `srot-control-board`: one PR (5 fix commits)

```bash
cd srot-control-board
git fetch origin && git checkout -b review-2026-09-24/safety origin/main   # base was f1d3ba9
git am ../mongla_ws/.claude/context/upstream/review-2026-09-24/srot-control-board/patches/*.patch
pio run          # NOT compiled in the review container (registry blocked) -- every env must build
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

### Round 2 (deeper firmware pass + control SOTA), same filing rule

| file | severity |
|---|---|
| 22 A running autotune overrides every SURFACE failsafe; no mode change stops it; at the end it disarms at depth | High |
| 23 MOTOR_DETECT ignores FRAME_REVERSE: a "successful" detect on this hull (FRAME_REVERSE=1) reverses every axis | High |
| 24 Pico or ESC loss is invisible to arming, failsafes and MAVLink, and a reset Bluejay ESC never re-arms under a depth-holding demand | High |
| 25 MOTOR_TUNE writes an inflated MOT_SPIN_MIN (used on every flight) and an RPM Ki 2000× too strong | High |
| 26 The BNO085 mount remap is done on Euler angles, not the quaternion: roll capped at ±90°, style_roll commands a huge pitch | High |
| 27 / 28 Addenda to 03 (the ArduPilot AC_PID reference) and 04/20 (baro OSR-256 noise numbers; LINEAR_ACCELERATION not enabled) | — |
| 29 An aborted autotune keeps partial gains live; the relay has no hysteresis | Medium |
| 30 The Pico's USB printf can block for up to 1 s and trip its 250 ms watchdog | Medium |
| 31 The RPM target is linear in a thrust-linear demand (it should be √); the same mistake is in thrust trim | Medium |
| 32 The angular "drag feedforward" is positive feedback on measured rate | Medium (latent, defaults 0) |
| 35 ESC_FLASHING.md puts Bluejay PWM (48/24 kHz) inside or next to the pinger bands; move to 96 kHz before any hydrophone work (host: fh1m/mongla_ws#45) | Medium |

**Caution: Not yet drafted.** The round-2 drafting was stopped part-way. What remains lives in the research reports below. Draft them from the reports, verifying each claim at `f1d3ba9` first:
- **[`research/firmware_deep.md`](research/firmware_deep.md):** M2 move brake is open-loop; M3 no max-depth limit; M4 reversals near zero; M7 SD log format and ~5 Hz actual rate; M8 unpinned libraries; the 10 Lows; the failsafe matrix; and the proposals (a single failsafe evaluator, Pico/ESC health as an input, a self-describing log).
- **[`research/cross_stack.md`](research/cross_stack.md):**
  - A: a move cut short by a failsafe is ACKed 100 %;
  - B: SROT_MOVE forces AUTO out of SURFACE;
  - C: a Pico reset is invisible to the Pi;
  - D: the LEAK/ESPNOW failsafes are off by default, and SAFETY_GATES is never enforced;
  - E: reconnecting reboots the board via DTR;
  - F: IMU stamps are the send time;
  - also TIMESYNC, the latency and link budgets (1 KB TX buffer bursts), and contract-drift gaps (the drift test skips in this layout; FLARE_ORDER is host-only).

  The **host half of A and B is fixed** in fh1m/mongla_ws#43 (`5cebe0d`); the firmware half still needs its draft.
- **[`research/control_sota.md`](research/control_sota.md)** and **[`research/perception_sota.md`](research/perception_sota.md):** the SOTA comparisons with citations. Their host items are filed as fh1m/mongla_ws#44–#52.
- **[`research/cad_review.md`](research/cad_review.md):** the full B-matrix derivation behind issue 00 and fh1m/mongla_ws#9.

## 3. `srot-ground-station`: issues

File each file in [`srot-ground-station/issues/`](srot-ground-station/issues/) the same way; evidence is at `1adc14c`. The top three are **High**:
- the LoRa uplink queue has no expiry or priority, so a stale ARM or MOVE is sent on reconnect and DISARM is silently dropped;
- Bondor accepts any sender's HEARTBEAT, so the Arm/Disarm toggle can send ARM;
- after a window reload, Disarm is disabled while "Link OK" is shown.

## 4. `srot-esc-flasher`: issues

[`srot-esc-flasher/issues/`](srot-esc-flasher/issues/), with evidence at `c30b843`:
- 01: InterfaceTestAlive always ACKs OK.
- 02: Bluejay PWM 24/48 kHz sits in the pinger band.
- 03: `ESC_FLASHING.md` claims a false "open-loop" safety property.

---

## What is NOT here, and where it is
- **Host-side (mongla_ws) fixes** from this review are on branch `claude/zealous-pascal-sdgidr`, with their PR on fh1m/mongla_ws.
- **Host-side issues** are filed directly on fh1m/mongla_ws; the tracking issue links them all.
- **Duplicates we deliberately did not refile:** the joystick arm bypass (#6), mixer scale-down / PIDs wind up (#20), REQUEST_MESSAGE (#5), the flasher's SET_ADDRESS abort (flasher #1), and the frame selector (#25; issue 00 builds on it).

## Filed as

All of it was filed on 2026-11-10. The drafts sat here seven weeks because the session
that wrote them could not reach these repos.

| draft | repo | filed as |
|---|---|---|
| safety PR (5 commits) | srot-control-board | PR [#75](https://github.com/RakibulIslam1/srot-control-board/pull/75) |
| 00 | srot-control-board | [#46](https://github.com/RakibulIslam1/srot-control-board/issues/46) |
| 01 | srot-control-board | [#47](https://github.com/RakibulIslam1/srot-control-board/issues/47) |
| 02 | srot-control-board | [#48](https://github.com/RakibulIslam1/srot-control-board/issues/48) |
| 03 | srot-control-board | [#49](https://github.com/RakibulIslam1/srot-control-board/issues/49) |
| 04 | srot-control-board | [#50](https://github.com/RakibulIslam1/srot-control-board/issues/50) |
| 05 | srot-control-board | [#51](https://github.com/RakibulIslam1/srot-control-board/issues/51) |
| 06 | srot-control-board | [#52](https://github.com/RakibulIslam1/srot-control-board/issues/52) |
| 07 | srot-control-board | [#53](https://github.com/RakibulIslam1/srot-control-board/issues/53) |
| 08 | srot-control-board | [#54](https://github.com/RakibulIslam1/srot-control-board/issues/54) |
| 09 | srot-control-board | [#55](https://github.com/RakibulIslam1/srot-control-board/issues/55) |
| 10 | srot-control-board | [#56](https://github.com/RakibulIslam1/srot-control-board/issues/56) |
| 12 | srot-control-board | [#57](https://github.com/RakibulIslam1/srot-control-board/issues/57) |
| 13 | srot-control-board | [#58](https://github.com/RakibulIslam1/srot-control-board/issues/58) |
| 14 | srot-control-board | [#59](https://github.com/RakibulIslam1/srot-control-board/issues/59) |
| 15 | srot-control-board | [#60](https://github.com/RakibulIslam1/srot-control-board/issues/60) |
| 16 | srot-control-board | [#61](https://github.com/RakibulIslam1/srot-control-board/issues/61) |
| 17 | srot-control-board | [#62](https://github.com/RakibulIslam1/srot-control-board/issues/62) |
| 18 | srot-control-board | [#63](https://github.com/RakibulIslam1/srot-control-board/issues/63) |
| 19 | srot-control-board | [#64](https://github.com/RakibulIslam1/srot-control-board/issues/64) |
| 20 | srot-control-board | [#65](https://github.com/RakibulIslam1/srot-control-board/issues/65) |
| 22 | srot-control-board | [#66](https://github.com/RakibulIslam1/srot-control-board/issues/66) |
| 24 | srot-control-board | [#67](https://github.com/RakibulIslam1/srot-control-board/issues/67) |
| 25 | srot-control-board | [#68](https://github.com/RakibulIslam1/srot-control-board/issues/68) |
| 26 | srot-control-board | [#69](https://github.com/RakibulIslam1/srot-control-board/issues/69) |
| 29 | srot-control-board | [#70](https://github.com/RakibulIslam1/srot-control-board/issues/70) |
| 30 | srot-control-board | [#71](https://github.com/RakibulIslam1/srot-control-board/issues/71) |
| 31 | srot-control-board | [#72](https://github.com/RakibulIslam1/srot-control-board/issues/72) |
| 32 | srot-control-board | [#73](https://github.com/RakibulIslam1/srot-control-board/issues/73) |
| 35 | srot-control-board | [#74](https://github.com/RakibulIslam1/srot-control-board/issues/74) |
| 01 | srot-ground-station | [#7](https://github.com/RakibulIslam1/srot-ground-station/issues/7) |
| 02 | srot-ground-station | [#8](https://github.com/RakibulIslam1/srot-ground-station/issues/8) |
| 03 | srot-ground-station | [#9](https://github.com/RakibulIslam1/srot-ground-station/issues/9) |
| 04 | srot-ground-station | [#10](https://github.com/RakibulIslam1/srot-ground-station/issues/10) |
| 05 | srot-ground-station | [#11](https://github.com/RakibulIslam1/srot-ground-station/issues/11) |
| 06 | srot-ground-station | [#12](https://github.com/RakibulIslam1/srot-ground-station/issues/12) |
| 07 | srot-ground-station | [#13](https://github.com/RakibulIslam1/srot-ground-station/issues/13) |
| 08 | srot-ground-station | [#14](https://github.com/RakibulIslam1/srot-ground-station/issues/14) |
| 09 | srot-ground-station | [#15](https://github.com/RakibulIslam1/srot-ground-station/issues/15) |
| 01 | srot-esc-flasher | [#6](https://github.com/RakibulIslam1/srot-esc-flasher/issues/6) |
| 02 | srot-esc-flasher | [#7](https://github.com/RakibulIslam1/srot-esc-flasher/issues/7) |
| 03 | srot-esc-flasher | [#8](https://github.com/RakibulIslam1/srot-esc-flasher/issues/8) |
| 11 | srot-control-board | comment on [#41](https://github.com/RakibulIslam1/srot-control-board/issues/41), the same defect seen from the other side |
| 23 | srot-control-board | comment on [#34](https://github.com/RakibulIslam1/srot-control-board/issues/34) (PR Q), the same finding |
| 27 | srot-control-board | comment on [#49](https://github.com/RakibulIslam1/srot-control-board/issues/49), the issue it amends |
| 28 | srot-control-board | comment on [#50](https://github.com/RakibulIslam1/srot-control-board/issues/50), and pointed at from [#65](https://github.com/RakibulIslam1/srot-control-board/issues/65) |
