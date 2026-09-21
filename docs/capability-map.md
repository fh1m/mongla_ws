---
layout: default
title: Capability Map
description: "What this vehicle can do, and the evidence behind every claim."
---

# Capability Map

### What this vehicle can do, and how we know

Every row carries **evidence** — a file in one of our repositories, or a measurement with a
date — and a **state**. The states are not blurred:

| | state | meaning |
|---|---|---|
| 🟢 | **WATER** | verified with the vehicle in water |
| 🟣 | **BENCH** | verified on the bench, on real recorded footage, or against the live board |
| 🟡 | **BUILT** | implemented and covered by tests, never flown |
| 🔴 | **BLOCKED** | waiting on hardware or on a firmware merge |

A capability map that hides the last two is marketing. The blocked rows are listed in full at
the bottom, and **no row in this document is 🟢 yet** — this platform has not been in water.

<figure class="tally" id="tally" aria-live="polite">
  <noscript>The tally below counts this page's rows by state; it needs JavaScript.</noscript>
</figure>
<script>
(function(){
  /* This page counts itself. Every table row below whose last cell carries a
     state is tallied, so the bar can never disagree with the tables -- there is
     no second copy of the numbers to drift. */
  var states = [['🟢','WATER','water'],['🟣','BENCH','bench'],['🟡','BUILT','built'],['🔴','BLOCKED','blocked']];
  function run(){
    var n = {}, total = 0;
    document.querySelectorAll('main table tbody tr').forEach(function(tr){
      var c = tr.lastElementChild; if (!c) return;
      states.forEach(function(s){ if (c.textContent.indexOf(s[0]) >= 0 && c.textContent.indexOf(s[1]) >= 0){ n[s[1]] = (n[s[1]]||0)+1; total++; } });
    });
    if (!total) return;
    var f = document.getElementById('tally'), bar = '', key = '';
    states.forEach(function(s){
      var k = n[s[1]] || 0;
      bar += '<i class="t--' + s[2] + '" style="flex-grow:' + k + '"></i>';
      key += '<span class="t--' + s[2] + '"><b>' + k + '</b> ' + s[1].toLowerCase() + '</span>';
    });
    f.innerHTML = '<div class="tally__bar">' + bar + '</div><div class="tally__key">' + key +
      '</div><figcaption>' + total + ' capabilities on this page, counted from its own tables ' +
      'as it loads.</figcaption>';
  }
  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', run); else run();
})();
</script>

New here? Read **[The Shift](the-shift.html)** first — it explains the two halves of the system
and why the split exists.

---

## 1. Control: keeping the vehicle where you put it

| Capability | How it works | Evidence | State |
|---|---|---|---|
| Inner control loop | **500 Hz**, our source, on a core reserved for it | fw `include/config.h` (`CONTROL_LOOP_HZ 500`), core pinning in `src/main.cpp` | 🟣 BENCH |
| Attitude and heading hold | ours: angle → rate → torque, with gains we can change | fw `src/control/attitude.*`, `task_control_loop.cpp` | 🟣 BENCH |
| Heading stability at rest | drift **< 0.01 °/min**, peak-to-peak **< 0.08° over 8 min** | `measured-bars.md` (srot heading, 2026-09-07, at rest) | 🟣 BENCH |
| Thruster mixing | vectored-8 geometry, and we can read the matrix (⚠ the current CAD hull has five thrusters — see `vehicle-spec.md`) | fw `src/control/mixer.h`, `docs/THRUSTER_MAP.md` | 🟣 BENCH |
| Saturation behaviour | the whole demand scales together so the *shape* of the move survives; the host also fits the demand yaw-first before sending | fw `mixer.h`; host `mongla_control/allocation.py` | 🟡 BUILT |
| Flight modes | **eleven**, including two tuning modes and a pattern runner | fw `include/state_types.h` | 🟣 BENCH |
| Movement primitives | **ten**, run and braked on the board: forward, back, strafe L/R, turn, dive, stop, hold, style, arc | `srot_protocol.py` (`CMD_SROT_MOVE = 31000`); fw `src/comms/mav_commands.cpp` | 🟡 BUILT |
| Automatic gain tuning | a relay self-tune on the board, driven from an operator command | fw `src/control/autotune.h`; host `mongla_manager/srot_autotune.py` | 🟡 BUILT |
| Depth hold | implemented, **never closed in water** — two bench checks gate every automatic move until it is | fw `src/control/depth_control.cpp`; `.claude/context/platform/srot-integration.md` | 🔴 BLOCKED |
| Link budget | one USB-C cable at **18.8 %** of capacity; heartbeat margin **10×** the failsafe | `measured-bars.md` §4 | 🟣 BENCH |

### Failsafes — the vehicle protects itself without the Pi

| Trigger | What the board does | Evidence | State |
|---|---|---|---|
| Water detected inside | surface; translation and yaw zeroed | fw `task_control_loop.cpp` | 🟡 BUILT |
| Thruster battery sagging (held 3 s, armed) | surface | fw `task_control_loop.cpp`, `config.h` | 🟡 BUILT |
| Operator link silent for 5 s | surface | `srot_protocol.py` (`GCS_FAILSAFE_MS = 5000`) | 🟡 BUILT |
| Companion computer silent | surface — but only once it has been heard at least once | fw `task_control_loop.cpp` | 🟡 BUILT |
| Surfaced and idle | auto-disarm | fw `config.h` | 🟡 BUILT |
| The command stream stops mid-move | authority fades to zero between 1.0 s and 1.5 s instead of latching | `measured-bars.md` §4 | 🟣 BENCH |
| Firmware too old to trust | the host **refuses to arm** below behaviour revision 10, because revision 10 inverted yaw | `srot_protocol.py` (`FW_BEHAVIOUR_REV_REQUIRED`), guarded by `test_doc_drift.py` | 🟡 BUILT |

---

## 2. Sensing: what the vehicle knows about itself

Every sensor is on the board, on one clock. Most of this table is not *new hardware* — it was
already inside the board and had nowhere to go until we asked.

| Capability | How it works | Evidence | State |
|---|---|---|---|
| Attitude and rotation rates | **on the flight board**, fused at up to 400 Hz, read at 500 Hz | fw `src/drivers/bno085.*` | 🟣 BENCH |
| Depth and water temperature | direct — plus the controller's own error, output and command, published for inspection | fw `src/drivers/bar30.cpp` | 🟣 BENCH |
| Leak and kill switch | both — and the kill switch's ambiguity is a known, written ask | `.claude/context/upstream/pr-h-kill-is-ambiguous.md` | 🟣 BENCH |
| Battery | **two packs**, electronics and thrusters, de-multiplexed by id | host `fc/srot_fc.py` (`note_battery`) | 🟣 BENCH |
| Per-thruster RPM | measured by the ESCs, returned on the command wire | fw Pico + `thruster_link.cpp`; needs Bluejay from `srot-esc-flasher` | 🟣 BENCH |
| Thruster presence | computed on the board and dropped before the wire — a one-line ask, open | `.claude/context/upstream/pr-f-esc-presence-on-the-wire.md`; 958/958 frames read zero with nothing attached | 🔴 BLOCKED |
| Trustworthy timestamps | stamped on the board's own clock: period **100.000 ms, sd 0.000** at the source, against **sd 4.741 ms** by arrival | `measured-bars.md` §14 | 🟣 BENCH |
| Reading the board live | `ros2 run mongla_manager connect` prints everything the board sends — no ROS graph, no built workspace needed | `mongla_manager/srot_connect.py` | 🟣 BENCH |
| A preflight that gates rather than guesses | `bringup_check --srot` grades each subsystem and exits non-zero on a real fault | `mongla_manager/bringup_check.py` | 🟣 BENCH |

---

## 3. Vision: what the vehicle sees

| Capability | How it works | Evidence | State |
|---|---|---|---|
| Detection hardware | a **Hailo-8** that does nothing else | `mongla_vision/detection/hailo.py` | 🟣 BENCH |
| Detection rate | **80.9 Hz** standalone, **53.9 Hz** through the live ROS graph | `measured-bars.md` §3 | 🟣 BENCH |
| Photon to detection | **18.0 ms** median | `measured-bars.md` §3 | 🟣 BENCH |
| Frame freshness | a mailbox that keeps the newest: **16.9 ms** stale against 396 ms | `measured-bars.md` §3; `cameras/v4l2_mailbox.py` | 🟣 BENCH |
| Segmentation | decoded without leaving the quantised domain: 31.5 ms → **0.93 ms**, bit-exact | `measured-bars.md` (Hailo segmentation); `detection/seg_decode.py` | 🟣 BENCH |
| Holding a target between detections | a **lock ladder** — tracker, optical-flow follower, then a feature anchor that held **175 consecutive frames** the detector had lost | `measured-bars.md` §2; `mongla_vision/lock_node.py` | 🟣 BENCH |
| Continuity budget | measured from 71 real gaps: the coast covers **91.5 %**, the track buffer **100 %** | `measured-bars.md` §2 | 🟣 BENCH |
| Lens geometry | measured per camera: **63.8° air, 46.7° water ±0.7°**, 25 views, held-out validated | `measured-bars.md` §3; `config/calibration/*.json` | 🟣 BENCH |
| Refraction through a flat port | corrected radially, by the same rectifier every metric consumer uses | `mongla_vision/optics.py` | 🟣 BENCH |
| Image preprocessing | measured in **17 configurations** and never positive; on the gate it destroyed 95 % of detections. Ships **off**, and a test keeps it off | `measured-bars.md` §1 | 🟣 BENCH |

---

## 4. Estimation: where the vehicle is

No DVL is fitted and no GPS reaches underwater, so every number in this table is built from the
IMU, the depth sensor and the downward camera. Without it, a distance move is a timed guess.

| Capability | How it works | Evidence | State |
|---|---|---|---|
| Velocity over the floor | optical flow from the downward camera, with rotation removed before scaling | `mongla_vision/flow/flow_node.py`; `measured-bars.md` §10 | 🟣 BENCH |
| Distance accuracy | three 30 cm slides: worst error **1.09 cm (3.6 %)**; recovered height 0.72 / 0.69 / 0.70 m against a 0.72 m tape | `measured-bars.md` §13 | 🟣 BENCH |
| Refusing to guess | 727 static intervals produced **zero** velocity reports: an unmeasurable interval is never published as "0 m/s" | `measured-bars.md` §10 | 🟣 BENCH |
| Height above the floor | from a single pixel on a known plane | `mongla_localization/floor_plane.py` | 🟡 BUILT |
| Pose filter | a right-invariant EKF over IMU, depth, flow, heading and fixes | `mongla_localization/inekf.py` | 🟡 BUILT |
| Late measurements | replayed at the instant they describe, with the tail re-run | `mongla_localization/retro.py`, `test_retro.py` | 🟡 BUILT |
| Absolute position | a fix from a prop whose location is known, or resection from two bearings | `course_map.py`, `resection.py`, `pnp_node.py` | 🟡 BUILT |
| Absolute heading | from a prop of known bearing, from pool lane lines, or from the floor tile grating | `heading_anchor.py`, `pool_lines.py`, `tile_grating.py` | 🟡 BUILT |
| Venue priors | a course file per competition, overridable on the deck without a rebuild, plus a tool to survey the real venue | `mongla_localization/courses/*.yaml`, `course_survey.py` | 🟡 BUILT |

---

## 5. Acting: missions, alignment and payload

| Capability | How it works | Evidence | State |
|---|---|---|---|
| Vision alignment | the same two verbs, now **50 Hz** on the srot path (soaked 90 s at 49.86 Hz, zero late ticks) over a hull the board stabilises at 500 Hz | `motion_vision.py`; `measured-bars.md` §3 | 🟣 BENCH |
| Downward-camera work | the same, and now without a streamed depth setpoint: the axes that need one are refused explicitly instead of silently doing nothing | `mongla_control/vision_verbs.py`, `test_srot_vision_actuation.py` | 🟡 BUILT |
| Driving through a target | unchanged — the verb is backend-independent | `motion_vision.move_loop` | 🟡 BUILT |
| Payload | MAVLink to the board's own outputs, with a typed outcome per shot: fired, refused, denied, no-ack, busy | `fc/base.py`, `fc/srot_fc.py` | 🟡 BUILT |
| Firing on a *fresh* frame | gated on a new live detection, so a frozen or predicted box can never trigger a shot | `motion_vision.py` | 🟡 BUILT |
| Knowing whether it fired | the board's own answer is carried back into the mission result | `mongla_control/vision_verbs.py` | 🟡 BUILT |
| Rationing a run | an opt-in budget: every task is asked whether it is still worth attempting, and the verdict is recorded | `mongla_planner/run_budget.py`, `missions/sauvc_full.py` | 🟡 BUILT |
| Giving up cleanly | a task deadline that cancels the goal in flight and hands the mission its fallback | `mongla_planner/client.py`, `mongla_dsl.task()` | 🟡 BUILT |
| Evidence after a run | a scorecard per run: every verb, its outcome, the goal id, what the camera saw, and optionally the frame itself | `mongla_dsl.py`, `vision_dsl.py` | 🟡 BUILT |
| Operator radio | LoRa through Bondor, including the competition's flare order sent to a hull already in the water | `mongla_manager/flare_order_send.py`; fw `src/comms/flare_order.h` | 🔴 BLOCKED |

---

## 6. What owning the stack actually gave us

Not the hardware — the *verbs*. Six things that follow from being able to read, measure or
change every layer, including the one that keeps the vehicle level:

1. **We can measure inside the loop.** The board publishes its own depth error, output and
   command. That is how we know the depth loop has never truly run — something a closed
   autopilot would never have told us.
2. **We can prove firmware and host agree.** A test here reads the firmware's headers
   directly and fails if our copy of a shared constant drifts. Two codebases, one truth.
3. **Sensors that were being thrown away came back.** Per-ESC RPM and current were computed
   on the board and discarded one line before the wire. We found them by reading the source.
4. **Time became trustworthy.** Because the board stamps its own messages, a late reading is
   still usable — we know when it was true, not merely when it arrived.
5. **The vehicle can be brought up one capability at a time.** Every feature is a switch with
   a defensible default, so a failure is isolated instead of argued about.
6. **A missing feature has an address.** Nine written asks, each with the evidence that
   produced it, sit in
   [`.claude/context/upstream/`](https://github.com/fh1m/mongla_ws/tree/main/.claude/context/upstream).
   The board is not final either: a capability that needs a new sensor is a conversation with
   the hardware team, not a wall.

---

## 7. The honest list: what is blocked, and on what

| Blocked | Why | Unblocked by |
|---|---|---|
| Depth hold in water | the board's depth loop has never run closed, and it gates every automatic move | two armed bench checks, in water |
| Thruster health | presence is computed then dropped before the wire, so "eight healthy" and "none attached" look identical | a one-line firmware merge |
| Distance moves on srot | `move_*_dist` are refused; they need a board-side measured move | firmware PR #23 |
| Velocity and position sent to the board | the board has no velocity ingest of any kind today | a firmware merge accepting the standard optical-flow message |
| Flare order over the air | a LoRa antenna inside an aluminium hull is inside a Faraday cage | an external antenna feedthrough, plus firmware PR #24 |
| Learned velocity estimation | needs thrusters fitted and spinning to gather data | thrusters on the hull |
| Flare detection range | a 16 mm pole is detectable only very close, and the real number is unmeasured | a pool session with the actual prop |
| Every mission | not one has been flown on this platform | water time |

Nothing here claims a pool result we have not taken. Where a row says BENCH, it means exactly
that: a bench, a recorded bag, or the board on a desk.

---

*The numbers are maintained in
[`measured-bars.md`](https://github.com/fh1m/mongla_ws/blob/main/.claude/context/measured-bars.md),
where each is recorded with the method that produced it, the conditions, and the bar it has to
clear. Tests read that file, so a number cannot quietly drift away from the code.*
