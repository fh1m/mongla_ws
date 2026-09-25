# PR U — two mission stores, both fully working, and nothing reads either

**Target:** `srot-control-board` · **File as an ISSUE**
**Status:** ⏳ NOT SENT — read-only GitHub access this session (403 on issue creation).

The board implements **two** complete mission-upload paths. Both parse, validate
and store. Neither is read by anything. And the LoRa one **ACKs every chunk**, so
from Bondor's side the upload does not merely appear to work — it is positively
confirmed, waypoint by waypoint.

Found by walking qualified call sites for every namespace in `src/`, not by grep
over names. Source-read; not bench-verified.

---

## 1. `comms/mission` — the MAVLink `MISSION_*` protocol

`mission.h` exports three things. `handle()` has one caller (`mav_commands`). The
other two have **none, anywhere**:

```
$ # qualified uses of mission:: outside comms/mission.cpp
      1 mission::handle
```

`count()` and `getItem()` are the only way to read what was stored. Nothing calls
them, so nothing can fly a stored mission.

What *does* work is the whole handshake: `MISSION_COUNT` → `MISSION_REQUEST_INT` →
`MISSION_ITEM_INT` → `MISSION_ACK`, and the download direction too (`sendCount`,
`sendItem`). So QGC can upload a mission, read it back item-for-item, and draw it
on the map. Every signal says success.

⛔ **And this actively misleads, rather than merely doing nothing.** Implementing
`MISSION_*` is how a vehicle announces "I am a waypoint-following autopilot". This
one is not: `AUTO` runs the `SROT_MOVE` primitive machine, which has no concept of
a waypoint. An operator who uploads a mission and selects AUTO gets timed
primitives, not their waypoints, with nothing anywhere reporting the mismatch.

## 2. `drivers/lora_mission` — the SX127x waypoint receiver

Same shape, one step worse:

```
$ # qualified uses of lora_mission:: outside the driver
      3 lora_mission::poll          <- wired
      1 lora_mission::setUplinkHandler
      1 lora_mission::sendTelemetry
      1 lora_mission::sendRaw
      1 lora_mission::healthy
      1 lora_mission::begin
```

`missionReady()`, `waypointCount()` and `getWaypoint()` — **zero callers.**

`poll()` is wired and does the work: it classifies the packet, writes
`s_wp[seq]`, sets `s_have[seq]`, computes `s_ready` when the set is complete, and
calls `sendAck(seq)` — which transmits `0xA5, seq` back to the ground station.

⛔ **So the board explicitly acknowledges each waypoint it will never use.** That
is the strongest possible false-success signal: not silence, a per-chunk ACK. Our
own doctrine calls a verb that reports success while the vehicle does nothing the
failure mode that ends competition runs; this is that, at the protocol layer.

## 3. Why neither can simply be wired

**The vehicle has no position estimate.** No GPS, no DVL. A waypoint in metres
from an unknown origin is not actionable, so `getWaypoint()` has no honest
consumer today even if someone called it. This is not a missing five lines.

That is exactly why we are filing it as a question rather than a patch.

## 4. What we suggest, and we would take any of the three

1. ⭐ **Say so on the wire.** Cheapest and it removes the lie: refuse the upload.
   `MISSION_COUNT` → `sendAck(MAV_MISSION_UNSUPPORTED)`, which is already written
   and already used for the wrong `mission_type`, and drop the LoRa mission-chunk
   branch (or NAK it) so Bondor stops being told yes. An operator then learns in
   one second instead of after a dive.
2. **Delete both.** ~280 lines that look current and will be copied. Our own
   register has a row for this exact hazard (`draw_strip.py`, 471 lines of dead
   UI). The MAVLink download path is genuinely useful for nothing on this
   architecture.
3. **Keep them and name the consumer**, in the file, in one sentence: what will
   read `getWaypoint()`, and what supplies the position estimate it needs. That
   is the standard we hold ourselves to (`test_no_capability_is_built_and_
   unreachable.py`), and it is the least work of the three.

⚠ We are not asking for a waypoint follower. On a hull with no position estimate
that would be a worse thing to have than nothing.

## 5. ⭐ The general ask, which is worth more than either of the above

**The firmware has no unreachable-capability guard at all.** We checked: nothing
in the repo tests that an exported symbol has a caller.

We found these two by sweeping every `namespace::function` declared in
`src/control`, `src/drivers` and `src/comms` and counting qualified uses outside
the defining module. It took one shell loop and it also surfaced:

| orphan | consequence |
|---|---|
| `thrust_trim::learned()` | the learned per-thruster gains are **unobservable** — see [PR R](pr-r-thrust-trim-predicts-the-wrong-duty.md), where those gains slam to their clamp and nothing can see it |
| `pca9685_aux::healthy()` | a dead payload expander is undetectable, on the board whose payload roles already cost us every drop and torpedo (our B53) |
| `bar30::jittery()` | a third copy of "is the baro healthy", beside `healthReason()` and `jitterP2P()` which are both used |
| `bno085::healthy()` | second copy beside `attitudeValid()`, which is the one used |
| `mission::count/getItem`, `lora_mission::missionReady/waypointCount/getWaypoint` | §1 and §2 |

⛔ **`thrust_trim::learned()` is the one to look at first**, because it is not just
dead — it is the *observability* for a live defect. PR R shows the trim learning
against a prediction that ignores the mixer's own shaping, driving every gain to
its −25 % clamp. There is no way to see that happen, because the only accessor
that would show it was never called. **The bug and its invisibility have the same
root.**

This is the defect class that has cost us more than any other — ten orphaned
modules in a single session once, five written that day, every one with correct
code and passing tests. Our guard is
`test_no_capability_is_built_and_unreachable.py`; this pass found it covered only
1 of our 7 packages and extended it, which is how the four host orphans above
turned up. **A dozen lines of shell in CI would give you the same thing.**

---

*Read at `comms/mission.{h,cpp}`, `drivers/lora_mission.{h,cpp}:113-170`,
`control/thrust_trim.h`, `drivers/pca9685_aux.h`, `drivers/bar30.h`,
`drivers/bno085.h`. Call counts are qualified `ns::fn` uses outside each defining
module.*
