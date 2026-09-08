# Mongla / duburi_ws — Unified Bug Register

> **This file is the SINGLE tracker for code defects in `duburi_ws`.**
> It replaces the bug content previously spread across `known-issues.md`,
> `development-board.md`, `robosub-2026-audit.md`, `water-owed.md` and
> `srot-pre-dive-gates.md`. See §6 for exactly what was migrated, what was
> deliberately left in place, and why.
>
> Produced by the system-wide audit of 2026-09-08, post the `srot`→`main` merge
> (`6db956a`). Scope: controls, vision, planner, sensors, managers, plus the
> three sibling repos.
>
> **STATUS: 35 of 42 fixed (2026-09-08).**
> B01, B02, B03, B05, B09, B10, B21 (the first SROT-path batch) · B16, B22, B23,
> B27 (vision/tooling) · B18, B30 (the srot vision axes) · B25, B26, B29 — found
> while fixing the others. Each landed with a test **verified to fail without the
> fix**.
>
> **No open item is reachable on the live srot flight path.** That claim is
> checked mechanically by `tools/srot_reachability.py`, which walks CALLS from the
> real srot entry verbs — **not** by grep and not by which modules are imported.
> Both of those gave wrong answers in the same pass (B12 false positive, B18 false
> negative), and chasing the contradiction is what surfaced **B30**, a live
> default-on `AttributeError` in the vision arrival brake. Re-run that tool before
> ever writing "not on this path" in here.
>
> The remaining open items are ArduSub-path (B06, B07, B08, B12, B13, B15, B17,
> J01 — preserved on the `pixhawk` branch), DVL (B04, B11, B14, B19, B20 — not
> fitted), FSM (J02, J03 — never run), or docs-only (B24, whose doc half is done
> and whose code half is B06).
>
> ⚠ **B28 is the one srot-path item still open, and only half of it is ours.** The
> host guard is fixed and tested; the firmware fix is upstream PR
> srot-control-board#15, source-verified but **not bench-demonstrated** — the board
> refuses to arm without the thruster pack.
>
> ⚠ **J02 is inert only because J03 is true.** `SurfaceState` swallows the failure
> of `set_depth(0.0)` and returns `SUCCEED` unconditionally — a failed ascent
> reported as success. Fix it **before** the FSM is ever switched on for a run.

## How to read this

Severity is about *consequence on the vehicle*, not about how hard the fix is.

| Grade | Meaning |
|---|---|
| **CRITICAL** | Can hurt the hull or hide a condition that can. Fix before water. |
| **HIGH** | Wrong control output, or a wrong number reported as right. |
| **MEDIUM** | Correct today, wrong under a reachable condition. |
| **LOW** | Real but bounded; style/robustness debt with a named trigger. |

`REPRODUCED` means a runnable check was written and executed in this pass, and
the output is quoted. Everything else is read from source and cited by line.

**The single most useful sentence in this document:** in the three worst
findings the *analysis was already correct and the wiring was not* — a leak
reporter that is never registered (B01), a calibration flag no caller consults
(B04), and a guard disarmed by its own caller (B05). Reviewing for "is this
logic right?" would have passed all three.

---

## 1. CRITICAL

### B01 — the LEAK health reporter is never registered  ✅ FIXED 2026-09-08 (`6fada73`)
**`duburi_manager/health_reporters.py:72` · `auv_manager_node.py:1191`**

Nine reporters are defined; six are registered. `leak_sensor`, `target_lock`
and `target_pose` are not.

`leak_sensor` is not a stub — it is the most carefully reasoned function in the
file. Its docstring records a measurement (`LEAK_EN = 0` on this board) and the
trap that follows from it: *"With the failsafe disabled the sensor still answers
'no leak', so the vehicle looks safe precisely when nothing is watching."* It
returns `failed('leak', 'LEAK_EN=0 -- NOTHING IS WATCHING …')` — written to fail
for the exact configuration the board was measured to be in.

It has never executed. The health ladder aggregates six reporters, and `OK` from
six reads identically to `OK` from nine.

**Fix shape:** register the three missing reporters. Then decide separately
whether `LEAK_EN=0` should block arming — that is a policy call, not a wiring one.

### B02 — 12 bytes of noise hang the DVL reader thread forever `REPRODUCED`  ✅ FIXED 2026-09-08 (`0c1099a`)
**`duburi_sensors/sources/nucleus_parser.py` · `PacketAccumulator.feed`**

`size_header` and `size_data` are taken off the wire with no lower bound. When
both are zero, `total = 0`, so `len(buf) < total` is False (not treated as a
partial packet), `del self._buf[:total]` deletes **nothing**, `parse_packet` on
the empty slice returns `None`, and `while True` returns to the top with the
buffer byte-identical. It never terminates and never yields.

```
acc.feed(bytes([0xa5,0x00,0xb4,0x20,0x00,0x00,0x00,0x00,0x00,0x00,0x11,0x22]))
→ timeout 10 killed it; exit 124
```

Trigger is a `0xa5` followed by a zero at offset 1 and a zero `uint16` at
offsets 4–5. It does not crash and does not log — the thread simply stops
delivering, so this presents as "the DVL is unplugged".

**Status:** dormant, because the team has lost the DVL hardware. The code was
flown on Duburi 4.5 and is tested, and the standing instruction is to maintain
it — so this stays on the fix list at full severity. An absent sensor does not
make a framing bug less real; it postpones it.

**Dormancy is narrower than it looks.** `bringup.launch.py` defaults
`dvl_auto_connect:=true`, so the reader thread and its TCP retry loop already
run on **every pool launch**. Nothing fires only because nothing answers on
`192.168.2.201:9000` — which is a *network* condition, not a hardware one.
Anything else on that address, or a stale BlueOS extension, feeds bytes
straight into `feed()`.

**Fix shape:** require `total >= 10`; anything smaller is a false sync — drop
one byte and rescan, exactly as the `_MAX_PACKET` branch three lines above
already does.

---

## 2. HIGH

### B03 — a false sync byte silently eats the next real packet `REPRODUCED`  ✅ FIXED 2026-09-08 (`0c1099a`)
**`nucleus_parser.py` · same function as B02**

On a checksum failure the accumulator has *already* consumed `total` bytes
before `parse_packet` gets to reject them. A spurious `0xa5` with a plausible
length field therefore discards up to `_MAX_PACKET` (2048) bytes — including
real packets that began inside that span.

```
fed: [false sync, len=0x0040] + [valid AHRS] + [valid AHRS]
→ 2 real packets fed, 1 recovered
```

No log, no counter. The lost packet is indistinguishable from one the DVL never
sent. The correct rule — drop one byte, rescan — is already present in the same
function and simply is not applied to the checksum-failure path, because the
length used to advance the stream came from the bytes that just failed
validation.

> B02 and B03 are one mistake seen from two sides: **the length field of an
> unvalidated packet is used to advance the stream.** Trusting it forward eats
> packets; trusting a zero-length version of it never terminates.

### B04 — an uncalibrated BNO reports healthy and steers as if Earth-referenced  ✅ FIXED 2026-09-08
**`duburi_sensors/sources/bno085.py:181, 243, 284`**

Calibration locks `offset = (pixhawk_yaw − bno_raw) % 360`. On failure the code
deliberately continues and leaves `offset_deg = None`, with the comment *"so
callers can detect the uncalibrated state."* **No caller implements that.**

```python
read_yaw():    if self._offset_deg is None: return raw   # "raw mode (diag only)"
is_healthy():  return self.read_yaw() is not None        # → True
```

So a failed calibration yields HEALTHY plus boot-relative yaw — a heading whose
zero is wherever the hull happened to point at power-on — consumed by
`HeadingLock`, `motion_yaw._YawPID` and `heading_error` as absolute. `turn(90)`
then goes to 90° in a frame nobody knows. The error is a fixed unknown bias in
`[0,360)`: it does not look like noise and does not decay.

The guarded failure is the *expected* one, named in its own comment ("Pixhawk
AHRS slow to warm") — a cold-boot race, i.e. pool day.

The only consumer of the flag is cosmetic: `auv_manager_node.py:607` uses it to
decorate the startup banner, and `if offset is not None:` means a **failed**
calibration renders as the banner line being *absent*. Absence displayed as
nothing — the inversion of this project's own "Absence is the signal" rule.
Worse, the branch is `if self._yaw_src_name == 'bno085'`, so on the
`bno085_dvl` composite the offset is never printed at all.

**Verified correct in the same file** (recorded so it is not re-audited): the
single negation to compass convention `(-yaw) % 360` at line 388, applied
exactly once at ingestion; the modular arithmetic across wrap in both
directions; `_fresh_raw_yaw` returning `None` past `_STALE_S`.

### B05 — a caller coerces `None` to `0.0` and thereby defeats the guard written to catch it  ✅ FIXED 2026-09-08 (`d2df03d`)
**`duburi_vision/distance/distance_estimation_node.py:208` · `flow_math.py:855`**

The accumulator refuses a degenerate frame on purpose:

```python
def add(self, flow_trans_px, height_m, f_px):
    """... No-op when not active or on a bad height / f_px."""
    if not self.active or height_m is None or f_px <= 1e-6:
        return
```

The caller hands it a value that can never be `None`:

```python
self._acc.add(self._flow_ema, self._last_height or 0.0, self._f_px)
```

`_last_height` is `float | None`, initialised `None` and set only once a valid
depth arrives. `or 0.0` turns the `None` into `0.0`, the `is None` test is
False, and execution reaches `distance_m += project(...) * 0.0 / f_px`.

The two behaviours are not equivalent, and that is the defect:

- **intended:** skip the frame — the accumulator knows nothing happened.
- **actual:** accumulate a zero — the accumulator believes it measured no motion.

Real motion in that window is recorded as *"the vehicle did not move"* rather
than *"we could not tell"*, so integrated distance under-reports silently, with
no count of degenerate frames — while the sibling `HeightEstimator.add`
immediately above takes the correct approach (`self._n_gated += 1; return None`).

### B06 — a mid-command heading-lock timeout leaves Ch4 with no author  ⏸ **DEFERRED — ArduSub path only; not worked by operator decision (2026-09-08)**
**`duburi_control/duburi.py:1362 _writers()` · `heading_lock` timeout path**

`_writers()` samples `_lock_active()` **once**, at command dispatch, and the
resulting `Writers` bundle is used for the whole command — a 20 Hz thrust loop
can run for many seconds. With a lock active the bundle is the
`release_yaw=True` flavour, whose stated contract is *"Ch4 stays released
(65535) so the HeadingLock thread is the sole author of Ch4."*

If the lock hits its own timeout mid-command, `_on_lock_timeout` sets
`self._heading_lock = None` and the lock thread stops writing Ch4 — but the
running loop keeps the stale writers and keeps asserting `Ch4 = 65535`. From
that instant until the command ends, **nothing writes Ch4**: the sole author was
retired while the loop is still explicitly releasing the slot to it. Ch5/Ch6
stay overridden so `FS_PILOT_INPUT` is still fed and no failsafe fires.

This is the module's own standard, not an outside opinion — `_on_lock_timeout`'s
docstring already names the consequence ("would … make translation verbs release
Ch4 to a thread that no longer exists") and then fixes it only for *subsequent*
commands, not the one in flight.

**⛔ SETTLED 2026-09-08, and it is WORSE than the above — see B24.** Ch4 = 65535
does not release the channel on Ch1–8; it means "ignore this field", so ArduSub
**retains the dead lock's last yaw-rate command** and does not refresh its
timestamp. The override stays live for `RC_OVERRIDE_TIME` (default **3.0 s**).
So it is not "nothing writes Ch4" — the hull keeps yawing at a dead controller's
last command for up to three seconds. The fix must actively write `1500`;
"releasing" it is exactly what does not work. Still deferred: ArduSub path.

### B07 — `arc()` steers on a fabricated heading reference  ⏸ **DEFERRED — ArduSub path only; not worked by operator decision (2026-09-08)**
**`duburi_control/motion_forward.py:129`**

```python
start_heading = read_heading(pixhawk, yaw_source) or 0.0
```

Seeds `last_heading`, which never updates while the source stays `None`, and
feeds `error = heading_error(target_yaw, last_heading)` → `_YawPID` → Ch4. A
stale yaw source therefore makes `arc` steer against a fabricated due-north
reference. `motion_yaw._lock_to_target` uses the same PID but **does** guard
with `STALE_HOLD_S`; `arc` has no stale guard. Same `or 0.0` idiom appears in
`motion_writers.py` (B12), but there it only reaches a log line.

### B08 — `prime_alt_hold` seeds the ramp from ungated telemetry, inverting its own purpose  ⏸ **DEFERRED — ArduSub path only; not worked by operator decision (2026-09-08)**
**`duburi_control/motion_depth.py`**

```python
starting = pixhawk.get_attitude()          # UNGATED
prime_d  = starting['depth'] if starting is not None else target_m
```

The `_fresh_depth()` freshness gate is applied to the arrival check but **not**
to the ramp seed. With no telemetry the fallback primes at `target_m` — which
inverts the phase whose entire purpose is to start at zero error. The file's own
comment: *"without this the first frame can fire the vertical thrusters at full
power."*

---

## 3. MEDIUM

### B09 — Kalman `Q` is not rescaled with `dt` while `F` is  ✅ FIXED 2026-09-08
**`duburi_vision/tracking/kalman.py`**

`_update_F(dt)` correctly rewrites `F[0,2] = F[1,3] = dt` for a variable frame
interval, but `Q = np.eye(4) * process_noise` is built once and never touched.
For a constant-velocity model the discrete process noise scales with `dt` (the
white-noise-acceleration form is `[[dt⁴/4, dt³/2], [dt³/2, dt²]] · σ_a²` per
axis). A fixed `Q` is correct at exactly one `dt`.

Measured detection intervals on this stack: **p50 32.5 ms, p95 48.4 ms, gaps to
2.4 s.** Across p50→p95 the position term alone moves by `(48.4/32.5)⁴ ≈ 4.9×`;
across a 2.4 s gap, ~10⁵. The filter is therefore over-confident exactly when a
detection is missing — the case the smoother exists to handle — so the predicted
box is trusted more than it has earned. `_update_F` shows the author knew `dt`
was variable; `Q` was not carried through the same reasoning.

### B10 — `_last_height` is an unbounded latch with no freshness  ✅ FIXED 2026-09-08 (`d2df03d`)
**`distance_estimation_node.py:207`**

Once set, `_last_height` is never invalidated — no timestamp, no staleness
bound. If the depth source stops, every later frame keeps scaling optical flow
by the last known height. Metres-per-pixel is directly proportional to height,
so a hull that has since changed altitude integrates distance at the wrong
scale, with a plausible number and no warning. `bno085._fresh_raw_yaw` in this
same codebase already has the right pattern.

Operator-facing companion: `f'height={height or 0.0:.2f}m'` prints an unknown
height as `height=0.00m`, and the debug topic does the same. Three `or 0.0` in
one file, each turning "unknown" into a number a human reads as measured.

### B11 — `CompositeBnoDvlSource.is_healthy` contradicts its own docstring  ✅ FIXED 2026-09-08
**`duburi_sensors/sources/composite_bno_dvl.py:17 vs :59`**

Docstring: `is_healthy() -> BNO healthy AND DVL streaming`.
Implementation: `return self._bno.is_healthy()`.

The implementation is arguably the *right* one — a `YawSource`'s health should
describe the yaw channel — so the likely defect is the stale docstring. It still
matters: `sensors_node.py:119` is the single caller and reports that as the
source's health for the composite pairing, so a reader who trusts the docstring
believes a green line covers the DVL. Separately, **nothing in the tree calls
`dvl_is_healthy()` at all** — DVL streaming health is computed and never
consulted.

### B12 — heading fabricated as due-north in the writers/log path  ⏸ **DEFERRED — ArduSub path only; not worked by operator decision (2026-09-08)**
**`duburi_control/motion_writers.py`**

`locked_heading = read_heading(pixhawk, yaw_source) or 0.0` — same idiom as B07
but reaching only a log line, so a stale source prints a confident `0.0°`.
Recorded separately from B07 because the fix is the same and the severity is
not. The same file also polls `get_attitude()` every 20 Hz tick purely to feed a
throttled log line.

### B13 — `motion_yaw` derivative term has no `dt`  ✅ FIXED 2026-09-08
**`duburi_control/motion_yaw.py`**

```python
d_term = YAW_KD * (error_deg - self._last_e)   # comment claims "Uses 1/YAW_RATE_HZ as dt"
```

`dt` appears nowhere in the expression. The gain is therefore in units of
per-tick rather than per-second, silently coupling `YAW_KD` to the loop rate: a
rate change retunes the controller without touching a gain. Related in the same
file: `_last_e = 0.0` gives a first-tick derivative kick (masked by the clamp,
not prevented); the timeout path reports the **entry** heading as current;
`frames_locked` is not reset across a stale gap; and `yaw_glide`'s total time is
`duration + timeout`, up to 6 s + timeout.

### B14 — the parser's tests cover the one garbage case that cannot fail  ✅ FIXED 2026-09-08
**`duburi_sensors/test/test_nucleus_parser.py`**

`test_accumulator_skips_leading_garbage` feeds `b'\x00\x11\x22'` before a good
packet. That garbage contains **no `0xa5`**, so it exercises only the
`find(0xa5)` skip — the easy path. Both real defects (B02, B03) require a `0xa5`
*inside* the garbage, which is the only case where the length field gets
trusted. Neither is covered.

This is why the parser looked well-tested at 30+ findings in: three accumulator
tests, all green, none touching the resync rule they exist to protect. The fix
for B02/B03 is worthless without a test that feeds a false sync byte — the two
exact inputs reproduced above.

### B15 — `LOCK_HOLD_DEADBAND_DEG < LOCK_APPROACH_BAND_DEG` is documented, unenforced  ✅ FIXED 2026-09-08
**`duburi_control/heading_lock.py`**

The constant is annotated *"Must stay < `LOCK_APPROACH_BAND_DEG`"*. Nothing
asserts it. Violated, the lock degrades silently into the relay it exists to
prevent — the exact limit-cycle that the tapering fix (`ab2014f`) was written to
remove.

### B16 — `bearing.py` promises defensive scaling it does not implement  ✅ RESOLVED 2026-09-08 — **as a DOC defect; the finding as written is partly RETRACTED**
**`duburi_vision/bearing.py`**

The comment promises to *"scale defensively rather than silently producing
bearings that are wrong by the resolution ratio"*. The code never scales and
takes no calibration-resolution parameter.

**The first half stands; the conclusion does NOT, and is retracted.** Reading the
callsites shows the defence exists — one layer up, where the information actually
is. `camera_node._fill_calibration` owns the calibration file, therefore knows
both resolutions, and rescales `fx/cx/fy/cy` before publishing `CameraInfo`
(*"Publishing the unscaled matrix would put the principal point off the image and
every derived angle would be wrong by 2x, silently."*). And `VisionState._on_info`
latches `k` and `width`/`height` from the **same** message, so the `K` a bearing is
computed from always matches the size it is handed. A mismatch is unreachable
through the only caller.

So the defect is a **comment misattributing an implemented defence to the wrong
file**, not a missing guard. Adding `calib_size` arguments would have taken an
8-argument function to 10 in order to guard a caller that does not exist, and
created a second place to apply the ratio — i.e. a second way to get it wrong.
Fixed by correcting the comment to name where the defence lives, and pinning both
halves plus the end-to-end invariant (a matched rescale must not change the
bearing) in `test_bearing_resolution_invariant.py`.

### B21 — a preflight check that can silently vanish from the report  ✅ FIXED 2026-09-08
**`duburi_manager/bringup_check.py:363`**

```python
try:
    mode = conn.flightmode
    out.append((PASS, 'flight mode', str(mode)))
except Exception:
    pass
```

`bringup_check` is the hard pre-mission gate — "each line is PASS/WARN/FAIL;
exit 0 unless a FAIL", with `--strict` making any WARN exit non-zero. If reading
`conn.flightmode` raises, **no row is appended at all**: the operator gets a
report with the flight-mode line simply absent, and the exit code is unchanged,
so the gate passes.

In a checklist a check that can disappear is worse than one that fails — a FAIL
is read, an absent row is not. The neighbouring checks in the same function do
it right: the armed check emits on both branches, and the battery read treats
`0/65535` as unknown rather than as a voltage.

Same family as B01 and B04: the analysis is right, the reporting of its absence
is not.

### B22 — the refractive index is hardcoded six times, twice inside the file that defines the constant  ✅ FIXED 2026-09-08
**`calibration/solver.py:77, 246, 248` · `distance/flow_math.py:115` · `distance/flow_node.py:209` · `tools/fov_calibrate.py:114`**

`solver.py:77` declares the single source of truth —
`N_WATER = 1.333  # sea/fresh water; a flat port refracts by Snell's law` — and
`fov_from_K.refract()` uses it. But `fov_for_medium`, the **inverse** transform
in the same file, hardcodes the literal instead:

```python
v['hfov_air'] = 2*degrees(asin(min(1.0, 1.333 * sin(radians(hw/2)))))
```

So the forward (air→water) and inverse (water→air) conversions read the index
from two different places. Change `N_WATER` — to 1.34 for salt water — and the
two stop being inverses. The result is a plausible FOV that is
self-inconsistent, in a file whose own docstring warns that applying the
refraction the wrong way is *"a ~1.33× error, in the direction that still looks
like a plausible camera."*

This is more than a DRY complaint because the quantity is **physical** and the
two uses are **inverses**: the failure mode is not "one is stale" but "the round
trip no longer closes."

**FIX.** `duburi_vision/optics.py` now holds `N_WATER` and both Snell transforms
as a pair; all six sites import them (`flow_math` could not import `solver` —
that pulls `cv2` into a deliberately dependency-light module, which is part of
how the literal got copied there). The transforms read `N_WATER` at **call** time,
not as a default argument, because a default is bound once at import and would
freeze the index — making the documented "change it for salt water" a silent
no-op, and the defect untestable.

⚠ **The obvious test does not work, and this is the point.** "air → water → air
round-trips" **passes on the broken code**, because the hardcoded literal equals
`N_WATER` today. Verified by re-injecting the bug: the 8 round-trip assertions at
the shipped index all still passed. Only a round trip at a *different* index
(1.34, salt) and an AST scan for numeric `1.333` caught it. `test_optics.py`.

---

## 4. LOW

### B17 — `HeadingLock.stop()` joins an unstarted thread  ✅ FIXED 2026-09-08 — **REPRODUCED**
**`heading_lock.py`** — `self._thread.join()` with no guard, while
`Heartbeat.stop()` one module over carries an explicit `is_alive()` check *with
a comment naming the crash it prevents* (`"cannot join thread before it is
started"`, raised on every srot shutdown). Same codebase, same hazard, one
guarded and one not.

### B18 — `percent_to_pwm` truncates asymmetrically about neutral  ✅ FIXED 2026-09-08
**`duburi_control/pixhawk.py`** — `int()` truncates toward zero, so `+0.1% →
1500` and `−0.1% → 1499`. Endpoints and clamping are otherwise correct
(verified). One LSB of dead-band bias on the negative side of every axis — below
thruster resolution today, but a rounding rule that differs by sign.
`round()` makes it symmetric.

⚠ **REACHABILITY CORRECTED 2026-09-08 — this is NOT ArduSub-only.** A sanity pass
classified B18 as pixhawk-path on the strength of a grep of `percent_to_pwm`
callers. That grep was piped through `head` and truncated at 10 lines, hiding all
**10** call sites in `motion_vision.py`. The srot vision path reaches it:

    VisionVerbs.vision_align -> motion_vision.align_loop -> Pixhawk.percent_to_pwm

so the one-LSB negative-side bias applies on the vehicle's live vision axes, not
only on the preserved ArduSub path. Severity is unchanged (still below thruster
resolution — and see B30 for what the *same* truncated grep was hiding, which was
not benign). Classify with `tools/srot_reachability.py`, never by grep.

**FIXED** once it was known to be on a live axis: `int()` -> `round()`, which is
symmetric about 1500 (banker's rounding about an even number: 1499.5 and 1500.5
both give 1500). Endpoints and the clamp are pinned unchanged by test.

### B19 — `CompositeBnoDvlSource.close()` leaks the second source  ✅ FIXED 2026-09-08 — **REPRODUCED**
**`composite_bno_dvl.py`** — `self._bno.close()` then `self._dvl.close()`, unsequenced.
An exception from the first (a serial handle already gone; the `ENOTTY`-on-PTY
case CLAUDE.md records) skips the DVL close entirely, leaving the TCP socket and
its reader thread alive. Shutdown is exactly where a first close is most likely
to raise.

### B20 — parser guards the exception that cannot happen  ✅ FIXED 2026-09-08 — **REPRODUCED**
**`nucleus_parser.py`** — both payload decoders sit inside
`except (StructError, IndexError)`, but Python slicing never raises
`IndexError`: `raw[96:100]` on a short buffer returns a short slice and the
failure surfaces from `unpack` as `StructError`. The truncated-payload case is
caught, but by accident, and returns a bare `None` indistinguishable from "not a
packet we decode". No length precondition is asserted for the documented field
offsets. Separately, `_checksum(buf[:size_header - 2])` uses an unvalidated
`size_header`; below 2 the slice silently becomes a negative index rather than
an error.

### B23 — a QBUF failure escapes the v4l2 pump thread  ✅ FIXED 2026-09-08 — **and there were TWO sites, not one**
**`cameras/v4l2_mailbox.py:425`** — in the frame-skip inner loop the
`try/except OSError` covers `select` and the `DQBUF`, but the re-queue of the
*older* buffer sits outside it. An `OSError` there propagates out of
`_pump_loop`, a bare `Thread` target, so the thread dies and the buffer leaks.

**Mitigated, hence LOW:** `is_healthy()` gates on
`(time.monotonic() - self._last_ok) < 2.0` as well as `_consec_fail < 30`, so a
dead pump goes unhealthy within 2 s regardless of *why* it died — the freshness
clock does the work an exception handler would. Recorded because the mitigation
is incidental and nothing logs the cause: the symptom is "camera went unhealthy"
with no reason in any log.

**FIXED — and the entry UNDERSTATED it.** Writing the test for the skip-loop site
turned up a **second** unguarded `VIDIOC_QBUF`, the main-path re-queue after the
payload copy. That one runs on **every frame**, not just when skipping, so it is
the more exposed of the two. Both are now inside `except OSError`, both count
`_consec_fail` (so the error cannot be caught and then hidden — worse than the
crash it replaced) and both log the cause once. The payload is copied out of the
mmap before the re-queue, so a failure costs one buffer slot, not the frame.

The test asserts the invariant for the **whole function** — *every* `ioctl` in
`_pump_loop` is lexically inside a handler catching `OSError` — rather than the
one line, because the next ioctl added there has exactly the same hazard. That is
what found the second site.

---

## 5. Design findings — JSF-AV lens

Kept separate from the defect list above on purpose: these are gradings against
the JSF-AV table in `CLAUDE.md` §7, not bugs. The codebase scores well overall —
`Writers`/`make_writers` is a genuinely elegant solution to RC-slot arbitration,
and the MAVLink TX path is fully serialised (verified by AST: `srot_fc.py` 7
sends, `pixhawk.py` 13 sends, **0 unguarded** — every one inside
`with self._tx_lock`).

### J01 — two suspension primitives, different semantics
`Heartbeat.pause/resume` is **reentrant and counter-based** (`_hold_count` under
`_hold_lock`, with an underflow guard and a "reentrant via a counter" banner).
`HeadingLock.suspend/resume` is a **boolean `threading.Event`** — nesting is
lossy, an inner resume cancels an outer suspend.

Latent, not live: every `with self._suspend_heading_lock()` sits inside a verb
body and `Duburi.lock` serialises verbs, so the block cannot nest (verified — no
self-call of `pause()`/`stop()` in `duburi.py`). Recorded because the safety of
the boolean rests on an invariant held in a *different file*, while the sibling
primitive one module over already pays for the counter. The next verb that
composes two suspending verbs reopens it with no error and no log. Violates
*"clear interfaces"*.

### J02 — `SurfaceState` swallows the failure of the one call it exists to make  ✅ FIXED 2026-09-08
`duburi_planner/state_machines/states/navigation.py`

```python
self.duburi.stop()
try:
    self.duburi.set_depth(0.0, timeout=60)
except Exception:
    pass                       # ← the ascent failed, silently
self.duburi.disarm()
return SUCCEED                 # ← unconditional
```

`SurfaceState` is the ABORT destination wired on every state in every plan — the
emergency-surface path. A `MovementTimeout` means the hull did **not** reach the
surface; the FSM records success and disarms, leaving a submerged vehicle
without thrust. What makes it a defect rather than a style choice:
`DuburiState.execute` already converts any exception into `stop()` + `ABORT`, so
this inner `except: pass` exists only to defeat that safety net, on the single
call the state exists to perform. Violates *"fail-safe defaults"*.

**Deprioritised per operator** — this FSM tree has never executed (see J03).

### J03 — 2,672 lines of FSM that has never run
`state_machines/` is 20 files / 2,501 lines plus 5 `missions/fsm_*.py` launchers
/ 171 lines. The operator confirms this code has **never executed**. CLAUDE.md
presents the YASMIN FSM as BUILT and lists `fsm_full_2026` as *"(recommended)"*
for a competition run.

Two real costs: it is audited, maintained and imported as if live (J02 sits
inside it); and a doc recommending an unexercised path for a competition run is
the highest-consequence kind of stale claim.

**Decision needed, not a fix to guess at:** exercise it and keep it, or delete
the tree and stop calling it built.

---

## 6. Consolidation record — what was deleted, migrated, and rewritten

**Executed 2026-09-08.** Inbound references were enumerated first, because
`known-issues.md` was cited by source code, not just docs.

| File | Disposition |
|---|---|
| `known-issues.md` (600 lines) | **DELETED.** Bug entries → §1–§4 here. FIXED entries whose IDs are cited from source (D7–D10, D14, D16, P1, P2) → §9 appendix. Environment traps E1–E5 + the fork evaluation → new [`jetson-and-env-traps.md`](jetson-and-env-traps.md). |
| `development-board.md` | **KEPT**, bug section replaced by a pointer here. Its phase status, P0.1 commitment and doc map are project state, not a bug list. |
| `robosub-2026-audit.md` | **KEPT**, bug entries pointed here. Carries the P0.1 Decision Record and the G1–G12 gap matrix, which CLAUDE.md treats as the authority for committed 2026 scope. |
| `water-owed.md` | **DELETED.** Content → §11 appendix verbatim. It is a measurement backlog rather than a defect list, so it is kept whole and labelled as such. |
| `srot-pre-dive-gates.md` | **DELETED.** Content → §10 appendix verbatim, flagged **live safety interlock, not history** — GATE 0/1/2 gate every AUTO move, `move_forward` included. |

**32 files rewritten** to drop the dead link, including five source/config files
that cited it in comments: `bringup_check.py`, `vision_state.py`,
`vision_tunables.py`, `detector_node.py`, `test_detector_recovery.py`, plus
`requirements.txt`, `requirements-jetson.txt`, `tools/setup_pi_hailo.sh`,
`README.md`, `CLAUDE.md`, both Jetson docs, the `pool-day` skill, 17 context docs, `tools/srot_axis_snapshot.py`,
`tools/flow_derot_ab.py`, `flow_node.py` and `test_calibration_water.py`. `grep -rn known-issues` now returns only the historical mentions
in this file and the traps file. Zero dangling references.

The JSF-AV design findings were renumbered `D01–D03` → **`J01–J03`** so they do
not collide with the migrated historical `D7`–`D16`.

## 7. Coverage — what this audit read, and what it verified as correct

**Suite state at the end of this pass:** `python3 -m pytest src -q` →
**1825 passed, 3 xfailed, 0 failed** (261 s). Two of the three xfails are the
B02/B03 reproductions added by this audit (`test_nucleus_parser.py`), marked
`xfail(strict=True)` so they flip to a failure the moment the bugs are fixed and
the markers need removing. This was a find-only pass: the tests document the
defects, they do not fix them.

### Read in depth
`duburi_control` — all motion modules, the `duburi.py` facade, `heading_lock`,
`heartbeat`, `pixhawk`, `motion_vision` (92 KB), the `fc/` HAL and
`port_guard`. `duburi_sensors` — every source plus the binary parser.
`duburi_vision` — `flow_math`, `kalman`, `bearing`, `confidence`, the distance
nodes, `calibration/solver`, `cameras/v4l2_mailbox`, `stamps`, `vision_state`.
`duburi_manager` — health path and registration, banner, executor/callback-group
layout, `bringup_check`.

### Swept mechanically across the whole tree
- **114 silent exception swallows** enumerated by AST; all 20 in
  `duburi_control`/`duburi_manager` inspected individually. One defect (B21);
  the rest are `close()`/`flock`-unlock cleanup and are correct.
- **MAVLink TX serialisation** — `srot_fc.py` 7 sends, `pixhawk.py` 13 sends,
  **0 unguarded**; every one inside `with self._tx_lock`.
- **Concurrency** — 27 thread sites, 19 ROS timers, 43 lock primitives mapped.
  Manager uses `MultiThreadedExecutor` with `ReentrantCallbackGroup` for actions
  and `MutuallyExclusiveCallbackGroup` for timers. Findings: B06, J01.
- **`VisionState` lock discipline** — AST-checked every access to shared state.
  Six unlocked accesses, **all six in `__init__`**, before the object is shared
  with any thread. Clean.
- **Clock-domain mixing** — every file using both `time.time()` and
  `time.monotonic()` identified and the conversion points read.
- **Degenerate denominators** in `motion_vision` — all four guarded.

### Verified correct — recorded so this surface is not re-audited
`Pixhawk.heading_error` across all wrap cases · `percent_to_pwm` endpoints ·
`BearingFilter` alpha-beta `v += (β/dt)·resid` · `_undistort` Brown-Conrady
inversion · `flow_math.axis_unit` / `height_above_floor` /
`add_body_velocity` (derived projection `vx·cos(y−a) − vy·sin(y−a)` matches, and
reduces correctly to axial and lateral) · `pool_depth_m` defaulting to NaN with
a refusal · `medium` validated `air|water` · `ConfidenceModel.bounds()` guarding
`hi−lo < 1e-3` · `nsa_factor` direction (not inverted) ·
`DuburiState.__init__` outcome merge · BNO085 negation, modular arithmetic and
staleness · `align_loop`/`move_loop` `half_w`/`half_h` cannot be zero
(`_on_info` sets `_info_seen` only inside `if msg.width and msg.height:`, so a
zero-dimension `CameraInfo` yields `NO_CAMERA` rather than a `ZeroDivisionError`
— and `0.0/0.0` does raise in Python, so that guard is load-bearing) ·
`solver.py` Snell direction correct in both transforms with `asin` domain
clamped, and `fov_for_medium` correctly refusing to refract an in-water
calibration twice · `holdout_rms` per-point RMS and RMS-of-RMS pooling ·
`dfov = 2·arctan(hypot(w,h)/(fx+fy))` correct for square pixels.

### Two exemplars worth protecting
Recorded because a bug register that only lists faults misrepresents the
codebase, and because these are the patterns the rest of the tree should be
measured against.

**`duburi_vision/stamps.py`** — the freshness-conversion module. It names the
defect it exists to stop, lists the four times this codebase shipped it, and
states the principle exactly: *"a clock read at the wrong place does not fail,
it flatters."* It handles both clock domains explicitly, and even its residual
imprecision (a scheduler pause between the monotonic and wall reads) errs toward
reporting a frame as **older** than it is — the safe direction, which is the
direction the module argues for. This is the standard.

**`motion_writers.make_writers`** — RC-slot arbitration solved by construction
rather than by convention: the per-axis modules never learn about lock state,
they just receive a bundle whose `release_yaw` flavour already encodes it. B06
is a flaw in *when* that bundle is sampled, not in the design.

### Not covered — stated plainly
`auv_manager_node` beyond the health/banner/executor/clock paths (91 KB),
`srot_fc.py` (135 KB) and `srot_protocol.py` beyond the TX-lock and clock
sweeps, `duburi_dsl.py` / `vision_dsl.py`, `detector_node`, `camera_node`,
`tracker_node`, `lock_node`, `hailo`, `calibration/guide.py`, `display_node`,
`mission_web_node`, `srot_connect`, `connection_config`, most of `tools/`, and
the entire `sim/` workspace. The FSM tree was deliberately deprioritised per
operator instruction (see J03).

These are listed rather than glossed because an audit that implies uniform
coverage is worse than one that says where it stopped.

---

## 7b. Sibling repos — audited, and what came out

The four names in the audit brief map onto three repos:
**srot board + Hengla firmware = `srot-control-board`**, **Bondor GCS =
`srot-ground-station`** (the firmware there is a thin LoRa relay; Bondor itself
is the Electron/TS app under `bondor/`), **ESC Flasher = `srot-esc-flasher`**.

| Repo | Read | Outcome |
|---|---|---|
| `srot-control-board` | ~13.3k lines first-party C++ (`src/control`, `src/tasks`, `src/comms`, `include/config.h`) | **PR #14 filed** — `wrapPi` non-terminating for a large finite input, 4 sites, with the fix. One suspected mixer defect measured and **withdrawn**. |
| `srot-ground-station` | 5.5k lines TS/TSX under `bondor/` + 568 lines firmware | **PR #4 filed** — absent telemetry rendered as `0.00 m`; joystick disable sends no neutral; 2 minor. Findings-only, no code. |
| `srot-esc-flasher` | 2.1k lines C++ (`src/esp32_4way`), **re-read line by line 2026-09-08** | **PR #3 filed** — `GetESC`'s overload lets the buffer cap be confused with the timeout, and that confusion has already cost one truncation bug in this very file. Fixed with a by-reference template so the compiler supplies the cap. ⚠ The earlier "no findings" was a **shallower read**: it verified the bounds (correctly) and stopped there. |

**What the deeper ESC-flasher read found, and what it cleared.** The trap is that
`GetESC(RX_Buf, 250)` sits directly under `uint8_t RX_Buf[250]` and reads as
"cap = 250" when the `250` is the *timeout*; it is correct only because the
default cap is coincidentally also 250. `4Way.cpp`'s own comment records the
consequence when the two got out of step — *"the old 250-byte cap silently
truncated it and corrupted every full-page read."* Two call sites were also
capped **below** their buffers (`RX_Buf[300]`/`rx[300]` at 250); one of those was
truncating a raw dump.

Verified CORRECT and recorded so it is not re-audited: the bit-bang UART samples
at **absolute offsets from the start edge**, so the 52 vs 52.083 µs rounding
cannot accumulate (0.79 µs over 9.5 bits against ±26 µs of margin), and a write
is exactly 1 start + 8 data + 1 stop bit-times; `RX_Buf[RX_Size-1]` is safe by the
`RX_Size >= 9` short-circuit rather than by luck; `process_serial` bounds-checks
after each write and before the next, so the last write lands at `sizeof-1`; and
the two CRCs are correctly separated (`0xA001` reflected CRC-16 on the ESC link,
xmodem on the 4-Way host frame). Two latent items left unfiled because neither is
reachable today: `MSP_Check` returning `uint8_t` is the only thing keeping
`serial_comm.cpp`'s `uint8_t b` write-back loop finite, and the MSP branch does
not check frame completeness the way the 4-Way branch does.

**A correction worth recording.** An earlier pass of this audit reported "zero
findings touch the siblings". That was a statement about code that had not been
opened — the sibling repos were skipped and the claim rested on a previous
round's memory. Both PRs above came from actually reading them. The lesson is
the one this project keeps relearning in a new costume: *an absent measurement
is not a negative result.*

Two things were also **measured and withdrawn** rather than filed, recorded so
nobody re-derives them:
- `mixer.cpp` battery-voltage filter: a stale ESP-NOW link zeroes the 0.5 Hz
  filter state, so it re-seeds unfiltered on re-acquisition. Real mechanism,
  but measured at **≤5% commanded throttle at low demand and 0% at full** —
  `thstExpo(t*lift_max)/v` is a faithful port of ArduPilot's
  `apply_thrust_curve_and_volt_scaling` and the two voltage terms largely cancel.
- Bondor's `protocol.ts` wire constants: hand-verified **correct**
  (`FORWARD..ARC = 0..9`, `SROT_MOVE = 31000`, matching `srot_protocol.py`).
  Filed only as "not drift-tested", not as drift.

---

## 7c. Findings from the fix pass (2026-09-08)

Three defects surfaced while fixing the audit's own list. Recorded here rather
than folded silently into the fixes.

### B24 — `65535` on Ch1–8 does NOT release the channel  ⛔ two context docs were wrong
Settled from primary source (`stuff/ardupilot`, ArduPilot 4.7.0-beta3),
`libraries/GCS_MAVLink/GCS_Common.cpp:4213-4218`:

```c
for (uint8_t i=0; i<8; i++) {
    // Per MAVLink spec a value of UINT16_MAX means to ignore this field.
    if (override_data[i] != UINT16_MAX) { RC_Channels::set_override(i, override_data[i], tnow); }
}
```

For channels 1–8, `UINT16_MAX` means **"ignore this field"** — `set_override` is
not called at all, so the channel **keeps its previous override value** and its
`last_override_time` is **not refreshed**. `RC_Channel::has_override()` stays
true until `RC_OVERRIDE_TIME` elapses — default **3.0 s**
(`RC_Channels_VarInfo.h:90`). Releasing is a different value: `0` clears the
override; on Ch9+ `UINT16_MAX-1` means "return to RC".

`mavlink-reference.md` and `ardusub-canon.md` both claimed 65535 released the
channel to RC input. Both corrected.

**This makes B06 worse than recorded.** When the heading lock dies mid-command
the loop keeps asserting `Ch4 = 65535`, so ArduSub retains the dead thread's
last yaw-rate command and the hull keeps yawing at it for up to three seconds —
rather than "nothing writes Ch4", as B06 says. The fix must actively write
`1500`, not "release". B06's text is updated; the fix stays deferred (ArduSub
path). Caveat: verified against the 4.7.0-beta3 checkout; re-confirm against the
`Sub-4.5` branch we fly before acting on the pixhawk path.

### B25 — the `detector` health reporter could never report  ✅ FIXED (`6fada73`)
`detector` **was** registered, and `_detection_rate_hz` read
`getattr(self, 'vision', None)`. Nothing on `AUVManagerNode` ever assigns
`self.vision` — VisionStates live in the `_vision_states` pool, keyed by camera.
The defensive `getattr` returned a plausible `None` instead of raising, so the
reporter degraded silently into one that cannot report, and the D16 signal it
exists to carry (an aborted detector keeps every topic; only the *rate* changes)
never fired on a live vehicle.

### B27 — the ledger guard silently disables itself when a newer plan is written  ✅ FIXED 2026-09-08
`src/duburi_manager/test/test_ledger_claims.py` pins the carried-work ledger
against the tree. `_ledger()` reads **"the most recently modified plan file"**,
so the moment any new plan is written beside the workspace the guard switches to
that file, finds no `### Open engineering` section, and `pytest.skip`s — five
assertions at once.

Observed this session: the suite went from `3 xfailed, 0 skipped` to
`1 xfailed, 5 skipped` purely because a new plan file was created. Nothing
failed and nothing warned; the guard just stopped guarding.

This is B21's shape one level up — **a check that can vanish is worse than one
that fails.** The tests were written to catch a stale ledger, and a stale ledger
is exactly the condition under which someone writes a new plan.

**FIXED — and it was still skipping when this was picked up.** The newest plan
was the branch-topology one, which has no ledger section, so all five assertions
were inert.

`_ledger()` now selects by **content** — the plan file that actually contains the
marker — and only breaks ties among *those* by mtime, which is the original intent
restricted to files that can answer the question. When no plan carries a ledger it
**raises** instead of skipping, and the message says how to re-point it: a renamed
or reformatted ledger is a finding about the guarded thing, not a reason to go
quiet. The `_section()` skip is gone with it.

Demonstrated under the exact trigger (touching the non-ledger plan so it is
newest): mtime selection picks a file with no ledger, content selection finds the
real one. **6 passed, 0 skipped**, where it was 5 skipped.

### B26 — and its tests were green, because the stub invented the attribute  ✅ FIXED (`6fada73`)
`test_health._manager_stub` set `m.vision = SimpleNamespace(stats=...)` — an
attribute production does not have. Two D16 tests passed against a lookup the
real node could not perform. **A test fixture that manufactures production state
can validate a path that does not exist.** The stub now mirrors the real
`_vision_states` pool.

### B28 — the board never leaves AUTO, so `MANUAL_CONTROL` is discarded after any move  ⛔ SROT-PATH

**Verified in the Hengla source, end to end, 2026-09-08.** Not bench-verified: the
board refuses to arm on the bench (`NO_STATE_CHANGE (timeout)` with no thruster
pack fitted), so the armed move that would demonstrate it could not be run. The
source chain is unambiguous and is quoted here so nobody re-derives it.

Three of the four self-terminating flight modes restore `STABILIZE` when their
state machine finishes. `AUTO` does not, and no other line does it for them:

```c
AUTOTUNE finished -> g_state.control.mode = FlightMode::STABILIZE;  task_control_loop.cpp:855
STUNT    finished -> g_state.control.mode = FlightMode::STABILIZE;  :865
PATTERN  finished -> g_state.control.mode = FlightMode::STABILIZE;  :874
AUTO     finished -> (no such line exists)
```

`MAV_CMD_SROT_MOVE` sets `c.mode = FlightMode::AUTO` (`mav_commands.cpp:352`). On
completion the loop publishes `mv_done_seq` and clears `mv_was_active`
(`task_control_loop.cpp:945-949`) — and touches `c.mode` nowhere. So the board
stays in `AUTO` indefinitely after every move.

What `AUTO` then does with our pilot frame, all four axes:

```c
// computeDemands() computes fwd/lat as pilot passthrough at the top ...   :158-165
// ... and the AUTO branch then OVERWRITES them:
fwd = md.fwd; lat = md.lat;                                   // :243
thr = depth::update(0, in.depth, dt, tgt);                    // :241  (sp_throttle ignored)
attitude::stabilize(0, 0, md.yaw, ...);                       // :239  (sp_yaw ignored)
```

and after `PH_DONE` the phase is `PH_IDLE`, which returns a default-constructed
`Demand` — `float fwd = 0, lat = 0, yaw = 0` (`movement.h:31`) — with
`depth_target` still latched. **Therefore every axis of a `MANUAL_CONTROL` frame
is discarded while the board sits in post-move AUTO, and nothing logs it.**

The resulting state is *safe* (depth-hold + heading-hold + zero translation, i.e.
station-keep). The defect is that it is **silent**: a host that streams `manual()`
believing it is driving gets no error, no STATUSTEXT, and no motion.

**Current exposure is bounded — verified by enumerating every caller.** There are
exactly two `manual()` call sites outside tests:

| caller | guarded? |
|---|---|
| `motion_vision._srot_drive` (`motion_vision.py:626`) | ✅ `vision_verbs._ensure_srot_vision_mode` sets **and verifies** `STABILIZE` first (`vision_verbs.py:120-137`) |
| `SrotFC.send_neutral` (`srot_fc.py:1989`) | ⚠️ streams zeros, so the *effect* is benign — but its docstring asserts "STABILIZE holds", which is false after a move |

So this is latent, not live. It becomes live the moment any third caller streams a
non-zero `manual()` without a mode guard — and the thing that would have caught
that is a comment, not a mechanism.

**Fix landed (host side, zero extra wire traffic):** `SrotFC.manual()` reads the
mode already present in the cached HEARTBEAT (`get_mode()`, `srot_fc.py:1855`) and
warns, rate-limited, when it is a mode that discards the frame. No new messages are
sent — this deliberately respects the split where the board owns the 500 Hz loops
and the Pi sends the bare minimum.

**Owed upstream (firmware):** `AUTO` should restore `STABILIZE` on movement
completion, exactly as `STUNT`/`PATTERN`/`AUTOTUNE` already do. Also
`computeDemands`'s header comment — *"Surge/sway (fwd/lat) are always pilot
passthrough"* (`task_control_loop.cpp:150`) — is **false for AUTO and SURFACE**,
which both overwrite them.

### B29 — a test replaces a class for the WHOLE session, because its `finally` is gated on `None`  ✅ FIXED 2026-09-08

**`duburi_vision/test/test_camera_profiles.py:118`** (found while fixing B23).

```python
orig = F._build_v4l2.__globals__.get('V4L2MailboxCamera')   # -> None, ALWAYS
import duburi_vision.cameras.v4l2_mailbox as vm
vm.V4L2MailboxCamera = _Spy
try:
    ...
finally:
    if orig is not None:          # never true -> THE SPY IS NEVER RESTORED
        vm.V4L2MailboxCamera = orig
```

`factory` imports `V4L2MailboxCamera` **inside** `_build_v4l2`, so the name is not
in the module's globals and `.get()` returns `None`. The restore branch therefore
never runs, and `_Spy` — a two-method stub — stands in for the real camera class
for **every test that runs afterwards in the session**. Verified directly:
`'V4L2MailboxCamera' in F._build_v4l2.__globals__` is `False`.

The two sibling tests in the same file (`:172`, `:189`) do it correctly, reading
`orig = vm.V4L2MailboxCamera` from the module being patched. So this is one site
out of three, which is why it never looked wrong on a skim.

**Why it stayed invisible:** nothing downstream introspected the real class. The
B23 pump-loop guards are the first tests that do, and they failed **only in a
full-suite run** while passing in isolation — the signature of session pollution,
and a genuinely confusing one to chase.

**A `finally:` gated on a value that is always `None` is not cleanup.** The guard
was presumably defensive; defensiveness is what disabled the restore. Fixed with
`monkeypatch.setattr`, which owns the restore and cannot be gated away.

### B30 — the vision arrival brake calls a method SrotFC does not have  ⛔ SROT-PATH, LIVE, DEFAULT-ON  ✅ FIXED 2026-09-08

**`motion_writers.make_writers` → `motion_vision._brake_axis`.** REPRODUCED:

```
AttributeError: 'SrotFC' object has no attribute 'send_rc_override'
```

`make_writers` built every `Writers` around `send_rc_override` /
`send_rc_translation`, and **SrotFC implements neither**. On srot the one path
that calls `writers.forward` / `writers.lateral` is `_brake_axis`, the vision
**arrival brake** — which is **on by default** (`brake=not bool(brake_off)`,
`brake_off=False`). So a plain `vision_align(lat=0)` raised on arrival, on the
vehicle's primary vision verb.

**Three things hid it, and each is worth keeping:**

1. **Three of the four writers worked.** `neutral` maps to `send_neutral`, which
   SrotFC *does* have, so the bundle looked healthy.
2. **It is intermittent by momentum.** `_brake_axis` returns early when
   `abs(ema_pct) < VISION_BRAKE_MIN_PCT` (6 %), so a gently-converged lock never
   reaches the call and a fast snap-in does. It fails on the approach that has
   inertia — the hardest shape to catch on a bench, and the one that matters in
   water.
3. **The facade drift tests could not see it.** They scan `duburi.py` /
   `vision_verbs.py` for `self.pixhawk.<attr>`. These calls live in
   `motion_writers`, inside a lambda, reached through a `Writers` field.

**Blast radius, stated precisely.** LIVE: `forward`/`lateral` via the arrival
brake. LATENT: `depth_keepalive` (used only by `style_roll` and `set_depth`, both
in `MOVE_VERBS` and therefore collapsed onto the board) and every
`release_yaw=True` variant (srot never has a heading lock, so `_lock_active()` is
always False).

**Fix.** `make_writers` is backend-aware: on srot it returns writers that convert
the pwm argument back through the exact inverse of `percent_to_pwm` and actuate
via `manual()`. Callers still pass pwm and still do not care which backend they
are on. `_is_srot` moved down into `motion_writers` as the single definition
(`is_srot`) so `make_writers` can branch on it — two copies of "which backend am
I on" is how the split goes wrong in one file and not the other, which is
literally what this bug was.

Verified: ema +40 % → capped to `VISION_BRAKE_CAP_PCT` 30 → −30 % → pwm 1380 →
−30 % → −0.30 units → `MANUAL_CONTROL` lateral −300, surge and yaw 0. The pwm
round trip is exact across the band. 12 tests, **9 fail** without the fix.

⚠ **HOW IT WAS FOUND, because the method matters more than the bug.** A sanity
check asked whether all active srot bugs were closed. The answer was assembled by
hand-tracing call sites, and the classification of B12 came within one grep of
being wrong. Redoing it *mechanically* — a call-graph reachability walk from the
real srot entry verbs — immediately contradicted the hand answer on B18, and
chasing that contradiction produced B30.

**The root cause of the mis-classification was a truncated command.** The
evidence for "B18 is ArduSub-only" was
`grep -rn percent_to_pwm src/ | grep -v test | head` — and `head` cut the output
at 10 lines, hiding all **10** `motion_vision.py` call sites. A truncated view
presented as a complete one: the register's own recurring defect class, applied to
the method used to maintain the register.

### B31 — the vision arrival brake delivers 73 % of its command on srot ⚠ TUNING, NOT A CRASH

**MEASURED ON THE BOARD 2026-09-08** (live, disarmed, read-only), not computed
from defaults:

| param | live value |
|---|---|
| `JS_GAIN_DEFAULT` | **1.0** (the manager's startup write persisted) |
| `PILOT_EXPO` | **0.30** |
| `PILOT_SPEED` | 1.0 |
| `FRAME_REVERSE` | **1.0** (set on our hull, as CLAUDE.md says) |
| `MOVE_BRAKE_GAIN` | 0.55 |

Hengla shapes every pilot axis before the mixer
(`task_control_loop.cpp:158-165`) and scales it at ingestion
(`mav_commands.cpp:716-719`):

```c
fwd = ((1 - pe) * f + pe * f^3) * ps;        // computeDemands
sp_forward = (x / 1000.0f) * gain;           // onManualControl
```

so with the live values above:

| host commands | board applies | ratio |
|---|---|---|
| 30 % | **21.81 %** | ×0.727 |
| 10 % | 7.03 % | ×0.703 |
| 5 % | 3.50 % | ×0.701 |

**Consequence.** `VISION_BRAKE_GAIN` and `VISION_BRAKE_CAP_PCT` were tuned on the
ArduSub RC path. The same nominal brake on srot bleeds roughly **27 % less
momentum**, and the deficit is worst exactly where it matters — the small-signal
end, where the cubic term contributes almost nothing and the ratio floors at
`1 - pe` = 0.70. So a standoff tuned on the pixhawk path will overshoot on srot,
and re-tuning is a pool measurement, not an arithmetic correction.

Not filed as a defect in either stack: the shaping is deliberate on the board
(precise small-stick control) and the host is right not to pre-compensate blindly.
Recorded because "the brake works on srot" (B30) and "the brake brakes as hard as
it does on ArduSub" are different claims, and only the first is now true.

**Verified while measuring, so it is not re-derived:** `FRAME_REVERSE` negates all
six axes uniformly, once, immediately before the mixer
(`task_control_loop.cpp:897-900`), and the host only *reports* it — never applies
it. There is no double negation, and the brake's direction relative to the drive
it opposes is preserved.

### B32 — the srot vision test suite faked the component that was broken  ✅ FIXED 2026-09-08

Recorded separately from B30 because it is the reason B30 survived a round of
review, and the same mistake is available in every backend test we have.

`test_srot_vision_actuation.py` was written to prove the vision loops actuate
correctly on srot. Its `_FakeSrot` is scrupulous — it deliberately omits
`send_rc_override`, so a loop that reaches for one raises, which its own docstring
calls "the honest outcome". Then every test passes **`writers=_FakeWriters()`**.

`make_writers` is where B30 lived. The suite stubbed out the component under
suspicion, so it could not observe it. `_FakeWriters`'s docstring even says *"only
the arrival brake (`_brake_axis`) writes through `writers.forward`/`.lateral`"* —
the single real path was identified and then replaced with a double.

**Fixed** by adding cases that run `align_loop` / `move_loop` with the **real**
`SrotFC` (on a fake link) and the **real** `make_writers`. Nothing between the verb
and the wire is doubled.

⚠ **My first version of those tests did not bite** — 21 passed with B30
re-injected — because `_brake_axis` self-gates below `VISION_BRAKE_MIN_PCT` and a
static off-centre target never converges, so the brake was never reached. The
fixed version drives a measured approach profile (hard off-centre for 8 frames to
build the EMA past the 6 % floor, then a snap to centre) **and asserts it reached
the writer**, so if a gain or deadband change stops the brake firing the test
fails as "this no longer covers B30" instead of passing quietly. Verified: 2 fail
with the srot branch removed.

**Two guards now cover the whole class**, because finding B30 by accident is not a
strategy:
- `tools/fc_surface_audit.py` — finds every attribute LOADED off an fc-like object
  anywhere (by parameter, by `self.<attr>`, by local alias, through closures), and
  diffs it against each backend's real `dir()`. It is what would have printed
  `send_rc_override` in `motion_writers` on day one.
- `test_fc_surface_frozen.py` — freezes the resulting set of Pixhawk-only names
  with a written reason each. A new one fails the suite at the moment it is
  introduced (verified by injection) rather than in the water.

## 7d. Verified against primary sources (2026-09-08) — cite these, do not re-derive

Every row was checked against the authority named, not against another one of our
own documents. Where our stack and a published standard disagree, that is stated
rather than smoothed over.

| Claim | Verdict | Primary source |
|---|---|---|
| `MANUAL_CONTROL` x/y/r are ±1000 | ✅ ours matches | [mavlink.io common.html#MANUAL_CONTROL](https://mavlink.io/en/messages/common.html#MANUAL_CONTROL) — "normalized to the range [-1000,1000]", `int16_t`, `INT16_MAX` = invalid |
| `MANUAL_CONTROL.z` is 0..1000, **500 neutral** | ⚠ **deviates from the spec, deliberately, and the whole fleet deviates together** | spec says `[-1000,1000]` / 0 neutral; ArduSub `joystick.cpp:61-63` (*"Scale 0-1000"*, `throttleBase = 1500-500*scale`) is where the convention comes from; Hengla `mav_commands.cpp:708,718` matches ArduSub; our `srot_protocol.unit_to_mc_z` and Bondor `useGamepad.ts:83` both match. **A spec-literal sender's `z=0` is FULL DESCENT here.** Filed upstream as srot-control-board **PR #16** (docs) |
| Host→board unit conversion in the B30 fix | ✅ exact | Hengla `mav_commands.cpp:716-719`: `sp_forward=(x/1000)*gain`, `sp_throttle=((z-500)/500)*gain` — the inverse of what we send |
| `FRAME_REVERSE` negates all six axes uniformly, once, before the mixer; host must NOT re-apply | ✅ confirmed, host only reports it | Hengla `task_control_loop.cpp:897-900` |
| Pilot authority is **not** unity | ⚠ see B31 | `computeDemands` `task_control_loop.cpp:158-165` + live board params measured 2026-09-08 |
| `RC_CHANNELS_OVERRIDE` 65535 on Ch1–8 = "ignore this field", not "release" | ✅ (B24) | ArduPilot `GCS_Common.cpp:4213-4218`; `RC_OVERRIDE_TIME` default 3.0 s in `RC_Channels_VarInfo.h:90` |
| White-noise-acceleration Kalman `Q` discretisation | ✅ (B09) | standard `[[dt⁴/4, dt³/2],[dt³/2, dt²]]·σ_a²` |
| Flat-port refraction `n = 1.333`, `asin(sin θ/n)` | ✅ (B22) | Snell's law; single definition now in `duburi_vision/optics.py` |

⛔ **The one thing to carry forward from this table:** four independent
components agreeing with each other is *not* the same as agreeing with the
standard. On `MANUAL_CONTROL.z` our whole fleet is self-consistent and all four
depart from the published spec together. That is fine — until something outside
the fleet joins the link.

### B33 — the DSL's own worked example teaches the recovery BACKWARDS  ✅ FIXED 2026-09-08

**`duburi_planner/vision_dsl.py`, `VisionResult`'s docstring.** Found in the
full-depth planner read.

```python
elif res.saw_target:            # tried, didn't fully centre
    if res.x_px < -30: duburi.move_right(1)     # target LEFT  -> goes RIGHT
    elif res.x_px > 30: duburi.move_left(1)     # target RIGHT -> goes LEFT
```

Both signs are inverted — against the docstring's **own** field list four lines
below (*"+x = right"*), against `vision-results.md` §3 (*"`x_px > 0` … strafe
**right**"*), and against the control law CLAUDE.md §6 states for the verb
itself (*"+ex → Ch6 > 1500 → strafe RIGHT (no negation)"*).

**Consequence.** A recovery copied as written drives **away** from the target,
roughly doubling the residual instead of closing it — and it fires exactly when a
verb has already struggled, so it converts a near-miss into a loss. This is the
one convention CLAUDE.md singles out as a trap: *"Recovery sign matches `align`
itself."*

**Why it matters more than an ordinary comment error.** This is not narrative
prose — it is the worked example on the class a mission author is holding, in the
place they look first. `vision-results.md` has it right, but nobody reads the
reference doc while the docstring is under the cursor.

**Blast radius: none yet.** Grepped every mission and every FSM state — no
`x_px`-driven recovery exists in the tree, so nothing has copied it. It was
waiting for the first mission that did.

Fixed, and `test_vision_result_sign_convention.py` now pins the docstring's signs
against `vision-results.md` and against the "+x = right" field definition that
makes them correct — two copies of a sign convention being exactly how these came
to disagree. Verified: 2 of 3 fail with the inversion restored.

## 7e. Full-depth read of planner + sensors (2026-09-08)

Added because the earlier pass gave these two a **targeted two-class grep**
(the B05 `or 0.0` idiom, J02-style swallowed failures) and reported them clean —
which is the same shape as the ESC flasher's earlier "no findings": a statement
about code that had not been opened. 4,632 lines read.

**`duburi_sensors` (1,802 lines) — one finding, already on the register.**

| File | Verdict |
|---|---|
| `mavlink_ahrs.py` | ✅ The only yaw source usable on srot. Staleness gate delegates to `get_attitude_age()` per the pipeline contract; `SrotFC.get_attitude()` returns `math.degrees(att.yaw) % 360.0`, so the [0,360) contract is honoured across the backend boundary |
| `base.py` | ✅ contract is explicit about `None` meaning "no fresh sample" |
| `factory.py` | ✅ clean dispatch; fail-loud on unknown name and on missing required kwargs |
| `bno085.py` | ✅ **arithmetic verified**: exactly ONE negation at ingestion (`(-yaw) % 360`, ENU +CCW → NED +CW), `offset = (ref − raw) % 360`, `earth = (raw + offset) % 360`. Wrap-safe in both directions; Python's `%` yields [0,360) for a positive modulus. (B04 lives here and is unchanged) |
| `nucleus_dvl.py` | ✅ integration guarded by `0 < dt < 0.5`, validity bits checked, integrator zeroed on reconnect |
| `composite_bno_dvl.py` | confirms **B11** (docstring says "BNO **AND** DVL", code returns only BNO) and **B19** (`close()` does call both, but with no `try/finally`, so an exception in the first skips the second) |

One caveat worth stating rather than filing: the DVL integrates in **body frame**
without rotating by heading, so the sum is only a valid displacement while heading
is held — which is what `heading_lock` does during `*_dist` moves. Honest in the
docstring, bounded in practice, and moot on srot where `*_dist` is refused.

**`duburi_planner` core (2,830 lines) — B33 above, nothing else.**

| File | Verdict |
|---|---|
| `client.py` | ✅ `_result_deadline` is the P1 fix and is correct: `max(timeout, duration)` + margin, with the quick-verb value a **floor, not a ceiling**. Its `or 0.0` chain is the documented rosidl "0 == unset" convention falling back to the same `COMMANDS` registry the server enforces — not B05's shape |
| `mission.py` | ✅ `_safe_shutdown` runs on **both** the success and exception paths, each step isolated by `_try_step` so a failed `stop()` cannot prevent `disarm()`; Ctrl-C has its own ordered path (cancel → stop → disarm). **The opposite discipline to J02**, which swallows and still returns `SUCCEED` |
| `duburi_dsl.py` | ✅ every `except Exception` is annotated with why it is best-effort (detector pause, HUD follow, graph read), and a missing detector **raises loudly** rather than degrading |
| `vision_dsl.py` | ⛔ **B33**. `NaN` defaults and the `saw_target` guard are otherwise correct |
| `cli.py`, `model_context.py` | ✅ |

### B34 — raising the board's speed cap silently does nothing to autonomous moves  ✅ FIXED 2026-09-08

**`srot_protocol.sanitize_speed` / `MOVE_CRUISE_MAX`.**

`_speed_from_gain` clamps every host-issued move to `sp.MOVE_CRUISE_MAX = 0.80`,
a **hard-coded copy of the firmware's DEFAULT**. The board's `MOVE_CRUISE_MAX` is
a *runtime parameter* (`params.cpp:221`), and the host **never reads it** —
verified: no `get_param('MOVE_CRUISE_MAX')` anywhere in `duburi_control` or
`duburi_manager`.

All three agree today (host 0.80, firmware `DEF_MOVE_CRUISE_MAX` 0.80, and the
live board measured 0.80 on 2026-09-08), so nothing is wrong right now.

**The failure is one-directional and silent.** Lower the board's param and the
board clamps harder — safe, the host's 0.80 is simply moot. But **raise** it (for
a faster transit) and every autonomous move is still capped at 0.80 by the host,
while a Bondor/teleop move — which does not pass through `sanitize_speed` — is
not. The operator changes a speed limit, watches the vehicle not go faster, and
has nothing in any log to explain it.

Same family as B31: a host-side constant standing in for a board-side tunable.
Not fixed here because the right fix is a decision — read the param at connect
(one more round-trip on a link we already round-trip on), or keep the host cap and
say plainly in the log that it is the binding one. Recorded so the choice is made
deliberately rather than discovered on a pool day.

**Note on the existing test.** `test_srot_protocol.py:110` asserts
`sanitize_speed(0.99) == sp.MOVE_CRUISE_MAX`, which is tautological with respect
to the VALUE — it passes whichever number the constant holds. The drift test pins
every discrete wire constant against the firmware headers but not the tunable
floats, which is defensible (the board's clamp is authoritative) and is exactly
why this gap is about *operator expectation*, not safety.

## 7f. The audit lens, and what it covered (2026-09-08)

Every defect this audit found by READING is one shape: **a written claim that is
not true of the code.**

| | the claim | the code |
|---|---|---|
| B11 | docstring: "BNO healthy **AND** DVL streaming" | returns only the BNO |
| B16 | comment: "scale defensively" | never scales (the defence is a layer up) |
| B25 | a health reporter | could never report |
| B26 | a test stub | invented an attribute the real class lacks |
| B30 | comment: the guard exists | it existed in a different file |
| B32 | a test proving srot actuation | faked the component that was broken |
| B33 | worked example: recover this way | inverted vs its own field list |
| ESC PR #3 | call site reads as "buffer cap" | it is the timeout |

So the lens is: **read the docstring, then check the body does exactly that** —
and treat a comment as a claim to verify, never as evidence.

**Mechanised.** Two tools now apply it across the whole tree:
- `tools/claim_vs_code_audit.py` — flags docstrings that join two names with
  "and" where the body touches only one (B11's shape), and claim-verbs
  (clamp/scale/normalise/reject/verify) with no matching operation. 5 candidates,
  4 false positives, 1 chain that produced **B34**.
- `tools/defect_class_sweep.py` — sweeps the classes actually found here:
  absence-coerced-to-zero (B05/B10/B25), blind silent `except`, unbounded `while`.

**Read in full this round:** `duburi_sensors` (1,802), `duburi_planner` core
(2,830), `srot-esc-flasher` (2,147). **Deep-read on the live srot path:**
`vision_state.bbox_error` + `square_within`, `srot_connect`'s absence rendering,
`motion_vision`'s freshness math, `motion_writers`, the Hengla handlers.

**Verified by execution rather than by reading**, because a stated invariant is
still a claim: `_fresh_bounds`'s four documented invariants hold across 121 input
combinations including infinities and the early-return path that skips the final
clamp — `full < zero` always, both caps respected, no ZeroDivision in
`_freshness`, authority always in [0,1].

⚠ **What is NOT line-by-line read**, stated so the next round does not inherit a
false clean bill: `srot_fc.py` (2,683), `duburi.py` (1,568), `auv_manager_node.py`
(1,854), `motion_vision.py` beyond the sections above, and most of
`duburi_vision`'s large nodes (`detector_node`, `flow_node`, `guide`,
`display_node`) — roughly 17k lines. Every one of them HAS been swept by both
tools above and by `fc_surface_audit` / `srot_reachability`. That is coverage by
mechanism, not by eye, and the difference is exactly what let B33 hide.

### B35 — the depth validator is bypassed by the value it exists to reject  ✅ FIXED 2026-09-08
### ⚠ THIS ENTRY'S ORIGINAL HEADLINE WAS WRONG AND IS RETRACTED IN FULL

**What I first wrote:** *"NaN walks through the verb table onto the wire"*, and
*"the host relies on the far side's guard"*. **Both are false.** `SrotFC.move()`
has always refused a non-finite frame host-side —

```python
if not _finite(p1, p2, p3, p4, p5):
    return MoveResult(DENIED, f'{verb}: non-finite parameter -- refused host-side')
```

— and `test_move_denied_on_nonfinite_param` has always asserted that the goal
never reaches the wire. I found that only because my "fix" collided with the
existing `_finite` and broke the test that proves it. **The suite told me my
finding was overstated.** Recorded at length because the failure mode of an audit
is confident overclaiming, and that is worth more to a future round than a
tidy entry.

**What is actually true, and still worth fixing:**

1. **`_depth_to_dive` is defeated by NaN and by `-inf`.** It exists solely to
   reject a bad depth target, and its own comparison lets both through:
   ```python
   if target > 0.0:      # NaN > 0.0 is False. So is -inf > 0.0.
       raise ValueError(...)
   return -target        # -(-inf) = +inf  ->  dive to infinite depth
   ```
   `+inf` IS caught, which is exactly why reading it looks fine. The frame was
   then stopped one layer up by `move()` — so the outcome was a correct refusal
   with a **vague** reason, not an unsafe command.
2. **A non-finite `gain` was silently coerced to speed 0.0**, which passes the
   finite check. The board accepts a valid "move at speed 0", the hull does not
   move, and the mission believes it did — *absence rendered as a number*, the
   register's own recurring defect. Now refused.
3. The converters passed NaN through (`manual()`'s `_safe()` was the only thing
   catching it). Now each fails to its own neutral.

**The principle this settled — refuse vs. coerce, by context.** A STREAMING path
cannot raise once per 20 Hz tick, so `sanitize_speed`/`unit_to_mc` fail safe
silently (0 = no motion, 500 = hold depth). A ONE-SHOT verb must refuse loudly:
silence there is a mission that thinks it moved. Both are "fail safe"; only one is
right per context, and conflating them is how a coerced zero becomes a phantom
manoeuvre.

⛔ **MY FIRST FIX WAS DANGEROUS AND MY OWN TEST PASSED IT.** I made `_clamp` map
NaN → `lo`. For `unit_to_mc` that is `-1.0` → **−1000: full reverse thrust**;
for `unit_to_mc_z`, full descend. A *range* assertion accepts −1000 happily, so
the range-only hammer reported **zero violations** on a change that commands
maximum thrust from a missing number. **A range check tests the interval, not the
meaning.** The tests now assert NEUTRALITY per converter, and `_clamp` carries a
comment forbidding the "fix" I attempted.

**Net value of this entry:** one real bypass (`_depth_to_dive`), one real silent
coercion (`gain`), better error messages that name the field, defence in depth on
the converters — and two lessons that cost more than the bug: an audit can
overclaim, and a test can pass a dangerous change.

## 7g. First principles: why this stack is shaped the way it is

Written after reading it as a whole, because the next round should inherit the
*reasoning*, not just the defect list.

**1. The vehicle is a two-computer system, and the split is the design.** The Pi
does perception and planning (not real-time); the SROT board closes every control
loop at 500 Hz (hard real-time). The wire between them carries **intent**, not
actuation — `MAV_CMD_SROT_MOVE` for a whole leg, `MANUAL_CONTROL` for a servo
tick. That is why `FlightController` exposes `move()` and `manual()` rather than
channels, and it is the right boundary: it survives the autopilot changing
underneath it, which is exactly what happened when the Pixhawk left.
*Cost of the choice:* every defect now has TWO homes, and the interesting ones
live in the seam — B28 (the board stays in AUTO), B30 (a writer the backend
lacks), B31 (authority differs by ×0.727), B34 (a cap the host copied), B35 (a
value the far side rejects for us). **The seam is where to look first.**

**2. "Absence is the signal" is the load-bearing invariant.** Firmware rev 3
refuses to report data it cannot stand behind; the host renders `--`, never
`0.0`. Nearly every defect on this register is a violation of it wearing a
different costume: B05 (`or 0.0`), B10 (a stale latch), B25 (a reporter that
never reports), B35 (NaN treated as a number by comparison). The reason it keeps
recurring is that **the safe default and the arithmetic default are different
values**, and the language supplies the second one for free.

**3. Guards must fail closed, and must be *ours*.** The register's best work is
the fail-closed choices: `check_behaviour_rev` refusing to arm below rev 2,
`square_within` returning False without a pose, `_fresh_bounds` capping trust in
ABSOLUTE time so a degraded pipeline cannot argue itself into more authority.
The recurring failure is the mirror image: a guard whose protection is
*incidental* — B23's freshness clock standing in for an exception handler, B35's
firmware check standing in for host validation. A guard you did not write is a
guard that can be removed without telling you.

**4. Two copies of one fact is the root cause, not the symptom.** B22 (a
refractive index in six places), B30 (`_is_srot` about to become two), B33 (a sign
convention in a doc and a docstring), B34 (a cap in the host and the board).
Every one was fixed by deleting a copy, not by synchronising them. Where a second
copy is unavoidable — the wire constants — the answer is a drift test that reads
the *firmware's own headers*, which is the pattern worth extending.

**5. A test that doubles what it tests proves nothing (B32), and a test that
checks the wrong quantity is worse (B35's range check).** Both passed while the
thing they existed to catch was broken. The discipline that works here is:
inject the defect and watch the test fail. Every fix on this register from
2026-09-08 onward was verified that way, and twice it caught a test that could
not bite.

### B36 — a failed `arm()` did not stop the mission; 16 of 16 missions discarded it  ✅ FIXED 2026-09-08

**`duburi_planner/duburi_dsl.py`.** Found by applying **NASA JPL Power-of-10 rule
7** to the tree — *"the return value of non-void functions must be checked by each
calling function"* (Holzmann, *The Power of 10: Rules for Developing
Safety-Critical Code*, IEEE Computer 39(6), 2006 —
[spinroot.com/gerard/pdf/P10.pdf](https://spinroot.com/gerard/pdf/P10.pdf)).

`DuburiMission._send()` does not raise. It logs the outcome, records
`success=False` on the scoreboard, and returns. **Every one of the 16 missions
that arms does `duburi.arm()` as a bare statement** — including
`task_full_2026`, the full competition run, which arms and then executes all five
task chunks.

**Verified by execution**, not by reading:

```
arm() -> success=False   and it did NOT raise
verbs sent AFTER the failed arm: ['set_depth', ...]
```

So a pre-arm refusal left the mission running its whole sequence **disarmed**: the
board refuses every move (*"SROT_MOVE refused: arm first"*), the hull sits still,
each verb logs its own failure, the run reports complete, and a pool slot is gone.
Nothing is physically unsafe — and nothing stops.

This is **J02's shape at the top of the stack** (the one call a step exists to
make, failure swallowed) in the worst possible place, because arm is the
precondition for everything after it.

**Fix.** `arm()` now logs at ERROR with the board's refusal reason and raises
`MoveFailed`. Raising is safe here: `mission.py`'s runner calls `_safe_shutdown`
on both the success and unhandled-exception paths, so the vehicle is still
released, stopped and disarmed. `arm(required=False)` preserves the old behaviour
for a diagnostic that wants to observe a refusal and continue.

**`disarm()` is deliberately NOT given the same treatment** — it is called from
`_safe_shutdown`'s isolated steps, and raising there would abort the cleanup that
exists to run after a failure. The asymmetry is a decision, and there is a test
saying so.

**Generalised:** `tools/unchecked_error_returns.py` finds every function that
reports failure in its return value (`return False, ...` or `MoveResult(...)`) and
every call site that discards it. 24 reporters, 5 discarded — the other four are
`disarm`, `stop` and cleanup paths where discarding is correct and now documented.

### B37 — error messages were accurate but not actionable  ✅ FIXED 2026-09-08

Error paths are the least-executed code in the stack, and they run exactly when
the operator can least afford to decode them. So they were **executed**, not
read — each failure triggered for real and graded on three questions: does it say
WHAT failed, WHY, and WHAT TO DO?

Five of eight had no remedy. Two of those matter at the pool:

| path | before | after |
|---|---|---|
| non-finite param | `non-finite duration: nan` | *"…a NaN here almost always comes from a VisionResult whose target was never seen — check `saw_target`"* |
| **move stall** | `no terminal ACK within 8s (stall)` | names `connect` to test the link, **and** warns that an unhealthy-Bar30 `DEPTH_HOLD`/`AUTO` refusal presents identically |
| unknown mode | `unknown SROT mode 'X'` | lists the valid modes |

The stall one is the important one. It is the commonest real failure, and its two
causes — a dead link and a board refusing every AUTO move because the Bar30 is
unhealthy — are **indistinguishable from the host side**. That ambiguity is
already documented in CLAUDE.md; it now appears where it is needed, in the message
itself, rather than in a file nobody opens mid-run.

Pinned by tests that assert the **remedy**, not the wording — rephrase freely,
just do not drop the pointer that makes it useful at 2am.

**Deliberately not "fixed":** `verb 'X' has no SROT_MOVE mapping` stays terse. It
is a programming error, not an operator one, and the traceback already names the
caller.

### B38 — a failure looked exactly like a success in the operator's log  ✅ FIXED 2026-09-08

Asked for directly: *"give warning for — no terminal ACK within 8s (stall)"*.

The stall — the commonest real failure on this link — was **never logged**. It
braked the hull and returned `MoveResult(TIMEOUT, ...)`; whether anyone saw it
depended entirely on the consumer. And the consumer was
`DuburiMission._send()`, which logged **every** outcome through one formatter at
`info`. So the moment the vehicle stopped responding produced a line visually
identical to the nineteen successful lines above it.

Two changes:
- `SrotFC` **WARNs at the instant of the stall**, so it is visible regardless of
  consumer, and says the hull was braked (nobody should have to wonder).
- `_send()` levels the line by outcome: `info` on success, **`warning` with a
  `!!` marker** on failure.

### B39 — a stall meant SLOWNESS, not silence, and braked healthy moves  ✅ FIXED 2026-09-08

Found by research, against the protocol we speak. [MAVLink command
protocol](https://mavlink.io/en/services/command.html):

> *"The GCS should have a **much increased timeout** after receiving an ACK with
> `MAV_RESULT_IN_PROGRESS`."*

`_relay_move_ack` armed **one** deadline before the loop and never extended it.
The `IN_PROGRESS` branch only forwarded progress to the caller. So a move that was
demonstrably alive — the board reporting rising progress every tick — was declared
stalled and **braked** the moment it exceeded ~2× its predicted time.

Braking a healthy manoeuvre mid-leg is worse than waiting for it: the leg is lost
and the mission continues from somewhere unplanned. It would bite exactly where
prediction is weakest — a dive fighting buoyancy, a long brake phase, a board
merely slower than `MOVE_DEPTH_RATE` assumes.

**Fix:** the deadline is a **silence window** refreshed by genuine progress, under
an absolute ceiling (`_STALL_HARD_MULT = 4`) so a board reporting progress for
ever still terminates. Measured after: a move alive for **4.0 s against a 1.4 s
budget succeeds** (would have been braked); silence still stalls at the budget;
endless progress is stopped at the cap.

⛔ **THE STALE-ACK TRAP, AND A TEST OF MINE THAT DID NOT BITE.** `_cache()`
returns the *same* `COMMAND_ACK` object every poll until a new one lands, so
refreshing on every *sighting* would let ONE stale IN_PROGRESS hold the deadline
open for ever — turning the backstop into a hang. The refresh therefore lives
inside the `prog != last_prog` branch.

My first test for that asserted only the returned **code**, and **the broken
version passed it**: the deadline was indeed held open for ever, but the hard cap
then terminated the run with the same `TIMEOUT` code, ~4× later. The cap masked
the bug. The test now asserts *when* the stall happened — the only thing that
tells "stalled on silence" apart from "ran to the ceiling". Verified failing on
both the un-extended and the over-extended implementations.

That is the second time this round a test checked the wrong quantity (B35's range
check was the first). Assert the property you actually care about, not a
consequence that has more than one cause.

### B40 — the MAVLink reader could die silently and present as a dead cable  ⛔ HOST-SIDE  ✅ FIXED 2026-09-08

**`auv_manager_node.reader_loop`.** Found by walking the srot ↔ Pi pipeline stage
by stage and asking, for each, *"what detects this failing?"*

`reader_loop` is a bare `threading.Thread` target and, by its own docstring, the
**only place `recv_match()` is called** and *"the only thing draining the link"*.
Its body had **no exception handling at all** — a recorder write, one of the three
demux callbacks (`note_named_value` / `note_battery` / `note_statustext`), or a
malformed frame could kill the thread outright.

**The consequence is a misdiagnosis, which is worse than a crash.** With the
thread gone, `master.messages` stops updating, every reading ages out, and
`board_link` reports *"no heartbeat within the stale window"* ~3 s later. A
host-side software fault therefore presents **identically to a dead cable** — at
the pool, with the hull in the water. Someone pulls the USB-C and re-seats the
board while the real cause sits in this process.

**Fix, both halves:**
1. The loop body is guarded. A fault is logged (first occurrence, then every
   200th — a persistent fault at 200 Hz would bury the log it is trying to be
   found in), counted, and survivable, with a back-off so it cannot spin.
2. The thread's death is **observable**: a new `mavlink_reader` health reporter,
   registered deliberately *next to* `board_link`, that says which side is at
   fault — *"READER THREAD IS DEAD — this is a HOST fault, not the link. Do not go
   looking at the cable."* Survived faults are DEGRADED (the telemetry they
   interrupted was real); a probe that raises is UNKNOWN, because nothing is then
   watching the watcher.

This is **B23's shape** (an unguarded bare `Thread` body) at the most critical
place in the stack, which is why it gets its own entry. B23's own note said the
mitigation there was *incidental*; here there was none at all.

**Verified by injection:** removing the guard fails the structural test;
a raising demux callback in a loop of the same shape keeps running and still
drains real messages; all four reporter states behave.

### B41 — the camera health line under-counted frames (read-then-zero race)  ✅ FIXED 2026-09-08

`CameraNode._sent`/`_dropped` are incremented by the **capture thread**;
`_log_health` runs on a **ROS timer thread** and did:

```python
hz = self._sent / elapsed
...  is_healthy(), format the line, get_logger().info(...)  ...
self._sent = 0            # everything published in between is LOST
```

**The window is neither theoretical nor narrow.** Between the read and the reset
the method probes the camera, builds a string and **logs** — and logging is I/O,
which releases the GIL. Every frame published during that window vanished from the
count.

Measured (yield in the window, i.e. what the real logging call is):

| scheme | published | counted | lost |
|---|---|---|---|
| read-then-zero | 83,719 | 114 | **99.86 %** |
| delta | 137,667 | 137,667 | 0 |

That readout is the line an operator uses to decide the pipeline is healthy, so
under-reporting **drops** is precisely the wrong direction to be wrong in.

**Fixed by structure, not by a lock:** counters are cumulative with exactly ONE
writer (the capture thread); the reader keeps its own high-water marks. Single
writer + private reader state needs no mutual exclusion, costs nothing in the
publish path, and stays correct under PEP 703 where `+=` is not atomic either.

⚠ **My first demonstration of this FAILED — LOST=0 for both schemes** — because it
had no work inside the window. That made the test unrepresentative of the real
`_log_health`, which does I/O there. Reported here because "I could not reproduce
it" was the honest state for one iteration, and the fix would have looked
unmotivated if I had stopped there.

## 7h. Threading and the GIL — the analysis, with evidence

The stack runs **~20 threads across 8 node processes**. Asked whether any should
become processes to escape the GIL; the answer is **no**, and the reasons are
worth recording so it is not re-litigated.

**1. It is already multi-process.** `camera_node`, `detector_node`, `auv_manager`,
`lock_node` and the rest are separate ROS 2 executables. Process isolation exists
at the node boundary, which is where the expensive data already stops moving.

**2. The GIL is not the constraint.** Every hot thread is blocked in a C
extension or on I/O, all of which release it: V4L2 `ioctl`, Hailo/ONNX inference,
OpenCV, `serial`/socket reads, `time.sleep`. Threads are the *correct* tool for
that shape.

**3. Converting would cost more than it saves.** A 640×480×3 frame is 921 kB;
moving it between processes means pickling or shared memory per frame, on a Pi
already measured as bandwidth-sensitive (`project_capture_publish_split`).

**4. What the GIL genuinely does NOT protect — and where the real bugs are.**
Per PEP 703, dict/list ops keep per-object locks even free-threaded, so
`cache[k] = v` is safe. The hazards are **compound** operations: read-modify-write
and check-then-act. Those were never atomic. `tools/race_audit.py` swept the tree
for exactly that shape:

- **`V4L2MailboxCamera`** — 8 counters flagged, **all clean**: every one has a
  single writer (`_pump_loop`), and `_idx` is written only by `read()`. Single
  writer needs no lock, and adding one to a 200 Hz pump would be cost for nothing.
- **`CameraNode`** — a genuine race, **B41** above.
- `Heartbeat`, `SrotRecorder`, `BNO085Source`, `NucleusDVLSource` — all already
  hold a lock in the class.
- `SrotFC._named_cache[name] = (value, time)` — written by the reader thread, read
  by the action thread. A single dict store of an immutable tuple: safe under the
  GIL **and** under PEP 703's per-object locks.

**5. The one thread-safety rule this stack already gets right**, verified against
the published warning that *"the mavlink_connection object is not thread safe"*:
`auv_manager_node` is documented and enforced as **the only thread calling
`recv_match()`**, and every write goes through `_tx_lock` because pymavlink shares
one sequence counter.

### B42 — Ctrl-C during a move did not brake the hull on srot  ⛔ SAFETY  ✅ FIXED 2026-09-08

**`auv_manager_node._emergency_stop`.** Found with a new lens: *what is specific
to a **ROS 2 autonomous vehicle** that has not been examined?* — executor topology,
QoS compatibility, and the shutdown path.

CLAUDE.md safety rule 1 is non-negotiable: *"Ctrl-C on the manager triggers
`Duburi.stop()` + `disarm()`."* The step that was supposed to stop the thrusters
was `pixhawk.send_neutral()`. On srot that is a **zero `MANUAL_CONTROL` frame** —
and a `SROT_MOVE` leaves the board in `AUTO`, where the firmware overwrites every
pilot axis from the movement primitive and the frame is **discarded** (B28).

Ctrl-C arrives most often *during* a move. So the one step that exists to halt the
hull was a no-op in exactly the case it exists for, leaving `disarm()` as the only
thing stopping it: **motors cut mid-leg instead of a commanded brake**, and nothing
at all if the disarm is the step that fails (its `[--]` path is real and printed).

`stop_motion()` — MOVE_STOP, honoured in AUTO, decelerating on-board since fw
rev 2 — was already used on the srot surface path (`:1175`) and simply not here.
Now called first, feature-detected because `PixhawkFC` has no `stop_motion`.

**Two verified negatives from the same lens**, recorded so they are not re-audited:
- **Executor topology is sound.** The action server is `Reentrant` and separate
  from the timers, so a long move cannot block the 2 Hz heartbeat that feeds the
  board's GCS failsafe. The heartbeat's own `MutuallyExclusive` group holds only
  non-blocking callbacks, and every `_tx_lock` hold is 2–15 lines with no sleep,
  loop or round-trip — no starvation path.
- **QoS is compatible everywhere.** 26 publishers, 54 subscribers, 13 topics with
  both ends in-tree: **zero** BEST_EFFORT-pub/RELIABLE-sub pairs, the mismatch
  that silently delivers nothing. `camera_info` is belt-and-braces —
  `TRANSIENT_LOCAL` on both ends *and* republished per frame, so a late-joining
  manager cannot miss it. `tools/qos_audit.py`.

⚠ **My first test for this failed on correct code.** It compared `src.index('stop_motion')`
with `src.index('send_neutral')` — and the explanatory comment above the fix
mentions `send_neutral` first, so it was comparing a comment against a call. Now
it parses the AST and compares actual call order. Third time this audit that a
test of mine measured the wrong quantity.

### B43 — a board reboot aborted the command but left the board misconfigured  ✅ FIXED 2026-09-08

**`auv_manager_node._publish_srot_telemetry`.** `check_for_reboot()` is wired and
aborts the active command — and its own message says the board is *"disarmed and
no longer configured"*, while **nothing reconfigured it**. The next verb then ran
on a board holding its compiled defaults. Both losses are silent:

| lost on reboot | consequence |
|---|---|
| `JS_GAIN_DEFAULT` → 0.5 | every `MANUAL_CONTROL` — every vision align, every arrival brake — at **half authority**. The hull just corrects more weakly. |
| stream rates → defaults | ATTITUDE ~55 Hz → ~11 Hz, so the vision loop's freshness decay bleeds translational authority on a link that looks healthy. |

A reboot is not exotic on this board: opening the serial port reboots it, so any
second process touching the device causes one — and a brownout on a thruster
current spike is the in-water version.

`_reapply_srot_config()` re-pushes exactly the two things a reboot loses and that
we set ourselves, and says plainly what was lost in the meantime. It deliberately
does **not** re-run `check_behaviour_rev` / `check_yaw_reference`: those answers
cannot change across a reboot of the same firmware, and their round-trips from a
2 Hz tick would be link traffic for no information. A failed reconfigure is an
ERROR naming the loss, not a swallow.

## 8. Provenance

Scratch working notes for this audit ran to 41 numbered findings (`F001`–`F041`)
plus amendments. **This document supersedes them**; the F-numbers exist only in
the session transcript and are not a durable reference. Cite `B01`–`B23` and
`J01`–`J03`.

Two gradings were corrected before shipping, both recorded here rather than
quietly amended:
- **B02/B03 severity** — first written as live, then downgraded on CLAUDE.md's
  "DVL not fitted, never validated in water", then corrected again by the
  operator: the DVL *was* fitted and flown on Duburi 4.5 and the code *is*
  tested; the hardware has since been lost and the code is to be maintained. The
  bugs are defects in trusted, exercised code that are dormant for want of an
  input — not speculative driver bugs.
- **B06 consequence** — the mechanism is verifiable in-repo and stated as HIGH;
  the claim about what ArduSub does with a released Ch4 and no receiver is
  *not* assertable from the Python and is flagged as an open question needing a
  bench read.

---

*Audit of 2026-09-08 · find-only · nothing fixed · suite green (1825 passed).*

---

## 9. Appendix — FIXED defects whose rationale is cited from source

Migrated verbatim from the retired `known-issues.md` on 2026-09-08. These are
all **FIXED**. They are kept because source comments and tests cite them by ID
(`detector_node.py` → D16, `vision_state.py` → D10, `test_detector_recovery.py`
→ D16, CLAUDE.md → P1/P2/D7–D10/D14) and the rationale is the part worth keeping.
Environment traps E1–E5 moved to [`jetson-and-env-traps.md`](jetson-and-env-traps.md).

- **Fix:** `_orchestrate` catches `MoveFailed`/`MoveRejected`/`Exception` and returns a non-fatal `VisionResult(False, 'FAILED', …)`; the mission logs it and continues to the next step.

### D6. Silent "sees but doesn't move" on a mis-scaled stream — **FIXED 2026-06-24**
- **Files:** `motion_vision.py`, `vision_state.py`
- **Symptom:** pixel math ran before `camera_info` arrived (image size still `(0,0)`), mis-scaling the error; and a class-name mismatch produced no diagnostic.
- **Fix:** the verbs return `NO_CAMERA` until `vision_state.info_seen()` is true, so the controller never steers on an unscaled pixel error. A throttled "`<class>` not among live detections […]" warning fires when boxes are present but none match the requested class. Class matching is case-insensitive.

### D7. `heading_lock` jittered the hull during a lat-only `vision_align` — **FIXED 2026-06-29**
- **File:** `heading_lock.py`
- **Symptom:** with a lock active, a lat/depth-only `vision_align` made the hull **wobble left/right in yaw** while lateral-correcting. The lock's `speed = max(LOCK_SPEED_MIN_PCT, …)` hard floor is a **relay on the Ch4 yaw RATE**: holding a still hull it barely fired (error stayed in the 1° deadband), but a lat align strafes on Ch6 and an off-CG vectored frame turns that into a **continuous yaw moment** that pushes heading out of the deadband every tick → the lock kicks ≥5% → overshoot → sign flip → **limit cycle**. Same mechanism as the `turn` bug fixed in `ab2014f` (`motion_yaw`), which deliberately left `heading_lock` untouched on the (correct-for-still-hold) assumption it was fine.
- **Fix:** taper the floor — `LOCK_APPROACH_BAND_DEG = 6.0` + `_lock_floor()`; `_lock_command()` now does `max(_lock_floor(|err|), min(LOCK_PCT_MAX, kp·|err|))`. Only the 1°–6° band softens (kills the relay); ≥6° keeps the full stiction-break floor, ≤1° still commands 0. Pure-P (no integral), so under a sustained disturbance it settles to a small **bounded heading offset** rather than wobbling — raise `LOCK_KP_PCT_PER_DEG` or add a `LOCK_KI` follow-up if that droop is too large. **Answer to "would releasing the lock fix it?": no — that hands yaw to ArduSub's aluminum-hull compass, the worst jitter source. Keep the BNO lock; the taper is the cure.** Pool re-verify straight-line heading hold during `move_forward_dist` (shared lock). Toy-plant test pins taper-settles vs hard-floor-wobbles.

### D8. `vision_align`/`move` commanded Ch4 when yaw wasn't requested — **FIXED 2026-06-29**
- **File:** `vision_verbs.py`
- **Symptom:** Ch4 release was gated on **lock-state** (`release_yaw = _lock_active() and not touches_yaw`), so a bare `align(lat=,depth=)` with **no** lock wrote `yaw=1500` every tick (and `vision_move` did likewise without a lock) — commanding the yaw channel the operator never asked for, and able to fight a later/other Ch4 author. (Refinement of D3, which only covered the lock-active case.)
- **Fix:** gate Ch4 on the **yaw axis**, not the lock: `vision_align` → `release_yaw = not touches_yaw`; `vision_move` → `release_yaw = True` always (it never computes a yaw command). The verb now writes Ch4 **only** when `yaw` is a requested align axis; otherwise it stays on `send_rc_translation` (lateral-only), leaving Ch4 to the lock / heartbeat / ArduSub.

### D9. `align(err=0)` logged "aligned (36px)" — **FIXED 2026-06-29**
- **Files:** `commands.py` (unchanged, documented), `motion_vision.py`, `detector_node.py`
- **Symptom:** an explicit `err=0` was silently turned into a 40px deadband, so the loop honestly reported `aligned (36px)`. Root cause is the **rosidl `0==unset` convention** in `commands.fields_for` (`unset = (value == 0.0)`) substituting the spec default — which is **load-bearing**: the CLI deliberately omits vision fields so the rosidl zero triggers `vision.*` ROS-param defaults (`_LIVE_TUNED_COMMANDS`). An explicit `0` and an omitted field are indistinguishable on the wire, so `err_px` can't be exempted.
- **Fix (honesty, not a sentinel):** `err=0` means "use the default/param" (documented); a **small positive `err` is the tight knob** (only literal `0` collides — e.g. `err=8` is honored). `align_loop` clamps the effective deadband to `MIN_ALIGN_ERR_PX` (≈5px bbox jitter) so an over-tight positive `err` can't perpetually TIMEOUT, prints the effective deadband at align start (floor noted, never silent), and the success line now states it: `aligned (N/Mpx)`. The detector's always-on line was reworded from `… align ['hole'] center -> (0,0)` to `[ offset … ] 'hole' bearing (live, off-centre)` so the operator never reads that raw-offset telemetry as the verb's alignment verdict.

### D10. Control coast through detection gaps — re-enabled SAFELY (opt-in) — **2026-06-30**
- **Files:** `vision_state.py`, `motion_vision.py`, `vision_verbs.py`, `vision_tunables.py`, `tracker_node.py`, `roboflow_tracker.py`
- **History (the bug we must not repeat):** commit `a8bf8ea` added `vision.use_tracks` → the control loop read `/tracks`; `8335afb` ripped it out. Root cause: coasted (Kalman-predicted) boxes carry `score=0.0`, which collided with the conf gate (`min_score`/`ctrl_conf>0`) → `bbox_error()` returned `None` → `_present()` False → **drive neutral → AUV stopped** despite a good predicted box. Latent second mode: `_freshness()` keys off *message* age, but a predicted box arrives every frame (age≈0) → **full authority on a drifting phantom**. The fix made control read raw `/detections` only, occlusion handled by mission `fallback`.
- **Re-enable design (Part B, `vision.coast_s`, default OFF):** the coast is now **additive and opt-in**, inverting both failure modes — (1) a live `/detections` box ALWAYS wins (`/tracks` is consulted only to fill an empty tick; coast never gates out or overrides a real box); (2) coast authority decays by **TRUE detection-age** via a separate `_coast_authority` curve dispatched by `_authority()` (a coasted box is fresh every tick, so applying `_freshness` would double-decay it — and a *live* box must NOT be coast-decayed); (3) conf-exempt **only for the locked `track_id`** (a predicted box of any other id is ignored — built outside the `min_score` path, only for `locked_id`). `coast_s=0` ⇒ byte-identical to the proven raw-`/detections` path.
- **Timeout ladder (4 rungs, must stay ordered):** `_freshness` (0.4s, per-frame live staleness) < `vision.coast_s` (~0.8, coast window) < `vision.lost_grace_s` (1.0, → LOST → `fallback`) < tracker `max_predict`/`lost_track_buffer` in **wall-time** (the track must outlive the coast). The 4th rung bit us once: the Kalman smoother drops a track from `/tracks` after `max_predict_frames`, so `max_predict` (launch default was 10=0.5s, yaml 15=0.75s — both **below** `coast_s=0.8`) silently truncated the coast; raised to 30 (1.5s @20Hz). **Pool-gated:** validate with `coast_s=0` first (byte-identical), then `≈0.8` on the torpedo hole/gate; slalom last and cautiously (a coasted lateral box drifts fastest there).

### D11. Fire could leave on a stale/coasted box; align declared on one frame at low FPS — **FIXED 2026-07-01 (completion audit)**
- **Files:** `motion_vision.py` (`align_loop`), `Move.action`, `vision_tunables.py`.
- **V-FIRE (the dangerous one):** the mid-hold `on_locked` (torpedo/dropper fire) gated only on `stable >= align_stable_frames`, with **no freshness check**. If the detector froze right after alignment, `stable` stayed at threshold on the ≤`_STALE_LIMIT_S` (1.0s) cached box (freshness zeroed the *drive* at 0.4s, but not the *fire*), and a torpedo could leave on a stale — or, with `coast_s>0`, a Kalman-**predicted** — box. **Fix:** fire now additionally requires `not sample.coasted and sample.age_s <= VISION_FRESH_FULL_S` (0.10s). Coast holds the lock; it never takes the shot. A dropping hole waits (within the hold) instead of firing blind.
- **V-STABLE:** `stable` (and the settle gate's `prev_worst`) counted 20 Hz **loop ticks**, not detections — so at 3-4 Hz a single in-band frame re-read 3× in 0.15s declared ALIGNED / armed the fire on effectively one frame, and the settle gate was degenerate (`worst==prev_worst` on re-reads). **Fix:** a detection's arrival time is `now - sample.age_s` (constant across re-reads); `stable`/`prev_worst` only advance on a **new** frame (`> last_frame_at + _FRAME_EPS_S`, 5ms). `align_stable_frames` now means "N distinct in-band detections" — FPS-independent. Backward-compatible: at healthy FPS (age≈0 every tick) every tick is a new frame, so behaviour is unchanged (and the `age_s=0` test doubles stay valid). Duration budgets re-verified (smallest align 4s ≫ ~1s worst-case declare @3-4Hz). **Raising real FPS (TensorRT engine) is still the primary lever** — this is the per-frame guard.

### D12. Torpedo 10/10: fire-would-not-leave, depth z-wobble, terminal yaw jitter — **FIXED 2026-07-01 (pool feedback, 6/10→target 9-10/10)**
- **Files:** `motion_vision.py` (`align_loop`, `_vision_yaw_floor`), `heading_lock.py` (`set_hold_mode`, deadband param), `vision_verbs.py`, `duburi.py` (`_set_lock_hold`), `commands.py`, `Move.action`, `vision_dsl.py`, `missions/{task_torpedo,pool_day_torpedo}.py`.
- **Fire would not leave despite a perfect lock (the 4/10 miss):** D11's fire-freshness gate (`age_s <= VISION_FRESH_FULL_S`, 0.10s) is *narrower than one frame period* at 3-4 Hz, so a perfectly-aligned hull kept missing the fresh-tick coincidence with `stable≥N and (now-aligned_at)≥fire_t`. **Fix:** gate the fire on **`is_new_frame and not sample.coasted`** — fire on the tick a genuinely NEW live box lands. Fresh by construction at ANY FPS (fixes the miss) AND strictly safer than D11: a frozen detector produces no new frame, so it can never fire on a stale box even while `stable` stands held at threshold through a mid-hold freeze (the freeze-AFTER-alignment case a looser age gate would reopen). Discriminating test: `test_fire_withheld_when_detector_freezes_after_alignment`.
- **`fire_pass` (opt-in, default off):** guaranteed partial-points shot — if no strict in-band fire landed, fire at a NATURAL exit (TIMEOUT / hold-complete) provided the target was seen LIVE within `lost_grace_s` (never on a never-seen or coasted-only target). DSL `align(fire_pass=True)`.
- **Depth axis z-wobble:** the setpoint was recomputed EVERY 20 Hz tick (up to 0.4 m/s slew ArduSub chased) and nudged *even inside the deadband* (dithering on bbox-y jitter). **Fix:** setpoint now steps **only in the 5 Hz block**, **frozen inside the deadband**, capped per-update by the new **`depth_step`** resolution arg (m; 0.02 slow .. 0.10 coarse; the sole depth-rate knob — `gain_depth` no longer scales depth). Max slew = `depth_step × 5 Hz`, so ArduSub's ALT_HOLD PID settles between steps. `_MIN_DEPTH_M` surface floor preserved.
- **Terminal yaw jitter (launcher wobble at the hole):** the reported wobble is NOT the vision yaw floor — both torpedo missions **drop the yaw axis** at the hole (`release_yaw=True`), so Ch4 is owned by `heading_lock`, which limit-cycled against the lateral-strafe yaw moment. **Fix:** (a) `heading_lock` **fire-window quiet mode** — `vision.align(hold_heading=True)` widens the lock deadband (`LOCK_DEADBAND_DEG` 1° → `LOCK_HOLD_DEADBAND_DEG` 3°) for the hold via `set_hold_mode()`/`_set_lock_hold()` (try/finally), so the lock holds steady instead of micro-correcting sub-deg noise; restored on exit. Wired into both torpedo terminal aligns. (b) The vision yaw floor (`motion_vision.py`) is ALSO tapered now (`_vision_yaw_floor`, mirrors `heading_lock._lock_floor` / `motion_yaw._yaw_floor`) so any yaw-axis align eases in instead of relay-slamming — correct-in-general, though it does not drive the *terminal* (yaw-dropped) path.
- **Pitch/roll:** owned entirely by ArduSub's stabilizer — an `ATC_*` tune + physical trim/ballast matter, not our loop. (A calmer depth slew may quiet pitch as a side effect on a 6dof frame with unbalanced vertical thrusters.)

### D13. Vision verb dropped into an autonomous fallback SEARCH *during* a camera switch — **FIXED 2026-07-10 (pool feedback)**
- **Files:** `duburi_dsl.py` (`_wait_detector_warm` + constants), `vision_dsl.py` (`_orchestrate` warm-gate; `align`/`move` reorder), `display_node.py` (switch indicator), `test_vision_dsl.py`.
- **The bug (mission-critical, and SILENT):** right after a camera/model/class switch the target detector is briefly **cold** — it has not yet produced a `/detections` frame under the new config. A vision verb sent immediately has its acquire clock start on that cold detector; `align_loop`'s `lost_since`/`lost_grace_s` (1.0s) returns **LOST regardless of whether the target was ever seen** (`saw_target` only swaps the *reason string*, not the timing). When a `fallback` is provided, the DSL `_orchestrate` then runs the mission's **search** — so the AUV wanders off mid-mission looking for a target that was about to appear. Invisible in logs because `LOST → fallback` is a *normal* path (no error). Long-standing since the auto-switch landed (`13bb53d`), not a fresh regression. Observed symptom: HUD still on the previous camera while the AUV had already started a fallback search.
- **Scope = DSL-only (this decides the fix layer):** `fallback` is a DSL construct, and `hold_through_loss=(fallback is None)` means a **no-fallback** verb holds through loss and never searches. So the danger is structurally *fallback-only* — the fix is in the DSL, **not** the high-criticality `align_loop` (touching it would be blast radius for a defect that can't fire without a DSL fallback).
- **Fix — warm-gate:** `_orchestrate` now blocks on `DuburiMission._wait_detector_warm(camera)` **before the first goal, only when a `fallback` is set**. It gates on the detector **PRODUCING frames** (the detector publishes every inference tick, empty or not — so a fresh `_det_cache` entry = "alive"), **NOT** on the target being visible. Consequence: a genuine "target simply absent" search is **not** delayed (a warm detector returns on the first pump), only a cold/just-switched detector waits. Covers **both** switch paths (ClassRef set-inside-`align` *and* string `set_model`/`set_classes`-before-`align`) because it gates at the goal boundary, not on how the config was set. Bounded by `_DETECTOR_WARMUP_S` (2.5s); on timeout it **warns** (`detector … still not producing frames after warm-up`) and proceeds. **Tuning tripwire:** if that warn line appears in pool logs, the detector's post-switch first-inference is exceeding 2.5s → **raise `_DETECTOR_WARMUP_S`**, it is not a hardware fault.
- **Fix — reorder (`align`/`move`):** was `_activate_camera` (resume + settle) → `_resolve_target` (program model/classes), so the settle warmed the **old** model and the detector briefly emitted wrong-class boxes. Now `_ensure_detector` → `_resolve_target` (program) → `_activate_camera` (resume + settle) so the settle warms the **new** config and the warm-gate returns instantly.
- **Fix — HUD:** during the gap between a switch and the new camera's first frame the viewer showed the *frozen old frame* ("stuck on forward"). Now shows a `CAMERA → <name> waiting for stream` screen for up to `_SWITCH_WAIT_S` (3s). **Cosmetic only** — the control danger was the fallback (fixed above). If the HUD stays stuck *past* the switch, that is a separate "downward stream never starts" camera bug, not this.
- **Verified:** live ROS-graph proof — cold detector → warm-gate **blocks 2.46s** (no immediate fallback); warm/producing → returns **0.01s** (zero search-case penalty). 5 DSL tests (gate wired when `fallback` / skipped without / program-before-activate order / warm+cold logic); planner 176, vision 53 green. The warm-gate sits *before* the `while` loop, so fallback re-entries don't re-gate — the ≤2.5s cost is paid at most once per verb, only when cold.
- **Mitigation with no code change** (for any existing mission on an older build): `if duburi.wait_for('fire', timeout=3): duburi.vision.align(...)` before the align.

### D14. Single-camera vision missions broke on model identity: `set_model`/ClassRef rejected, and a missing registry model crashed the whole pipeline — **FIXED 2026-07-10 (pool feedback)**
- **Files:** `detector_node.py` (`_model_stem`, `_resolve_model_key`, single-model `_single_model_name`, resilient registry load, `active_model` handler), `model_context.py` (ClassRef carries the STEM), `test_model_context.py`, `test_model_identity.py`.
- **The bug (reproduced live, two failure modes on a single-camera launch):**
  1. **`set_model`/ClassRef rejected in single-model mode.** `vision.launch.py model:=gate_rescue_repair` loads ONE model with no registry. Any mission that switched model — an explicit `set_model('gate_rescue_repair')`/`use(...)` **or** a `ClassRef` target (`duburi.models(...).x.cls`, which auto-`set_model`s) — hit `active_model: no registry loaded (use 'models' param at startup)` and **aborted**, *even though the requested model was the one already loaded*. (`set_classes`/`set_conf` worked — only model-switch was broken.)
  2. **`models:=` registry was all-or-nothing → "no pipeline".** The eager registry load `raise`d `RuntimeError` if **any** model failed, taking the **whole detector node down** (no `/detections`, no OpenCV) — even if other models loaded fine. So a full-competition `models:=gate=…,slalom=…,torpedo=…` **crashed** whenever a task's `.pt` wasn't on the box yet (slalom/torpedo weights absent). Single-model mode loads async and survives; multi-model was eager + fatal — the asymmetry.
- **Root cause (identity fracture):** `ClassRef` carried the **DSL alias** (`duburi.models(alias='stem')` key), not the stem — `ModelHandle` stored `_stem` but never used it. Missions register `duburi.models(gate='gate_flare_medium_100ep')` / `robosub=('gate_rescue_repair',…)` where **alias ≠ stem**, so `set_model('<alias>')` named something no detector knew. The stem is the one identity both sides share (it's what actually loads).
- **Fix — stem is the universal model identity:**
  - `ClassRef` now carries the **stem** (`_stem`), so `_resolve_target` sends `set_model('<stem>')`. Only consumer was `vision_dsl.py:_resolve_target`.
  - **Single-model** detector records `_single_model_name` (basename-stem of `model_path`); `set_model(<that stem>)` is a **no-op SUCCESS**, any other name a **clear reject** (`single-model launch loaded 'X'; … relaunch with model:=Y`) — no `'registry'` word, so the DSL surfaces this message directly.
  - **Registry** detector builds a `stem → key` index; `set_model` accepts **KEY OR STEM** (`_resolve_model_key`), so `use('gate')` (alias key) *and* `set_model('gate_rescue_repair')` (stem/ClassRef) both switch. Stem→key collision = last-wins + warn.
  - **Registry load is resilient:** a failed model is **SKIPPED with a loud ERROR** (`… FAILED — SKIPPED … loaded: […]`), fatal **only** if the registry ends empty. If the *startup* `active_model` was the one that failed, it falls back to a loaded model with an unmissable ERROR (never a silent wrong-model substitution).
- **Verified (live detector, CUDA):** single-model `set_model('gate_rescue_repair')`→success, `set_model('slalom_red_pipe')`→clear reject, `resume_detector`/`set_classes`/`set_conf`→success; **real `ClassRef('gate_rescue_repair'/'gate')` through `_resolve_target`**→OK. Registry-with-missing-model→**node UP + `/detections` present**; `set_model` by KEY *and* STEM both succeed; skipped model rejects clearly. 10 new unit tests; planner 181, vision 53 green.
- **No `models:=` needed for a single-model pool run:** `vision.launch.py model:=<stem>` + a mission that refers to `<stem>` (ClassRef or `set_model`) now Just Works. Use `models:=` only for genuine mid-mission *multi*-model switching; prefer **bare stems** (`models:=gate_rescue_repair,slalom_red_pipe`) so key==stem.
- **Dual is the same code path — verified.** Node-routed `set_model('<stem>', node=_FWD/_DWN)` and `ClassRef(camera=…)` land on the right detector; a cross-request (forward asked for the downward model) rejects. "Dual worked before this fix" was launch-config, not code: dual launched with a bare-stem `fwd_models:=` registry (key==stem) dodged both bugs; the **default** dual single-model-per-cam config would have hit Bug 1 too — now fixed for every launch form.
- **Partial-weights competition run (know before the pool):** the resilient registry brings the pipeline **up on whatever loaded** (e.g. gate alone), so gate scores. But a mission that later *reaches* an unloaded task and calls `set_model('slalom_red_pipe')` raises out of `_resolve_target` — the mission runner then aborts (loud, safe: release+stop+disarm) **at that task**, so tasks *after* it don't run. Net vs. before: pre-fix the missing `.pt` crashed the detector at launch so **nothing** scored; now everything up to the missing model does. To run a partial set cleanly, launch with only the models you have.

### D15. Silent-bug audit (control/vision/planner) — `.engine` verified; failsafe disarm + per-model-conf fixed — **2026-07-10**
Follow-up audit for the *same class* of hidden bug as D14 (config-dependent silent divergence, identity mismatch, all-or-nothing). `.engine` (TensorRT) was verified, three fixes landed, the rest documented.

- **`.engine` (TensorRT) — VERIFIED, works with the D14 identity fix.** `yolo._resolve_model_path` prefers `<stem>.engine` over `.pt`; identity is the extension-less **stem**, so `model:=`/`models:=`/ClassRef/`set_model('<stem>')` all match a `.engine`. Live `conf`/`iou`/`max_det` tuning **DOES** take effect on an engine (passed to `predict()` each call + post-inference class filter — NOT baked). Class labels come from the sidecar `<stem>.yaml` (`_load_class_index`), preferred over embedded names, SAME path for `.pt`/`.engine`; the present sidecars (`gate_rescue_repair`, `bin_fire_blood`) match their models' class order (verified). **Only `imgsz`/`half` are export-baked** — `imgsz:=` silently only rescales the `.pt` fallback; re-export the engine to change it. **Silent risk:** a `.engine` with NO sidecar AND no embedded `names` → class indices → empty allowlist → detector returns `[]` every frame (warned once). Mitigation: **keep each `<stem>.yaml` sidecar beside the exported `<stem>.engine`.**
- **FIXED — F1: mid-command FS_PILOT disarm on a long `set_depth`/`surface` (field-observed).** `_command_scope` pauses the 5 Hz heartbeat for the whole command; `wait_for_depth` streamed only `SET_POSITION_TARGET` (no `RC_CHANNELS_OVERRIDE`). With **no heading-lock** active (a lock's Ch4 stream would otherwise feed the failsafe), RC went silent for the hold → ArduSub `FS_PILOT_INPUT` (timeout 3 s) disarmed mid-command — worst on a 60 s emergency `surface()`. **Fix:** `wait_for_depth` now streams a lock-aware `depth_keepalive` each tick (`motion_writers.make_writers`): no-lock → `send_rc_override(throttle=NO_OVERRIDE)` (the heartbeat's all-neutral frame **minus Ch3**, 5 channels feed FS_PILOT); lock → `send_rc_translation(throttle=NO_OVERRIDE)` (Ch4 left to the lock). **Ch3 is RELEASED** so ALT_HOLD's position controller still drives to the setpoint — depth is REACHED *and* the failsafe stays fed. This mirrors the pool-proven vision pattern ("Release Ch3 to ALT_HOLD whenever depth is in play", motion_vision) — vision streams the same `SET_POSITION_TARGET` with Ch3 released for 20 s and reaches target, so depth-reaching is proven by parity. Files: `motion_writers.py` (`depth_keepalive`), `motion_depth.py` (`wait_for_depth`/`hold_depth` `keepalive=`), `duburi.py` (`set_depth` + style dives). **Bench gate (on-vehicle confirmation):** a 30 s `set_depth` and a 60 s `surface()` with NO lock active must **reach depth / ascend to ~0 AND not disarm**. Frame: closes the most likely disarm mechanism; if disarms persist, cause is elsewhere (battery sag / tether / EKF).
- **FIXED — F2: `set_conf(model='<stem>')` silently no-op'd (the D14 identity-bug class, unfixed spot).** `_apply_model_conf` did an exact-key registry lookup; unlike the `active_model` handler it never used `_resolve_model_key`/`_single_model_name` (its docstring even claimed "or its stem"). So a per-model conf tighten dropped on a single-model launch (`_active_name` is None) and an aliased registry — e.g. `demo_dual_camera` running the torpedo model tight to stop a spurious box winning the terminal lock. **Fix:** resolve by KEY or STEM in registry mode, match `_single_model_name` in single mode. Live-verified: `set_conf(model='gate_rescue_repair')` now applies. **Safe on the competition bare-stem dual registry** (key==stem).
- **FIXED — F3 (diagnostic only): silent low-FPS translation stall.** `_freshness` zeroes lat/fwd for a live bbox 0.4–1.0 s old while NOT declaring LOST (no fallback fires) → the hull barely translates yet never searches, a mystery TIMEOUT at low detector FPS. **Mitigated by `.engine`** (20-30 Hz → fresh≈1.0). Added a throttled `[VIS ] low detector FPS: lat/fwd authority N%` line (`_warn_low_fps`) at both align/move freshness sites — pure observability, no behaviour change.
- **DOCUMENTED, not fixed (imperative path unaffected / phase-2):**
  - **VF1 (operator P0, not code):** `slalom_red_pipe` / `torpedo_blood_hole` / `octagon` have committed `.yaml` sidecars but **no `.pt` and no `.engine` in the tree** — those tasks cannot run and engines can't be exported without the source weights. **Get the weights on the Jetson + export engines before the run.** (Loud at `set_model` time; still the single biggest task risk.)
  - **VF6:** `vision_dual.launch.py` default `dwn_device=4` vs `config.py` `downward` device `2` — with two identical USB cameras the int index is unreliable; **use the by-path `dwn_device_path`/`fwd_device_path` symlinks** on pool day (the right int is unknowable from code).
  - **C1:** raw CLI `vision_move --fwd_fill 0` → 0==unset makes it a 95% fill-stop, not pass-through; use `--fwd_fill -1`. **DSL `move(fwd=None)` is SAFE** (sends −1).
  - **H2:** a no-yaw `vision_align` with no active lock releases Ch4 → ArduSub's untrusted compass owns heading → slow drift. Mission rule: `lock_heading` before a lat/depth-only align.
  - **FSM (phase-2, NOT run — imperative `detected()` is the competition path):** `SetDetectorState` bypasses `use_camera()` (no pause/resume handoff → can't do a forward→downward dual run); a rejected `set_model` ABORTs the whole FSM run (should skip the task); bin search times out on the default `paused:=true` dual launch. Fix when phase-2 FSM is activated, mirroring the imperative `use_camera` + per-chunk try/continue.

---


### Pool-day 1 audit (2026-07) — FIXED

### P1. Arm intermittently reports "doesn't arm, 12 s crossed" — **FIXED 2026-07**
`DuburiClient._result_deadline` gave `arm`/`disarm`/`mission_reset` a **fixed 12 s** result
backstop (`_QUICK_CMDS`), but their real server budgets are larger — **arm ~18 s** (3 s ACK +
15 s `is_armed()` poll), **disarm ~26 s**. `send()` never applies the `COMMANDS` defaults, so
`goal.timeout` is `0.0` on the wire; the client raised `MoveTimeout` and cancelled the goal
**before arming completed** (worst on a slow post-baro-rezero EKF arm-readiness — the "sometimes").
**Fix:** the quick deadline is now a **floor, not a ceiling** — it reads the effective budget from
the same registry the server enforces: `stop/surface/unlock` keep 12 s; **arm → 30 s, disarm → 35 s**.
The disarm emergency-bail floor is preserved (bounded, just covers its real budget). `client.py`,
`test_client.py`.

### P2. Aborting an arm could strand the hull ARMED (fail-open) — **FIXED 2026-07**
`pixhawk.arm()` gained an abort hook (a goal cancel mid-arm must not leave a hull that arms a beat
later — ArduSub runs pre-arm checks **after** the ACK, so `is_armed()` reads False while the arm is
still pending). Two review findings hardened it: (a) arm now **clears the abort slate at entry** so a
STALE abort from a prior cancelled command can't insta-abort + disarm a fresh arm; (b) the abort
branch does a **verified** disarm (`_disarm_after_abort` re-sends DISARM across a window and requires
the disarmed state to HOLD) — **fail-closed** with a distinct `ABORTED_DISARM_UNCONFIRMED` reason if
it can't confirm, so no caller assumes "safe" on an unverified state. `NOT_ARMED_AFTER_ACK` also now
appends ArduSub's STATUSTEXT pre-arm reason. `pixhawk.py`, `duburi.py`, `test_pixhawk_helpers.py`.

### P3. Distance estimator absent → `calc_distance('stop')` returned a phantom 0.0 m — **FIXED 2026-07**
After the latched-topic refactor, `DistanceState.start()/stop()` always returned success even when
`distance_estimation_node` wasn't running (fire-and-forget, no ack), so a distance-gated move would
trust a clean 0.0 m. `stop()` now verifies a `distance_traveled` sample arrived **after** the bracket
started; absent → `success=False` with a clear reason. Latent today (no shipped mission gates on it).
`distance_state.py`, `test_distance_state.py`. *(distance/optical-flow is experimental, off by default.)*

### P4. Bin task (downward camera) dives to the pool floor, ignoring `set_depth` — **FIXED 2026-07-15 (pool feedback)**
A **downward SURGE-only** `vision.align` (the bin task: `lat` + `fwd`→Ch5 surge, **no** fill→depth
descent) **released Ch3 throttle (65535) while streaming NO depth setpoint**. With nothing asserting
depth-hold, ArduSub had no Ch3 authority to hold on, and the negatively-buoyant hull **sank to the
floor** — armed + ALT_HOLD, `set_depth` effectively ignored. `align_loop` streamed the depth setpoint
only on a downward *descent* (`stream_depth = use_vdepth or (downward and use_fwd)`); a surge-only align
had `use_fwd=False` → no stream, bare Ch3 release. The shipped `task_bin.py` hit this too.
**Fix (`motion_vision.py`):** stream the depth setpoint on **any** downward align
(`stream_depth = use_vdepth or downward`); with no descent, `fill_deficit` stays 0 so the 5 Hz block
re-streams the **constant** captured depth (holds `set_depth` via ArduSub's position controller — the
**same proven mechanism** the forward torpedo-standoff depth axis uses), and Ch3 is released
(`65535 if stream_depth else 1500`) so ArduSub's depth PID is the sole Ch3 consumer. The loss branch
re-asserts the hold (`if stream_depth`). Forward paths byte-unchanged; mavlink-reviewer clean.
`motion_vision.py`, `test_motion_vision.py` (4 new tests).

**Related trap — the depth-bound knobs are not tunable the way it looks.** `vision.max_depth_m` /
`vision.depth_ceiling` are (a) **overridden by per-call `align(max_depth_m=…, depth_ceiling=…)` kwargs**
(a per-call value beats any `ros2 param set`), and (b) re-set every run by a mission's
`set_vision_param(...)`. They are also **NEGATIVE metres** — a POSITIVE value (e.g. `0.6`) is a sign
error that silently reads as OFF (now warned loudly). They only bound the optional fill→depth **descent**;
they are **not needed to hold depth** — plain `set_depth` holds after this fix. **Build note:**
`build_dubomini.sh` is a plain `colcon build` (no `--symlink-install`), so editing
`competition_config.py`/`vision_tunables.py`/missions requires **`./build_dubomini.sh` then restart** —
a node restart alone does nothing.

---


### D16 — a Hailo stream abort leaves the detector a ZOMBIE (FIXED, 2026-09-06)

**Observed on the vehicle.** Restarting the stack while a previous detector
still held the `VDevice` put the chip into `HAILO_STREAM_ABORT(63)`. The
detector node then:

* stayed **alive** — `pgrep` sees it, the node is listed, its subscriptions are
  up;
* logged `inference failed: HailoRTStreamAborted` on **every frame**, forever;
* published **zero detections**, indefinitely, with no escalation.

```
[ERROR] [DET  ] inference failed: HailoRTStreamAborted('Stream was aborted')
[HailoRT] [error] ... pipeline status is HAILO_STREAM_ABORT(63).
```

**Why it matters more than the error itself.** Every liveness check we have
passes: the process exists, the topics exist, the graph looks correct. Only the
detection RATE reveals it, and nothing was watching the rate. This is the
"absence is not zero" family again — a subsystem that has stopped working while
continuing to exist.

**Recovery today is a manual restart**, and the stack does come back cleanly
(299 inferences/interval, frame age 42.2 ms mean).

**FIXED.** Three tiers in `detector_node._on_infer_failure`, because the
failures are not one thing:

1. **isolated** failures are tolerated — a bad frame is a bad frame, and
   killing the node for one is worse than the fault;
2. **15 consecutive** means the DEVICE is gone, not the frame → rebuild the
   detector through the same construction path that built it (dropping the old
   one first: on the Hailo path the device is held by the object, and a second
   `VDevice` while the first lives is `HAILO_OUT_OF_PHYSICAL_DEVICES`);
3. **45 consecutive**, i.e. the rebuild did not help → `os._exit(1)` so a
   supervisor restarts the process.

Thresholds are in FRAMES, not seconds, so they behave the same at 3 Hz and at
80. Any success resets the counter, so a chip that recovers by itself never
reaches tier 2.

**A test lesson worth keeping.** The first test set drove `_on_infer_failure`
directly and passed while two injected defects went undetected — deleting the
LOOP's call to it, and deleting the success reset. The escalation was tested
and the WIRING was not, which is the same shape as a guard that greps the
source. Tests that run `_infer_loop` itself now cover both, and all four
injections are caught.

**Related:** the same restart also logged
`detector init FAILED: Failure in hailort driver ioctl` on the first attempt
and succeeded on the retry, so VDevice contention is transient but real. One
process, one `VDevice` (`detection/hailo.py:158-176`) is still the rule.

---

## 10. Appendix — SROT pre-dive gates (migrated 2026-09-08)

> Migrated verbatim from the retired `srot-pre-dive-gates.md`.
> **This is a live safety interlock, not history.** GATE 0/1/2 gate every
> AUTO move, `move_forward` included — `SROT_MOVE` enters AUTO and the AUTO
> branch closes the depth loop under every primitive. Referenced by
> `tools/srot_axis_snapshot.py` and `srot-integration.md`.

# SROT pre-dive gates — final review before water, 2026-08-07

> Firmware on the vehicle: `srot-control-board` @ `3bd247b`, **`SROT_FW_BEHAVIOUR_REV 9`**,
> flashed and confirmed over USB. 232 params re-read after the flash: **zero drift, zero
> missing**. `CAL_MAG_*`, `CAL_LVL_*` and `FS_GCS_COMPID = 191` all intact.
>
> Read this with [`srot-integration.md`](srot-integration.md). This file is only the
> go/no-go list for the next in-water session.

---

### ⛔ GATE 0 — the axis configuration is UNKNOWN. Settle it first.

**This supersedes the "config is correct" conclusion in `srot-integration.md` §8.2.** That
block was written against `CAL_MDIR = [+1]×8` and no longer describes the board.

`MOT_n_DIRECTION` and `CAL_MDIR` **multiply** (`task_control_loop.cpp:140-145`). Read off
the vehicle 2026-08-07:

```
MOT_n_DIRECTION : [-1, +1, +1, +1, +1, +1, +1, -1]
CAL_MDIR        : [-1, +1, +1, +1, +1, +1, +1, -1]     <-- NOT all +1
effective       : [+1, +1, +1, +1, +1, +1, +1, +1]
```

`CAL_MDIR` changed **between 10:02 and 10:27 on 2026-08-07** (snapshot-diffed). Nobody has
claimed the write. It was `[+1]×8` before and has been `[-1,+1,+1,+1,+1,+1,+1,-1]` since.

### Why this is a gate and not a note

With the effective product, the earlier derivation's conclusion **inverts**:

```
w = [-1,+1,+1,+1,+1,+1,+1,-1]           (IF M1/M8 are the backwards-wired pair)
s_net = -(w · effective_dir)
      = -(w · [+1]*8) = [+1,-1,-1,-1,-1,-1,-1,+1]      NON-UNIFORM
```

M1/M8 out of step with M2–M7 — the **2026-08-06 yaw-spin fault**, pushed through the yaw
column `[+1,-1,-1,+1]` it collapses to three thrusters pushing the same way.

**But this is NOT a claim that the vehicle is broken.** It depends entirely on `w`, the
true wiring, and **parameters cannot tell us `w`**. If all eight are in fact wired correctly
(`w = [+1]×8`), then effective `[+1]×8` gives `s_net = [-1]×8` — uniform, and fine. Both
readings of the same numbers are self-consistent. **Only physical thrust separates them.**

### How to settle it

Either is sufficient, and one of them **must** run before any armed manoeuvre:

1. **Per-thruster physical check**, props on, hull restrained, one motor at a time — confirm
   each thruster pushes the direction `docs/THRUSTER_MAP.md` says it should.
2. **In-water MOTOR_DETECT, armed, hull free to rotate** — this is gate 3 anyway. Require
   **SUCCESS**; a `FAIL` is inconclusive and writes nothing, so re-run rather than editing
   params by hand.

Until one passes, **treat every axis as unverified**.

---

### GATE 1 — MOTOR_DETECT, then `FRAME_REVERSE = 0`, in that order

Adopted in `fd701fc` (PR #6) and unchanged. Restating because gate 0 makes it live.

The detect run itself is safe — `driveTestMotor()` never reaches `mixer::mix()`, so
`FRAME_REVERSE` cannot contaminate what it measures. The problem is what detect **converges
to**: every thruster ends up agreeing with its mixer column (`s = +1` uniformly), and
`FRAME_REVERSE = 1` then negates all six demands and turns that corrected frame back into a
uniform `[-1]×8` flip.

```
1. record   MOT_1..8_DIRECTION, CAL_MDIR1..8, FRAME_REVERSE      (before arming)
2. detect   in water, armed, free to rotate -- require SUCCESS
3. set      FRAME_REVERSE = 0     and save
4. verify   MANUAL axes, then STABILIZE, then autotune
```

Wait for the `"Params saved to flash"` statustext at step 3, **not** the `COMMAND_ACK` — the
ACK fires the instant the command is parsed and the NVS write is deferred. Match on
`"Params saved"`, not `"saved"`: the board emits the *calibration* line first.

If axes are still inverted at `FRAME_REVERSE = 0` after a SUCCESS detect, the inversion is
**not** in the motor directions. Look at the attitude/gyro convention, and do not simply put
`FRAME_REVERSE` back — that hides which of the two is wrong.

---

### GATE 2 — the depth loop has still never run closed

Unchanged and still the hard blocker. It gates **every `SROT_MOVE`**
(`task_control_loop.cpp:240-241`), including `move_forward`. An in-air `move_forward` is not
partial validation.

Rev 9 makes the disarmed half honest rather than fixing this: `DEPTH_ERR`/`DEPTH_OUT` are now
**absent** while the controller is not running, instead of streaming a frozen register.

✅ **DONE (round 26) — this section described work that is now built.**
`check_depth_loop_settled()` reads **`DEPTH_CMD`** and derives its threshold from
`DEPTH_P`, exactly as specified below. The drift test then caught a second bug in
it: our `DEPTH_P` fallback was a stale **3.0** against a board running **0.5**,
so the guard divided every reading by 6× too much and a barometer 0.90 m off at
the surface — three times the limit — back-converted to 0.20 m and **armed**. It
failed OPEN, on the one interlock between a phantom baro and full vertical
thrust. `sp.DEPTH_P_DEFAULT` is 0.5 now and the arithmetic is pinned by
`test_srot_safety.py`.

The in-water half of GATE 2 is still open. The rationale below is kept because it
is the reasoning the implementation follows:

- Same `±1.0` clamp, so the 2026-08-02 phantom-baro case (`−3…−6.7 m` → `3.0 × −3.1`)
  still pins at **−1.00**.
- **Better than the old signal**: `preview()` is proportional-only with no integrator, so it
  reflects the *current* baro sample rather than accumulated windup. For "is the baro lying
  right now", that is the property you want.
- ⚠ **Derive the threshold from `DEPTH_P`, don't hardcode `0.90`.** The mapping is
  `|DEPTH_CMD| ≥ 0.90 ⟺ |depth − 0.10| ≥ 0.30 m` **at `DEPTH_P = 3.0`**, and it moves with
  the gain. A healthy surface reading (`depth ≈ 0.026`) gives `≈ −0.22`, comfortably clear.

---

### GATE 2b — configuration, found by reading the firmware (2026-09-03)

Three parameter states that are not defects but will bite in water. All read off
the live board.

**`LEAK_EN = 0.0`** — and that is the FIRMWARE DEFAULT, not just our board's
setting (`params.cpp`). It gates the leak failsafe **and** the pre-arm refusal,
so as configured a leak neither blocks arming nor surfaces the hull. Verified
safe to enable: with `LEAK_EN = 1` the sensor reads **DRY**, so it will not cause
spurious refusals. The only honest read is
`SYS_STATUS.onboard_control_sensors_enabled_extended`, which is what
`srot_fc.sys_status_leak()` keys on — it returns **None** when nothing is
watching, never `False`.

**`FS_GCS_COMPID` must be 191.** Bondor's bench-mode button writes **0**
(wildcard) and its own banner warns *"do NOT press Save on the Parameters tab
while it is active, or it becomes permanent"* — while its payload workflow tells
operators to press Save. A permanent 0 means **a dead companion is
indistinguishable from a live Bondor**, and the vehicle station-keeps at depth
instead of surfacing. Add it to the preflight read.

**`MOT_BAT_V_MAX = 0`** with `ESPNOW_EN = 1`: the battery feedforward is
available and switched off, so a timed leg travels further on a full pack than a
flat one. Their own doc calls this "usually the better first move — it needs no
water and no learning period". Setting it is a tuning decision, not a fix.

---

### GATE 4 — do not share UDP 14550 with Bondor

Measured on their side, and the danger runs the opposite way to the intuition:
both processes bind with `SO_REUSEADDR`, both binds succeed, and **the newcomer
takes the stream** — 544 datagrams in 6 s while the incumbent got a trickle.
Bondor does not go blind when it opens on top of a running mission; it works
perfectly and silently starves the companion of telemetry and command ACKs, with
nothing in either UI to say why.

USB serial is single-owner, so there the conflict is loud. UDP is the quiet one.

---

### GATE 3 — `FRAME_REVERSE` under thrust

Still unverified. MANUAL stick direction first, STABILIZE second, autotune last. Gate 0 has
to clear before this means anything.

---

### Verified, no action needed

| item | state |
|---|---|
| `YAW_REF` | **2 = LOCKED** at `MAGACC 2` — heading is absolute, absolute `MOVE_TURN` is safe |
| `DEPTH_ERR`/`DEPTH_OUT` disarmed | **absent** (rev 9, as specified) |
| `DEPTH_CMD` | live, `−0.648` |
| `FS_GCS_COMPID` | **191**, in flash, survived the flash |
| params after flash | 232 read, **0 drift, 0 missing** |
| CFG banner | repeats ~60 s; caught gate 0 on its first run |
| wildcard warning | correctly silent (`COMPID = 191`, not `0`) |

### NOT verified — needs arming, therefore an operator decision

- `DEPTH_ERR`/`DEPTH_OUT` **reappearing** when armed in DEPTH_HOLD. Rev 9's suppression is
  confirmed; the un-suppression is not. **Props off.**
- The `companionLost()` seen-then-lost latch on hardware.

---

### Order of operations for the session

1. **Gate 0** — physical thrust check *or* in-water MOTOR_DETECT (SUCCESS).
2. **Gate 1** — `FRAME_REVERSE = 0`, saved and confirmed by statustext.
3. Armed, props off: confirm `DEPTH_ERR`/`DEPTH_OUT` reappear in DEPTH_HOLD.
4. **Gate 2** — depth hold closed-loop, in water.
5. **Gate 3** — MANUAL axes → STABILIZE → autotune.
6. Only then: `SROT_MOVE` primitives.

Nothing in steps 4–6 is meaningful until 1 and 2 have passed.

---

## 11. Appendix — what we owe the water (migrated 2026-09-08)

> Migrated verbatim from the retired `water-owed.md`. This is a
> **measurement backlog**, not a defect list: things shipped on dry
> evidence that need an in-water number before they can be trusted.
> Referenced by `tools/flow_derot_ab.py` and `measured-bars.md`.

# What we owe the water — the measurement backlog

> **This is not `pool-day.md`.** That file is the *runbook*: how to bring the
> vehicle up, what to check, how to recover. **This file is the debt**: every
> quantity that is shipped, wired, or claimed and that **only water can
> settle**, with the procedure, the pass bar, and what each one unblocks.
>
> An item leaves this file by being **measured**, not by being believed.
> Written 2026-09-07, srot `b0f5404`. Keep it current — an item that quietly
> disappears is how a dry number becomes a competition assumption.

### Why a separate file

Rounds 33–38 shipped a great deal of behaviour and **every headline number is
dry**. That is not a complaint about the work; it is the honest boundary of a
bench. Three things are true at once and they must not be blurred:

- **Measured in air, on this hardware** — real, reproducible, and possibly
  irrelevant underwater (refraction, turbidity, texture, lighting all change).
- **Measured in a simulator** — settles logic, never optics or transport.
- **Shipped and never measured anywhere** — the dangerous class, because a
  default that nobody chose still multiplies every output.

The bars below are written so a pool session produces **numbers**, not
impressions. Where a bar cannot be met the item says what to do instead,
because "it looked fine" is how `recommend()`, CLAHE and the 25 s gap all got
into the record and had to be retracted.

---

### ⛔ PRE-WATER GATES — do these before the hull is wet

These are not measurements. They are conditions under which a run is unsafe or
uninformative, and every one is verified-false today.

| # | gate | state | why it blocks |
|---|---|---|---|
| **G1** | **`LEAK_EN = 0` on the board** | **verified 0** | Both the leak failsafe and the pre-arm refusal are gated on it. As configured a leak **neither blocks arming nor surfaces the hull**. The sensor reads DRY, so enabling it looks safe — that is exactly why it was never noticed. **Set `LEAK_EN = 1` and confirm the pre-arm refusal actually fires.** |
| **G2** | **`FRAME_REVERSE = 1` + `MOT_n_DIRECTION = -1` on M1/M8** | set, **runtime-only, unsaved** | A double inversion nobody has resolved. `CAL_MDIRn` **multiplies** with `MOT_n_DIRECTION`, so a successful MOTOR_DETECT makes `FRAME_REVERSE` wrong. **Read all three back and write them down before arming.** |
| **G3** | **GATE 0 — axis configuration** | **never verified** | Needs physical thrust, one motor at a time, hull restrained. `DO_MOTOR_TEST` (209) is fully implemented in firmware and runnable from `duburi_ws` today. **Until this is done every axis sign is an assumption**, including the ones the flow verification below depends on. |
| **G4** | **GATE 2 — depth loop closed** | Bar30 healthy, loop *running*, never closed in water | It gates every `SROT_MOVE`. The host half is done. |
| **G5** | **`MOT_BAT_V_MAX`** | unset | Without it a timed leg is **pack-state dependent** — the same command travels different distances at 16 V and 13 V (measured: 14.2 % less). |

---

### 1. ⭐ Flow-as-DVL in water — the biggest single unknown

**Status:** verified in **air** to 3.6 % worst-case error over 30 cm on three
axes (`measured-bars.md` §13). Every constant in that path was measured in air.

### 1a. `f_water = 741` — validate, do not re-calibrate

`f_air = 513.94`, `f_water = 741.0`, ratio **1.442**, from the round-25
held-out-validated FOV work (63.8° air / 46.7° water ±0.7°).

> ⛔ **CORRECTION (2026-09-07). Two claims in the previous version of this
> paragraph were wrong, and they were wrong in the direction that made the
> water number look better supported than it is.**
>
> 1. **"The literature says 25–33 %; ours is 44 % — the measurement wins."**
>    There is no discrepancy. `f_water/f_air` is a ratio of **tangents**;
>    the refractive index is a ratio of **sines**. Our own pair implies
>    `sin(31.9°)/sin(23.36°)` = **1.3333** — plain Snell for water, to four
>    decimals. The two numbers were never in conflict; they are different
>    quantities.
> 2. **`f_water` was never measured.** It is `f_air` put through that same
>    Snell conversion (`fov_from_K` derives the water figure from the air
>    fit). Predicting 46.71° from 63.8° and then citing 46.7° as a measured
>    result is circular. Only the **air** half of that pair is a
>    measurement.
>
> So the flat-port refraction model is, as of today, **assumed and
> unvalidated** — which is exactly what makes the item below worth doing,
> and it is a stronger reason than the one originally written here.

- **Do:** a known-length slide **in water** at a known height, `medium:=water`.
- **Bar:** measured distance within **±5 %** of tape. A systematic error near
  **−31 %** means `f_air` is being used where `f_water` belongs
  (`514/741 = 0.694`). There is no separate "1.33 guess" failure to look for:
  our `f_water` **is** the 1.333 conversion, per the correction above.
- **Do NOT** re-fit `f_water` from a single slide. If it disagrees by more
  than the bar, that is a finding to investigate, not a number to overwrite
  — one hand slide is far weaker evidence than 25 held-out views.
- **⭐ An in-water CALIBRATION is the real answer, and it is now buildable
  (2026-09-07).** `ros2 run duburi_vision calibrate --medium water`, or the
  WATER button on the page. It fits the intrinsics through the port with the
  board in the pool, so it measures the air-plus-port-plus-water system
  *directly* rather than assuming the model — the first genuinely
  independent number for `f_water`. It installs as
  `<profile>_<w>x<h>_water.json`, **never overwrites the air file**, and the
  launch keeps loading the air one.
  **Reading the result:** if a ≥15-view in-water fit disagrees with the
  Snell prediction by more than the ±0.7° air bar, believe the fit and
  record the port's contribution — a real flat port is what the
  literature's 25–33 % figures describe, and a thin-port ideal (our current
  assumption) omits it.

### 1b. `pool_depth_m` — the parameter that multiplies everything

It defaults to **NaN** and the node **refuses to publish velocity without it**
(`flow_node.py:337`), which is the fix for the class of bug that produced an
83 % scale error from an unset height. Height is a clean multiplier on every
velocity.

- **Do:** measure the actual pool depth with a tape. Set it explicitly. Read
  the startup banner back and confirm the printed value.
- **Bar:** the banner prints what you measured, and `HeightFromDivergence`
  agrees with the `pool_depth_m` path to within its **20 %** warn threshold.
  A persistent disagreement means one of the two is wrong — and in air the
  **optics measured height better than the tape did** (0.7025 m vs 0.720 m),
  so do not assume the tape wins.

### 1c. Drift over a real leg — the number the bench cannot produce

Every distance number we have is a **30 cm hand slide with ~±1 cm of
operator precision**, so the operator sits inside our error bar. The quantity
that decides whether `move_forward_dist` closes over a mission leg is **drift
accumulation over metres**, and it is unmeasured.

- **Do:** known-length legs of **2–3 m** at **2–3 known altitudes**, out and
  back, `move_forward_dist` against a tape.
- **Bar:** report as **% of distance travelled**, the same unit as the
  benchmarks — Nortek DVL bottom-track **0.5–1 %**, DVL-aided INS **0.08 %**,
  Ferrera et al. monocular VO in turbid water **0.89–1.88 % ATE**. Under
  **2 %** is competitive; over **5 %** means the flow path is not yet a DVL.
- **Also record:** the symmetric-leg residual (out then back should return to
  origin). Cross-track heading drift dominated in sim; expect the same.

### 1d. Does the pool floor give features at all?

Tile grout lines are a strong, repeatable feature set — and a tile lattice is
also the documented **perceptual-ambiguity** case, where LK can lock one tile
off with `status=1`. Our forward-backward rejection at 2 px and the planar
rigid fit both exist for this.

- **Do:** log `flow_quality`, surviving point count, and the fit residual over
  a full leg. **Also** point it at a plain (untextured) section of floor.
- **Bar:** on tile, ≥ **40** surviving points and residual < **0.5 px**. On
  plain floor the node must **refuse** (quality 0) rather than emit a
  confident wrong velocity. **A refusal on featureless floor is a PASS.**

### 1e. `ROT_FRACTION_MAX` — re-derive, do not just move it

Currently **0.80**. The dry evidence is two-sided and must not be read
one-sided (`measured-bars.md` §12): at 0.638 rad/s the gate **refused 533 of
670 intervals** while de-rotation was working (85.5 % of truth recovered), and
at 1.128 rad/s de-rotation made the answer **worse**. So 0.80 is too strict at
the low end and a real ceiling exists at the high end.

- **Do:** collect the residual and the rotation fraction together over a leg
  with deliberate yaw.
- **Bar:** replace the fraction test with a **residual-based** criterion. Do
  **not** simply raise 0.80 — the first row alone would justify that and the
  second row says it would be wrong.

---

### 2. ⭐ Camera↔IMU `td` — the one term with no measurement anywhere

Board-clock stamping is now measured on hardware (**sd 6.402 → 0.528 ms, 12.1×**,
`measured-bars.md` §14). The other three corrections are wired, unit-tested,
and **unmeasured on hardware**:

| term | size | state |
|---|---|---|
| interval midpoint | up to **375 ms** | pure arithmetic, unit-tested |
| half-exposure | **7.85 ms**, moves with light | read live, unmeasured |
| **`td`** | **unknown** | every number is injected-offset recovery |

`td` is **bounded at 150 ms** (`time_offset_max_s`) precisely because we have
no in-water value to sanity-check a large estimate against.

- **Do (can be done DRY, needs only a hand):** oscillate the rig gently about
  the optical axis. It must be a **changing** rate — Li & Mourikis show
  constant velocity is degenerate and the estimator will refuse it.
- **Bar:** `quality ≥ 0.5`, and **successive windows agree** to within a few
  ms. Disagreement means the excitation gate is doing its job and the answer
  is not ready — **that is not a failure to work around.**
- **In water additionally:** exposure rises in darker water, so half-exposure
  grows. Log `exposure_us` through the run; if it moves, the offset moves with
  it and that is the term to watch.
- **Fallback:** if no stable `td` is obtained, ship `estimate_time_offset:=false`
  rather than a value nobody trusts.

---

### 3. Vision — five things shipped on dry evidence

| # | item | shipped state | what water decides | bar |
|---|---|---|---|---|
| **3a** | **Adaptive Kalman R** (confidence-driven) | **ON by default** | measured dry on a person in air (selectivity 0.78× → 5.37×, the shipped filter had been *inverted*); the archive agrees, neither is water | selectivity > 1 on real props, and alignment converges **faster**, not merely differently |
| **3b** | **conf floor 0.10** | shipped | §8 precision says 0.10 is defensible and the original **+8.5-presence justification was not** — presence is blind to false positives by construction | recall **and precision** on real props; a false gate detection steers the hull at a wall |
| **3c** | **`ConfidenceTrend`** | shipped, **wired to nothing** | on `bin`, confidence fell 0.53 **before** the box wandered — a genuine early warning if it holds | does confidence lead the error signal in water? If yes, wire it |
| **3d** | **`vision.ctrl_conf`** | `ConfidenceModel.authority()` implemented, **called by nothing** | it moves thrusters, which is why it is not enabled dry | enable only after 3a and 3b hold |
| **3e** | **`range_gain_floor`** | measured, **still off (1.0)** | the close-in instability lever: loop gain rises ~1/range, so a `kp` stable far-field over-drives close in | does the measured jitter-vs-fill curve cancel? This is the terminal-alignment fix |

---

### 4. Control — closed-loop gains

`kp_lat`, `kp_yaw`, `ki_lat` **need water and the replay rig deliberately does
not pretend otherwise.** Archived footage can prove sign, deadband, freshness
decay, per-axis cap *ratios* and the fire gate — it **cannot** prove
convergence, overshoot, settle time or `kp` magnitude, because the image does
not respond to the command.

Also unmeasured in water: **`PILOT_EXPO = 0.30` is inside our loop.** The board
applies `((1−EXPO)·sp + EXPO·sp³)·PILOT_SPEED` to translation *and* yaw, so
small-signal authority is `GAIN·0.70`, never 1.0, plus a cubic term that makes
one `kp` wrong at both ends — exactly the terminal-alignment regime.

- **Bar:** tune with `GAIN` **read from the board**, not assumed. And remember
  `set_default_gain()` is a **no-op for its own session** (latch at boot); the
  proven in-session path is pulsing `JS_GAIN_INC = 42`.

---

### 5. The return leg — one command, highest consequence

Every 2025 gate model scores **0.0 %** on the 302 labelled back-side images.
Production `gate_rescue_repair` declares `gate / rescue / repair` — **no
back-side class**. `tools/return_check.py` is written and **unrun**; the
weights are on the Pi.

> **RUN 2026-09-07 — and the finding above is wrong in three places.** See
> `measured-bars.md` §15. Short version: the production model is **not blind**
> (41.7 % at conf 0.10, held out) but is **~26 % at the shipped operating
> point** — a coin flip, which fails on the day rather than loudly. Only the
> older SAUVC-family models score a true 0.0 %. It is **302 images and 72
> labels**, not 302 labelled. And the archived back-side specialist's 88.9 %
> is **memorisation** — its `data.yaml` says `val: train`, so it was scored on
> its own training set and that number must not be deployed on.

**What the pool owes this item:** point the forward camera at the gate **from
the far side** and record a full pass. We have 72 labelled back-side frames
from one session; that is not enough to decide anything, and it is the
scarcest asset in the archive for the one scoring task nobody has measured.

- **Bar:** enough frames from a **second** session to build a held-out split.
  Never a random split — these are consecutive video frames and neighbours are
  near-duplicates, which is exactly how the specialist came to score 98.6 % on
  nothing.

---

### 5b. The commands, verified on the vehicle 2026-09-07

Not written from the launch file — **run on the Pi and the output pasted
below**, because the whole of §17 in `measured-bars.md` is about a
configuration that was correct in a tool and wrong in the launch.

```bash
# The DVL. flow:=true is OFF by default and pool_depth_m has NO default.
ros2 launch duburi_vision vision_pi.launch.py flow:=true pool_depth_m:=1.6
```

Expected banner, and check every field of it:

```
[FLOW ] camera='downward' medium='water' f=685.1px (air 513.9 / water 741.0)
        port=RECTIFIED  pool_depth=1.60m  gyro_gain=(-1.000,-1.000)
```

- **`port=RECTIFIED`** — the flat-port correction is live. `single-f` means the
  calibration did not load, and you are carrying a 1–2.8 % anisotropic scale
  error (§14).
- **`f=685.1px`** — the rectified reference focal length. Seeing **741** means
  `port=single-f`.
- **`pool_depth=1.60m`** — what you measured with a tape. `nan` prints
  `VELOCITY PATH DISABLED` and nothing is published.

Both refusal paths were exercised live and both are worth recognising:

```
[FLOW ] VELOCITY PATH DISABLED -- pool_depth_m was never set.
[FLOW ] REFUSING: no trackable texture (0/19 points survived)
        -- dark or featureless floor, not a tracking fault
[FLOW ] REFUSING: LK lost the anchor (2/12 survived)
```

The middle one is the murky-water message and it is **not a bug report**: it
means the floor gave nothing to track. The last one means tracking started and
degraded. Telling them apart at 2 a.m. is the reason they are separate lines.

---

### 6. Session logistics — so the data survives

- **`.tlog` recorder on for every run** (round 27). It captures both
  directions; a reader-only log has no *decisions* in it.
- **`source scripts/pool_session.sh <name>`** in every terminal — pins
  `DUBURI_RUN_DIR` and `ROS_LOG_DIR` to one folder per run.
- **Record `image_raw`** for at least one run per venue. `pool_record.sh`
  **excludes it by default**, and without frames no offline A/B is possible —
  bag playback is wall-clock paced and drops a different frame subset each
  time, so it cannot be an A/B rig.
- **Write down the water**: turbidity, lighting, time of day, and a Laplacian
  sharpness number. Our archive shows the gate clip is **3.7× blurrier** than
  the bin clip at identical brightness, and that was the variable that
  mattered — not the model.
- **One variable per arm.** The CLAHE retraction happened because a tracker
  fix, a clamp fix and a conf drop were stacked into one arm.

---

### Ledger — everything blocked on water, one line each

| # | item | owner section |
|---|---|---|
| 1 | `LEAK_EN = 1` and the pre-arm refusal fires | G1 |
| 2 | `FRAME_REVERSE` / `MOT_n_DIRECTION` read back and recorded | G2 |
| 3 | GATE 0 — per-axis thrust direction via `DO_MOTOR_TEST` | G3 |
| 4 | GATE 2 — depth loop closed in water | G4 |
| 5 | `MOT_BAT_V_MAX` set | G5 |
| 6 | `f_water = 741` validated against tape | 1a |
| 7 | `pool_depth_m` set and cross-checked vs divergence height | 1b |
| 8 | drift over 2–3 m legs, as % of distance | 1c |
| 9 | pool-floor feature yield + **refusal** on plain floor | 1d |
| 10 | `ROT_FRACTION_MAX` re-derived as a residual criterion | 1e |
| 11 | real `td` measured, or `estimate_time_offset:=false` | 2 |
| 12 | ✅ **CLOSED 2026-09-07** (§23) — auto exposure chose a **200 ms shutter** (66 px of blur at 0.64 rad/s); `webcam.py` now pins a manual shutter with a `f·ω·t` blur cap. Gain measured INERT. Next: drive the cap from the LIVE gyro rather than a static max rate | §23 |
| 13 | adaptive Kalman R in water | 3a |
| 14 | conf floor: recall **and** precision on real props | 3b |
| 15 | `ConfidenceTrend` — does confidence lead error? | 3c |
| 16 | `vision.ctrl_conf` enabled only after 3a/3b | 3d |
| 17 | `range_gain_floor` — the close-in instability fix | 3e |
| 18 | `kp_lat` / `kp_yaw` / `ki_lat` with GAIN read from the board | 4 |
| 19 | `PILOT_EXPO` accounted for in terminal alignment | 4 |
| 20 | return leg — **run `return_check.py` BEFORE the session** | 5 |
| 27 | **ChArUco board** — OpenCV recommends it over a chessboard and 4.6 on the Pi has the full API. Partial views become legal (the board may leave frame) and the 180° pose ambiguity goes. Directly targets the 'cannot hit the pose' failure. See `camera-and-calibration.md` §8 | §8 |
| 28 | **does the Fantech actually autofocus?** Marketing says yes; v4l2 exposes NO focus control, so we could not lock it. If it refocuses, its intrinsics are not constant and every bearing drifts — and it is a candidate cause of the historical fx spread | §6 |
| 21 | ✅ **DONE 2026-09-07 — fx 851.23, HFOV 73.88 air / 53.59 water, RO 5/5 folds, sigma(fx) 0.27 %, WELL CONDITIONED.** Installed as `pi_forward_1280x720.json`, which the launch already names. ⚠ Max ERE 14.47 px is still above AprilCal's 1 px bar, so more views (below centre) would tighten it. ~~calibrate the FANTECH forward camera~~ — never done; it is the camera the vision uplink AIMS with, and it was publishing the downward camera's intrinsics until 2026-09-07 (`measured-bars.md` §17 — LATENT, since the uplink is default-off, so this is 'wrong the moment it is switched on', not 'wrong every mission'). Needs a printed board and 25 views; **does not need water** | §17 |
| 22 | ✅ **DONE 2026-09-07 — 31.94 cm on a 30 cm truth, 106.5 %, through the launch** (`measured-bars.md` §18). The four wrong causes on the way were all instrument faults, not sensor faults. ~~re-verify the 30 cm result **through the launch**~~, not through `flow_console --calibration <path>`. Every §13 number came from a tool that passed the path by hand, and the launch wired the calibration to the wrong camera | §17 |
| 23 | ✅ **DONE 2026-09-07 by the same slide** — the launch path in `medium:=air` produces a correct measurement. ⚠ the RECTIFIED water path still has no measured velocity; that half stays water-only. ~~one dry slide with `medium:=air` through the launch~~ — the regression check on the rectifier refactor. `port=RECTIFIED` is verified to come up and the maths is verified against a pinhole, but the rectified path has produced **no measured velocity at all**; every §13 number predates the refactor. Needs light and one hand slide, **no water, no rig** | §13 |
| 24 | **HIGH-ROTATION de-rotation test at the vehicle's own pivot** — the gains were re-derived (`+1.058/+0.830` about the lens) and then REFUTED by the validation A/B (§21): they over-correct on a slide because the pivot differs. Default is now **0.0 = off**, which is best on translation. §12 measured de-rotation halving the error at 0.638 rad/s, so it must be re-enabled once tested in the regime that needs it. ~~re-derive the de-rotation gains on THIS mount~~ — measured three ways on the vehicle (`measured-bars.md` §19): shipped `(-1,-1)` 109.4 %, **off `(0,0)` 105.0 % and 3x tighter**, flipped `(+1,+1)` 94.6 %. The true gain is inside (−1,+1) and near zero. §12 predicted this and it was never done. `tools/flow_derot_calibrate.py`, tilt-only, one axis per run. **Needs no water** | §19 |
| 25 | ✅ **CLOSED 2026-09-07 — measured and WITHDRAWN** (§22). Holding the anchor across a refusal is worse at every refusal rate and *raises* the refusal count, because the widened window still contains the disturbance and it cascades. Parameter deleted, behaviour reverted, numbers pinned in a test | §22 |
| 26 | the RECTIFIED water path has still produced **no measured velocity** — verified to select and its maths verified against a pinhole, but never measured wet. **Water only** | §13 |

### 5c. THE DRY DVL CHECK NEEDS THREE PROCESSES, NOT ONE — measured 2026-09-07

Found by running it. `vision_pi.launch.py flow:=true` alone gets you a node
that comes up, prints a correct banner, and **refuses every interval**. The
first two reasons it gives are misleading and the third is the real one:

| symptom | actual cause |
|---|---|
| `no trackable texture (0/8 points survived)` | transient — the live scene measured **190 corners** on every frame seconds later. Do not chase texture on a single sample. |
| `no depth yet, so no height above the floor` | no manager, and **no barometer is fitted on this bench anyway**. `flow_launch_check.py` publishes `/duburi/state` with depth 0 so height == the tape measure. The manager's own NaN depth is ignored by the node, so the two coexist. |
| **`no gyro sample for this interval`** | **the real blocker.** The node will not measure without `/duburi/imu_rates`, which only `auv_manager_node` publishes (measured **exactly 50.0 Hz**, sd 1.9 ms). No manager, no DVL — on a dry bench and in a pool alike. |

So the dry check is:

    1  ros2 run duburi_manager start                    # gyro @ 50 Hz
    2  ros2 launch duburi_vision vision_pi.launch.py \
           flow:=true flow_medium:=air pool_depth_m:=<tape m>
    3  python3 tools/flow_launch_check.py --height <tape m> --truth-cm 30

**Do NOT pass `vision:=off`** — that value is coerced to boolean `False` and
kills the composed process that owns BOTH cameras, so flow silently receives
no frames while every node looks healthy. Round 33's launch-type coercion,
met again.

**Verified working, stationary rig, 2026-09-07:** 22 intervals, **0 refused**,
reading **−0.68 cm over 10 s** — correctly near zero, which is the negative
control for the whole chain. A moving measurement is the operator's slide.


