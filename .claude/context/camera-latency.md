# Camera latency: the queue, and why the standard fix does not work

> Every number here was measured on our Pi 5 with `tools/v4l2_latency.py`,
> against the **kernel's own capture timestamps**. Where a published source
> disagrees, the disagreement is stated rather than smoothed over.

## 0. The question nobody was asking

Every FPS figure in this repo answers "how many frames per second". None of
them answers **"how old is the picture when the control loop acts on it"**, and
those are different numbers that can move in opposite directions.

Worse, the obvious way to measure it is wrong. Timing `cap.read()` measures how
long the CALL took, which is close to **anti-correlated** with staleness: a
frame already sitting in the driver's queue returns in microseconds and is
400 ms old, while a frame we waited for returns in 16 ms and is brand new.

The kernel knows. `v4l2_buffer.timestamp` is stamped when the frame finishes
arriving, on `CLOCK_MONOTONIC`. OpenCV's V4L2 backend exposes no `CAP_PROP` for
it, which is why `v4l2_latency.py` talks to the device through ioctl.

## 1. THE FINDING: a full V4L2 queue holds the OLDEST frames

This is the fact the standard advice does not account for, and it inverts the
conclusion.

Stall a consumer for 400 ms at 60 fps and read the driver's own `sequence`
counter off what is waiting:

```
  last frame before the stall: seq 31
  the camera should have produced ~24 more (up to seq ~56)

  what is actually in the queue:
    seq 32   age 396.1 ms      seq 34   age 364.2 ms
    seq 33   age 380.2 ms      seq 35   age 344.2 ms
```

Sequence **+1..+4** — the four frames from the *start* of the stall. Frames
+5..+24 were **never captured at all**: the kernel had no free buffer, so the
driver dropped them on the floor.

**The queue is not a backlog of recent frames you can skip through. It is a
fossil record of the moment the consumer stalled.** Confirmed on two cameras
from different vendors (Sonix and Fantech), so it is the V4L2/UVC contract, not
one driver's policy.

## 2. Four policies, measured

Same camera, same 400 ms stall, 10 reps:

| policy | frame age | blocked |
|---|---|---|
| **A** plain `read()` — what we shipped | **396.1 ms** | 0.0 ms |
| **B** drain the queue, take the last — *the standard recipe* | **348.1 ms** | 0.1 ms |
| **C** drain, then wait for a new frame | **12.0 ms** | 28.0 ms |
| **D** keep-up thread (what we now ship) | **16.9 ms** | 0.0 ms |

**B is the advice in essentially every reference on this problem — the OpenCV
issue thread, the "bufferless VideoCapture" gists, the threaded-VideoCapture
library — and it recovers 12 % of the staleness.** It empties a fossil record
and hands you the least-old fossil. §1 is why.

C and D recover 97 %. D is free because a thread that dequeues and requeues
immediately means the driver **always has a free buffer**, so it never drops a
frame and the newest one is always about one frame old.

Under a busy consumer (25 ms of work per tick, the shape of an inference):
**22.4 ms median, 31.95 p95** — bounded — with the consumer's own tick at
33.39/33.40 ms median/p95, i.e. no GIL starvation, and the pump holding the
camera's full 62.5 Hz with 0 sequence gaps.

## 3. Settled negatives — things that look like levers and are not

**`CAP_PROP_BUFFERSIZE` is not the fix.** Our `webcam.py` set it to 1 with the
comment *"avoid stale frames"*. Measured under the mailbox at 2/3/4/6/8
buffers: **26.4–26.6 ms median, 35.3–35.5 p95, flat within noise**, 1 dropped
frame in ~400 at every setting. The pump is what fixes staleness; the buffer
count only decides how much slack exists before the driver starts dropping.

**More buffers do not add latency** either, which is the counter-intuitive half
— they are what lets the driver keep capturing while you are busy.

**GStreamer's `appsink drop=true max-buffers=1` is the same idea** and is the
right answer *if* you are already on a GStreamer pipeline. The discourse thread
on `max-buffers` being "ignored" resolves to the same lesson as §1: dropping at
the SINK is too late, you must leak upstream of the slow element
(`queue leaky=downstream`). We are not on GStreamer, and going there to get a
property we can implement in 40 lines of ioctl would be the larger change.

**`qos_profile_sensor_data` is NOT a mailbox.** It is `BEST_EFFORT` but
`KEEP_LAST` **depth 5** — still a five-deep queue. For a stream where only the
newest frame has value, the depth is 1. (Vulkan names this distinction
properly: `MAILBOX` replaces what is queued, `FIFO` queues behind it.)

## 4. Where the time actually goes now

Stage by stage against the same kernel stamp, single camera, no contention:

```
  age when read() returns        14.56 ms
    after cv_bridge encode       14.88     (encode 0.31 ms)
    after publish() returns      15.20     (publish 0.30 ms)
  age at the SUBSCRIBER          16.96     (transport 1.76 ms)
```

**2.4 ms of software on top of ~1 frame period.** The frame period is
irreducible: the sensor integrates and the USB isochronous transfer takes what
it takes.

In the live stack (both cameras, two detectors, tracking) the image reaches the
subscriber at **26.0 ms median / 32.1 p95**, and the extra ~9 ms is contention,
not the pipeline — verified by pausing the second detector, which returns it to
25.9 ms with a tight max.

Full chain to `detections`, one camera: **47.5 ms**, of which the detector adds
21.6 (inference is ~10).

## 5. Two bugs this found that were not about latency

**The header stamp was a correctness bug.** `camera_node` stamped `now()` at
PUBLISH time, so `header.stamp` claimed every frame was 0 ms old however long
it had queued. `_freshness` decays the lateral command by that field, the coast
ladder is measured in it, and **`is_new_frame` gates the mid-hold torpedo
fire** on it. All of them were measuring age-since-publish and were
structurally blind to the latency. It now carries the kernel's capture time.

**A fixed-rate timer draining a one-deep slot costs half a period.** Measured
15.4 ms of age at the slot, 23.8 ms after a 60 Hz timer — 8.4 ms of pure
sitting still, on a pipeline built to remove exactly that. The publish is now
driven by the event that matters (a frame exists) rather than by a clock that
cannot know when that happened.

## 6. Method note, because two guesses were wrong

The first two hypotheses for the residual latency — "the pump is not running"
and "a 20 ms idle sleep is the cause" — were both plausible and both wrong. The
first came from a flawed instrument (`ps -eLo comm` truncates thread names at
15 characters, so it reported 0 pump threads when the pump was running fine);
the second moved the number by 3 ms.

What worked was **measuring each stage against one clock**, plus a
control-vs-treatment run (both cameras live vs one paused) that separated
contention from pipeline. Guessing at causes and testing them one at a time
cost two round trips; the staged measurement answered it in one.

---

# Round 31 — capture fast, publish slow, and one finding that did not survive

## 10. The result

Mission configuration on the vehicle (forward live, downward paused), age from
the kernel's capture stamp:

```
                        round 30      round 31
  image at subscriber   25.9 ms       15.0 ms      p95 28.8 -> 23.0
  detections            47.5 ms       32.5 ms      p95 64.9 -> 48.4
```

Three changes, in order of what they bought.

## 11. THE FPS CAP WAS MINE AND IT WAS WRONG

Round 30 capped the forward camera to 60 fps and measured **+53 % detections**
(20.2/21.3 → 32.3/31.2 det/s). That measurement was correct and the conclusion
has expired, because the mailbox changed what a captured frame costs.

`cap.read()` used to DECODE every frame, so a camera outrunning its consumer
burned a core on JPEGs nobody used and throttling the camera throttled the
waste. The pump only COPIES (13 µs); `read()` decodes on demand. So capture
rate stopped being a throughput knob and became a **latency** one — frame age
at dequeue is about one capture period:

```
  consumer pinned at 30 Hz, decode on demand
    capture  30    age 46.12 ms   p95 59.52    CPU 8.0 %
    capture  60    age 23.54 ms   p95 33.32    CPU 7.8 %
    capture 120    age 13.45 ms   p95 18.09    CPU 8.0 %
    capture 210    age  8.94 ms   p95 14.28    CPU 7.8 %
```

**2.6× fresher for identical CPU.** The cap was costing 14.6 ms and buying
nothing.

MJPEG survives review on measurement, not inheritance: YUYV needs no decode
(0.55 ms vs 1.94) but 640×360 YUYV is 450 kB a frame and the bus caps it at
**35.4 Hz, age 20 ms**. Paying 1.4 ms of CPU to halve the age is the right side
of that trade. (The Pi 5 has no hardware JPEG decoder — the VideoCore VII drops
the block the Pi 4 had — so every frame is decoded on the A76 cores.)

## 12. CAPTURE RATE AND PUBLISH RATE ARE DIFFERENT KNOBS

Uncapping `fps` alone made detections **worse**: image age 26.0 → 23.6 ms and
detection age 47.5 → 62.6. Every captured frame was also PUBLISHED — 94
messages a second of 691 kB, which the executor and DDS pay for whether or not
anyone wanted them.

The mailbox decouples capture from DECODE. `publish_rate_hz` decouples it from
TRANSPORT, and with capture pinned at 210:

```
  publish every frame (~107 Hz)   detections 55.3 ms   p95 79.0
  publish  60 Hz                  detections 36.6 ms   p95 51.3
  publish  40 Hz                  detections 31.7 ms   p95 39.6
```

**Publishing slower gives fresher detections**, which is only paradoxical if
you think of the topic as a stream. It is a mailbox: the consumer reads ~36 Hz
whatever we publish, so anything above that is 691 kB messages it discards —
work that delays the frame it does want.

## 13. LOCK-FREE, AND WHAT THAT IS AND IS NOT WORTH

`self._slot = (payload, cap_t, seq)` is one `STORE_ATTR` publishing an
immutable tuple, so a reader's one `LOAD_ATTR` gets the whole old frame or the
whole new one. That is the atomic pointer swap; nothing else is needed.

The buffering is **by refcount, which is better than double buffering**: a
consumer mid-decode holds its tuple, so the pump's next store cannot touch that
memory, and it is freed the instant nobody is reading. A fixed pair has to
reason about whether the reader finished with B before the writer wraps onto
it; refcounting answers that by construction.

**The lock it replaces was measured: 0.198 µs, 2.0× the bare store, 0.001 % of
a frame.** So this is not a latency change and is not claimed as one.

## 14. ZERO-COPY IS NOT WORTH IT HERE, AND THE NUMBER SAYS SO

```
  bytes(mmap[:n])   -- what we do     13.1 µs
  memoryview(mmap)  -- zero-copy       1.6 µs
  cv2.imdecode      -- the real work  2450.2 µs
```

The copy is **0.5 % of the decode it precedes**. True zero-copy means holding
the buffer until the consumer releases it, which costs the driver one of four
buffers and risks a stalled consumer pinning one forever — to save 11.5 µs. The
ROS hop is the bigger number (build 0.845 ms + serialise 1.469 + transport
1.76) and rclpy has no loaned-message path, so §12 addresses it by sending
fewer, fresher messages rather than cheaper ones.

## 15. THE FINDING THAT DID NOT SURVIVE ITS OWN TEST

A 100 Hz sensor read by a thread beside CPU-bound **Python** threads:

```
  competing threads    lag median    delivered rate
  none                     0.3 ms          99.2 Hz
  one                    108.4 ms          50.2 Hz
  three                 2428.8 ms           1.4 Hz
```

One competitor turns a 100 Hz sensor into a 50 Hz sensor with 108 ms of lag,
**stamped as fresh** — the reader is not broken, merely unscheduled.
`sys.setswitchinterval(0.0001)` against the CPython default of 5 ms recovered
it: 577.6 → 1.2 ms with two competitors, for 19 % of the CPU-bound throughput.
A one-line 480× fix.

**It was then tested against the real vehicle and it is not reachable.** Our
hot threads are not CPU-bound Python: the V4L2 pump blocks in `ioctl`, the
decode is inside OpenCV, the reader blocks in `serial` — all of which release
the GIL. Against the real srot board with 225 Hz of real JPEG decoding beside
it:

```
  drain alone         @5.0 ms   p95 109.75 ms
  drain + vision      @5.0 ms   p95 108.70 ms   <- 225 Hz of decode costs NOTHING
  drain alone         @0.1 ms   p95 130.61 ms   <- the "fix" is WORSE
  drain + vision      @0.1 ms   p95 118.85 ms
```

More switches on threads that already yield is pure overhead. **Reverted.**

What survives is the BNO reader's drain-to-newest and its `_stale_lines`
counter — defence in depth, said to be that rather than a fix for a measured
bug. The tty backlog stayed at 132 bytes through the whole synthetic table, so
watching `in_waiting` would not have caught it either.

## 16. Two bugs found while verifying, neither about latency

**A profile's `device_path` reached nothing.** Every profile that names one
passed it as a kwarg into the builder's `**_`, and the builder read `device`,
which those profiles do not set. All four resolved to index 0: whichever
camera_node started first won and the second died EBUSY — with an error message
recommending the very key the profile already set. Masked for as long as the
operator passed the launch arg explicitly.

**The udev rule earned itself on its first reboot.** `/dev/video0` was the
Sonix before and the Fantech after, so the raw index had already swapped the
two cameras. `/dev/duburi_cam_*` now comes from ID_PATH, and the Pi profiles
name it instead of the `by-path` symlink Raspberry Pi OS never creates.

---

# Round 32 — the links themselves, and a clock that tells the truth

## 17. TWO LINKS WERE DEAD, AND NEITHER LOGGED ANYTHING

A RELIABLE subscriber against a BEST_EFFORT publisher receives **nothing**.
rclpy logs one warning at construction and is silent for ever after.

When `image_raw` became a BEST_EFFORT mailbox in round 30, five subscribers
were updated to match and **one was missed: `vision_state`, the control host.**
Its frame counter therefore sat at 0 — and `preflight.wait_vision_state_ready`
gated on exactly that counter, so **every mission's first vision verb burned
its full 10 s timeout** and reported *"did not pass within 10s"*, which reads
as a slow pipeline rather than a QoS mismatch.

The warning about this trap was already written down, verbatim, in **five
files**. A comment repeated five times is not a mechanism. `duburi_vision/qos.py`
is one table both ends import; ten call sites across four packages now take
their profiles from it, and `test_qos_contract.py` computes the pairings and
scans the tree for anything that bypasses it. The scan found two more
hand-rolled profiles while it was being written.

`vision_state` no longer subscribes to `image_raw` **at all**. The control host
steers on detections; a raw-image subscription costs a full-frame
deserialisation per message on the machine that owes a 50 Hz loop.

Second dead link, same shape: `classes_filter` was published VOLATILE, so a HUD
or console starting after the detector received nothing and showed blank class
chips. It is LATCHED now.

## 18. `age_s` WAS AGE SINCE ARRIVAL, NOT AGE SINCE CAPTURE

`detector_node` passes `msg.header` through onto `detections`, so the capture
stamp survives inference and reaches the control host correctly. Then
`vision_state._on_detections` overwrote it with `time.monotonic()`.

So `_freshness`, the coast ladder, `is_new_frame` (which gates the **mid-hold
torpedo fire**) and `align_stable_frames` all measured age since the message
landed. The entire pipeline delay was invisible to the loop built to react to
it. `motion_vision:1105` even says *"sampled_at is the frame's arrival time"* —
the fact was recorded, the consequence never drawn. Same defect as round 30's
`header.stamp = now()`, one layer downstream.

`is_new_frame` **improves** rather than changes: keyed on the same value
increasing, it now means a distinct FRAME rather than a distinct MESSAGE, which
is what its docstring always claimed.

## 19. THE FRESHNESS CONSTANTS WERE PREDICTED WRONG AND MEASURE FINE

The expectation going in was that an honest `age_s` would push the loop off
full authority permanently, forcing `VISION_FRESH_FULL_S` up. Measured on the
Pi (forward camera, downward paused, 60 s, `72e1e83`):

```
  image_raw    32.5 Hz   age  med 11.46  p95 14.80  max 18.32 ms
  detections   34.5 Hz   age  med 27.25  p95 30.76  max 37.91 ms
  interval             med 27.80  p95 35.54  max 66.87 ms
```

The detector adds **15.8 ms**. Age when the control loop reads a sample is the
pipeline age plus the wait for the next tick, so against `FULL=0.10 / ZERO=0.40`:

```
  med  27.25 + 27.80 =  55.05 ms  -> authority 1.000
  p95  30.76 + 35.54 =  66.30 ms  -> authority 1.000
  max  37.91 + 66.87 = 104.78 ms  -> authority 0.984
  max + one DROPPED detection = 171.65 ms -> authority 0.761
```

**No change.** Full authority at median and p95; the worst measured case costs
1.6 %; a genuinely dropped detection costs 24 %, which is the decay doing its
job. The prediction was wrong because it assumed a higher pipeline age than the
hardware actually has — recorded here so the constants are not churned again on
the same reasoning.

## 20. THE VEHICLE WAS RUNNING CODE FROM 18 COMMITS AGO

Found while setting up the bench. The Pi was on a local branch `master` at
`ac242b1`, tracking nothing, with **972 lines of uncommitted rsync'd work** on
top — and it could not `git fetch`, because there are no GitHub credentials on
it. Every file that differed from `srot` turned out to be the *older* version,
so nothing was lost, but no measurement taken there was measuring the code we
believed.

`git bundle` is the fix that needs no credentials on the vehicle:

```bash
git bundle create /tmp/u.bundle <pi-HEAD>..srot          # dev box
scp /tmp/u.bundle fh1m@10.42.0.28:/tmp/                  # dev box
git fetch /tmp/u.bundle srot:refs/remotes/origin/srot -f # Pi
git merge --ff-only origin/srot                          # Pi
```

The Pi now tracks `origin/srot` properly. **Check `git log --oneline -1` on the
vehicle before trusting any number taken there.**
