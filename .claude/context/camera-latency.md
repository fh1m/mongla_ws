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
