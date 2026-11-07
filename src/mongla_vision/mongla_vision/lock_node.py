#!/usr/bin/env python3
"""lock_node -- run the ladder, publish one target position.

    detections ─┐
    image_raw ──┼─→ follower (LK, fast) ─┐
                └─→ anchor  (XFeat)  ────┼─→ arbitrate ─→ /lock
                                          │
                                    authority decays from the
                                    last REAL detection

Publishes a `Detection2DArray` on `<ns>/lock` carrying ONE box: whatever rung
can still stand behind an answer. `class_id` names the rung, `score` is the
arbiter's confidence (rung trust x time decay), so a consumer that already
knows how to read detections needs no new message type and no new units.

Why a separate topic and not `/detections`. Publishing a followed or anchored
box onto the detector's topic would make it indistinguishable from something
the detector actually saw -- every downstream consumer, the HUD included, would
report a detection that never happened. The rung name and the separate topic
are what keep "the vehicle has a position" from becoming "the vehicle saw the
target".

The heavy rungs run on a WORKER THREAD off the subscription callbacks: the
anchor is 33 ms at 320x240 and blocking a callback with it would back the
executor up behind the very frames it is meant to bridge.

The files:
    lock_node.py      the node, its worker loop, the ladder tick, arbitration
    lock_setup.py     parameters, the anchor and bank, live retuning
    lock_wiring.py    the steps __init__ runs: state, publishers, subscriptions
    lock_inputs.py    the attitude, odometry, floor-height and camera-info callbacks
    lock_frames.py    the image and detection callbacks, and the direct feed
    lock_anchor.py    the anchor rung: enrol or locate against the bank
    lock_publish.py   what goes out on /lock and its companions
"""
import os

os.environ.setdefault('RCUTILS_CONSOLE_OUTPUT_FORMAT', '[{severity}] {message}')

import threading
import time
from typing import NamedTuple

import cv2

import rclpy
from rclpy.node import Node

from mongla_vision.tracking.lock_state import Rung, arbitrate, live_window
from .lock_setup import LockSetupMixin
from .lock_wiring import LockWiringMixin
from .lock_inputs import LockInputsMixin
from .lock_frames import LockFramesMixin
from .lock_publish import LockPublishMixin
from .lock_anchor import AnchorMixin


def header_for(rung, *, detection, frame, anchor):
    """The stamp of the frame the WINNING RUNG actually observed.

    Each rung answers from a different instant. The follower ran on this frame;
    the anchor ran on whichever frame it last evaluated, up to its own period
    ago (0.333 s at 3 Hz); the detector finished on whichever frame it last
    got through. Publishing all three under the newest header makes every rung
    claim the freshness of the fastest one -- and it does so ONLY on the lower
    rungs, so it looks correct exactly while a detection is present and lies
    exactly when the ladder is doing its job.

    Falls back to the current frame when a rung has no header yet: a stamp that
    is merely too NEW makes the consumer act with less authority than it could,
    which is the safe direction. There is no correct answer to hand it, and
    `None` would suppress the publish entirely.
    """
    return {Rung.DETECTION: detection,
            Rung.FOLLOW: frame,
            Rung.ANCHOR: anchor}.get(rung, frame) or frame


class _Tick(NamedTuple):
    """What one ladder tick works on: the newest frame and detection, snapped
    together under the lock so every stage sees the same instant."""
    gray: object
    header: object
    det_header: object
    det_box: object
    det_conf: float
    det_t: float
    det_cls: object
    live_s: float
    now: float


class LockNode(AnchorMixin, LockSetupMixin, LockWiringMixin, LockInputsMixin,
               LockFramesMixin, LockPublishMixin, Node):
    def __init__(self, node_name: str = 'mongla_lock', *,
                 parameter_overrides=None):
        super().__init__(node_name,
                         parameter_overrides=parameter_overrides or [])
        self._declare_parameters()

        cam = str(self.get_parameter('camera').value)
        self._cam = cam
        ns = f'/mongla/vision/{cam}'
        # The steps run in this order on purpose: each comment inside them that
        # says "placed after" or "set beside the state it guards" depends on it.
        self._setup_target()
        self._setup_pose_inputs(ns)
        self._setup_ladder_state()
        self._setup_rungs()
        self._setup_loop_closure(ns)
        self._setup_outputs(ns)
        self._stopping = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        # LIVE, like the detector and the tracker. The ladder was aimable only
        # at launch, so a mission could not point it at the object it was about
        # to steer on -- it had to be relaunched. `target_class` and
        # `publish_hz` now take effect on the next tick.
        self.add_on_set_parameters_callback(self._on_param_change)
        self.create_timer(5.0, self._log_health)
        self._n_by_rung = {r: 0 for r in Rung}

        self.get_logger().info(
            f'[LOCK ] {cam}: follow={self._follower is not None} '
            f'anchor={self._anchor is not None} '
            f'authority {self._full:.2f}->{self._zero:.2f}s  -> {ns}/lock')

    def _loop(self):
        """The lock ladder. GUARDED (B44 class).

        A bare Thread target running XFeat/LK on live frames. One malformed
        frame or ONNX hiccup killed it, after which the ladder silently never
        locked again while every node looked healthy. Faults are counted and
        reported; the ladder degrades to "not locked", which consumers already
        handle, instead of vanishing.
        """
        faults = 0
        while rclpy.ok() and not self._stopping.is_set():
            try:
                self._ladder_tick()
            except Exception as exc:        # noqa: BLE001 -- B44 class
                faults += 1
                if faults in (1, 50):
                    self.get_logger().error(
                        f'[LOCK ] ladder fault #{faults} (thread kept alive): '
                        f'{type(exc).__name__}: {exc}. The lock reports NOT LOCKED '
                        f'while this persists, which consumers already handle.')
                time.sleep(0.05)

    # ── one tick of the ladder, stage by stage ─────────────────────────────
    # Each stage below was a span of `_loop` until 2026-11-07; the code and its
    # comments moved verbatim. The order is the order they run in.

    def _ladder_tick(self) -> None:
        """Take the newest frame, run each rung on it, arbitrate, publish."""
        f = self._take_frame()
        if f is None:
            return
        fb, fc = self._follow_rung(f)
        ab = ac = None
        anchor_ran = False
        if self._anchor is not None:
            anchor_ran = self._enrol_or_locate(f)
            ab, ac = self._anchor_box(f.gray, anchor_ran)
        # Places ride the same frame the ladder just used. Kept out
        # of the ladder itself on purpose: a place is evidence about the
        # VEHICLE, and the ladder is evidence about a TARGET. Blending
        # the two is how a lock starts following the floor.
        if self._closer_on:
            self._place_tick(f.gray, f.header, f.now)
        self._arbitrate_and_publish(f, fb, fc, ab, ac, anchor_ran)

    def _take_frame(self):
        """The newest frame and detection, or None when there is nothing to do.

        Grey is converted only when a later stage will read it."""
        if not self._fresh.wait(0.5):
            return None
        self._fresh.clear()
        with self._lock:
            pending, self._pending = self._pending, None
            gray = self._gray
            header = self._header
            det_header = self._det_header
            det_box, det_conf, det_t = (self._det_box, self._det_conf,
                                        self._det_t)
            det_cls = self._det_cls
            live_s = live_window(self._det_dt)
        now = time.monotonic()
        # Grey only when this tick reads it. Every use below is
        # behind one of these: a detection to reseed on, a follower to
        # step, an anchor evaluation that is due, or place recognition.
        # Lost with none of them -- the common case between targets --
        # the downward camera's 104 frames a second were each converted
        # for nothing, on the thread that shares a GIL with the
        # detectors (26 % of the ladder's wall time, measured).
        need = (det_box is not None
                or (self._follower is not None
                    and self._follower.active)
                or (self._anchor is not None
                    and self._anchor.has_reference
                    and now >= self._anchor_next)
                or self._closer_on)
        if pending is not None and need:
            # Outside the lock: cvtColor releases the GIL, and the
            # camera thread must never wait on the ladder.
            gray = cv2.cvtColor(pending, cv2.COLOR_BGR2GRAY)
            self._gray = gray
        elif pending is not None:
            # Not read this tick; kept so a later reader (a timer) can
            # still convert the newest frame.
            with self._lock:
                if self._pending is None:
                    self._pending = pending
        if gray is None and need:
            return None
        return _Tick(gray=gray, header=header, det_header=det_header,
                     det_box=det_box, det_conf=det_conf, det_t=det_t,
                     det_cls=det_cls, live_s=live_s, now=now)

    def _follow_rung(self, f):
        """The fast rung: reseed LK on a detection, else step it. -> (box, conf)."""
        fb = fc = None
        if self._follower is not None:
            gray, det_box = f.gray, f.det_box
            if det_box is not None:
                # Reseed on every accepted detection: this is what keeps
                # the follower's drift bounded to a single gap.
                self._follower.reset(gray, det_box)
            elif self._follower.active:
                r = self._follower.step(gray)
                if r.ok:
                    fb, fc = r.xyxy, r.confidence
        return fb, fc

    def _arbitrate_and_publish(self, f, fb, fc, ab, ac, anchor_ran: bool) -> None:
        """Pick the rung, publish the lock, and the anchor's correspondences."""
        now, header, live_s = f.now, f.header, f.live_s
        det_t, det_box, det_conf, det_header = (f.det_t, f.det_box, f.det_conf,
                                                f.det_header)
        st = arbitrate(now=now, last_detection_t=det_t,
                       detection=det_box, detection_conf=det_conf,
                       follow=fb, follow_conf=fc or 0.0,
                       anchor=ab, anchor_conf=ac or 0.0,
                       live_s=min(live_s, self._full),
                       full_s=self._full, zero_s=self._zero)
        self._n_by_rung[st.rung] = self._n_by_rung.get(st.rung, 0) + 1
        self._publish(st, header_for(st.rung, detection=det_header,
                                     frame=header,
                                     anchor=self._anchor_header))
        # The pose rides the ANCHOR's header: it is derived from that
        # frame's correspondences, not from whichever frame just arrived.
        #
        # Published only when the anchor actually RE-EVALUATED, or when its
        # ok/not-ok state changed. The loop runs at frame rate and the
        # anchor at `anchor_hz` (3), so publishing every tick sent ~32
        # duplicates for every real evaluation -- measured at 96.5 Hz on
        # the vehicle for a 3 Hz quantity. Each message now corresponds to
        # one evaluation, which is also what makes its stamp meaningful.
        if self._anchor is not None:
            ok_now = bool(self._anchor_pose is not None
                          and self._anchor_pose.ok)
            # ...OR on a slow heartbeat. Without one this went SILENT:
            # with no reference snapped the anchor never re-evaluates, so
            # `anchor_ran` stays False and the state never changes, and
            # after the first message nothing was published again. A
            # consumer then cannot tell "no target" from "lock_node is not
            # running", and the health board cannot age what it never sees.
            # Absence of a TARGET is carried by `ok=False`, not by absence
            # of the message.
            due = (now - self._pose_pub_t) >= self._anchor_period
            if anchor_ran or due or ok_now != self._pose_was_ok:
                self._pose_was_ok = ok_now
                self._pose_pub_t = now
                self._publish_correspondences(
                    self._anchor_pose, self._anchor_header or header)

    def destroy_node(self):
        """Stop the ladder thread BEFORE the publishers go.

        Found by tools/composed_lock_harness.py: destroyed while its thread
        was mid-tick, the ladder published on a destroyed handle and logged
        `ladder fault #1 ... InvalidHandle` on every shutdown -- an ERROR on
        every Ctrl-C, which teaches an operator to ignore that line.
        """
        if hasattr(self, '_stopping'):     # a half-built node has no thread
            self._stopping.set()
            self._fresh.set()              # wake a tick waiting for a frame
        t = getattr(self, '_thread', None)
        if t is not None and t is not threading.current_thread():
            t.join(2.0)
        return super().destroy_node()


def main():
    from mongla_localization.orphan import die_with_parent
    die_with_parent()        # an orphan holds devices and duplicates topics
    rclpy.init()
    node = LockNode()
    from mongla_localization.params_file import refuse_unknown
    refuse_unknown(node)       # bringup.yaml `nodes:` typos stop here
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
