"""DetectorNode, part 2 of 4: how a camera frame reaches the detector.

The mailbox keeps only the newest frame, so inference never works on a stale one.
"""
import queue as _queue
import time
from sensor_msgs.msg import Image
from mongla_vision import qos
from mongla_vision import image_msg
from typing import NamedTuple


# How long a `direct_feed` detector waits for its first frame by
# reference before deciding there is no camera in its process and
# subscribing instead. Long enough that a slow camera open does not
# trip it, short enough to be invisible at startup.
_DIRECT_FALLBACK_S = 2.0


class _DirectFrame(NamedTuple):
    """A decoded frame handed straight across in one process.

    A tuple rather than a bare ndarray so the HEADER travels with the pixels.
    They are separable only by accident: the header carries the CAPTURE time,
    and every freshness gate downstream -- `_freshness`, the coast ladder, the
    mid-hold torpedo fire -- reads it.
    """
    frame: object
    header: object


class DetectorFramesMixin:
    """Methods of `DetectorNode`; see `detector_node.py` for the node."""

    def _on_image(self, msg: Image):
        # Single-slot: drop stale frame, enqueue latest only.
        self._offer(msg)

    # ------------------------------------------------------------------ #
    #  Direct in-process feed (composed process)                         #
    # ------------------------------------------------------------------ #
    def wants_frame(self) -> bool:
        """True when the worker is idle and would consume a frame NOW.

        A composed camera gates its DECODE on this. `paused` is included
        because a paused detector consumes frames only to discard them, and on
        the vehicle the unused camera is paused for most of a mission -- so
        this is also what stops it decoding 15 fps of pictures nobody reads.
        """
        return (self._want.is_set()
                and not self.get_parameter('paused').value)

    def _check_direct_feed(self) -> None:
        """Subscribe after all, if nothing was handed to us by reference.

        A detector with `direct_feed` on and no composed camera receives
        NOTHING, for ever, with no error -- the failure mode this whole round
        has been about. Two seconds of silence is enough to be sure, and
        cheap enough that a slow-starting camera does not trip it.
        """
        self._fallback_timer.cancel()
        if self._fed_direct or self._sub is not None:
            return
        self.get_logger().warn(
            f'[DET  ] direct_feed is ON but no frame arrived by reference in '
            f'{_DIRECT_FALLBACK_S:.0f}s -- there is no camera in this '
            f'process. Subscribing to {self._ns_in} instead. This works, but '
            f'it pays the encode+serialise+transport this setting exists to '
            f'skip; run the composed launcher, or set direct_feed:=false to '
            f'silence this.')
        self._direct = False
        self._sub = self.create_subscription(
            Image, self._ns_in, self._on_image, qos.IMAGE)

    def submit_frame(self, frame_bgr, header, *, spare: bool = False) -> None:
        """Hand over an ALREADY-DECODED frame plus the header that describes it.

        The header must be the one built from the frame's own capture time.
        Passing the frame without it would leave `detections` stamped with
        whatever the detector felt like, which is the defect the round-30 and
        round-32 stamp fixes removed at the two layers either side of this one.
        """
        self._fed_direct = True
        self._offer(_DirectFrame(frame_bgr, header), waste=not spare)

    def _offer(self, item, waste: bool = True) -> None:
        """Newest wins. `waste=False` for a frame decoded for ANOTHER
        consumer anyway (the composed lock, `spare=True`): superseding it
        costs no decode, so it must not count as one."""
        with self._offer_lock:
            while not self._infer_q.empty():
                try:
                    self._infer_q.get_nowait()
                    if not waste:
                        continue
                    # Something was waiting and is now discarded. Under the
                    # composed design this should be ~0: the camera only
                    # decodes when the worker is idle, so nothing should ever
                    # queue behind an unconsumed frame. A rising count means we
                    # are decoding frames the chip never looks at.
                    self._evicted += 1
                except _queue.Empty:
                    break
            self._infer_q.put_nowait(item)

    def _next_frame(self):
        """The next queued frame, or None (queue empty, or the detector paused)."""
        try:
            self._want.set()
            item = self._infer_q.get(timeout=0.5)
        except _queue.Empty:
            return None
        finally:
            self._want.clear()
        if self.get_parameter('paused').value:
            # Say so. A paused detector consumes frames and publishes
            # NOTHING: the camera runs, `image_raw` flows, the
            # `/detections` publisher is registered and live, and zero
            # messages ever appear. Every liveness check passes.
            #
            # `vision_pi.launch.py` declares `paused` default TRUE, so this
            # is the SHIPPED state until an operator says otherwise -- and
            # it cost hours on the vehicle (B-XF1) being mistaken for a
            # broken model, a wrong threshold, a bad allowlist and a dead
            # worker thread, because the one thing the node never mentioned
            # was that it had been told not to work.
            self._paused_logged = getattr(self, '_paused_logged', 0) + 1
            if self._paused_logged == 1 or self._paused_logged % 300 == 0:
                self.get_logger().warn(
                    f'[DET  ] PAUSED -- consuming frames and publishing '
                    f'NOTHING ({self._paused_logged} so far). '
                    f'`paused:=false` to infer.')
            return None  # frame consumed from queue; skip decode + infer
        return item

    def _decode(self, item):
        """(frame, header) from a queued item, or None when it cannot be decoded."""
        if isinstance(item, _DirectFrame):
            # Already decoded, in this process, by the camera that captured
            # it. No serialise, no transport, no second copy.
            return item.frame, item.header
        try:
            frame = image_msg.to_array(item, 'bgr8')
        except Exception as exc:
            self.get_logger().warning(
                f"[DET  ] image decode failed: {exc!r}")
            return None
        return frame, item.header

    def _note_frame_age(self, header) -> None:
        # Age of this frame at the instant the chip starts on it. The
        # floor is capture->available plus the decode; anything beyond
        # that is the chip being fed something it should not be.
        try:
            cap = header.stamp.sec + header.stamp.nanosec * 1e-9
            age = time.time() - cap
            # Gated on HAVING a detector: this counter is reported as
            # "inferences", and with `_det` None the loop drops the frame a
            # few lines below. It reported "300 inferences" during a run in
            # which zero inferences happened, which is worse than no
            # counter -- it was the reason a dead detector looked busy.
            if 0.0 <= age < 5.0 and self._det is not None:
                self._infers += 1
                self._stale_sum += age
                self._stale_max = max(self._stale_max, age)
        except AttributeError:
            pass

    def _preprocess(self, frame):
        """The optional preprocessor; on a fault, the raw frame and no more preprocessing."""
        if self._pre is not None:
            try:
                frame = self._pre(frame)
            except Exception as exc:
                # Never let a preprocessing fault stop detection: a
                # degraded frame beats no frame.
                self.get_logger().warning(
                    f'[DET  ] preprocess failed, using the raw frame: '
                    f'{exc!r}')
                self._pre = None
        return frame
