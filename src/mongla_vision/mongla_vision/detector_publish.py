"""DetectorNode, part 3 of 4: every topic the node writes, and its logs.
"""
import time
import rclpy
from mongla_vision import draw
from mongla_vision import image_msg


# Throttle for the always-on operator alignment line (seconds). Matches the
# control-side vision throttle so the rate feels consistent between the detector
# status line and the manager's mission logs.
ALIGN_LOG_THROTTLE_S = 0.5


class DetectorPublishMixin:
    """Methods of `DetectorNode`; see `detector_node.py` for the node."""

    def _publish_vision_info(self) -> None:
        """Say which model is live, and bump the epoch so a switch is visible.

        `database_location` is the model STEM, which is this stack's model
        identity everywhere else (`set_model`, `ClassRef`, the `.yaml`
        sidecar) -- a second naming scheme here would be a third answer to
        "which model is that".
        """
        from vision_msgs.msg import VisionInfo
        m = VisionInfo()
        m.header.stamp = self.get_clock().now().to_msg()
        m.method = 'yolo'
        # Every model that is actually running, primary first. A consumer
        # attributing a detection to "the model" when two are live would
        # attribute half of them to the wrong one, so the field names both --
        # the same comma form `active_model` accepts.
        names = [str(self._active_name or self._single_model_name or '')]
        names += list(getattr(self, '_extra_names', []) or [])
        m.database_location = ','.join(n for n in names if n)
        self._model_epoch += 1
        m.database_version = int(self._model_epoch)
        self._pub_vinfo.publish(m)

    def _publish_classes(self, classes_str: str) -> None:
        """Publish the current classes filter so display_node can light up active classes."""
        from std_msgs.msg import String
        msg = String()
        msg.data = classes_str
        self._pub_classes.publish(msg)

    def _publish_contours(self, frame, detections, header) -> None:
        if self._pub_contours is not None and detections:
            # Built from the SAME list on the SAME header, so a consumer
            # can pair them by stamp and index without a second lookup.
            try:
                from mongla_vision.detection.messages import (
                    detections_to_contours)
                h, w = frame.shape[:2]
                self._pub_contours.publish(detections_to_contours(
                    detections, header, camera=self._cam_name,
                    width=w, height=h))
            except Exception as exc:                    # noqa: BLE001
                # An outline is evidence, not control. Losing it must never
                # cost the detection that is steering the vehicle.
                self.get_logger().warning(
                    f'[DET  ] contour publish failed: {exc!r}', once=True)

    def _publish_debug_image(self, frame, detections, header, dt, primary) -> None:
        # Skip the overlay render + encode + publish entirely when no one is
        # subscribed to image_debug (autonomous runs with viewer:=false). This
        # is pure overhead on the inference thread otherwise -- a free win.
        if (self._publish_dbg
                and self._pub_dbg.get_subscription_count() > 0
                and (time.monotonic() - self._last_dbg) >= self._dbg_min_dt):
            try:
                fps = 1.0 / dt if dt > 1e-6 else 0.0
                overlay = draw.render_all(
                    frame, detections,
                    source=self._cam_name, fps=fps,
                    healthy=True, deadband=self._deadband, primary=primary)
                dbg = image_msg.to_msg(overlay, 'bgr8')
                dbg.header = header
                self._pub_dbg.publish(dbg)
                self._last_dbg = time.monotonic()
            except Exception as exc:
                if rclpy.ok():
                    self.get_logger().warning(f"[DET  ] debug image failed: {exc!r}")

    def _report_seeing(self, frame) -> None:
        """~0.6 ms per 640x480 frame. NEVER allowed to stop detection.

        The WHOLE body is guarded, not just the check: this runs inside the
        worker loop, and an exception anywhere here -- the log line, the
        publish -- escaped and ended the loop in the first draft's test.
        """
        try:
            state = self._seeing.observe(frame)
            if state == self._seeing_state:
                return
            self._seeing_state = state
            if state != 'ok':
                self.get_logger().warning(
                    f'[DET  ] camera {getattr(self, "_cam_name", "?")} is BLIND '
                    f'({state}): no boxes from here mean "cannot see", not '
                    f'"nothing there".')
            from std_msgs.msg import String
            self._pub_seeing.publish(String(data=state))
        except Exception as exc:                        # noqa: BLE001
            try:
                self.get_logger().warning(f'[DET  ] seeing check failed: {exc!r}')
            except Exception:                           # noqa: BLE001
                pass

    def _log_alignment(self, primary, frame) -> None:
        """Always-on operator BEARING line for the currently-loaded class.

        Fires whenever the detector sees its loaded class (``detections`` are
        already class-filtered, so ``primary`` is that class) -- independent of
        whether a vision verb is running. Reports the RAW bbox-centre pixel
        offset from frame centre on lateral (x) and depth (y). This is live
        telemetry, NOT an alignment verdict: it is deliberately worded distinct
        from a vision verb's ``aligned (N/Mpx)`` outcome so the operator never
        reads this continuous offset as "the verb aligned N px off-target".
        Throttled so it doesn't flood at detection rate.
        """
        now = time.monotonic()
        if (now - self._last_align_log) < ALIGN_LOG_THROTTLE_S:
            return
        self._last_align_log = now
        h, w   = frame.shape[:2]
        cx, cy = primary.cx, primary.cy
        x_off  = cx - w * 0.5    # +right of centre
        y_off  = cy - h * 0.5    # +below centre (drives depth)
        self.get_logger().info(
            f"[ offset lat={x_off:+.0f} depth={y_off:+.0f}px ] "
            f"'{primary.class_name}' bearing (live, off-centre)")

    def _log_efficiency(self):
        """Is the chip only ever looking at the freshest frame?

        `stale` is the frame's age when inference STARTS. Its floor is
        capture->available plus the decode -- about 6 ms on this hardware --
        and it is the number that grows first if anything upstream starts
        queueing.

        `wasted` counts frames decoded and then discarded before the chip saw
        them. Under the composed design it should be ZERO: the camera decodes
        only when the inference worker is idle, so nothing can queue behind an
        unconsumed frame. A non-zero value is ~1.9 ms of a core thrown away
        per frame, which on a 27 TOPS budget is exactly what we are trying not
        to do.
        """
        n = self._infers
        if not n:
            return
        mean_ms = 1000.0 * self._stale_sum / n
        waste = self._evicted
        line = (f'[DET  ] chip efficiency: {n} inferences, frame age at '
                f'infer-start {mean_ms:5.1f} ms mean / '
                f'{1000.0 * self._stale_max:5.1f} max, '
                f'{waste} decoded-but-never-inferred')
        if waste:
            self.get_logger().warn(
                line + ' <- WASTED DECODES: frames are queueing behind the '
                       'worker, which should be impossible on the direct feed')
        else:
            self.get_logger().info(line)
        self._infers = 0
        self._evicted = 0
        self._stale_sum = 0.0
        self._stale_max = 0.0

    def _log_health(self):
        now = time.monotonic()
        elapsed = max(now - self._last_log, 1e-3)
        in_hz   = self._frames / elapsed
        avg_ms  = (self._infer_total_s / max(self._frames, 1)) * 1000.0
        target_pct = 100.0 * self._with_target / max(self._frames, 1)
        model_tag = f'  model={self._active_name!r}' if self._active_name else ''
        self.get_logger().debug(
            f"[DET  ] in_hz={in_hz:5.1f}  avg_infer={avg_ms:5.1f}ms  "
            f"with_target={target_pct:4.0f}%  total={self._frames}{model_tag}")
        self._frames = 0
        self._with_target = 0
        self._infer_total_s = 0.0
        self._last_log = now
