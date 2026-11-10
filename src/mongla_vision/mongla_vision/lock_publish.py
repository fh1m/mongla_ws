"""LockNode's outputs: the /lock box, its decomposition, the pose
correspondences, and what the node remembers about where the target was.

The /lock box names its rung in the hypothesis and the class it follows in
`Detection2D.id`, so a consumer can refuse a box for another target (#74).
"""
import math
import time
import numpy as np
try:                                   # noqa: SIM105
    from mongla_interfaces.msg import TargetCorrespondences
except ImportError:                    # message not built in this workspace
    # The LADDER must survive it. Correspondences are an addition to what this
    # node does, not a precondition, so an unbuilt interface costs the 6-DoF
    # path and nothing else -- the follower and anchor rungs still publish
    # `/lock`, which is what the control loop steers on.
    TargetCorrespondences = None
from geometry_msgs.msg import Vector3Stamped
try:
    from mongla_interfaces.msg import MonglaState
except ImportError:                                              # noqa: BLE001
    MonglaState = None
from mongla_vision.detection.detector import Detection
from mongla_vision.detection.messages import detections_to_array
from mongla_vision.tracking.lock_state import Rung


class LockPublishMixin:
    """Methods of `LockNode`; see `lock_node.py` for the node."""

    def _publish_correspondences(self, pose, header):
        """The anchor's inliers as EVIDENCE, for `pnp_node` to solve.

        This node used to solve the pose here. It no longer does, and the split
        is the point: a pose is a lossy summary of the points it was fitted to,
        so publishing only the pose made a missed shot unexplainable -- bad
        evidence and a bad solve look identical afterwards. Correspondences on
        a bag are re-solvable with a different gate, months later.

        The object side is rectified here; the image side is not. Object
        points are reference PIXELS scaled to metres, so they must go through
        the flat-port map or the object model itself is distorted -- that is
        this node's business, because this node owns the patch. The live pixels
        are sent RAW and marked `OPTICS_RAW`, because rectifying the image side
        is the SOLVER's business and doing it in both places is how the two
        would come to disagree. The message states which, so neither has to
        guess and nothing can apply the map twice.
        """
        if self._pub_corr is None or pose is None or not pose.ok:
            return
        if self._K is None or self._target_w_m <= 0.0:
            return          # `_report_geometry_problems` already said why
        roi = self._anchor.reference_roi
        wh = ((roi[2] - roi[0], roi[3] - roi[1]) if roi
              else (self._anchor._be.w, self._anchor._be.h))
        w_px, h_px = float(wh[0]), float(wh[1])
        if w_px <= 0 or h_px <= 0:
            return
        ref_pts = pose.ref_pts
        if self._rect is not None:
            ref_pts = self._rect.rectify(ref_pts)
        ref = np.asarray(ref_pts, np.float64).reshape(-1, 2)
        live = np.asarray(pose.live_pts, np.float64).reshape(-1, 2)
        if len(ref) < 4 or len(ref) != len(live):
            return

        # Reference pixels -> object-plane metres, origin at the patch centre.
        # The patch is planar BY CONSTRUCTION (one snapped view), so z = 0.
        m_per_px = float(self._target_w_m) / w_px
        obj = np.zeros((len(ref), 3), np.float64)
        obj[:, 0] = (ref[:, 0] - w_px * 0.5) * m_per_px
        obj[:, 1] = (ref[:, 1] - h_px * 0.5) * m_per_px

        m = TargetCorrespondences()
        if header is not None:
            m.header = header
        m.camera = self._cam
        # What the anchor enrolled on, not the class the mission asked for:
        # the pose downstream is trusted as a pose OF this label (#76).
        m.target_label = self._anchor_label or ''
        m.source = 'anchor'
        m.optics = TargetCorrespondences.OPTICS_RAW
        # The anchor matches in its BACKEND's pixels, not the camera's full
        # frame. Saying so is what lets the solver scale K instead of silently
        # scaling every angle and range it recovers.
        m.image_width = int(self._anchor._be.w)
        m.image_height = int(self._anchor._be.h)
        m.object_points = obj.reshape(-1).tolist()
        m.image_points = live.reshape(-1).tolist()
        self._pub_corr.publish(m)

    def _publish(self, st, header):
        """`header` is the frame the WINNING RUNG observed, not the newest one.

        Each rung answers from a different instant -- the follower from this
        frame, the anchor from whichever frame it last ran on, the detector
        from whichever frame it last finished. One shared stamp would make all
        three claim the freshest of them.
        """
        # Where the lock is, for `_on_det` to judge the next weak box against.
        # Set before the rate limit below could return, so a throttled publish
        # still moves the prior.
        self._lock_centre = None
        if not st.have_target:
            self._confirms = 0
        if st.have_target:
            x1, y1, x2, y2 = st.xyxy
            self._lock_centre = ((x1 + x2) * 0.5, (y1 + y2) * 0.5)
        # Rate limit when one is configured, EXCEPT on a rung change. Default
        # is no limit: the board-side loop is 500 Hz and wants every frame.
        rung = st.rung.value if st.have_target else None
        if self._pub_min_dt > 0.0 and rung == self._last_pub_rung:
            now = time.monotonic()
            if now - self._last_pub_t < self._pub_min_dt:
                return
            self._last_pub_t = now
        else:
            self._last_pub_t = time.monotonic()
        self._last_pub_rung = rung

        dets = []
        if st.have_target:
            x1, y1, x2, y2 = st.xyxy
            dets.append(Detection(class_id=0, class_name=st.rung.value,
                                  score=float(st.authority),
                                  xyxy=(x1, y1, x2, y2)))
            self._note_visibility(x1, y1, x2, y2)
            self._remember_world(x1, y1, x2, y2)
        # The score is the AUTHORITY, not rung trust times authority (#78).
        # A rung's own confidence is a gate (`MIN_RUNG_CONF`, applied in
        # `arbitrate`), the same job a live detection's score does against
        # `ctrl_conf`. Multiplied in, a carried detection at an underwater
        # score of 0.3 steered at 30 % on a lock the ladder trusted fully.
        if header is not None:
            arr = detections_to_array(dets, header)
            # The class the ladder is following, beside the rung name: the
            # consumer refuses a box whose identity is not the class it asked
            # for (#74). The anchor answers for what it enrolled on; the
            # detection and follow rungs for the last real detection.
            for d in arr.detections:
                d.id = self._lock_identity(st)
            self._pub.publish(arr)

    def _lock_identity(self, st) -> str:
        """What was OBSERVED, never what was asked for: a pinned class the
        ladder has not seen is not evidence that it holds one."""
        if st.rung.value == 'anchor':
            return self._anchor_label or ''
        return self._det_cls or ''

    def _publish_decomposition(self, pose, header) -> None:
        """theta, scale and the range they imply -- the anchor's other outputs.

        What each one is.

        `scale` > 1 means the reference patch appears LARGER than when it was
        stored, i.e. we are CLOSER. It is a range RATIO against the moment of
        enrolment, so it becomes metres only if the reference's true width is
        known -- which is what `target_width_m` is for, and why that parameter
        now reaches this node.

        `theta` is in-plane rotation against a FIXED reference, not an
        integrated rate. It is hull ROLL only on the DOWNWARD camera; on the
        forward camera the same number is a mixture of roll and the target
        being tilted, and publishing it as roll would be the plausible-number
        defect this repo keeps finding. The frame_id says which camera it came
        from so a consumer can decide.

        NaN WHEN UNKNOWN, never 0.0. A scale of 1.0 means "the same
        distance"; absent must not read as that.
        """
        if self._decomp_pub is None or pose is None or not getattr(pose, 'ok', False):
            return
        th = float(getattr(pose, 'theta', float('nan')))
        sc = float(getattr(pose, 'scale', float('nan')))
        if not (math.isfinite(th) or math.isfinite(sc)):
            return
        m = Vector3Stamped()
        if header is not None:
            m.header = header
        m.header.frame_id = self._cam
        m.vector.x = th
        m.vector.y = sc
        # The per-class table, not the operator parameter.
        # `target_geometry.yaml` replaced `target_width_m` precisely because
        # one width for every prop is wrong, and `width_for()` is what the
        # 6-DoF path already uses. An explicit `target_width_m` still wins --
        # a measured prop beats a table entry -- but the table is the default.
        w = float(self._ladder('target_width_m') or 0.0)
        if w <= 0.0:
            try:
                from mongla_vision.target_geometry import width_for
                w = float(width_for(self._cls) or 0.0)
            except Exception:                                    # noqa: BLE001
                w = 0.0
        m.vector.z = (w / sc) if (w > 0.0 and math.isfinite(sc) and sc > 1e-6) \
            else float('nan')
        self._decomp_pub.publish(m)

    def _remember_world(self, x1, y1, x2, y2) -> None:
        """Hold the target in WORLD coordinates, so leaving frame is not
        forgetting (`visibility.WorldTarget`).

        Downward camera only, and that is the whole point. `WorldTarget`
        needs an absolute RANGE, and it refuses a guessed one by design: "a
        remembered position built from a guessed range would send the vehicle
        confidently to the wrong place." For a FORWARD target we genuinely do
        not have one -- the anchor gives a relative patch scale, metric only
        if the prop's true width is known -- so this stays disabled there
        rather than inventing a number.

        For the DOWNWARD camera the range is measured: `_altitude_m()` is
        the height above the floor from `flow_node`'s tile grating, and a
        target on the floor is exactly that far away, corrected for the
        off-axis angle. That is why this rung can be switched on for one
        camera and not the other -- not a limitation of the memory, a fact
        about which sensor measures what.

        Every input is gated on being FRESH and FINITE. Yaw, odometry and
        altitude each go NaN when their source cannot stand behind them, and
        the memory refuses rather than storing a confident wrong place.
        """
        if self._cam != 'downward' or self._world is None:
            return
        K = self._K_rect if getattr(self, '_K_rect', None) is not None \
            else getattr(self, '_K', None)
        gray = self._gray_now()
        if K is None or gray is None or self._odom_xy is None:
            return
        yaw = self._yaw_fresh()
        alt = self._altitude_m()
        if not (math.isfinite(yaw) and math.isfinite(alt) and alt > 0.0):
            return
        try:
            from .tracking.visibility import bearing_from_pixel
            h, w = gray.shape[:2]
            cx, cy = 0.5 * (x1 + x2), 0.5 * (y1 + y2)
            bx, by, bz = bearing_from_pixel(cx, cy, float(w), float(h),
                                            float(K[0][0]), float(K[1][1]))
            if not math.isfinite(bz) or bz <= 1e-6:
                return
            # De-rotate by hull roll, from the operator's R&D fusion:
            #     dx' = dx*cos(roll) - dy*sin(roll)
            #     dy' = dx*sin(roll) + dy*cos(roll)
            # The downward camera is bolted to the hull, so a rolled hull
            # rotates the IMAGE about the optical axis. Without this, a target
            # dead ahead on a hull rolled 20 deg is remembered 20 deg off, and
            # the error is INVISIBLE -- the bearing is perfectly
            # self-consistent, just in the wrong frame.
            #
            # Roll is unactuated on the current five-thruster CAD, so the
            # controller cannot null it. The final hull adds a roll pair, and
            # a rolled hull still rotates the image, so it is compensated here
            # either way.
            #
            # NaN roll leaves the bearing ALONE rather than assuming level.
            # Assuming zero is a confident claim that the hull is flat, which
            # is the plausible-number defect; leaving it uncorrected is an
            # honest smaller claim.
            roll = self._roll_fresh()
            if math.isfinite(roll):
                cr, sr = math.cos(roll), math.sin(roll)
                bx, by = (bx * cr - by * sr), (bx * sr + by * cr)
            # Looking DOWN: the optical axis is the vertical, so the slant
            # range to a floor target is the altitude divided by the cosine
            # of the off-axis angle -- which `bearing_from_pixel` already
            # returns as the z component of a unit vector.
            rng = alt / bz
            # The downward camera's x/y map to the hull's lateral/forward
            # axes, so the in-plane bearing is measured from the SAME axes
            # yaw is measured from. `downward-camera.md` records the remap.
            brg = math.degrees(math.atan2(bx, by))
            self._world.observe(self._odom_xy, yaw, brg, rng,
                                time.monotonic())
        except Exception:                                        # noqa: BLE001
            return

    # -- the field-of-view guard, ADVISORY ---------------------------------- #
    def _note_visibility(self, x1, y1, x2, y2) -> None:
        """How much field of view is left before the target leaves the frame.

        The largest remaining gap in the pipeline. There is no field-of-view
        constraint anywhere in `mongla_control`, so perception can be perfect
        and the lock still dies because the vehicle turned away from a target
        it was tracking successfully. `tracking/visibility.py` has existed,
        measured and tested, importable by nothing.

        Advisory only, and deliberately so. The real fix is to scale the
        yaw/lateral demand by `Visibility.scale` -- and ONLY the component that
        makes the bearing worse, or the vehicle stops approaching a target that
        is dead centre. That is a control-path change, and B-59 is a fresh
        reminder of what shipping an unmeasured one costs. So this rung
        MEASURES and REPORTS; it does not steer. The pool day then has the
        margin trace it needs to justify switching the scaling on, instead of
        the scaling arriving untested on the same day as the water.

        The barrier is RECTANGULAR, not the conical one the CBF papers use:
        a camera image is a rectangle, and on our own geometry a target one
        pixel from the right edge scores h = +0.061 against a cone -- "safe"
        while being gone.
        """
        K = self._K_rect if getattr(self, '_K_rect', None) is not None \
            else getattr(self, '_K', None)
        gray = self._gray_now()
        if K is None or gray is None:
            return
        try:
            from .tracking.visibility import assess
            h, w = gray.shape[:2]
            v = assess(0.5 * (x1 + x2), 0.5 * (y1 + y2), float(w), float(h),
                       float(K[0][0]), float(K[1][1]))
        except Exception:                                        # noqa: BLE001
            return
        self._vis_state = v
        # Log only on a TRANSITION. A per-frame line at 50 Hz is noise and
        # would be filtered out by whoever reads the log, which is how the
        # baked-floor warning in B-58 went unread for weeks.
        if v.state != getattr(self, '_vis_last', None):
            self._vis_last = v.state
            if not v.safe:
                self.get_logger().warn(
                    f'[LOCK ] target near frame edge: {v.state} '
                    f'margin={v.h:.3f} (advisory -- no demand is scaled yet; '
                    f'see ladder-register.md)')

    def _log_health(self):
        n = self._n_by_rung
        tot = max(sum(n.values()), 1)
        self.get_logger().info(
            '[LOCK ] ' + '  '.join(
                f'{r.value}={100.0 * n.get(r, 0) / tot:.0f}%' for r in Rung))
        for r in Rung:
            n[r] = 0
